using E_VCSP.Formatting;
using E_VCSP.Objects;
using Gurobi;
using Microsoft.Msagl.Drawing;
using Microsoft.Msagl.Layout.Layered;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Solver
{
    internal abstract class DeadheadPoint
    {
        internal string Id = "";
    }
    internal class DiscreteTrip : DeadheadPoint
    {
        internal required Trip Trip;
        internal double StartingSoC;
    }
    internal class DiscreteDepot : DeadheadPoint
    {
        internal required Location Location;
    }

    internal class DiscreteDeadhead
    {
        internal required DeadheadPoint From;
        internal required DeadheadPoint To;
        internal List<int> DrivingTimes = new();
        internal int IdleTime;
        internal int ChargingTime;
        internal double ChargeGained;
        internal double ChargeUsed;
        internal double Costs;
    }

    internal class EVSPDiscrete
    {
        private double floorToDiscreteValue(double inp) => Math.Floor(inp / (100.0 / Config.DISCRETE_FACTOR)) * (100.0 / Config.DISCRETE_FACTOR);

        public EVSPDiscrete(Instance instance)
        {
            LoadInstance(instance);
        }

        private DiscreteDepot depotStart;
        private DiscreteDepot depotEnd;
        private List<List<DiscreteTrip>> discreteTrips = new();
        private List<DiscreteDeadhead> feasibleDeadheads = new();
        private Dictionary<string, DiscreteDeadhead> deadheadVarMapping = new();
        private GRBModel model;

        /// <summary>
        /// Load the instance
        /// </summary>
        public void LoadInstance(Instance instance)
        {
            Console.WriteLine("Transforming instance input into discrete graph");
            // TODO: beter kiezen
            VehicleType vh = instance.VehicleTypes[0];

            // Create all discrete trips
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                discreteTrips.Add(new(Config.DISCRETE_FACTOR));
                for (int j = 0; j <= Config.DISCRETE_FACTOR; j++)
                {
                    double SoC = Math.Floor(j * 100.0 / Config.DISCRETE_FACTOR);
                    discreteTrips[i].Add(new DiscreteTrip()
                    {
                        Id = $"{instance.Trips[i].Id}@{SoC}%",
                        Trip = instance.Trips[i],
                        StartingSoC = SoC,
                    });
                }
            }

            foreach ((var dl1, var dl2) in discreteTrips.SelectMany(x => discreteTrips.Select(y => (x, y))))
            {
                foreach ((DiscreteTrip dt1, DiscreteTrip dt2) in dl1.SelectMany(x => dl2.Select(y => (x, y))))
                {
                    // Check each pair of discrete nodes
                    if (dt1.Trip.EndTime > dt2.Trip.StartTime) continue;
                    //Console.WriteLine($"dt1: {dt1.Trip.Id}, dt2: {dt2.Trip.Id}");

                    // Early return when the initial trip can't even be driven at the starting SoC.
                    double SoCusedInDT1 = dt1.Trip.Distance * vh.DriveUsage;
                    if (SoCusedInDT1 > dt1.StartingSoC) continue;

                    // Two possibilities: direct drive from dt1 to dt2 or drive via charging station. Both also need to be time-feasible
                    // First: check if direct drive is timefeasible
                    // If it's not, trips via a charging station aren't feasible either (or deadhead times are incorrect, but we assume that they are consistent).

                    // TODO: use deadheads instead of deadhead templates
                    DeadheadTemplate? dht_direct = instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == dt2.Trip.From);
                    if (dht_direct != null && dt1.Trip.EndTime + dht_direct.Duration <= dt2.Trip.StartTime)
                    {
                        // Check if charge is compatible between the two trips
                        double SoC = dt1.StartingSoC - SoCusedInDT1;
                        SoC -= dht_direct.Distance * vh.DriveUsage;
                        if (SoC > 0)
                        {
                            // May be compatible; Note that SoC < 0 may still be possible via indirect trip, therefore no early termination. 
                            SoC = floorToDiscreteValue(SoC);
                            if (SoC == dt2.StartingSoC)
                            {
                                // Determine costs of deadhead; only driving time is used
                                // TODO: idle wordt helemaal niet meegenomen
                                double drivingCost = dht_direct.Distance * Config.M_COST;
                                feasibleDeadheads.Add(new()
                                {
                                    From = dt1,
                                    To = dt2,
                                    Costs = drivingCost,
                                    ChargeGained = 0,
                                    ChargeUsed = dt1.StartingSoC - SoC,
                                    DrivingTimes = [dht_direct.Duration],
                                    IdleTime = dt2.Trip.StartTime - (dt1.Trip.EndTime + dht_direct.Duration),
                                });
                                continue;
                            }
                        }
                    }

                    // Iterate over all charging locations, see whether we can find a valid match in here.
                    // TODO: misschien handig om over alles heen te itereren / duplicate deadheads to te voegen 
                    // om te checken of het goedkoper kan dmv idle time/charging
                    for (int i = 0; i < instance.ChargingLocations.Count; i++)
                    {
                        Location chargeLoc = instance.ChargingLocations[i];
                        // Check if detour via charging location is time-feasible.
                        DeadheadTemplate? dht_toCharge = instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == chargeLoc);
                        DeadheadTemplate? dht_fromCharge = instance.DeadheadTemplates.Find(dht => dht.From == chargeLoc && dht.To == dt2.Trip.From);

                        // No deadhead found / not feasible 
                        if (dht_toCharge == null || dht_fromCharge == null) continue;
                        int idleTime = dt2.Trip.StartTime - dt1.Trip.EndTime - dht_toCharge.Duration - dht_fromCharge.Duration;
                        if (idleTime < 0) continue;

                        // Deadhead feasible; see how much charge can be gained.
                        double SoCAtCharge = dt1.StartingSoC - SoCusedInDT1 - (dht_toCharge.Distance * vh.DriveUsage);
                        if (SoCAtCharge < 0) continue;

                        ChargingCurve cc = chargeLoc.ChargingCurves[vh.Id];
                        ChargeResult cr = cc.MaxChargeGained(SoCAtCharge, idleTime);

                        // Now, check if the charge is actually compatible; In order to do so, we need to check if we can use some partial charge such that our target SOC is reached.
                        double SoCDiff = dt2.StartingSoC - (SoCAtCharge - (dht_fromCharge.Distance * vh.DriveUsage));
                        double alpha = SoCDiff / cr.SoCGained;
                        if (alpha >= 0 && alpha <= 1)
                        {
                            // A feasible charge schedule was found; we now rerun the charge sequence in order to get costs
                            ChargeResult crUsed = cc.ChargeCosts(SoCAtCharge, SoCAtCharge + SoCDiff);

                            // Add deadhead that takes into account 2x driving costs and charging costs
                            // TODO: add time-dependent prices, staat nu gewoon vast

                            double drivingCosts = Config.M_COST * (dht_toCharge.Distance + dht_fromCharge.Distance);
                            int idleTimeAfterCharge = idleTime - (int)Math.Ceiling(crUsed.TimeUsed);
                            feasibleDeadheads.Add(new()
                            {
                                From = dt1,
                                To = dt2,
                                ChargeGained = crUsed.SoCGained,
                                ChargeUsed = SoCDiff,
                                ChargingTime = (int)Math.Ceiling(crUsed.TimeUsed),
                                Costs = drivingCosts + crUsed.Cost,
                                DrivingTimes = [dht_toCharge.Duration, dht_fromCharge.Duration],
                                IdleTime = idleTimeAfterCharge
                            });
                            continue;
                        }
                    }
                }
            }

            Location? depot = instance.Locations.Find(loc => loc.IsDepot);
            if (depot == null) throw new InvalidDataException("At least one location should be flagged as a depot.");
            depotStart = new()
            {
                Id = "depot_start",
                Location = depot,
            };
            depotEnd = new()
            {
                Id = "depot_end",
                Location = depot,
            };

            // From depot to trip
            // TODO: depot to trip and vice versa can also do charging actions
            foreach (DiscreteTrip dt in discreteTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = instance.DeadheadTemplates.Find(dh => dh.From == depot && dh.To == dt.Trip.From);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtTrip = vh.StartCharge - (dh.Distance * vh.DriveUsage);
                double discretizedSoC = floorToDiscreteValue(SoCAtTrip);
                if (discretizedSoC == dt.StartingSoC)
                {
                    double drivingCosts = dh.Distance * Config.M_COST;
                    double pullOutCosts = Config.PULLOUT_COST;
                    feasibleDeadheads.Add(new()
                    {
                        From = depotStart,
                        To = dt,
                        Costs = drivingCosts + pullOutCosts,
                        ChargeGained = 0,
                        ChargeUsed = vh.StartCharge - SoCAtTrip,
                        ChargingTime = 0,
                        DrivingTimes = [dh.Duration],
                        IdleTime = 0,
                    });
                }
            }

            // From trip to depot
            foreach (DiscreteTrip dt in discreteTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = instance.DeadheadTemplates.Find(dh => dh.From == dt.Trip.To && dh.To == depot);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtDepot = dt.StartingSoC - ((dt.Trip.Distance + dh.Distance) * vh.DriveUsage);
                if (SoCAtDepot >= 0)
                {
                    double drivingCosts = dh.Distance * Config.M_COST;
                    feasibleDeadheads.Add(new()
                    {
                        From = dt,
                        To = depotEnd,
                        Costs = drivingCosts,
                        ChargeGained = 0,
                        ChargeUsed = dt.StartingSoC - (dt.Trip.Distance * vh.DriveUsage) - SoCAtDepot,
                        ChargingTime = 0,
                        DrivingTimes = [dh.Duration],
                        IdleTime = 0,
                    });
                }
            }
            Console.WriteLine("Transformation complete");
        }

        public Graph GenerateDiscreteGraph()
        {
            Console.WriteLine("Generating visual representation of graph");
            Graph graph = new();
            // Reset graph
            graph.LayoutAlgorithmSettings = new SugiyamaLayoutSettings()
            {
                LiftCrossEdges = true,
            };

            foreach (var dts in discreteTrips)
            {
                foreach (DiscreteTrip dt in dts)
                {
                    Node node = Formatting.GraphElement.TripNode(dt);
                    graph.AddNode(node);
                }
            }
            graph.AddNode(new Node(depotStart.Id.ToString()) { Label = { Text = "Depot" } });
            graph.AddNode(new Node(depotEnd.Id.ToString()) { Label = { Text = "Depot" } });

            foreach (var fdh in feasibleDeadheads)
            {
                // Dont include edges even if their not show to save space
                Edge edge = Formatting.GraphElement.DeadheadEdge(fdh.From.Id.ToString(), fdh.To.Id.ToString(), fdh.Costs, graph);
                graph.AddPrecalculatedEdge(edge);
            }

            graph.LayerConstraints.RemoveAllConstraints();
            // Ensure that different charge same trips are above eachother
            for (int i = 0; i < discreteTrips.Count; i++)
            {
                for (int j = 0; j < discreteTrips[i].Count; j++)
                {
                    for (int k = j + 1; k < discreteTrips[i].Count; k++)
                    {
                        graph.LayerConstraints.AddUpDownVerticalConstraint(
                            graph.FindNode(discreteTrips[i][j].Id.ToString()),
                            graph.FindNode(discreteTrips[i][k].Id.ToString())
                        );
                    }
                }
            }

            // Depot start left of everything
            Node depotStartNode = graph.FindNode(depotStart.Id.ToString());
            for (int i = 0; i < discreteTrips.Count; i++)
            {
                for (int j = 0; j < discreteTrips[i].Count; j++)
                {
                    graph.LayerConstraints.AddLeftRightConstraint(
                        depotStartNode,
                        graph.FindNode(discreteTrips[i][j].Id.ToString())
                    );
                }
            }
            // Depot end right left of everything
            Node depotEndNode = graph.FindNode(depotEnd.Id.ToString());
            for (int i = 0; i < discreteTrips.Count; i++)
            {
                for (int j = 0; j < discreteTrips[i].Count; j++)
                {
                    graph.LayerConstraints.AddLeftRightConstraint(
                        graph.FindNode(discreteTrips[i][j].Id.ToString()),
                        depotEndNode
                    );
                }
            }

            // Time dependency between individual nodes
            foreach ((var dl1, var dl2) in discreteTrips.SelectMany(x => discreteTrips.Select(y => (x, y))))
            {
                foreach ((DiscreteTrip dt1, DiscreteTrip dt2) in dl1.SelectMany(x => dl2.Select(y => (x, y))))
                {
                    if (dt1.Trip.EndTime <= dt2.Trip.StartTime || dt1.Trip.StartTime < dt2.Trip.StartTime)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(
                            graph.FindNode(dt1.Id.ToString()),
                            graph.FindNode(dt2.Id.ToString())
                        );
                    }
                }
            }

            graph.Directed = true;
            Console.WriteLine("Graph generated");
            return graph;
        }

        public Graph GenerateSolutionGraph()
        {
            Console.WriteLine("Reconstructing paths");

            List<List<(int, DiscreteDeadhead)>> adj = new(); // trips 1->n are map to 0->n-1, depot start = n, depot end = n + 1
            for (int i = 0; i < discreteTrips.Count + 2; i++) adj.Add(new());
            GRBVar[] vars = model.GetVars();
            for (int i = 0; i < vars.Length; i++)
            {
                GRBVar v = vars[i];
                if (v.X == 1)
                {
                    // Corresponding feasibleDeadhead
                    DiscreteDeadhead fdh = deadheadVarMapping[v.VarName];

                    int from = fdh.From is DiscreteDepot
                        ? (fdh.From.Id.EndsWith("start") ? discreteTrips.Count : discreteTrips.Count + 1)
                        : int.Parse(((DiscreteTrip)fdh.From).Trip.Id.Substring(1)) - 1;
                    int to = fdh.To is DiscreteDepot
                        ? (fdh.To.Id.EndsWith("start") ? discreteTrips.Count : discreteTrips.Count + 1)
                        : int.Parse(((DiscreteTrip)fdh.To).Trip.Id.Substring(1)) - 1;
                    adj[from].Add((to, fdh));
                    Console.WriteLine($"{v.VarName} used; adj[{from}][{to}] = 1");
                }
            }



            List<List<DiscreteDeadhead>> paths = new();
            List<DiscreteDeadhead> dfs(int curr, List<List<(int, DiscreteDeadhead)>> adj, List<DiscreteDeadhead> currPath)
            {
                if (curr == adj.Count - 1)
                {
                    return currPath;
                }

                if (adj[curr].Count == 0) throw new InvalidDataException("Something went wrong; there should always be a path from source to sink");

                (int next, DiscreteDeadhead dh) = adj[curr][^1];
                adj[curr].RemoveAt(adj[curr].Count - 1);

                // add dh mataching curr -> next to path
                currPath.Add(dh);
                return dfs(next, adj, currPath);
            }
            while (adj[^2].Count > 0)
            {
                paths.Add(dfs(adj.Count - 2, adj, []));
            }

            Console.WriteLine("Paths reconstructed");
            // Find flow paths in order to define vehicle schedules. 

            // Time to create the actual nodes representing the times; We now distinguish between 4 seperate node types: 
            // 1. Trips
            // 2. Deadhead drives
            // 3. Charging actions
            // 4. Idle times
            // We'll simply use color to distinguish between the 4 of these for now, and let width be completely time-dependent. 

            Graph graph = new();

            List<List<Node>> nodes = new();
            for (int i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                List<Node> pathNodes = new();
                nodes.Add(pathNodes);
                foreach (var pathPart in path)
                {
                    int currTime = -1;
                    DiscreteTrip? dtf = null, dtt = null;
                    if (pathPart.From is DiscreteTrip)
                    {
                        dtf = (DiscreteTrip)pathPart.From;
                        currTime = dtf.Trip.StartTime;
                    }
                    if (currTime == -1 && pathPart.To is DiscreteTrip)
                    {
                        dtt = (DiscreteTrip)pathPart.To;
                        currTime = dtt.Trip.StartTime - pathPart.DrivingTimes.Sum() - pathPart.ChargingTime - pathPart.IdleTime;
                    }
                    if (currTime == -1) throw new InvalidDataException("No time base found");

                    if (dtf != null)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(currTime, dtf.Trip.Duration, pathPart.From.Id, Color.LightBlue, i);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += dtf.Trip.Duration;
                    }
                    if (pathPart.DrivingTimes.Count >= 1)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.DrivingTimes[0],
                            pathPart.DrivingTimes.Count == 1 ? $"{pathPart.From.Id} -> {pathPart.To.Id}" : $"{pathPart.From.Id} -> charger",
                            Color.Blue,
                            i
                        );
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.DrivingTimes[0];
                    }
                    if (pathPart.ChargingTime > 0)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.ChargingTime,
                            $"charge {Formatting.Time.HHMMSS(pathPart.ChargingTime)} / {pathPart.ChargeGained:0.#}%",
                            Color.Yellow,
                            i
                        );
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.ChargingTime;
                    }
                    if (pathPart.DrivingTimes.Count == 2)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.DrivingTimes[1],
                            $"charger -> {pathPart.To.Id}",
                            Color.Blue,
                            i
                        );
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.DrivingTimes[1];
                    }
                    if (pathPart.IdleTime > 0)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.IdleTime,
                            $"idle @ {pathPart.To.Id}",
                            Color.LightGray,
                            i
                        );
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.IdleTime;
                    }
                }
            }
            graph.LayerConstraints.RemoveAllConstraints();

            foreach (var pn in nodes)
            {
                for (int i = 0; i < pn.Count; i++)
                {
                    for (int j = i + 1; j < pn.Count; j++)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(pn[i], pn[j]);
                    }
                }
            }

            for (int i = 0; i < nodes.Count; i++)
            {
                for (int j = i + 1; j < nodes.Count; j++)
                {
                    var pn1 = nodes[i];
                    var pn2 = nodes[j];
                    foreach (var n1 in pn1)
                    {
                        foreach (var n2 in pn2)
                        {
                            graph.LayerConstraints.AddUpDownConstraint(n1, n2);
                        }
                    }
                }
            }

            return graph;
        }

        public bool Solve()
        {
            GRBEnv env = new();
            env.LogToConsole = 1;
            env.LogFile = "evsp_discrete.log";

            model = new(env);
            model.SetCallback(new CustomGRBCallback());

            // Minimize the costs of all driven deadheads
            List<GRBVar> deadheadVars = new(feasibleDeadheads.Count);
            deadheadVarMapping = new();
            GRBLinExpr obj = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                var fdh = feasibleDeadheads[i];
                string id = $"x_{fdh.From.Id}->{fdh.To.Id}";
                GRBVar v = model.AddVar(0, 1, 1, GRB.INTEGER, id);
                deadheadVars.Add(v);
                deadheadVarMapping[id] = fdh;
                obj.AddTerm(fdh.Costs, v);
            }
            model.SetObjective(obj);

            // For each trip node, ensure that flow constraints are met.
            Dictionary<string, GRBLinExpr> tripFlowConstraints = new();
            Dictionary<string, GRBLinExpr> tripCoveredContraints = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                var fdh = feasibleDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (fdh.From is DiscreteTrip)
                {
                    if (!tripFlowConstraints.ContainsKey(fdh.From.Id)) tripFlowConstraints[fdh.From.Id] = new();
                    GRBLinExpr fromFlowExpr = tripFlowConstraints[fdh.From.Id];
                    fromFlowExpr.AddTerm(-1, dhVar);
                }
                if (fdh.To is DiscreteTrip dtTo)
                {
                    if (!tripFlowConstraints.ContainsKey(fdh.To.Id)) tripFlowConstraints[fdh.To.Id] = new();
                    GRBLinExpr fromFlowExpr = tripFlowConstraints[fdh.To.Id];
                    fromFlowExpr.AddTerm(1, dhVar);

                    // Only use trip id as key here, as we dont care which soc a trip is covered at
                    if (!tripCoveredContraints.ContainsKey(dtTo.Trip.Id)) tripCoveredContraints[dtTo.Trip.Id] = new();
                    GRBLinExpr fromCoverExpr = tripCoveredContraints[dtTo.Trip.Id];
                    fromCoverExpr.AddTerm(1, dhVar);
                }
            }
            // Finalize by setting expressions equal to 0
            foreach ((string name, GRBLinExpr expr) in tripFlowConstraints)
            {
                model.AddConstr(expr == 0, "flow-" + name);
            }
            foreach ((string name, GRBLinExpr expr) in tripCoveredContraints)
            {
                model.AddConstr(expr >= 1, "cover-" + name);
            }

            // Lastly, ensure that depot trips are constrained. 
            // Max outgoing = n, ingoing = outgoing
            GRBLinExpr depotBalanced = new();
            GRBLinExpr depotOutgoingConstrained = new();
            for (int i = 0; i < feasibleDeadheads.Count; i++)
            {
                var fdh = feasibleDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (fdh.From is DiscreteDepot)
                {
                    depotBalanced.AddTerm(1, dhVar);
                    depotOutgoingConstrained.AddTerm(1, dhVar);
                }
                if (fdh.To is DiscreteDepot)
                {
                    depotBalanced.AddTerm(-1, dhVar);
                }
            }
            model.AddConstr(depotBalanced == 0, "depot_balanced");
            model.AddConstr(depotOutgoingConstrained <= Config.MAX_VEHICLES, "depot_balanced");


            Console.WriteLine("Model initialization finished");
            Console.WriteLine("Starting solver");

            // Solve
            model.Optimize();

            Console.WriteLine("Solver finished");
            if (model.Status == GRB.Status.INFEASIBLE)
            {
                Console.Write("Result was infeasible");
                if (Config.DETERMINE_IIS)
                {
                    model.ComputeIIS();
                    model.Write("infeasible.ilp");
                }
                return false;
            }

            return true;
        }
    }
}
