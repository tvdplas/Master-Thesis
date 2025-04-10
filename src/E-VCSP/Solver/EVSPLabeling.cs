using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Solver
{
    internal abstract class EVSPNode
    {
        internal int Index;
    }

    internal class TripNode : EVSPNode
    {
        internal required Trip Trip;
    }

    internal class DepotNode : EVSPNode
    {
        internal required Location Depot;
    }

    internal class Arc
    {
        internal required EVSPNode From;
        internal required EVSPNode To;
        internal required LabelDeadhead Deadhead;
    }

    internal class ChargingAction
    {
        internal required Location ChargeLocation;
        internal double ChargeUsedTo;
        internal double ChargeUsedFrom;
        internal double DrivingCost;
        internal int DrivingDistanceTo;
        internal int DrivingDistanceFrom;
        internal int DrivingTimeTo;
        internal int DrivingTimeFrom;
        internal int TimeAtLocation;
    }

    internal class LabelDeadhead
    {
        internal required DeadheadTemplate DeadheadTemplate;
        internal required List<ChargingAction> ChargingActions;
        internal double BaseCost;
    }

    internal record SPLabel(int prevIndex, int prevId, double currSoC, double costs, int id);

    internal class Front
    {
        private SortedList<double, SPLabel> front = new();

        internal void Insert(SPLabel label)
        {
            int l = 0, r = front.Count;
            // find first item that is larger than or equal to label currSoC
            while (l < r)
            {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.currSoC) r = m;
                else l = m + 1;
            }

            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++)
            {
                // Skip once domination is found
                if (front.Values[i].costs <= label.costs) return;
            }

            // Label is inserted, all dominated items need to be removed
            for (int i = l - 1; i >= 0 && i < front.Count; i--)
            {
                var b = front.Values[i];
                if (label.costs <= b.costs)
                {
                    front.RemoveAt(i);
                }
            }
            front[label.currSoC] = label;
        }
        internal int Count => front.Count;

        internal void Clear() => front.Clear();

        internal SPLabel Pop()
        {
            int lastIndex = front.Count - 1;
            var label = front.Values[lastIndex];
            front.RemoveAt(lastIndex);
            return label;
        }
    }


    internal class EVSPLabeling : Solver
    {
        private Instance instance;

        private List<VehicleTask> tasks = new();

        private List<EVSPNode> nodes = new();
        private List<List<Arc?>> adjFull = new();
        private List<List<Arc>> adj = new();

        private List<List<DeadheadTemplate?>> locationDHTMapping = new();
        private Dictionary<string, VehicleTask> varTaskMapping = new();

        private VehicleType vt;

        private GRBModel model;

        private List<List<SPLabel>> allLabels = new();
        private List<Front> activeLabels = new();

        internal EVSPLabeling(Instance instance)
        {
            this.instance = instance;
            vt = instance.VehicleTypes[0];

            // Generate lookup table for deadhead templates 
            foreach (Location l1 in instance.Locations)
            {
                locationDHTMapping.Add(new());
                foreach (Location l2 in instance.Locations)
                {
                    DeadheadTemplate? dht = instance.DeadheadTemplates.Find((x) => x.From == l1 && x.To == l2);
                    locationDHTMapping[l1.Index].Add(dht);
                }
            }

            // Generate graph 
            GenerateGraph();

            // Generate initial set of vehicle tasks
            GenerateInitialTasks();

            // Initialize labels
            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels.Add(new());
                activeLabels.Add(new());
            }
        }

        private void GenerateGraph()
        {
            // Transform trips into nodes, add depot start/end
            // Depot start = ^2, depot end = ^1
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                nodes.Add(new TripNode() { Index = i, Trip = instance.Trips[i] });
            }

            Location? depot = instance.Locations.Find(loc => loc.IsDepot);
            if (depot == null) throw new InvalidDataException("No depot found in location list");
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // Start depot
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // End depot


            // Initialize adjacency lists
            for (int i = 0; i < nodes.Count; i++)
            {
                adjFull.Add(new());
                adj.Add(new());
                for (int j = 0; j < nodes.Count; j++)
                {
                    adjFull[i].Add(null);
                }
            }

            // depot start arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate? dht = locationDHTMapping[depot.Index][tn.Trip.From.Index];
                if (dht == null) throw new InvalidDataException("No travel possible from depot to trip");
                double baseCost = Config.PULLOUT_COST + (dht.Distance * Config.M_COST);

                // TODO: Charge directly after depot?
                adjFull[^2][i] = new Arc() { From = nodes[^2], To = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } };
                adj[^2].Add(new Arc() { From = nodes[^2], To = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } });
            }
            // depot end arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate? dht = locationDHTMapping[tn.Trip.To.Index][depot.Index];
                if (dht == null) throw new InvalidDataException("No travel possible from trip to depot");
                double baseCost = dht.Distance * Config.M_COST;
                // TODO: Charge directly before depot?
                adjFull[i][^1] = new Arc() { To = nodes[^1], From = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } };
                adj[i].Add(new Arc() { To = nodes[^1], From = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } });
            }

            // Trip to trip arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn1 = (TripNode)nodes[i];

                for (int j = 0; j < nodes.Count - 2; j++)
                {
                    if (i == j) continue;

                    TripNode tn2 = (TripNode)nodes[j];
                    DeadheadTemplate? dht = locationDHTMapping[tn1.Trip.To.Index][tn2.Trip.From.Index];
                    if (dht == null) continue; // not a possible drive
                    if (tn1.Trip.EndTime + dht.Duration > tn2.Trip.StartTime) continue; // Deadhead not time feasible

                    double baseCost = dht.Distance * Config.M_COST;
                    List<ChargingAction> chargingActions = new List<ChargingAction>();
                    foreach (Location chargeLocation in instance.ChargingLocations)
                    {
                        DeadheadTemplate? dhtTo = locationDHTMapping[tn1.Trip.To.Index][chargeLocation.Index];
                        DeadheadTemplate? dhtFrom = locationDHTMapping[chargeLocation.Index][tn2.Trip.From.Index];

                        if (dhtTo == null || dhtFrom == null) continue;

                        int chargeTime = tn2.Trip.StartTime - (tn1.Trip.EndTime + dhtTo.Duration + dhtFrom.Duration);
                        if (chargeTime < Config.MIN_CHARGE_TIME) continue; // Charging at location not feasible

                        chargingActions.Add(new ChargingAction()
                        {
                            ChargeLocation = chargeLocation,
                            ChargeUsedTo = dhtTo.Distance * vt.DriveUsage,
                            ChargeUsedFrom = dhtFrom.Distance * vt.DriveUsage,
                            DrivingCost = (dhtTo.Distance + dhtFrom.Distance) * Config.M_COST,
                            DrivingTimeFrom = dhtFrom.Duration,
                            DrivingTimeTo = dhtTo.Duration,
                            DrivingDistanceTo = dhtTo.Distance,
                            DrivingDistanceFrom = dhtFrom.Distance,
                            TimeAtLocation = chargeTime,
                        });
                    }

                    adjFull[i][j] = new Arc() { To = tn2, From = tn1, Deadhead = new() { ChargingActions = chargingActions, BaseCost = baseCost, DeadheadTemplate = dht } };
                    adj[i].Add(new Arc() { To = tn2, From = tn1, Deadhead = new() { ChargingActions = chargingActions, BaseCost = baseCost, DeadheadTemplate = dht } });
                }
            }
        }

        private void GenerateInitialTasks()
        {
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                LabelDeadhead? dhTo = adjFull[^2][i]?.Deadhead;
                if (dhTo == null) throw new InvalidDataException("No arc from depot to trip");
                VEDeadhead toTrip = new VEDeadhead()
                {
                    Deadhead = dhTo,
                    EndTime = ((TripNode)nodes[dhTo.DeadheadTemplate.To.Index]).Trip.StartTime,
                    StartTime = ((TripNode)nodes[dhTo.DeadheadTemplate.To.Index]).Trip.StartTime - dhTo.DeadheadTemplate.Duration,
                };

                LabelDeadhead? dhFrom = adjFull[i][^1]?.Deadhead;
                if (dhFrom == null) throw new InvalidDataException("No arc from trip to depot");
                VEDeadhead fromTrip = new VEDeadhead()
                {
                    Deadhead = dhFrom,
                    StartTime = ((TripNode)nodes[dhFrom.DeadheadTemplate.From.Index]).Trip.EndTime,
                    EndTime = ((TripNode)nodes[dhFrom.DeadheadTemplate.To.Index]).Trip.EndTime + dhFrom.DeadheadTemplate.Duration,
                };

                Trip t = ((TripNode)nodes[i]).Trip;
                VETrip trip = new VETrip()
                {
                    Trip = t,
                    EndTime = t.EndTime,
                    StartTime = t.StartTime,
                };

                VehicleTask vt = new([toTrip, trip, fromTrip]);
                tasks.Add(vt);
            }
        }

        internal override Graph GenerateSolutionGraph()
        {
            List<VehicleTask> tasks = new();
            int[] covered = new int[instance.Trips.Count];
            foreach (GRBVar v in model.GetVars())
            {
                if (v.X >= Config.COL_GEN_GEQ_THRESHOLD)
                {
                    VehicleTask dvt = varTaskMapping[v.VarName];
                    Console.WriteLine($"Reduced costs: {v.RC}");
                    tasks.Add(dvt);
                    foreach (int i in dvt.Covers)
                    {
                        covered[i]++;
                    }
                }
            }
            int coveredTotal = 0;
            for (int i = 0; i < covered.Length; i++)
            {
                int val = covered[i];
                if (val >= 1) coveredTotal++;
                if (val >= 2) Console.WriteLine($"(!) Trip {instance.Trips[i]} covered {val} times");
            }
            Console.WriteLine($"Covered {coveredTotal}/{instance.Trips.Count} trips");
            if (coveredTotal < instance.Trips.Count) Console.WriteLine("(!) Not all trips covered");

            Graph graph = new();

            List<(int startTime, int endTime, List<Node> nodes)> taskNodes = new();

            for (int i = 0; i < tasks.Count; i++)
            {
                var task = tasks[i];
                List<Node> pathNodes = new();
                int startTime = task.Elements[0].StartTime, endTime = task.Elements[^1].EndTime;

                foreach (var element in task.Elements)
                {
                    int currTime = element.StartTime;

                    if (element is VETrip dvet)
                    {
                        Trip trip = dvet.Trip;
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + trip.Duration,
                            $"{trip.From} -> {trip.To} ({trip.Route})",
                            Color.LightBlue,
                        i);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                    }
                    if (element is VEDeadhead dved)
                    {
                        //Deadhead ddh = dved.Deadhead;
                        //if (ddh.DrivingTimes.Count >= 1)
                        //{
                        //    string textFrom = ddh.From is DTrip ppf ? ppf.Trip.To.Id : ddh.From.Id;
                        //    string textTo = ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id;

                        //    var node = Formatting.GraphElement.ScheduleNode(
                        //        currTime,
                        //        currTime + ddh.DrivingTimes[0],
                        //        ddh.DrivingTimes.Count == 1 ? $"{textFrom} -> {textTo}" : $"{textFrom} -> charger",
                        //        Color.Blue,
                        //        i
                        //    );
                        //    graph.AddNode(node);
                        //    pathNodes.Add(node);
                        //    currTime += ddh.DrivingTimes[0];
                        //}
                        //if (ddh.ChargingTime > 0)
                        //{
                        //    var node = Formatting.GraphElement.ScheduleNode(
                        //        currTime,
                        //        currTime + ddh.ChargingTime,
                        //        $"charge {Formatting.Time.HHMMSS(ddh.ChargingTime)} / {ddh.ChargeGained:0.#}%",
                        //        Color.Yellow,
                        //        i
                        //    );
                        //    graph.AddNode(node);
                        //    pathNodes.Add(node);
                        //    currTime += ddh.ChargingTime;
                        //}
                        //if (ddh.DrivingTimes.Count == 2)
                        //{
                        //    string text = "charger -> " + (ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id);
                        //    var node = Formatting.GraphElement.ScheduleNode(
                        //        currTime,
                        //        currTime + ddh.DrivingTimes[1],
                        //        text,
                        //        Color.Blue,
                        //        i
                        //    );
                        //    graph.AddNode(node);
                        //    pathNodes.Add(node);
                        //    currTime += ddh.DrivingTimes[1];
                        //}
                        //if (ddh.IdleTime > 0)
                        //{
                        //    string text = "idle @ " + (ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id);
                        //    var node = Formatting.GraphElement.ScheduleNode(
                        //        currTime,
                        //        currTime + ddh.IdleTime,
                        //        text,
                        //        Color.LightGray,
                        //        i
                        //    );
                        //    graph.AddNode(node);
                        //    pathNodes.Add(node);
                        //    currTime += ddh.IdleTime;
                        //}

                    }
                }
                taskNodes.Add((startTime, endTime, pathNodes));
            }

            // Add padding to the start / end of each vehicle task in order align tasks
            int minTime = taskNodes.Min(x => x.startTime);
            int maxTime = taskNodes.Max(x => x.endTime);
            for (int i = 0; i < taskNodes.Count; i++)
            {
                (int s, int e, var ns) = taskNodes[i];
                var align = Formatting.GraphElement.ScheduleNode(minTime - 300, minTime, "align" + i, Color.White, 0);
                graph.AddNode(align);
                ns.Insert(0, align);

                if (minTime < s)
                {
                    var node = Formatting.GraphElement.ScheduleNode(minTime, s, "padding1" + i, Color.White, 0);
                    graph.AddNode(node);
                    ns.Insert(1, node);
                }
                if (maxTime > e)
                {
                    var node = Formatting.GraphElement.ScheduleNode(e, maxTime, "padding2" + i, Color.White, 0);
                    graph.AddNode(node);
                    ns.Add(node);
                }

                var align2 = Formatting.GraphElement.ScheduleNode(maxTime, maxTime + 300, "align2" + i, Color.White, 0);
                graph.AddNode(align2);
                ns.Add(align2);
            }

            graph.LayoutAlgorithmSettings.NodeSeparation = 0;
            var lc = graph.LayerConstraints;
            lc.RemoveAllConstraints();

            // Force tasks to be on a single row
            foreach (var p in taskNodes) lc.AddSameLayerNeighbors(p.nodes);

            // This library isn't really built for alining nodes in a single layer;
            // force by centering nodes at beginning and end of each task
            List<Node> leftAlign = taskNodes.Select(x => x.nodes[0]).ToList(),
                       rightAlign = taskNodes.Select(x => x.nodes[^1]).ToList();
            for (int i = 0; i < leftAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(leftAlign[i], leftAlign[i + 1]);
            for (int i = 0; i < rightAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(rightAlign[i], rightAlign[i + 1]);
            return graph;
        }



        private (double minCosts, VehicleTask vehicleTask) getShortestPath()
        {
            if (Config.CONSOLE_LABELING) Console.WriteLine("Starting label correction");

            if (model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            List<double> reducedCosts = new();
            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCosts.Add(constrs[i].Pi);
            }

            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels[i].Clear();
                activeLabels[i].Clear();
            }

            int labelId = 0;
            void addLabel(SPLabel spl, int index)
            {
                allLabels[index].Add(spl);
                activeLabels[index].Insert(spl);
            }


            addLabel(new SPLabel(nodes.Count - 2, -1, vt.StartCharge, 0, labelId++), nodes.Count - 2);

            while (activeLabels.Sum(x => x.Count) > 0)
            {
                // Get label to expand
                SPLabel? l = null;
                int currIndex = -1;
                for (int i = 0; i < activeLabels.Count; i++) if (activeLabels[i].Count > 0)
                    {
                        l = activeLabels[i].Pop();
                        currIndex = i;
                        break;
                    }

                // Try to expand label
                for (int i = 0; i < adj[currIndex].Count; i++)
                {
                    Arc arc = adj[currIndex][i];
                    int targetIndex = arc.To.Index;

                    List<(double newSoC, double cost)> options = new();
                    int tripDistance = currIndex < instance.Trips.Count - 2 ? instance.Trips[currIndex].Distance : 0;
                    double tripCost = currIndex < instance.Trips.Count - 2 ? reducedCosts[currIndex] : 0;
                    double directTravelSoC = l!.currSoC -
                        ((tripDistance + arc.Deadhead.DeadheadTemplate.Distance) * vt.DriveUsage);
                    if (directTravelSoC >= vt.MinCharge)
                        options.Add((directTravelSoC, l.costs - tripCost + (arc.Deadhead.DeadheadTemplate.Distance * Config.M_COST)));

                    // TODO: dit moet VEEL efficienter
                    foreach (var chargeAction in arc.Deadhead.ChargingActions)
                    {
                        double chargeAtStation = l.currSoC - (tripDistance * vt.DriveUsage) - chargeAction.ChargeUsedTo;
                        if (chargeAtStation < vt.MinCharge) continue; // Not feasible

                        ChargingCurve cc = chargeAction.ChargeLocation.ChargingCurves[vt.Id];
                        var maxCharge = cc.MaxChargeGained(chargeAtStation, chargeAction.TimeAtLocation);

                        // Try some charging actions. 
                        double chargeRange = maxCharge.SoCGained;
                        for (int j = 0; j <= Config.DISCRETE_FACTOR; j++)
                        {
                            var res = cc.ChargeCosts(chargeAtStation, Math.Min(chargeAtStation + (j * chargeRange / Config.DISCRETE_FACTOR), 100.0));
                            double SoCAtNextTrip = chargeAtStation + res.SoCGained - chargeAction.ChargeUsedFrom;
                            if (SoCAtNextTrip < vt.MinCharge) continue;
                            options.Add((SoCAtNextTrip, l.costs - tripCost + chargeAction.DrivingCost + res.Cost));
                        }
                    }

                    // For each label possibility, check if it is not already dominated at the target node. 
                    foreach (var option in options)
                    {
                        addLabel(new SPLabel(currIndex, l.id, option.newSoC, option.cost, labelId++), targetIndex);
                    }
                }
            }

            if (Config.CONSOLE_LABELING) Console.WriteLine($"Total of {labelId} labels considered");

            // Backtrack in order to get path
            // indexes to nodes
            List<(int, SPLabel)> path = [(allLabels.Count - 1, allLabels[^1].MinBy(x => x.costs))];
            double minCosts = allLabels[^1].Min(x => x.costs);
            while (path[^1].Item2.prevId != -1)
            {
                var prev = allLabels[path[^1].Item2.prevIndex].Find(x => x.id == path[^1].Item2.prevId);
                if (prev == null) throw new Exception("waar is mn pad gebleven");
                path.Add((path[^1].Item2.prevIndex, prev));
            }

            path.Reverse();

            List<VehicleElement> taskElements = new();
            for (int i = 0; i < path.Count - 1; i++)
            {
                int index = path[i].Item1;
                int nextIndex = path[i + 1].Item1;
                var node = nodes[index];
                if (index < instance.Trips.Count)
                {
                    taskElements.Add(new VETrip()
                    {
                        Trip = instance.Trips[index],
                        EndTime = instance.Trips[index].EndTime,
                        StartTime = instance.Trips[index].StartTime,
                    });
                }

                int startTime, endTime;
                LabelDeadhead? dh = adjFull[index][nextIndex]?.Deadhead;
                if (dh == null) throw new InvalidDataException("Huh waar is mn deadhead");

                if (index == instance.Trips.Count)
                {
                    startTime = instance.Trips[nextIndex].StartTime - dh.DeadheadTemplate.Duration;
                    endTime = instance.Trips[nextIndex].StartTime;
                }
                else if (index == instance.Trips.Count + 1)
                {
                    endTime = instance.Trips[index].EndTime;
                    startTime = instance.Trips[index].EndTime + dh.DeadheadTemplate.Duration;
                }
                else
                {
                    startTime = instance.Trips[index].StartTime;
                    endTime = instance.Trips[index].EndTime;
                }
                taskElements.Add(new VEDeadhead()
                {
                    Deadhead = dh,
                    StartTime = startTime,
                    EndTime = endTime,
                });
            }

            if (Config.CONSOLE_LABELING) Console.WriteLine($"Generated column with reduced cost of {minCosts}");

            return (minCosts, new VehicleTask(taskElements));
        }

        internal override bool Solve()
        {
            GRBEnv env = new();
            env.LogToConsole = 1;
            env.LogFile = "evsp_discrete.log";

            model = new(env);
            model.SetCallback(new CustomGRBCallback());

            GRBLinExpr maxVehicles = new();
            List<GRBVar> vars = new();
            for (int i = 0; i < tasks.Count; i++)
            {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, tasks[i].Cost, GRB.CONTINUOUS, $"vt_{i}");
                maxVehicles += v;
                vars.Add(v);
                varTaskMapping[name] = tasks[i];
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip
            foreach (Trip t in instance.Trips)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < tasks.Count; i++)
                {
                    if (tasks[i].Covers.Contains(t.Index))
                        expr.AddTerm(1, vars[i]);
                }
                model.AddConstr(expr >= 1, "cover_" + t.Id);
            }

            // Add max vehicles constraint
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES, "max_vehicles");


            model.Optimize();

            bool shouldStop = false;
            int currIts = 1;
            while (currIts < Config.MAX_COL_GEN_ITS && !shouldStop && model.Status != GRB.Status.INFEASIBLE)
            {
                // Generate new column with shortest labeled pat
                (double minCosts, VehicleTask vehicleTask) = getShortestPath();

                // Add column to model 
                if (minCosts < 0)
                {
                    tasks.Add(vehicleTask);
                    GRBConstr[] constrs = model.GetConstrs().Where((_, i) => vehicleTask.Covers.Contains(i)).ToArray();
                    GRBColumn col = new();
                    col.AddTerms(constrs.Select(_ => 1.0).ToArray(), constrs);
                    string name = $"vt_{tasks.Count - 1}";
                    model.AddVar(0, 1, vehicleTask.Cost, GRB.CONTINUOUS, col, name);
                    varTaskMapping[name] = tasks[^1];
                }

                // Continue.......
                model.Optimize();
                currIts++;
            }

            if (model.Status == GRB.Status.INFEASIBLE)
            {
                Console.WriteLine("Model infeasible");
                if (Config.DETERMINE_IIS)
                {
                    model.ComputeIIS();
                    model.Write("infeasible_CG.ilp");
                }
                return false;
            }

            return true;
        }
    }
}
