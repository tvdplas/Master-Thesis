using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Solver
{
    public class EVSPDiscrete : Solver
    {

        public DGraph DGraph;

        private Dictionary<string, DDeadhead> deadheadVarMapping = new();
        private GRBModel? model;

        public EVSPDiscrete(Instance instance)
        {
            DGraph = new(instance);
        }

        public override bool Solve(CancellationToken cancellationToken)
        {
            GRBEnv env = new();
            env.LogToConsole = 1;
            env.LogFile = "evsp_discrete.log";

            model = new(env);
            model.SetCallback(new CustomGRBCallback());

            // Minimize the costs of all driven deadheads
            List<GRBVar> deadheadVars = new(DGraph.DDeadheads.Count);
            deadheadVarMapping = new();
            GRBLinExpr obj = new();
            for (int i = 0; i < DGraph.DDeadheads.Count; i++)
            {
                var fdh = DGraph.DDeadheads[i];
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

            // Ensure that constraints are added even if there are no headheads
            // Useful for IIS if there are mistakes in the deadheads
            foreach (var dts in DGraph.DTrips)
            {
                tripCoveredContraints[dts.First().Trip.Id] = new();
                foreach (var dt in dts)
                {
                    tripFlowConstraints[dt.Id] = new();
                }
            }

            for (int i = 0; i < DGraph.DDeadheads.Count; i++)
            {
                var fdh = DGraph.DDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (fdh.From is DTrip)
                {
                    GRBLinExpr fromFlowExpr = tripFlowConstraints[fdh.From.Id];
                    fromFlowExpr.AddTerm(-1, dhVar);
                }
                if (fdh.To is DTrip dtTo)
                {
                    GRBLinExpr fromFlowExpr = tripFlowConstraints[fdh.To.Id];
                    fromFlowExpr.AddTerm(1, dhVar);

                    // Only use trip id as key here, as we dont care which soc a trip is covered at
                    GRBLinExpr fromCoverExpr = tripCoveredContraints[dtTo.Trip.Id];
                    fromCoverExpr.AddTerm(1, dhVar);
                }
            }
            // Finalize by setting expressions equal to 0
            foreach ((string name, GRBLinExpr expr) in tripFlowConstraints)
            {
                model.AddConstr(expr == 0, "flow-" + 7);
            }
            foreach ((string name, GRBLinExpr expr) in tripCoveredContraints)
            {
                model.AddConstr(expr >= 1, "cover-" + name);
            }

            // Lastly, ensure that depot trips are constrained. 
            // Max outgoing = n, ingoing = outgoing
            GRBLinExpr depotBalanced = new();
            GRBLinExpr depotOutgoingConstrained = new();
            for (int i = 0; i < DGraph.DDeadheads.Count; i++)
            {
                var fdh = DGraph.DDeadheads[i];
                GRBVar dhVar = deadheadVars[i];
                if (fdh.From is DDepot)
                {
                    depotBalanced.AddTerm(1, dhVar);
                    depotOutgoingConstrained.AddTerm(1, dhVar);
                }
                if (fdh.To is DDepot)
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

        public override Graph GenerateSolutionGraph(bool blockView)
        {
            if (model == null) throw new InvalidDataException("Cannot generate solution graph without model instance");

            Console.WriteLine("Reconstructing paths");
            List<List<(int, DDeadhead)>> adj = new(); // trips 1->n are map to 0->n-1, depot start = n, depot end = n + 1
            HashSet<int> visited = new(); // Validation that all nodes are actually visited
            for (int i = 0; i < DGraph.DTrips.Count + 2; i++) adj.Add(new());
            GRBVar[] vars = model.GetVars();
            for (int i = 0; i < vars.Length; i++)
            {
                GRBVar v = vars[i];
                if (v.X == 1)
                {
                    // Corresponding feasibleDeadhead
                    DDeadhead fdh = deadheadVarMapping[v.VarName];

                    int from = fdh.From is DDepot
                        ? (fdh.From.Id.Contains("start") ? DGraph.DTrips.Count : DGraph.DTrips.Count + 1)
                        : int.Parse(((DTrip)fdh.From).Trip.Id.Substring(1)) - 1;
                    int to = fdh.To is DDepot
                        ? (fdh.To.Id.Contains("start") ? DGraph.DTrips.Count : DGraph.DTrips.Count + 1)
                        : int.Parse(((DTrip)fdh.To).Trip.Id.Substring(1)) - 1;
                    adj[from].Add((to, fdh));
                    Console.WriteLine($"{v.VarName} used; adj[{from}][{to}] = 1");
                }
            }

            List<List<DDeadhead>> paths = new();
            List<DDeadhead> dfs(int curr, List<List<(int, DDeadhead)>> adj, List<DDeadhead> currPath)
            {
                visited.Add(curr);
                if (curr == adj.Count - 1)
                {
                    return currPath;
                }

                if (adj[curr].Count == 0) throw new InvalidDataException("Something went wrong; there should always be a path from source to sink");

                (int next, DDeadhead dh) = adj[curr][^1];
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
            Console.WriteLine($"Total trips: {DGraph.DTrips.Count}");
            Console.WriteLine($"Trips covered: {visited.Count - 2}"); // -2 due to depot start and end


            // Time to create the actual nodes representing the times; We now distinguish between 4 seperate node types: 
            // 1. Trips (light blue)
            // 2. Deadhead drives (dark blue)
            // 3. Charging actions (yellow)
            // 4. Idle times (light gray)
            // Idle times at the beginning / end of the day are padded in order to ensure that all graph elements are time-aligned

            Graph graph = new();

            List<(int startTime, int endTime, List<Node?> nodes)> nodes = new();
            for (int i = 0; i < paths.Count; i++)
            {
                var path = paths[i];
                List<Node?> pathNodes = new();
                int startTime = int.MaxValue, endTime = int.MinValue;

                foreach (var pathPart in path)
                {
                    int currTime = -1;
                    DTrip? dtf = null, dtt = null;
                    if (pathPart.From is DTrip)
                    {
                        dtf = (DTrip)pathPart.From;
                        currTime = dtf.Trip.StartTime;
                    }
                    if (currTime == -1 && pathPart.To is DTrip)
                    {
                        dtt = (DTrip)pathPart.To;
                        currTime = dtt.Trip.StartTime - pathPart.DrivingTimes.Sum() - pathPart.ChargingTime - pathPart.IdleTime;
                    }
                    if (currTime == -1) throw new InvalidDataException("No time base found");

                    if (currTime < startTime) startTime = currTime;

                    if (dtf != null)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(currTime, currTime + dtf.Trip.Duration, $"{dtf.Trip.From} -> {dtf.Trip.To} ({dtf.Trip.Route})", Color.LightBlue);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += dtf.Trip.Duration;
                    }
                    if (pathPart.DrivingTimes.Count == 1)
                    {
                        string textFrom = pathPart.From is DTrip ppf ? ppf.Trip.To.Id : pathPart.From.Id;
                        string textTo = pathPart.To is DTrip ppt ? ppt.Trip.From.Id : pathPart.To.Id;


                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.DrivingTimes[0],
                            $"{textFrom} -> {textTo}",
                            Color.Blue);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.DrivingTimes[0];
                    }
                    if (pathPart.DrivingTimes.Count > 1)
                    {
                        string textFrom = pathPart.From is DTrip ppf ? ppf.Trip.To.Id : pathPart.From.Id;
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.DrivingTimes[0],
                            $"{textFrom} -> charger",
                            Color.Blue);
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
                            Color.Yellow);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.ChargingTime;
                    }
                    if (pathPart.DrivingTimes.Count == 2)
                    {
                        string text = "charger -> " + (pathPart.To is DTrip ppt ? ppt.Trip.From.Id : pathPart.To.Id);
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.DrivingTimes[1],
                            text,
                            Color.Blue);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.DrivingTimes[1];
                    }
                    if (pathPart.IdleTime > 0)
                    {
                        string text = "idle @ " + (pathPart.To is DTrip ppt ? ppt.Trip.From.Id : pathPart.To.Id);
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + pathPart.IdleTime,
                            text,
                            Color.LightGray);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                        currTime += pathPart.IdleTime;
                    }

                    if (currTime > endTime) endTime = currTime;
                }
                nodes.Add((startTime, endTime, pathNodes));
            }

            // Add padding to the start / end of each vehicle task in order align tasks
            int minTime = nodes.Min(x => x.startTime);
            int maxTime = nodes.Max(x => x.endTime);
            for (int i = 0; i < nodes.Count; i++)
            {
                (int s, int e, var ns) = nodes[i];
                var align = GraphElement.ScheduleNode(minTime - 300, minTime, "align" + i, Color.White);
                graph.AddNode(align);
                ns.Insert(0, align);

                if (minTime < s)
                {
                    var node = GraphElement.ScheduleNode(minTime, s, "padding1" + i, Color.White);
                    graph.AddNode(node);
                    ns.Insert(1, node);
                }
                if (maxTime > e)
                {
                    var node = GraphElement.ScheduleNode(e, maxTime, "padding2" + i, Color.White);
                    graph.AddNode(node);
                    ns.Add(node);
                }

                var align2 = GraphElement.ScheduleNode(minTime - 300, minTime, "align2" + i, Color.White);
                graph.AddNode(align2);
                ns.Add(align2);
            }

            graph.LayoutAlgorithmSettings.NodeSeparation = 0;
            var lc = graph.LayerConstraints;
            lc.RemoveAllConstraints();

            // Force tasks to be on a single row
            foreach (var p in nodes) lc.AddSameLayerNeighbors(p.nodes);

            // This library isn't really built for alining nodes in a single layer;
            // force by centering nodes at beginning and end of each task
            List<Node?> leftAlign = nodes.Select(x => x.nodes[0]).ToList(),
                       rightAlign = nodes.Select(x => x.nodes[^1]).ToList();
            for (int i = 0; i < leftAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(leftAlign[i], leftAlign[i + 1]);
            for (int i = 0; i < rightAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(rightAlign[i], rightAlign[i + 1]);
            return graph;
        }
    }
}
