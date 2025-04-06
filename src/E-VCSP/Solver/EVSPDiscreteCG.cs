using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Solver
{
    internal class EVSPDiscreteCG : Solver
    {
        internal DGraph DGraph;
        internal List<DVehicleTask> DVehicleTasks = new();

        private Instance instance;
        private Dictionary<string, DVehicleTask> varTaskMapping = new();
        private GRBModel model;

        internal EVSPDiscreteCG(Instance instance)
        {
            this.instance = instance;
            DGraph = new(instance);

            // Create base set of columns
            foreach (var tripSoCs in DGraph.DTrips)
            {
                bool added = false;
                for (int i = 0; i < tripSoCs.Count && !added; i++)
                {
                    DTrip dt = tripSoCs[i];

                    // Find deadheads from/to depot; 
                    DDeadhead? dhDepotTrip = DGraph.DDeadheads.Find(dh => dh.From == DGraph.DDepotStart && dh.To == dt);
                    DDeadhead? dhTripDepot = DGraph.DDeadheads.Find(dh => dh.From == dt && dh.To == DGraph.DDepotEnd);

                    if (dhDepotTrip == null || dhTripDepot == null) continue;
                    DVehicleTask dvt = new([
                        new DVEDeadhead() { DDeadhead = dhDepotTrip, StartTime = dt.Trip.StartTime - dhDepotTrip.DrivingTimes[0], EndTime = dt.Trip.StartTime },
                        new DVETrip() { DTrip = dt, StartTime = dt.Trip.StartTime, EndTime = dt.Trip.EndTime },
                        new DVEDeadhead() { DDeadhead = dhTripDepot, StartTime = dt.Trip.EndTime, EndTime = dt.Trip.EndTime + dhTripDepot.DrivingTimes[0] },
                    ]);
                    DVehicleTasks.Add(dvt);
                    added = true;
                }

                if (!added) throw new InvalidDataException("Could not create initial set of columns covering all trips.");
            }
        }

        internal override Graph GenerateSolutionGraph()
        {
            List<DVehicleTask> tasks = new();
            foreach (GRBVar v in model.GetVars())
            {
                if (v.X == 1)
                {
                    DVehicleTask dvt = varTaskMapping[v.VarName];
                    tasks.Add(dvt);
                }
            }

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

                    if (element is DVETrip dvet)
                    {
                        Trip trip = dvet.DTrip.Trip;
                        var node = Formatting.GraphElement.ScheduleNode(
                            currTime,
                            currTime + trip.Duration,
                            $"{trip.From} -> {trip.To} ({trip.Route})",
                            Color.LightBlue,
                        i);
                        graph.AddNode(node);
                        pathNodes.Add(node);
                    }
                    if (element is DVEDeadhead dved)
                    {
                        DDeadhead ddh = dved.DDeadhead;
                        if (ddh.DrivingTimes.Count >= 1)
                        {
                            string textFrom = ddh.From is DTrip ppf ? ppf.Trip.To.Id : ddh.From.Id;
                            string textTo = ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id;

                            var node = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ddh.DrivingTimes[0],
                                ddh.DrivingTimes.Count == 1 ? $"{textFrom} -> {textTo}" : $"{textFrom} -> charger",
                                Color.Blue,
                                i
                            );
                            graph.AddNode(node);
                            pathNodes.Add(node);
                            currTime += ddh.DrivingTimes[0];
                        }
                        if (ddh.ChargingTime > 0)
                        {
                            var node = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ddh.ChargingTime,
                                $"charge {Formatting.Time.HHMMSS(ddh.ChargingTime)} / {ddh.ChargeGained:0.#}%",
                                Color.Yellow,
                                i
                            );
                            graph.AddNode(node);
                            pathNodes.Add(node);
                            currTime += ddh.ChargingTime;
                        }
                        if (ddh.DrivingTimes.Count == 2)
                        {
                            string text = "charger -> " + (ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id);
                            var node = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ddh.DrivingTimes[1],
                                text,
                                Color.Blue,
                                i
                            );
                            graph.AddNode(node);
                            pathNodes.Add(node);
                            currTime += ddh.DrivingTimes[1];
                        }
                        if (ddh.IdleTime > 0)
                        {
                            string text = "idle @ " + (ddh.To is DTrip ppt ? ppt.Trip.From.Id : ddh.To.Id);
                            var node = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ddh.IdleTime,
                                text,
                                Color.LightGray,
                                i
                            );
                            graph.AddNode(node);
                            pathNodes.Add(node);
                            currTime += ddh.IdleTime;
                        }

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

        internal override bool Solve()
        {
            GRBEnv env = new();
            env.LogToConsole = 1;
            env.LogFile = "evsp_discrete.log";

            model = new(env);
            model.SetCallback(new CustomGRBCallback());

            GRBLinExpr maxVehicles = new();
            List<GRBVar> vars = new();
            for (int i = 0; i < DVehicleTasks.Count; i++)
            {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, DVehicleTasks[i].Cost, GRB.BINARY, $"vt_{i}");
                maxVehicles += v;
                vars.Add(v);
                varTaskMapping[name] = DVehicleTasks[i];
            }
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES, "max_vehicles");

            // Add cover constraint for each of the trips
            foreach (Trip t in instance.Trips)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < DVehicleTasks.Count; i++)
                {
                    if (DVehicleTasks[i].Covers.Contains(t.Id))
                        expr.AddTerm(1, vars[i]);
                }
                model.AddConstr(expr >= 1, "cover_" + t.Id);
            }

            model.Relax();
            model.Optimize();

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
