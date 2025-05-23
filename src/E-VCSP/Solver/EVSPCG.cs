﻿using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;
using Microsoft.Msagl.Drawing;
using System.Collections;
using Color = Microsoft.Msagl.Drawing.Color;
using Node = Microsoft.Msagl.Drawing.Node;

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
        internal required Deadhead Deadhead;
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

    internal class Deadhead
    {
        internal required DeadheadTemplate DeadheadTemplate;
        internal required List<ChargingAction> ChargingActions;
        internal double BaseDrivingCost;
    }

    internal class EVSPCG : Solver
    {
        private Instance instance;

        private List<VehicleTask> tasks = [];

        private List<EVSPNode> nodes = [];
        private List<List<Arc?>> adjFull = [];
        private List<List<Arc>> adj = [];

        private List<List<DeadheadTemplate?>> locationDHTMapping = [];
        private Dictionary<string, VehicleTask> varnameTaskMapping = [];
        private Dictionary<BitArray, VehicleTask> coverTaskMapping = new(new Utils.BitArrayComparer());

        private VehicleType vt;

        private GRBModel? model;

        internal EVSPCG(Instance instance)
        {
            this.instance = instance;
            vt = instance.VehicleTypes[0];

            // Generate lookup table for deadhead templates 
            foreach (Location l1 in instance.Locations)
            {
                locationDHTMapping.Add([]);
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
        }

        private void GenerateGraph()
        {
            // Transform trips into nodes, add depot start/end
            // Depot start = ^2, depot end = ^1
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                nodes.Add(new TripNode() { Index = i, Trip = instance.Trips[i] });
            }

            Location? depot = instance.Locations.Find(loc => loc.IsDepot) ?? throw new InvalidDataException("No depot found in location list");
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // Start depot
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // End depot


            // Initialize adjacency lists
            for (int i = 0; i < nodes.Count; i++)
            {
                adjFull.Add([]);
                adj.Add([]);
                for (int j = 0; j < nodes.Count; j++)
                {
                    adjFull[i].Add(null);
                }
            }

            // depot start -> trip arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate? dht = locationDHTMapping[depot.Index][tn.Trip.From.Index] ?? throw new InvalidDataException("No travel possible from depot to trip");
                double baseCost = Config.PULLOUT_COST + (dht.Distance * Config.M_COST);

                // TODO: Charge directly after depot?
                Deadhead dh = new() { ChargingActions = [], BaseDrivingCost = baseCost, DeadheadTemplate = dht };
                adjFull[^2][i] = new Arc() { From = nodes[^2], To = tn, Deadhead = dh };
                adj[^2].Add(new Arc() { From = nodes[^2], To = tn, Deadhead = dh });
            }
            // trip -> depot end arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate? dht = locationDHTMapping[tn.Trip.To.Index][depot.Index] ?? throw new InvalidDataException("No travel possible from trip to depot");
                double baseCost = dht.Distance * Config.M_COST;
                // TODO: Charge directly before depot?
                Deadhead dh = new() { ChargingActions = [], BaseDrivingCost = baseCost, DeadheadTemplate = dht };
                adjFull[i][^1] = new Arc() { To = nodes[^1], From = tn, Deadhead = dh };
                adj[i].Add(new Arc() { To = nodes[^1], From = tn, Deadhead = dh });
            }
            // depot -> depot arc
            {
                DeadheadTemplate? dht = locationDHTMapping[depot.Index][depot.Index] ?? throw new InvalidDataException("No travel possible from depot to depot");
                double baseCost = dht.Distance * Config.M_COST;
                // TODO: Charge directly before depot?
                Deadhead dh = new() { ChargingActions = [], BaseDrivingCost = baseCost, DeadheadTemplate = dht };
                adjFull[^2][^1] = new Arc() { To = nodes[^1], From = nodes[^2], Deadhead = dh };
                adj[^2].Add(new Arc() { To = nodes[^1], From = nodes[^2], Deadhead = dh });
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
                    List<ChargingAction> chargingActions = [];
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

                    Deadhead dh = new() { ChargingActions = chargingActions, BaseDrivingCost = baseCost, DeadheadTemplate = dht };

                    adjFull[i][j] = new Arc() { To = tn2, From = tn1, Deadhead = dh };
                    adj[i].Add(new Arc() { To = tn2, From = tn1, Deadhead = dh });
                }
            }
        }

        private void GenerateInitialTasks()
        {
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                Deadhead dhTo = (adjFull[^2][i]?.Deadhead) ?? throw new InvalidDataException("No arc from depot to trip");
                VEDeadhead toTrip = new()
                {
                    Deadhead = dhTo,
                    EndTime = ((TripNode)nodes[dhTo.DeadheadTemplate.To.Index]).Trip.StartTime,
                    StartTime = ((TripNode)nodes[dhTo.DeadheadTemplate.To.Index]).Trip.StartTime - dhTo.DeadheadTemplate.Duration,
                    DrivingCost = dhTo.BaseDrivingCost,
                    SoCDiff = -dhTo.DeadheadTemplate.Duration * vt.DriveUsage,
                };

                Deadhead dhFrom = adjFull[i][^1]?.Deadhead ?? throw new InvalidDataException("No arc from trip to depot");
                VEDeadhead fromTrip = new()
                {
                    Deadhead = dhFrom,
                    StartTime = ((TripNode)nodes[dhFrom.DeadheadTemplate.From.Index]).Trip.EndTime,
                    EndTime = ((TripNode)nodes[dhFrom.DeadheadTemplate.To.Index]).Trip.EndTime + dhFrom.DeadheadTemplate.Duration,
                    DrivingCost = dhFrom.BaseDrivingCost,
                    SoCDiff = -dhFrom.DeadheadTemplate.Duration * vt.DriveUsage,
                };

                Trip t = ((TripNode)nodes[i]).Trip;
                VETrip trip = new()
                {
                    Trip = t,
                    EndTime = t.EndTime,
                    StartTime = t.StartTime,
                    DrivingCost = 0,
                    SoCDiff = -t.Duration * vt.DriveUsage,
                };

                VehicleTask vehicleTask = new([toTrip, trip, fromTrip])
                {
                    Index = i,
                };
                tasks.Add(vehicleTask);
            }
        }

        internal override Graph GenerateSolutionGraph()
        {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<VehicleTask> tasks = [];
            int[] covered = new int[instance.Trips.Count];
            foreach (GRBVar v in model.GetVars())
            {
                if (v.VarName.StartsWith("vt_") && v.X == 1)
                {
                    VehicleTask dvt = varnameTaskMapping[v.VarName];
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

            List<(int startTime, int endTime, List<Node?> nodes)> taskNodes = [];

            for (int i = 0; i < tasks.Count; i++)
            {
                var task = tasks[i];
                List<Node?> pathNodes = [];
                int startTime = task.Elements[0].StartTime, endTime = task.Elements[^1].EndTime;
                void add(Node? node)
                {
                    if (node != null)
                    {
                        graph.AddNode(node);
                        pathNodes.Add(node);
                    }
                }

                foreach (var element in task.Elements)
                {
                    string SoCAtStart = element.StartSoCInTask != null ? ((int)element.StartSoCInTask).ToString() : "";
                    string SoCAtEnd = element.EndSoCInTask != null ? ((int)element.EndSoCInTask).ToString() : "";

                    if (element is VEDepot) continue;
                    if (element is VETrip dvet)
                    {
                        Trip trip = dvet.Trip;
                        var node = Formatting.GraphElement.ScheduleNode(
                            element.StartTime,
                            element.EndTime,
                            $"{trip.From}@{SoCAtStart} -> {trip.To}@{SoCAtEnd} ({trip.Route})",
                            Color.LightBlue);
                        add(node);
                    }
                    if (element is VEIdle idle)
                    {
                        var node = Formatting.GraphElement.ScheduleNode(
                            element.StartTime,
                            element.EndTime,
                            $"idle {idle.Location.Id}@{SoCAtStart}",
                            Color.White);
                        add(node);
                    }
                    if (element is VEDeadhead dved)
                    {
                        Deadhead ddh = dved.Deadhead;

                        if (dved.SelectedAction == -1)
                        {
                            var node = Formatting.GraphElement.ScheduleNode(
                                element.StartTime,
                                element.EndTime,
                                $"{ddh.DeadheadTemplate.From}@{SoCAtStart} -> {ddh.DeadheadTemplate.To}@{SoCAtEnd} ",
                                Color.LightGreen);
                            add(node);
                        }
                        else
                        {
                            ChargingAction ca = ddh.ChargingActions[dved.SelectedAction];
                            int currTime = element.StartTime;
                            double currSoC = element.StartSoCInTask ?? -10000000;
                            var toCharger = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ca.DrivingTimeTo,
                                $"{ddh.DeadheadTemplate.From}@{(int)currSoC} -> {ca.ChargeLocation}@{(int)(currSoC - ca.ChargeUsedTo)}",
                                Color.GreenYellow);
                            currTime += ca.DrivingTimeTo;
                            currSoC -= ca.ChargeUsedTo;
                            var charge = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + dved.ChargeTime,
                                $"{ca.ChargeLocation} {(int)currSoC}% -> {(int)(currSoC + dved.ChargeGained)}%",
                                Color.Yellow);
                            currTime += dved.ChargeTime;
                            currSoC += dved.ChargeGained;
                            var fromCharger = Formatting.GraphElement.ScheduleNode(
                                currTime,
                                currTime + ca.DrivingTimeFrom,
                                $"{ca.ChargeLocation}@{(int)currSoC} -> {ddh.DeadheadTemplate.To}@{(int)(currSoC - ca.ChargeUsedFrom)}",
                                Color.GreenYellow);
                            add(toCharger);
                            add(charge);
                            add(fromCharger);
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
                var align = Formatting.GraphElement.ScheduleNode(
                    minTime - 1000,
                    minTime,
                    $"VT {tasks[i].Index}\nCosts: {tasks[i].Cost}",
                    Color.White);
                graph.AddNode(align);
                ns.Insert(0, align);

                if (minTime < s)
                {
                    var node = Formatting.GraphElement.ScheduleNode(minTime, s, "padding1" + i, Color.White);
                    graph.AddNode(node);
                    ns.Insert(1, node);
                }
                if (maxTime > e)
                {
                    var node = Formatting.GraphElement.ScheduleNode(e, maxTime, "padding2" + i, Color.White);
                    graph.AddNode(node);
                    ns.Add(node);
                }

                var align2 = Formatting.GraphElement.ScheduleNode(maxTime, maxTime + 300, "align2" + i, Color.White);
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
            List<Node?> leftAlign = [.. taskNodes.Select(x => x.nodes[0])],
                       rightAlign = [.. taskNodes.Select(x => x.nodes[^1])];
            for (int i = 0; i < leftAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(leftAlign[i], leftAlign[i + 1]);
            for (int i = 0; i < rightAlign.Count - 1; i++) lc.AddUpDownVerticalConstraint(rightAlign[i], rightAlign[i + 1]);
            return graph;
        }

        /// <summary>
        /// Initialize model for column generaton
        /// </summary>
        /// <param name="ct">Cancellation token</param>
        /// <returns>Model where first <c>n</c> constraints correspond to the <c>n</c> trips, and a list of initial vehicle task vars</returns>
        private (GRBModel model, List<GRBVar> taskVars) InitModel(CancellationToken ct)
        {
            // Env
            GRBEnv env = new()
            {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "evspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.SOLVER_TIMEOUT_SEC;
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() =>
            {
                if (model == null) return;

                Console.WriteLine("Cancellation requested. Terminating Gurobi model...");
                try { model.Terminate(); }
                catch (Exception ex) { Console.WriteLine($"Error terminating Gurobi: {ex.Message}"); }
            });


            // Add variable for each task/column; add to maxVehicle constraint
            GRBLinExpr maxVehicles = new();
            List<GRBVar> taskVars = [];
            for (int i = 0; i < tasks.Count; i++)
            {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, tasks[i].Cost, GRB.CONTINUOUS, name);
                maxVehicles += v;

                // Bookkeeping to find variable based on name / cover easily
                taskVars.Add(v);
                varnameTaskMapping[name] = tasks[i];
                coverTaskMapping.Add(tasks[i].ToBitArray(instance.Trips.Count), tasks[i]);
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip 
            foreach (Trip t in instance.Trips)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < tasks.Count; i++)
                {
                    if (tasks[i].Covers.Contains(t.Index))
                        expr.AddTerm(1, taskVars[i]);
                }

                // Switch between set partition and cover
                char sense = Config.ALLOW_VH_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_" + t.Id);
            }

            // Finalize max vehicle constraint with slack
            // Note: added after trips so trips have easier indexing. 
            GRBVar vehicleCountSlack = model.AddVar(0, instance.Trips.Count - Config.MAX_VEHICLES, Config.MAX_VEHICLES_OVER_COST, GRB.CONTINUOUS, "vehicle_count_slack");
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES + vehicleCountSlack, "max_vehicles");

            this.model = model;
            return (model, taskVars);
        }

        internal override bool Solve(CancellationToken ct)
        {
            (GRBModel model, List<GRBVar> taskVars) = InitModel(ct);
            model.Optimize();

            // Tracking generated columns
            int maxColumns = Config.THREADS * Config.MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                seqWithoutRC = 0,           // Number of sequential columns without reduced cost found
                totWithoutRC = 0,           // Total columns generated with no RC
                notFound = 0,               // Number of columns that could not be generated
                discardedNewColumns = 0,    // Number of columns discarded due to better one in model
                discardedOldColumns = 0;    // Number of columns in model discarded due to better one found

            // Multithreaded shortestpath searching
            List<ShortestPathLS> splss = [.. Enumerable.Range(0, Config.THREADS).Select(_ => new ShortestPathLS(instance, model, adjFull))];


            Console.WriteLine("Column generation started");
            Console.WriteLine("%\tT\tNF\tDN\tDO\tWRC");

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE)
            {
                // Terminate column generation if cancelled
                if (ct.IsCancellationRequested) return false;

                // Generate batch of new tasks using pricing information from previous solve
                (double, VehicleTask)?[] generatedTasks = new (double, VehicleTask)?[Config.THREADS];
                Parallel.For(0, Config.THREADS, (i) =>
                {
                    splss[i].Reset();
                    generatedTasks[i] = splss[i].getVehicleTaskLS();
                });
                totalGenerated += generatedTasks.Length;

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10)
                {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{notFound}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}");
                }

                foreach (var task in generatedTasks)
                {
                    if (task == null)
                    {
                        notFound++;
                        continue;
                    }
                    (double reducedCost, VehicleTask newTask) = ((double, VehicleTask))task;

                    // Check if task is already in model 
                    BitArray ba = newTask.ToBitArray(instance.Trips.Count);
                    bool coverExists = coverTaskMapping.ContainsKey(ba);

                    if (coverExists)
                    {
                        VehicleTask vt = coverTaskMapping[ba];

                        // Same cover but higher cost can be ignored safely.
                        if (vt.Cost < newTask.Cost)
                        {
                            discardedNewColumns++;
                            continue;
                        }
                    }

                    // Add column to model 
                    if (reducedCost < 0)
                    {
                        // Reset non-reduced costs iterations
                        seqWithoutRC = 0;

                        // Replace existing column with this task, as it has lower costs
                        if (coverExists)
                        {
                            VehicleTask toBeReplaced = coverTaskMapping[ba];
                            int index = toBeReplaced.Index;
                            newTask.Index = index;

                            // Bookkeeping; replace task in internal datastructures
                            tasks[index] = newTask;
                            string name = $"vt_{index}";
                            varnameTaskMapping[name] = newTask;
                            coverTaskMapping[ba] = newTask;

                            // Adjust costs in model
                            taskVars[index].Obj = newTask.Cost;
                            discardedOldColumns++;
                        }
                        // Create new column for task, add it to model.
                        else
                        {
                            int index = tasks.Count;
                            string name = $"vt_{index}";
                            tasks.Add(newTask);
                            newTask.Index = index;

                            // Create new column to add to model
                            var modelConstrs = model.GetConstrs();
                            GRBConstr[] constrs = [.. modelConstrs.Where(
                                (_, i) => newTask.Covers.Contains(i)    // Covers trip
                                || i == modelConstrs.Length - 1        // Add to used vehicles
                            )];
                            GRBColumn col = new();
                            col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                            // Add column to model
                            taskVars.Add(model.AddVar(0, 1, newTask.Cost, GRB.CONTINUOUS, col, name));
                            varnameTaskMapping[name] = tasks[^1];
                            coverTaskMapping[ba] = tasks[^1];
                        }

                    }
                    else
                    {
                        seqWithoutRC++;
                        totWithoutRC++;
                    }
                }


                if (seqWithoutRC >= Config.LS_OPT_IT_THRESHOLD)
                {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.LS_OPT_IT_THRESHOLD} consecutive tasks");
                    break;
                }

                // Continue.......
                model.Update();
                model.Optimize();
                currIts++;
            }

            // Make model binary again
            foreach (GRBVar var in taskVars)
            {
                if (var.VarName.StartsWith(""))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }

            if (!Config.ALLOW_VH_SLACK_FINAL_SOLVE)
            {
                // Remove ability to go over vehicle bounds -> hopefully speeds up solving at end.
                model.GetVarByName("vehicle_count_slack").UB = 0;
            }

            Console.WriteLine($"Total generation attempts: ${totalGenerated}");
            Console.WriteLine($"LS failed to generate charge-feasible trip {notFound} times during generation");
            Console.WriteLine($"Discarded {discardedOldColumns} old, {discardedNewColumns} new columns during generation");
            Console.WriteLine($"{totWithoutRC} columns were not added due to positive reduced costs.");
            Console.WriteLine($"Solving non-relaxed model with total of {tasks.Count} columns");
            model.Update();
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true; // Force enable console at end as this solve takes a long time
            model.Optimize();

            Console.Write($"Costs: {model.ObjVal}, vehicle slack: {model.GetVarByName("vehicle_count_slack").X}");

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED)
            {
                Console.WriteLine("Model infeasible / canceled");
                if (Config.DETERMINE_IIS)
                {
                    model.ComputeIIS();
                    model.Write("infeasible_CG.ilp");
                }

                Config.CONSOLE_GUROBI = configState;
                return false;
            }

            Config.CONSOLE_GUROBI = configState;
            return true;
        }
    }
}
