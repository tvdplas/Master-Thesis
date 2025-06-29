using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using Gurobi;
using Microsoft.Msagl.Drawing;
using System.Collections;
using System.Text.Json;

namespace E_VCSP.Solver
{
    public abstract class EVSPNode
    {
        public int Index;
    }

    public class TripNode : EVSPNode
    {
        public required Trip Trip;
    }

    public class DepotNode : EVSPNode
    {
        public required Location Depot;
    }

    public class VSPArc
    {
        public required EVSPNode From;
        public required EVSPNode To;
        public required DeadheadTemplate DeadheadTemplate;
    }

    public class ChargingAction
    {
        public required Location ChargeLocation;
        public required DeadheadTemplate TemplateTo;
        public required DeadheadTemplate TemplateFrom;
        public double ChargeUsedTo;
        public double ChargeUsedFrom;
        public double DrivingCost;
        public int DrivingDistanceTo;
        public int DrivingDistanceFrom;
        public int DrivingTimeTo;
        public int DrivingTimeFrom;
        public int TimeAtLocation;
    }

    public class EVSPCG : Solver
    {
        private Instance instance;

        private List<VehicleTask> tasks = [];

        private List<EVSPNode> nodes = [];
        private List<List<VSPArc?>> adjFull = [];
        private List<List<VSPArc>> adj = [];

        private List<List<DeadheadTemplate?>> locationDHTMapping = [];
        private Dictionary<string, VehicleTask> varnameTaskMapping = [];
        private Dictionary<BitArray, VehicleTask> coverTaskMapping = new(new Utils.BitArrayComparer());

        private VehicleType vehicleType;

        private GRBModel? model;

        public EVSPCG(Instance instance)
        {
            this.instance = instance;
            vehicleType = instance.VehicleTypes[0];

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
                double baseCost = Config.VH_PULLOUT_COST + (dht.Distance * Config.VH_M_COST);

                adjFull[^2][i] = new VSPArc() { From = nodes[^2], To = tn, DeadheadTemplate = dht };
                adj[^2].Add(new VSPArc() { From = nodes[^2], To = tn, DeadheadTemplate = dht });
            }
            // trip -> depot end arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate? dht = locationDHTMapping[tn.Trip.To.Index][depot.Index] ?? throw new InvalidDataException("No travel possible from trip to depot");
                adjFull[i][^1] = new VSPArc() { To = nodes[^1], From = tn, DeadheadTemplate = dht };
                adj[i].Add(new VSPArc() { To = nodes[^1], From = tn, DeadheadTemplate = dht });
            }
            // depot -> depot arc
            {
                DeadheadTemplate? dht = locationDHTMapping[depot.Index][depot.Index] ?? throw new InvalidDataException("No travel possible from depot to depot");
                adjFull[^2][^1] = new VSPArc() { To = nodes[^1], From = nodes[^2], DeadheadTemplate = dht };
                adj[^2].Add(new VSPArc() { To = nodes[^1], From = nodes[^2], DeadheadTemplate = dht });
            }

            int totalSimplified = 0;

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

                    adjFull[i][j] = new VSPArc() { To = tn2, From = tn1, DeadheadTemplate = dht };
                    adj[i].Add(new VSPArc() { To = tn2, From = tn1, DeadheadTemplate = dht });
                }
            }

            double avgOutgoingBeforeSimplify = adj.Where((x, i) => i < instance.Trips.Count).Sum(x => x.Count) / ((double)adj.Count - 2);

            // Preprocessing: filter by min length, if one is found with low time only use that 
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn1 = (TripNode)nodes[i];
                List<int> directArcIndexes = [];

                for (int dai = 0; dai < adj[i].Count; dai++)
                {
                    VSPArc arc = adj[i][dai];
                    if (arc.To.Index < instance.Trips.Count)
                    {
                        Trip t2 = ((TripNode)arc.To).Trip;

                        int timeDiff = t2.StartTime - tn1.Trip.EndTime;
                        if (timeDiff <= Config.VSP_PRE_DIRECT_TIME)
                        {
                            directArcIndexes.Add(t2.Index);
                        }
                    }
                }

                if (directArcIndexes.Count > 0)
                {
                    totalSimplified++;
                    adj[i] = adj[i].Where((x, i) => x.To.Index >= instance.Trips.Count || directArcIndexes.Contains(x.To.Index)).ToList();
                    adjFull[i] = adjFull[i].Select((x, i) => x == null || x.To.Index >= instance.Trips.Count || directArcIndexes.Contains(x.To.Index) ? x : null).ToList();
                }
            }

            double avgOutgoingAfterSimplify = adj.Where((x, i) => i < instance.Trips.Count).Sum(x => x.Count) / ((double)adj.Count - 2);


            Console.WriteLine($"Simplified the arcs of {totalSimplified} trips");
            Console.WriteLine($"Avg outgoing arcs before simplification: {avgOutgoingBeforeSimplify}");
            Console.WriteLine($"Avg outgoing arcs after simplification: {avgOutgoingAfterSimplify}");
        }

        private void GenerateInitialTasks()
        {
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                Trip t = instance.Trips[i];
                DeadheadTemplate dhTo = adjFull[instance.DepotStartIndex][i]?.DeadheadTemplate ?? throw new InvalidDataException("No arc from depot to trip");
                DeadheadTemplate dhFrom = adjFull[i][instance.DepotEndIndex]?.DeadheadTemplate ?? throw new InvalidDataException("No arc from trip to depot");

                double currSoC = vehicleType.StartSoC;
                VEDeadhead toTrip = new(dhTo, instance.Trips[i].StartTime - dhTo.Duration, instance.Trips[i].StartTime, vehicleType)
                {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - dhTo.Distance * vehicleType.DriveUsage
                };
                currSoC -= dhTo.Distance * vehicleType.DriveUsage;
                VETrip trip = new(t, vehicleType)
                {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - t.Distance * vehicleType.DriveUsage
                };
                currSoC -= t.Distance * vehicleType.DriveUsage;
                VEDeadhead fromTrip = new(dhFrom, instance.Trips[i].EndTime, instance.Trips[i].EndTime + dhFrom.Duration, vehicleType)
                {
                    StartSoCInTask = currSoC,
                    EndSoCInTask = currSoC - dhFrom.Distance * vehicleType.DriveUsage
                };

                VehicleTask vehicleTask = new([toTrip, trip, fromTrip])
                {
                    vehicleType = vehicleType,
                    Index = i,
                };

                tasks.Add(vehicleTask);
            }
        }

        private List<VehicleTask> getSelectedTasks(bool console)
        {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            // trip index -> [selected vehicle task index]
            List<List<int>> coveredBy = Enumerable.Range(0, instance.Trips.Count).Select(x => new List<int>()).ToList();
            List<VehicleTask> selectedTasks = [];
            foreach (GRBVar v in model.GetVars())
            {
                if (!v.VarName.StartsWith("vt_") || v.X != 1) continue;

                VehicleTask vt = varnameTaskMapping[v.VarName];
                selectedTasks.Add(vt);
                foreach (int i in vt.Covers)
                {
                    coveredBy[i].Add(selectedTasks.Count - 1);
                }
            }


            int coveredTotal = 0;
            bool postprocessingRequired = false;
            for (int i = 0; i < coveredBy.Count; i++)
            {
                int coverCount = coveredBy[i].Count;
                if (coverCount >= 1) coveredTotal++;
                if (coverCount >= 2 && console)
                {
                    Console.WriteLine($"(!) Trip {instance.Trips[i]} covered {coverCount} times");
                    postprocessingRequired = true;
                }
            }

            if (console)
            {
                Console.WriteLine($"Covered {coveredTotal}/{instance.Trips.Count} trips");
                if (postprocessingRequired)
                    Console.WriteLine("(!) Duplicate trips found; applying postprocessing.");
                if (coveredTotal < instance.Trips.Count)
                    Console.WriteLine("(!) Not all trips covered");
            }

            if (!postprocessingRequired) return selectedTasks;

            // Two phase approach: 
            // Phase 1: replace all trips with dummy idle blocks with current from/to and currSoCs
            // Phase 2: combine adjacent blocks into 1 deadhead / idle combo. 
            List<DeadheadTemplate> extendedTemplates = [.. instance.DeadheadTemplates];
            // For each pair that was not yet included, find either a trip which does this route, or a route via the depot; 
            // take the minimum time / distance. 
            foreach (Location loc1 in instance.Locations)
            {
                foreach (Location loc2 in instance.Locations)
                {
                    if (extendedTemplates.Find(x => x.From == loc1 && x.To == loc2) != null) continue;

                    // Trip which has this route
                    Trip? trip = instance.Trips.Find(x => x.From == loc1 && x.To == loc2);
                    int tripDistance = trip?.Distance ?? int.MaxValue;
                    int tripDuration = trip?.Duration ?? int.MaxValue;

                    // Via other point (probably depot)
                    Location? loc3 = instance.Locations.Find(x =>
                        extendedTemplates.Find(y => y.From == loc1 && y.To == x) != null
                        && extendedTemplates.Find(y => y.From == x && y.To == loc2) != null
                    );
                    DeadheadTemplate? dht1 = extendedTemplates.Find(y => y.From == loc1 && y.To == loc3);
                    DeadheadTemplate? dht2 = extendedTemplates.Find(y => y.From == loc3 && y.To == loc2);

                    int detourDistance = (dht1 != null && dht2 != null) ? dht1.Distance + dht2.Distance : int.MaxValue;
                    int detourDuration = (dht1 != null && dht2 != null) ? dht1.Duration + dht2.Duration : int.MaxValue;

                    extendedTemplates.Add(new DeadheadTemplate()
                    {
                        From = loc1,
                        To = loc2,
                        Duration = Math.Min(tripDuration, detourDuration),
                        Distance = Math.Min(tripDistance, detourDistance),
                        Id = $"dht-generated-{loc1}-{loc2}",
                    });
                }
            }

            // Phase 1: dummy idle blocks
            for (int i = 0; i < coveredBy.Count; i++)
            {
                List<int> coverList = coveredBy[i];
                if (coverList.Count == 1) continue;

                // No consideration for which one is target; just use first.
                for (int j = 1; j < coverList.Count; j++)
                {
                    VehicleTask vt = selectedTasks[coverList[j]];
                    int veIndex = vt.Elements.FindIndex(x => x.Type == VEType.Trip && ((VETrip)x).Trip.Index == i);
                    VehicleElement ve = vt.Elements[veIndex];
                    Console.WriteLine($"Removing {ve} from vt {coverList[j]}");

                    // Check next / prev to see if we can combine;
                    VehicleElement? prevInteresting = null;
                    int prevInterestingIndex = veIndex - 1;
                    while (prevInterestingIndex >= 0 && prevInteresting == null)
                    {
                        List<VEType> interestingTypes = [VEType.Deadhead, VEType.Charge, VEType.Charge];
                        if (interestingTypes.Contains(vt.Elements[prevInterestingIndex].Type))
                        {
                            prevInteresting = vt.Elements[prevInterestingIndex];
                            break;
                        }
                        else prevInterestingIndex--;
                    }

                    VehicleElement? nextInteresting = null;
                    int nextInterestingIndex = veIndex + 1;
                    while (nextInterestingIndex < vt.Elements.Count && nextInteresting == null)
                    {
                        List<VEType> interestingTypes = [VEType.Deadhead, VEType.Charge, VEType.Charge];
                        if (interestingTypes.Contains(vt.Elements[nextInterestingIndex].Type))
                        {
                            nextInteresting = vt.Elements[nextInterestingIndex];
                            break;
                        }
                        else nextInterestingIndex--;
                    }

                    List<VehicleElement> newElements = [];


                    Location from = prevInteresting?.EndLocation ?? ((VETrip)ve).Trip.From;
                    Location to = nextInteresting?.StartLocation ?? ((VETrip)ve).Trip.To;
                    int startIndex = prevInterestingIndex + 1;
                    int endIndex = nextInterestingIndex - 1;

                    // We want to glue the two interesting parts together; if either end is a deadhead, more can be compressed.
                    if (prevInteresting != null && prevInteresting.Type == VEType.Deadhead)
                    {
                        from = prevInteresting.StartLocation!;
                        startIndex = prevInterestingIndex;
                    }
                    if (nextInteresting != null && nextInteresting.Type == VEType.Deadhead)
                    {
                        to = nextInteresting.EndLocation!;
                        endIndex = nextInterestingIndex;
                    }

                    // Connect t
                    DeadheadTemplate dht = extendedTemplates.Find(x => x.From == from && x.To == to)!;
                    double startSoC = vt.Elements[startIndex].StartSoCInTask;
                    double endSoC = vt.Elements[endIndex].EndSoCInTask;
                    int startTime = vt.Elements[startIndex].StartTime;
                    int endTime = vt.Elements[endIndex].EndTime;
                    int idleTime = endTime - startTime - dht.Duration;

                    var newDeadhead = new VEDeadhead(dht, startTime, startTime + dht.Duration, vehicleType)
                    {
                        StartSoCInTask = startSoC,
                        EndSoCInTask = endSoC,
                        Postprocessed = true,
                    };
                    var newIdle = new VEIdle(to, startTime + dht.Duration, endTime)
                    {
                        StartSoCInTask = endSoC,
                        EndSoCInTask = endSoC,
                        Postprocessed = true,
                    };

                    vt.Elements.RemoveRange(startIndex, endIndex - startIndex);
                    vt.Elements.InsertRange(startIndex, [newDeadhead, newIdle]);
                }
            }

            /* 
             
             
             */.


            return selectedTasks;
        }

        private List<Block> getBlocks(bool console)
        {
            return getSelectedTasks(console)
                .SelectMany(t => Block.FromVehicleTask(t))
                .Select((b, i) => { b.Index = i; return b; })
                .ToList();
        }

        public override Graph GenerateSolutionGraph(bool blockView)
        {
            List<VehicleTask> tasks = getSelectedTasks(false);

            File.WriteAllText(Config.RUN_LOG_FOLDER + "vehicleTasks.json", JsonSerializer.Serialize(tasks));

            if (blockView)
            {
                List<List<Block>> blocktasks = tasks.Select(t => Block.FromVehicleTask(t)).ToList();
                return SolutionGraph.GenerateBlockGraph(blocktasks);
            }
            else
            {
                return SolutionGraph.GenerateVehicleTaskGraph(tasks);
            }
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
            model.Parameters.TimeLimit = Config.VSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound

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
                GRBVar v = model.AddVar(0, GRB.INFINITY, tasks[i].Cost, GRB.CONTINUOUS, name);
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
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_" + t.Id);
            }

            // Finalize max vehicle constraint with slack
            // Note: added after trips so trips have easier indexing. 
            GRBVar vehicleCountSlack = model.AddVar(0, instance.Trips.Count - Config.MAX_VEHICLES, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_count_slack");
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES + vehicleCountSlack, "max_vehicles");

            this.model = model;
            return (model, taskVars);
        }

        public override bool Solve(CancellationToken ct)
        {
            (GRBModel model, List<GRBVar> taskVars) = InitModel(ct);
            model.Optimize();

            // Tracking generated columns
            int maxColumns = Config.VSP_INSTANCES_PER_IT * Config.VSP_MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                singleGenerated = 0,
                globalGenerated = 0,
                lbGenerated = 0,
                seqWithoutRC = 0,           // Number of sequential columns without reduced cost found
                totWithoutRC = 0,           // Total columns generated with no RC
                addedNew = 0,               // Total columns added to model (not initial)
                notFound = 0,               // Number of columns that could not be generated
                discardedNewColumns = 0,    // Number of columns discarded due to better one in model
                discardedOldColumns = 0;    // Number of columns in model discarded due to better one found

            // Multithreaded shortestpath searching
            List<List<VehicleShortestPath>> instances = [
                [.. Enumerable.Range(0, Config.VSP_INSTANCES_PER_IT).Select(_ => new VSPLabeling(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj))], // SP
                [.. Enumerable.Range(0, Config.VSP_INSTANCES_PER_IT).Select(_ => new VSPLSSingle(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj, locationDHTMapping))], // LS_SINGLE
                [.. Enumerable.Range(0, Config.VSP_INSTANCES_PER_IT).Select(_ => new VSPLSGlobal(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj, locationDHTMapping))], // LS_GLOBAL
            ];
            List<double> operationChances = [Config.VSP_LB_WEIGHT, Config.VSP_LS_SINGLE_WEIGHT, Config.VSP_LS_GLOBAL_WEIGHT];
            List<double> sums = [operationChances[0]];
            for (int i = 1; i < operationChances.Count; i++) sums.Add(sums[i - 1] + operationChances[i]);
            List<int> predefinedOperations = Config.VSP_OPERATION_SEQUENCE
                .Where(x => '0' <= x && x <= '9')
                .Select(x => int.Parse(x.ToString())).Reverse().ToList();

            Console.WriteLine("Column generation started");
            Console.WriteLine("%\tT\tLB\tLSS\tLSG\tAN\tNF\tDN\tDO\tWRC\tMV");

            Random rnd = new();

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.VSP_MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE)
            {
                // Terminate column generation if cancelled
                if (ct.IsCancellationRequested) return false;

                var reducedCosts = model.GetConstrs().Select(x => x.Pi);

                // Generate batch of new tasks using pricing information from previous solve
                List<(double, VehicleTask)>[] generatedTasks = new List<(double, VehicleTask)>[Config.VSP_INSTANCES_PER_IT];

                int selectedMethodIndex = -1;
                if (predefinedOperations.Count > 0)
                {
                    selectedMethodIndex = predefinedOperations[^1];
                    predefinedOperations.RemoveAt(predefinedOperations.Count - 1);
                }
                else
                {
                    double r = rnd.NextDouble() * sums[^1];
                    selectedMethodIndex = sums.FindIndex(x => r <= x);
                }

                List<VehicleShortestPath> selectedMethod = instances[selectedMethodIndex];

                if (Config.VSP_INSTANCES_PER_IT > 1)
                {
                    Parallel.For(0, Config.VSP_INSTANCES_PER_IT, (i) =>
                    {
                        generatedTasks[i] = selectedMethod[i].GenerateVehicleTasks();
                    });
                }
                else
                {
                    generatedTasks[0] = selectedMethod[0].GenerateVehicleTasks();
                }

                totalGenerated += generatedTasks.Length;
                int colsGenerated = generatedTasks.Sum(t => t.Count);
                switch (selectedMethodIndex)
                {
                    case 0: lbGenerated += colsGenerated; break;
                    case 1: singleGenerated += colsGenerated; break;
                    case 2: globalGenerated += colsGenerated; break;
                    default: throw new InvalidOperationException("You forgot to add a case");
                }

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10)
                {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{singleGenerated}\t{globalGenerated}\t{addedNew}\t{notFound}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}\t{model.ObjVal}");
                }

                foreach (var taskSet in generatedTasks)
                {
                    if (taskSet.Count == 0)
                    {
                        notFound++;
                        continue;
                    }

                    foreach (var task in taskSet)
                    {
                        (double reducedCost, VehicleTask newTask) = ((double, VehicleTask))task;

                        // Check if task is already in model 
                        BitArray ba = newTask.ToBitArray(instance.Trips.Count);
                        bool coverExists = coverTaskMapping.ContainsKey(ba);

                        // Cover already exists and is cheaper -> skip
                        if (coverExists && coverTaskMapping[ba].Cost < newTask.Cost)
                        {
                            discardedNewColumns++;
                            continue;
                        }

                        // Do not add column to model
                        if (reducedCost > 0)
                        {
                            seqWithoutRC++;
                            totWithoutRC++;
                        }
                        else
                        {
                            // Reset non-reduced costs iterations
                            seqWithoutRC = 0;

                            // Replace existing column with this task, as it has lower costs
                            if (coverExists)
                            {
                                VehicleTask toBeReplaced = coverTaskMapping[ba];
                                int index = toBeReplaced.Index;
                                newTask.Index = index;

                                // Bookkeeping; replace task in public datastructures
                                tasks[index] = newTask;
                                varnameTaskMapping[$"vt_{index}"] = newTask;
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
                                taskVars.Add(model.AddVar(0, GRB.INFINITY, newTask.Cost, GRB.CONTINUOUS, col, name));
                                varnameTaskMapping[name] = tasks[^1];
                                coverTaskMapping[ba] = tasks[^1];
                                addedNew++;
                            }
                        }
                    }
                }


                // Continue.......
                model.Update();
                model.Optimize();
                currIts++;

                if (seqWithoutRC >= Config.VSP_OPT_IT_THRESHOLD)
                {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.VSP_OPT_IT_THRESHOLD} consecutive tasks");
                    break;
                }
            }

            Console.WriteLine($"Value of relaxation: {model.ObjVal}");

            // Make model binary again
            foreach (GRBVar var in taskVars)
            {
                if (var.VarName.StartsWith("vt_"))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }

            if (!Config.VSP_ALLOW_SLACK_FINAL_SOLVE)
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

            Console.WriteLine($"Costs: {model.ObjVal}, vehicle slack: {model.GetVarByName("vehicle_count_slack").X}");

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED)
            {
                Console.WriteLine("Model infeasible / canceled");
                if (Config.VSP_DETERMINE_IIS)
                {
                    model.ComputeIIS();
                    model.Write("infeasible_CG.ilp");
                }

                Config.CONSOLE_GUROBI = configState;
                return false;
            }


            instance.Blocks = getBlocks(true);
            Config.CONSOLE_GUROBI = configState;
            return true;
        }
    }
}
