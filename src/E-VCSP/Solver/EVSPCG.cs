using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
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

    public class Arc
    {
        public required EVSPNode From;
        public required EVSPNode To;
        public required Deadhead Deadhead;
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

    public class Deadhead
    {
        public required DeadheadTemplate DeadheadTemplate;
        public required List<ChargingAction> ChargingActions;
        public double BaseDrivingCost;
    }

    public class EVSPCG : Solver
    {
        private Instance instance;

        private List<VehicleTask> tasks = [];

        private List<EVSPNode> nodes = [];
        private List<List<Arc?>> adjFull = [];
        private List<List<Arc>> adj = [];

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
                            TemplateFrom = dhtFrom,
                            TemplateTo = dhtTo,
                            ChargeUsedTo = dhtTo.Distance * vehicleType.DriveUsage,
                            ChargeUsedFrom = dhtFrom.Distance * vehicleType.DriveUsage,
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
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                Trip t = instance.Trips[i];
                DeadheadTemplate dhTo = adjFull[instance.DepotStartIndex][i]?.Deadhead?.DeadheadTemplate ?? throw new InvalidDataException("No arc from depot to trip");
                DeadheadTemplate dhFrom = adjFull[i][instance.DepotEndIndex]?.Deadhead?.DeadheadTemplate ?? throw new InvalidDataException("No arc from trip to depot");

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

        private List<VehicleTask> getSelectedTasks()
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
            return tasks;
        }

        public override Graph GenerateSolutionGraph(bool blockView)
        {
            List<VehicleTask> tasks = getSelectedTasks();

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

        public override bool Solve(CancellationToken ct)
        {
            (GRBModel model, List<GRBVar> taskVars) = InitModel(ct);
            model.Optimize();

            // Tracking generated columns
            int maxColumns = Config.THREADS * Config.MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                singleGenerated = 0,
                globalGenerated = 0,
                lbGenerated = 0,
                seqWithoutRC = 0,           // Number of sequential columns without reduced cost found
                totWithoutRC = 0,           // Total columns generated with no RC
                notFound = 0,               // Number of columns that could not be generated
                discardedNewColumns = 0,    // Number of columns discarded due to better one in model
                discardedOldColumns = 0;    // Number of columns in model discarded due to better one found

            // Multithreaded shortestpath searching
            List<List<VehicleShortestPath>> instances = [
                [.. Enumerable.Range(0, Config.THREADS).Select(_ => new VSPLabeling(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj))], // SP
                [.. Enumerable.Range(0, Config.THREADS).Select(_ => new VSPLSSingle(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj, locationDHTMapping))], // LS_SINGLE
                //[.. Enumerable.Range(0, Config.THREADS).Select(_ => new VSPLSGlobal(model, instance, instance.VehicleTypes[0], nodes, adjFull, adj))], // LS_GLOBAL
            ];
            List<double> operationChances = [Config.VSP_LB_WEIGHT, Config.VSP_LS_SINGLE_WEIGHT, Config.VSP_LS_GLOBAL_WEIGHT];
            List<double> sums = [operationChances[0]];
            for (int i = 1; i < operationChances.Count; i++) sums.Add(sums[i - 1] + operationChances[i]);

            Console.WriteLine("Column generation started");
            Console.WriteLine("%\tT\tLB\tLSS\tLSG\tNF\tDN\tDO\tWRC\tMV");

            Random rnd = new();

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE)
            {
                // Terminate column generation if cancelled
                if (ct.IsCancellationRequested) return false;

                var reducedCosts = model.GetConstrs().Select(x => x.Pi);

                // Generate batch of new tasks using pricing information from previous solve
                List<(double, VehicleTask)>[] generatedTasks = new List<(double, VehicleTask)>[Config.THREADS];


                double r = rnd.NextDouble() * sums[^1];
                int selectedMethodÍndex = sums.FindIndex(x => r <= x);
                List<VehicleShortestPath> selectedMethod = instances[selectedMethodÍndex];

                if (Config.THREADS > 1)
                {
                    Parallel.For(0, Config.THREADS, (i) =>
                    {
                        generatedTasks[i] = selectedMethod[i].GenerateVehicleTasks();
                    });
                }
                else
                {
                    generatedTasks[0] = selectedMethod[0].GenerateVehicleTasks();
                }


                totalGenerated += generatedTasks.Length;
                switch (selectedMethodÍndex)
                {
                    case 0: lbGenerated += generatedTasks.Length; break;
                    case 1: singleGenerated += generatedTasks.Length; break;
                    case 2: globalGenerated += generatedTasks.Length; break;
                    default: throw new InvalidOperationException("You forgot to add a case");
                }

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10)
                {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{singleGenerated}\t{globalGenerated}\t{notFound}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}\t{model.ObjVal}");
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
                        if (Config.CONSOLE_COVER)
                        {
                            Console.WriteLine($"Cover: {String.Join("", Enumerable.Range(0, ba.Count).Select(i => ba[i] ? "1" : "0"))}");
                        }

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

                                // Bookkeeping; replace task in public datastructures
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
                }


                if (seqWithoutRC >= Config.OPT_IT_THRESHOLD)
                {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.OPT_IT_THRESHOLD} consecutive tasks");
                    break;
                }

                // Continue.......
                model.Update();
                model.Optimize();
                currIts++;
            }

            Console.WriteLine($"Value of relaxation: {model.ObjVal}");

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

            Console.WriteLine($"Costs: {model.ObjVal}, vehicle slack: {model.GetVarByName("vehicle_count_slack").X}");

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
