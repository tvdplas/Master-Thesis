﻿using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;
using System.Collections;
using System.Text.Json;

namespace E_VCSP.Solver {
    public class EVSPCG : Solver {
        private GRBModel? model;
        public VehicleSolutionState vss;

        public EVSPCG(VehicleSolutionState vss) {
            this.vss = vss;
        }

        private List<VehicleTask> getSelectedTasks(bool console) {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<VehicleTask> selectedTasks = [];

            // trip index -> [selected vehicle task index]
            List<List<int>> coveredBy = Enumerable.Range(0, vss.Instance.Trips.Count).Select(x => new List<int>()).ToList();
            List<VehicleTask> tasks = [];
            foreach (GRBVar v in model.GetVars()) {
                if (!v.VarName.StartsWith("vt_") || v.X != 1) continue;

                VehicleTask vt = vss.VarnameTaskMapping[v.VarName];
                selectedTasks.Add(vt);
                foreach (int i in vt.TripCover) {
                    coveredBy[i].Add(selectedTasks.Count - 1);
                }
            }

            int coveredTotal = 0;
            bool postprocessingRequired = false;
            for (int i = 0; i < coveredBy.Count; i++) {
                int coverCount = coveredBy[i].Count;
                if (coverCount >= 1) coveredTotal++;
                if (coverCount >= 2 && console) {
                    Console.WriteLine($"(!) Trip {vss.Instance.Trips[i]} covered {coverCount} times");
                    postprocessingRequired = true;
                }
            }

            if (console) {
                Console.WriteLine($"Covered {coveredTotal}/{vss.Instance.Trips.Count} trips");
                if (postprocessingRequired)
                    Console.WriteLine("(!) Duplicate trips found; applying postprocessing.");
                if (coveredTotal < vss.Instance.Trips.Count)
                    Console.WriteLine("(!) Not all trips covered");
            }

            if (!postprocessingRequired) return selectedTasks;

            vss.RemoveOvercoverageFromTasks(selectedTasks);
            return selectedTasks;
        }

        private (List<VehicleTask>, List<Block>) finalizeResults(bool console) {
            var selectedTasks = getSelectedTasks(console);

            return (selectedTasks, selectedTasks
                .SelectMany(t => Block.FromVehicleTask(t))
                .Select((b, i) => { b.Index = i; return b; })
                .ToList());
        }

        /// <summary>
        /// Initialize model for column generaton
        /// </summary>
        /// <param name="ct">Cancellation token</param>
        /// <returns>Model where first <c>n</c> constraints correspond to the <c>n</c> trips, and a list of initial vehicle task vars</returns>
        private (GRBModel model, List<GRBVar> taskVars) InitModel(CancellationToken ct) {
            // Env
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "evspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() => {
                Console.WriteLine("Cancellation requested. Terminating Gurobi model...");
                model.Terminate();
            });


            // Add variable for each task/column; add to maxVehicle constraint
            GRBLinExpr maxVehicles = new();
            List<GRBVar> taskVars = [];
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, vss.Tasks[i].Cost, GRB.CONTINUOUS, name);
                maxVehicles += v;

                // Bookkeeping to find variable based on name / cover easily
                taskVars.Add(v);
                vss.VarnameTaskMapping[name] = vss.Tasks[i];
                vss.CoverTaskMapping.Add(vss.Tasks[i].ToBitArray(vss.Instance.Trips.Count), vss.Tasks[i]);
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip 
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }

                // Switch between set partition and cover
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_trip_" + t.Index);
            }

            // Finalize max vehicle constraint with slack
            // Note: added after trips so trips have easier indexing. 
            GRBVar vehicleCountSlack = model.AddVar(0, vss.Instance.Trips.Count - Config.MAX_VEHICLES, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_count_slack");
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES + vehicleCountSlack, "max_vehicles");

            this.model = model;
            return (model, taskVars);
        }

        public override bool Solve(CancellationToken ct) {
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
            List<List<VehicleColumnGen>> instances = [
                [new VSPLabeling(model, vss)], // Labeling
                [.. Enumerable.Range(0, Config.VSP_INSTANCES_PER_IT).Select(_ => new VSPLSSingle(model, vss))], // LS_SINGLE
                [.. Enumerable.Range(0, Config.VSP_INSTANCES_PER_IT).Select(_ => new VSPLSGlobal(model, vss))], // LS_GLOBAL
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
            while (currIts < Config.VSP_MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE) {
                // Terminate column generation if cancelled
                if (ct.IsCancellationRequested) return false;

                var reducedCosts = model.GetConstrs().Select(x => x.Pi);

                // Generate batch of new tasks using pricing information from previous solve
                List<List<(double, VehicleTask)>> generatedTasks = [];

                int selectedMethodIndex = -1;
                if (predefinedOperations.Count > 0) {
                    selectedMethodIndex = predefinedOperations[^1];
                    predefinedOperations.RemoveAt(predefinedOperations.Count - 1);
                }
                else {
                    double r = rnd.NextDouble() * sums[^1];
                    selectedMethodIndex = sums.FindIndex(x => r <= x);
                }

                List<VehicleColumnGen> selectedMethod = instances[selectedMethodIndex];

                if (selectedMethod.Count > 1) {
                    Parallel.For(0, selectedMethod.Count, (i) => {
                        generatedTasks.Add(selectedMethod[i].GenerateVehicleTasks());
                    });
                }
                else {
                    generatedTasks.Add(selectedMethod[0].GenerateVehicleTasks());
                }

                totalGenerated += generatedTasks.Count;
                int colsGenerated = generatedTasks.Sum(t => t.Count);
                switch (selectedMethodIndex) {
                    case 0: lbGenerated += colsGenerated; break;
                    case 1: singleGenerated += colsGenerated; break;
                    case 2: globalGenerated += colsGenerated; break;
                    default: throw new InvalidOperationException("You forgot to add a case");
                }

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10) {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{singleGenerated}\t{globalGenerated}\t{addedNew}\t{notFound}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}\t{model.ObjVal}");
                }

                foreach (var taskSet in generatedTasks) {
                    if (taskSet.Count == 0) {
                        notFound++;
                        continue;
                    }

                    foreach (var task in taskSet) {
                        (double reducedCost, VehicleTask newTask) = ((double, VehicleTask))task;

                        // Check if task is already in model 
                        BitArray ba = newTask.ToBitArray(vss.Instance.Trips.Count);
                        bool coverExists = vss.CoverTaskMapping.ContainsKey(ba);

                        // Cover already exists and is cheaper -> skip
                        if (coverExists && vss.CoverTaskMapping[ba].Cost <= newTask.Cost) {
                            discardedNewColumns++;
                            continue;
                        }

                        // Do not add column to model
                        if (reducedCost > 0) {
                            seqWithoutRC++;
                            totWithoutRC++;
                        }
                        else {
                            // Reset non-reduced costs iterations
                            seqWithoutRC = 0;

                            // Replace existing column with this task, as it has lower costs
                            if (coverExists) {
                                VehicleTask toBeReplaced = vss.CoverTaskMapping[ba];
                                int index = toBeReplaced.Index;
                                newTask.Index = index;

                                // Bookkeeping; replace task in public datastructures
                                vss.Tasks[index] = newTask;
                                vss.VarnameTaskMapping[$"vt_{index}"] = newTask;
                                vss.CoverTaskMapping[ba] = newTask;

                                // Adjust costs in model
                                taskVars[index].Obj = newTask.Cost;
                                discardedOldColumns++;
                            }
                            // Create new column for task, add it to model.
                            else {
                                int index = vss.Tasks.Count;
                                string name = $"vt_{index}";
                                vss.Tasks.Add(newTask);
                                newTask.Index = index;

                                // Create new column to add to model
                                var modelConstrs = model.GetConstrs();
                                GRBConstr[] constrs = [.. modelConstrs.Where(
                                    (_, i) => newTask.TripCover.Contains(i)    // Covers trip
                                    || i == modelConstrs.Length - 1        // Add to used vehicles
                                )];
                                GRBColumn col = new();
                                col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                                // Add column to model
                                taskVars.Add(model.AddVar(0, GRB.INFINITY, newTask.Cost, GRB.CONTINUOUS, col, name));
                                vss.VarnameTaskMapping[name] = vss.Tasks[^1];
                                vss.CoverTaskMapping[ba] = vss.Tasks[^1];
                                addedNew++;
                            }
                        }
                    }
                }


                // Continue.......
                model.Update();
                model.Optimize();
                currIts++;

                if (seqWithoutRC >= Config.VSP_OPT_IT_THRESHOLD) {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.VSP_OPT_IT_THRESHOLD} consecutive tasks");
                    break;
                }
            }

            Console.WriteLine($"Value of relaxation: {model.ObjVal}");

            // Make model binary again
            foreach (GRBVar var in taskVars) {
                if (var.VarName.StartsWith("vt_"))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }

            if (!Config.VSP_ALLOW_SLACK_FINAL_SOLVE) {
                // Remove ability to go over vehicle bounds -> hopefully speeds up solving at end.
                model.GetVarByName("vehicle_count_slack").UB = 0;
            }

            Console.WriteLine($"Total generation attempts: ${totalGenerated}");
            Console.WriteLine($"LS failed to generate charge-feasible trip {notFound} times during generation");
            Console.WriteLine($"Discarded {discardedOldColumns} old, {discardedNewColumns} new columns during generation");
            Console.WriteLine($"{totWithoutRC} columns were not added due to positive reduced costs.");
            Console.WriteLine($"Solving non-relaxed model with total of {vss.Tasks.Count} columns");
            model.Update();
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true; // Force enable console at end as this solve takes a long time
            model.Optimize();

            Console.WriteLine($"Costs: {model.ObjVal}, vehicle slack: {model.GetVarByName("vehicle_count_slack").X}");

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED) {
                Console.WriteLine("Model infeasible / canceled");
                if (Config.VSP_DETERMINE_IIS) {
                    model.ComputeIIS();
                    model.Write("infeasible_CG.ilp");
                }

                Config.CONSOLE_GUROBI = configState;
                return false;
            }

            (var selectedTasks, var blocks) = finalizeResults(true);
            vss.SelectedTasks = selectedTasks;
            if (Config.DUMP_VSP) {
                File.WriteAllText(Config.RUN_LOG_FOLDER + "evspcg-res.json", JsonSerializer.Serialize(new VehicleSolutionStateDump() {
                    selectedTasks = selectedTasks,
                    vehicleType = vss.VehicleType,
                    path = vss.Instance.Path,
                }));
            }

            Config.CONSOLE_GUROBI = configState;
            return true;
        }
    }
}
