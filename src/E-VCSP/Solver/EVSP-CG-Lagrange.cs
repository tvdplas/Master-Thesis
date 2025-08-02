using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    public class EVSPCGLagrange : Solver {
        private GRBModel? model;
        public VehicleSolutionState vss;
        List<GRBVar> taskVars = [];
        List<double> lambdaTrips = [];

        double upperBoundValue = double.MaxValue;

        public EVSPCGLagrange(VehicleSolutionState vss) {
            this.vss = vss;
        }

        private List<VehicleTask> getSelectedTasks(bool console) {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<VehicleTask> selectedTasks = [];

            // trip index -> [selected vehicle task index]
            List<List<int>> coveredBy = Enumerable.Range(0, vss.Instance.Trips.Count).Select(x => new List<int>()).ToList();
            for (int i = 0; i < taskVars.Count; i++) {
                GRBVar v = taskVars[i];
                if (v.X != 1) continue;

                selectedTasks.Add(vss.Tasks[i]);
                foreach (int j in vss.Tasks[i].TripCover) {
                    coveredBy[j].Add(selectedTasks.Count - 1);
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
        private GRBModel InitModel(CancellationToken ct) {
            // Env
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "evspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound
            model.Parameters.Presolve = 2; // aggresive presolve
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() => {
                Console.WriteLine("Cancellation requested. Terminating Gurobi model...");
                model.Terminate();
            });


            // Add variable for each task/column; add to maxVehicle constraint
            taskVars = [];

            GRBLinExpr maxVehicles = new();
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, vss.Tasks[i].Cost, GRB.BINARY, name);
                maxVehicles += v;

                // Bookkeeping to find variable based on name / cover easily
                taskVars.Add(v);
                vss.VarnameTaskMapping[name] = vss.Tasks[i];
                vss.CoverTaskMapping.Add(vss.Tasks[i].ToBitArray(vss.Instance.Trips.Count), vss.Tasks[i]);
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) lambdaTrips.Add(1);

            //GRBVar vehicleCountSlack = model.AddVar(0, vss.Instance.Trips.Count - Config.MAX_VEHICLES, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_count_slack");
            //model.AddConstr(maxVehicles <= Config.MAX_VEHICLES + vehicleCountSlack, "max_vehicles");

            this.model = model;

            // Determine upper bound for use in gradient descent
            VSPLSGlobal global = new(vss);
            global.updateDualCosts(vss.Instance.Trips.Select(_ => 0.0).ToList(), [], []);
            bool initialPenaltyState = Config.VSP_LS_SHR_ALLOW_PENALTY;
            Config.VSP_LS_SHR_ALLOW_PENALTY = false;
            var initialTasks = global.GenerateVehicleTasks();
            Config.VSP_LS_SHR_ALLOW_PENALTY = initialPenaltyState;

            upperBoundValue = initialTasks.Sum(x => x.vehicleTask.Cost);
            //+ Math.Max(0, initialTasks.Count - Config.MAX_VEHICLES) * Config.VH_OVER_MAX_COST;

            Console.WriteLine($"Upper bound found using global local search with {initialTasks.Count} vehicles, total cost {upperBoundValue}");

            return model;
        }

        public void updateTaskCosts() {
            for (int taskIndex = 0; taskIndex < taskVars.Count; taskIndex++) {
                VehicleTask vt = vss.Tasks[taskIndex];
                double newModelCost = vt.Cost;

                foreach (int tripIndex in vt.TripCover) {
                    newModelCost -= lambdaTrips[tripIndex];
                }

                taskVars[taskIndex].Obj = newModelCost;
            }
        }

        public void gradientDescent() {
            // based on https://people.brunel.ac.uk/~mastjjb/jeb/natcor_ip_rest.pdf
            List<(double C_i, int i)> costsByTaskIndex = vss.Tasks.Select((x, i) => (x.Cost, i)).ToList();
            for (int i = 0; i < 10; i++) ;

            for (int x = 0; x < 10; x++) {
                updateTaskCosts();
                model.Optimize();
                Console.WriteLine("Model value: " + model.ObjVal);

                List<double> G = [];

                for (int i = 0; i < lambdaTrips.Count; i++) {
                    double G_i = 1; // b

                    for (int j = 0; j < taskVars.Count; j++) {
                        if (vss.Tasks[j].TripCover.Contains(i)) {
                            G_i -= taskVars[j].X;
                        }
                    }

                    G.Add(G_i);
                }

                double GSquaredSum = 0;
                foreach (var g in G) GSquaredSum += g * g;
                double T = Config.LAGRANGE_VSP_PI * (upperBoundValue - model.ObjVal) / GSquaredSum;

                for (int i = 0; i < lambdaTrips.Count; i++) {
                    lambdaTrips[i] = Math.Max(0, lambdaTrips[i] + T * G[i]);
                }
            }
        }

        public override bool Solve(CancellationToken ct) {
            model = InitModel(ct);
            gradientDescent();

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
                discardedNewColumns = 0,    // Number of columns discarded due to better one in model
                discardedOldColumns = 0;    // Number of columns in model discarded due to better one found

            // Multithreaded shortestpath searching
            VSPLabeling colgenInstance = new VSPLabeling(vss);
            Console.WriteLine("Column generation started");
            Console.WriteLine("%\tT\tLB\tLSS\tLSG\tAN\tNF\tDN\tDO\tWRC\tMV");

            while (currIts < Config.VSP_MAX_COL_GEN_ITS) {
                // Terminate column generation if cancelled
                if (ct.IsCancellationRequested) return false;

                colgenInstance.updateDualCosts(lambdaTrips, [], []);
                List<(double, VehicleTask)> generatedTasks = colgenInstance.GenerateVehicleTasks();

                totalGenerated += generatedTasks.Count;
                lbGenerated += generatedTasks.Count;

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10) {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{singleGenerated}\t{globalGenerated}\t{addedNew}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}\t{model.ObjVal}");
                }

                foreach (var task in generatedTasks) {
                    (double reducedCost, VehicleTask newTask) = task;

                    // Do not add column to model
                    if (reducedCost > 0) {
                        seqWithoutRC++;
                        totWithoutRC++;
                    }
                    else {
                        // Reset non-reduced costs iterations
                        seqWithoutRC = 0;

                        // Create new column for task, add it to model.
                        int index = vss.Tasks.Count;
                        string name = $"vt_{index}";
                        vss.Tasks.Add(newTask);
                        newTask.Index = index;

                        // Create new column to add to model
                        var modelConstrs = model.GetConstrs();
                        GRBConstr[] constrs = [.. modelConstrs.Where((x) => x.ConstrName == "max_vehicles")];
                        GRBColumn col = new();
                        col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                        // Add column to model
                        taskVars.Add(model.AddVar(0, 1, newTask.Cost, GRB.BINARY, col, name));
                        vss.VarnameTaskMapping[name] = vss.Tasks[^1];
                        vss.CoverTaskMapping[newTask.ToBitArray(vss.Instance.Trips.Count)] = vss.Tasks[^1];
                        addedNew++;
                    }
                }

                // Continue.......
                updateTaskCosts();
                model.Update();
                gradientDescent();
                currIts++;

                if (seqWithoutRC >= Config.VSP_OPT_IT_THRESHOLD) {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.VSP_OPT_IT_THRESHOLD} consecutive tasks");
                    break;
                }
            }

            model.Update();
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true; // Force enable console at end as this solve takes a long time
            model.Optimize();

            Console.WriteLine($"Costs: {model.ObjVal}");

            (var selectedTasks, var blocks) = finalizeResults(true);
            vss.SelectedTasks = selectedTasks;

            if (Config.DUMP_VSP) {
                vss.Dump();
            }
            Config.CONSOLE_GUROBI = configState;
            return true;
        }
    }
}
