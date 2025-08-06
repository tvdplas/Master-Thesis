using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
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

        private (List<VehicleTask>, List<(int count, Block block)>) finalizeResults(bool console) {
            var selectedTasks = getSelectedTasks(console);

            List<Block> selectedBlocks = selectedTasks.SelectMany(t => Block.FromVehicleTask(t))
                .Select((b, i) => { b.Index = i; return b; })
                .ToList();
            Dictionary<string, (int count, Block firstRef)> blockCounts = new();
            foreach (Block block in selectedBlocks) {
                string descriptor = block.Descriptor;
                if (!blockCounts.ContainsKey(descriptor)) blockCounts[descriptor] = (1, block);
                else {
                    (int count, Block firstRef) = blockCounts[descriptor];
                    blockCounts[descriptor] = (count + 1, firstRef);
                }
            }
            var selectedBlocksWithCount = blockCounts.Values.ToList();

            return (selectedTasks, selectedBlocksWithCount);
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

            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }

                // Switch between set partition and cover
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, Constants.CSTR_BLOCK_COVER + t.Index);
            }
            GRBVar vehicleCountSlack = model.AddVar(0, vss.Instance.Trips.Count - Config.MAX_VEHICLES, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_count_slack");
            model.AddConstr(maxVehicles <= Config.MAX_VEHICLES + vehicleCountSlack, Constants.CSTR_MAX_VEHICLES);


            // Determine upper bound for use in gradient descent
            VSPLSGlobal global = new(vss);
            global.UpdateDualCosts(vss.Instance.Trips.Select(_ => 0.0).ToList(), [], []);
            bool initialPenaltyState = Config.VSP_LS_SHR_ALLOW_PENALTY;
            Config.VSP_LS_SHR_ALLOW_PENALTY = false;
            var initialTasks = global.GenerateVehicleTasks();
            Config.VSP_LS_SHR_ALLOW_PENALTY = initialPenaltyState;

            upperBoundValue = initialTasks.Sum(x => x.vehicleTask.Cost);
            //+ Math.Max(0, initialTasks.Count - Config.MAX_VEHICLES) * Config.VH_OVER_MAX_COST;

            Console.WriteLine($"Upper bound found using global local search with {initialTasks.Count} vehicles, total cost {upperBoundValue}");

            model.Update();
            this.model = model;
            return model;
        }

        public void updateCostsInModel() {
            for (int taskIndex = 0; taskIndex < taskVars.Count; taskIndex++) {
                VehicleTask vt = vss.Tasks[taskIndex];
                double newModelCost = vt.Cost;

                foreach (int tripIndex in vt.TripCover) {
                    newModelCost -= lambdaTrips[tripIndex];
                }

                taskVars[taskIndex].Obj = newModelCost;
            }
        }

        public (double objVal, List<bool> X) gradientDescent() {
            // based on https://people.brunel.ac.uk/~mastjjb/jeb/natcor_ip_rest.pdf

            // Reset the current multiplier set in order to get quicker convergence (i hope)
            for (int i = 0; i < lambdaTrips.Count; i++) lambdaTrips[i] = 1;

            // Initialize costs of each task based on current lagrangean multiplier set
            List<(double C_i, int taskIndex)> costsByTaskIndex = vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
            List<bool> X = vss.Tasks.Select(_ => false).ToList();

            void updateTaskCostsWithMultipliers() {
                for (int i = 0; i < costsByTaskIndex.Count; i++) {
                    int taskIndex = costsByTaskIndex[i].taskIndex;
                    double cost = vss.Tasks[taskIndex].Cost;
                    foreach (var tripIndex in vss.Tasks[taskIndex].TripCover) {
                        cost -= lambdaTrips[tripIndex];
                    }
                    costsByTaskIndex[i] = (cost, taskIndex);
                }

            }

            double solve() {
                // Reset current solution value
                for (int i = 0; i < X.Count; i++) X[i] = false;

                List<(double C_i, int taskIndex)> targetTasks = new();
                for (int i = 0; i < costsByTaskIndex.Count; i++) if (costsByTaskIndex[i].C_i < 0) targetTasks.Add(costsByTaskIndex[i]);
                double cost = 0;
                for (int i = 0; i < lambdaTrips.Count; i++) cost += lambdaTrips[i];

                var cappedTargetTasks = targetTasks.OrderBy(x => x.C_i).Take(Config.MAX_VEHICLES);
                foreach (var targetTask in cappedTargetTasks) {
                    cost += targetTask.C_i;
                    X[targetTask.taskIndex] = true;
                }

                return cost;
            }

            double lastSolutionVal = upperBoundValue;
            int roundsInThreshold = 0;

            for (int round = 0; round < Config.LANGRANGE_MAX_ROUNDS; round++) {
                updateTaskCostsWithMultipliers();
                double z_curr = solve();

                double[] G = new double[lambdaTrips.Count];
                for (int i = 0; i < G.Length; i++) G[i] = 1; // b
                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (!X[taskIndex]) continue;
                    foreach (var tripIndex in vss.Tasks[taskIndex].TripCover) G[tripIndex] -= 1;
                }

                double GSquaredSum = 0;
                foreach (var g in G) GSquaredSum += g * g;
                double T = Config.LAGRANGE_VSP_PI * (upperBoundValue - z_curr) / GSquaredSum;

                for (int i = 0; i < lambdaTrips.Count; i++) {
                    lambdaTrips[i] = Math.Max(0, lambdaTrips[i] + T * G[i]);
                }

                // Solution might have stabelized
                double percentageDiff = Math.Abs(Math.Abs(z_curr - lastSolutionVal) / lastSolutionVal);
                bool diffInThreshold = percentageDiff < Config.LANGRANGE_THRS;
                lastSolutionVal = z_curr;

                if (diffInThreshold) {
                    roundsInThreshold++;
                }
                else {
                    roundsInThreshold = 0;
                }

                if (roundsInThreshold >= Config.LANGRANGE_THRS_SEQ) break;
            }

            return (lastSolutionVal, X);
        }

        public override bool Solve(CancellationToken ct) {
            model = InitModel(ct);
            (double objVal, _) = gradientDescent();

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

                colgenInstance.UpdateDualCosts(lambdaTrips, [], []);
                List<(double, VehicleTask)> generatedTasks = colgenInstance.GenerateVehicleTasks();

                totalGenerated += generatedTasks.Count;
                lbGenerated += generatedTasks.Count;

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
                        GRBConstr[] constrs = [.. modelConstrs.Where((constr) => {
                            if (constr.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER)) {
                                int tripIndex = int.Parse(constr.ConstrName.Split(Constants.CSTR_BLOCK_COVER)[1]);
                                return newTask.TripCover.Contains(tripIndex);
                            }

                            else
                                return constr.ConstrName == Constants.CSTR_MAX_VEHICLES;
                        })];
                        GRBColumn col = new();
                        col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                        // Add column to model
                        taskVars.Add(model.AddVar(0, 1, newTask.Cost, GRB.BINARY, col, name));
                        vss.VarnameTaskMapping[name] = vss.Tasks[^1];
                        vss.CoverTaskMapping[newTask.ToBitArray(vss.Instance.Trips.Count)] = vss.Tasks[^1];
                        addedNew++;
                    }
                }

                (double newObjVal, _) = gradientDescent();
                objVal = newObjVal;
                currIts++;

                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10) {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{singleGenerated}\t{globalGenerated}\t{addedNew}\t{discardedNewColumns}\t{discardedOldColumns}\t{totWithoutRC}\t{objVal}");
                }

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

            (var selectedTasks, _) = finalizeResults(true);
            vss.SelectedTasks = selectedTasks;

            if (Config.DUMP_VSP) {
                vss.Dump();
            }
            Config.CONSOLE_GUROBI = configState;
            return true;
        }
    }
}
