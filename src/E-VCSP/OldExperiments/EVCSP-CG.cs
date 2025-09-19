using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;
using Gurobi;

namespace E_VCSP.OldExperiments {
    public class BlockWrapper {
        public required Block Block;
        public double CoveredCount;
    }

    internal class EVCSPCG : Solver.Solver {

        #region DEBUG
        private void DEBUG_printDualCostsTripCovers(GRBModel model) {
            List<string> dualCosts = [];
            foreach (var constr in model.GetConstrs()) {
                if (constr.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER)) dualCosts.Add(constr.Pi.ToString("0.##"));
            }
            Console.WriteLine(string.Join(' ', dualCosts));
        }

        private void DEBUG_printDualCostsBlockCovers(GRBModel model) {
            List<string> dualCosts = [];
            foreach (var constr in model.GetConstrs()) {
                if (constr.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER)) dualCosts.Add(constr.Pi.ToString("0.##"));
            }
            Console.WriteLine(string.Join(' ', dualCosts));
        }

        private void DEBUG_printBlockCoverRequirements(bool filterZero = true) {
            for (int i = 0; i < knownBlocks.Count; i++) {
                if (filterZero && knownBlocks[i].CoveredCount == 0) continue;
                Console.WriteLine($"{knownBlocks[i].Block.Descriptor}: {knownBlocks[i].CoveredCount}");
            }
        }
        #endregion
        GRBModel? model;
        public readonly VehicleSolutionState vss;
        public readonly CrewSolutionState css;

        private Dictionary<string, int> descriptorToKnownBlockIndex = [];
        private List<BlockWrapper> knownBlocks = [];
        private List<GRBVar> taskVars = [];
        private List<GRBVar> dutyVars = [];
        private Dictionary<string, GRBConstr> vehicleConstrs = [];
        private Dictionary<string, GRBConstr> crewConstrs = [];

        public EVCSPCG(VehicleSolutionState vss, CrewSolutionState css) {
            this.vss = vss;
            this.css = css;

            // Dump blocks from vss initial solution, add to crew solution
            for (int i = 0; i < vss.Tasks.Count; i++) {
                processVTBlockCover(vss.Tasks[i], false);
            }
        }

        #region results

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
                foreach (int i in vt.TripIndexCover) {
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

            if (!postprocessingRequired || true) return selectedTasks;

            vss.RemoveOvercoverageFromTasks(selectedTasks);
            return selectedTasks;
        }
        private List<(CrewDuty duty, int count)> getSelectedDuties() {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<(CrewDuty duty, int count)> duties = [];
            int[] covered = new int[css.Blocks.Count];
            foreach (GRBVar v in model.GetVars()) {
                if (v.VarName.StartsWith("cd_") && v.X > 0) {
                    int count = (int)v.X;
                    CrewDuty dvt = css.VarnameDutyMapping[v.VarName];
                    duties.Add((dvt, count));
                    foreach (int i in dvt.BlockIndexCover) covered[i] += count;
                }
            }

            int coveredTotal = 0;
            for (int i = 0; i < covered.Length; i++) {
                int val = covered[i];
                if (val >= 1) coveredTotal++;
                if (val >= 2) Console.WriteLine($"(!) Block {css.Blocks[i]} covered {val} times");
            }

            Console.WriteLine($"Covered {coveredTotal}/{css.ActiveBlockCount} blocks using {duties.Count} duties");
            if (coveredTotal < css.ActiveBlockCount)
                Console.WriteLine("(!) Not all blocks covered");

            return duties;
        }

        private (List<VehicleTask>, List<(Block block, int count)>, List<(CrewDuty duty, int count)>) finalizeResults(bool console) {
            var selectedTasks = getSelectedTasks(console);
            var selectedBlocksWithCount = Block.FromVehicleTasks(selectedTasks);
            var selectedDuties = getSelectedDuties();

            return (selectedTasks, selectedBlocksWithCount, selectedDuties);
        }

        #endregion

        private void processVTBlockCover(VehicleTask vt, bool addNewConstrs) {
            List<Block> blocks = Block.FromVehicleTask(vt);
            List<int> BlockCover = [];

            for (int j = 0; j < blocks.Count; j++) {
                Block block = blocks[j];
                string descriptor = block.Descriptor;
                int knownBlockIndex = -1;

                if (descriptorToKnownBlockIndex.ContainsKey(descriptor)) {
                    knownBlockIndex = descriptorToKnownBlockIndex[descriptor];
                }
                else {
                    knownBlockIndex = knownBlocks.Count;
                    descriptorToKnownBlockIndex[descriptor] = knownBlockIndex;
                    knownBlocks.Add(new BlockWrapper() { Block = block, CoveredCount = 1 });

                    // New base block needs to be added to css
                    CrewDuty unitDuty = css.AddBlock(block, 1);

                    if (addNewConstrs) {
                        if (model == null) throw new InvalidOperationException("Cannot add block constraints to model if no model exists");

                        // Model needs new constraint; adding of task to constraint is handled seperately
                        string name = $"cd_{unitDuty.Index}";
                        GRBVar unitDutyVar = model.AddVar(0, GRB.INFINITY, unitDuty.Cost, GRB.CONTINUOUS, name);
                        dutyVars.Add(unitDutyVar);
                        css.VarnameDutyMapping[name] = unitDuty;
                        css.CoverDutyMapping.Add(unitDuty.ToBlockBitArray(knownBlocks.Count), unitDuty);

                        GRBLinExpr expr = new();
                        expr -= unitDutyVar;
                        string blockCoverName = Constants.CSTR_BLOCK_COVER + descriptor;
                        crewConstrs[blockCoverName] = model.AddConstr(expr <= 0, blockCoverName);
                    }
                }

                BlockCover.Add(knownBlockIndex);
            }

            vt.BlockIndexCover = BlockCover;
            if (addNewConstrs) model!.Update();
        }

        private GRBModel initModel(CancellationToken ct) {
            // Env
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Constants.RUN_LOG_FOLDER, "evcspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound
            model.Parameters.Presolve = 2; // aggresive presolve
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() => {
                Console.WriteLine("Cancellation requested.");
                model.Terminate();
            });

            // Create column selection vars
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, vss.Tasks[i].Cost, GRB.CONTINUOUS, name);

                taskVars.Add(v);
                vss.VarnameTaskMapping[name] = vss.Tasks[i];
                vss.CoverTaskMapping.Add(vss.Tasks[i].ToTripBitArray(vss.Instance.Trips.Count), vss.Tasks[i]);
            }
            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost, GRB.CONTINUOUS, name);

                dutyVars.Add(v);
                css.VarnameDutyMapping[name] = css.Duties[i];
                css.CoverDutyMapping.Add(css.Duties[i].ToBlockBitArray(knownBlocks.Count), css.Duties[i]);
            }

            // Max selected vehicle tasks
            GRBLinExpr maxVehiclesExpr = new();
            GRBVar maxVehiclesSlack = model.AddVar(0, GRB.INFINITY, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_slack");
            foreach (GRBVar v in taskVars) {
                maxVehiclesExpr += v;
            }
            model.AddConstr(maxVehiclesExpr - maxVehiclesSlack <= Config.MAX_VEHICLES, Constants.CSTR_MAX_VEHICLES);

            // Trip cover by vehicle tasks
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripIndexCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                string name = Constants.CSTR_TRIP_COVER + t.Index;
                GRBConstr constr = model.AddConstr(expr, sense, 1, name);
                vehicleConstrs[name] = constr;
            }

            // Block cover of selected tasks by duties
            for (int blockIndex = 0; blockIndex < knownBlocks.Count; blockIndex++) {
                GRBLinExpr expr = new();

                for (int vtIndex = 0; vtIndex < taskVars.Count; vtIndex++) {
                    if (vss.Tasks[vtIndex].BlockIndexCover.Contains(blockIndex))
                        expr += taskVars[vtIndex];
                }

                for (int cdIndex = 0; cdIndex < dutyVars.Count; cdIndex++) {
                    if (css.Duties[cdIndex].BlockIndexCover.Contains(blockIndex))
                        expr -= dutyVars[cdIndex];
                }

                string name = Constants.CSTR_BLOCK_COVER + knownBlocks[blockIndex].Block.Descriptor;
                crewConstrs[name] = model.AddConstr(expr <= 0, name);
            }

            // Duty type / avg duration
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between

            GRBVar noExcessiveLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++) {
                GRBVar v = dutyVars[i];
                CrewDuty duty = css.Duties[i];

                noExcessiveLength += Constants.CR_MAX_OVER_LONG_DUTY * v - duty.IsLongDuty * v;
                limitedAverageLength += v * (duty.PaidDuration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty * v;
                maxBetween += v * Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty * v;
            }

            crewConstrs[Constants.CSTR_CR_LONG_DUTIES] =
                model.AddConstr(noExcessiveLength + noExcessiveLengthSlack >= 0, Constants.CSTR_CR_LONG_DUTIES);
            crewConstrs[Constants.CSTR_CR_AVG_TIME] =
                model.AddConstr(limitedAverageLength - limitedAverageLengthSlack <= 0, Constants.CSTR_CR_AVG_TIME);
            crewConstrs[Constants.CSTR_CR_BROKEN_DUTIES] =
                model.AddConstr(maxBroken + maxBrokenSlack >= 0, Constants.CSTR_CR_BROKEN_DUTIES);
            crewConstrs[Constants.CSTR_CR_BETWEEN_DUTIES] =
                model.AddConstr(maxBetween + maxBetweenSlack >= 0, Constants.CSTR_CR_BETWEEN_DUTIES);

            this.model = model;
            return model;
        }

        // Update what vehicle tasks are currently selected for blocks
        private void updateBlockCover() {
            // Reset all block counts to 0
            for (int i = 0; i < knownBlocks.Count; i++) knownBlocks[i].CoveredCount = 0;
            // Add new block counts
            for (int i = 0; i < taskVars.Count; i++) {
                if (taskVars[i].X > 0) {
                    // TODO: dit werkt zo niet meer
                    foreach (int knownBlockIndex in vss.Tasks[i].BlockIndexCover)
                        knownBlocks[knownBlockIndex].CoveredCount += taskVars[i].X;
                }
            }
            // Edit the css to reflect current used count state
            for (int i = 0; i < knownBlocks.Count; i++) css.BlockCount[i] = (int)Math.Ceiling(knownBlocks[i].CoveredCount);
        }

        private void runVehicleIts(bool initial) {
            if (model == null) throw new InvalidOperationException();
            int maxIts = initial ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;
            VSPLabeling vspLabelingInstance = new(vss);

            void updateDualCosts() {
                List<double> tripDualCosts = [];
                Dictionary<string, double> blockDualCosts = new();
                Dictionary<string, List<double>> blockDualCostsByStart = new();

                foreach (var constr in vehicleConstrs.Values) {
                    string name = constr.ConstrName;
                    if (constr.ConstrName.StartsWith("cover_trip")) {
                        tripDualCosts.Add(constr.Pi);
                    }

                    if (constr.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER)) {
                        string descriptor = name.Split("_")[^1];
                        string descriptorStart = Descriptor.GetStart(descriptor);
                        blockDualCosts[descriptor] = constr.Pi;
                        if (blockDualCostsByStart.ContainsKey(descriptorStart)) blockDualCostsByStart[descriptorStart].Add(constr.Pi);
                        else blockDualCostsByStart[descriptorStart] = [constr.Pi];
                    }
                }

                vspLabelingInstance.UpdateDualCosts(tripDualCosts, blockDualCosts, blockDualCostsByStart);
            }

            void processNewTask(double reducedCosts, VehicleTask newTask) {
                // Determine the block coverage of the new column
                // If new blocks are found within the task, constraints
                // will be added to the model.
                processVTBlockCover(newTask, true);

                // Add it to the model
                int index = vss.Tasks.Count;
                string name = $"vt_{index}";
                vss.Tasks.Add(newTask);
                newTask.Index = index;

                var modelConstrs = model!.GetConstrs();
                GRBConstr[] constrs = [.. modelConstrs.Where((constr) => {
                    string name = constr.ConstrName;
                    // Is a vehicle
                    if (name == Constants.CSTR_MAX_VEHICLES)
                        return true;
                    // Vehicle task contains trip
                    if (name.StartsWith(Constants.CSTR_BLOCK_COVER)) {
                        int tripIndex = int.Parse(name.Split("_")[^1]);
                        return newTask.TripIndexCover.Contains(tripIndex);
                    }
                    // Vehicle task contains block
                    if (name.StartsWith(Constants.CSTR_BLOCK_COVER)) {
                        string blockDescriptor = name.Split("_")[^1];
                        return newTask.BlockIndexCover.Contains(descriptorToKnownBlockIndex[blockDescriptor]);
                    }
                    // Constraint not relevant
                    return false;
                })];
                GRBColumn col = new();
                col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                // Add column to model
                taskVars.Add(model.AddVar(0, GRB.INFINITY, newTask.Cost, GRB.CONTINUOUS, col, name));
                vss.VarnameTaskMapping[name] = newTask;
                vss.CoverTaskMapping[newTask.ToTripBitArray(vss.Instance.Trips.Count)] = newTask;
            }

            for (int currIt = 0; currIt < maxIts; currIt++) {
                updateDualCosts();
                var newColumns = vspLabelingInstance.GenerateVehicleTasks();

                foreach ((var reducedCosts, var newTask) in newColumns) {
                    processNewTask(reducedCosts, newTask);
                }
                model.Optimize();
            }
        }

        private void runCrewIts(bool initial) {
            if (model == null) throw new InvalidOperationException();

            int maxIts = initial ? Config.VCSP_CR_ITS_INIT : Config.VCSP_CR_ITS_ROUND;

            void processNewDuty(double reducedCosts, CrewDuty newDuty) {
                // Add it to the model
                int index = css.Duties.Count;
                string name = $"cd_{index}";
                css.Duties.Add(newDuty);
                newDuty.Index = index;

                // overall schedule contributions
                double excessiveLength = Constants.CR_MAX_OVER_LONG_DUTY - newDuty.IsLongDuty;
                double limitedAverageLength = newDuty.PaidDuration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1;
                double maxBroken = Constants.CR_MAX_BROKEN_SHIFTS - newDuty.IsBrokenDuty;
                double maxBetween = Constants.CR_MAX_BETWEEN_SHIFTS - newDuty.IsBetweenDuty;

                var modelConstrs = model.GetConstrs();
                GRBConstr[] constrs = [.. modelConstrs.Where((constr) => {
                    string name = constr.ConstrName;
                    if (name.StartsWith(Constants.CSTR_BLOCK_COVER)) {
                        string blockDescriptor = name.Split("_")[^1];
                        return newDuty.BlockIndexCover.Contains(descriptorToKnownBlockIndex[blockDescriptor]);
                    }

                    // Overall crew schedule constraints
                    if (name.StartsWith("cr_overall_")) return true;
                    return false;
                })];
                GRBColumn col = new();
                col.AddTerms([..constrs.Select(c => {
                    if (c.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER)) return -1.0;
                    else if (c.ConstrName == Constants.CSTR_CR_LONG_DUTIES) return Constants.CR_MAX_OVER_LONG_DUTY - newDuty.IsLongDuty;
                    else if (c.ConstrName == Constants.CSTR_CR_AVG_TIME) return newDuty.PaidDuration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1;
                    else if (c.ConstrName == Constants.CSTR_CR_BROKEN_DUTIES) return Constants.CR_MAX_BROKEN_SHIFTS - newDuty.IsBrokenDuty;
                    else if (c.ConstrName == Constants.CSTR_CR_BETWEEN_DUTIES) return Constants.CR_MAX_BETWEEN_SHIFTS - newDuty.IsBetweenDuty;
                    else throw new InvalidOperationException($"Constraint {c.ConstrName} not handled when adding new column");
                })], constrs);

                // Add column to model
                dutyVars.Add(model.AddVar(0, GRB.INFINITY, newDuty.Cost, GRB.CONTINUOUS, col, name));
                css.VarnameDutyMapping[name] = newDuty;
                css.CoverDutyMapping[newDuty.ToBlockBitArray(knownBlocks.Count)] = newDuty;
            }

            // Attempt to do column generation for crew

            CSPLabeling[] cspLabelingInstances = [.. Enumerable.Range(0, Config.VCSP_CR_INSTANCES).Select(_ => new CSPLabeling(css))];
            for (int it = 0; it < maxIts; it++) {
                List<(double reducedCosts, CrewDuty newDuty)> newColumns = new();
                Parallel.For(0, cspLabelingInstances.Length, (i) => {
                    var crewDualCosts = crewConstrs.Select((kv) => (kv.Key, kv.Value.Pi)).ToDictionary();
                    cspLabelingInstances[i].UpdateDualCosts(crewDualCosts, -1);
                    var res = cspLabelingInstances[i].GenerateDuties();
                    foreach (var resCol in res) newColumns.Add(resCol);
                });

                foreach ((var reducedCosts, var newDuty) in newColumns) {
                    processNewDuty(reducedCosts, newDuty);
                }
                model.Optimize();
            }
        }

        private (double vtCount, double gap) swapVehicleObjective(bool makeBinary, bool optimize = true) {
            if (model == null) throw new Exception("Cant switch vehicle objective if no model is found");

            foreach (GRBVar var in taskVars) {
                var.Set(GRB.CharAttr.VType, makeBinary ? GRB.BINARY : GRB.CONTINUOUS);
            }

            model.GetEnv().TimeLimit = makeBinary ? 60 : Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Update();
            if (optimize) model.Optimize();

            return (makeBinary && optimize ? taskVars.Sum(x => x.X) : -1, makeBinary && optimize ? model.MIPGap : -1);
        }

        private void swapCrewObjective(bool makeBinary, bool optimize = true) {
            if (model == null) throw new Exception("Cant switch vehicle objective if no model is found");

            foreach (GRBVar var in dutyVars) {
                var.Set(GRB.CharAttr.VType, makeBinary ? GRB.BINARY : GRB.CONTINUOUS);
            }

            Config.CONSOLE_GUROBI = makeBinary;
            model.GetEnv().TimeLimit = makeBinary ? 60 : Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Update();
            if (optimize) model.Optimize();
        }

        /// <summary>
        /// Show current model state; should be called on relaxed model
        /// </summary>
        private void PrintModelState(double ilpVH, double ilpVHGap, int round) {
            if (model == null) throw new InvalidOperationException("Cannot dump model state without model instance");
            string r = round == -1 ? "I" : round.ToString();
            string objVal = model.ObjVal.ToString("0.##");
            string lpVH = taskVars.Sum(x => x.X).ToString("0.##");
            string lpCD = dutyVars.Sum(x => x.X).ToString("0.##");
            string ilpVHGapStr = ilpVHGap.ToString("0.##");
            string numBlocks = css.ActiveBlockCount.ToString();

            Dictionary<string, string> entries = new Dictionary<string, string> {
                { "R", r },
                { "MV\t", objVal },
                { "ILP-GAP", ilpVHGapStr },
                { "#VT-ILP", ilpVH.ToString() },
                { "#VT-LP", lpVH },
                { "#BL-ILP", numBlocks },
                { "#CD-LP", lpCD },
            };

            if (round == -1) {
                Console.WriteLine(string.Join('\t', entries.Keys));
            }

            Console.WriteLine(string.Join('\t', entries.Values));
        }

        public override bool Solve(CancellationToken ct) {
            model = initModel(ct);
            model.Optimize();

            Console.WriteLine($"Initializing solution.");

            // Properly initialize model
            runVehicleIts(true);
            (double initVHCount, double initGap) = swapVehicleObjective(true);
            updateBlockCover();
            swapVehicleObjective(false);
            runCrewIts(true);
            PrintModelState(initVHCount, initGap, -1);

            // Continue running using initial good covering
            for (int round = 0; round < Config.VCSP_ROUNDS; round++) {
                runVehicleIts(false);
                (double vhCount, double gap) = swapVehicleObjective(true);
                updateBlockCover();
                swapVehicleObjective(false);
                runCrewIts(false);
                PrintModelState(vhCount, gap, round);
            }

            // Make binary in two phases: first duty, then vehicle ofzo
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true;

            swapVehicleObjective(true, false);
            swapCrewObjective(true, false);
            model.Update();
            model.Optimize();
            Config.CONSOLE_GUROBI = configState;

            (var selectedTasks, var selectedBlocks, var selectedDuties) = finalizeResults(true);
            vss.SelectedTasks = selectedTasks;
            css.SelectedDuties = selectedDuties;

            Console.WriteLine($"Final solution has {selectedTasks.Count} tasks, {selectedBlocks.Count} blocks, {selectedDuties.Count} duties.");
            return true;
        }
    }
}
