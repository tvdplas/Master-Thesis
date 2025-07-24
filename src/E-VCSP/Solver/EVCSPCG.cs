using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    public class BlockWrapper {
        public required Block Block;
        public double CoveredCount;
    }

    internal class EVCSPCG : Solver {

        #region DEBUG
        private void DEBUG_printDualCostsTripCovers(GRBModel model) {
            List<string> dualCosts = [];
            foreach (var constr in model.GetConstrs()) {
                if (constr.ConstrName.StartsWith("cover_trip_")) dualCosts.Add(constr.Pi.ToString("0.##"));
            }
            Console.WriteLine(string.Join(' ', dualCosts));
        }

        private void DEBUG_printDualCostsBlockCovers(GRBModel model) {
            List<string> dualCosts = [];
            foreach (var constr in model.GetConstrs()) {
                if (constr.ConstrName.StartsWith("cover_block_")) dualCosts.Add(constr.Pi.ToString("0.##"));
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

        public EVCSPCG(VehicleSolutionState vss, CrewSolutionState css) {
            this.vss = vss;
            this.css = css;

            // Dump blocks from vss initial solution, add to crew solution
            for (int i = 0; i < vss.Tasks.Count; i++) {
                processVTBlockCover(vss.Tasks[i]);
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
        private List<CrewDuty> getSelectedDuties() {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<CrewDuty> duties = [];
            int[] covered = new int[css.Blocks.Count];
            foreach (GRBVar v in model.GetVars()) {
                if (v.VarName.StartsWith("cd_") && v.X > 0) {
                    CrewDuty dvt = css.VarnameDutyMapping[v.VarName];
                    duties.Add(dvt);
                    foreach (int i in dvt.BlockCover) covered[i]++;
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

        private (List<VehicleTask>, List<Block>, List<CrewDuty>) finalizeResults(bool console) {
            var selectedTasks = getSelectedTasks(console);
            var selectedBlocks = selectedTasks.SelectMany(t => Block.FromVehicleTask(t))
                .Select((b, i) => { b.Index = i; return b; })
                .ToList();
            var selectedDuties = getSelectedDuties();

            return (selectedTasks, selectedBlocks, selectedDuties);
        }

        #endregion

        private void processVTBlockCover(VehicleTask vt, bool addNewConstrs = false) {
            List<Block> blocks = Block.FromVehicleTask(vt);
            List<int> BlockCover = [];

            for (int j = 0; j < blocks.Count; j++) {
                Block block = blocks[j];
                string descriptor = block.Descriptor;
                int knownBlockIndex = -1;

                if (descriptorToKnownBlockIndex.ContainsKey(descriptor)) {
                    knownBlockIndex = descriptorToKnownBlockIndex[descriptor];
                    knownBlocks[knownBlockIndex].CoveredCount++;

                    // Add the vehicle task to the 
                }
                else {
                    knownBlockIndex = knownBlocks.Count;
                    descriptorToKnownBlockIndex[descriptor] = knownBlockIndex;
                    knownBlocks.Add(new BlockWrapper() { Block = block, CoveredCount = 1 });

                    // New base block needs to be added to css
                    CrewDuty unitDuty = css.AddBlock(block);

                    if (addNewConstrs) {
                        if (model == null) throw new InvalidOperationException("Cannot add block constraints to model if no model exists");

                        // Model needs new constraint; adding of task to constraint is handled seperately
                        string name = $"cd_{unitDuty.Index}";
                        GRBVar unitDutyVar = model.AddVar(0, GRB.INFINITY, unitDuty.Cost, GRB.CONTINUOUS, name);
                        dutyVars.Add(unitDutyVar);
                        css.VarnameDutyMapping[name] = unitDuty;
                        css.CoverDutyMapping.Add(unitDuty.ToBitArray(knownBlocks.Count), unitDuty);

                        GRBLinExpr expr = new();
                        expr -= unitDutyVar;
                        model!.AddConstr(expr <= 0, $"cover_block_{descriptor}");
                    }
                }

                BlockCover.Add(knownBlockIndex);
            }

            vt.BlockCover = BlockCover;
            if (addNewConstrs) model!.Update();
        }

        private GRBModel initModel(CancellationToken ct) {
            // Env
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "evcspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound
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
                vss.CoverTaskMapping.Add(vss.Tasks[i].ToBitArray(vss.Instance.Trips.Count), vss.Tasks[i]);
            }
            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost, GRB.CONTINUOUS, name);

                dutyVars.Add(v);
                css.VarnameDutyMapping[name] = css.Duties[i];
                css.CoverDutyMapping.Add(css.Duties[i].ToBitArray(knownBlocks.Count), css.Duties[i]);
            }

            // Max selected vehicle tasks
            GRBLinExpr maxVehiclesExpr = new();
            GRBVar maxVehiclesSlack = model.AddVar(0, GRB.INFINITY, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_slack");
            foreach (GRBVar v in taskVars) {
                maxVehiclesExpr += v;
            }
            model.AddConstr(maxVehiclesExpr - maxVehiclesSlack <= Config.MAX_VEHICLES, "max_vehicles");

            // Trip cover by vehicle tasks
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_trip_" + t.Index);
            }

            // Block cover of selected tasks by duties
            for (int blockIndex = 0; blockIndex < knownBlocks.Count; blockIndex++) {
                GRBLinExpr expr = new();

                for (int vtIndex = 0; vtIndex < taskVars.Count; vtIndex++) {
                    if (vss.Tasks[vtIndex].BlockCover.Contains(blockIndex))
                        expr += taskVars[vtIndex];
                }

                for (int cdIndex = 0; cdIndex < dutyVars.Count; cdIndex++) {
                    if (css.Duties[cdIndex].BlockCover.Contains(blockIndex))
                        expr -= dutyVars[cdIndex];
                }

                model.AddConstr(expr <= 0, $"cover_block_{knownBlocks[blockIndex].Block.Descriptor}");
            }

            // Duty type / avg duration
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between
            GRBLinExpr maxSingle = new(); // Use singles at a cost

            GRBVar noExcessiveLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++) {
                GRBVar v = dutyVars[i];
                CrewDuty duty = css.Duties[i];

                int duration = duty.Elements[^1].EndTime - duty.Elements[0].StartTime;
                noExcessiveLength += Config.CR_MAX_OVER_LONG_SHIFT * v - (duration > Config.CR_LONG_SHIFT_LENGTH ? v : 0);
                limitedAverageLength += v * (duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Config.CR_MAX_BROKEN_SHIFTS - (duty.Type == DutyType.Broken ? v : 0);
                maxBetween += v * Config.CR_MAX_BETWEEN_SHIFTS - (duty.Type == DutyType.Between ? v : 0);
            }

            model.AddConstr(noExcessiveLength + noExcessiveLengthSlack >= 0, "cr_overall_no_excessive_length");
            model.AddConstr(limitedAverageLength - limitedAverageLengthSlack <= 0, "cr_overall_limited_average_length");
            model.AddConstr(maxBroken + maxBrokenSlack >= 0, "cr_overall_max_broken");
            model.AddConstr(maxBetween + maxBetweenSlack >= 0, "cr_overall_max_between");

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
                    foreach (int knownBlockIndex in vss.Tasks[i].BlockCover)
                        knownBlocks[knownBlockIndex].CoveredCount += taskVars[i].X;
                }
            }
            // Edit the css to reflect current used count state
            for (int i = 0; i < knownBlocks.Count; i++) css.BlockActive[i] = knownBlocks[i].CoveredCount > 0;
        }

        private void runVehicleIts(bool initial) {
            if (model == null) throw new InvalidOperationException();

            int maxIts = initial ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;

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
                    if (name == "max_vehicles")
                        return true;
                    // Vehicle task contains trip
                    if (name.StartsWith("cover_trip_")) {
                        int tripIndex = int.Parse(name.Split("_")[^1]);
                        return newTask.TripCover.Contains(tripIndex);
                    }
                    // Vehicle task contains block
                    if (name.StartsWith("cover_block_")) {
                        string blockDescriptor = name.Split("_")[^1];
                        return newTask.BlockCover.Contains(descriptorToKnownBlockIndex[blockDescriptor]);
                    }
                    // Constraint not relevant
                    return false;
                })];
                GRBColumn col = new();
                col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                // Add column to model
                taskVars.Add(model.AddVar(0, GRB.INFINITY, newTask.Cost, GRB.CONTINUOUS, col, name));
                vss.VarnameTaskMapping[name] = newTask;
                vss.CoverTaskMapping[newTask.ToBitArray(vss.Instance.Trips.Count)] = newTask;
            }

            VSPLabeling vspLabelingInstance = new(model, vss);
            for (int currIt = 0; currIt < maxIts; currIt++) {
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
                double excessiveLength = Config.CR_MAX_OVER_LONG_SHIFT - (newDuty.Duration > Config.CR_LONG_SHIFT_LENGTH ? 1 : 0);
                double limitedAverageLength = newDuty.Duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1;
                double maxBroken = Config.CR_MAX_BROKEN_SHIFTS - (newDuty.Type == DutyType.Broken ? 1 : 0);
                double maxBetween = Config.CR_MAX_BETWEEN_SHIFTS - (newDuty.Type == DutyType.Between ? 1 : 0);

                var modelConstrs = model.GetConstrs();
                GRBConstr[] constrs = [.. modelConstrs.Where((constr) => {
                    string name = constr.ConstrName;
                    if (name.StartsWith("cover_block_")) {
                        string blockDescriptor = name.Split("_")[^1];
                        return newDuty.BlockCover.Contains(descriptorToKnownBlockIndex[blockDescriptor]);
                    }

                    // Overall crew schedule constraints
                    if (name.StartsWith("cr_overall_")) return true;
                    return false;
                })];
                GRBColumn col = new();
                col.AddTerms([..constrs.Select(c => {
                    if (c.ConstrName.StartsWith("cover_block_")) return -1.0;
                    else if (c.ConstrName == "cr_overall_no_excessive_length") return newDuty.Duration > Config.CR_LONG_SHIFT_LENGTH ? Config.CR_MAX_OVER_LONG_SHIFT - 1 : Config.CR_MAX_OVER_LONG_SHIFT;
                    else if (c.ConstrName == "cr_overall_limited_average_length") return (newDuty.Duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                    else if (c.ConstrName == "cr_overall_max_broken") return newDuty.Type == DutyType.Broken ? Config.CR_MAX_BROKEN_SHIFTS - 1 : Config.CR_MAX_BROKEN_SHIFTS;
                    else if (c.ConstrName == "cr_overall_max_between") return newDuty.Type == DutyType.Between ? Config.CR_MAX_BETWEEN_SHIFTS - 1 : Config.CR_MAX_BETWEEN_SHIFTS;
                    else throw new InvalidOperationException($"Constraint {c.ConstrName} not handled when adding new column");
                })], constrs);

                // Add column to model
                dutyVars.Add(model.AddVar(0, GRB.INFINITY, newDuty.Cost, GRB.CONTINUOUS, col, name));
                css.VarnameDutyMapping[name] = newDuty;
                css.CoverDutyMapping[newDuty.ToBitArray(knownBlocks.Count)] = newDuty;
            }

            // Attempt to do column generation for crew
            CSPLabeling cspLabelingInstance = new(model, css);
            for (int it = 0; it < maxIts; it++) {
                var newColumns = cspLabelingInstance.GenerateDuties();

                foreach ((var reducedCosts, var newDuty) in newColumns) {
                    processNewDuty(reducedCosts, newDuty);
                }
                model.Optimize();
            }
        }

        public override bool Solve(CancellationToken ct) {
            model = initModel(ct);
            model.Optimize();

            Console.WriteLine($"Initializing solution.");


            // Properly initialize model
            runVehicleIts(true);
            updateBlockCover();
            Console.WriteLine($"Vehicle solution found with {taskVars.Sum(x => x.X):0.##} vehicles, {css.ActiveBlockCount} blocks");
            runCrewIts(true);
            Console.WriteLine($"Corresponding crew solution found with {dutyVars.Sum(x => x.X):0.##} members");

            Console.WriteLine($"Solution initialized.");
            Console.WriteLine("R\tMV\t#VT\tVHS\t#CD");
            Console.WriteLine($"I\t{model.ObjVal:0.##}\t{taskVars.Sum(x => x.X):0.##}\t{model.GetVarByName("vehicle_slack").X:0.##}\t{dutyVars.Sum(x => x.X):0.##}");

            // Continue running using initial good covering
            for (int round = 0; round < Config.VCSP_ROUNDS; round++) {
                runVehicleIts(false);
                updateBlockCover();
                runCrewIts(false);
                Console.WriteLine($"{round}\t{model.ObjVal:0.##}\t{taskVars.Sum(x => x.X):0.##}\t{model.GetVarByName("vehicle_slack").X:0.##}\t{dutyVars.Sum(x => x.X):0.##}");
            }

            // Make binary
            foreach (GRBVar var in taskVars) {
                var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }
            foreach (GRBVar var in dutyVars) {
                var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }

            updateBlockCover();
            model.Update();
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true;
            model.Optimize();
            Config.CONSOLE_GUROBI = configState;

            (var selectedTasks, _, var selectedDuties) = finalizeResults(true);
            vss.SelectedTasks = selectedTasks;
            css.SelectedDuties = selectedDuties;

            return true;
        }
    }
}
