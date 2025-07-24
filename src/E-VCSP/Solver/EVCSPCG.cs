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

        public EVCSPCG(VehicleSolutionState vss, CrewSolutionState css) {
            this.vss = vss;
            this.css = css;

            // Dump blocks from vss initial solution, add to crew solution
            for (int i = 0; i < vss.Tasks.Count; i++) {
                processVTBlockCover(vss.Tasks[i]);
            }
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


        private void processVTBlockCover(VehicleTask vt) {
            List<Block> blocks = Block.FromVehicleTask(vt);
            List<int> BlockCover = [];

            for (int j = 0; j < blocks.Count; j++) {
                Block block = blocks[j];
                string descriptor = block.Descriptor;
                int knownBlockIndex = -1;

                if (descriptorToKnownBlockIndex.ContainsKey(descriptor)) {
                    knownBlockIndex = descriptorToKnownBlockIndex[descriptor];
                    knownBlocks[knownBlockIndex].CoveredCount++;
                }
                else {
                    knownBlockIndex = knownBlocks.Count;
                    descriptorToKnownBlockIndex[descriptor] = knownBlockIndex;
                    knownBlocks.Add(new BlockWrapper() { Block = block, CoveredCount = 1 });

                    // New base block needs to be added to css
                    css.AddBlock(block);
                }

                BlockCover.Add(knownBlockIndex);
            }

            vt.BlockCover = BlockCover;
        }

        private (GRBModel model, List<GRBVar> taskVars) initModel(CancellationToken ct) {
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
            // Vehicle cover

            List<GRBVar> taskVars = [];
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, vss.Tasks[i].Cost, GRB.CONTINUOUS, name);
                taskVars.Add(v);
                vss.VarnameTaskMapping[name] = vss.Tasks[i];
                vss.CoverTaskMapping.Add(vss.Tasks[i].ToBitArray(vss.Instance.Trips.Count), vss.Tasks[i]);
            }
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_trip_" + t.Id);
            }

            this.model = model;
            return (model, taskVars);
        }

        public override bool Solve(CancellationToken ct) {
            (GRBModel model, List<GRBVar> taskVars) = initModel(ct);
            model.Optimize();

            DEBUG_printBlockCoverRequirements();


            // Generate new columns
            VSPLabeling vspLabelingInstance = new(model, vss);

            for (int djfas = 0; djfas < 10; djfas++) {
                var newColumns = vspLabelingInstance.GenerateVehicleTasks();

                foreach ((var reducedCosts, var newTask) in newColumns) {
                    // Determine the block coverage of the new column
                    processVTBlockCover(newTask);

                    // Add it to the model
                    int index = vss.Tasks.Count;
                    string name = $"vt_{index}";
                    vss.Tasks.Add(newTask);
                    newTask.Index = index;

                    var modelConstrs = model.GetConstrs();
                    // TODO: dit werkt zometeen niet meer, doen aan de hand van naam
                    GRBConstr[] constrs = [.. modelConstrs.Where((_, i) => newTask.TripCover.Contains(i))];    // Covers trip
                    GRBColumn col = new();
                    col.AddTerms([.. constrs.Select(_ => 1.0)], constrs);

                    // Add column to model
                    taskVars.Add(model.AddVar(0, GRB.INFINITY, newTask.Cost, GRB.CONTINUOUS, col, name));
                    vss.VarnameTaskMapping[name] = vss.Tasks[^1];
                    vss.CoverTaskMapping[newTask.ToBitArray(vss.Instance.Trips.Count)] = vss.Tasks[^1];
                }

                model.Optimize();

                // Reevaluate covered block counts
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
                Console.WriteLine("---------------------------------------------------");
                DEBUG_printBlockCoverRequirements();
            }

            foreach (GRBVar var in taskVars) {
                if (var.VarName.StartsWith("vt_"))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }
            model.Update();
            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true;
            model.Optimize();
            Config.CONSOLE_GUROBI = configState;

            (var selectedTasks, var blocks) = finalizeResults(true);

            return true;
            // Find initial solution for vehicle part

            // Transform into blocks

            // Add (new) block constraints to model

            // Resolve for crew

            // Etc. 
        }
    }
}
