using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;
using System.Collections;
using System.Data;

namespace E_VCSP.Solver {
    public class CSPCG(CrewSolutionState css) : Solver {
        public GRBModel? model;
        private CrewSolutionState css = css;

        /// <summary>
        /// Initialize model for column generaton
        /// </summary>
        /// <param name="ct">Cancellation token</param>
        /// <returns>Model where first <c>n</c> constraints correspond to the <c>n</c> trips, and a list of initial vehicle task vars</returns>
        private (GRBModel model, List<GRBVar> dutyVars) InitModel() {
            // Env
            GRBEnv env = new() {
                LogToConsole = 0,
                LogFile = Path.Combine(Constants.RUN_LOG_FOLDER, "cspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.CSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 1; // upper bound
            model.Parameters.Heuristics = 0.8;
            model.Parameters.RINS = 10;
            model.Parameters.SubMIPNodes = 5000;
            model.Parameters.PumpPasses = 20;
            model.Parameters.NoRelHeurTime = Config.CSP_SOLVER_TIMEOUT_SEC / 4;
            model.Parameters.ImproveStartTime = Config.CSP_SOLVER_TIMEOUT_SEC / 4;
            model.Parameters.Cuts = 1;
            model.Parameters.Presolve = 2;
            model.Parameters.Symmetry = 2;
            model.SetCallback(new CustomGRBCallback(model));

            List<GRBVar> dutyVars = [];
            GRBLinExpr maxDuties = new();

            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost, GRB.CONTINUOUS, name);

                maxDuties += v;

                // Bookkeeping to find variable based on name / cover easily
                BitArray blockCover = css.Duties[i].ToBlockBitArray(css.Blocks.Count);
                int dutyType = (int)css.Duties[i].Type;
                dutyVars.Add(v);
                css.VarnameDutyMapping[name] = css.Duties[i];
                css.CoverTypeDutyMapping.Add((blockCover, dutyType), css.Duties[i]);
            }

            // Add cover constraint for each of the trips
            for (int blockIndex = 0; blockIndex < css.Blocks.Count; blockIndex++) {
                Block b = css.Blocks[blockIndex];
                int count = css.BlockCount[blockIndex];
                GRBLinExpr expr = new();
                for (int i = 0; i < css.Duties.Count; i++) {
                    if (css.Duties[i].BlockIndexCover.Contains(b.Index)) expr.AddTerm(1, dutyVars[i]);
                }

                // Switch between set partition and cover
                char sense = Config.CSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, count, Constants.CSTR_BLOCK_COVER + b.Descriptor);
            }

            // Shift type/time limit constraints
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between

            GRBVar maxDutySlack = model.AddVar(0, GRB.INFINITY, Config.CR_OVER_MAX_COST, GRB.CONTINUOUS, "maxDutySlack");
            GRBVar noExcessiveLengthSlack = model.AddVar(0, GRB.INFINITY, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, GRB.INFINITY, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, GRB.INFINITY, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, GRB.INFINITY, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++) {
                GRBVar v = dutyVars[i];
                CrewDuty duty = css.Duties[i];

                int duration = duty.Elements[^1].EndTime - duty.Elements[0].StartTime;
                noExcessiveLength += Constants.CR_MAX_OVER_LONG_DUTY * v - (duration > Constants.CR_LONG_SHIFT_LENGTH ? v : 0);
                limitedAverageLength += v * (duration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Constants.CR_MAX_BROKEN_SHIFTS - (duty.Type == DutyType.Broken ? v : 0);
                maxBetween += v * Constants.CR_MAX_BETWEEN_SHIFTS - (duty.Type == DutyType.Between ? v : 0);
            }

            model.AddConstr(maxDuties - maxDutySlack <= Config.MAX_DUTIES, Constants.CSTR_MAX_DUTIES);
            model.AddConstr(noExcessiveLength + noExcessiveLengthSlack >= 0, Constants.CSTR_CR_LONG_DUTIES);
            model.AddConstr(limitedAverageLength - limitedAverageLengthSlack <= 0, Constants.CSTR_CR_AVG_TIME);
            model.AddConstr(maxBroken + maxBrokenSlack >= 0, Constants.CSTR_CR_BROKEN_DUTIES);
            model.AddConstr(maxBetween + maxBetweenSlack >= 0, Constants.CSTR_CR_BETWEEN_DUTIES);

            this.model = model;
            return (model, dutyVars);
        }

        private List<(CrewDuty duty, int count)> getSelectedDuties() {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<(CrewDuty duty, int count)> duties = [];
            int[] covered = new int[css.Blocks.Count];
            foreach (GRBVar v in model.GetVars()) {
                if (v.VarName.StartsWith("cd_") && Math.Round(v.X) >= 1) {
                    int count = (int)Math.Round(v.X);
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

            Console.WriteLine($"Covered {coveredTotal}/{css.Blocks.Count} blocks using {duties.Sum(x => x.count)} duties");
            if (coveredTotal < css.Blocks.Count)
                Console.WriteLine("(!) Not all blocks covered");

            return duties;
        }

        public override bool Solve() {
            (var model, var dutyVars) = InitModel();
            model.Optimize();

            double z_prev = double.PositiveInfinity;
            int itsWithoutImprovement = 0;

            // Tracking generated columns
            int maxColumns = Config.CSP_INSTANCES_PER_IT * Config.CSP_MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                lbGenerated = 0,            // Generated with labeling
                lsGenerated = 0;            // Generated with labeling

            // Multithreaded shortestpath searching
            List<List<CrewColumnGen>> instances = [
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLabeling(css))], // Labeling
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLSGlobal(css))], // Labeling
            ];
            List<double> operationChances = [Config.CSP_LABELING_WEIGHT, Config.CSP_LS_GLOBAL_WEIGHT];
            List<double> sums = [operationChances[0]];
            for (int i = 1; i < operationChances.Count; i++) sums.Add(sums[i - 1] + operationChances[i]);

            Console.WriteLine("CSP Column generation started");
            Console.WriteLine("%\tT\tLB\tNF\tDN\tDO\tWRC\tMV");

            Random rnd = new();

            // Process a collection of generated css.duties
            void processDuties(List<(double reducedCost, CrewDuty cd)> dutySet) {
                foreach (var task in dutySet) {
                    (double reducedCost, CrewDuty newDuty) = ((double, CrewDuty))task;

                    // Check if task is already in model 
                    BitArray ba = newDuty.ToBlockBitArray(css.Blocks.Count);
                    int dutyType = (int)newDuty.Type;
                    bool coverExists = css.CoverTypeDutyMapping.ContainsKey((ba, dutyType));

                    // Skip adding column if we already have an equivalent one which is cheaper
                    if (coverExists && css.CoverTypeDutyMapping[(ba, dutyType)].Cost < newDuty.Cost) continue;

                    // Create new column to add to model
                    var modelConstrs = model.GetConstrs();
                    GRBConstr[] constrs = [.. modelConstrs.Where(
                        (_, i) => newDuty.BlockIndexCover.Contains(i) || i >= css.Blocks.Count  // Covers block || one of the shift type.length constraints
                    )];
                    double[] coefficients = new double[constrs.Length];
                    for (int i = 0; i < constrs.Length; i++) {
                        GRBConstr c = constrs[i];
                        double coeff = 0;
                        if (c.ConstrName.StartsWith(Constants.CSTR_BLOCK_COVER))
                            coeff = 1.0;
                        else if (c.ConstrName == Constants.CSTR_MAX_DUTIES)
                            coeff = 1.0;
                        else if (c.ConstrName == Constants.CSTR_CR_LONG_DUTIES)
                            coeff = Constants.CR_MAX_OVER_LONG_DUTY - newDuty.IsLongDuty;
                        else if (c.ConstrName == Constants.CSTR_CR_AVG_TIME)
                            coeff = (newDuty.PaidDuration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1);
                        else if (c.ConstrName == Constants.CSTR_CR_BROKEN_DUTIES)
                            coeff = Constants.CR_MAX_BROKEN_SHIFTS - newDuty.IsBrokenDuty;
                        else if (c.ConstrName == Constants.CSTR_CR_BETWEEN_DUTIES)
                            coeff = Constants.CR_MAX_BETWEEN_SHIFTS - newDuty.IsBetweenDuty;
                        else
                            throw new InvalidOperationException($"Constraint {c.ConstrName} not handled when adding new column");
                        coefficients[i] = coeff;
                    }

                    if (coverExists) {
                        CrewDuty toBeReplaced = css.CoverTypeDutyMapping[(ba, dutyType)];
                        int index = toBeReplaced.Index;
                        newDuty.Index = index;

                        // Bookkeeping; replace task in public datastructures
                        css.Duties[index] = newDuty;
                        css.VarnameDutyMapping[$"cd_{index}"] = newDuty;
                        css.CoverTypeDutyMapping[(ba, dutyType)] = newDuty;

                        // Adjust costs / coefficients in model
                        dutyVars[index].Obj = newDuty.Cost;
                        for (int i = 0; i < constrs.Length; i++) {
                            model.ChgCoeff(constrs[i], dutyVars[index], coefficients[i]);
                        }
                    }
                    else {
                        // Reset non-reduced costs iterations
                        int index = css.Duties.Count;
                        string name = $"cd_{index}";
                        css.Duties.Add(newDuty);
                        newDuty.Index = index;

                        GRBColumn col = new();
                        col.AddTerms(coefficients, constrs);

                        // Add column to model
                        dutyVars.Add(model.AddVar(0, GRB.INFINITY, newDuty.Cost, GRB.CONTINUOUS, col, name));
                        css.VarnameDutyMapping[name] = css.Duties[^1];
                        css.CoverTypeDutyMapping[(ba, dutyType)] = css.Duties[^1];
                    }
                }
            }

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.CSP_MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE) {
                // Display progress
                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10) {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{model.ObjVal}");
                }

                // Select solution strategy, generate new css.css.duties
                List<(double, CrewDuty)>[] generatedDuties = new List<(double, CrewDuty)>[Config.CSP_INSTANCES_PER_IT];
                double r = rnd.NextDouble() * sums[^1];
                int selectedMethodIndex = sums.FindIndex(x => r <= x);
                List<CrewColumnGen> selectedMethod = instances[selectedMethodIndex];
                Parallel.For(0, Config.CSP_INSTANCES_PER_IT, (i) => {
                    Dictionary<string, double> crewConstrs = model.GetConstrs().ToDictionary(c => c.ConstrName, c => c.Pi);
                    selectedMethod[i].UpdateDualCosts(model.GetConstrs().ToDictionary(c => c.ConstrName, c => c.Pi), 1);
                    generatedDuties[i] = selectedMethod[i].GenerateDuties();
                });

                // Update generated totals
                totalGenerated += generatedDuties.Length;
                int colsGenerated = generatedDuties.Sum(t => t.Count);
                switch (selectedMethodIndex) {
                    case 0: lbGenerated += colsGenerated; break;
                    case 1: lsGenerated += colsGenerated; break;
                    default: throw new InvalidOperationException("You forgot to add a case");
                }

                // Add new duty columns to model, keep track of totals 
                foreach (var dutySet in generatedDuties) processDuties(dutySet);

                // Resolve model
                model.Update();
                model.Set(GRB.IntParam.Presolve, 0);
                model.Optimize();
                if (model.ObjVal < z_prev) {
                    itsWithoutImprovement = 0;
                }
                else {
                    itsWithoutImprovement++;
                    if (itsWithoutImprovement > Config.CSP_OPT_IT_THRESHOLD) {
                        Console.WriteLine($"Stopped due to no improvement for {Config.CSP_OPT_IT_THRESHOLD} consecutive rounds");
                        break;
                    }
                }
                z_prev = model.ObjVal;
                currIts++;
            }

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED) {
                Console.WriteLine("Model infeasible / canceled");
                model.ComputeIIS();
                model.Write("infeasible_CSPCG.ilp");
                return false;
            }

            // Info dump
            Console.WriteLine($"100%\t{totalGenerated}\t{lbGenerated}\t{model.ObjVal}");
            Console.WriteLine($"Value of relaxation: {model.ObjVal}");
            Console.WriteLine($"Total generation attempts: ${totalGenerated}");
            Console.WriteLine($"Solving non-relaxed model with total of {css.Duties.Count} columns");

            // Make model binary again
            foreach (GRBVar var in dutyVars) {
                if (var.VarName.StartsWith("cd_"))
                    var.Set(GRB.CharAttr.VType, GRB.BINARY);
            }
            model.Update();

            bool configState = Config.CONSOLE_GUROBI;
            Config.CONSOLE_GUROBI = true; // Force enable console at end as this solve takes a long time
            model.Optimize();

            bool succes = !(model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED);

            if (!succes) {
                Console.WriteLine("Model infeasible / canceled");
                model.ComputeIIS();
                model.Write("infeasible_CSPCG.ilp");
            }
            else Console.WriteLine($"Costs: {model.ObjVal}");

            Config.CONSOLE_GUROBI = configState;
            css.SelectedDuties = getSelectedDuties();
            css.PrintCostBreakdown(
                (int)model.GetVarByName("maxDutySlack").X,
                model.GetVarByName("limitedAverageLengthSlack").X,
                model.GetVarByName("noExcessiveLengthSlack").X,
                model.GetVarByName("maxBrokenSlack").X,
                model.GetVarByName("maxBetweenSlack").X
            );

            if (Config.DUMP_CSP) css.Dump();
            return succes;
        }
    }
}
