using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;
using System.Collections;
using System.Data;

namespace E_VCSP.Solver {
    public class CSPCG(CrewSolutionState css) : Solver {
        private GRBModel? model;
        private CrewSolutionState css = css;

        /// <summary>
        /// Initialize model for column generaton
        /// </summary>
        /// <param name="ct">Cancellation token</param>
        /// <returns>Model where first <c>n</c> constraints correspond to the <c>n</c> trips, and a list of initial vehicle task vars</returns>
        private (GRBModel model, List<GRBVar> dutyVars) InitModel(CancellationToken ct) {
            // Env
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "cspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.CSP_SOLVER_TIMEOUT_SEC;
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() => {
                Console.WriteLine("Cancellation requested during crew scheduling. Terminating Gurobi model...");
                model?.Terminate();
            });

            List<GRBVar> dutyVars = [];
            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost + 1_000, GRB.CONTINUOUS, name);

                // Bookkeeping to find variable based on name / cover easily
                dutyVars.Add(v);
                css.VarnameDutyMapping[name] = css.Duties[i];
                css.CoverDutyMapping.Add(css.Duties[i].ToBitArray(css.Blocks.Count), css.Duties[i]);
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip 
            foreach (Block b in css.Blocks) {
                GRBLinExpr expr = new();
                for (int i = 0; i < css.Duties.Count; i++) {
                    if (css.Duties[i].BlockCover.Contains(b.Index)) expr.AddTerm(1, dutyVars[i]);
                }

                // Switch between set partition and cover
                char sense = Config.CSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_block_" + b.Descriptor);
            }

            // Shift type/time limit constraints
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
            return (model, dutyVars);
        }

        private List<CrewDuty> getSelectedDuties() {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<CrewDuty> duties = [];
            int[] covered = new int[css.Blocks.Count];
            foreach (GRBVar v in model.GetVars()) {
                if (v.VarName.StartsWith("cd_") && v.X == 1) {
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

            Console.WriteLine($"Covered {coveredTotal}/{css.Blocks.Count} blocks using {duties.Count} duties");
            if (coveredTotal < css.Blocks.Count)
                Console.WriteLine("(!) Not all blocks covered");

            return duties;
        }

        public override bool Solve(CancellationToken cancellationToken) {
            (var model, var dutyVars) = InitModel(cancellationToken);
            model.Optimize();

            // Assume all blocks are active
            css.BlockActive = css.Blocks.Select(_ => true).ToList();

            // Tracking generated columns
            int maxColumns = Config.CSP_INSTANCES_PER_IT * Config.CSP_MAX_COL_GEN_ITS,
                lastReportedPercent = 0,    // Percentage of total reporting
                currIts = 1,                // Number of CG / solution rounds had
                totalGenerated = 0,         // Total number of columns generated
                lbGenerated = 0,            // Generated with labeling
                lsGenerated = 0,            // Generated with labeling
                seqWithoutRC = 0,           // Number of sequential columns without reduced cost found
                totWithoutRC = 0,           // Total columns generated with no RC
                notFound = 0;               // Number of columns that could not be generated

            // Multithreaded shortestpath searching
            List<List<CrewColumnGen>> instances = [
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLabeling(model, css, 1))], // Labeling
                [.. Enumerable.Range(0, Config.CSP_INSTANCES_PER_IT).Select(_ => new CSPLSGlobal(model, css))], // Labeling
            ];
            List<double> operationChances = [Config.CSP_LABELING_WEIGHT, Config.CSP_LS_GLOBAL_WEIGHT];
            List<double> sums = [operationChances[0]];
            for (int i = 1; i < operationChances.Count; i++) sums.Add(sums[i - 1] + operationChances[i]);

            Console.WriteLine("CSP Column generation started");
            Console.WriteLine("%\tT\tLB\tNF\tDN\tDO\tWRC\tMV");

            Random rnd = new();

            // Process a collection of generated css.duties
            void processDuties(List<(double reducedCost, CrewDuty cd)> dutySet) {
                if (dutySet.Count == 0) {
                    notFound++;
                    return;
                }

                foreach (var task in dutySet) {
                    (double reducedCost, CrewDuty newDuty) = ((double, CrewDuty))task;

                    // Check if task is already in model 
                    BitArray ba = newDuty.ToBitArray(css.Blocks.Count);
                    bool coverExists = css.CoverDutyMapping.ContainsKey(ba);

                    // Add column to model 
                    if (reducedCost < 0) {
                        // Reset non-reduced costs iterations
                        int index = css.Duties.Count;
                        string name = $"cd_{index}";
                        css.Duties.Add(newDuty);
                        newDuty.Index = index;

                        // Create new column to add to model
                        var modelConstrs = model.GetConstrs();
                        GRBConstr[] constrs = [.. modelConstrs.Where(
                            (_, i) => newDuty.BlockCover.Contains(i) || i >= css.Blocks.Count  // Covers block || one of the shift type.length constraints
                        )];
                        GRBColumn col = new();
                        col.AddTerms([.. constrs.Select(c => {
                            if (c.ConstrName.StartsWith("cover_block_")) return 1.0;
                            else if (c.ConstrName == "cr_overall_no_excessive_length") return newDuty.Duration > Config.CR_LONG_SHIFT_LENGTH ? Config.CR_MAX_OVER_LONG_SHIFT - 1 : Config.CR_MAX_OVER_LONG_SHIFT;
                            else if (c.ConstrName == "cr_overall_limited_average_length") return (newDuty.Duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                            else if (c.ConstrName == "cr_overall_max_broken") return newDuty.Type == DutyType.Broken ? Config.CR_MAX_BROKEN_SHIFTS - 1 : Config.CR_MAX_BROKEN_SHIFTS;
                            else if (c.ConstrName == "cr_overall_max_between") return newDuty.Type == DutyType.Between ? Config.CR_MAX_BETWEEN_SHIFTS - 1 : Config.CR_MAX_BETWEEN_SHIFTS;
                            else throw new InvalidOperationException($"Constraint {c.ConstrName} not handled when adding new column");
                        })], constrs);

                        // Add column to model
                        dutyVars.Add(model.AddVar(0, GRB.INFINITY, newDuty.Cost, GRB.CONTINUOUS, col, name));
                        css.VarnameDutyMapping[name] = css.Duties[^1];
                        css.CoverDutyMapping[ba] = css.Duties[^1];
                    }
                    else {
                        seqWithoutRC++;
                        totWithoutRC++;
                    }
                }
            }

            // Continue until max number of columns is found, model isn't feasible during solve or break
            // due to RC constraint. 
            while (currIts < Config.CSP_MAX_COL_GEN_ITS && model.Status != GRB.Status.INFEASIBLE && !cancellationToken.IsCancellationRequested) {
                // Display progress
                int percent = (int)((totalGenerated / (double)maxColumns) * 100);
                if (percent >= lastReportedPercent + 10) {
                    lastReportedPercent = percent - (percent % 10);
                    Console.WriteLine($"{lastReportedPercent}%\t{totalGenerated}\t{lbGenerated}\t{notFound}\t{totWithoutRC}\t{model.ObjVal}");
                }

                // Select solution strategy, generate new css.css.duties
                List<(double, CrewDuty)>[] generatedDuties = new List<(double, CrewDuty)>[Config.CSP_INSTANCES_PER_IT];
                double r = rnd.NextDouble() * sums[^1];
                int selectedMethodÍndex = sums.FindIndex(x => r <= x);
                List<CrewColumnGen> selectedMethod = instances[selectedMethodÍndex];
                Parallel.For(0, Config.CSP_INSTANCES_PER_IT, (i) => generatedDuties[i] = selectedMethod[i].GenerateDuties());

                // Update generated totals
                totalGenerated += generatedDuties.Length;
                int colsGenerated = generatedDuties.Sum(t => t.Count);
                switch (selectedMethodÍndex) {
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
                currIts++;

                // Stop model solving if no improvements are found
                if (seqWithoutRC >= Config.CSP_OPT_IT_THRESHOLD) {
                    Console.WriteLine($"Stopped due to RC > 0 for {Config.CSP_OPT_IT_THRESHOLD} consecutive css.css.duties");
                    break;
                }
            }

            if (model.Status == GRB.Status.INFEASIBLE || model.Status == GRB.Status.INTERRUPTED) {
                Console.WriteLine("Model infeasible / canceled");
                model.ComputeIIS();
                model.Write("infeasible_CSPCG.ilp");
                return false;
            }

            // Info dump
            Console.WriteLine($"100%\t{totalGenerated}\t{lbGenerated}\t{notFound}\t{totWithoutRC}\t{model.ObjVal}");
            Console.WriteLine($"Value of relaxation: {model.ObjVal}");
            Console.WriteLine($"Total generation attempts: ${totalGenerated}");
            Console.WriteLine($"{totWithoutRC} columns were not added due to positive reduced costs.");
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
            return succes;
        }
    }
}
