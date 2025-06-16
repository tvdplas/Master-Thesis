using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using Gurobi;
using Microsoft.Msagl.Drawing;
using System.Collections;

namespace E_VCSP.Solver
{
    public class CSPCG : Solver
    {
        private GRBModel? model;
        private Instance instance;

        private List<CrewDuty> duties = [];
        private Dictionary<string, CrewDuty> varnameDutyMapping = [];
        private Dictionary<BitArray, CrewDuty> coverDutyMapping = new(new Utils.BitArrayComparer());

        public CSPCG(Instance instance)
        {
            this.instance = instance;
            this.duties = [];

            // Generate initial set of vehicle tasks
            GenerateInitialDuties();
        }

        private void GenerateInitialDuties()
        {
            // For each block, generate a unit duty
            foreach (var block in instance.Blocks) duties.Add(new CrewDuty([new CDEBlock(block)]));
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
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "cspcg_gurobi.log")
            };

            // Model
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.CSP_SOLVER_TIMEOUT_SEC;
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() =>
            {
                Console.WriteLine("Cancellation requested during crew scheduling. Terminating Gurobi model...");
                model?.Terminate();
            });

            List<GRBVar> dutyVars = [];
            for (int i = 0; i < duties.Count; i++)
            {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, 1, duties[i].Cost, GRB.CONTINUOUS, name);

                // Bookkeeping to find variable based on name / cover easily
                dutyVars.Add(v);
                varnameDutyMapping[name] = duties[i];
                coverDutyMapping.Add(duties[i].ToBitArray(instance.Blocks.Count), duties[i]);
            }

            // Add cover constraint for each of the trips
            // Note: index of constraint corresponds directly to index of trip 
            foreach (Block b in instance.Blocks)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < duties.Count; i++)
                {
                    if (duties[i].Covers.Contains(b.Index)) expr.AddTerm(1, dutyVars[i]);
                }

                // Switch between set partition and cover
                char sense = Config.CSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                model.AddConstr(expr, sense, 1, "cover_" + b.Index);
            }

            this.model = model;
            return (model, dutyVars);
        }

        private List<CrewDuty> getSelectedDuties()
        {
            if (model == null) throw new InvalidOperationException("Cannot generate solution graph without model instance");

            List<CrewDuty> duties = [];
            int[] covered = new int[instance.Blocks.Count];

            foreach (GRBVar v in model.GetVars())
            {
                if (v.VarName.StartsWith("cd_") && v.X == 1)
                {
                    CrewDuty dvt = varnameDutyMapping[v.VarName];
                    duties.Add(dvt);
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
                if (val >= 2) Console.WriteLine($"(!) Block {instance.Blocks[i]} covered {val} times");
            }
            Console.WriteLine($"Covered {coveredTotal}/{instance.Blocks.Count} trips");
            if (coveredTotal < instance.Blocks.Count) Console.WriteLine("(!) Not all trips covered");
            return duties;
        }

        public override Graph GenerateSolutionGraph(bool blockView)
        {
            List<CrewDuty> selectedDuties = getSelectedDuties();
            return SolutionGraph.GenerateCrewDutyGraph(selectedDuties);
        }

        public override bool Solve(CancellationToken cancellationToken)
        {
            (var model, var dutyVars) = InitModel(cancellationToken);
            model.Optimize();

            CSPLabeling cspl = new(instance.Blocks);

            List<double> reducedCosts = [];
            var constrs = model.GetConstrs();
            for (int i = 0; i < instance.Blocks.Count; i++)
            {
                reducedCosts.Add(constrs[i].Pi);
            }

            var x = cspl.generateShortestPath(reducedCosts);


            return true; // :D
        }
    }
}
