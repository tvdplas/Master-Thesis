using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    public abstract class CrewColumnGen {
        // Information about current instance / model state
        public CrewSolutionState css;
        public List<double> blockDualCosts = [];
        public double maxLongDualCost = 0;
        public double maxAvgDurationDualCost = 0;
        public double maxBrokenDualCost = 0;
        public double maxBetweenDualCost = 0;

        internal CrewColumnGen(CrewSolutionState css) {
            this.css = css;
        }

        internal void UpdateDualCosts(Dictionary<string, GRBConstr> crewConstrs, int blockSign) {
            blockDualCosts = css.Blocks.Select(_ => 0.0).ToList();

            for (int i = 0; i < css.Blocks.Count; i++) {
                if (css.BlockCount[i] == 0) continue; // RC dont matter, save time querying
                var constr = crewConstrs[Constants.CSTR_BLOCK_COVER + css.Blocks[i].Descriptor];
                blockDualCosts[i] = blockSign * constr.Pi;
            }

            maxLongDualCost = crewConstrs[Constants.CSTR_CR_LONG_DUTIES].Pi;
            maxAvgDurationDualCost = crewConstrs[Constants.CSTR_CR_AVG_TIME].Pi;
            maxBrokenDualCost = crewConstrs[Constants.CSTR_CR_BROKEN_DUTIES].Pi;
            maxBetweenDualCost = crewConstrs["cr_overall_max_between"].Pi;
        }

        public abstract List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties();
    }
}
