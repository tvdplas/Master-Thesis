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
                var constr = crewConstrs["cover_block_" + css.Blocks[i].Descriptor];
                blockDualCosts[i] = blockSign * constr.Pi;
            }
            maxLongDualCost = crewConstrs["cr_overall_no_excessive_length"].Pi;
            maxAvgDurationDualCost = crewConstrs["cr_overall_limited_average_length"].Pi;
            maxBrokenDualCost = crewConstrs["cr_overall_max_broken"].Pi;
            maxBetweenDualCost = crewConstrs["cr_overall_max_between"].Pi;
        }

        public abstract List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties();
    }
}
