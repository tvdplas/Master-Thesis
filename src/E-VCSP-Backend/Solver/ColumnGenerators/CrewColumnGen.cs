using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;

namespace E_VCSP.Solver {
    public abstract class CrewColumnGen {
        // Information about current instance / model state
        public CrewSolutionState css;

        public Dictionary<string, double> rawDualCost = new();
        public int rawBlockSign = 1;

        public List<double> blockDualCosts = [];
        public double maxLongDualCost = 0;
        public double maxAvgDurationDualCost = 0;
        public double maxBrokenDualCost = 0;
        public double maxBetweenDualCost = 0;

        internal CrewColumnGen(CrewSolutionState css) {
            this.css = css;
        }

        internal void UpdateDualCosts(Dictionary<string, double> crewConstrs, int blockSign) {
            rawDualCost = crewConstrs;
            rawBlockSign = blockSign;

            blockDualCosts = css.Blocks.Select(_ => 0.0).ToList();

            for (int i = 0; i < css.Blocks.Count; i++) {
                if (css.BlockCount[i] == 0) continue; // RC dont matter, save time querying
                var constr = crewConstrs[Constants.CSTR_BLOCK_COVER + css.Blocks[i].Descriptor];
                blockDualCosts[i] = blockSign * constr;
            }

            maxLongDualCost = crewConstrs[Constants.CSTR_CR_LONG_DUTIES];
            maxAvgDurationDualCost = crewConstrs[Constants.CSTR_CR_AVG_TIME];
            maxBrokenDualCost = crewConstrs[Constants.CSTR_CR_BROKEN_DUTIES];
            maxBetweenDualCost = crewConstrs[Constants.CSTR_CR_BETWEEN_DUTIES];
        }

        public abstract List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties();
    }
}
