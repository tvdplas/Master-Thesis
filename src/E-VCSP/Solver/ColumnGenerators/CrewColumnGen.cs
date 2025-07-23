using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    public abstract class CrewColumnGen {
        // Information about current instance / model state
        public GRBModel model;
        public CrewSolutionState css;

        internal CrewColumnGen(GRBModel model, CrewSolutionState css) {
            this.model = model;
            this.css = css;
        }

        public abstract List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties();
    }
}
