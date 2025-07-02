using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver
{
    public abstract class CrewColumnGen
    {
        // Information about current instance / model state
        internal List<Block> blocks;
        internal GRBModel model;

        internal List<List<BlockArc>> adj = [];
        internal List<List<BlockArc?>> adjFull = [];

        internal CrewColumnGen(List<Block> blocks, GRBModel model, List<List<BlockArc>> adj, List<List<BlockArc?>> adjFull)
        {
            this.blocks = blocks;
            this.model = model;
            this.adj = adj;
            this.adjFull = adjFull;
        }

        public abstract List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties();
    }
}
