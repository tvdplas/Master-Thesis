using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class CSPLSDuty
    {
        public DutyType Type = DutyType.Single;
        public required CSPLSNode Head;
        public required double Cost;

        public double BaseCost
        {
            get
            {
                double cost = Config.CR_SHIFT_COST;
                if (Type == DutyType.Single) cost += Config.CR_SINGLE_SHIFT_COST;
                if (Type == DutyType.Broken) cost += Config.CR_BROKEN_SHIFT_COST;
                return cost;
            }
        }
    }

    internal class CSPLSNode
    {
        public Block Block;
        public CSPLSNode? Prev;
        public CSPLSNode? Next;

        public CSPLSNode(Block block)
        {
            Block = block;
        }
    }

    internal class CSPLSGlobal
    {
        private List<Block> blocks;
        private GRBModel model;

        private List<List<BlockArc>> adj = [];
        private List<List<BlockArc?>> adjFull = [];

        private List<CSPLSDuty> duties = [];

        private Random random;
        private double T;
        private double alpha;
        private int Q;

        internal CSPLSGlobal(List<Block> blocks, GRBModel model, List<List<BlockArc>> adj, List<List<BlockArc?>> adjFull)
        {
            this.blocks = blocks;
            this.model = model;
            this.adj = adj;
            this.adjFull = adjFull;
            this.random = LSShared.random;
        }

        private void reset()
        {
            // Reset params
            T = Config.CSP_LS_G_STARTING_T;
            alpha = Config.CSP_LS_G_COOLING_RATE;
            Q = (int)Math.Round(-Config.CSP_LS_G_ITERATIONS / (Math.Log(Config.CSP_LS_G_STARTING_T / Config.CSP_LS_G_ENDING_T) / Math.Log(alpha)));

            // Reset duties to unit
            for (int i = 0; i < blocks.Count; i++)
            {
                Block b = blocks[i];
                CSPLSNode node = new(b);
                duties.Add(new() { Cost = 0, Head = node, Type = DutyType.Single });
            }
        }


        private double CalcCost(CSPLSDuty duty)
        {
            double cost = duty.BaseCost;

            return cost;
        }

        private LSOpResult moveSingle()
        {
            // Select two random duties
            int dutyIndex1 = random.Next(duties.Count);
            int dutyIndex2 = random.Next(duties.Count);
            while (dutyIndex2 == dutyIndex1) dutyIndex2 = random.Next(duties.Count);

            CSPLSDuty duty1 = duties[dutyIndex1];
            CSPLSDuty duty2 = duties[dutyIndex2];

            // Select a random part of duty 2 to transfer to duty 1.


            return LSOpResult.Invalid;
        }

        private LSOpResult swapTails()
        {
            return LSOpResult.Invalid;
        }

        private LSOpResult changeType()
        {
            return LSOpResult.Invalid;
        }

        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        internal List<(double reducedCosts, CrewDuty crewDuty)> generateColumns()
        {
            reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (changeType, Config.CSP_LS_G_CHANGE_TYPE),
                (moveSingle, Config.CSP_LS_G_MOVE_SINGLE),
                (swapTails, Config.CSP_LS_G_SWAP_TAILS),
            ];
            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;
            while (currIts < Config.CSP_LS_G_ITERATIONS)
            {
                currIts++;
                if (currIts % Q == 0)
                {
                    T *= alpha;
                }
                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
            }
            return [];
        }
    }
}