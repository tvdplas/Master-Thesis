using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class CSPLSDuty
    {
        public DutyType Type = DutyType.Single;
        public List<CrewDutyElement> Elements = [];
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

        public double CalcCost()
        {
            if (Elements.Count == 0) return 0;

            double cost = BaseCost;

            // TODO: step on / step off
            cost += Elements[^1].EndTime - Elements[0].StartTime / (60.0 * 60.0) * Config.CR_HOURLY_COST;

            // Broken shift largest idle is free
            if (Type == DutyType.Broken)
            {
                var largestIdle = Elements.Where(x => x.Type == CrewDutyElementType.Idle).OrderByDescending(x => x.EndTime - x.StartTime).FirstOrDefault();
                if (largestIdle != null)
                    cost -= (largestIdle.EndTime - largestIdle.StartTime) / (60.0 * 60.0) * Config.CR_HOURLY_COST;
            }

            Cost = cost;
            return cost;
        }
    }

    internal class CSPLSGlobal : CrewColumnGen
    {
        private List<CSPLSDuty> duties = [];

        private Random random;
        private double T;
        private double alpha;
        private int Q;

        internal CSPLSGlobal(List<Block> blocks, GRBModel model, List<List<BlockArc>> adj, List<List<BlockArc?>> adjFull) : base(blocks, model, adj, adjFull)
        {
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
                duties.Add(new() { Cost = -1, Elements = [new CDEBlock(b)], Type = DutyType.Single });
                duties[^1].CalcCost();
            }
        }

        private LSOpResult moveSingle()
        {
            //// Select two random duties
            //int dutyIndex1 = random.Next(duties.Count);
            //int dutyIndex2 = random.Next(duties.Count);
            //while (dutyIndex2 == dutyIndex1) dutyIndex2 = random.Next(duties.Count);

            //CSPLSDuty duty1 = duties[dutyIndex1];
            //CSPLSDuty duty2 = duties[dutyIndex2];

            //// Select a random block from duty 2 to transfer to duty 1.
            //int d2Index = random.Next(duty2.Elements.Count);
            //while (duty2.Elements[d2Index].Type != CrewDutyElementType.Block) random.Next(duty2.Elements.Count);
            //CDEBlock cdeb2 = (CDEBlock)duty2.Elements[d2Index];

            //// Overlap with existing block
            //if (
            //    duty1.Elements.Find(
            //        x => x.Type == CrewDutyElementType.Block
            //        && (cdeb2.StartTime <= x.EndTime && cdeb2.EndTime >= x.StartTime)
            //    ) != null
            //) return LSOpResult.Invalid;

            //// Check if connection can be made
            //// Find last before
            //int d1PrevTrip = duty1.Elements.FindLastIndex(x => x.Type == CrewDutyElementType.Block && x.EndTime <= cdeb2.StartTime);
            //// Either arc must exist from d1prev or d1prev is -1 and cdeb2.startlocation can be used to get on
            //if ((d1PrevTrip >= 0 && adjFull[((CDEBLock)duty1.Elements[d1PrevTrip]).Block.Index][cdeb2.Block.Index] == null)
            //    || (d1PrevTrip == -1 && !cdeb2.StartLocation.CrewBase)
            //) return LSOpResult.Invalid;

            //int d1NextTrip = duty1.Elements.FindIndex(x => x.Type == CrewDutyElementType.Block && x.StartTime >= cdeb2.EndTime);

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

        public override List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties()
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