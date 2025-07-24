using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using System.Collections;

namespace E_VCSP.Solver.SolutionState {
    public enum BlockArcType {
        Break,
        LongIdle,
        ShortIdle,
        Travel,
        SignOnOff,
        Invalid,
    }

    public class BlockArc {
        public Block? FromBlock;
        public Block? ToBlock;
        public int BreakTime;
        public int BruttoNettoTime;
        public int IdleTime;
        public int TravelTime;
        public BlockArcType Type = BlockArcType.Invalid;

        public int TotalTime => BreakTime + BruttoNettoTime + IdleTime + TravelTime;
    }

    public class CrewSolutionState {
        public Instance Instance;
        // Invariant: adj/adjfull last two entries are day start/day end
        public List<List<BlockArc>> Adj = [[], []];
        public List<List<BlockArc?>> AdjFull = [[null, null], [null, null]];

        public List<Block> Blocks = [];
        public List<bool> BlockActive = [];

        public List<CrewDuty> SelectedDuties = [];
        public List<CrewDuty> Duties = [];
        public Dictionary<string, CrewDuty> VarnameDutyMapping = [];
        public Dictionary<BitArray, CrewDuty> CoverDutyMapping = new(new Utils.BitArrayComparer());

        public int ActiveBlockCount => BlockActive.Count(b => b);

        public CrewSolutionState(Instance instance, List<Block> initialBlocks) {
            this.Instance = instance;

            foreach (Block b in initialBlocks) {
                AddBlock(b);
            }
        }

        private BlockArc? LinkBlocks(Block from, Block to) {
            if (from.EndTime > to.StartTime || from.EndLocation != to.StartLocation) return null;

            Location idleLocation = from.EndLocation;
            BlockArc? arc = null;
            BlockArcType blockArcType = BlockArcType.Invalid;
            int idleTime = to.StartTime - from.EndTime;
            int breakTime = 0;
            int bruttoNettoTime = 0;
            int travelTime = 0;
            int nettoBreakTime = idleLocation.BreakAllowed
                ? idleTime - idleLocation.BrutoNetto
                : 0;

            if (nettoBreakTime >= Config.CR_MIN_BREAK_TIME && nettoBreakTime <= Config.CR_MAX_BREAK_TIME) {
                breakTime = idleTime - idleLocation.BrutoNetto;
                bruttoNettoTime = idleLocation.BrutoNetto;
                idleTime = 0;
                blockArcType = BlockArcType.Break;
            }

            // Short idle; can happen anywhere (driver remains in bus)
            else if (Config.CR_MIN_SHORT_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_SHORT_IDLE_TIME)
                blockArcType = BlockArcType.ShortIdle;

            // Long idle used for split shifts
            else if (idleLocation.CrewHub && Config.CR_MIN_LONG_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_LONG_IDLE_TIME)
                blockArcType = BlockArcType.LongIdle;

            if (blockArcType != BlockArcType.Invalid) {
                arc = new() {
                    FromBlock = from,
                    ToBlock = to,
                    IdleTime = idleTime,
                    BreakTime = breakTime,
                    BruttoNettoTime = bruttoNettoTime,
                    TravelTime = travelTime,
                    Type = blockArcType
                };
            }

            return arc;
        }

        public CrewDuty AddBlock(Block b) {
            int newBlockIndex = Blocks.Count;
            b.Index = newBlockIndex;
            Blocks.Add(b);
            BlockActive.Add(false);

            // Create a unit duty in order to process the block
            var duty = new CrewDuty([new CDEBlock(b)]) {
                Index = Duties.Count,
                Type = DutyType.Single
            };
            Duties.Add(duty);

            // Process the block in the adjacency matrixes
            AdjFull.Insert(newBlockIndex, []);
            Adj.Insert(newBlockIndex, []);

            // Connect new block to other blocks
            for (int bPrimeIndex = 0; bPrimeIndex < Blocks.Count - 1; bPrimeIndex++) {
                Block bPrime = Blocks[bPrimeIndex];

                // b -> b'
                BlockArc? bToBPrime = LinkBlocks(b, bPrime);
                AdjFull[newBlockIndex].Add(bToBPrime);
                if (bToBPrime != null) Adj[newBlockIndex].Add(bToBPrime);

                // b' -> b
                BlockArc? bPrimeToB = LinkBlocks(bPrime, b);
                AdjFull[bPrimeIndex].Insert(newBlockIndex, bPrimeToB);
                if (bPrimeToB != null) Adj[bPrimeIndex].Add(bPrimeToB);
            }

            // Self reference
            AdjFull[newBlockIndex].Add(null);

            // Add depot arcs if signon / signoff is allowed
            BlockArc? start = b.StartLocation.CrewHub ? new BlockArc() {
                ToBlock = b,
                IdleTime = 0,
                BreakTime = 0,
                BruttoNettoTime = 0,
                TravelTime = b.StartLocation.SignOnTime,
                Type = BlockArcType.SignOnOff
            } : null;
            if (start != null) Adj[^2].Add(start);
            AdjFull[^2].Insert(newBlockIndex, start);
            AdjFull[newBlockIndex].Add(null);

            BlockArc? end = b.EndLocation.CrewHub ? new BlockArc() {
                FromBlock = b,
                IdleTime = 0,
                BreakTime = 0,
                BruttoNettoTime = 0,
                TravelTime = b.StartLocation.SignOffTime,
                Type = BlockArcType.SignOnOff
            } : null;
            if (end != null) Adj[newBlockIndex].Add(end);
            AdjFull[newBlockIndex].Add(end);
            AdjFull[^1].Add(null);

            // Return the unit duty used to cover the new block
            return duty;
        }
    }
}
