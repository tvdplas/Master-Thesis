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
        public Instance instance;
        public List<List<BlockArc>> adj = [];
        public List<List<BlockArc?>> adjFull = [];

        public List<CrewDuty> duties = [];
        public Dictionary<string, CrewDuty> varnameDutyMapping = [];
        public Dictionary<BitArray, CrewDuty> coverDutyMapping = new(new Utils.BitArrayComparer());

        public CrewSolutionState(Instance instance) {
            this.instance = instance;

            generateInitialDuties();
            generateArcs();
        }

        private void generateInitialDuties() {
            // For each block, generate a unit duty
            for (int i = 0; i < instance.Blocks.Count; i++) {
                duties.Add(new CrewDuty([new CDEBlock(instance.Blocks[i])]) { Index = i, Type = DutyType.Single });
            }
        }

        private void generateArcs() {
            adjFull = Enumerable.Range(0, instance.Blocks.Count + 2).Select(x => new List<BlockArc?>()).ToList();
            adj = Enumerable.Range(0, instance.Blocks.Count + 2).Select(x => new List<BlockArc>()).ToList();

            for (int blockIndex1 = 0; blockIndex1 < instance.Blocks.Count; blockIndex1++) {
                Block block1 = instance.Blocks[blockIndex1];
                for (int blockIndex2 = 0; blockIndex2 < instance.Blocks.Count; blockIndex2++) {
                    Block block2 = instance.Blocks[blockIndex2];

                    // Determine whether or not it is feasible to string to arcs together
                    // If so: what actually happens during this time. 

                    // For now; only allow transfer if already at same location
                    // Based on times, determine whether its idle / break / whatever. 
                    BlockArc? arc = null;

                    if (block1.EndTime <= block2.StartTime && block1.EndLocation == block2.StartLocation) {
                        // Arc might be formed; start with base time layout, check for validity
                        BlockArcType blockArcType = BlockArcType.Invalid;
                        int idleTime = block2.StartTime - block1.EndTime;
                        int breakTime = 0;
                        int bruttoNettoTime = 0;
                        int travelTime = 0;
                        int nettoBreakTime = block1.EndLocation.BreakAllowed
                            ? idleTime - block1.EndLocation.BrutoNetto
                            : 0;

                        if (nettoBreakTime >= Config.CR_MIN_BREAK_TIME && nettoBreakTime <= Config.CR_MAX_BREAK_TIME) {
                            breakTime = idleTime - block1.EndLocation.BrutoNetto;
                            bruttoNettoTime = block1.EndLocation.BrutoNetto;
                            idleTime = 0;
                            blockArcType = BlockArcType.Break;
                        }

                        // Short idle; can happen anywhere (driver remains in bus)
                        else if (Config.CR_MIN_SHORT_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_SHORT_IDLE_TIME)
                            blockArcType = BlockArcType.ShortIdle;

                        // Long idle used for split shifts
                        else if (block1.EndLocation.CrewHub && Config.CR_MIN_LONG_IDLE_TIME <= idleTime && idleTime <= Config.CR_MAX_LONG_IDLE_TIME)
                            blockArcType = BlockArcType.LongIdle;

                        if (blockArcType != BlockArcType.Invalid) {
                            arc = new() {
                                FromBlock = block1,
                                ToBlock = block2,
                                IdleTime = idleTime,
                                BreakTime = breakTime,
                                BruttoNettoTime = bruttoNettoTime,
                                TravelTime = travelTime,
                                Type = blockArcType
                            };
                        }
                    }

                    if (arc != null) adj[blockIndex1].Add(arc);
                    adjFull[blockIndex1].Add(arc);
                }
            }

            // Add depot arcs if signon / signoff is allowed
            for (int blockIndex = 0; blockIndex < instance.Blocks.Count; blockIndex++) {
                Block block = instance.Blocks[blockIndex];
                BlockArc? start = block.StartLocation.CrewHub ? new BlockArc() {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = block.StartLocation.SignOnTime,
                    Type = BlockArcType.SignOnOff
                } : null;
                if (start != null) adj[^2].Add(start);
                adjFull[^2].Add(start);
                adjFull[blockIndex].Add(null);

                BlockArc? end = block.EndLocation.CrewHub ? new BlockArc() {
                    FromBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = block.StartLocation.SignOffTime,
                    Type = BlockArcType.SignOnOff
                } : null;
                if (end != null)
                    adj[block.Index].Add(end);
                adjFull[block.Index].Add(end);
            }
        }
    }
}
