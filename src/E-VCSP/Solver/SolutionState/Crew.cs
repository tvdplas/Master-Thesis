
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using System.Collections;
using System.Text.Json.Serialization;

namespace E_VCSP.Solver.SolutionState {
    public class CrewSolutionStateDump {
        [JsonInclude]
        public required string path;
        [JsonInclude]
        public List<BlockDump> blocks = [];
        [JsonInclude]
        public List<(CrewDuty duty, int count)> selectedDuties = [];
    }

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

        public HashSet<string> KnownBlocks = [];
        public List<Block> Blocks = [];
        public List<int> BlockCount = [];

        public List<(CrewDuty duty, int count)> SelectedDuties = [];
        public List<CrewDuty> Duties = [];
        public Dictionary<string, CrewDuty> VarnameDutyMapping = [];
        public Dictionary<BitArray, CrewDuty> CoverDutyMapping = new(new Utils.BitArrayComparer());

        public int ActiveBlockCount => BlockCount.Sum(b => b);

        public CrewSolutionState(Instance instance, List<(Block block, int count)> initialBlocks) {
            this.Instance = instance;

            foreach ((Block block, int count) in initialBlocks) {
                AddBlock(block, count);
            }
        }

        #region dump
        public void Dump() {
            File.WriteAllText(Path.Join(Constants.RUN_LOG_FOLDER, "css-result.json"),
            System.Text.Json.JsonSerializer.Serialize(new CrewSolutionStateDump {
                path = Instance.Path,
                blocks = Blocks.Select(b => BlockDump.FromBlock(b)).ToList(),
                selectedDuties = SelectedDuties
            }));
        }

        public void LoadFromDump(CrewSolutionStateDump dump) {
            if (dump.path != Instance.Path)
                throw new InvalidOperationException("Cannot read dump for instance different than one loaded");

            List<Block> blocks = dump.blocks.Select(b => {
                Location startLocation = Instance.Locations.Find(l => l.Id == b.StartLocation.Id)!;
                Location endLocation = Instance.Locations.Find(l => l.Id == b.EndLocation.Id)!;
                return Block.FromDescriptor(startLocation, b.StartTime, endLocation, b.EndTime);
            }).ToList();

            List<CrewDuty> duties = dump.selectedDuties.Select((dc, i) => {
                CrewDuty d = dc.duty;
                d.BlockIndexCover = [];
                d.Elements = d.Elements.Select(e => {
                    e.StartLocation = Instance.Locations.Find(l => l.Id == e.StartLocation.Id)!;
                    e.EndLocation = Instance.Locations.Find(l => l.Id == e.EndLocation.Id)!;

                    if (e is CDEBlock cdeb) {
                        int blockIndex = blocks.FindIndex(x => {
                            return x.EndTime == cdeb.EndTime
                            && x.StartTime == cdeb.StartTime
                            && x.StartLocation.Id == cdeb.StartLocation.Id
                            && x.EndLocation.Id == cdeb.EndLocation.Id;
                        });
                        cdeb.Block = blocks[blockIndex];
                        d.BlockIndexCover.Add(blockIndex);
                    }
                    return e;
                }).ToList();
                d.Index = i + blocks.Count;
                return d;
            }).ToList();

            Blocks = blocks;
            ResetFromBlocks();
            Duties.AddRange(duties);
            SelectedDuties = duties.Select((d, i) => (d, dump.selectedDuties[i].count)).ToList();

            PrintCostBreakdown();
        }
        #endregion

        public void ResetFromBlocks() {
            SelectedDuties.Clear();
            Duties.Clear();
            VarnameDutyMapping.Clear();
            CoverDutyMapping.Clear();
            Adj = [[], []];
            AdjFull = [[null, null], [null, null]];

            HashSet<string> oldKnownBlocks = [.. KnownBlocks];
            List<Block> oldBlocks = [.. Blocks];
            List<int> oldCounts = [.. BlockCount];
            Blocks.Clear();
            BlockCount.Clear();
            oldKnownBlocks.Clear();

            for (int i = 0; i < oldBlocks.Count; i++) {
                AddBlock(oldBlocks[i], oldCounts[i]);
            }
        }

        /// <summary>
        /// Resets the block and adjacency, removes all unit block tasks
        /// </summary>
        public void ResetBlocks() {
            Adj = [[], []];
            AdjFull = [[null, null], [null, null]];
            Blocks = [];
            BlockCount = [];
            KnownBlocks = [];
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

            if (nettoBreakTime >= Constants.CR_MIN_BREAK_TIME && nettoBreakTime <= Constants.CR_MAX_BREAK_TIME) {
                breakTime = idleTime - idleLocation.BrutoNetto;
                bruttoNettoTime = idleLocation.BrutoNetto;
                idleTime = 0;
                blockArcType = BlockArcType.Break;
            }

            // Short idle; can happen anywhere (driver remains in bus)
            else if (Constants.CR_MIN_SHORT_IDLE_TIME <= idleTime && idleTime <= Constants.CR_MAX_SHORT_IDLE_TIME)
                blockArcType = BlockArcType.ShortIdle;

            // Long idle used for split shifts
            else if (idleLocation.CrewBase) {
                int nettoRestTime = idleTime - idleLocation.SignOnTime - idleLocation.SignOffTime;
                if (Constants.CR_MIN_LONG_IDLE_TIME <= nettoRestTime && nettoRestTime <= Constants.CR_MAX_LONG_IDLE_TIME) {
                    blockArcType = BlockArcType.LongIdle;
                }
            }

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

        public CrewDuty? AddBlock(Block b, int count) {
            string desc = b.Descriptor;
            if (KnownBlocks.Contains(desc))
                return null;

            int newBlockIndex = Blocks.Count;
            b.Index = newBlockIndex;
            Blocks.Add(b);
            BlockCount.Add(count);
            KnownBlocks.Add(desc);

            // Check if we already have a unit duty available
            bool addUnit = true;
            for (int i = 0; i < Duties.Count; i++) {
                CrewDuty cd = Duties[i];
                if (cd.IsUnit && cd.BlockDescriptorCover.Count == 1 && cd.BlockDescriptorCover[0] == desc) {
                    addUnit = false;
                    break;
                }
            }
            // If not, add it
            if (addUnit) {
                var duty = new CrewDuty([new CDEBlock(b)]) {
                    Index = Duties.Count,
                    Type = DutyType.Single,
                    IsUnit = true,
                };
                Duties.Add(duty);
            }

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
            BlockArc? start = b.StartLocation.CrewBase ? new BlockArc() {
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

            BlockArc? end = b.EndLocation.CrewBase ? new BlockArc() {
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
            return addUnit ? Duties[^1] : null;
        }

        public void PrintCostBreakdown(
            int countSlack = 0,
            double avgDutySlack = 0,
            double longDutySlack = 0,
            double brokenDutySlack = 0,
            double betweenDutySlack = 0
        ) {
            double countPenalty = countSlack * Config.CR_OVER_MAX_COST;
            double avgPenalty = avgDutySlack * Constants.CR_HARD_CONSTR_PENALTY;
            double longPenalty = longDutySlack * Constants.CR_HARD_CONSTR_PENALTY;
            double maxBrokenPenalty = brokenDutySlack * Constants.CR_HARD_CONSTR_PENALTY;
            double maxBetweenPenalty = betweenDutySlack * Constants.CR_HARD_CONSTR_PENALTY;
            double totalPenalty = countPenalty + avgPenalty + longPenalty + maxBrokenPenalty + maxBetweenPenalty;

            double workingHours = SelectedDuties.Sum(x => x.duty.PaidDuration * x.count) / 60.0 / 60.0;
            double blockHours = SelectedDuties.Sum(x => x.count * x.duty.Elements.Sum(y => {
                if (y is CDEBlock) return y.EndTime - y.StartTime;
                else return 0;
            })) / 60.0 / 60.0;
            double blockDrivingHours = SelectedDuties.Sum(x => x.count * x.duty.Elements.Sum(y => {
                if (y is CDEBlock b) return b.Block.Elements.Sum(z => {
                    if (z is BETrip) return z.EndTime - z.StartTime;
                    else return 0;
                });
                else return 0;
            })) / 60.0 / 60.0;
            double breakHours = SelectedDuties.Sum(x => x.count * x.duty.Elements.Sum(y => {
                if (y is CDEBreak) return y.EndTime - y.StartTime;
                else return 0;
            })) / 60.0 / 60.0;

            string breakdown =
            $"""
            Overall crew costs: {SelectedDuties.Sum(x => x.count * x.duty.Cost) + totalPenalty}
            Cost breakdown:
            Crew duties: {SelectedDuties.Sum(x => x.count * x.duty.Cost)}
                Overall paid time: {workingHours * Constants.CR_HOURLY_COST} ({workingHours.ToString("0.##")}h)
                    Block: {blockHours * Constants.CR_HOURLY_COST} ({blockHours.ToString("0.##")}h)
                        Driving: {blockDrivingHours * Constants.CR_HOURLY_COST} ({blockDrivingHours.ToString("0.##")}h)
                        Idle: {(blockHours - blockDrivingHours) * Constants.CR_HOURLY_COST} ({(blockHours - blockDrivingHours).ToString("0.##")}h)
                    Break: {breakHours * Constants.CR_HOURLY_COST} ({breakHours.ToString("0.##")}h)
                    Other: {(workingHours - blockHours - breakHours) * Constants.CR_HOURLY_COST} ({(workingHours - blockHours - breakHours).ToString("0.##")}h)
                Base duty cost: {SelectedDuties.Count * Config.CR_SHIFT_COST}
                Broken duty costs: {SelectedDuties.Count * Constants.CR_BROKEN_SHIFT_COST}
                Single duties: {SelectedDuties.Sum(x => x.count * (x.duty.Type == DutyType.Single ? 1 : 0)) * Config.CR_SINGLE_SHIFT_COST}
            Penalties:
                Duty slack: {countSlack * Config.CR_OVER_MAX_COST}
                Avg duty length slack: {avgPenalty}
                Long duty slack: {longPenalty}
                Broken duty slack: {maxBrokenPenalty}
                Between duty slack: {maxBetweenPenalty}
            """;

            Console.WriteLine(breakdown);
        }
    }
}
