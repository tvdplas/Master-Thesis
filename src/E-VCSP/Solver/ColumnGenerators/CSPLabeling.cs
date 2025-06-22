using E_VCSP.Objects;
using Gurobi;
using System.Collections;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class BlockArc
    {
        internal Block? FromBlock;
        internal Block? ToBlock;
        internal int BreakTime;
        internal int BruttoNettoTime;
        internal int IdleTime;
        internal int TravelTime;
        internal double ReducedCost = 0;

        internal int TotalTime => BreakTime + BruttoNettoTime + IdleTime + TravelTime;
    }

    internal class CSPLabel
    {
        private static int ID_COUNTER = 0;
        internal readonly int Id = -1;
        internal CSPLabel()
        {
            Id = ID_COUNTER++;
        }

        internal int PrevLabelId = -1;
        internal int PrevBlockId = -1;
        internal DutyType Type;
        internal double Cost = 0;
        internal int StartTime = 0;
        internal required BitArray CoveredBlockIds;
        internal List<(int startTime, int breakTime)> Breaks = new();
        internal List<(int startTime, int idleTime)> Idles = new();

        internal bool breaksFeasible(int currEndTime, bool final)
        {
            // Check driving time between breaks
            int lastBreak = StartTime;
            // Reset driving time after largest idle > 1.5 hours in broken shift
            (int startTimeBigIdle, int durationBigIdle) = (Type == DutyType.Broken && Idles.Count > 0 && Idles.MaxBy(x => x.idleTime).idleTime >= 1.5 * 60 * 60) ? Idles.MaxBy(x => x.idleTime) : (-1, -1);

            for (int i = 0; i < Breaks.Count; i++)
            {
                (int st, int bt) = Breaks[i];
                if (startTimeBigIdle != -1 && startTimeBigIdle < st)
                {
                    // First check time traveled before big idle, then update last break time accordingly
                    if (startTimeBigIdle - lastBreak > Config.MAX_DRIVE_TIME) return false;
                    lastBreak = startTimeBigIdle + durationBigIdle;
                }
                // Too long between breaks
                if (st - lastBreak > Config.MAX_DRIVE_TIME) return false;
                lastBreak = st + bt;
            }
            if (currEndTime - lastBreak > Config.MAX_DRIVE_TIME) return false;

            if (final)
            {
                // Broken shift with big idle included
                if (startTimeBigIdle != -1)
                {
                    // Two parts: before break and after idle
                    int beforeDuration = startTimeBigIdle - StartTime;
                    int afterDuration = currEndTime - (startTimeBigIdle + durationBigIdle);
                    var beforeBreaks = Breaks.Where(x => x.startTime + x.breakTime < StartTime + beforeDuration).ToList();
                    var afterBreaks = Breaks.Where(x => x.startTime + x.breakTime > startTimeBigIdle + durationBigIdle).ToList();

                    if (beforeDuration > 4 * 60 * 60 && beforeDuration <= 5.5 * 60 * 60)
                    {
                        if (beforeBreaks.Count() == 0) return false;
                    }
                    else if (beforeDuration >= 5.5 * 60 * 60)
                    {
                        if (beforeBreaks.Sum(x => x.breakTime) < 40 * 60 || beforeBreaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;
                    }
                    if (afterDuration > 4 * 60 * 60 && afterDuration <= 5.5 * 60 * 60)
                    {
                        if (afterBreaks.Count() == 0) return false;
                    }
                    else if (afterDuration >= 5.5 * 60 * 60)
                    {
                        if (afterBreaks.Sum(x => x.breakTime) < 40 * 60 || afterBreaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;

                    }
                }
                // Continouos shift
                else
                {
                    int duration = currEndTime - StartTime;
                    if (duration > 4 * 60 * 60 && duration <= 5.5 * 60 * 60)
                    {
                        // At least 1 of >= 15 min
                        if (Breaks.Count == 0) return false;
                    }
                    else if (duration > 5 * 60 * 60)
                    {
                        // >= 40 min, at least one >= 20 min
                        if (Breaks.Sum(x => x.breakTime) < 40 * 60 || Breaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;
                    }
                }

                // Extra check for times for late
                if (Type == DutyType.Late)
                {
                    // must be 20 min break between 16:30 and 20:30 if starting before 15:00
                    if (
                        StartTime < 15 * 60 * 60
                        && currEndTime >= 20.5 * 60 * 60
                        && Breaks.FindIndex(
                            x => x.breakTime >= 20 * 60
                            && x.startTime >= 16.5 * 60 * 60
                            && x.startTime <= 20.5 * 60 * 60 - x.breakTime
                        ) == -1
                    ) return false;
                }

                return true;
            }

            return true;
        }

        internal bool isFeasible(int currentEndTime, bool final)
        {
            if (PrevBlockId == 1 && final)
            {
                int x = 0;
            }
            int duration = currentEndTime - StartTime;
            if (duration > Config.MAX_DRIVE_TIME && Type != DutyType.Broken) return false;
            if (Type == DutyType.Early && currentEndTime > 16.5 * 60 * 60) return false;
            if (Type == DutyType.Day && currentEndTime > 18.25 * 60 * 60) return false;
            if (Type == DutyType.Late && (StartTime < 13 * 60 * 60 || currentEndTime > 26.5 * 60 * 60)) return false;
            if (Type == DutyType.Night && (duration > 7 * 60 * 60 || StartTime > 24 * 60 * 60)) return false;
            if (Type == DutyType.Between && StartTime > 13 * 60 * 60) return false;
            if (Type == DutyType.Broken && (StartTime < 5.5 * 60 * 60 || currentEndTime > 19.5 * 60 * 60)) return false;

            if (final)
            {
                if (Type == DutyType.Broken)
                {
                    if (Idles.Count == 0) return false; // not broken

                    // Check if total driving time is okay after removal of longest idle block
                    (int startTime, int idleTime) = Idles.MaxBy(x => x.idleTime);
                    if (duration - idleTime >= 9 * 60 * 60) return false;
                }
                if (Type == DutyType.Day && !(currentEndTime >= 16.5 * 60 * 60 && currentEndTime <= 18.25 * 60 * 60)) return false;
                if (Type == DutyType.Between && (Idles.FindIndex(x => x.idleTime > 1.5 * 60 * 60) == -1 || currentEndTime < 18.25 * 60 * 60)) return false;
                if (Type == DutyType.Night && currentEndTime < 26.5 * 60 * 60) return false;

            }
            if (!breaksFeasible(currentEndTime, final)) return false;

            return true;
        }

        public override string ToString()
        {
            List<int> coveredIds = [];
            for (int i = 0; i < CoveredBlockIds.Length; i++) if (CoveredBlockIds[i]) coveredIds.Add(i);
            return $"BS: {String.Join(", ", coveredIds)} C: {Cost}";
        }
    }

    internal class CSPLabeling
    {
        private List<Block> blocks;
        private GRBModel model;

        List<List<BlockArc?>> adjFull = [];
        List<List<BlockArc>> adj = [];

        List<double> rcBlocks = [];
        double rcExcessiveLength;
        double rcLlimitedAvgLength;
        double rcMaxBroken;
        double rcMaxBetween;

        internal CSPLabeling(List<Block> blocks, GRBModel model)
        {
            this.blocks = blocks;
            this.model = model;
        }

        private void reset()
        {
            var constrs = model.GetConstrs();
            rcBlocks = Enumerable.Range(0, blocks.Count).Select(_ => 0.0).ToList();
            if (model.Status != GRB.Status.INFEASIBLE)
            {
                for (int i = 0; i < blocks.Count; i++) rcBlocks[i] = constrs[i].Pi;
                rcExcessiveLength = model.GetConstrByName("no_excessive_length").Pi;
                rcLlimitedAvgLength = model.GetConstrByName("limited_average_length").Pi;
                rcMaxBroken = model.GetConstrByName("max_broken").Pi;
                rcMaxBetween = model.GetConstrByName("max_between").Pi;
            }
            adjFull = Enumerable.Range(0, blocks.Count + 2).Select(x => new List<BlockArc?>()).ToList();
            adj = Enumerable.Range(0, blocks.Count + 2).Select(x => new List<BlockArc>()).ToList();

            // TODO: add block arcs for walking
            for (int blockIndex1 = 0; blockIndex1 < blocks.Count; blockIndex1++)
            {
                Block block1 = blocks[blockIndex1];
                for (int blockIndex2 = 0; blockIndex2 < blocks.Count; blockIndex2++)
                {
                    Block block2 = blocks[blockIndex2];

                    // Determine whether or not it is feasible to string to arcs together
                    // If so: what actually happens during this time. 

                    // For now; only allow transfer if already at same location
                    // Based on times, determine whether its idle / break / whatever. 
                    BlockArc? arc = null;

                    if (block1.EndTime <= block2.StartTime && block1.EndLocation == block2.StartLocation)
                    {
                        // Arc will be formed; need to fill in details.
                        int idleTime = block2.StartTime - block1.EndTime;
                        int breakTime = 0;
                        int bruttoNettoTime = 0;
                        int travelTime = 0;

                        int nettoBreakTime = block1.EndLocation.BreakAllowed
                            ? idleTime - block1.EndLocation.BrutoNetto
                            : 0;

                        if (nettoBreakTime > Config.CR_MIN_BREAK_TIME && nettoBreakTime < Config.CR_MAX_BREAK_TIME)
                        {
                            breakTime = idleTime - block1.EndLocation.BrutoNetto;
                            bruttoNettoTime = block1.EndLocation.BrutoNetto;
                            idleTime = 0;
                        }

                        if (idleTime < Config.CR_MAX_IDLE_TIME) // either "short" idle or break
                        {
                            arc = new()
                            {
                                FromBlock = block1,
                                ToBlock = block2,
                                IdleTime = idleTime,
                                BreakTime = breakTime,
                                BruttoNettoTime = bruttoNettoTime,
                                TravelTime = travelTime,
                                ReducedCost = rcBlocks[blockIndex2],
                            };
                        }
                    }

                    if (arc != null) adj[blockIndex1].Add(arc);
                    adjFull[blockIndex1].Add(arc);
                }
            }

            // TODO: better depot handling
            for (int blockIndex = 0; blockIndex < blocks.Count; blockIndex++)
            {
                Block block = blocks[blockIndex];
                BlockArc? start = block.StartLocation.IsDepot ? new BlockArc()
                {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = 0,
                } : null;
                if (start != null) adj[^2].Add(start);
                adjFull[^2].Add(start);
                adjFull[blockIndex].Add(null); // cannot go back to depot

                BlockArc? end = block.EndLocation.IsDepot ? new BlockArc()
                {
                    ToBlock = block,
                    IdleTime = 0,
                    BreakTime = 0,
                    BruttoNettoTime = 0,
                    TravelTime = 0,
                } : null;
                if (end != null) adj[block.Index].Add(end);
                adjFull[block.Index].Add(end);
            }
        }


        internal List<(double reducedCosts, CrewDuty crewDuty)> generateShortestPath()
        {
            reset();

            List<List<CSPLabel>> allLabels = [.. Enumerable.Range(0, blocks.Count + 2).Select(x => new List<CSPLabel>())];
            List<List<CSPLabel>> activeLabels = [.. Enumerable.Range(0, blocks.Count + 2).Select(x => new List<CSPLabel>())];

            void addLabel(CSPLabel l, int index)
            {
                allLabels[index].Add(l);
                activeLabels[index].Add(l);
            }

            for (int i = 0; i < (int)DutyType.Count; i++)
            {
                DutyType dt = (DutyType)i;

                double baseCost = Config.CR_SHIFT_COST;
                if (dt != DutyType.Broken) baseCost -= rcMaxBroken;
                else baseCost += Config.CR_BROKEN_SHIFT_COST;
                if (dt != DutyType.Between) baseCost -= rcMaxBetween;

                // Start by setting out all possible duty types from the depot start
                for (int targetBlockId = 0; targetBlockId < blocks.Count; targetBlockId++)
                {
                    BlockArc? arc = adjFull[^2][targetBlockId];
                    if (arc == null) continue;
                    CSPLabel l = new()
                    {
                        StartTime = blocks[targetBlockId].StartTime,
                        CoveredBlockIds = new BitArray(blocks.Count, false),
                        PrevLabelId = -1,
                        PrevBlockId = -1,
                        Type = dt,
                        Cost = baseCost - rcBlocks[targetBlockId],
                        Breaks = [],
                        Idles = [],
                    };

                    l.CoveredBlockIds.Set(targetBlockId, true);

                    if (l.isFeasible(blocks[i].EndTime, false))
                        addLabel(l, arc.ToBlock!.Index);
                }
            }

            while (activeLabels.Where((x, i) => i <= blocks.Count).Sum(x => x.Count) > 0)
            {
                // Find node with active label
                CSPLabel? currentLabel = null;
                int currentNodeIndex = -1;
                for (int i = 0; i < activeLabels.Count - 1; i++)
                {
                    if (activeLabels[i].Count > 0)
                    {
                        currentLabel = activeLabels[i][^1];
                        activeLabels[i].RemoveAt(activeLabels[i].Count - 1);
                        currentNodeIndex = i;
                        break;
                    }
                }
                if (currentLabel == null) throw new InvalidOperationException("heh");

                // Expand node if possible 
                for (int targetBlockIndex = 0; targetBlockIndex < blocks.Count + 2; targetBlockIndex++)
                {
                    if (targetBlockIndex == blocks.Count) continue;

                    BlockArc? arc = adjFull[currentNodeIndex][targetBlockIndex];
                    if (arc == null) continue;

                    BitArray newCoverage = new(currentLabel.CoveredBlockIds);
                    if (targetBlockIndex < blocks.Count) newCoverage.Set(targetBlockIndex, true);

                    CSPLabel newLabel = new()
                    {
                        CoveredBlockIds = newCoverage,
                        Breaks = arc.BreakTime == 0 ? currentLabel.Breaks : [.. currentLabel.Breaks, (arc.FromBlock!.EndTime, arc.BreakTime)],
                        Idles = arc.IdleTime == 0 ? currentLabel.Idles : [.. currentLabel.Idles, (arc.FromBlock!.EndTime, arc.IdleTime)],
                        PrevBlockId = currentNodeIndex,
                        Cost = currentLabel.Cost + (targetBlockIndex < blocks.Count ? -rcBlocks[targetBlockIndex] : 0),
                        PrevLabelId = currentLabel.Id,
                        Type = currentLabel.Type,
                        StartTime = currentLabel.StartTime,
                    };

                    if (!newLabel.isFeasible(blocks[targetBlockIndex >= blocks.Count ? currentNodeIndex : targetBlockIndex].EndTime, false)) continue;

                    // Dominated if feasible and less coverage (i think)
                    else addLabel(newLabel, targetBlockIndex);
                }
            }


            // Walk back through in order to get the minimum costs
            var feasibleEnds = allLabels[^1]
                .Where(x => x.isFeasible(blocks[x.PrevBlockId].EndTime, true) && x.Cost < 0)
                .OrderBy(x => x.Cost).ToList();

            List<CSPLabel> validTargets = [];
            if (!Config.CSP_LB_ATTEMPT_DISJOINT) validTargets = feasibleEnds.Take(Config.CSP_LB_MAX_COLS).ToList();
            else
            {
                BitArray currCover = new(blocks.Count);
                for (int i = 0; i < feasibleEnds.Count; i++)
                {
                    BitArray ba = new(currCover);
                    if (ba.And(feasibleEnds[i].CoveredBlockIds).HasAnySet()) continue;
                    validTargets.Add(feasibleEnds[i]);
                    currCover.Or(feasibleEnds[i].CoveredBlockIds);
                }
            }

            List<(double reducedCost, CrewDuty duty)> duties = new();
            foreach (var cl in validTargets)
            {
                var currLabel = cl; //non-readonly
                double reducedCost = currLabel.Cost;
                DutyType type = currLabel.Type;
                List<(int currBlockId, CSPLabel label)> usedLabels = [(blocks.Count + 1, currLabel)];
                while (currLabel.PrevBlockId != -1)
                {
                    int currBlockId = currLabel.PrevBlockId;
                    currLabel = allLabels[currLabel.PrevBlockId].Find(x => x.Id == currLabel.PrevLabelId)!;
                    usedLabels.Add((currBlockId, currLabel));
                }

                usedLabels.Reverse();

                List<CrewDutyElement> cdes = [new CDEBlock(blocks[usedLabels[0].currBlockId])];
                for (int j = 0; j < usedLabels.Count - 1; j++)
                {
                    var labelFrom = usedLabels[j];
                    var labelTo = usedLabels[j + 1];
                    BlockArc arc = adjFull[labelFrom.currBlockId][labelTo.currBlockId] ?? throw new InvalidDataException("hier gaat nog iets mis");
                    if (arc.IdleTime > 0)
                    {
                        cdes.Add(new CDEIdle(arc.FromBlock!.EndTime, arc.FromBlock.EndTime + arc.IdleTime, arc.FromBlock.EndLocation, arc.ToBlock!.StartLocation));
                    }
                    if (arc.BreakTime > 0)
                    {
                        cdes.Add(new CDEBreak(arc.FromBlock!.EndTime, arc.FromBlock.EndTime + arc.BreakTime + arc.BruttoNettoTime, arc.FromBlock.EndLocation, arc.ToBlock!.StartLocation));
                    }
                    if (labelTo.currBlockId < blocks.Count)
                    {
                        cdes.Add(new CDEBlock(blocks[labelTo.currBlockId]));
                    }
                }

                CrewDuty cd = new CrewDuty(cdes)
                {
                    Type = type,
                };
                duties.Add((reducedCost, cd));
                //Console.WriteLine($"New duty {cd} {Formatting.Time.HHMMSS(cd.Elements[0].StartTime)} - {Formatting.Time.HHMMSS(cd.Elements[^1].EndTime)}");
            }

            return duties;
        }
    }
}