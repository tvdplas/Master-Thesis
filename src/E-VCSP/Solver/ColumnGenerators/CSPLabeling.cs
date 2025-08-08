using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;
using System.Collections;

namespace E_VCSP.Solver.ColumnGenerators {
    internal struct CSPLabel {
        private static int ID_COUNTER = 0;
        internal readonly int Id = -1;
        public CSPLabel() {
            Id = ID_COUNTER++;
        }

        internal int PrevLabelId = -1;
        internal int PrevBlockId = -1;
        internal DutyType Type;
        internal double Cost = 0;
        internal int StartTime = 0;
        internal required BitArray CoveredBlockIds;
        internal List<(int startTime, int breakTime)> Breaks = new();
        internal (int startTime, int longIdleTime) Idle = (-1, 0); // Only set once for split

        internal bool breaksFeasible(int currEndTime, bool final) {
            // Check driving time between breaks
            int lastBreak = StartTime;
            bool longIdleUsed = false;

            for (int i = 0; i < Breaks.Count; i++) {
                (int st, int bt) = Breaks[i];
                if (!longIdleUsed && Idle.startTime != -1 && Idle.startTime < st) {
                    // First check time traveled before big idle, then update last break time accordingly
                    if (Idle.startTime - lastBreak > Config.MAX_STEERING_TIME) return false;
                    lastBreak = Idle.startTime + Idle.longIdleTime;
                    longIdleUsed = true;
                }

                // Too long between breaks
                if (st - lastBreak > Config.MAX_STEERING_TIME) return false;
                lastBreak = st + bt;
            }
            if (!longIdleUsed && Idle.startTime != -1 && Idle.startTime < currEndTime) {
                // First check time traveled before big idle, then update last break time accordingly
                if (Idle.startTime - lastBreak > Config.MAX_STEERING_TIME) return false;
                lastBreak = Idle.startTime + Idle.longIdleTime;
                longIdleUsed = true;
            }
            if (currEndTime - lastBreak > Config.MAX_STEERING_TIME) return false;

            if (final) {
                // Reset for break time during idle, so seperately handled
                if (Type == DutyType.Broken) {
                    // Two parts: before break and after idle
                    int beforeDuration = Idle.startTime - StartTime;
                    int afterDuration = currEndTime - (Idle.startTime + Idle.longIdleTime);
                    List<(int startTime, int breakTime)> beforeBreaks = [], afterBreaks = [];
                    foreach (var br in Breaks) {
                        if (br.startTime + br.breakTime < StartTime + beforeDuration) beforeBreaks.Add(br);
                        if (br.startTime + br.breakTime > Idle.startTime + Idle.longIdleTime) afterBreaks.Add(br);
                    }

                    if (beforeDuration > 4 * 60 * 60 && beforeDuration <= 5.5 * 60 * 60) {
                        if (beforeBreaks.Count() == 0) return false;
                    }
                    else if (beforeDuration >= 5.5 * 60 * 60) {
                        if (beforeBreaks.Sum(x => x.breakTime) < 40 * 60 || beforeBreaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;
                    }
                    if (afterDuration > 4 * 60 * 60 && afterDuration <= 5.5 * 60 * 60) {
                        if (afterBreaks.Count() == 0) return false;
                    }
                    else if (afterDuration >= 5.5 * 60 * 60) {
                        if (afterBreaks.Sum(x => x.breakTime) < 40 * 60 || afterBreaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;
                    }
                }
                // Continouos shift
                else {
                    int duration = currEndTime - StartTime;
                    if (duration > 4 * 60 * 60 && duration <= 5.5 * 60 * 60) {
                        // At least 1 of >= 15 min
                        if (Breaks.Count == 0) return false;
                    }
                    else if (duration > 5 * 60 * 60) {
                        // >= 40 min, at least one >= 20 min
                        if (Breaks.Sum(x => x.breakTime) < 40 * 60 || Breaks.FindIndex(x => x.breakTime >= 20 * 60) == -1) return false;
                    }
                }

                // Extra check for times for late
                if (Type == DutyType.Late) {
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

        internal bool isFeasible(int currentEndTime, bool final) {
            int duration = currentEndTime - StartTime;
            // Max shift length (normal / broken)
            if (duration > Config.CR_MAX_SHIFT_LENGTH && Type != DutyType.Broken) return false;
            if (duration > Config.CR_MAX_SHIFT_LENGTH + Config.CR_MAX_LONG_IDLE_TIME) return false;
            if (Idle.startTime != -1 && duration - Idle.longIdleTime > Config.CR_MAX_SHIFT_LENGTH) return false;
            // No long idles in non-broken shifts
            if (Type != DutyType.Broken && Idle.startTime != -1)
                return false;
            // Shift start / end times
            if (Type == DutyType.Early && currentEndTime > 16.5 * 60 * 60) return false;
            if (Type == DutyType.Day && currentEndTime > 18.25 * 60 * 60) return false;
            if (Type == DutyType.Late && (StartTime < 13 * 60 * 60 || currentEndTime > 26.5 * 60 * 60)) return false;
            if (Type == DutyType.Night && (duration > 7 * 60 * 60 || StartTime > 24 * 60 * 60)) return false;
            if (Type == DutyType.Between && StartTime > 13 * 60 * 60) return false;
            if (Type == DutyType.Broken && (StartTime < 5.5 * 60 * 60 || currentEndTime > 19.5 * 60 * 60)) return false;

            if (final) {
                if (Type == DutyType.Broken) {
                    // not broken
                    if (Idle.startTime == -1)
                        return false;
                    // Too long
                    if (duration - Idle.longIdleTime > Config.CR_MAX_SHIFT_LENGTH)
                        return false;
                }
                // End times in range
                if (Type == DutyType.Day && !(currentEndTime >= 16.5 * 60 * 60 && currentEndTime <= 18.25 * 60 * 60)) return false;
                if (Type == DutyType.Between && (currentEndTime < 18.25 * 60 * 60)) return false;
                if (Type == DutyType.Night && currentEndTime < 26.5 * 60 * 60) return false;

            }
            if (!breaksFeasible(currentEndTime, final)) return false;

            return true;
        }

        public override string ToString() {
            List<int> coveredIds = [];
            for (int i = 0; i < CoveredBlockIds.Length; i++) if (CoveredBlockIds[i]) coveredIds.Add(i);
            return $"BS: {String.Join(", ", coveredIds)} C: {Cost}";
        }
    }

    internal class CSPLabeling : CrewColumnGen {
        List<List<CSPLabel>> allLabels = [];
        List<List<CSPLabel>> activeLabels = [];
        List<bool> blockedBlock = [];

        internal CSPLabeling(CrewSolutionState css) : base(css) { }

        private void reset() {
            blockedBlock = [.. css.BlockCount.Select(x => x == 0), false, false]; // Only use blocks which are active
            allLabels = [.. Enumerable.Range(0, css.Blocks.Count + 2).Select(x => new List<CSPLabel>())];
            activeLabels = [.. Enumerable.Range(0, css.Blocks.Count + 2).Select(x => new List<CSPLabel>())];
        }

        private void runLabeling() {
            List<(List<CSPLabel>, int nodeIndex)> activeLabelsWithContent = [];
            List<int> indexInWithContent = Enumerable.Range(0, css.Blocks.Count + 2).Select(x => -1).ToList();

            void addLabel(CSPLabel l, int index) {
                if (index <= css.Blocks.Count && indexInWithContent[index] == -1) {
                    // add it to with content
                    indexInWithContent[index] = activeLabelsWithContent.Count;
                    activeLabelsWithContent.Add((activeLabels[index], index));
                }

                allLabels[index].Add(l);
                activeLabels[index].Add(l);
            }

            for (int i = 0; i < (int)DutyType.Count; i++) {
                DutyType dt = (DutyType)i;
                int rho_broken = dt == DutyType.Broken ? 1 : 0;
                int rho_between = dt == DutyType.Between ? 1 : 0;

                double baseCost = Config.CR_SHIFT_COST + rho_broken * Config.CR_BROKEN_SHIFT_COST;
                baseCost -= maxBrokenDualCost * (Config.CR_MAX_BROKEN_SHIFTS - rho_broken);
                baseCost -= maxBetweenDualCost * (Config.CR_MAX_BETWEEN_SHIFTS - rho_between);

                // Start by setting out all possible duty types from the depot start
                for (int targetBlockIndex = 0; targetBlockIndex < css.Blocks.Count; targetBlockIndex++) {
                    BlockArc? arc = css.AdjFull[^2][targetBlockIndex];
                    if (arc == null || blockedBlock[targetBlockIndex]) continue;
                    CSPLabel l = new() {
                        StartTime = css.Blocks[targetBlockIndex].StartTime - arc.TravelTime, // signon
                        CoveredBlockIds = new BitArray(css.Blocks.Count, false),
                        PrevLabelId = -1,
                        PrevBlockId = -1,
                        Type = dt,
                        Cost = baseCost - blockDualCosts[targetBlockIndex],
                        Breaks = [],
                    };

                    l.CoveredBlockIds.Set(targetBlockIndex, true);

                    if (l.isFeasible(css.Blocks[i].EndTime, false))
                        addLabel(l, arc.ToBlock!.Index);
                }
            }

            while (allLabels[^1].Count < Config.CSP_LB_MAX_LABELS_IN_END && activeLabelsWithContent.Count > 0) {
                // Find node with active label
                int xxx = LSShared.random.Next(0, activeLabelsWithContent.Count);
                (List<CSPLabel> targetLabels, int currentNodeIndex) = activeLabelsWithContent[xxx];
                CSPLabel currentLabel = targetLabels[^1];
                targetLabels.RemoveAt(targetLabels.Count - 1);

                if (targetLabels.Count == 0) {
                    // Move last item in activeLabels to position which is now empty
                    activeLabelsWithContent[xxx] = activeLabelsWithContent[^1];
                    indexInWithContent[activeLabelsWithContent[xxx].nodeIndex] = xxx;

                    // Reset the now empty label collection, remove it 
                    indexInWithContent[currentNodeIndex] = -1;
                    activeLabelsWithContent.RemoveAt(activeLabelsWithContent.Count - 1);
                }


                // Expand node if possible 
                for (int targetBlockIndex = 0; targetBlockIndex < css.Blocks.Count + 2; targetBlockIndex++) {
                    if (targetBlockIndex == css.Blocks.Count || blockedBlock[targetBlockIndex]) continue;

                    BlockArc? arc = css.AdjFull[currentNodeIndex][targetBlockIndex];
                    if (arc == null) continue;

                    // Cant use long idle arcs / already used in shift.
                    if (arc.Type == BlockArcType.LongIdle && (currentLabel.Type != DutyType.Broken || currentLabel.Idle.startTime != -1)) {
                        continue;
                    }

                    BitArray newCoverage = new(currentLabel.CoveredBlockIds);
                    if (targetBlockIndex < css.Blocks.Count) newCoverage.Set(targetBlockIndex, true);

                    int prevEndTime = currentNodeIndex >= css.Blocks.Count ? currentLabel.StartTime : css.Blocks[currentNodeIndex].EndTime;
                    int currentEndTime = targetBlockIndex >= css.Blocks.Count ? arc.TravelTime + css.Blocks[currentNodeIndex].EndTime : css.Blocks[targetBlockIndex].EndTime;

                    int timeDiff = currentEndTime - prevEndTime;

                    int addedTime = arc.Type != BlockArcType.LongIdle ? timeDiff : timeDiff - arc.IdleTime;

                    double costFromTime = addedTime / (60.0 * 60.0) * Config.CR_HOURLY_COST;
                    double costFromRc = targetBlockIndex < css.Blocks.Count ? -blockDualCosts[targetBlockIndex] : 0;

                    CSPLabel newLabel = new() {
                        CoveredBlockIds = newCoverage,
                        Breaks = arc.BreakTime == 0 ? currentLabel.Breaks : [.. currentLabel.Breaks, (arc.FromBlock!.EndTime, arc.BreakTime)],
                        Idle = arc.Type == BlockArcType.LongIdle ? (arc.FromBlock!.EndTime, arc.IdleTime) : currentLabel.Idle,
                        PrevBlockId = currentNodeIndex,
                        Cost = currentLabel.Cost + costFromTime + costFromRc,
                        PrevLabelId = currentLabel.Id,
                        Type = currentLabel.Type,
                        StartTime = currentLabel.StartTime,
                    };

                    if (!newLabel.isFeasible(currentEndTime, false)) continue;

                    // Dominated if feasible and less coverage (i think)
                    else addLabel(newLabel, targetBlockIndex);
                }
            }

            // Adjust costs to include finalized duration
            for (int i = 0; i < allLabels[^1].Count; i++) {
                CSPLabel l = allLabels[^1][i];
                int duration = css.Blocks[l.PrevBlockId].EndTime - l.StartTime;
                if (l.Type == DutyType.Broken && l.Idle.startTime != -1) duration -= l.Idle.longIdleTime;

                l.Cost -= maxAvgDurationDualCost * ((duration / (double)Config.CR_TARGET_SHIFT_LENGTH) - 1);
                l.Cost -= maxLongDualCost * (Config.CR_MAX_OVER_LONG_DUTY - (duration > Config.CR_TARGET_SHIFT_LENGTH ? 1 : 0));
            }
        }

        private List<(double reducedCost, CrewDuty crewDuty)> extractDuties(string source) {
            // Walk back through in order to get the minimum costs
            var feasibleEnds = allLabels[^1]
                .Where(x => x.isFeasible(css.Blocks[x.PrevBlockId].EndTime, true) && x.Cost < 0)
                .OrderBy(x => x.Cost).ToList();

            List<CSPLabel> validTargets = [];
            if (!Config.CSP_LB_ATTEMPT_DISJOINT) validTargets = feasibleEnds.Take(Config.CSP_LB_MAX_COLS).ToList();
            else {
                BitArray currCover = new(css.Blocks.Count);
                // Priority on disjoint labels
                for (int i = 0; i < feasibleEnds.Count; i++) {
                    BitArray ba = new(currCover);
                    if (ba.And(feasibleEnds[i].CoveredBlockIds).HasAnySet()) continue;
                    validTargets.Add(feasibleEnds[i]);
                    currCover.Or(feasibleEnds[i].CoveredBlockIds);
                }
                // Add extra labels if available
                for (int i = 0; i < feasibleEnds.Count && validTargets.Count < Config.CSP_LB_MAX_COLS; i++) {
                    if (validTargets.Contains(feasibleEnds[i])) continue;
                    validTargets.Add(feasibleEnds[i]);
                }
            }

            List<(double reducedCost, CrewDuty duty)> duties = new();
            foreach (var cl in validTargets) {
                var currLabel = cl; //non-readonly
                double reducedCost = currLabel.Cost;
                DutyType type = currLabel.Type;
                List<(int currBlockId, CSPLabel label)> usedLabels = [(css.Blocks.Count + 1, currLabel)];
                while (currLabel.PrevBlockId != -1) {
                    int currBlockId = currLabel.PrevBlockId;
                    currLabel = allLabels[currLabel.PrevBlockId].Find(x => x.Id == currLabel.PrevLabelId)!;
                    usedLabels.Add((currBlockId, currLabel));
                }
                usedLabels.Reverse();

                BlockArc signOnArc = css.AdjFull[^2][usedLabels[0].currBlockId] ?? throw new InvalidDataException("SignOn arc not found");
                int firstBlockTime = css.Blocks[usedLabels[0].currBlockId].StartTime;

                List<CrewDutyElement> cdes = [
                    new CDESignOnOff(firstBlockTime - signOnArc.TotalTime, firstBlockTime, css.Blocks[usedLabels[0].currBlockId].StartLocation),
                    new CDEBlock(css.Blocks[usedLabels[0].currBlockId])
                ];
                for (int j = 0; j < usedLabels.Count - 1; j++) {
                    var labelFrom = usedLabels[j];
                    var labelTo = usedLabels[j + 1];
                    BlockArc arc = css.AdjFull[labelFrom.currBlockId][labelTo.currBlockId] ?? throw new InvalidDataException("hier gaat nog iets mis");
                    if (arc.Type == BlockArcType.LongIdle) {
                        Block from = arc.FromBlock!;
                        Location loc = from.EndLocation;

                        int startTime = from!.EndTime;
                        int nettoIdleTime = arc.IdleTime - loc.SignOffTime - loc.SignOnTime;

                        cdes.Add(new CDESignOnOff(startTime, startTime + loc.SignOffTime, loc));
                        cdes.Add(new CDEIdle(startTime + loc.SignOffTime, startTime + loc.SignOffTime + nettoIdleTime, loc, loc));
                        cdes.Add(new CDESignOnOff(startTime + loc.SignOffTime + nettoIdleTime, startTime + loc.SignOffTime + nettoIdleTime + loc.SignOnTime, loc));
                        cdes.Add(new CDEBlock(css.Blocks[labelTo.currBlockId]));
                    }
                    else if (arc.Type == BlockArcType.SignOnOff) {
                        cdes.Add(new CDESignOnOff(css.Blocks[labelFrom.currBlockId].EndTime, css.Blocks[labelFrom.currBlockId].EndTime + css.Blocks[labelFrom.currBlockId].EndLocation.SignOffTime, css.Blocks[labelFrom.currBlockId].EndLocation));
                    }
                    else {
                        if (arc.IdleTime > 0) {
                            cdes.Add(new CDEIdle(arc.FromBlock!.EndTime, arc.FromBlock.EndTime + arc.IdleTime, arc.FromBlock.EndLocation, arc.ToBlock!.StartLocation));
                        }
                        if (arc.BreakTime > 0) {
                            cdes.Add(new CDEBreak(arc.FromBlock!.EndTime, arc.FromBlock.EndTime + arc.BreakTime + arc.BruttoNettoTime, arc.FromBlock.EndLocation, arc.ToBlock!.StartLocation));
                        }
                        if (labelTo.currBlockId < css.Blocks.Count) {
                            cdes.Add(new CDEBlock(css.Blocks[labelTo.currBlockId]));
                        }
                    }
                }

                CrewDuty cd = new CrewDuty(cdes) {
                    Type = type,
                };
                duties.Add((reducedCost, cd));
            }

            return duties;
        }

        public override List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties() {
            reset();

            runLabeling();
            List<(double reducedCost, CrewDuty crewDuty)> primaryTasks = extractDuties("primary");
            List<(double reducedCost, CrewDuty crewDuty)> secondaryTasks = [];

            for (int i = 0; i < Math.Min(primaryTasks.Count, Config.CSP_LB_SEC_COL_COUNT); i++) {
                var baseTask = primaryTasks[i];

                for (int j = 1; j < (Config.CSP_LB_SEC_COL_ATTEMPTS + 1); j++) {
                    reset();
                    for (int k = 0; k < (int)((double)j * baseTask.crewDuty.BlockIndexCover.Count / (Config.CSP_LB_SEC_COL_ATTEMPTS + 1)); k++) {
                        blockedBlock[baseTask.crewDuty.BlockIndexCover[k]] = true;
                    }
                    runLabeling();
                    secondaryTasks.AddRange(extractDuties("secondary"));
                }
            }

            primaryTasks.AddRange(secondaryTasks);
            return primaryTasks;
        }
    }
}