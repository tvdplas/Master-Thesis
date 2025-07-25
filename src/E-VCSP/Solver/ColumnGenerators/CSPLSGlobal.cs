﻿using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators {
    public class CSPLSNode {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion
        /// <summary>
        /// Crew duty elementrepresented in node
        /// </summary>
        public required CrewDutyElement CDE;

        /// <summary>
        /// Previous crew duty node
        /// </summary>
        public CSPLSNode? Prev;
        /// <summary>
        /// Next crew duty node
        /// </summary>
        public CSPLSNode? Next;

        public CSPLSNode() {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString() {
            return $"{DEBUG_INDEX}: {CDE}";
        }

        #region find
        public CSPLSNode? FindBefore(Func<CSPLSNode, bool> predicate, bool first) {
            CSPLSNode? curr = Prev;
            CSPLSNode? res = null;
            while (curr != null) {
                if (predicate(curr)) {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Prev;
            }
            return res;
        }

        public CSPLSNode? FindFirstBefore(Func<CSPLSNode, bool> predicate) => FindBefore(predicate, true);
        public CSPLSNode? FindLastBefore(Func<CSPLSNode, bool> predicate) => FindBefore(predicate, false);

        public CSPLSNode? FindAfter(Func<CSPLSNode, bool> predicate, bool first) {
            CSPLSNode? curr = Next;
            CSPLSNode? res = null;
            while (curr != null) {
                if (predicate(curr)) {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Next;
            }
            return res;
        }
        public CSPLSNode? FindFirstAfter(Func<CSPLSNode, bool> predicate) => FindAfter(predicate, true);
        public CSPLSNode? FindLastAfter(Func<CSPLSNode, bool> predicate) => FindAfter(predicate, false);

        #endregion

        #region count

        public int Count(bool head) {
            CSPLSNode? curr = this;
            int c = 0;
            while (curr != null) {
                c++;
                curr = head ? curr.Next : curr.Next;
            }
            return c;
        }
        public int HeadCount() => Count(true);
        public int TailCount() => Count(false);

        #endregion

        #region modify
        /// <summary>
        /// Removes <paramref name="count"/> nodes after <c>this</c>. Does <b>not</b> update structure of removed nodes.
        /// </summary>
        /// <param name="count">Amount of nodes to remove</param>
        /// <returns>List of removed nodes</returns>
        public List<CSPLSNode> RemoveAfter(int count) {
            List<CSPLSNode> removedNodes = new(count);
            CSPLSNode? curr = Next;
            for (int i = 0; i < count && curr != null; i++) {
                removedNodes.Add(curr);
                curr = curr.Next;
            }
            this.Next = curr;
            if (curr != null) curr.Prev = this;

            return removedNodes;
        }

        public CSPLSNode AddAfter(CSPLSNode node) {
            ArgumentNullException.ThrowIfNull(node);

            if (CDE.EndTime != node.CDE.StartTime)
                throw new InvalidOperationException("Vehicle task is not continuous");

            // Set next correctly
            if (Next != null) {
                Next.Prev = node;
            }

            // Update ordering
            node.Next = Next;
            node.Prev = this;
            this.Next = node;

            return node;
        }

        public CSPLSNode AddAfter(CrewDutyElement cde) {
            return AddAfter(new CSPLSNode() {
                CDE = cde,
            });
        }
        #endregion
    }

    internal class CSPLSDuty {
        public required DutyType Type = DutyType.Single;
        // Invariant: Dont include the sign on / off elements at start/end of shift
        public required CSPLSNode? head;
        public CSPLSNode? tail => head?.FindLastAfter(_ => true) ?? head;

        public List<int> CoveredBlocks() {
            List<int> res = [];
            CSPLSNode? curr = head;
            while (curr != null) {
                if (curr.CDE.Type == CrewDutyElementType.Block)
                    res.Add(((CDEBlock)curr.CDE).Block.Index);
                curr = curr.Next;
            }
            return res;
        }

        private (int minStartTime, int maxStartTime, int minEndTime, int maxEndTime)[] DutyTypeTimeframes =
        [
            (int.MinValue, int.MaxValue, int.MinValue, (int)(16.5 * 60 * 60)), // Early
            (int.MinValue, int.MaxValue, (int)(16.5 * 60 * 60), (int)(18.25 * 60 * 60)), // Day
            (13 * 60 * 60, int.MaxValue, int.MinValue, (int)(26.5 * 60 * 60)), // Late
            (int.MinValue, 24 * 60 * 60, (int)(26.5 * 60 * 60), int.MaxValue), // Night
            (int.MinValue, 13 * 60 * 60, (int)(18.25 * 60 * 60), int.MaxValue), // Between
            ((int)(5.5 * 60 * 60), int.MaxValue, int.MinValue, (int)(19.5 * 60 * 60)), // Broken
        ];

        private int[] DutyTypeMaxDurations = [
            Config.CR_MAX_SHIFT_LENGTH, // Early
            Config.CR_MAX_SHIFT_LENGTH, // Day
            Config.CR_MAX_SHIFT_LENGTH, // Late
            7 * 60 * 60, // Night
            Config.CR_MAX_SHIFT_LENGTH, // Between
            Config.CR_MAX_SHIFT_LENGTH, // Broken
        ];

        public bool IsFeasible(DutyType dt, bool final = false) {
            if (final) {
                if (!head!.CDE.StartLocation.CrewHub || !tail!.CDE.EndLocation.CrewHub) return false;
            }

            // Check start / end times
            var timeframe = DutyTypeTimeframes[(int)dt];
            if (timeframe.minStartTime > head!.CDE.StartTime || timeframe.maxStartTime < head.CDE.StartTime) return false;
            if (timeframe.minEndTime > tail!.CDE.EndTime || timeframe.maxEndTime < tail.CDE.EndTime) return false;

            // Check duration
            if (PaidDuration > DutyTypeMaxDurations[(int)dt]) return false;

            List<(int startTime, int duration)> breaks = [];

            // Check total break time + max driving duration
            int lastBreak = head!.CDE.StartTime;
            (int startTime, int duration) longIdle = (-1, 0);
            bool longIdleUsed = false;

            if (dt == DutyType.Broken) {
                var longestIdle = head.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Idle && x.CDE.EndTime - x.CDE.StartTime > Config.CR_MIN_LONG_IDLE_TIME);
                if (longestIdle != null) longIdle = (longestIdle.CDE.StartTime, longestIdle.CDE.EndTime - longestIdle.CDE.EndTime);
            }

            CSPLSNode? curr = head;
            while (curr != null) {
                CrewDutyElement cde = curr.CDE;
                if (cde.Type != CrewDutyElementType.Break) {
                    curr = curr.Next;
                    continue;
                }

                int breakStartTime = cde.StartTime;
                int breakDuration = cde.StartTime - cde.EndTime - cde.StartLocation.BrutoNetto;
                breaks.Add((breakStartTime, breakDuration));

                if (!longIdleUsed && longIdle.startTime != -1 && longIdle.startTime < breakStartTime) {
                    // First check time traveled before big idle, then update last break time accordingly
                    if (longIdle.startTime - lastBreak > Config.MAX_STEERING_TIME) return false;
                    lastBreak = longIdle.startTime + longIdle.duration;
                    longIdleUsed = true;
                }

                // Too long between breaks
                if (cde.StartTime - lastBreak > Config.MAX_STEERING_TIME) {
                    return false;
                }
                lastBreak = cde.EndTime;

                curr = curr.Next;
            }

            if (!longIdleUsed && longIdle.startTime != -1 && longIdle.startTime > lastBreak) {
                // Check if there is a long idle after the last break
                if (longIdle.startTime - lastBreak > Config.MAX_STEERING_TIME) return false;
                lastBreak = longIdle.startTime + longIdle.duration;
                longIdleUsed = true;
            }
            if (tail.CDE.EndTime - lastBreak > Config.MAX_STEERING_TIME) return false;

            if (longIdle.startTime != -1) {
                // Before / after long idle is handled seperately
                int beforeDuration = longIdle.startTime - head.CDE.StartTime;
                int afterDuration = tail.CDE.EndTime - (longIdle.startTime + longIdle.duration);
                var beforeBreaks = breaks.Where(x => x.startTime + x.duration < head.CDE.StartTime + beforeDuration).ToList();
                var afterBreaks = breaks.Where(x => x.startTime + x.duration > longIdle.startTime + longIdle.duration).ToList();

                if (beforeDuration > 4 * 60 * 60 && beforeDuration <= 5.5 * 60 * 60) {
                    if (beforeBreaks.Count() == 0) return false;
                }
                else if (beforeDuration >= 5.5 * 60 * 60) {
                    if (beforeBreaks.Sum(x => x.duration) < 40 * 60 || beforeBreaks.FindIndex(x => x.duration >= 20 * 60) == -1) return false;
                }
                if (afterDuration > 4 * 60 * 60 && afterDuration <= 5.5 * 60 * 60) {
                    if (afterBreaks.Count() == 0) return false;
                }
                else if (afterDuration >= 5.5 * 60 * 60) {
                    if (afterBreaks.Sum(x => x.duration) < 40 * 60 || afterBreaks.FindIndex(x => x.duration >= 20 * 60) == -1) return false;
                }
            }
            else {
                // Min 1 break of 15 minutes
                if (PaidDuration > 4 * 60 * 60 && PaidDuration <= 5.5 * 60 * 60 && breaks.Count == 0) return false;
                // Min 40 min break, min 1 of length >= 20min
                else if (PaidDuration >= 5.5 * 60 * 60 && (breaks.Sum(x => x.duration) < 40 * 60 || !breaks.Any(x => x.duration > 20 * 60))) return false;
            }

            // Additional check for late duties with dinner break
            if (dt == DutyType.Late && head.CDE.StartTime <= 15 * 60 * 60 && tail.CDE.EndTime >= 20.5 * 60 * 60) {
                if (breaks.FindIndex(
                    x => x.duration >= 20 * 60
                    && x.startTime >= 16.5 * 60 * 60
                    && x.startTime <= 20.5 * 60 * 60 - x.duration
                ) == -1) return false;
            }

            return true;
        }

        /// <summary>
        /// Determines which duty types are feasible for this duty.
        /// </summary>
        /// <returns></returns>
        public List<DutyType> FeasibleTypes() {
            if (head == null) {
                return [DutyType.Single]; // No elements, no duty    
            }

            List<DutyType> feasibleTypes = [];
            for (int i = 0; i < (int)DutyType.Count; i++) {
                DutyType type = (DutyType)i;
                if (IsFeasible(type)) {
                    feasibleTypes.Add(type);
                }
            }

            return feasibleTypes;
        }

        public int PaidDuration {
            get {
                if (head == null || tail == null) return 0;
                int baseDuration = tail.CDE.EndTime - head.CDE.StartTime;
                if (Type == DutyType.Broken) {
                    var longestIdle = head.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Idle && x.CDE.EndTime - x.CDE.StartTime > Config.CR_MIN_LONG_IDLE_TIME);

                    if (longestIdle == null) return baseDuration;
                    return baseDuration - (longestIdle.CDE.EndTime - longestIdle.CDE.StartTime);
                }

                return baseDuration;
            }
        }

        public double Cost {
            get {
                double cost = BaseCost;
                cost += PaidDuration / (60.0 * 60.0) * Config.CR_HOURLY_COST;
                if (head == null || tail == null) return cost;

                if (!head.CDE.StartLocation.CrewHub) cost += Config.CSP_LS_G_CREWHUB_PENALTY;
                if (!tail.CDE.EndLocation.CrewHub) cost += Config.CSP_LS_G_CREWHUB_PENALTY;

                return cost;
            }
        }

        public double BaseCost {
            get {
                double cost = Config.CR_SHIFT_COST;
                if (Type == DutyType.Single) cost += Config.CR_SINGLE_SHIFT_COST;
                if (Type == DutyType.Broken) cost += Config.CR_BROKEN_SHIFT_COST;
                return cost;
            }
        }

        public CrewDuty? ToCrewDuty() {
            if (!IsFeasible(Type, true)) return null;

            List<CrewDutyElement> cdes = [];
            var curr = head;
            while (curr != null) {
                cdes.Add(curr.CDE);
                curr = curr.Next;
            }
            return new CrewDuty(cdes) {
                Type = Type
            };
        }
    }

    internal class CSPLSGlobal : CrewColumnGen {
        private List<CSPLSDuty> duties = [];

        private Random random;
        private double T;
        private double alpha;
        private int Q;

        internal CSPLSGlobal(GRBModel model, CrewSolutionState css) : base(model, css) {
            this.random = LSShared.random;
        }

        private void reset() {
            duties = [];

            // Reset params
            T = Config.CSP_LS_G_STARTING_T;
            alpha = Config.CSP_LS_G_COOLING_RATE;
            Q = (int)Math.Round(-Config.CSP_LS_G_ITERATIONS / (Math.Log(Config.CSP_LS_G_STARTING_T / Config.CSP_LS_G_ENDING_T) / Math.Log(alpha)));

            // Reset duties to unit
            for (int i = 0; i < css.Blocks.Count; i++) {
                Block b = css.Blocks[i];

                CrewDutyElement cde = new CDEBlock(b);
                CSPLSNode head = new CSPLSNode() { CDE = cde };

                duties.Add(new() {
                    head = head,
                    Type = DutyType.Single
                });

                duties[^1].Type = duties[^1].FeasibleTypes()[0];
            }
        }

        /// <summary>
        /// Generates the linking node of two Blocks if it exists.
        /// </summary>
        /// <param name="prev">CDE must be block</param>
        /// <param name="next">CDE must be block</param>
        private (bool feasible, CSPLSNode? link) linkBlocks(CSPLSNode? prev, CSPLSNode? next) {
            if (prev == null || next == null) return (true, null);
            BlockArc? arc = css.AdjFull[((CDEBlock)prev.CDE).Block.Index][((CDEBlock)next.CDE).Block.Index];
            if (arc == null) return (false, null);
            if (prev == next) throw new Exception("Dat hoort nie");

            CrewDutyElement? cde = null;
            if (arc.BreakTime > 0) {
                cde = new CDEBreak(prev.CDE.EndTime, next.CDE.StartTime, prev.CDE.EndLocation, next.CDE.StartLocation);
            }
            else {
                cde = new CDEIdle(prev.CDE.EndTime, next.CDE.StartTime, prev.CDE.EndLocation, next.CDE.StartLocation);
            }

            return (true, new CSPLSNode() {
                CDE = cde,
                Prev = prev,
                Next = next,
            });
        }

        private LSOpResult moveRange(int maxRangeSize = 10000000) {
            // Select two random duties, select a range from the second to insert into the first
            // At most <c>maxRangeSize</c> elements are moved

            int dutyIndex1 = random.Next(duties.Count);
            int dutyIndex2 = random.Next(duties.Count);
            while (dutyIndex2 == dutyIndex1) dutyIndex2 = random.Next(duties.Count);

            CSPLSDuty duty1 = duties[dutyIndex1];
            CSPLSDuty duty2 = duties[dutyIndex2];

            List<CSPLSNode> blocksIn2 = [];
            CSPLSNode? curr = duty2.head;

            while (curr != null) {
                if (curr.CDE.Type == CrewDutyElementType.Block) {
                    blocksIn2.Add(curr);
                }
                curr = curr.Next;
            }

            // Select a random range of css.instance.Blocks to move  
            int startIndex = random.Next(blocksIn2.Count);
            int endIndex = startIndex + random.Next(0, Math.Min(maxRangeSize, blocksIn2.Count - startIndex) - 1);
            CSPLSNode rangeStart = blocksIn2[startIndex];
            CSPLSNode rangeEnd = blocksIn2[endIndex];

            // Next, find the connecting css.instance.Blocks in duty1 (if they exist)
            // In the meantime, check if a blo in duty1 overlaps with the selected range in duty2
            CSPLSNode? prevIn1 = null, nextIn1 = null;
            curr = duty1.head;
            while (curr != null) {
                if (curr.CDE.Type == CrewDutyElementType.Block) {
                    // Check for overlap
                    if (curr.CDE.StartTime <= rangeEnd.CDE.EndTime && curr.CDE.EndTime >= rangeStart.CDE.StartTime) {
                        // Selected range overlaps with css.instance.Blocks in duty1
                        return LSOpResult.Invalid;
                    }

                    // Update prev / next
                    if (curr.CDE.EndTime <= rangeStart.CDE.StartTime) prevIn1 = curr;
                    if (rangeEnd.CDE.EndTime <= curr.CDE.StartTime && nextIn1 == null) nextIn1 = curr;
                }
                curr = curr.Next;
            }

            // Do the same for duty2;
            CSPLSNode? prevIn2 = rangeStart.Prev?.Prev, nextIn2 = rangeEnd.Next?.Next;

            // We can now start to generate connecting arcs: we have the current setup known:
            // Duty1: ... -> prevIn1? -> Idle/Break -> nextIn1? -> ...
            // Duty2: ... -> prevIn2? -> idle? -> rangeStart -> ... -> rangeEnd -> idle? -> nextIn2? -> ...
            // Target is as follows: 
            // Duty1: ... -> prevIn1? -> Idle?/Break? -> rangeStart -> ... -> rangeEnd -> Idle?/Break? -> nextIn1? -> ...
            // Duty2: ... -> prevIn2? -> idle? -> nextIn2? -> ...

            (bool feasible1, CSPLSNode? linkPrev1ToRange) = linkBlocks(prevIn1, rangeStart);
            (bool feasible2, CSPLSNode? linkNext1ToRange) = linkBlocks(rangeEnd, nextIn1);
            (bool feasible3, CSPLSNode? linkPrev2ToNext2) = linkBlocks(prevIn2, nextIn2);
            if (!feasible1 || !feasible2 || !feasible3) {
                // At least one of the arcs does not exist
                return LSOpResult.Invalid;
            }

            // Store copy of current links + heads
            CSPLSNode? prevHead1 = duty1.head;
            CSPLSNode? prevHead2 = duty2.head;
            CSPLSNode? prevIn1Next = prevIn1?.Next;
            CSPLSNode? nextIn1Prev = nextIn1?.Prev;
            CSPLSNode? prevIn2Next = prevIn2?.Next;
            CSPLSNode? nextIn2Prev = nextIn2?.Prev;
            CSPLSNode? rangeStartPrev = rangeStart.Prev;
            CSPLSNode? rangeEndNext = rangeEnd.Next;

            // Calculate costs before move
            double costDiff = 0;
            costDiff -= duty1.Cost;
            costDiff -= duty2.Cost;

            if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

            // Change links, reevaluate heads
            if (prevIn1 != null) prevIn1.Next = linkPrev1ToRange;
            if (nextIn1 != null) nextIn1.Prev = linkNext1ToRange;
            if (prevIn2 != null) prevIn2.Next = linkPrev2ToNext2;
            if (nextIn2 != null) nextIn2.Prev = linkPrev2ToNext2;

            rangeStart.Prev = linkPrev1ToRange;
            rangeEnd.Next = linkNext1ToRange;

            duty1.head = rangeStart.FindLastBefore(_ => true) ?? rangeStart;
            duty2.head = prevIn2?.FindLastBefore(_ => true) ?? prevIn2 ?? nextIn2;

            // Determine costs / feasibility
            var feasibleTypes1 = duty1.FeasibleTypes();
            var feasibleTypes2 = duty2.FeasibleTypes();

            // Check costs
            costDiff += duty1.Cost;
            costDiff += duty2.Cost;

            // If the second duty is now empty, remove it
            if (duty2.head == null) costDiff -= duty2.BaseCost;

            if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

            // If infeasible or cost change not acceptable, revert  
            if (feasibleTypes1.Count == 0 || feasibleTypes2.Count == 0 || !accept(costDiff)) {
                // Revert changes
                if (prevIn1 != null) prevIn1.Next = prevIn1Next;
                if (nextIn1 != null) nextIn1.Prev = nextIn1Prev;
                if (prevIn2 != null) prevIn2.Next = prevIn2Next;
                if (nextIn2 != null) nextIn2.Prev = nextIn2Prev;
                rangeStart.Prev = rangeStartPrev;
                rangeEnd.Next = rangeEndNext;
                duty1.head = prevHead1;
                duty2.head = prevHead2;

                if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

                return (feasibleTypes1.Count == 0 || feasibleTypes2.Count == 0) ? LSOpResult.Invalid : LSOpResult.Decline;
            }

            if (duty2.head == null) duties.Remove(duty2);

            return costDiff >= 0 ? LSOpResult.Accept : LSOpResult.Improvement;
        }

        private LSOpResult moveSingle() => moveRange(maxRangeSize: 1);

        private LSOpResult swapTails() {
            int dutyIndex1 = random.Next(duties.Count);
            int dutyIndex2 = random.Next(duties.Count);
            while (dutyIndex2 == dutyIndex1) dutyIndex2 = random.Next(duties.Count);

            CSPLSDuty duty1 = duties[dutyIndex1];
            CSPLSDuty duty2 = duties[dutyIndex2];

            // Select a random time to swap the two tails
            List<int> interestingTimes = [];
            List<(int, int)> startEndTimes = [];

            CSPLSNode? curr = duty1.head;
            while (curr != null) {
                startEndTimes.Add((curr.CDE.StartTime, curr.CDE.EndTime));
                curr = curr.Next;
            }
            curr = duty2.head;
            while (curr != null) {
                startEndTimes.Add((curr.CDE.StartTime, curr.CDE.EndTime));
                curr = curr.Next;
            }

            // Filter out times which would cause 2 css.instance.Blocks to overlap when swap occurs
            for (int i = 0; i < startEndTimes.Count; i++) {
                (int s, int e) = startEndTimes[i];
                bool startValid = true;
                bool endValid = true;
                for (int j = 0; j < startEndTimes.Count && (startValid || endValid); j++) {
                    (int sb, int eb) = startEndTimes[j];
                    if (sb < s && s < eb) startValid = false;
                    if (sb < e && e < eb) endValid = false;
                }

                if (startValid) interestingTimes.Add(s);
                if (endValid) interestingTimes.Add(e);
            }
            interestingTimes.Sort();
            int time = interestingTimes[random.Next(interestingTimes.Count - 1) + 1];

            // Always follows this pattern:
            // bl -> i/br -> bl -> i/br -> bl

            // Find s.t
            // d1: bl -> i/br -> ... -> lastRemainingBlock1 |t -> i/br -> bl
            // d2: bl -> i/br -> ... -> lastRemainingBlock2 |t -> i/br -> bl
            // Aka: find block preceding time shift, if it doesn't exist use all

            CSPLSNode? lastRemainingBlock1 = duty1.head!.FindLastAfter(x => x.CDE.Type == CrewDutyElementType.Block && x.CDE.EndTime <= time);
            if (lastRemainingBlock1 == null && duty1.head.CDE.Type == CrewDutyElementType.Block && duty1.head.CDE.EndTime <= time) {
                // Check if head can be used; if not, entire duty is shifted
                lastRemainingBlock1 = duty1.head;
            }
            CSPLSNode? lastRemainingBlock2 = duty2.head!.FindLastAfter(x => x.CDE.Type == CrewDutyElementType.Block && x.CDE.EndTime <= time);
            if (lastRemainingBlock2 == null && duty2.head.CDE.Type == CrewDutyElementType.Block && duty2.head.CDE.EndTime <= time) {
                // Check if head can be used; if not, entire duty is shifted
                lastRemainingBlock2 = duty2.head;
            }

            CSPLSNode? firstBlockInTail1 = duty1.head!.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Block && x.CDE.StartTime >= time);
            if (duty1.head.CDE.Type == CrewDutyElementType.Block && duty1.head.CDE.StartTime >= time) {
                // Head takes priority as it is always earlier
                firstBlockInTail1 = duty1.head;
            }
            CSPLSNode? firstBlockInTail2 = duty2.head!.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Block && x.CDE.StartTime >= time);
            if (duty2.head.CDE.Type == CrewDutyElementType.Block && duty2.head.CDE.StartTime >= time) {
                // Head takes priority as it is always earlier
                firstBlockInTail2 = duty2.head;
            }

            // Attempt to link the two tails; if null at either end, note that it just means we will be starting from a crewhub there
            (bool link1To2Feasible, CSPLSNode? link1To2) = linkBlocks(lastRemainingBlock1, firstBlockInTail2);
            (bool link2To1Feasible, CSPLSNode? link2To1) = linkBlocks(lastRemainingBlock2, firstBlockInTail1);
            if (!link1To2Feasible || !link2To1Feasible) {
                // Arc was not feasible, so we cannot swap the tails
                return LSOpResult.Invalid;
            }

            // Calculate costs before swap
            double costDiff = 0;
            costDiff -= duty1.Cost;
            costDiff -= duty2.Cost;

            // Check if tail swap is even feasible
            CSPLSNode? lastRemainingBlock1Next = lastRemainingBlock1?.Next;
            CSPLSNode? lastRemainingBlock2Next = lastRemainingBlock2?.Next;
            CSPLSNode? firstBlockInTail1Prev = firstBlockInTail1?.Prev;
            CSPLSNode? firstBlockInTail2Prev = firstBlockInTail2?.Prev;
            CSPLSNode? prevHead1 = duty1.head;
            CSPLSNode? prevHead2 = duty2.head;

            if (lastRemainingBlock1 != null) lastRemainingBlock1.Next = link1To2;
            if (lastRemainingBlock2 != null) lastRemainingBlock2.Next = link2To1;
            if (firstBlockInTail1 != null) firstBlockInTail1.Prev = link2To1;
            if (firstBlockInTail2 != null) firstBlockInTail2.Prev = link1To2;
            duty1.head = lastRemainingBlock1?.FindLastBefore(_ => true)
                ?? lastRemainingBlock1
                ?? firstBlockInTail2?.FindLastBefore(_ => true)
                ?? firstBlockInTail2;
            duty2.head = lastRemainingBlock2?.FindLastBefore(_ => true)
                ?? lastRemainingBlock2
                ?? firstBlockInTail1?.FindLastBefore(_ => true)
                ?? firstBlockInTail1;

            // Check feasibility
            var feasibleTypes1 = duty1.FeasibleTypes();
            var feasibleTypes2 = duty2.FeasibleTypes();

            // Check costs
            costDiff += duty1.Cost;
            costDiff += duty2.Cost;

            // If one of the two duties is empty, we can also additionally remove extra cost
            if (duty1.head == null) costDiff -= duty1.BaseCost;
            if (duty2.head == null) costDiff -= duty2.BaseCost;

            if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

            // If either of the changes is not feasible or costs are not accepted, revert
            if (feasibleTypes1.Count == 0 || feasibleTypes2.Count == 0 || !accept(costDiff)) {
                if (lastRemainingBlock1 != null) lastRemainingBlock1.Next = lastRemainingBlock1Next;
                if (lastRemainingBlock2 != null) lastRemainingBlock2.Next = lastRemainingBlock2Next;
                if (firstBlockInTail1 != null) firstBlockInTail1.Prev = firstBlockInTail1Prev;
                if (firstBlockInTail2 != null) firstBlockInTail2.Prev = firstBlockInTail2Prev;
                duty1.head = prevHead1;
                duty2.head = prevHead2;

                if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

                return LSOpResult.Invalid;
            }

            // Finalize operation 
            duty1.Type = feasibleTypes1[0];
            duty2.Type = feasibleTypes2[0];

            if (duties.Sum(x => x.CoveredBlocks().Count) != css.Blocks.Count) throw new Exception();

            if (duty1.head == null) duties.Remove(duty1);
            if (duty2.head == null) duties.Remove(duty2);

            return costDiff >= 0 ? LSOpResult.Accept : LSOpResult.Improvement;
        }

        private bool accept(double deltaScore) {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        public override List<(double reducedCost, CrewDuty crewDuty)> GenerateDuties() {
            reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (() => moveRange(), Config.CSP_LS_G_MOVE_RANGE),
                (moveSingle, Config.CSP_LS_G_MOVE_SINGLE),
                (swapTails, Config.CSP_LS_G_SWAP_TAILS),
            ];
            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;

            List<int> resCounts = Enumerable.Range(0, (int)LSOpResult.Count).Select(x => 0).ToList();
            while (currIts < Config.CSP_LS_G_ITERATIONS) {
                currIts++;
                if (currIts % Q == 0) {
                    T *= alpha;
                }
                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
                resCounts[(int)res]++;
            }

            //Console.WriteLine("Op results: " + string.Join('\t', resCounts.Select(x => x.ToString())));
            //Console.WriteLine($"Total duties/css.instance.Blocks covered: {duties.Count} / {duties.Sum(x => x.CoveredBlocks().Count)}");
            var fs = duties.Select(x => (-1.0, x.ToCrewDuty())).Where(x => x.Item2 != null).ToList();
            //Console.WriteLine($"Feasible duties/css.instance.Blocks covered: {fs.Count} / {fs.Sum(x => x.Item2!.Covers.Count)}");
            return fs;
        }
    }
}