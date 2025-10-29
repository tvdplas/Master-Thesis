using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;

namespace E_VCSP.Solver.ColumnGenerators {
    public struct CSPLSCosts {
        public double BaseCosts;
        public double PenaltyCosts;

        public readonly double OverallCosts => BaseCosts + PenaltyCosts;
    }

    public class CSPLSNode {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion
        /// <summary>
        /// Crew duty elementrepresented in node
        /// </summary>
        public required CrewDutyElement CDE;
        public (double cost, int duration, Block? block, BlockArc? arc) altStart;
        public (double cost, int duration, Block? block, BlockArc? arc) altEnd;

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
        // Invariant: Dont include the sign on / off elements at start/end of shift
        public required CSPLSNode? head;
        public CSPLSNode? tail => head?.FindLastAfter(_ => true) ?? head;

        private (int minStartTime, int maxStartTime, int minEndTime, int maxEndTime)[] DutyTypeTimeframes =
        [
            (int.MinValue, int.MaxValue, int.MinValue, (int)(16.5 * 60 * 60)), // Early
            (int.MinValue, int.MaxValue, (int)(16.5 * 60 * 60), (int)(18.25 * 60 * 60)), // Day
            (13 * 60 * 60, int.MaxValue, int.MinValue, (int)(26.5 * 60 * 60)), // Late
            (int.MinValue, 24 * 60 * 60, (int)(26.5 * 60 * 60), int.MaxValue), // Night
            (int.MinValue, 13 * 60 * 60, (int)(18.25 * 60 * 60), int.MaxValue), // Between
            ((int)(5.5 * 60 * 60), int.MaxValue, int.MinValue, (int)(19.5 * 60 * 60)), // Broken

            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
            //(int.MinValue, int.MaxValue, int.MinValue, int.MaxValue), // Early
        ];

        private int[] DutyTypeMaxDurations = [
            Constants.CR_MAX_SHIFT_LENGTH, // Early
            Constants.CR_MAX_SHIFT_LENGTH, // Day
            Constants.CR_MAX_SHIFT_LENGTH, // Late
            7 * 60 * 60, // Night
            Constants.CR_MAX_SHIFT_LENGTH, // Between
            Constants.CR_MAX_SHIFT_LENGTH, // Broken
        ];

        public (bool feasible, double penaltyCosts) CheckFeasibility(List<double> blockDualCosts, DutyType dt, double penaltyMultiplier = 1) {
            if (head == null) return (true, 0);

            int paidDuration = PaidDuration(dt);
            int numUncoveredBlocks = 0;
            CSPLSNode? curr = head;
            while (curr != null) {
                if (curr.CDE.Type == CrewDutyElementType.Block && blockDualCosts[((CDEBlock)curr.CDE).Block.Index] > Config.CR_SINGLE_SHIFT_COST) numUncoveredBlocks++;
                curr = curr.Next;
            }

            double penalty = 0;
            bool feasible = true;
            void applyPenalty(double p) {
                feasible = false;
                penalty += p * penaltyMultiplier * paidDuration;
            }

            if (!head!.CDE.StartLocation.CrewBase && head.altStart.block == null)
                applyPenalty(Config.CSP_LS_G_CREWHUB_PENALTY);
            if (!tail!.CDE.EndLocation.CrewBase && tail.altEnd.block == null)
                applyPenalty(Config.CSP_LS_G_CREWHUB_PENALTY);

            bool useAltStart = !head.CDE.StartLocation.CrewBase && head.altStart.block != null;
            bool useAltEnd = !tail!.CDE.EndLocation.CrewBase && tail.altEnd.block != null;

            // Check start / end times
            int startTime = useAltStart
                ? head.altStart.block!.StartTime
                : head.CDE.StartTime;
            int endTime = useAltEnd
                ? tail.altEnd.block!.EndTime
                : tail.CDE.EndTime;

            var timeframe = DutyTypeTimeframes[(int)dt];
            int startTimeDeviation = 0;
            if (timeframe.minStartTime > startTime)
                startTimeDeviation = timeframe.minStartTime - startTime;
            if (startTime > timeframe.maxStartTime)
                startTimeDeviation = Math.Max(startTimeDeviation, startTime - timeframe.maxStartTime);
            if (startTimeDeviation > 0) applyPenalty(startTimeDeviation * Config.CSP_LS_G_TIME_PENALTY);

            int endTimeDeviation = 0;
            if (timeframe.minEndTime > endTime)
                endTimeDeviation = timeframe.minEndTime - endTime;
            if (endTime > timeframe.maxEndTime)
                endTimeDeviation = Math.Max(endTimeDeviation, endTime - timeframe.maxEndTime);
            if (endTimeDeviation > 0) applyPenalty(endTimeDeviation * Config.CSP_LS_G_TIME_PENALTY);

            // Check duration
            int durationDeviation = Math.Max(paidDuration - DutyTypeMaxDurations[(int)dt], 0);
            if (durationDeviation > 0) applyPenalty(durationDeviation * Config.CSP_LS_G_TIME_PENALTY);

            List<(int startTime, int duration)> breaks = [];

            // Check total break time + max driving duration
            int lastBreak = startTime;
            (int startTime, int duration) longIdle = (-1, 0);
            bool longIdleUsed = false;
            if (dt == DutyType.Broken) {
                (int startTime, int duration) longestIdle = (-1, -1);

                if (useAltStart && head.altStart.arc!.Type == BlockArcType.LongIdle && head.altStart.arc.TotalTime > longestIdle.duration) {
                    longestIdle = (head.altStart.block!.EndTime, head.altStart.arc.TotalTime);
                }
                if (useAltEnd && tail!.altEnd.arc!.Type == BlockArcType.LongIdle && tail.altEnd.arc.TotalTime > longestIdle.duration) {
                    longestIdle = (tail.altEnd.block!.StartTime - tail.altEnd.arc.TotalTime, tail.altEnd.arc.TotalTime);
                }

                var longestIdleInDuty = head.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Idle && x.CDE.EndTime - x.CDE.StartTime > Config.CR_MIN_LONG_IDLE_TIME);
                if (longestIdleInDuty != null && longestIdleInDuty.CDE.EndTime - longestIdleInDuty.CDE.StartTime > longestIdle.duration) {
                    longIdle = (longestIdleInDuty.CDE.StartTime, longestIdleInDuty.CDE.EndTime - longestIdleInDuty.CDE.StartTime);
                }
            }


            if (useAltStart && head.altStart.arc!.Type == BlockArcType.Break) {
                int breakStartTime = head.altStart.block!.EndTime;
                int breakDuration = head.altStart.arc.TotalTime - head.altStart.duration - head.altStart.block.EndLocation.BrutoNetto;
                breaks.Add((breakStartTime, breakDuration));
                // Too long between breaks
                if (breakStartTime - lastBreak > Constants.MAX_STEERING_TIME) {
                    applyPenalty((breakStartTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                }
                lastBreak = breakStartTime + breakDuration;
            }

            curr = head;
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
                    if (longIdle.startTime - lastBreak > Constants.MAX_STEERING_TIME)
                        applyPenalty((longIdle.startTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                    lastBreak = longIdle.startTime + longIdle.duration;
                    longIdleUsed = true;
                }

                // Too long between breaks
                if (cde.StartTime - lastBreak > Constants.MAX_STEERING_TIME) {
                    applyPenalty((cde.StartTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                }
                lastBreak = cde.EndTime;

                curr = curr.Next;
            }

            if (useAltEnd && tail!.altEnd.arc!.Type == BlockArcType.Break) {
                int breakStartTime = tail.CDE.EndTime;
                int breakDuration = tail.altEnd.arc.TotalTime - tail.CDE.EndLocation.BrutoNetto;
                breaks.Add((breakStartTime, breakDuration));
                if (!longIdleUsed && longIdle.startTime != -1 && longIdle.startTime < breakStartTime) {
                    // First check time traveled before big idle, then update last break time accordingly
                    if (longIdle.startTime - lastBreak > Constants.MAX_STEERING_TIME)
                        applyPenalty((longIdle.startTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                    lastBreak = longIdle.startTime + longIdle.duration;
                    longIdleUsed = true;
                }
                // Too long between breaks
                if (breakStartTime - lastBreak > Constants.MAX_STEERING_TIME) {
                    applyPenalty((breakStartTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                }
                lastBreak = breakStartTime + breakDuration;
            }

            if (!longIdleUsed && longIdle.startTime != -1 && longIdle.startTime > lastBreak) {
                // Check if there is a long idle after the last break
                if (longIdle.startTime - lastBreak > Constants.MAX_STEERING_TIME)
                    applyPenalty((longIdle.startTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);
                lastBreak = longIdle.startTime + longIdle.duration;
                longIdleUsed = true;
            }
            if (tail.CDE.EndTime - lastBreak > Constants.MAX_STEERING_TIME)
                applyPenalty((tail.CDE.EndTime - lastBreak - Constants.MAX_STEERING_TIME) * Config.CSP_LS_G_STEER_PENALTY);


            if (longIdle.startTime != -1) {
                // Before / after long idle is handled seperately
                int beforeDuration = longIdle.startTime - head.CDE.StartTime;
                int afterDuration = tail.CDE.EndTime - (longIdle.startTime + longIdle.duration);
                var beforeBreaks = breaks.Where(x => x.startTime + x.duration < head.CDE.StartTime + beforeDuration).ToList();
                var afterBreaks = breaks.Where(x => x.startTime + x.duration > longIdle.startTime + longIdle.duration).ToList();

                if (beforeDuration > 4 * 60 * 60 && beforeDuration <= 5.5 * 60 * 60) {
                    if (beforeBreaks.Count() == 0) applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
                }
                else if (beforeDuration >= 5.5 * 60 * 60) {
                    if (beforeBreaks.Sum(x => x.duration) < 40 * 60 || beforeBreaks.FindIndex(x => x.duration >= 20 * 60) == -1)
                        applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
                }
                if (afterDuration > 4 * 60 * 60 && afterDuration <= 5.5 * 60 * 60) {
                    if (afterBreaks.Count() == 0) applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
                }
                else if (afterDuration >= 5.5 * 60 * 60) {
                    if (afterBreaks.Sum(x => x.duration) < 40 * 60 || afterBreaks.FindIndex(x => x.duration >= 20 * 60) == -1)
                        applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
                }
            }
            else {
                // Min 1 break of 15 minutes
                if (paidDuration > 4 * 60 * 60 && paidDuration <= 5.5 * 60 * 60 && breaks.Count == 0)
                    applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
                // Min 40 min break, min 1 of length >= 20min
                else if (paidDuration >= 5.5 * 60 * 60 && (breaks.Sum(x => x.duration) < 40 * 60 || !breaks.Any(x => x.duration > 20 * 60)))
                    applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
            }

            // Additional check for late duties with dinner break
            if (dt == DutyType.Late && head.CDE.StartTime <= 15 * 60 * 60 && tail.CDE.EndTime >= 20.5 * 60 * 60) {
                if (breaks.FindIndex(
                    x => x.duration >= 20 * 60
                    && x.startTime >= 16.5 * 60 * 60
                    && x.startTime <= 20.5 * 60 * 60 - x.duration
                ) == -1) applyPenalty(Config.CSP_LS_G_BREAK_PENALTY);
            }

            //if (feasible) {
            // subtract the block reduced costs from the penalty
            //curr = head;
            //while (curr != null) {
            //    if (curr.CDE.Type == CrewDutyElementType.Block) {
            //        penalty -= blockDualCosts[((CDEBlock)curr.CDE).Block.Index];
            //    }
            //    curr = curr.Next;
            //}
            //}

            return (feasible, penalty);
        }

        public (DutyType dt, bool feasible, double cost, double penaltyCost) MinTypeCost(List<double> blockDualCosts, double penaltyMultiplier = 1) {
            (DutyType, bool, double cost, double penaltyCost) min = (DutyType.Single, false, double.PositiveInfinity, 0);

            for (int i = 0; i < (int)DutyType.Count; i++) {
                DutyType type = (DutyType)i;
                double cost = Cost(type);
                var feasibilityRes = CheckFeasibility(blockDualCosts, type, penaltyMultiplier);

                if (cost + feasibilityRes.penaltyCosts < min.cost + min.penaltyCost)
                    min = (type, feasibilityRes.feasible, cost, feasibilityRes.penaltyCosts);
            }

            return min;
        }

        public int PaidDuration(DutyType dt) {
            if (head == null || tail == null) return 0;
            int baseDuration = tail.CDE.EndTime - head.CDE.StartTime;

            if (head.CDE.StartLocation.CrewBase)
                baseDuration += head.CDE.StartLocation.SignOnTime;
            else if (head.altStart.block != null)
                baseDuration += head.altStart.duration + head.altStart.block.StartLocation.SignOnTime;

            if (tail.CDE.EndLocation.CrewBase)
                baseDuration += tail.CDE.EndLocation.SignOffTime;
            else if (tail.altEnd.block != null)
                baseDuration += tail.altEnd.duration + tail.altEnd.block.EndLocation.SignOffTime;

            if (dt == DutyType.Broken) {
                var longestIdle = head.FindFirstAfter(x => x.CDE.Type == CrewDutyElementType.Idle && x.CDE.EndTime - x.CDE.StartTime > Config.CR_MIN_LONG_IDLE_TIME);

                if (longestIdle == null) return baseDuration;
                return baseDuration - (longestIdle.CDE.EndTime - longestIdle.CDE.StartTime);
            }

            return baseDuration;
        }

        public double Cost(DutyType dt) {
            double cost = BaseCost(dt);
            cost += PaidDuration(dt) / (60.0 * 60.0) * Constants.CR_HOURLY_COST;
            return cost;
        }

        public double BaseCost(DutyType dt) {
            double cost = Config.CR_SHIFT_COST;
            if (dt == DutyType.Broken) cost += Constants.CR_BROKEN_SHIFT_COST;
            return cost;
        }

        public List<CrewDuty> ToCrewDuty(List<double> blockDualCosts) {
            // If no type exists which can be feasibly used, return
            var minType = MinTypeCost(blockDualCosts, 10000);
            if (!minType.feasible) {
                // attempt to cover every block in the best way possible
                List<CSPLSNode> blocks = [];
                var c = head;
                while (c != null) {
                    if (c.CDE.Type == CrewDutyElementType.Block) blocks.Add(c);
                    c = c.Next;
                }

                List<CrewDuty> cds = [];
                foreach (var b in blocks) {
                    if ((!b.CDE.StartLocation.CrewBase && b!.altStart.block == null)
                        || (!b.CDE.EndLocation.CrewBase && b!.altEnd.block == null)) continue;

                    List<CrewDutyElement> elements = [];

                    // create crew duty from alt start / end
                    if (b.CDE.StartLocation.CrewBase) {
                        elements.Add(new CDESignOnOff(b.CDE.StartTime - b.CDE.StartLocation.SignOnTime, b.CDE.StartTime, b.CDE.StartLocation));
                    }
                    else {
                        elements.Add(new CDESignOnOff(b.altStart.block.StartTime - b.altStart.block.StartLocation.SignOnTime, b.altStart.block.StartTime, b.altStart.block.StartLocation));
                        elements.Add(new CDEBlock(b.altStart.block));
                        if (b.altStart.block.EndLocation.BreakAllowed) {
                            elements.Add(new CDEBreak(b.altStart.block.EndTime, b.CDE.StartTime, b.altStart.block.EndLocation, b.CDE.StartLocation));
                        }
                        else {
                            elements.Add(new CDEIdle(b.altStart.block.EndTime, b.CDE.StartTime, b.altStart.block.EndLocation, b.CDE.StartLocation));
                        }
                    }
                    elements.Add(b.CDE);
                    if (b.CDE.EndLocation.CrewBase) {
                        elements.Add(new CDESignOnOff(b.CDE.EndTime, b.CDE.EndTime + b.CDE.EndLocation.SignOnTime, b.CDE.EndLocation));
                    }
                    else {
                        if (b.altEnd.block.StartLocation.BreakAllowed) {
                            elements.Add(new CDEBreak(b.CDE.EndTime, b.altEnd.block.StartTime, b.CDE.EndLocation, b.altEnd.block.StartLocation));
                        }
                        else {
                            elements.Add(new CDEIdle(b.CDE.EndTime, b.altEnd.block.StartTime, b.CDE.EndLocation, b.altEnd.block.StartLocation));
                        }
                        elements.Add(new CDEBlock(b.altEnd.block));
                        elements.Add(new CDESignOnOff(b.altEnd.block.EndTime, b.altEnd.block.EndTime + b.altEnd.block.EndLocation.SignOffTime, b.altEnd.block.EndLocation));
                    }

                    CrewDuty cd = new CrewDuty(elements) {
                        Type = DutyType.Single,
                    };

                    for (int i = 0; i < (int)DutyType.Count; i++) {
                        if (cd.Elements[0].StartTime >= DutyTypeTimeframes[i].minStartTime &&
                            cd.Elements[0].StartTime <= DutyTypeTimeframes[i].maxStartTime &&
                            cd.Elements[^1].EndTime >= DutyTypeTimeframes[i].minEndTime &&
                            cd.Elements[^1].EndTime <= DutyTypeTimeframes[i].maxEndTime &&
                            cd.Elements[^1].EndTime - cd.Elements[0].StartTime <= DutyTypeMaxDurations[i]) {
                            cd.Type = (DutyType)i;
                        }
                    }
                    cds.Add(cd);
                }
                return cds;
            }

            // TODO: alternatieve arcs toevoegen

            var curr = head;
            List<CrewDutyElement> cdes = [];
            if (!curr.CDE.StartLocation.CrewBase) {
                cdes.Add(new CDEBlock(curr.altStart.block));
                if (curr.altStart.block.EndLocation.BreakAllowed) {
                    cdes.Add(new CDEBreak(curr.altStart.block.EndTime, curr.CDE.StartTime, curr.altStart.block.StartLocation, curr.CDE.StartLocation));
                }
                else {
                    cdes.Add(new CDEIdle(curr.altStart.block.EndTime, curr.CDE.StartTime, curr.altStart.block.StartLocation, curr.CDE.StartLocation));
                }
            }
            while (curr != null) {
                cdes.Add(curr.CDE);
                curr = curr.Next;
            }
            curr = head.FindLastAfter(_ => true) ?? head;
            if (!curr.CDE.EndLocation.CrewBase) {
                if (curr.altEnd.block.StartLocation.BreakAllowed) {
                    cdes.Add(new CDEBreak(curr.CDE.EndTime, curr.altEnd.block.StartTime, curr.CDE.EndLocation, curr.altEnd.block.StartLocation));
                }
                else {
                    cdes.Add(new CDEIdle(curr.CDE.EndTime, curr.altEnd.block.StartTime, curr.CDE.EndLocation, curr.altEnd.block.StartLocation));
                }
                cdes.Add(new CDEBlock(curr.altEnd.block));
            }

            cdes.Insert(0, new CDESignOnOff(cdes[0].StartTime - cdes[0].StartLocation.SignOnTime, cdes[0].StartTime, cdes[0].StartLocation));
            cdes.Add(new CDESignOnOff(cdes[^1].EndTime, cdes[^1].EndTime + cdes[^1].EndLocation.SignOffTime, cdes[^1].EndLocation));

            return [new CrewDuty(cdes) {
                Type = minType.dt
            }];
        }

        public override string ToString() {
            var min = MinTypeCost(Enumerable.Range(0, 1000).Select(x => 0.0).ToList(), 1);
            return $"Dur: {PaidDuration(DutyType.Single)}, MinTypeCost: {min.dt}/{min.cost}/{min.feasible}/{min.penaltyCost}";
        }
    }

    internal class CSPLSGlobal : CrewColumnGen {
        private List<CSPLSDuty> duties = [];

        private Random random;
        private double T;
        private double alpha;
        private int Q;

        internal CSPLSGlobal(CrewSolutionState css) : base(css) {
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
                if (css.BlockCount[i] == 0) continue;

                Block b = css.Blocks[i];

                // Add alternative costs in the event that a duty requires overcoverage 
                (double costs, int duration, Block? block, BlockArc? arc) altStart = (b.StartLocation.CrewBase ? 0 : double.PositiveInfinity, -1, null, null);
                (double costs, int duration, Block? block, BlockArc? arc) altEnd = (b.EndLocation.CrewBase ? 0 : double.PositiveInfinity, -1, null, null);

                for (int j = 0; j < css.Blocks.Count; j++) {
                    if (css.BlockCount[j] == 0) continue;

                    Block bAlt = css.Blocks[j];

                    if (bAlt.StartLocation.CrewBase && css.AdjFull[bAlt.Index][b.Index] != null) {
                        BlockArc arc = css.AdjFull[bAlt.Index][b.Index]!;
                        int duration = bAlt.Duration + arc.TotalTime;
                        double additionalCosts = (duration) / 3600.0 * Constants.CR_HOURLY_COST;
                        if (additionalCosts < altStart.costs) {
                            altStart = (additionalCosts, duration, bAlt, arc);
                        }
                    }

                    if (bAlt.EndLocation.CrewBase && css.AdjFull[b.Index][bAlt.Index] != null) {
                        BlockArc arc = css.AdjFull[b.Index][bAlt.Index]!;
                        int duration = bAlt.Duration + arc.TotalTime;
                        double additionalCosts = (duration) / 3600.0 * Constants.CR_HOURLY_COST;
                        if (additionalCosts < altEnd.costs) {
                            altEnd = (additionalCosts, duration, bAlt, arc);
                        }
                    }
                }

                CrewDutyElement cde = new CDEBlock(b);
                CSPLSNode head = new CSPLSNode() {
                    CDE = cde,
                    altEnd = altEnd,
                    altStart = altStart,
                };

                duties.Add(new() {
                    head = head,
                });
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
            var d1OldRes = duty1.MinTypeCost(blockDualCosts, 1 / T);
            var d2OldRes = duty2.MinTypeCost(blockDualCosts, 1 / T);
            costDiff -= d1OldRes.cost + d1OldRes.penaltyCost;
            costDiff -= d2OldRes.cost + d2OldRes.penaltyCost;

            // Change links, reevaluate heads
            if (prevIn1 != null) prevIn1.Next = linkPrev1ToRange;
            if (nextIn1 != null) nextIn1.Prev = linkNext1ToRange;
            if (prevIn2 != null) prevIn2.Next = linkPrev2ToNext2;
            if (nextIn2 != null) nextIn2.Prev = linkPrev2ToNext2;

            rangeStart.Prev = linkPrev1ToRange;
            rangeEnd.Next = linkNext1ToRange;

            duty1.head = rangeStart.FindLastBefore(_ => true) ?? rangeStart;
            duty2.head = prevIn2?.FindLastBefore(_ => true) ?? prevIn2 ?? nextIn2;


            // Check costs
            var d1NewRes = duty1.MinTypeCost(blockDualCosts, 1 / T);
            var d2NewRes = duty2.MinTypeCost(blockDualCosts, 1 / T);
            costDiff += d1NewRes.cost + d1NewRes.penaltyCost;
            costDiff += d2NewRes.cost + d2NewRes.penaltyCost;

            // If the second duty is now empty, remove it
            if (duty2.head == null) {
                costDiff -= duty2.BaseCost(d2OldRes.dt);
                if (duties.Count > Config.MAX_DUTIES)
                    costDiff -= Config.CR_OVER_MAX_COST;
            }

            // If infeasible or cost change not acceptable, revert  
            if (!accept(costDiff)) {
                // Revert changes
                if (prevIn1 != null) prevIn1.Next = prevIn1Next;
                if (nextIn1 != null) nextIn1.Prev = nextIn1Prev;
                if (prevIn2 != null) prevIn2.Next = prevIn2Next;
                if (nextIn2 != null) nextIn2.Prev = nextIn2Prev;
                rangeStart.Prev = rangeStartPrev;
                rangeEnd.Next = rangeEndNext;
                duty1.head = prevHead1;
                duty2.head = prevHead2;

                return LSOpResult.Decline;
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
            var d1OldRes = duty1.MinTypeCost(blockDualCosts, 1 / T);
            var d2OldRes = duty2.MinTypeCost(blockDualCosts, 1 / T);
            costDiff -= d1OldRes.cost + d1OldRes.penaltyCost;
            costDiff -= d2OldRes.cost + d2OldRes.penaltyCost;

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

            // Check costs
            var d1NewRes = duty1.MinTypeCost(blockDualCosts, 1 / T);
            var d2NewRes = duty2.MinTypeCost(blockDualCosts, 1 / T);
            costDiff += d1NewRes.cost + d1NewRes.penaltyCost;
            costDiff += d2NewRes.cost + d2NewRes.penaltyCost;

            // If one of the two duties is empty, we can also additionally remove extra cost
            if (duty1.head == null) {
                costDiff -= duty1.BaseCost(d1OldRes.dt);
                if (duties.Count > Config.MAX_DUTIES)
                    costDiff -= Config.CR_OVER_MAX_COST;
            }
            if (duty2.head == null) {
                costDiff -= duty2.BaseCost(d2OldRes.dt);
                if (duties.Count > Config.MAX_DUTIES)
                    costDiff -= Config.CR_OVER_MAX_COST;
            }

            // If either of the changes is not feasible or costs are not accepted, revert
            if (!accept(costDiff)) {
                if (lastRemainingBlock1 != null) lastRemainingBlock1.Next = lastRemainingBlock1Next;
                if (lastRemainingBlock2 != null) lastRemainingBlock2.Next = lastRemainingBlock2Next;
                if (firstBlockInTail1 != null) firstBlockInTail1.Prev = firstBlockInTail1Prev;
                if (firstBlockInTail2 != null) firstBlockInTail2.Prev = firstBlockInTail2Prev;
                duty1.head = prevHead1;
                duty2.head = prevHead2;

                return LSOpResult.Decline;
            }

            // Finalize operation 
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
            int prevSum = 0;

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

                int sum = 0;
                for (int i = 0; i < duties.Count; i++) {
                    var head = duties[i].head;
                    while (head != null) {
                        if (head.CDE.Type == CrewDutyElementType.Block) {
                            sum++;
                        }
                        head = head.Next;
                    }
                }
                if (currIts != 1 && sum != prevSum) {
                    Console.WriteLine("oh god oh fuck");
                }
                prevSum = sum;
            }

            //Console.WriteLine("Op results: " + string.Join('\t', resCounts.Select(x => x.ToString())));
            //Console.WriteLine($"Total duties/css.instance.Blocks covered: {duties.Count} / {duties.Sum(x => x.CoveredBlocks().Count)}");
            var fs = duties.SelectMany(x => x.ToCrewDuty(blockDualCosts).Select(y => (-1.0, y))).ToList();
            //Console.WriteLine($"Feasible duties/css.instance.Blocks covered: {fs.Count} / {fs.Sum(x => x.Item2!.Covers.Count)}");
            return fs;
        }
    }
}