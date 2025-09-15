using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;
using Gurobi;

namespace E_VCSP.Solver {
    internal class EVCSPCGLagrange : Solver {
        public readonly VehicleSolutionState vss;
        public readonly CrewSolutionState css;

        private readonly Random random = new();

        private Dictionary<string, int> knownBlocks = [];

        private List<(double C_i, int taskIndex)> taskReducedCosts = [];
        private List<(double C_j, int dutyIndex)> dutyReducedCosts = [];

        private List<bool> X = []; // Select vt at index
        private int vehicleSlack = 0; // Amount of over slack we are going
        private List<bool> Y = []; // Select cd at index
        private int maxDutiesSlack = 0; // Amount of over slack we are going

        /// <summary> >= 0 \forall i </summary>
        private List<double> lambdaTrips = [];
        /// <summary> \in R \forall j </summary>
        private List<double> lambdaBlocks = [];
        /// <summary> >= 0 </summary>
        private double lambdaAvgDutyLength = 0;
        /// <summary> >= 0 </summary>
        private double lambdaMaxLongDuty = 0;
        /// <summary> >= 0 </summary>
        private double lambdaMaxBroken = 0;
        /// <summary> >= 0 </summary>
        private double lambdaMaxBetween = 0;

        private double costUpperBound = double.MaxValue;
        private (double val, bool converged) objVal = (double.MaxValue, true);

        public EVCSPCGLagrange(VehicleSolutionState vss, CrewSolutionState css) {
            this.vss = vss;
            this.css = css;

            ThreadPool.SetMaxThreads(10, 10);
        }

        #region Init
        private void initializeCover() {
            // Create initial set of vehicle tasks 
            vss.Reset(false);
            VSPLSGlobal vspGlobal = new(vss);
            vspGlobal.UpdateDualCosts(vss.Instance.Trips.Select(_ => 0.0).ToList(), [], []);

            bool initialPenaltyState = Config.VSP_LS_SHR_ALLOW_PENALTY;
            Config.VSP_LS_SHR_ALLOW_PENALTY = false;
            var initialTasks = vspGlobal.GenerateVehicleTasks();
            Config.VSP_LS_SHR_ALLOW_PENALTY = initialPenaltyState;

            vss.Tasks = initialTasks.Select(x => x.vehicleTask).ToList();

            // Process blocks in the initial tasks
            css.Blocks = [];
            css.BlockCount = [];
            css.ResetFromBlocks();
            var blocksWithCount = Block.FromVehicleTasks(vss.Tasks);
            Dictionary<string, double> dummyDualCosts = [];
            foreach ((Block b, int c) in blocksWithCount) {
                knownBlocks.Add(b.Descriptor, css.Blocks.Count);
                css.AddBlock(b, c);
                dummyDualCosts[Constants.CSTR_BLOCK_COVER + b.Descriptor] = 0;
            }
            dummyDualCosts[Constants.CSTR_CR_AVG_TIME] = 0;
            dummyDualCosts[Constants.CSTR_CR_LONG_DUTIES] = 0;
            dummyDualCosts[Constants.CSTR_CR_BROKEN_DUTIES] = 0;
            dummyDualCosts[Constants.CSTR_CR_BETWEEN_DUTIES] = 0;
            foreach (var task in vss.Tasks) {
                // Ensure block index mapping for tasks
                foreach (string desc in task.BlockDescriptorCover)
                    task.BlockIndexCover.Add(knownBlocks[desc]);
                task.IsUnit = true;
            }

            // Create initial set of crew duties
            CSPLSGlobal cspGlobal = new(css);
            cspGlobal.UpdateDualCosts(dummyDualCosts, 0);
            List<CrewDuty> initialDuties = cspGlobal.GenerateDuties().Select(x => x.crewDuty).ToList();
            HashSet<string> initialDutyCovering = [.. initialDuties.SelectMany(x => x.BlockDescriptorCover)];

            // Add single duties if local search was not able to cover all blocks
            foreach (var b in css.Blocks) {
                if (initialDutyCovering.Contains(b.Descriptor)) continue;
                initialDuties.Add(new CrewDuty([new CDEBlock(b)]) { Type = DutyType.Single });
            }

            css.Duties = initialDuties.Select((x, i) => {
                x.Index = i;
                x.IsUnit = true;
                return x;
            }).ToList();
        }

        private void initializeUpperBound() {
            // Determine upper bound
            costUpperBound = 0;

            // vehicle related costs
            foreach (var vt in vss.Tasks) costUpperBound += vt.Cost;
            costUpperBound += Math.Max(0, vss.Tasks.Count - Config.MAX_VEHICLES) * Config.VH_OVER_MAX_COST;

            // crew related costs
            foreach (var cd in css.Duties) costUpperBound += cd.Cost;
            costUpperBound += Math.Max(0, css.Duties.Count - Config.MAX_DUTIES) * Config.CR_OVER_MAX_COST;

            objVal = (costUpperBound, true);
        }

        private void resetLamdba(bool clear) {
            if (clear) {
                lambdaTrips = vss.Instance.Trips.Select(x => 0.0).ToList();
                lambdaBlocks = css.Blocks.Select(x => 0.0).ToList();
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) lambdaTrips[i] = 0;
            for (int i = 0; i < css.Blocks.Count; i++) lambdaBlocks[css.Blocks[i].Index] = 0;

            lambdaAvgDutyLength = 0;
            lambdaMaxLongDuty = 0;
            lambdaMaxBroken = 0;
            lambdaMaxBetween = 0;
        }

        private void initializeLagrangeModel() {
            resetLamdba(true);

            for (int i = 0; i < vss.Tasks.Count; i++) X.Add(false);
            vehicleSlack = 0;
            for (int i = 0; i < css.Duties.Count; i++) Y.Add(false);
            maxDutiesSlack = 0;
        }
        #endregion

        private async void updateReducedCosts() {
            List<Task> tasks = new(2 * Config.THREAD_COUNT);

            int vtTotal = taskReducedCosts.Count;
            int vtChunks = Math.Min(Config.THREAD_COUNT, vtTotal);
            int vtChunkSize = (vtTotal + vtChunks - 1) / vtChunks;
            for (int c = 0, start = 0; c < vtChunks; c++) {
                int s = start;
                int end = s + vtChunkSize;
                if (end > vtTotal) end = vtTotal;
                if (s >= end) break;
                else {
                    tasks.Add(Task.Run(() => {
                        for (int i = s; i < end; i++) {
                            int taskIndex = taskReducedCosts[i].taskIndex;
                            VehicleTask task = vss.Tasks[taskIndex];
                            double cost = task.Cost;

                            for (int j = 0; j < task.TripCover.Count; j++)
                                cost -= lambdaTrips[task.TripCover[j]];
                            for (int j = 0; j < task.BlockIndexCover.Count; j++)
                                cost -= lambdaBlocks[task.BlockIndexCover[j]];

                            taskReducedCosts[i] = (cost, taskIndex);
                        }
                    }));
                }
                start = end;
                if (start >= vtTotal) break;
            }

            int cdTotal = dutyReducedCosts.Count;
            int cdChunks = Math.Min(Config.THREAD_COUNT, cdTotal);
            int cdChunkSize = (cdTotal + cdChunks - 1) / cdChunks;
            for (int c = 0, start = 0; c < cdChunks; c++) {
                int s = start;
                int end = s + cdChunkSize;
                if (end > cdTotal) end = cdTotal;
                if (s >= end) break;
                else {
                    tasks.Add(Task.Run(() => {
                        for (int i = s; i < end; i++) {
                            int dutyIndex = dutyReducedCosts[i].dutyIndex;
                            CrewDuty duty = css.Duties[dutyIndex];

                            double cost = duty.Cost;

                            for (int j = 0; j < duty.BlockIndexCover.Count; j++)
                                cost += lambdaBlocks[duty.BlockIndexCover[j]];

                            cost += lambdaAvgDutyLength * ((double)duty.Duration / Constants.CR_TARGET_SHIFT_LENGTH - 1);
                            cost -= lambdaMaxLongDuty * (Constants.CR_MAX_OVER_LONG_DUTY - duty.IsLongDuty);
                            cost -= lambdaMaxBroken * (Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty);
                            cost -= lambdaMaxBetween * (Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty);

                            dutyReducedCosts[i] = (cost, dutyIndex);
                        }
                    }));
                }
                start = end;
                if (start >= cdTotal) break;
            }

            await Task.WhenAll(tasks).ConfigureAwait(false);
        }

        private bool gradientDescent(bool allowDiscard = true) {
            if (!objVal.converged) resetLamdba(false);

            double alpha = Math.Pow(Config.LAGRANGE_PI_END / Config.LAGRANGE_PI_START, 1.0 / Config.LANGRANGE_MAX_ROUNDS);
            double pi = Config.LAGRANGE_PI_START;

            taskReducedCosts = vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
            dutyReducedCosts = css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();

            double solve() {
                // Reset current solution value
                for (int i = 0; i < X.Count; i++) X[i] = false;
                vehicleSlack = 0;
                for (int i = 0; i < Y.Count; i++) Y[i] = false;
                maxDutiesSlack = 0;

                double cost = 0;

                // Base cost: lamdbda for trips
                for (int i = 0; i < lambdaTrips.Count; i++)
                    cost += lambdaTrips[i];

                // Vehicles
                taskReducedCosts.Sort();
                int selectedTaskCount = 0;
                double taskSlackPenalty = Config.VH_OVER_MAX_COST;
                for (int i = 0; i < taskReducedCosts.Count; i++) {
                    var targetTask = taskReducedCosts[i];
                    if (selectedTaskCount < Config.MAX_VEHICLES && targetTask.C_i < 0) {
                        // Select normally
                        X[targetTask.taskIndex] = true;
                        cost += targetTask.C_i;
                        selectedTaskCount++;
                    }
                    else if (selectedTaskCount >= Config.MAX_VEHICLES && targetTask.C_i + taskSlackPenalty < 0) {
                        // Select as slack vehicle
                        X[targetTask.taskIndex] = true;
                        cost += targetTask.C_i + taskSlackPenalty;
                        selectedTaskCount++;
                        vehicleSlack++;
                    }
                    else {
                        // No profit to be made anymore, stop
                        break;
                    }
                }

                // Crew
                dutyReducedCosts.Sort();
                int selectedDutyCount = 0;
                double crewSlackPenalty = Config.CR_OVER_MAX_COST;
                for (int i = 0; i < dutyReducedCosts.Count; i++) {
                    var targetDuty = dutyReducedCosts[i];

                    if (selectedDutyCount < Config.MAX_DUTIES && targetDuty.C_j < 0) {
                        // Select normally
                        cost += targetDuty.C_j;
                        Y[targetDuty.dutyIndex] = true;
                        selectedDutyCount++;
                    }
                    else if (selectedDutyCount >= Config.MAX_DUTIES && targetDuty.C_j + crewSlackPenalty < 0) {
                        // Select as slack vehicle
                        cost += targetDuty.C_j + crewSlackPenalty;
                        Y[targetDuty.dutyIndex] = true;
                        selectedDutyCount++;
                        maxDutiesSlack++;
                    }
                    else {
                        break;
                    }
                }

                return cost;
            }

            void updateMultipliers(double z_curr) {
                // Trips
                double[] GTrips = new double[lambdaTrips.Count];
                for (int i = 0; i < GTrips.Length; i++) GTrips[i] = 1; // b
                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (!X[taskIndex]) continue;
                    foreach (var tripIndex in vss.Tasks[taskIndex].TripCover)
                        GTrips[tripIndex] -= 1;
                }

                // Blocks / duties
                double[] GBlocks = new double[lambdaBlocks.Count]; // b == 0
                double GAvgDutyLength = 0;
                double GMaxLongDuty = 0;
                double GMaxBroken = 0;
                double GMaxBetween = 0;

                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (!X[taskIndex]) continue;
                    foreach (var blockIndex in vss.Tasks[taskIndex].BlockIndexCover) {
                        GBlocks[blockIndex] -= 1;
                    }
                }
                for (int dutyIndex = 0; dutyIndex < Y.Count; dutyIndex++) {
                    if (!Y[dutyIndex]) continue;
                    var duty = css.Duties[dutyIndex];
                    foreach (var blockIndex in duty.BlockIndexCover) {
                        GBlocks[blockIndex] += 1;
                    }

                    GAvgDutyLength += ((double)duty.Duration / Constants.CR_TARGET_SHIFT_LENGTH - 1.0);
                    GMaxLongDuty -= (Constants.CR_MAX_OVER_LONG_DUTY - duty.IsLongDuty);
                    GMaxBroken -= (Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty);
                    GMaxBetween -= (Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty);
                }

                double GSquaredSum = 0;
                for (int i = 0; i < GTrips.Length; i++) GSquaredSum += GTrips[i] * GTrips[i];
                for (int i = 0; i < GBlocks.Length; i++) GSquaredSum += GBlocks[i] * GBlocks[i];
                GSquaredSum += GAvgDutyLength * GAvgDutyLength;
                GSquaredSum += GMaxLongDuty * GMaxLongDuty;
                GSquaredSum += GMaxBroken * GMaxBroken;
                GSquaredSum += GMaxBetween * GMaxBetween;

                double T = pi * (costUpperBound - z_curr) / GSquaredSum;

                // Everything >= / == constraint
                for (int i = 0; i < lambdaTrips.Count; i++) {
                    lambdaTrips[i] = Math.Max(0, lambdaTrips[i] + T * GTrips[i]);
                }

                for (int i = 0; i < lambdaBlocks.Count; i++) {
                    // == constraint
                    lambdaBlocks[i] = lambdaBlocks[i] + T * GBlocks[i];
                }

                lambdaAvgDutyLength = Math.Max(0, lambdaAvgDutyLength + T * GAvgDutyLength);
                lambdaMaxLongDuty = Math.Max(0, lambdaMaxLongDuty + T * GMaxLongDuty);
                lambdaMaxBroken = Math.Max(0, lambdaMaxBroken + T * GMaxBroken);
                lambdaMaxBetween = Math.Max(0, lambdaMaxBetween + T * GMaxBetween);
            }

            double lastSolutionVal = costUpperBound;
            int roundsInThreshold = 0;
            bool converged = false;

            int round = 0;
            for (; round < Config.LANGRANGE_MAX_ROUNDS && !converged; round++) {
                updateReducedCosts();
                double z_curr = solve();
                if (double.IsNaN(z_curr)) {
                    Console.WriteLine("Divergeert");
                    throw new InvalidOperationException();
                }
                updateMultipliers(z_curr);
                pi *= alpha;

                // Solution might have stabelized
                double percentageDiff = Math.Abs(Math.Abs(z_curr - lastSolutionVal) / lastSolutionVal);
                bool diffInThreshold = percentageDiff < Config.LANGRANGE_THRS;
                lastSolutionVal = z_curr;

                if (diffInThreshold) {
                    roundsInThreshold++;
                }
                else {
                    roundsInThreshold = 0;
                }

                if (roundsInThreshold >= Config.LANGRANGE_THRS_SEQ) converged = true;
            }

            objVal = (lastSolutionVal, converged);

            // Finalize reduced costs given the current set of lamdba parameters
            updateReducedCosts();

            // throw away the worst vehicle tasks / crew duties
            if (allowDiscard) {
                bool rcUpdateRequired = false;

                if (vss.Tasks.Count > Config.VCSP_MAX_TASKS_DURING) {
                    rcUpdateRequired = true;
                    taskReducedCosts.Sort();
                    HashSet<int> taskIndexesToRemove = new();
                    for (int j = Config.VCSP_MAX_TASKS_DURING; j < taskReducedCosts.Count; j++) {
                        if (!vss.Tasks[taskReducedCosts[j].taskIndex].IsUnit)
                            taskIndexesToRemove.Add(taskReducedCosts[j].taskIndex);
                    }
                    vss.Tasks = vss.Tasks.Where((vt, i) => !taskIndexesToRemove.Contains(i)).Select((x, i) => { x.Index = i; return x; }).ToList();
                    X = X.Where((_, i) => !taskIndexesToRemove.Contains(i)).ToList();
                    taskReducedCosts = vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
                }
                if (css.Duties.Count > Config.VCSP_MAX_DUTIES_DURING) {
                    rcUpdateRequired = true;
                    dutyReducedCosts.Sort();
                    HashSet<int> dutyIndexesToRemove = new();
                    for (int j = Config.VCSP_MAX_DUTIES_DURING; j < dutyReducedCosts.Count; j++) {
                        if (!css.Duties[dutyReducedCosts[j].dutyIndex].IsUnit)
                            dutyIndexesToRemove.Add(dutyReducedCosts[j].dutyIndex);
                    }
                    css.Duties = css.Duties.Where((cd, i) => !dutyIndexesToRemove.Contains(i)).Select((x, i) => { x.Index = i; return x; }).ToList();
                    Y = Y.Where((_, i) => !dutyIndexesToRemove.Contains(i)).ToList();
                    dutyReducedCosts = css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();
                }
                if (rcUpdateRequired) updateReducedCosts();
            }

            return converged;
        }

        private void disruptMultipliers() {
            // Adjust multipliers semi-arbitrarily
            for (int i = 0; i < lambdaTrips.Count; i++) {
                double disruption = random.NextDouble() * (Config.LAGRANGE_DISRUPT_UPR - Config.LAGRANGE_DISRUPT_LWR) + Config.LAGRANGE_DISRUPT_LWR; // [0.5, 2)
                lambdaTrips[i] *= disruption;
            }
            for (int i = 0; i < lambdaBlocks.Count; i++) {
                double disruption = random.NextDouble() * (Config.LAGRANGE_DISRUPT_UPR - Config.LAGRANGE_DISRUPT_LWR) + Config.LAGRANGE_DISRUPT_LWR; // [0.5, 2)
                lambdaBlocks[i] *= disruption;
            }
        }

        private Dictionary<string, double> crewDualCost() {
            Dictionary<string, double> res = lambdaBlocks.Select((l, i) => (Constants.CSTR_BLOCK_COVER + css.Blocks[i].Descriptor, -l)).ToDictionary();
            res[Constants.CSTR_CR_AVG_TIME] = lambdaAvgDutyLength;
            res[Constants.CSTR_CR_LONG_DUTIES] = lambdaMaxLongDuty;
            res[Constants.CSTR_CR_BROKEN_DUTIES] = lambdaMaxBroken;
            res[Constants.CSTR_CR_BETWEEN_DUTIES] = lambdaMaxBetween;
            return res;
        }

        private void runVehicleIts(int round, bool disrupt = false) {
            int maxIts = round == 0 ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;
            VSPLabeling vspLabelingInstance = new(vss);

            void makeVTsNonNegativeRC() {
                if (!Config.VCSP_NONNEGATIVE_RC_VSP) return;

                // Apply heuristic based on that of Marcel which attempts to rescale lambda s.t. 
                // all currently known vehicle task columns do not have negative reduced cost.
                List<(double C_i, int taskIndex)> tasksToUpdate = [];
                for (int i = 0; i < taskReducedCosts.Count; i++) {
                    if (taskReducedCosts[i].C_i < 0 && X[i])
                        tasksToUpdate.Add(taskReducedCosts[i]);
                }
                for (int i = 0; i < tasksToUpdate.Count; i++) {
                    (double C_i, int taskIndex) = tasksToUpdate[i];
                    List<int> tripCover = vss.Tasks[taskIndex].TripCover;
                    List<int> blockCover = vss.Tasks[taskIndex].BlockIndexCover;
                    double delta = C_i / (tripCover.Count + blockCover.Count);

                    for (int j = 0; j < tripCover.Count; j++) {
                        lambdaTrips[tripCover[j]] += delta;
                    }
                    for (int j = 0; j < blockCover.Count; j++) {
                        lambdaBlocks[blockCover[j]] += delta;
                    }
                }
            }

            void updateDualCosts() {
                Dictionary<string, double> blockDualCosts = new();
                Dictionary<string, List<double>> blockDualCostsByStart = new();

                for (int bi = 0; bi < css.Blocks.Count; bi++) {
                    Block b = css.Blocks[bi];
                    string descriptor = b.Descriptor;
                    string descriptorStart = Descriptor.GetStart(descriptor);

                    blockDualCosts[descriptor] = lambdaBlocks[bi];
                    if (blockDualCostsByStart.ContainsKey(descriptorStart))
                        blockDualCostsByStart[descriptorStart].Add(lambdaBlocks[bi]);
                    else blockDualCostsByStart[descriptorStart] =
                            [lambdaBlocks[bi]];
                }

                vspLabelingInstance.UpdateDualCosts(
                    lambdaTrips,
                    blockDualCosts,
                    blockDualCostsByStart
                );
            }

            for (int currIt = 0; currIt < maxIts; currIt++) {
                makeVTsNonNegativeRC();
                if (disrupt)
                    disruptMultipliers();
                updateDualCosts();
                var newColumns = vspLabelingInstance.GenerateVehicleTasks();

                foreach ((var reducedCosts, var newTask) in newColumns) {
                    if (reducedCosts > 0) continue;

                    newTask.Index = vss.Tasks.Count;

                    // Determine blocks for task; add any not yet known ones to css
                    List<Block> blocks = Block.FromVehicleTask(newTask);
                    foreach (Block b in blocks) {
                        string desc = b.Descriptor;
                        if (!knownBlocks.ContainsKey(desc)) {
                            knownBlocks[desc] = css.Blocks.Count;
                            css.AddBlock(b, 1);
                            Y.Add(false); //unit duty from block being added
                            lambdaBlocks.Add(0); // New block multiplier
                        }

                        int blockIndex = knownBlocks[desc];
                        newTask.BlockIndexCover.Add(blockIndex);
                    }

                    vss.Tasks.Add(newTask);
                    X.Add(false);
                }

                Console.WriteLine($"{round}.V{currIt}\t{objVal.val:0.##}\t{X.Count(x => x)}\t{X.Count}\t{Y.Count(y => y)}\t{Y.Count}\t{newColumns.Count}");
                gradientDescent(disrupt);
                Console.WriteLine($"{round}.V{currIt}\t{objVal.val:0.##}\t{X.Count(x => x)}\t{X.Count}\t{Y.Count(y => y)}\t{Y.Count}\t{newColumns.Count}");
            }
        }

        private void runCrewIts(int round, bool disrupt = false) {
            int maxIts = round == 0 ? Config.VCSP_CR_ITS_INIT : Config.VCSP_CR_ITS_ROUND;

            List<int> blockCount = new(css.Blocks.Count);
            for (int i = 0; i < css.Blocks.Count; i++) blockCount.Add(0);

            int updateBlockCount() {
                int total = 0;
                for (int i = 0; i < css.Blocks.Count; i++) blockCount[i] = 0;
                for (int i = 0; i < X.Count; i++) {
                    if (!X[i]) continue;

                    VehicleTask task = vss.Tasks[i];
                    foreach (int blockIndex in task.BlockIndexCover) {
                        blockCount[blockIndex]++;
                        total++;
                    }
                }

                css.BlockCount = blockCount;
                return total;
            }

            void makeCDsNonNegativeRC() {
                if (!Config.VCSP_NONNEGATIVE_RC_CSP) return;
                List<(double C_j, int dutyIndex)> dutiesToUpdate = [];
                for (int j = 0; j < dutyReducedCosts.Count; j++) {
                    if (dutyReducedCosts[j].C_j < 0 && Y[j])
                        dutiesToUpdate.Add(dutyReducedCosts[j]);
                }
                for (int i = 0; i < dutiesToUpdate.Count; i++) {
                    (double C_j, int dutyIndex) = dutiesToUpdate[i];
                    List<int> blockCover = css.Duties[dutyIndex].BlockIndexCover;
                    double delta = C_j / (blockCover.Count);

                    for (int j = 0; j < blockCover.Count; j++) {
                        lambdaBlocks[blockCover[j]] -= delta;
                    }
                }
            }

            void addRandomBlocks() {
                // Add some random blocks 
                for (int i = 0; i < css.Blocks.Count; i++) {
                    // Block will be used as a basis for expanding
                    if (random.NextDouble() >= Config.VSCP_BLOCK_ADD_CHANCE * css.BlockCount[i]) continue;

                    Block block = css.Blocks[i];

                    List<(double cost, bool front, Location startLocation, int startTime)> candidateDescriptors = [];

                    // Check if first element of block can be expanded
                    // Expansion as trip: add trip in front/at back (for now)
                    if (block.Elements[0].Type == BlockElementType.Trip || block.Elements[0].Type == BlockElementType.Deadhead) {
                        // Check if there is a trip that can be used to extend the block backwards
                        for (int ti = 0; ti < vss.Instance.Trips.Count; ti++) {
                            Trip t = vss.Instance.Trips[ti];
                            if (t.EndLocation != block.StartLocation || t.EndTime > block.StartTime) continue;

                            // Block transition allowed; can only be done with short idle
                            // as break / long idle start a new block
                            int waitingTime = block.StartTime - t.EndTime;
                            bool transitionAllowed = Constants.CR_MIN_SHORT_IDLE_TIME <= waitingTime && waitingTime <= Constants.CR_MAX_SHORT_IDLE_TIME;
                            bool overallTimeAllowed = t.Duration + waitingTime + block.Duration <= Constants.MAX_STEERING_TIME;
                            if (!transitionAllowed || !overallTimeAllowed) continue;

                            // Check if this block is already known
                            string candidateDescriptor = Descriptor.Create(t.StartLocation, t.StartTime, block.EndLocation, block.EndTime);
                            if (knownBlocks.ContainsKey(candidateDescriptor)) continue;

                            candidateDescriptors.Add((-lambdaTrips[t.Index], true, t.StartLocation, t.StartTime));
                        }

                        // Check if we can add an incoming deadhead; can only be done if were not directly followed
                        // by a deadhead (would be weird)
                        if (block.Elements[0].Type == BlockElementType.Trip) {
                            Location curr = block.Elements[0].StartLocation;
                            for (int li = 0; li < vss.Instance.Locations.Count; li++) {
                                DeadheadTemplate? dht = vss.LocationDHT[li][curr.Index];
                                if (dht == null) continue;

                                string candidateDescriptor = Descriptor.Create(
                                    dht.StartLocation,
                                    block.StartTime - dht.Duration,
                                    block.EndLocation,
                                    block.EndTime
                                );
                                if (knownBlocks.ContainsKey(candidateDescriptor)) continue;
                                // Ignore driving costs
                                candidateDescriptors.Add((0, true, dht.StartLocation, block.StartTime - dht.Duration));
                            }
                        }
                    }

                    if (block.Elements[^1].Type == BlockElementType.Trip || block.Elements[^1].Type == BlockElementType.Deadhead) {
                        // Same but forwards
                        for (int ti = 0; ti < vss.Instance.Trips.Count; ti++) {
                            Trip t = vss.Instance.Trips[ti];
                            if (t.StartLocation != block.EndLocation || block.EndTime > t.StartTime) continue;

                            // Block transition allowed; can only be done with short idle
                            // as break / long idle start a new block
                            int waitingTime = t.StartTime - block.EndTime;
                            bool transitionAllowed = Constants.CR_MIN_SHORT_IDLE_TIME <= waitingTime && waitingTime <= Constants.CR_MAX_SHORT_IDLE_TIME;
                            bool overallTimeAllowed = t.Duration + waitingTime + block.Duration <= Constants.MAX_STEERING_TIME;
                            if (!transitionAllowed || !overallTimeAllowed) continue;

                            // Check if this block is already known
                            string candidateDescriptor = Descriptor.Create(block.StartLocation, block.StartTime, t.EndLocation, t.EndTime);
                            if (knownBlocks.ContainsKey(candidateDescriptor)) continue;

                            candidateDescriptors.Add((-lambdaTrips[t.Index], false, t.EndLocation, t.EndTime));
                        }

                        // Check if we can add an incoming deadhead; can only be done if were not directly followed
                        // by a deadhead (would be weird)
                        if (block.Elements[^1].Type == BlockElementType.Trip) {
                            Location curr = block.Elements[^1].StartLocation;
                            for (int li = 0; li < vss.Instance.Locations.Count; li++) {
                                DeadheadTemplate? dht = vss.LocationDHT[curr.Index][li];
                                if (dht == null) continue;

                                string candidateDescriptor = Descriptor.Create(
                                    block.StartLocation,
                                    block.StartTime,
                                    dht.EndLocation,
                                    block.EndTime + dht.Duration
                                );
                                if (knownBlocks.ContainsKey(candidateDescriptor)) continue;
                                // Ignore driving costs
                                candidateDescriptors.Add((0, false, dht.EndLocation, block.EndTime + dht.Duration));
                            }
                        }
                    }

                    candidateDescriptors = candidateDescriptors.OrderBy((x) => x.cost).ToList();
                    for (int ci = 0; ci < candidateDescriptors.Count; ci++) {
                        (_, bool front, Location l, int t) = candidateDescriptors[ci];
                        string desc = front ? Descriptor.Create(l, t, block.EndLocation, block.EndTime) : Descriptor.Create(block.StartLocation, block.StartTime, l, t);
                        if (knownBlocks.ContainsKey(desc)) continue;
                        Block newBlock = front ? Block.FromDescriptor(l, t, block.EndLocation, block.EndTime) : Block.FromDescriptor(block.StartLocation, block.StartTime, l, t);

                        knownBlocks[desc] = css.Blocks.Count;
                        css.AddBlock(newBlock, 1);
                        Y.Add(false); //unit duty from block being added
                        lambdaBlocks.Add(0); // New block multiplier
                        break; // only add one
                    }
                }
            }

            CSPLabeling[] cspLabelingInstances = [.. Enumerable.Range(0, Config.VCSP_CR_INSTANCES).Select(_ => new CSPLabeling(css))];
            for (int it = 0; it < maxIts; it++) {
                makeCDsNonNegativeRC();
                if (disrupt) disruptMultipliers();
                int totalBlocks = updateBlockCount();

                addRandomBlocks();

                var dualCost = crewDualCost();
                List<(double reducedCosts, CrewDuty newDuty)> newColumns = [];

                Parallel.For(0, cspLabelingInstances.Length, (i) => {
                    cspLabelingInstances[i].UpdateDualCosts(dualCost, 1);
                    var res = cspLabelingInstances[i].GenerateDuties();
                    foreach (var resCol in res)
                        if (resCol.crewDuty != null) {
                            newColumns.Add(resCol);
                        }
                });

                foreach ((double reducedCost, CrewDuty newDuty) in newColumns) {
                    if (reducedCost >= 0) continue;

                    newDuty.Index = css.Duties.Count;
                    css.Duties.Add(newDuty);
                    Y.Add(false);
                }
                gradientDescent(disrupt);
                Console.WriteLine($"{round}.C{it}\t{objVal.val:0.##}\t{X.Count(x => x)}\t{X.Count}\t{Y.Count(y => y)}\t{Y.Count}\t{newColumns.Count}");
            }
        }

        private void solveILP() {
            // Solve using ILP
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Constants.RUN_LOG_FOLDER, "evcspcg_gurobi.log")
            };
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 1; // upper bound
            model.Parameters.Heuristics = 0.8;
            model.Parameters.RINS = 10;
            model.Parameters.SubMIPNodes = 5000;
            model.Parameters.PumpPasses = 20;
            model.Parameters.NoRelHeurTime = Config.VCSP_SOLVER_TIMEOUT_SEC / 4;
            model.Parameters.ImproveStartTime = Config.VCSP_SOLVER_TIMEOUT_SEC / 4;
            model.Parameters.Cuts = 1;
            model.Parameters.Presolve = 2;
            model.Parameters.Symmetry = 2;
            model.SetCallback(new CustomGRBCallback());

            List<GRBVar> taskVars = [], dutyVars = [];
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, vss.Tasks[i].Cost, GRB.BINARY, name);
                taskVars.Add(v);
            }
            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost, GRB.INTEGER, name);
                dutyVars.Add(v);
            }

            // Max selected vehicle tasks
            GRBLinExpr maxVehiclesExpr = new();
            GRBVar maxVehiclesSlack = model.AddVar(
                0, Config.VCSP_VH_CSTR_SLACK ? GRB.INFINITY : 0, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_slack"
            );
            foreach (GRBVar v in taskVars) {
                maxVehiclesExpr += v;
            }
            model.AddConstr(maxVehiclesExpr - maxVehiclesSlack <= Config.MAX_VEHICLES, Constants.CSTR_MAX_VEHICLES);

            // Trip cover by vehicle tasks
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                string name = Constants.CSTR_TRIP_COVER + t.Index;
                model.AddConstr(expr, sense, 1, name);
            }

            // Block cover of selected tasks by duties
            for (int blockIndex = 0; blockIndex < css.Blocks.Count; blockIndex++) {
                string blockDescriptor = css.Blocks[blockIndex].Descriptor;
                GRBLinExpr expr = new();

                for (int vtIndex = 0; vtIndex < taskVars.Count; vtIndex++) {
                    if (vss.Tasks[vtIndex].BlockIndexCover.Contains(blockIndex))
                        expr += taskVars[vtIndex];
                }

                for (int cdIndex = 0; cdIndex < dutyVars.Count; cdIndex++) {
                    if (css.Duties[cdIndex].BlockIndexCover.Contains(blockIndex))
                        expr -= dutyVars[cdIndex];
                }

                string name = Constants.CSTR_BLOCK_COVER + blockDescriptor;
                model.AddConstr(expr <= 0, name);
            }

            GRBLinExpr maxDutiesExpr = new();
            GRBVar maxDutiesSlack = model.AddVar(
                0, Config.VCSP_CR_MAX_CSTR_SLACK ? GRB.INFINITY : 0, Config.CR_OVER_MAX_COST, GRB.CONTINUOUS, "duty_slack"
            );
            foreach (GRBVar v in dutyVars) {
                maxDutiesExpr += v;
            }
            model.AddConstr(maxDutiesExpr - maxDutiesSlack <= Config.MAX_DUTIES, Constants.CSTR_MAX_DUTIES);

            // Duty type / avg duration
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between

            double maxSlack = Config.VCSP_CR_OTH_CSTR_SLACK ? 0 : GRB.INFINITY;

            GRBVar noExcessiveLengthSlack = model.AddVar(0, maxSlack, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, maxSlack, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, maxSlack, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, maxSlack, Constants.CR_HARD_CONSTR_PENALTY, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++) {
                GRBVar v = dutyVars[i];
                CrewDuty duty = css.Duties[i];

                int duration = duty.Elements[^1].EndTime - duty.Elements[0].StartTime;
                noExcessiveLength += Constants.CR_MAX_OVER_LONG_DUTY * v - (duty.IsLongDuty * v);
                limitedAverageLength += v * (duration / (double)Constants.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Constants.CR_MAX_BROKEN_SHIFTS - (duty.IsBrokenDuty * v);
                maxBetween += v * Constants.CR_MAX_BETWEEN_SHIFTS - (duty.IsBetweenDuty * v);
            }

            model.AddConstr(noExcessiveLength + noExcessiveLengthSlack >= 0, Constants.CSTR_CR_LONG_DUTIES);
            model.AddConstr(limitedAverageLength - limitedAverageLengthSlack <= 0, Constants.CSTR_CR_AVG_TIME);
            model.AddConstr(maxBroken + maxBrokenSlack >= 0, Constants.CSTR_CR_BROKEN_DUTIES);
            model.AddConstr(maxBetween + maxBetweenSlack >= 0, Constants.CSTR_CR_BETWEEN_DUTIES);

            Config.CONSOLE_GUROBI = true;
            model.Optimize();
            Config.CONSOLE_GUROBI = false;

            vss.SelectedTasks = [];
            css.SelectedDuties = [];
            for (int i = 0; i < taskVars.Count; i++) {
                if (Math.Round(taskVars[i].X) < 1) continue;
                vss.SelectedTasks.Add(vss.Tasks[i]);
            }
            for (int i = 0; i < dutyVars.Count; i++) {
                if (Math.Round(dutyVars[i].X) < 1) continue;
                css.SelectedDuties.Add((css.Duties[i], (int)Math.Round(dutyVars[i].X)));
            }

            Console.WriteLine($"Solution found with {vss.SelectedTasks.Count} vehicles, {css.SelectedDuties.Count} duties, overall costs {model.ObjVal}");
            Dictionary<int, int> vehicleTripCover = [];
            Dictionary<string, int> vehicleBlockCover = [];
            Dictionary<string, int> crewBlockCover = [];
            HashSet<string> blockDescriptors = [];
            for (int i = 0; i < vss.SelectedTasks.Count; i++) {
                foreach (var elem in vss.SelectedTasks[i].Elements) {
                    if (elem is VETrip vet) {
                        vehicleTripCover.TryAdd(vet.Trip.Index, 0);
                        vehicleTripCover[vet.Trip.Index]++;
                    }
                }

                foreach (var bi in vss.SelectedTasks[i].BlockDescriptorCover) {
                    vehicleBlockCover.TryAdd(bi, 0);
                    vehicleBlockCover[bi]++;
                    blockDescriptors.Add(bi);
                }
            }

            for (int i = 0; i < css.SelectedDuties.Count; i++) {
                (CrewDuty duty, int count) = css.SelectedDuties[i];
                foreach (var bi in duty.BlockDescriptorCover) {
                    crewBlockCover.TryAdd(bi, 0);
                    crewBlockCover[bi] += count;
                    blockDescriptors.Add(bi);
                }
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) {
                if (!vehicleTripCover.ContainsKey(i)) {
                    Console.WriteLine($"!!! Trip {i} not covered !!!");
                }
                else if (vehicleTripCover[i] > 1) {
                    Console.WriteLine($"--- Trip {i} covered {vehicleTripCover[i]} times ---");
                }
            }

            foreach (var bi in blockDescriptors) {
                if (!crewBlockCover.ContainsKey(bi) && vehicleBlockCover.ContainsKey(bi)) {
                    Console.WriteLine($"!!! Vehicle uses block {bi}, crew does not !!!");
                }
                else if (crewBlockCover.ContainsKey(bi) && !vehicleBlockCover.ContainsKey(bi)) {
                    Console.WriteLine($"\\/\\/ Crew uses block {bi}, vehicle does not \\/\\/");
                }
                else if (crewBlockCover[bi] != vehicleBlockCover[bi]) {
                    Console.WriteLine($"\\/\\/ Block {bi} used {vehicleBlockCover[bi]} times by vehicle, {crewBlockCover[bi]} by crew \\/\\/");
                }
            }

            vss.PrintCostBreakdown((int)maxVehiclesSlack.X);
            css.PrintCostBreakdown((int)maxDutiesSlack.X, limitedAverageLengthSlack.X, noExcessiveLengthSlack.X, maxBrokenSlack.X, maxBetweenSlack.X);
        }

        public override bool Solve(CancellationToken ct) {
            initializeCover();
            initializeUpperBound();
            initializeLagrangeModel();

            // Initialize dual cost
            gradientDescent();
            Console.WriteLine($"I\t{objVal.val:0.##}\t{X.Count(x => x)}\t{X.Count}\t{Y.Count(y => y)}\t{Y.Count}");

            int round = 0;
            for (; round < Config.VCSP_ROUNDS; round++) {
                runVehicleIts(round);
                runCrewIts(round);

                Console.WriteLine($"{round}\t{objVal.val:0.##}\t{X.Count(x => x)}\t{X.Count}\t{Y.Count(y => y)}\t{Y.Count}");
            }

            if (Config.LAGRANGE_DISRUPTION_ROUNDS < 1) {
                Console.WriteLine("Skipping disruption rounds");
            }
            else {
                Console.WriteLine($"Doing {Config.LAGRANGE_DISRUPTION_ROUNDS} additional round{((Config.LAGRANGE_DISRUPTION_ROUNDS > 1) ? "s" : "")} with disrupted lambda");
                for (int i = 0; i < Config.LAGRANGE_DISRUPTION_ROUNDS; i++) {
                    runVehicleIts(round + i, true);
                    runCrewIts(round + i, true);
                }
            }

            // Actually solve
            solveILP();

            return true;
        }
    }
}
