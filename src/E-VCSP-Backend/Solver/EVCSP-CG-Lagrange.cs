using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;
using Gurobi;
using System.Collections;

namespace E_VCSP.Solver {
    public class EVCSPCGLagrange : Solver {
        public readonly VehicleSolutionState vss;
        public readonly CrewSolutionState css;
        public double initialSolutionQuality = double.PositiveInfinity;
        public GRBModel? model;

        private readonly Random random = new();

        /// <summary>
        /// Dictionary from block descriptor to block index in css
        /// </summary>
        private Dictionary<Descriptor, int> knownBlocks = [];
        /// <summary>
        /// Vehicle tasks, checked on covered trips / blocks. Maps to an index into vss
        /// </summary>
        private Dictionary<(BitArray, BitArray), int> knownVTs = new(new BitArrayTupleComparer());
        /// <summary>
        /// Crew duties, checked on covered blocks and duty type. Maps to an index into css
        /// </summary>
        private Dictionary<(BitArray, int), int> knownCDs = new(new BitArrayIntComparer());

        private List<(double C_i, int taskIndex)> taskReducedCosts = [];
        private List<(double C_j, int dutyIndex)> dutyReducedCosts = [];

        private List<int> X = []; // Select vt at index
        private List<int> Y = []; // Select cd at index

        /// <summary> >= 0 \forall i </summary>
        private List<double> lambdaTrips = [];
        /// <summary> >= 0 \forall j </summary>
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
        }

        #region Init
        public (VehicleSolutionState vss, CrewSolutionState css) initializeCover() {
            if (vss.SelectedTasks.Count > 0 && css.SelectedDuties.Count > 0) {
                initializeExistingCover();
                return (vss, css);
            }

            EVSPCG vspSolver = new(vss);
            vspSolver.Solve();
            List<(Block block, int count)> initialBlocks = Block.FromVehicleTasks(vss.SelectedTasks);
            foreach ((var b, int c) in initialBlocks)
                css.AddBlock(b, c);
            CSPCG cspSolver = new(css);
            cspSolver.Solve();

            // Initial solution is (vss.SelectedTasks, css.SelectedDuties)
            costUpperBound = 0;
            foreach (var vt in vss.SelectedTasks) costUpperBound += vt.Cost;
            costUpperBound += Math.Max(0, vss.SelectedTasks.Count - Config.MAX_VEHICLES) * Config.VH_OVER_MAX_COST;
            foreach (var cd in css.SelectedDuties) costUpperBound += cd.duty.Cost * cd.count;
            costUpperBound += Math.Max(0, css.SelectedDuties.Sum(x => x.count) - Config.MAX_DUTIES) * Config.CR_OVER_MAX_COST;
            initialSolutionQuality = costUpperBound;

            // Index blocks in both tasks and duties; allows reuse of generated columns in integrated approach
            List<(Block b, int c)> allBlocks = Block.FromVehicleTasks(vss.Tasks);
            css.ResetBlocks();
            for (int bi = 0; bi < allBlocks.Count; bi++) {
                css.AddBlock(allBlocks[bi].b, 1);
                knownBlocks[allBlocks[bi].b.Descriptor] = bi;
            }

            // reindex tasks, add them to known columns
            for (int i = 0; i < vss.Tasks.Count; i++) {
                var task = vss.Tasks[i];
                if (task.Index != i) throw new InvalidDataException("Task indexing in initial solution is wrong");
                task.BlockIndexCover = [];
                foreach (Descriptor desc in task.BlockDescriptorCover) {
                    task.BlockIndexCover.Add(knownBlocks[desc]);
                }

                BitArray tripCover = task.ToTripBitArray(vss.Instance.Trips.Count);
                BitArray blockCover = task.ToBlockBitArray(css.Blocks.Count);
                if (knownVTs.ContainsKey((tripCover, blockCover))) {
                    Console.WriteLine("Duplicate columns present in initial vh solution");
                }
                knownVTs.Add((tripCover, blockCover), i);
            }
            // reindex duties, add them to known columns
            for (int i = 0; i < css.Duties.Count; i++) {
                var duty = css.Duties[i];
                if (duty.Index != i) throw new InvalidDataException("Duty indexing in initial solution is wrong");
                duty.BlockIndexCover = [];
                foreach (Descriptor desc in duty.BlockDescriptorCover) {
                    duty.BlockIndexCover.Add(knownBlocks[desc]);
                }

                BitArray blockCover = duty.ToBlockBitArray(css.Blocks.Count);
                int dutyType = (int)duty.Type;
                if (knownCDs.ContainsKey((blockCover, dutyType))) {
                    Console.WriteLine("Duplicate columns present in initial cr solution");
                }
                knownCDs.Add((blockCover, dutyType), i);
            }

            foreach (var task in vss.SelectedTasks) {
                task.IsUnit = true;
            }
            foreach ((var duty, _) in css.SelectedDuties) {
                duty.IsUnit = true;
            }

            return (vss, css);
        }

        private void initializeExistingCover() {
            if (vss.SelectedTasks.Count == 0 || css.SelectedDuties.Count == 0)
                throw new InvalidOperationException("Cannot initialize from nonexistent cover");

            Console.WriteLine("Using existing cover");
            costUpperBound = 0;
            foreach (var vt in vss.SelectedTasks) costUpperBound += vt.Cost;
            costUpperBound += Math.Max(0, vss.SelectedTasks.Count - Config.MAX_VEHICLES) * Config.VH_OVER_MAX_COST;
            foreach (var cd in css.SelectedDuties) costUpperBound += cd.duty.Cost * cd.count;
            costUpperBound += Math.Max(0, css.SelectedDuties.Sum(x => x.count) - Config.MAX_DUTIES) * Config.CR_OVER_MAX_COST;
            initialSolutionQuality = costUpperBound;

            for (int bi = 0; bi < css.Blocks.Count; bi++) {
                knownBlocks[css.Blocks[bi].Descriptor] = bi;
            }

            for (int i = 0; i < vss.Tasks.Count; i++) {
                VehicleTask task = vss.Tasks[i];
                BitArray tripCover = task.ToTripBitArray(vss.Instance.Trips.Count);
                BitArray blockCover = task.ToBlockBitArray(css.Blocks.Count);
                if (knownVTs.ContainsKey((tripCover, blockCover))) {
                    Console.WriteLine("Duplicate columns present in initial vh solution");
                }
                knownVTs.Add((tripCover, blockCover), i);
            }

            for (int i = 0; i < css.Duties.Count; i++) {
                CrewDuty duty = css.Duties[i];
                BitArray blockCover = duty.ToBlockBitArray(css.Blocks.Count);
                int dutyType = (int)duty.Type;
                if (knownCDs.ContainsKey((blockCover, dutyType))) {
                    Console.WriteLine("Duplicate columns present in initial cr solution");
                }
                knownCDs.Add((blockCover, dutyType), i);
            }
        }

        private void resetLamdba(bool clear) {
            if (clear) {
                lambdaTrips = vss.Instance.Trips.Select(x => 0.0).ToList();
                lambdaBlocks = css.Blocks.Select(x => 0.0).ToList();
            }

            for (int i = 0; i < lambdaTrips.Count; i++) lambdaTrips[i] = 0;
            for (int i = 0; i < lambdaBlocks.Count; i++) lambdaBlocks[i] = 0;

            lambdaAvgDutyLength = 0;
            lambdaMaxLongDuty = 0;
            lambdaMaxBroken = 0;
            lambdaMaxBetween = 0;
        }

        private void initializeLagrangeModel() {
            resetLamdba(true);

            for (int i = 0; i < vss.Tasks.Count; i++) X.Add(0);
            for (int i = 0; i < css.Duties.Count; i++) Y.Add(0);
        }
        #endregion

        #region Lagrangean Updates
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

                            for (int j = 0; j < task.TripIndexCover.Count; j++)
                                cost -= lambdaTrips[task.TripIndexCover[j]];
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

                            cost += lambdaAvgDutyLength * ((double)duty.PaidDuration / Constants.CR_TARGET_SHIFT_LENGTH - 1);
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
            if (!objVal.converged || objVal.val > costUpperBound || objVal.val < 0)
                resetLamdba(false);

            int roundsWithoutImprovement = 0;
            double z_best = double.NegativeInfinity;
            List<double> bestLambdaTrips = [.. lambdaTrips];
            List<double> bestLambdaBlocks = [.. lambdaBlocks];
            double bestLambdaAvgDutyLength = lambdaAvgDutyLength;
            double bestLambdaMaxLongDuty = lambdaMaxLongDuty;
            double bestLambdaMaxBroken = lambdaMaxBroken;
            double bestLambdaMaxBetween = lambdaMaxBetween;

            double pi = Config.LAGRANGE_PI_START;

            taskReducedCosts = vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
            dutyReducedCosts = css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();

            double solve() {
                // Reset current solution value
                for (int i = 0; i < X.Count; i++) X[i] = 0;
                for (int i = 0; i < Y.Count; i++) Y[i] = 0;

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
                        X[targetTask.taskIndex] = 1;
                        cost += targetTask.C_i;
                        selectedTaskCount++;
                    }
                    else if (selectedTaskCount >= Config.MAX_VEHICLES && targetTask.C_i + taskSlackPenalty < 0) {
                        // Select as slack vehicle
                        X[targetTask.taskIndex] = 1;
                        cost += targetTask.C_i + taskSlackPenalty;
                        selectedTaskCount++;
                    }
                    else {
                        // No profit to be made anymore, stop
                        break;
                    }
                }

                // Block covering from x
                int[] blockCover = new int[css.Blocks.Count];
                for (int i = 0; i < X.Count; i++) {
                    if (X[i] == 0) continue;
                    foreach (int bi in vss.Tasks[i].BlockIndexCover) {
                        blockCover[bi]++;
                    }
                }

                // Determine \bar{y}_i
                int[] barY = new int[css.Duties.Count];
                for (int i = 0; i < Y.Count; i++) {
                    int bar = 1;
                    foreach (int bi in css.Duties[i].BlockIndexCover) {
                        bar = Math.Max(bar, blockCover[bi]);
                    }
                    barY[i] = bar;
                }

                // Crew
                dutyReducedCosts.Sort();
                int selectedDutyCount = 0;
                double crewSlackPenalty = Config.CR_OVER_MAX_COST;
                for (int i = 0; i < dutyReducedCosts.Count; i++) {
                    var targetDuty = dutyReducedCosts[i];

                    bool done = false;
                    for (int k = 0; k < barY[i] && !done; k++) {
                        if (selectedDutyCount < Config.MAX_DUTIES && targetDuty.C_j < 0) {
                            // Select normally
                            cost += targetDuty.C_j;
                            Y[targetDuty.dutyIndex]++;
                            selectedDutyCount++;
                        }
                        else if (selectedDutyCount >= Config.MAX_DUTIES && targetDuty.C_j + crewSlackPenalty < 0) {
                            // Select as slack vehicle
                            cost += targetDuty.C_j + crewSlackPenalty;
                            Y[targetDuty.dutyIndex]++;
                            selectedDutyCount++;
                        }
                        else {
                            done = true;
                        }
                    }
                    if (done) break;
                }

                return cost;
            }

            void updateMultipliers(double z_curr) {
                // Trips
                double[] GTrips = new double[lambdaTrips.Count];
                for (int i = 0; i < GTrips.Length; i++) GTrips[i] = 1; // b
                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (X[taskIndex] == 0) continue;
                    foreach (var tripIndex in vss.Tasks[taskIndex].TripIndexCover)
                        GTrips[tripIndex] -= 1;
                }

                // Blocks / duties
                double[] GBlocks = new double[lambdaBlocks.Count]; // b == 0
                double GAvgDutyLength = 0;
                double GMaxLongDuty = 0;
                double GMaxBroken = 0;
                double GMaxBetween = 0;

                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (X[taskIndex] == 0) continue;
                    foreach (var blockIndex in vss.Tasks[taskIndex].BlockIndexCover) {
                        GBlocks[blockIndex] -= 1;
                    }
                }
                for (int dutyIndex = 0; dutyIndex < Y.Count; dutyIndex++) {
                    if (Y[dutyIndex] == 0) continue;
                    var duty = css.Duties[dutyIndex];
                    foreach (var blockIndex in duty.BlockIndexCover) {
                        GBlocks[blockIndex] += Y[dutyIndex];
                    }

                    GAvgDutyLength += ((double)duty.PaidDuration / Constants.CR_TARGET_SHIFT_LENGTH - 1.0);
                    GMaxLongDuty -= (Constants.CR_MAX_OVER_LONG_DUTY - duty.IsLongDuty);
                    GMaxBroken -= (Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty);
                    GMaxBetween -= (Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty);
                }

                double GSquaredSum = 0;
                for (int i = 0; i < GTrips.Length; i++) {
                    if (lambdaTrips[i] == 0 && GTrips[i] < 0) GTrips[i] = 0;
                    GSquaredSum += GTrips[i] * GTrips[i];
                }
                for (int i = 0; i < GBlocks.Length; i++) {
                    // No g = 0 as blocks are an == constraint
                    GSquaredSum += GBlocks[i] * GBlocks[i];
                }
                if (GAvgDutyLength < 0 && lambdaAvgDutyLength == 0) GAvgDutyLength = 0;
                GSquaredSum += GAvgDutyLength * GAvgDutyLength;
                if (GMaxLongDuty < 0 && lambdaMaxLongDuty == 0) GMaxLongDuty = 0;
                GSquaredSum += GMaxLongDuty * GMaxLongDuty;
                if (GMaxBroken < 0 && lambdaMaxBroken == 0) GMaxBroken = 0;
                GSquaredSum += GMaxBroken * GMaxBroken;
                if (GMaxBetween < 0 && lambdaMaxBetween == 0) GMaxBetween = 0;
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

            void discardWorstColumns() {
                bool rcUpdateRequired = false;

                if (vss.Tasks.Count > Config.VCSP_MAX_TASKS_DURING) {
                    rcUpdateRequired = true;
                    taskReducedCosts.Sort();
                    // Determine tasks with worst reduced costs
                    HashSet<int> taskIndexesToRemove = new();
                    for (int j = Config.VCSP_MAX_TASKS_DURING; j < taskReducedCosts.Count; j++) {
                        if (!vss.Tasks[taskReducedCosts[j].taskIndex].IsUnit)
                            taskIndexesToRemove.Add(taskReducedCosts[j].taskIndex);
                    }

                    // Remove from known tasks
                    foreach (int vi in taskIndexesToRemove) {
                        VehicleTask vt = vss.Tasks[vi];
                        knownVTs.Remove((vt.ToTripBitArray(vss.Instance.Trips.Count), vt.ToBlockBitArray(css.Blocks.Count)));
                    }

                    // Remove from vss
                    vss.Tasks = vss.Tasks
                        .Where((vt, i) => !taskIndexesToRemove.Contains(i))
                        .Select((x, i) => {
                            x.Index = i;
                            knownVTs[(x.ToTripBitArray(vss.Instance.Trips.Count), x.ToBlockBitArray(css.Blocks.Count))] = i;
                            return x;
                        }).ToList();

                    // Update X/reduced costs 
                    X = X.Where((_, i) => !taskIndexesToRemove.Contains(i)).ToList();
                    taskReducedCosts = vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
                }
                if (css.Duties.Count > Config.VCSP_MAX_DUTIES_DURING) {
                    rcUpdateRequired = true;
                    dutyReducedCosts.Sort();
                    // Determine duties with worst reduced cost
                    HashSet<int> dutyIndexesToRemove = new();
                    for (int j = Config.VCSP_MAX_DUTIES_DURING; j < dutyReducedCosts.Count; j++) {
                        if (!css.Duties[dutyReducedCosts[j].dutyIndex].IsUnit)
                            dutyIndexesToRemove.Add(dutyReducedCosts[j].dutyIndex);
                    }

                    // Remove from known duties
                    foreach (int ci in dutyIndexesToRemove) {
                        CrewDuty cd = css.Duties[ci];
                        knownCDs.Remove((cd.ToBlockBitArray(css.Blocks.Count), (int)cd.Type));
                    }


                    // Remove from css
                    css.Duties = css.Duties
                        .Where((cd, i) => !dutyIndexesToRemove.Contains(i))
                        .Select((x, i) => {
                            x.Index = i;
                            knownCDs[(x.ToBlockBitArray(css.Blocks.Count), (int)x.Type)] = i;
                            return x;
                        }).ToList();

                    // Update Y/reduced costs
                    Y = Y.Where((_, i) => !dutyIndexesToRemove.Contains(i)).ToList();
                    dutyReducedCosts = css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();
                }

                if (rcUpdateRequired) updateReducedCosts();
            }

            while (pi >= Config.LAGRANGE_PI_END) {
                updateReducedCosts();
                double z_curr = solve();
                updateMultipliers(z_curr);

                if (z_curr > z_best) {
                    z_best = z_curr;
                    // Expensive!
                    bestLambdaTrips = [.. lambdaTrips];
                    bestLambdaBlocks = [.. lambdaBlocks];
                    bestLambdaAvgDutyLength = lambdaAvgDutyLength;
                    bestLambdaMaxLongDuty = lambdaMaxLongDuty;
                    bestLambdaMaxBroken = lambdaMaxBroken;
                    bestLambdaMaxBetween = lambdaMaxBetween;
                }
                else if (roundsWithoutImprovement >= Config.LAGRANGE_N) {
                    roundsWithoutImprovement = 0;
                    pi *= Config.LAGRANGE_PI_COOLING;
                }
                else {
                    roundsWithoutImprovement++;
                }
            }


            // Get back best solution
            lambdaTrips = [.. bestLambdaTrips];
            lambdaBlocks = [.. bestLambdaBlocks];
            lambdaAvgDutyLength = bestLambdaAvgDutyLength;
            lambdaMaxLongDuty = bestLambdaMaxLongDuty;
            lambdaMaxBroken = bestLambdaMaxBroken;
            lambdaMaxBetween = bestLambdaMaxBetween;
            objVal = (z_best, true);

            // Finalize reduced costs given the current set of lamdba parameters
            updateReducedCosts();

            // throw away the worst vehicle tasks / crew duties
            if (allowDiscard) discardWorstColumns();

            return true;
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
        #endregion

        #region Vehicle its 
        private void runVehicleIts(int round, bool disrupt = false) {
            int maxIts = round == 0 ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;
            VSPLabeling vspLabelingInstance = new(vss);

            void makeVTsNonNegativeRC() {
                if (!Config.VCSP_NONNEGATIVE_RC_VSP) return;

                // Apply heuristic based on that of Marcel which attempts to rescale lambda s.t. 
                // all currently known vehicle task columns do not have negative reduced cost.
                List<(double C_i, int taskIndex)> tasksToUpdate = [];
                for (int i = 0; i < taskReducedCosts.Count; i++) {
                    if (taskReducedCosts[i].C_i < 0 && X[i] > 0)
                        tasksToUpdate.Add(taskReducedCosts[i]);
                }
                for (int i = 0; i < tasksToUpdate.Count; i++) {
                    (double C_i, int taskIndex) = tasksToUpdate[i];
                    List<int> tripCover = vss.Tasks[taskIndex].TripIndexCover;
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
                Dictionary<Descriptor, double> blockDualCosts = new();
                Dictionary<DescriptorHalf, List<double>> blockDualCostsByStart = new();

                for (int bi = 0; bi < css.Blocks.Count; bi++) {
                    Block b = css.Blocks[bi];
                    Descriptor descriptor = b.Descriptor;
                    DescriptorHalf descriptorStart = descriptor.GetStart();

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

                int discardedColumns = 0, improvedColumns = 0;
                foreach ((double reducedCosts, VehicleTask newTask) in newColumns) {
                    //if (reducedCosts > 0) continue;
                    if (newTask == null) continue;

                    // Determine blocks for task; add any not yet known ones to css
                    List<Block> blocks = Block.FromVehicleTask(newTask);
                    foreach (Block b in blocks) {
                        Descriptor desc = b.Descriptor;
                        if (!knownBlocks.ContainsKey(desc)) {
                            knownBlocks[desc] = css.Blocks.Count;
                            css.AddBlock(b, 1);
                            Y.Add(0); //unit duty from block being added
                            lambdaBlocks.Add(0); // New block multiplier
                        }

                        int blockIndex = knownBlocks[desc];
                        newTask.BlockIndexCover.Add(blockIndex);
                    }

                    BitArray taskCover = newTask.ToTripBitArray(vss.Instance.Trips.Count);
                    BitArray blockCover = newTask.ToBlockBitArray(css.Blocks.Count);

                    // If we already know this column, check if it is an improvement
                    if (knownVTs.ContainsKey((taskCover, blockCover))) {
                        int vi = knownVTs[(taskCover, blockCover)];
                        if (vss.Tasks[vi].Cost <= newTask.Cost) { // todo fix other ref
                            // Skip new column
                            discardedColumns++;
                        }
                        else {
                            // Replace existing column
                            improvedColumns++;
                            newTask.Index = vi;
                            vss.Tasks[vi] = newTask;
                        }
                    }
                    else {
                        newTask.Index = vss.Tasks.Count;
                        vss.Tasks.Add(newTask);
                        X.Add(0);
                        knownVTs.Add((taskCover, blockCover), newTask.Index);
                    }
                }

                gradientDescent(!disrupt);
                Console.WriteLine($"{round}.V{currIt}\t{objVal.val:0.##}\t{X.Sum(x => x)}\t{X.Count}\t{Y.Sum(y => y)}\t{Y.Count}\t{newColumns.Count}\t{improvedColumns}\t{discardedColumns}");
            }
        }
        #endregion

        #region Crew its
        private void runCrewIts(int round, bool disrupt = false) {
            int rbi = 0;
            int maxIts = round == 0 ? Config.VCSP_CR_ITS_INIT : Config.VCSP_CR_ITS_ROUND;

            List<int> blockCount = new(css.Blocks.Count);
            for (int i = 0; i < css.Blocks.Count; i++) blockCount.Add(0);

            int updateBlockCount() {
                int total = 0;
                for (int i = 0; i < css.Blocks.Count; i++) blockCount[i] = 0;
                for (int i = 0; i < X.Count; i++) {
                    if (X[i] == 0) continue;

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
                    if (dutyReducedCosts[j].C_j < 0 && Y[j] > 0)
                        dutiesToUpdate.Add(dutyReducedCosts[j]);
                }
                for (int i = 0; i < dutiesToUpdate.Count; i++) {
                    (double C_j, int dutyIndex) = dutiesToUpdate[i];
                    CrewDuty duty = css.Duties[dutyIndex];
                    List<int> blockCover = duty.BlockIndexCover;
                    double delta = C_j / (
                        -blockCover.Count
                        - (duty.PaidDuration / Constants.CR_TARGET_SHIFT_LENGTH - 1)
                        + (Constants.CR_MAX_OVER_LONG_DUTY - duty.IsLongDuty)
                        + (Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty)
                        + (Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty)
                    );

                    for (int j = 0; j < blockCover.Count; j++) {
                        lambdaBlocks[blockCover[j]] += delta;
                    }
                    lambdaAvgDutyLength += delta;
                    lambdaMaxLongDuty += delta;
                    lambdaMaxBroken += delta;
                    lambdaMaxBetween += delta;
                }
            }

            void addRandomBlocks() {
                // Add some random blocks 
                for (int i = 0; i < css.Blocks.Count; rbi++, i++) {
                    // Block will be used as a basis for expanding
                    if (rbi % Config.VCSP_BLOCK_PER_X_ADD != 0) continue;

                    Block block = css.Blocks[i];

                    List<(double cost, bool atFront, Location location, int time)> expansions = [];

                    // Check if first element of block can be expanded
                    // Expansion as trip: add trip in front/at back (for now)
                    if (block.Elements[0].Type == BlockElementType.Trip || block.Elements[0].Type == BlockElementType.Deadhead) {
                        // Check if there is a trip that can be used to extend the block backwards
                        for (int ti = 0; ti < vss.Instance.Trips.Count; ti++) {
                            Trip t = vss.Instance.Trips[ti];
                            if (!t.StartLocation.HandoverAllowed || t.EndLocation != block.StartLocation || t.EndTime > block.StartTime) continue;

                            // Block transition allowed; can only be done with short idle
                            // and if there is no time for handover
                            int waitingTime = block.StartTime - t.EndTime;
                            bool transitionAllowed = Config.CR_MIN_SHORT_IDLE_TIME <= waitingTime && waitingTime < block.StartLocation.MinHandoverTime;
                            bool overallTimeAllowed = t.Duration + waitingTime + block.Duration <= Constants.MAX_STEERING_TIME;
                            if (!transitionAllowed || !overallTimeAllowed) continue;

                            // Check if this block is already known
                            Descriptor candidateDescriptor = new Descriptor(t.StartLocation, t.StartTime, block.EndLocation, block.EndTime);
                            if (knownBlocks.ContainsKey(candidateDescriptor)) continue;

                            expansions.Add((-lambdaTrips[t.Index], true, t.StartLocation, t.StartTime));
                        }

                        // Check if we can add an incoming deadhead; can only be done if were not directly followed
                        // by a deadhead (would be weird)
                        if (block.Elements[0].Type == BlockElementType.Trip) {
                            Location curr = block.Elements[0].StartLocation;
                            for (int li = 0; li < vss.Instance.Locations.Count; li++) {
                                DeadheadTemplate? dht = vss.LocationDHT[li][curr.Index];
                                if (dht == null || !dht.StartLocation.HandoverAllowed) continue;

                                Descriptor candidateDescriptor = new Descriptor(
                                    dht.StartLocation,
                                    block.StartTime - dht.Duration,
                                    block.EndLocation,
                                    block.EndTime
                                );
                                if (knownBlocks.ContainsKey(candidateDescriptor)) continue;
                                // Ignore driving costs
                                expansions.Add((0, true, dht.StartLocation, block.StartTime - dht.Duration));
                            }
                        }
                    }

                    if (block.Elements[^1].Type == BlockElementType.Trip || block.Elements[^1].Type == BlockElementType.Deadhead) {
                        // Same but forwards
                        for (int ti = 0; ti < vss.Instance.Trips.Count; ti++) {
                            Trip t = vss.Instance.Trips[ti];
                            if (!t.StartLocation.HandoverAllowed || t.StartLocation != block.EndLocation || block.EndTime > t.StartTime) continue;

                            // Block transition allowed; can only be done with short idle
                            // as break / long idle start a new block
                            int waitingTime = t.StartTime - block.EndTime;
                            bool transitionAllowed = Config.CR_MIN_SHORT_IDLE_TIME <= waitingTime && waitingTime <= block.EndLocation.MinHandoverTime;
                            bool overallTimeAllowed = t.Duration + waitingTime + block.Duration <= Constants.MAX_STEERING_TIME;
                            if (!transitionAllowed || !overallTimeAllowed) continue;

                            // Check if this block is already known
                            Descriptor candidateDescriptor = new Descriptor(block.StartLocation, block.StartTime, t.EndLocation, t.EndTime);
                            if (knownBlocks.ContainsKey(candidateDescriptor)) continue;

                            expansions.Add((-lambdaTrips[t.Index], false, t.EndLocation, t.EndTime));
                        }

                        // Check if we can add an incoming deadhead; can only be done if were not directly followed
                        // by a deadhead (would be weird)
                        if (block.Elements[^1].Type == BlockElementType.Trip) {
                            Location curr = block.Elements[^1].StartLocation;
                            for (int li = 0; li < vss.Instance.Locations.Count; li++) {
                                DeadheadTemplate? dht = vss.LocationDHT[curr.Index][li];
                                if (dht == null || !dht.StartLocation.HandoverAllowed) continue;

                                Descriptor candidateDescriptor = new Descriptor(
                                    block.StartLocation,
                                    block.StartTime,
                                    dht.EndLocation,
                                    block.EndTime + dht.Duration
                                );
                                if (knownBlocks.ContainsKey(candidateDescriptor)) continue;
                                // Ignore driving costs
                                expansions.Add((0, false, dht.EndLocation, block.EndTime + dht.Duration));
                            }
                        }
                    }

                    expansions = expansions.OrderBy((x) => x.cost).ToList();

                    List<(Descriptor desc, int idx)> descs = expansions.Select((x, i) => {
                        (_, bool front, Location l, int t) = x;
                        var desc = front
                            ? new Descriptor(l, t, block.EndLocation, block.EndTime)
                            : new Descriptor(block.StartLocation, block.StartTime, l, t);
                        return (desc, i);
                    }).ToList();

                    List<(Descriptor desc, int idx)> unknownDescs = descs.Where(x => !knownBlocks.ContainsKey(x.desc)).ToList();
                    (Descriptor desc, int idx)? chosenDescriptor = null;
                    if (descs.Count > 0) chosenDescriptor = descs[0];
                    if (unknownDescs.Count > 0) chosenDescriptor = unknownDescs[0];

                    if (!chosenDescriptor.HasValue) continue;

                    var expansion = expansions[chosenDescriptor.Value.idx];
                    Block newBlock = expansion.atFront
                        ? Block.FromDescriptor(expansion.location, expansion.time, block.EndLocation, block.EndTime)
                        : Block.FromDescriptor(block.StartLocation, block.StartTime, expansion.location, expansion.time);

                    knownBlocks[chosenDescriptor.Value.desc] = css.Blocks.Count;
                    css.AddBlock(newBlock, 1);
                    Y.Add(0); //unit duty from block being added
                    lambdaBlocks.Add(0); // New block multiplier
                }
            }

            Dictionary<string, double> crewDualCost() {
                Dictionary<string, double> res = lambdaBlocks.Select((l, i) => (Constants.CSTR_BLOCK_COVER + css.Blocks[i].Descriptor, -l)).ToDictionary();
                res[Constants.CSTR_CR_AVG_TIME] = -lambdaAvgDutyLength;
                res[Constants.CSTR_CR_LONG_DUTIES] = lambdaMaxLongDuty;
                res[Constants.CSTR_CR_BROKEN_DUTIES] = lambdaMaxBroken;
                res[Constants.CSTR_CR_BETWEEN_DUTIES] = lambdaMaxBetween;
                return res;
            }

            CSPLabeling cspLabelingInstance = new CSPLabeling(css);

            for (int it = 0; it < maxIts; it++) {
                makeCDsNonNegativeRC();
                if (disrupt) disruptMultipliers();
                updateReducedCosts();
                int totalBlocks = updateBlockCount();

                addRandomBlocks();

                var dualCost = crewDualCost();
                List<(double reducedCosts, CrewDuty newDuty)> newColumns = [];

                cspLabelingInstance.UpdateDualCosts(dualCost, 1);
                var res = cspLabelingInstance.GenerateDuties();
                foreach (var resCol in res) {
                    if (resCol.crewDuty != null) {
                        newColumns.Add(resCol);
                    }
                }

                int discardedColumns = 0, improvedColumns = 0;
                foreach ((double reducedCost, CrewDuty newDuty) in newColumns) {
                    if (newDuty == null) continue;

                    BitArray blockCover = newDuty.ToBlockBitArray(css.Blocks.Count);
                    int dutyType = (int)newDuty.Type;

                    // If we already know this column, skip adding it
                    if (knownCDs.ContainsKey((blockCover, dutyType))) {
                        int ci = knownCDs[(blockCover, dutyType)];
                        if (css.Duties[ci].Cost <= newDuty.Cost) {
                            // Skip new column
                            discardedColumns++;
                        }
                        else {
                            // Replace existing column
                            improvedColumns++;
                            newDuty.Index = ci;
                            css.Duties[ci] = newDuty;
                        }
                    }
                    else {
                        newDuty.Index = css.Duties.Count;
                        css.Duties.Add(newDuty);
                        Y.Add(0);
                        knownCDs.Add((blockCover, dutyType), newDuty.Index);
                    }

                }
                gradientDescent(!disrupt);
                Console.WriteLine($"{round}.C{it}\t{objVal.val:0.##}\t{X.Sum(x => x)}\t{X.Count}\t{Y.Sum(y => y)}\t{Y.Count}\t{newColumns.Count}\t{improvedColumns}\t{discardedColumns}");
            }
        }

        #endregion

        private void solveILP() {
            // Solve using ILP
            GRBEnv env = new() {
                OutputFlag = 0,
                LogToConsole = 0,
                LogFile = Path.Combine(Constants.RUN_LOG_FOLDER, "evcspcg_gurobi.log")
            };
            model = new(env);
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
            model.SetCallback(new CustomGRBCallback(model));

            // Add selection variables for vehicle tasks and crew duties, set initial solution
            List<GRBVar> taskVars = [], dutyVars = [];
            for (int i = 0; i < vss.Tasks.Count; i++) {
                GRBVar v = model.AddVar(0, GRB.INFINITY, vss.Tasks[i].Cost, GRB.BINARY, $"vt_{i}");
                v.Set(GRB.DoubleAttr.Start, 0);
                taskVars.Add(v);
            }
            for (int i = 0; i < css.Duties.Count; i++) {
                GRBVar v = model.AddVar(0, GRB.INFINITY, css.Duties[i].Cost, GRB.INTEGER, $"cd_{i}");
                v.Set(GRB.DoubleAttr.Start, 0);
                dutyVars.Add(v);
            }
            foreach (var task in vss.SelectedTasks)
                taskVars[task.Index].Set(GRB.DoubleAttr.Start, 1.0);
            foreach ((var duty, int count) in css.SelectedDuties)
                dutyVars[duty.Index].Set(GRB.DoubleAttr.Start, count);

            // Max selected vehicle tasks
            GRBLinExpr maxVehiclesExpr = new();
            GRBVar maxVehiclesSlack = model.AddVar(
                0,
                Config.VCSP_VH_CSTR_SLACK ? GRB.INFINITY : 0,
                Config.VCSP_SLACK_IN_FINAL_OBJ ? Config.VH_OVER_MAX_COST : 0,
                GRB.CONTINUOUS,
                "vehicle_slack"
            );
            foreach (GRBVar v in taskVars) {
                maxVehiclesExpr += v;
            }
            model.AddConstr(maxVehiclesExpr - maxVehiclesSlack <= Config.MAX_VEHICLES, Constants.CSTR_MAX_VEHICLES);


            // Trip cover by vehicle tasks
            foreach (Trip t in vss.Instance.Trips) {
                GRBLinExpr expr = new();
                for (int i = 0; i < vss.Tasks.Count; i++) {
                    if (vss.Tasks[i].TripIndexCover.Contains(t.Index)) {
                        expr.AddTerm(1, taskVars[i]);
                    }
                }
                char sense = Config.VSP_ALLOW_OVERCOVER ? GRB.GREATER_EQUAL : GRB.EQUAL;
                string name = Constants.CSTR_TRIP_COVER + t.Index;
                model.AddConstr(expr, sense, 1, name);
            }

            // Block cover of selected tasks by duties
            for (int blockIndex = 0; blockIndex < css.Blocks.Count; blockIndex++) {
                Descriptor blockDescriptor = css.Blocks[blockIndex].Descriptor;
                GRBLinExpr expr = new();

                for (int vtIndex = 0; vtIndex < taskVars.Count; vtIndex++) {
                    if (vss.Tasks[vtIndex].BlockDescriptorCover.Contains(blockDescriptor))
                        expr += taskVars[vtIndex];
                }

                for (int cdIndex = 0; cdIndex < dutyVars.Count; cdIndex++) {
                    if (css.Duties[cdIndex].BlockDescriptorCover.Contains(blockDescriptor))
                        expr -= dutyVars[cdIndex];
                }

                string name = Constants.CSTR_BLOCK_COVER + blockDescriptor;
                model.AddConstr(expr <= 0, name);
            }

            GRBLinExpr maxDutiesExpr = new();
            GRBVar maxDutiesSlack = model.AddVar(
                0,
                Config.VCSP_CR_MAX_CSTR_SLACK ? GRB.INFINITY : 0,
                Config.VCSP_SLACK_IN_FINAL_OBJ ? Config.CR_OVER_MAX_COST : 0,
                GRB.CONTINUOUS,
                "duty_slack"
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
            Dictionary<Descriptor, int> vehicleBlockCover = [];
            Dictionary<Descriptor, int> crewBlockCover = [];
            HashSet<Descriptor> blockDescriptors = [];
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

        public override bool Solve() {
            initializeCover();
            initializeLagrangeModel();

            // Initialize dual cost
            Console.WriteLine($"I\t{costUpperBound:0.##}\t{X.Sum(x => x)}\t{X.Count}\t{Y.Sum(y => y)}\t{Y.Count}");
            gradientDescent();

            int round = 0;
            for (; round < Config.VCSP_ROUNDS; round++) {
                runVehicleIts(round);
                runCrewIts(round);

                Console.WriteLine($"{round}\t{objVal.val:0.##}\t{X.Sum(x => x)}\t{X.Count}\t{Y.Sum(y => y)}\t{Y.Count}");
            }

            if (Config.LAGRANGE_DISRUPT_ROUNDS < 1) {
                Console.WriteLine("Skipping disruption rounds");
            }
            else {
                Console.WriteLine($"Doing {Config.LAGRANGE_DISRUPT_ROUNDS} additional round{((Config.LAGRANGE_DISRUPT_ROUNDS > 1) ? "s" : "")} with disrupted lambda");
                for (int i = 0; i < Config.LAGRANGE_DISRUPT_ROUNDS; i++) {
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
