using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.ColumnGenerators;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    internal class EVCSPCGLagrange : Solver {
        public readonly VehicleSolutionState vss;
        public readonly CrewSolutionState css;

        private readonly Random random = new();

        private Dictionary<string, int> knownBlocks = [];

        private List<bool> X = []; // Select vt at index
        private int vehicleSlack = 0; // Amount of over slack we are going
        private List<bool> Y = []; // Select cd at index
        private int maxDutiesSlack = 0; // Amount of over slack we are going

        /// <summary> >= 0 \forall i </summary>
        private List<double> lambdaTrips = [];
        /// <summary> \in R \forall j </summary>
        private List<double> lambdaBlocks = [];
        /// <summary> >= 0 </summary>
        private double lamdbaMaxVehicles = 0;
        /// <summary> >= 0 </summary>
        private double lamdbaMaxDuties = 0;
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
            costUpperBound += Math.Max(0, vss.Tasks.Count - Config.MAX_DUTIES) * Config.VH_OVER_MAX_COST;

            objVal = (costUpperBound, true);
        }

        private void resetLamdba(bool clear) {
            if (clear) {
                lambdaTrips = vss.Instance.Trips.Select(x => 0.0).ToList();
                lambdaBlocks = css.Blocks.Select(x => 0.0).ToList();
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) lambdaTrips[i] = 0;
            lamdbaMaxVehicles = 0;
            for (int i = 0; i < css.Blocks.Count; i++) lambdaBlocks[css.Blocks[i].Index] = 0;
            lamdbaMaxDuties = 0;

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

        private bool gradientDescent() {
            if (!objVal.converged) resetLamdba(false);

            double alpha = Math.Pow(Config.LAGRANGE_PI_END / Config.LAGRANGE_PI_START, 1.0 / Config.LANGRANGE_MAX_ROUNDS);
            double pi = Config.LAGRANGE_PI_START;

            List<(double C_i, int taskIndex)> costByTaskIndex =
                vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
            List<(double C_j, int dutyIndex)> costByDutyIndex =
                css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();

            void updateCostsWithMultipliers() {
                // Cost for selection of vehicle task vt
                for (int i = 0; i < costByTaskIndex.Count; i++) {
                    int taskIndex = costByTaskIndex[i].taskIndex;
                    VehicleTask task = vss.Tasks[taskIndex];
                    double cost = task.Cost;

                    foreach (var tripIndex in task.TripCover)
                        cost -= lambdaTrips[tripIndex];
                    foreach (var blockIndex in task.BlockIndexCover)
                        cost -= lambdaBlocks[blockIndex];
                    cost += lamdbaMaxVehicles;

                    costByTaskIndex[i] = (cost, taskIndex);
                }

                // Cost for selection of crew task 
                for (int i = 0; i < costByDutyIndex.Count; i++) {
                    int dutyIndex = costByDutyIndex[i].dutyIndex;
                    CrewDuty duty = css.Duties[dutyIndex];

                    double cost = duty.Cost;

                    foreach (var blockIndex in duty.BlockIndexCover)
                        cost += lambdaBlocks[blockIndex];

                    cost += lambdaAvgDutyLength * ((double)duty.Duration / Constants.CR_TARGET_SHIFT_LENGTH - 1);
                    cost -= lambdaMaxLongDuty * (Constants.CR_MAX_OVER_LONG_DUTY - duty.IsLongDuty);
                    cost -= lambdaMaxBroken * (Constants.CR_MAX_BROKEN_SHIFTS - duty.IsBrokenDuty);
                    cost -= lambdaMaxBetween * (Constants.CR_MAX_BETWEEN_SHIFTS - duty.IsBetweenDuty);

                    costByDutyIndex[i] = (cost, dutyIndex);
                }
            }

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
                costByTaskIndex.Sort();
                int selectedTaskCount = 0;
                double taskSlackPenalty = Config.VH_OVER_MAX_COST - lamdbaMaxVehicles;
                for (int i = 0; i < costByTaskIndex.Count; i++) {
                    var targetTask = costByTaskIndex[i];
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
                costByDutyIndex.Sort();
                int selectedDutyCount = 0;
                double crewSlackPenalty = Config.CR_OVER_MAX_COST - lamdbaMaxDuties;
                for (int i = 0; i < costByDutyIndex.Count; i++) {
                    var targetDuty = costByDutyIndex[i];

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

                // Vehicle slack
                double GMaxVehicles = -vehicleSlack - Config.MAX_VEHICLES;
                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (X[taskIndex]) GMaxVehicles++;
                }

                // Blocks / duties
                double[] GBlocks = new double[lambdaBlocks.Count]; // b == 0
                double GMaxDuties = -maxDutiesSlack - Config.MAX_DUTIES;
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
                    GMaxDuties++;
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
                GSquaredSum += GMaxVehicles * GMaxVehicles;
                for (int i = 0; i < GBlocks.Length; i++) GSquaredSum += GBlocks[i] * GBlocks[i];
                GSquaredSum += GMaxDuties * GMaxDuties;
                GSquaredSum += GAvgDutyLength * GAvgDutyLength;
                GSquaredSum += GMaxLongDuty * GMaxLongDuty;
                GSquaredSum += GMaxBroken * GMaxBroken;
                GSquaredSum += GMaxBetween * GMaxBetween;

                double T = pi * (costUpperBound - z_curr) / GSquaredSum;

                // Everything >= / == constraint
                for (int i = 0; i < lambdaTrips.Count; i++) {
                    lambdaTrips[i] = Math.Max(0, lambdaTrips[i] + T * GTrips[i]);
                }
                lamdbaMaxVehicles = Math.Max(0, lamdbaMaxVehicles + T * GMaxVehicles);

                for (int i = 0; i < lambdaBlocks.Count; i++) {
                    // == constraint
                    lambdaBlocks[i] = lambdaBlocks[i] + T * GBlocks[i];
                }
                lamdbaMaxDuties = Math.Max(0, lamdbaMaxDuties + T * GMaxDuties);

                lambdaAvgDutyLength = Math.Max(0, lambdaAvgDutyLength + T * GAvgDutyLength);
                lambdaMaxLongDuty = Math.Max(0, lambdaMaxLongDuty + T * GMaxLongDuty);
                lambdaMaxBroken = Math.Max(0, lambdaMaxBroken + T * GMaxBroken);
                lambdaMaxBetween = Math.Max(0, lambdaMaxBetween + T * GMaxBetween);
            }

            double lastSolutionVal = costUpperBound;
            int roundsInThreshold = 0;
            bool converged = false;

            int i = 0;
            for (; i < Config.LANGRANGE_MAX_ROUNDS && !converged; i++) {
                updateCostsWithMultipliers();
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

            // throw away the worst vehicle tasks / crew duties
            if (vss.Tasks.Count > Config.VCSP_MAX_TASKS_DURING_SOLVE) {
                costByTaskIndex.Sort();
                HashSet<int> indicesToRemove = new();
                for (int j = Config.VCSP_MAX_TASKS_DURING_SOLVE; j < costByTaskIndex.Count; j++) {
                    if (!vss.Tasks[costByTaskIndex[j].taskIndex].IsUnit)
                        indicesToRemove.Add(costByTaskIndex[j].taskIndex);
                }
                vss.Tasks = vss.Tasks.Where((vt, i) => !indicesToRemove.Contains(i)).Select((x, i) => { x.Index = i; return x; }).ToList();
                X = X.Where((_, i) => !indicesToRemove.Contains(i)).ToList();
            }
            if (css.Duties.Count > Config.VCSP_MAX_DUTIES_DURING_SOLVE) {
                costByDutyIndex.Sort();
                HashSet<int> indicesToRemove = new();
                for (int j = Config.VCSP_MAX_DUTIES_DURING_SOLVE; j < costByDutyIndex.Count; j++) {
                    if (!css.Duties[costByDutyIndex[j].dutyIndex].IsUnit)
                        indicesToRemove.Add(costByDutyIndex[j].dutyIndex);
                }
                css.Duties = css.Duties.Where((cd, i) => !indicesToRemove.Contains(i)).Select((x, i) => { x.Index = i; return x; }).ToList();
                Y = Y.Where((_, i) => !indicesToRemove.Contains(i)).ToList();
            }

            objVal = (lastSolutionVal, converged);

            return converged;
        }

        private Dictionary<string, double> crewDualCost() {
            Dictionary<string, double> res = lambdaBlocks.Select((l, i) => (Constants.CSTR_BLOCK_COVER + css.Blocks[i].Descriptor, -l)).ToDictionary();
            res[Constants.CSTR_CR_AVG_TIME] = lambdaAvgDutyLength;
            res[Constants.CSTR_CR_LONG_DUTIES] = lambdaMaxLongDuty;
            res[Constants.CSTR_CR_BROKEN_DUTIES] = lambdaMaxBroken;
            res[Constants.CSTR_CR_BETWEEN_DUTIES] = lambdaMaxBetween;
            return res;
        }

        private void runVehicleIts(int round) {
            int maxIts = round == 0 ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;
            VSPLabeling vspLabelingInstance = new(vss);

            void updateDualCosts() {
                Dictionary<string, double> blockDualCosts = new();
                Dictionary<string, List<double>> blockDualCostsByStart = new();

                for (int i = 0; i < css.Blocks.Count; i++) {
                    Block b = css.Blocks[i];
                    string descriptor = b.Descriptor;
                    string descriptorStart = String.Join("#", descriptor.Split("#").Take(2));

                    blockDualCosts[descriptor] = lambdaBlocks[i];
                    if (blockDualCostsByStart.ContainsKey(descriptorStart))
                        blockDualCostsByStart[descriptorStart].Add(lambdaBlocks[i]);
                    else blockDualCostsByStart[descriptorStart] =
                            [lambdaBlocks[i]];
                }

                // TODO: validate if this is correct
                vspLabelingInstance.UpdateDualCosts(
                    lambdaTrips,
                    blockDualCosts,
                    blockDualCostsByStart
                );
            }

            for (int currIt = 0; currIt < maxIts; currIt++) {
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

                gradientDescent();
                Console.WriteLine($"{round}.V{currIt}\t{objVal.val:0.##}\t{objVal.converged}\t{X.Count(x => x)}\t{Y.Count(y => y)}\t{newColumns.Count}");
            }
        }

        private void runCrewIts(int round) {
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

            CSPLabeling[] cspLabelingInstances = [.. Enumerable.Range(0, Config.VCSP_CR_INSTANCES).Select(_ => new CSPLabeling(css))];
            for (int it = 0; it < maxIts; it++) {
                int totalBlocks = updateBlockCount();
                var dualCost = crewDualCost();
                List<(double reducedCosts, CrewDuty newDuty)> newColumns = [];

                // 
                for (int i = 0; i < css.Blocks.Count; i++) {
                    // Block will be used as a basis for expanding
                    if (css.BlockCount[i] == 0) continue;
                    if (random.Next() >= Config.VSCP_BLOCK_ADD_CHANCE * css.BlockCount[i]) continue;

                    Block block = css.Blocks[i];

                    // Check if first element of block can be expanded
                    if (block.Elements[0].Type == BlockElementType.Deadhead) {

                    }
                }

                Parallel.For(0, cspLabelingInstances.Length, (i) => {
                    cspLabelingInstances[i].UpdateDualCosts(dualCost, 1);
                    var res = cspLabelingInstances[i].GenerateDuties();
                    foreach (var resCol in res)
                        if (resCol.crewDuty != null) newColumns.Add(resCol);
                });

                foreach ((double reducedCost, CrewDuty newDuty) in newColumns) {
                    if (reducedCost >= 0) continue;

                    newDuty.Index = css.Duties.Count;
                    css.Duties.Add(newDuty);
                    Y.Add(false);
                }
                gradientDescent();
                Console.WriteLine($"{round}.C{it}\t{objVal.val:0.##}\t{objVal.converged}\t{X.Count(x => x)}\t{Y.Count(y => y)}\t{newColumns.Count}");
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
            model.Parameters.MIPFocus = 3; // upper bound
            model.Parameters.Presolve = 2; // aggresive presolve
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

            int slackCosts = 10000;
            double maxSlack = Config.VCSP_CR_OTH_CSTR_SLACK ? 0 : GRB.INFINITY;

            GRBVar noExcessiveLengthSlack = model.AddVar(0, maxSlack, slackCosts, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, maxSlack, slackCosts, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, maxSlack, slackCosts, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, maxSlack, slackCosts, GRB.CONTINUOUS, "maxBetweenSlack");

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
                if (taskVars[i].X != 1) continue;
                vss.SelectedTasks.Add(vss.Tasks[i]);
            }
            for (int i = 0; i < dutyVars.Count; i++) {
                if (dutyVars[i].X != 1) continue;
                css.SelectedDuties.Add(css.Duties[i]);
            }

            Console.WriteLine($"Solution found with {vss.SelectedTasks.Count} vehicles, {css.SelectedDuties.Count} duties, overall costs {model.ObjVal}");
            Console.WriteLine("Cost breakdown:");
            Console.WriteLine($"Vehicle tasks: {vss.SelectedTasks.Sum(x => x.Cost)}");
            Console.WriteLine($"Crew duties: {css.SelectedDuties.Sum(x => x.Cost)}");
            Console.WriteLine($"Penalties:");
            Console.WriteLine($"\tVehicle slack: {maxVehiclesSlack.X * Config.VH_OVER_MAX_COST}");
            Console.WriteLine($"\tDuty slack: {maxDutiesSlack.X * Config.CR_OVER_MAX_COST}");
            Console.WriteLine($"\tSingle shifts: {css.SelectedDuties.Sum(x => x.Type == DutyType.Single ? 1 : 0) * Config.CR_SINGLE_SHIFT_COST} (included in duty costs)");
            Console.WriteLine($"\tAvg duty length slack: {limitedAverageLengthSlack.X * slackCosts}");
            Console.WriteLine($"\tLong duty slack: {noExcessiveLengthSlack.X * slackCosts}");
            Console.WriteLine($"\tBroken duty slack: {maxBrokenSlack.X * slackCosts}");
            Console.WriteLine($"\tBetween duty slack: {maxBetweenSlack.X * slackCosts}");
        }

        public override bool Solve(CancellationToken ct) {
            initializeCover();
            initializeUpperBound();
            initializeLagrangeModel();

            // Initialize dual cost
            gradientDescent();
            Console.WriteLine($"I\t{objVal.val:0.##}\t{objVal.converged}\t{X.Count(x => x)}\t{Y.Count(y => y)}");

            for (int round = 0; round < Config.VCSP_ROUNDS; round++) {
                runVehicleIts(round);
                runCrewIts(round);

                Console.WriteLine($"{round}\t{objVal.val:0.##}\t{objVal.converged}\t{X.Count(x => x)}\t{Y.Count(y => y)}");
            }

            solveILP();

            return true;
        }
    }
}
