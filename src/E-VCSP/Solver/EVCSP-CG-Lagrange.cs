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

        Dictionary<string, int> knownBlocks = [];

        private List<double> lambdaTrips = [];
        private Dictionary<string, double> lambdaBlocks = [];
        private double lambdaAvgDutyLength = 0;
        private double lambdaMaxLongDuty = 0;
        //private double lambdaMaxBroken = 0;
        //private double lambdaMaxBetween = 0;
        private List<bool> X = []; // Select vt at index
        private List<bool> Y = []; // Select cd at index

        private double costUpperBound = double.MaxValue;
        private double objVal = double.MaxValue;

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

            // Create initial set of crew duties
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
                return x;
            }).ToList();
        }

        private void initializeUpperBound() {
            // Determine upper bound
            costUpperBound = 0;
            foreach (var vt in vss.Tasks) costUpperBound += vt.Cost;
            foreach (var cd in css.Duties) costUpperBound += cd.Cost;

            objVal = costUpperBound;
        }

        private void resetLamdba(bool clear) {
            if (clear) {
                lambdaTrips = vss.Instance.Trips.Select(x => 0.0).ToList();
                lambdaBlocks = [];
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) lambdaTrips[i] = 0;
            for (int i = 0; i < css.Blocks.Count; i++) lambdaBlocks[css.Blocks[i].Descriptor] = 0;

            lambdaAvgDutyLength = 0;
            lambdaMaxLongDuty = 0;
            //lambdaMaxBroken = 0;
            //lambdaMaxBetween = 0;
        }

        private void initializeLagrangeModel() {
            resetLamdba(true);

            for (int i = 0; i < vss.Tasks.Count; i++) X.Add(false);
            for (int i = 0; i < css.Duties.Count; i++) Y.Add(false);
        }
        #endregion

        private bool gradientDescent() {
            resetLamdba(false);

            List<(double C_i, int taskIndex)> costByTaskIndex =
                vss.Tasks.Select((x, i) => (double.MaxValue, i)).ToList();
            List<(double C_j, int dutyIndex)> costByDutyIndex =
                css.Duties.Select((x, i) => (double.MaxValue, i)).ToList();

            void updateCostsWithMultipliers() {
                for (int i = 0; i < costByTaskIndex.Count; i++) {
                    int taskIndex = costByTaskIndex[i].taskIndex;
                    VehicleTask task = vss.Tasks[taskIndex];
                    double cost = task.Cost;

                    foreach (var tripIndex in task.TripCover)
                        cost -= lambdaTrips[tripIndex];
                    foreach (var blockDescriptor in task.BlockDescriptorCover)
                        cost += lambdaBlocks[blockDescriptor];

                    costByTaskIndex[i] = (cost, taskIndex);
                }

                for (int i = 0; i < costByDutyIndex.Count; i++) {
                    int dutyIndex = costByDutyIndex[i].dutyIndex;
                    CrewDuty duty = css.Duties[dutyIndex];
                    double cost = duty.Cost;

                    foreach (var blockDescriptor in duty.BlockDescriptorCover)
                        cost -= lambdaBlocks[blockDescriptor];

                    int isLongDuty = duty.Duration > Config.CR_LONG_SHIFT_LENGTH ? 1 : 0;
                    int isBetweenDuty = duty.Type == DutyType.Between ? 1 : 0;
                    int isBrokenDuty = duty.Type == DutyType.Broken ? 1 : 0;

                    cost += lambdaAvgDutyLength * ((double)duty.Duration / Config.CR_TARGET_SHIFT_LENGTH - 1);
                    cost += lambdaMaxLongDuty * (Config.CR_MAX_OVER_LONG_DUTY - isLongDuty);
                    //cost += lambdaMaxBroken * (Config.CR_MAX_BROKEN_SHIFTS - isBrokenDuty);
                    //cost += lambdaMaxBetween * (Config.CR_MAX_BETWEEN_SHIFTS - isBetweenDuty);

                    costByDutyIndex[i] = (cost, dutyIndex);
                }
            }

            double solve() {
                // Reset current solution value
                for (int i = 0; i < X.Count; i++)
                    X[i] = false;
                for (int i = 0; i < Y.Count; i++)
                    Y[i] = false;

                double cost = 0;
                // Seperate lambda term
                for (int i = 0; i < lambdaTrips.Count; i++)
                    cost += lambdaTrips[i];

                // Vehicles
                List<(double C_i, int taskIndex)> targetTasks = new();
                for (int i = 0; i < costByTaskIndex.Count; i++)
                    if (costByTaskIndex[i].C_i < 0) targetTasks.Add(costByTaskIndex[i]);
                var cappedTargetTasks = targetTasks.OrderBy(x => x.C_i).Take(Config.MAX_VEHICLES);
                foreach (var targetTask in cappedTargetTasks) {
                    cost += targetTask.C_i;
                    X[targetTask.taskIndex] = true;
                }

                // Crew
                costByDutyIndex.Sort();
                List<(double C_j, int dutyIndex)> cappedTargetDuties = new();

                int totalBetweenDuties = 0;
                Queue<(double C_j, int dutyIndex)> buffBetweenDuties = new();
                int totalBrokenDuties = 0;
                Queue<(double C_j, int dutyIndex)> buffBrokenDuties = new();
                for (
                    int i = 0;
                    i < costByDutyIndex.Count && cappedTargetDuties.Count < Config.MAX_DUTIES;
                    i++
                ) {
                    var candidate = costByDutyIndex[i];
                    if (candidate.C_j >= 0) continue;

                    var dutyType = css.Duties[candidate.dutyIndex].Type;
                    if (dutyType == DutyType.Broken) {
                        buffBrokenDuties.Enqueue(candidate);
                    }
                    else if (dutyType == DutyType.Between) {
                        buffBetweenDuties.Enqueue(candidate);
                    }
                    else {
                        cappedTargetDuties.Add(candidate);
                    }

                    // Check if we are allowed to add a queued duty; they always have priority over 
                    // something coming in a next iteration
                    // TODO: should be sorted still but oh well
                    if (cappedTargetDuties.Count < Config.MAX_DUTIES && buffBetweenDuties.Count > 0
                        && Config.CR_MAX_BETWEEN_SHIFTS >= (totalBetweenDuties + 1) / (cappedTargetDuties.Count + 1)
                    ) {
                        cappedTargetDuties.Add(buffBetweenDuties.Dequeue());
                        totalBetweenDuties++;
                    }
                    if (cappedTargetDuties.Count < Config.MAX_DUTIES && buffBrokenDuties.Count > 0
                        && Config.CR_MAX_BROKEN_SHIFTS >= (totalBrokenDuties + 1) / (cappedTargetDuties.Count + 1)
                    ) {
                        cappedTargetDuties.Add(buffBrokenDuties.Dequeue());
                        totalBrokenDuties++;
                    }
                }

                foreach (var targetDuty in cappedTargetDuties) {
                    cost += targetDuty.C_j;
                    Y[targetDuty.dutyIndex] = true;
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

                // Blocks
                Dictionary<string, double> GBlocks = []; // b == 0
                double GAvgDutyLength = 0;
                double GMaxLongDuty = 0;
                //double GMaxBroken = 0; 
                //double GMaxBetween = 0;
                for (int i = 0; i < css.Blocks.Count; i++) GBlocks[css.Blocks[i].Descriptor] = 0.0;
                for (int taskIndex = 0; taskIndex < X.Count; taskIndex++) {
                    if (!X[taskIndex]) continue;
                    foreach (var blockDescriptor in vss.Tasks[taskIndex].BlockDescriptorCover) {
                        GBlocks[blockDescriptor] += 1;
                    }
                }
                for (int dutyIndex = 0; dutyIndex < Y.Count; dutyIndex++) {
                    if (!Y[dutyIndex]) continue;
                    var duty = css.Duties[dutyIndex];
                    foreach (var blockDescriptor in duty.BlockDescriptorCover) {
                        GBlocks[blockDescriptor] -= 1;
                    }

                    int isLongDuty = duty.Duration > Config.CR_LONG_SHIFT_LENGTH ? 1 : 0;
                    int isBetweenDuty = duty.Type == DutyType.Between ? 1 : 0;
                    int isBrokenDuty = duty.Type == DutyType.Broken ? 1 : 0;

                    GAvgDutyLength += ((double)duty.Duration / Config.CR_TARGET_SHIFT_LENGTH - 1);
                    GMaxLongDuty -= (Config.CR_MAX_OVER_LONG_DUTY - isLongDuty);
                    //GMaxBroken -= (Config.CR_MAX_BROKEN_SHIFTS - isBrokenDuty);
                    //GMaxBetween -= (Config.CR_MAX_BETWEEN_SHIFTS - isBetweenDuty);
                }

                double GSquaredSum = 0;
                for (int i = 0; i < GTrips.Length; i++) GSquaredSum += GTrips[i] * GTrips[i];
                foreach (var g in GBlocks.Values) GSquaredSum += g * g;
                GSquaredSum += GAvgDutyLength * GAvgDutyLength;
                GSquaredSum += GMaxLongDuty * GMaxLongDuty;
                //GSquaredSum += GMaxBroken * GMaxBroken;
                //GSquaredSum += GMaxBetween * GMaxBetween;

                double T = Config.LAGRANGE_PI * (costUpperBound - z_curr) / GSquaredSum;

                for (int i = 0; i < lambdaTrips.Count; i++) {
                    // >= constraint
                    lambdaTrips[i] = Math.Max(0, lambdaTrips[i] + T * GTrips[i]);
                }
                foreach (string descriptor in lambdaBlocks.Keys) {
                    // == constraint
                    lambdaBlocks[descriptor] = lambdaBlocks[descriptor] + T * GBlocks[descriptor];
                }

                // >= constraint
                lambdaAvgDutyLength = Math.Max(0, lambdaAvgDutyLength + T * GAvgDutyLength);
                // <= constraints
                lambdaMaxLongDuty = Math.Min(0, lambdaMaxLongDuty - T * GMaxLongDuty);
                //lambdaMaxBroken = Math.Min(0, lambdaMaxBroken - T * GMaxBroken);
                //lambdaMaxBetween = Math.Min(0, lambdaMaxBetween - T * GMaxBetween);
            }

            double lastSolutionVal = costUpperBound;
            int roundsInThreshold = 0;
            bool converged = false;

            for (int i = 0; i < Config.LANGRANGE_MAX_ROUNDS && !converged; i++) {
                updateCostsWithMultipliers();
                double z_curr = solve();
                if (double.IsNaN(z_curr)) {
                    Console.WriteLine("Convergeert de verkeerde kant op");
                    throw new InvalidOperationException();
                }
                updateMultipliers(z_curr);

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

            objVal = lastSolutionVal;
            return converged;
        }

        private Dictionary<string, double> crewDualCost() {
            Dictionary<string, double> res = lambdaBlocks.Select(kv => (Constants.CSTR_BLOCK_COVER + kv.Key, kv.Value)).ToDictionary();
            res[Constants.CSTR_CR_AVG_TIME] = lambdaAvgDutyLength;
            res[Constants.CSTR_CR_LONG_DUTIES] = lambdaMaxLongDuty;
            res[Constants.CSTR_CR_BROKEN_DUTIES] = 0;
            res[Constants.CSTR_CR_BETWEEN_DUTIES] = 0;
            return res;
        }

        private void runVehicleIts(bool initial) {
            int maxIts = initial ? Config.VCSP_VH_ITS_INIT : Config.VCSP_VH_ITS_ROUND;
            VSPLabeling vspLabelingInstance = new(vss);


            void updateDualCosts() {
                Dictionary<string, double> blockDualCosts = new();
                Dictionary<string, List<double>> blockDualCostsByStart = new();

                foreach (var b in css.Blocks) {
                    string descriptor = b.Descriptor;
                    string descriptorStart = String.Join("#", descriptor.Split("#").Take(2));
                    if (blockDualCostsByStart.ContainsKey(descriptorStart))
                        blockDualCostsByStart[descriptorStart].Add(lambdaBlocks[descriptor]);
                    else blockDualCostsByStart[descriptorStart] =
                            [lambdaBlocks[descriptor]];
                }

                vspLabelingInstance.UpdateDualCosts(lambdaTrips, blockDualCosts, blockDualCostsByStart);
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
                            lambdaBlocks[desc] = 0;
                        }
                    }
                    vss.Tasks.Add(newTask);
                    X.Add(false);
                }

                gradientDescent();
            }
        }

        private void runCrewIts(bool initial) {
            int maxIts = initial ? Config.VCSP_CR_ITS_INIT : Config.VCSP_CR_ITS_ROUND;

            Dictionary<string, int> activeBlockCounts = [];
            for (int i = 0; i < X.Count; i++) {
                if (!X[i]) continue;

                VehicleTask task = vss.Tasks[i];
                for (int j = 0; j < task.BlockDescriptorCover.Count; j++) {
                    if (activeBlockCounts.ContainsKey(task.BlockDescriptorCover[j])) activeBlockCounts[task.BlockDescriptorCover[j]]++;
                    else activeBlockCounts[task.BlockDescriptorCover[j]] = 1;
                }
            }

            List<int> blockCount = css.Blocks.Select(x => activeBlockCounts.ContainsKey(x.Descriptor) ? activeBlockCounts[x.Descriptor] : 0).ToList();
            css.BlockCount = blockCount;

            CSPLabeling[] cspLabelingInstances = [.. Enumerable.Range(0, Config.VCSP_CR_INSTANCES).Select(_ => new CSPLabeling(css))];
            for (int it = 0; it < maxIts; it++) {
                List<(double reducedCosts, CrewDuty newDuty)> newColumns = new();

                Parallel.For(0, cspLabelingInstances.Length, (i) => {
                    cspLabelingInstances[i].UpdateDualCosts(crewDualCost(), 1);
                    var res = cspLabelingInstances[i].GenerateDuties();
                    foreach (var resCol in res) newColumns.Add(resCol);
                });

                foreach ((double reducedCost, CrewDuty newDuty) in newColumns) {
                    if (reducedCost > 0) continue;

                    newDuty.Index = css.Duties.Count;
                    css.Duties.Add(newDuty);
                    Y.Add(false);
                }
                gradientDescent();
            }
        }

        public override bool Solve(CancellationToken ct) {
            initializeCover();
            initializeUpperBound();
            initializeLagrangeModel();

            // Initialize dual cost
            bool initiallyConverged = gradientDescent();
            Console.WriteLine($"ObjVal: {objVal}, Converged: {initiallyConverged}");

            for (int i = 0; i < Config.VCSP_ROUNDS; i++) {
                // Generate new vehicle tasks
                runVehicleIts(false);

                // Generate new crew duties based on vehicle tasks in X
                runCrewIts(false);

                Console.WriteLine($"ObjVal: {objVal}");
            }

            // Solve using ILP
            GRBEnv env = new() {
                LogToConsole = 1,
                LogFile = Path.Combine(Config.RUN_LOG_FOLDER, "evcspcg_gurobi.log")
            };
            GRBModel model = new(env);
            model.Parameters.TimeLimit = Config.VCSP_SOLVER_TIMEOUT_SEC;
            model.Parameters.MIPFocus = 3; // upper bound
            model.Parameters.Presolve = 2; // aggresive presolve
            model.SetCallback(new CustomGRBCallback());
            ct.Register(() => {
                Console.WriteLine("Cancellation requested.");
                model.Terminate();
            });
            List<GRBVar> taskVars = [], dutyVars = [];
            for (int i = 0; i < vss.Tasks.Count; i++) {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, vss.Tasks[i].Cost, GRB.BINARY, name);
                taskVars.Add(v);
            }
            for (int i = 0; i < css.Duties.Count; i++) {
                string name = $"cd_{i}";
                GRBVar v = model.AddVar(0, 1, css.Duties[i].Cost, GRB.BINARY, name);
                dutyVars.Add(v);
            }

            // Max selected vehicle tasks
            GRBLinExpr maxVehiclesExpr = new();
            GRBVar maxVehiclesSlack = model.AddVar(0, GRB.INFINITY, Config.VH_OVER_MAX_COST, GRB.CONTINUOUS, "vehicle_slack");
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

            // Duty type / avg duration
            GRBLinExpr noExcessiveLength = new(); // max 15% > 8.5h
            GRBLinExpr limitedAverageLength = new(); // avg 8 hours
            GRBLinExpr maxBroken = new(); // max 30% broken
            GRBLinExpr maxBetween = new(); // max 10% between

            GRBVar noExcessiveLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "noExcessiveLengthSlack");
            GRBVar limitedAverageLengthSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "limitedAverageLengthSlack");
            GRBVar maxBrokenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBrokenSlack");
            GRBVar maxBetweenSlack = model.AddVar(0, GRB.INFINITY, 10000, GRB.CONTINUOUS, "maxBetweenSlack");

            for (int i = 0; i < dutyVars.Count; i++) {
                GRBVar v = dutyVars[i];
                CrewDuty duty = css.Duties[i];

                int duration = duty.Elements[^1].EndTime - duty.Elements[0].StartTime;
                noExcessiveLength += Config.CR_MAX_OVER_LONG_DUTY * v - (duration > Config.CR_LONG_SHIFT_LENGTH ? v : 0);
                limitedAverageLength += v * (duration / (double)Config.CR_TARGET_SHIFT_LENGTH - 1);
                maxBroken += v * Config.CR_MAX_BROKEN_SHIFTS - (duty.Type == DutyType.Broken ? v : 0);
                maxBetween += v * Config.CR_MAX_BETWEEN_SHIFTS - (duty.Type == DutyType.Between ? v : 0);
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

            return true;
        }
    }
}
