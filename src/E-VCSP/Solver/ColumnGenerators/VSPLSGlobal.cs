using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators {
    public class VSPLSGlobal(GRBModel model, VehicleSolutionState vss) : VehicleColumnGen(model, vss) {
        private LSOperations ops = new(vss, Config.VSP_LS_G_STARTING_T, "LS Global");
        private List<VSPLSNode> tasks = new();
        private List<double> reducedCostsTrip = new();
        private List<List<DeadheadTemplate?>> locationDHT = [];
        private Random rnd = new();

        private double T;
        private double alpha;
        private int Q;

        private void reset() {
            T = Config.VSP_LS_G_STARTING_T;
            alpha = Config.VSP_LS_G_COOLING_RATE;
            Q = (int)Math.Round(-Config.VSP_LS_G_ITERATIONS / (Math.Log(Config.VSP_LS_G_STARTING_T / Config.VSP_LS_G_ENDING_T) / Math.Log(alpha)));
            ops = new(vss, T, "LS Global");

            tasks.Clear();
            reducedCostsTrip.Clear();

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < vss.Instance.Trips.Count; i++) {
                reducedCostsTrip.Add(constrs[i].Pi);
            }

            // Generate initial set of routes
            for (int i = 0; i < vss.Instance.Trips.Count; i++) {
                Trip t = vss.Instance.Trips[i];
                if (vss.Depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
                VSPArc depotTripArc = vss.AdjFull[vss.Instance.DepotStartIndex][i] ?? throw new InvalidDataException("No depot trip arc found in initial LSGLobal");
                VSPArc tripDepotArc = vss.AdjFull[i][vss.Instance.DepotEndIndex] ?? throw new InvalidDataException("No trip depot arc found in initial LSGLobal");

                // Create depot -> dh1 -> idle1 -> trip -> dh2 -> idle2 -> depot
                VSPLSNode depotStart = new() {
                    PVE = new PVEDepot(vss.Depot, vss.StartTime - Config.MIN_NODE_TIME, vss.StartTime),
                };
                VSPLSNode travel1 = depotStart.AddAfter(new PVETravel(depotTripArc.DeadheadTemplate, vss.StartTime, t.StartTime, vss.VehicleType));
                VSPLSNode trip = travel1.AddAfter(new PVETrip(t, vss.VehicleType));
                VSPLSNode travel2 = trip.AddAfter(new PVETravel(tripDepotArc.DeadheadTemplate, t.EndTime, vss.EndTime, vss.VehicleType));
                VSPLSNode depotEnd = travel2.AddAfter(new PVEDepot(vss.Depot, vss.EndTime, vss.EndTime + Config.MIN_NODE_TIME));

                tasks.Add(depotStart);
            }
        }

        /// <summary>
        /// Attempt to do a 2opt operation; Select a rnd time in the day, swap all trip assignments after that point
        /// </summary>
        private LSOpResult opt2(VSPLSNode? _) {
            int t1Index = rnd.Next(tasks.Count);
            int t2Index = rnd.Next(tasks.Count);
            while (t1Index == t2Index) t2Index = rnd.Next(tasks.Count);
            VSPLSNode t1Head = tasks[t1Index];
            VSPLSNode t2Head = tasks[t2Index];

            // Select a random point in time 
            int time = rnd.Next(vss.StartTime, vss.EndTime);

            // Go through the tasks; find the first trip after the crossover point
            //                  time
            // d -> t1 -> t2 ... -> tat1 | tfa1 -> ... -> d 
            // d -> t1 -> ...... -> tat2 | tfa2 -> ... -> d
            VSPLSNode? t1FirstAffected = null;
            VSPLSNode? t2FirstAffected = null;
            int t1TripsBefore = 0, t1TripsAfter = 0, t2TripsBefore = 0, t2TripsAfter = 0;

            VSPLSNode? curr = t1Head;
            while (curr != null) {
                if (t1FirstAffected == null && (curr.PVE.Type == PVEType.Depot || curr.PVE.Type == PVEType.Trip) && curr.PVE.StartTime >= time) {
                    t1FirstAffected = curr;
                }

                if (curr.PVE.Type == PVEType.Trip) {
                    if (t1FirstAffected == null) t1TripsBefore++;
                    else t1TripsAfter++;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null) {
                if (t2FirstAffected == null && (curr.PVE.Type == PVEType.Depot || curr.PVE.Type == PVEType.Trip) && curr.PVE.StartTime >= time) {
                    t2FirstAffected = curr;
                }

                if (curr.PVE.Type == PVEType.Trip) {
                    if (t2FirstAffected == null) t2TripsBefore++;
                    else t2TripsAfter++;
                }
                curr = curr.Next;
            }

            if (t1TripsAfter == 0 && t2TripsAfter == 0) {
                // Skip; cant do anything interesting
                return LSOpResult.Invalid;
            }

            if (t1FirstAffected == null || t2FirstAffected == null)
                throw new InvalidDataException("Something went wrong; no depot / trips found after " + time);

            // Case like this may also exist; switching tail of task 1 to task 2 may be interesting, however if both dont have a tail
            // we can simply skip
            //                      time
            // d -> t1 -> ... tat1 -> | tfa1 -> ... -> d 
            // d -> t1 -> ... -> tat2 | -> d (=tfa2)
            if (t1FirstAffected.PVE.Type == PVEType.Depot && t2FirstAffected.PVE.Type == PVEType.Depot) return LSOpResult.Invalid;

            VSPLSNode? t1AdditionTarget = t1FirstAffected.FindFirstBefore(x => x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip);
            VSPLSNode? t2AdditionTarget = t2FirstAffected.FindFirstBefore(x => x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip);

            if (t1AdditionTarget == null || t2AdditionTarget == null) throw new InvalidDataException("Heh");

            (bool feasible, double costDiff, VSPLSNode? travel) glue(VSPLSNode head, VSPLSNode additionTarget, VSPLSNode firstAffected) {
                // Attempt to glue the two trip parts together; if this results in infeasibility / , return invalid
                PVETrip? additionTargetAsTrip = additionTarget.PVE.Type == PVEType.Trip ? ((PVETrip)additionTarget.PVE) : null;
                int fromIndex = additionTargetAsTrip?.Trip.Index ?? vss.Instance.DepotStartIndex;
                int toIndex = firstAffected.PVE.Type == PVEType.Trip
                    ? ((PVETrip)firstAffected.PVE).Trip.Index
                    : vss.Instance.DepotEndIndex;
                VSPArc? arc = vss.AdjFull[fromIndex][toIndex];
                if (arc == null) return (false, double.MinValue, null);

                // Arc exists; check if results in invalid charge by temporarily replacing at tail with fa (with travel to glue)
                Location from = additionTargetAsTrip?.Trip?.From ?? vss.Depot;

                VSPLSNode travel = new() {
                    Prev = additionTarget,
                    Next = firstAffected,
                    PVE = new PVETravel(arc.DeadheadTemplate, additionTarget.PVE.EndTime, firstAffected.PVE.StartTime, vss.VehicleType)
                };

                // save copies of previous "glue"
                VSPLSNode atNext = additionTarget.Next!;
                VSPLSNode faPrev = firstAffected.Prev!;

                // Current state: (?) is onesided link
                // tat2at -> ..... -> ........ -> t2fa -> ....
                //                              ?
                //       /-? dh1    -> idle1 -/  
                //      /                        
                // tat1at -> prevdh -> previdle -> t1fa -> ...
                // Now going to compare costs of both solutions; charging costs + driving costs
                double costDiff = 0;

                var originalRes = head.validateTail(vss.VehicleType);
                if (Config.VSP_LS_SHR_ALLOW_PENALTY) costDiff -= originalRes.penaltyCost;
                else {
                    if (originalRes.penaltyCost != 0) throw new Exception("Original state not valid");
                }
                costDiff -= originalRes.chargingCost;
                costDiff -= originalRes.drivingCost;

                // Change tail
                additionTarget.Next = travel;
                firstAffected.Prev = travel;

                var newRes = head.validateTail(vss.VehicleType);
                // Continue with calculation either way; otherwise revert might be skipped
                if (Config.VSP_LS_SHR_ALLOW_PENALTY) newRes.penaltyCost += costDiff;
                costDiff += newRes.chargingCost;
                costDiff += newRes.drivingCost;

                // Revert changes
                additionTarget.Next = atNext;
                firstAffected.Prev = faPrev;

                if (newRes.penaltyCost != 0) return (false, double.MinValue, null);
                else return (true, costDiff, travel);
            }

            var res1 = glue(t1Head, t1AdditionTarget, t2FirstAffected);
            var res2 = glue(t2Head, t2AdditionTarget, t1FirstAffected);

            if (!res1.feasible || !res2.feasible) {
                // At least one glue failed
                return LSOpResult.Invalid;
            }

            double overallCostDiff = res1.costDiff + res2.costDiff;
            // Extra savings if amount of used vehicles would be reduced
            if (t1TripsBefore + t2TripsAfter == 0 || t2TripsBefore + t1TripsAfter == 0) {
                overallCostDiff -= tasks.Count > Config.MAX_VEHICLES
                    ? Config.VH_OVER_MAX_COST + Config.VH_PULLOUT_COST // TODO: check if this cost is correct
                    : Config.VH_PULLOUT_COST;
            }

            if (!accept(overallCostDiff)) {
                return LSOpResult.Decline;
            }

            // Redo-add new glue to current state
            t1AdditionTarget.Next = res1.travel;
            t2FirstAffected.Prev = res1.travel;
            t2AdditionTarget.Next = res2.travel;
            t1FirstAffected.Prev = res2.travel;

            // Check if one of the two tasks is now empty as a result of our operations; if so, discard.
            bool t1HasTrip = false;
            bool t2HasTrip = false;
            curr = t1Head;
            while (curr != null) {
                if (curr.PVE.Type == PVEType.Trip) {
                    t1HasTrip = true;
                    //break;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null) {
                if (curr.PVE.Type == PVEType.Trip) {
                    t2HasTrip = true;
                    //break;
                }
                curr = curr.Next;
            }

            if (!t1HasTrip && !t2HasTrip)
                throw new InvalidOperationException("Something went wrong; somehow both tasks dont have trips anymore");
            if (!t1HasTrip)
                tasks.RemoveAt(t1Index);
            if (!t2HasTrip)
                tasks.RemoveAt(t2Index);

            return (overallCostDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        /// <summary>
        /// Select two tasks, select a range of entries from the one to try and insert into the other
        /// </summary>
        private LSOpResult moveRange(VSPLSNode? _) {
            int t1Index = rnd.Next(tasks.Count);
            int t2Index = rnd.Next(tasks.Count);
            while (t1Index == t2Index) t2Index = rnd.Next(tasks.Count);
            VSPLSNode t1Head = tasks[t1Index];
            VSPLSNode t2Head = tasks[t2Index];

            // Get trips from t2 in order to attempt to move to t1
            List<VSPLSNode> t2trips = new();
            VSPLSNode? curr = t2Head;
            while (curr != null) {
                if (curr.PVE.Type == PVEType.Trip) t2trips.Add(curr);
                curr = curr.Next;
            }

            if (t2trips.Count == 0) throw new InvalidDataException("No trips in t2; should be at least 1");

            int t2StartIndex = rnd.Next(t2trips.Count),
                t2EndIndex = rnd.Next(t2StartIndex, t2trips.Count);

            VSPLSNode t2Start = t2trips[t2StartIndex];
            VSPLSNode t2End = t2trips[t2EndIndex];

            if (t2Start == null || t2End == null) throw new InvalidDataException("could not get trip nodes");

            // Check if connecting the remainder of t2 is feasible
            VSPLSNode? t2Prev = t2Start.FindFirstBefore(x => x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip);
            VSPLSNode? t2Next = t2End.FindFirstAfter(x => x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip);

            if (t2Prev == null || t2Next == null) throw new InvalidDataException("heh");

            PVETrip? t2PrevAsTrip = t2Prev.PVE.Type == PVEType.Trip ? (PVETrip)t2Prev.PVE : null,
                    t2NextAsTrip = t2Next.PVE.Type == PVEType.Trip ? (PVETrip)t2Next.PVE : null;
            VSPArc? t2PrevTot2Next = vss.AdjFull[t2PrevAsTrip?.Trip?.Index ?? vss.Instance.DepotStartIndex][t2NextAsTrip?.Trip?.Index ?? vss.Instance.DepotEndIndex];

            if (t2PrevTot2Next == null) {
                // Cannot repair t2 if trips are removed;
                return LSOpResult.Invalid;
            }

            // Check if addition of t2 range into t1 is time-feasible;
            // Can be checked by finding the trip preceding t2S in t1,
            // then checking the trip coming after that to see if it is possible
            // to travel from t2E to it.
            VSPLSNode? t1Prev = t1Head.FindLastAfter(x => x.PVE.EndTime < t2Start.PVE.StartTime && (x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip)) ?? t1Head;
            if (t1Prev == null) throw new InvalidOperationException("heh");
            VSPLSNode? t1Next = t1Prev.FindFirstAfter(x => x.PVE.Type == PVEType.Depot || x.PVE.Type == PVEType.Trip);
            if (t1Next == null) throw new InvalidOperationException("heh");
            PVETrip t2StartRangeAsTrip = (PVETrip)t2Start.PVE,
                    t2EndRangeAsTrip = (PVETrip)t2End.PVE;
            PVETrip? t1PrevAsTrip = t1Prev.PVE.Type == PVEType.Trip ? (PVETrip)t1Prev.PVE : null,
                    t1NextAsTrip = t1Next.PVE.Type == PVEType.Trip ? (PVETrip)t1Next.PVE : null;

            VSPArc? t1PrevTot2Start = vss.AdjFull[t1PrevAsTrip?.Trip?.Index ?? vss.Instance.DepotStartIndex][t2StartRangeAsTrip.Trip.Index];
            VSPArc? t2EndTot1Next = vss.AdjFull[t2EndRangeAsTrip.Trip.Index][t1NextAsTrip?.Trip?.Index ?? vss.Instance.DepotEndIndex];

            if (t1PrevTot2Start == null || t2EndTot1Next == null) {
                // Cannot connect the two trip parts
                return LSOpResult.Invalid;
            }

            // Get new connecting pieces ready
            // Previous state: 
            // t1: ... -> t1Prev -(t1PrevTravel)> ... -(t1NextTravel)> t1Next -> ...
            // t2: ... -> t2Prev -(t2PrevTravel)> ... -(t2StartTravel)> t2Start -> ... -> t2End -(t2EndTravel)> ... -(t2NextTravel)> t2Next -> ...
            // Target state: 
            // t1: ... -> t1Prev -(newStartTravel)> t2Start -> ... -> t2End -(newEndTravel)> t1Next -> ...
            // t2: ... -> t2Prev -(newTravel)> t2Next -> ...

            // Copies of previous dh/idle
            VSPLSNode t1PrevTravel = t1Prev.Next!,
                    t1NextTravel = t1Next.Prev!,
                    t2PrevTravel = t2Prev.Next!,
                    t2StartTravel = t2Start.Prev!,
                    t2EndTravel = t2End.Next!,
                    t2NextTravel = t2Next.Prev!;

            // New glue
            // t1Prev -> t2Start
            VSPLSNode newStartTravel = new() {
                PVE = new PVETravel(t1PrevTot2Start.DeadheadTemplate, t1Prev.PVE.EndTime, t2Start.PVE.StartTime, vss.VehicleType),
                Prev = t1Prev,
                Next = t2Start,
            };
            // t2End -> t1Next
            VSPLSNode newEndTravel = new() {
                PVE = new PVETravel(t2EndTot1Next.DeadheadTemplate, t2End.PVE.EndTime, t1Next.PVE.StartTime, vss.VehicleType),
                Prev = t2End,
                Next = t1Next,
            };
            // t2Prev -> t2Next
            VSPLSNode newTravel = new() {
                PVE = new PVETravel(t2PrevTot2Next.DeadheadTemplate, t2Prev.PVE.EndTime, t2Next.PVE.StartTime, vss.VehicleType),
                Prev = t2Prev,
                Next = t2Next,
            };

            // Find previous costs
            double costDiff = 0;
            var t1OriginalRes = t1Head.validateTail(vss.VehicleType);
            var t2OriginalRes = t2Head.validateTail(vss.VehicleType);
            if (Config.VSP_LS_SHR_ALLOW_PENALTY) {
                costDiff -= t1OriginalRes.penaltyCost;
                costDiff -= t2OriginalRes.penaltyCost;
            }
            else {
                if (t1OriginalRes.penaltyCost != 0 || t2OriginalRes.penaltyCost != 0)
                    throw new InvalidOperationException("Something went wrong in previous operations; the original state was not valid");
            }
            costDiff -= t1OriginalRes.drivingCost;
            costDiff -= t2OriginalRes.drivingCost;
            costDiff -= t1OriginalRes.chargingCost;
            costDiff -= t2OriginalRes.chargingCost;

            // Perform operation 
            t1Prev.Next = newStartTravel;
            t2Start.Prev = newStartTravel;
            t2End.Next = newEndTravel;
            t1Next.Prev = newEndTravel;
            t2Prev.Next = newTravel;
            t2Next.Prev = newTravel;

            // Add new costs
            var t1NewRes = t1Head.validateTail(vss.VehicleType);
            var t2NewRes = t2Head.validateTail(vss.VehicleType);
            if (Config.VSP_LS_SHR_ALLOW_PENALTY) {
                costDiff += t1NewRes.penaltyCost;
                costDiff += t2NewRes.penaltyCost;
            }

            costDiff += t1NewRes.drivingCost;
            costDiff += t1NewRes.chargingCost;
            costDiff += t2NewRes.drivingCost;
            costDiff += t2NewRes.chargingCost;

            bool t2Remove = t2Head.TailCount() <= 3;
            if (t2Remove) {
                costDiff -= tasks.Count > Config.MAX_VEHICLES
                    ? Config.VH_OVER_MAX_COST + Config.VH_PULLOUT_COST// TODO: check if this cost is correct
                    : Config.VH_PULLOUT_COST;
            }

            // Check acceptance
            if ((!Config.VSP_LS_SHR_ALLOW_PENALTY && (t1NewRes.penaltyCost != 0 || t2NewRes.penaltyCost != 0)) || !accept(costDiff)) {
                // Restore original structue
                t1Prev.Next = t1PrevTravel;
                t1Next.Prev = t1NextTravel;
                t2Prev.Next = t2PrevTravel;
                t2Start.Prev = t2StartTravel;
                t2End.Next = t2EndTravel;
                t2Next.Prev = t2NextTravel;
                return LSOpResult.Decline;
            }

            if (t2Remove) tasks.RemoveAt(t2Index);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        /// <summary>
        /// Accept simulated annealing iteration
        /// </summary>
        /// <param name="deltaScore">score change</param>
        /// <returns>accept/deny</returns>
        private bool accept(double deltaScore) {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > rnd.NextDouble();
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks() {
            reset();

            List<(Func<VSPLSNode?, LSOpResult> operation, double chance)> operations = [
                (opt2, Config.VSP_LS_G_2OPT),
                (moveRange, Config.VSP_LS_G_MOVE_RANGE),
                (ops.addChargeStop, Config.VSP_LS_G_ADD_CHARGE),
                (ops.removeChargeStop, Config.VSP_LS_G_ADD_CHARGE),
        ];

            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);
            int itsPerColumn = (int)Config.VSP_LS_G_ITERATIONS / Config.VSP_LS_G_NUM_COLS;

            List<(double reducedCosts, VehicleTask vehicleTask)> generatedColumns = [];

            void dumpTasks() {
                generatedColumns.AddRange(tasks
                    .Select<VSPLSNode, (double reducedCost, VehicleTask vehicleTask)?>(taskHead => {
                        VehicleTask? vehicleTask = taskHead.ToVehicleTask(vss.VehicleType, "LS Global");
                        if (vehicleTask == null) return null;

                        double reducedCost = vehicleTask.Cost;
                        foreach (int coveredTripIndex in vehicleTask.TripCover) {
                            reducedCost -= reducedCostsTrip[coveredTripIndex];
                        }
                        return (reducedCost, vehicleTask);
                    })
                   .Where(x => x != null)
                   .Select(x => x!.Value));
            }

            int currIts = 0;
            while (currIts < Config.VSP_LS_G_ITERATIONS) {
                currIts++;
                if (currIts % Q == 0) {
                    T *= alpha;
                    ops.T = T;
                }
                if (currIts % itsPerColumn == 0) {

                }

                double r = rnd.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                int selectedIndex = rnd.Next(tasks.Count);
                VSPLSNode selectedHead = tasks[selectedIndex] ?? throw new InvalidDataException("selected head invalid");
                var res = operations[operationIndex].operation(selectedHead);
                if (operationIndex >= 2 && selectedHead.TailCount() <= 3) tasks.RemoveAt(selectedIndex);
            }

            dumpTasks();
            return generatedColumns;
        }
    }
}
