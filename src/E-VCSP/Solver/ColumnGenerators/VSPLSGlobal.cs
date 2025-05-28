using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class VSPLSGlobal : VehicleShortestPath
    {
        public VSPLSGlobal(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj) : base(model, instance, vehicleType, nodes, adjFull, adj)
        {
        }

        private List<LLNode> tasks = new();
        private List<double> reducedCostsTrip = new();
        private Random rnd = new();

        private double T;
        private double alpha;
        private int Q;

        private void reset()
        {
            T = Config.VSP_LS_G_STARTING_T;
            alpha = Config.VSP_LS_G_COOLING_RATE;
            Q = (int)Math.Round(-Config.VSP_LS_G_ITERATIONS / (Math.Log(Config.VSP_LS_G_STARTING_T / Config.VSP_LS_G_ENDING_T) / Math.Log(alpha)));

            tasks.Clear();
            reducedCostsTrip.Clear();

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCostsTrip.Add(constrs[i].Pi);
            }

            // Generate initial set of routes
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                Trip t = instance.Trips[i];
                if (Depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
                Arc depotTripArc = adjFull[instance.DepotStartIndex][i] ?? throw new InvalidDataException("No depot trip arc found in initial LSGLobal");
                Arc tripDepotArc = adjFull[i][instance.DepotEndIndex] ?? throw new InvalidDataException("No trip depot arc found in initial LSGLobal");

                // Create depot -> dh1 -> idle1 -> trip -> dh2 -> idle2 -> depot
                LLNode depotStart = new()
                {
                    NodeType = LLNodeType.Depot,
                    VehicleElement = new VEDepot(Depot, StartTime - Config.MIN_NODE_TIME, StartTime),
                    SoCAtStart = vehicleType.StartCharge,
                    SoCAtEnd = vehicleType.StartCharge,
                };
                LLNode dh1 = depotStart.AddAfter(LLNodeType.Deadhead, new VEDeadhead(depotTripArc.Deadhead, StartTime, vehicleType));
                LLNode idle1 = dh1.AddAfter(LLNodeType.Idle, new VEIdle(t.From, dh1.VehicleElement.EndTime, t.StartTime, vehicleType));
                LLNode trip = idle1.AddAfter(LLNodeType.Trip, new VETrip(t, vehicleType));
                LLNode dh2 = trip.AddAfter(LLNodeType.Deadhead, new VEDeadhead(tripDepotArc.Deadhead, t.EndTime, vehicleType));
                LLNode idle2 = dh2.AddAfter(LLNodeType.Idle, new VEIdle(Depot, dh2.VehicleElement.EndTime, EndTime, vehicleType));
                LLNode depotEnd = idle2.AddAfter(LLNodeType.Depot, new VEDepot(Depot, EndTime, EndTime + Config.MIN_NODE_TIME));

                tasks.Add(depotStart);
            }
        }

        /// <summary>
        /// Attempt to do a 2opt operation; Select a random time in the day, swap all trip assignments after that point
        /// </summary>
        private LSOpResult opt2()
        {
            int t1Index = rnd.Next(tasks.Count);
            int t2Index = rnd.Next(tasks.Count);
            while (t1Index == t2Index) t2Index = rnd.Next(tasks.Count);
            LLNode t1Head = tasks[t1Index];
            LLNode t2Head = tasks[t2Index];

            // Select a random point in time 
            int time = rnd.Next(StartTime, EndTime);

            // Go through the tasks; find the first trip after the crossover point
            //                  time
            // d -> t1 -> t2 ... -> tat1 | tfa1 -> ... -> d 
            // d -> t1 -> ...... -> tat2 | tfa2 -> ... -> d
            LLNode? t1FirstAffected = null;
            LLNode? t2FirstAffected = null;
            int t1TripsBefore = 0, t1TripsAfter = 0, t2TripsBefore = 0, t2TripsAfter = 0;

            LLNode? curr = t1Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
                    if (t1FirstAffected == null) t1TripsBefore++;
                    else t1TripsAfter++;
                }

                if (t1FirstAffected == null && (curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip) && curr.VehicleElement.StartTime >= time)
                {
                    t1FirstAffected = curr;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
                    if (t2FirstAffected == null) t2TripsBefore++;
                    else t2TripsAfter++;
                }

                if (t2FirstAffected == null && (curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip) && curr.VehicleElement.StartTime >= time)
                {
                    t2FirstAffected = curr;
                }
                curr = curr.Next;
            }

            if (t1FirstAffected == null || t2FirstAffected == null)
                throw new InvalidDataException("Something went wrong; no depot / trips found after " + time);

            // Case like this may also exist; switching tail of task 1 to task 2 may be interesting, however if both dont have a tail
            // we can simply skip
            //                      time
            // d -> t1 -> ... tat1 -> | tfa1 -> ... -> d 
            // d -> t1 -> ... -> tat2 | -> d (=tfa2)
            if (t1FirstAffected.NodeType == LLNodeType.Depot && t2FirstAffected.NodeType == LLNodeType.Depot) return LSOpResult.Invalid;

            // Trip 1 with new end                    idle  dh    trip/depot
            LLNode t1AdditionTarget = t1FirstAffected.Prev!.Prev!.Prev!;
            LLNode t2AdditionTarget = t1FirstAffected.Prev!.Prev!.Prev!;


            (bool feasible, double costDiff, LLNode? dh, LLNode? idle) glue(LLNode additionTarget, LLNode firstAffected)
            {
                // Attempt to glue the two trip parts together; if this results in infeasible deadhead / charging issues, return invalid
                VETrip? additionTargetAsTrip = additionTarget.NodeType == LLNodeType.Trip ? ((VETrip)additionTarget.VehicleElement) : null;
                int fromIndex = additionTargetAsTrip?.Trip.Index ?? instance.DepotStartIndex;
                int toIndex = firstAffected.NodeType == LLNodeType.Trip
                    ? ((VETrip)firstAffected.VehicleElement).Trip.Index
                    : instance.DepotEndIndex;
                Arc? arc = adjFull[fromIndex][toIndex];
                if (arc == null) return (false, double.MinValue, null, null);

                // Arc exists; check if results in invalid charge by temporarily replacing at tail with fa (with dh, idle to glue
                Location from1 = additionTargetAsTrip?.Trip?.From ?? Depot;
                VEDeadhead? ved = null;
                if (arc.Deadhead.ChargingActions.FindIndex(x => x.ChargeLocation == from1) != -1)
                {
                    ved = new VEDeadhead(
                        arc.Deadhead,
                        additionTarget.VehicleElement.EndTime,
                        vehicleType,
                        arc.Deadhead.ChargingActions.FindIndex(x => x.ChargeLocation == from1),
                        additionTarget.SoCAtEnd
                    );
                }
                else
                {
                    ved = new VEDeadhead(
                        arc.Deadhead,
                        additionTarget.VehicleElement.EndTime,
                        vehicleType
                    );
                }

                LLNode dh = new()
                {
                    NodeType = LLNodeType.Deadhead,
                    Prev = additionTarget,
                    SoCAtStart = additionTarget.SoCAtEnd,
                    SoCAtEnd = additionTarget.SoCAtEnd + ved.SoCDiff,
                    VehicleElement = ved,
                };
                LLNode idle = dh.AddAfter(LLNodeType.Idle, new VEIdle(ved.Deadhead.DeadheadTemplate.To, ved.EndTime, firstAffected.VehicleElement.StartTime, vehicleType));
                idle.Next = firstAffected;

                // save copies of previous "glue"
                LLNode prevDh = t1AdditionTarget.Next!;
                LLNode prevIdle = t1AdditionTarget.Next!.Next!;

                // Current state: (?) is onesided link
                // tat2at -> ..... -> ........ -> t2fa -> ....
                //                              ?
                //       /-? dh1    -> idle1 -/  
                //      /                        
                // tat1at -> prevdh -> previdle -> t1fa -> ...
                // Now going to compare costs of both solutions; charging costs + driving costs
                double costDiff = 0;

                (bool originalValid, double originalChargeCosts) = t1AdditionTarget.CalcSoCValues(vehicleType, false);
                //if (!originalValid) throw new InvalidOperationException("Something went wrong in previous operations; the original state was not valid");
                costDiff -= originalChargeCosts;
                costDiff -= t1AdditionTarget.CostOfTail();

                // Change tail
                t1AdditionTarget.Next = dh;
                t2FirstAffected.Prev = idle;

                (bool changeValid, double newChargeCosts) = t1AdditionTarget.CalcSoCValues(vehicleType, false);
                // Continue with calculation either way; otherwise revert might be skipped
                costDiff += originalChargeCosts;
                costDiff += t1AdditionTarget.CostOfTail();

                // Revert changes
                t1AdditionTarget.Next = prevDh;
                t2FirstAffected.Prev = prevIdle;

                if (!changeValid) return (false, double.MinValue, null, null);
                else return (true, costDiff, dh, idle);
            }

            var res1 = glue(t1AdditionTarget, t2FirstAffected);
            var res2 = glue(t2AdditionTarget, t1FirstAffected);

            if (!res1.feasible || !res2.feasible)
            {
                // At least one glue failed
                return LSOpResult.Invalid;
            }


            double overallCostDiff = res1.costDiff + res2.costDiff;
            // Extra savings if amount of used vehicles would be reduced
            if (t1TripsBefore + t2TripsAfter == 0 || t2TripsBefore + t1TripsAfter == 0)
            {
                overallCostDiff -= tasks.Count > Config.MAX_VEHICLES
                    ? Config.MAX_VEHICLES_OVER_COST + Config.PULLOUT_COST // TODO: check if this cost is correct
                    : Config.PULLOUT_COST;
            }


            if (!accept(overallCostDiff))
            {
                return LSOpResult.Decline;
            }

            // Redo-add new glue to current state
            t1AdditionTarget.Next = res1.dh;
            t2FirstAffected.Prev = res1.idle;
            t2AdditionTarget.Next = res2.dh;
            t1FirstAffected.Prev = res2.idle;

            // Check if one of the two tasks is now empty as a result of our operations; if so, discard.
            bool t1HasTrip = false;
            bool t2HasTrip = false;
            curr = t1Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
                    t1HasTrip = true;
                    break;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
                    t2HasTrip = true;
                    break;
                }
                curr = curr.Next;
            }
            if (!t1HasTrip && !t2HasTrip) throw new InvalidOperationException("Something went wrong; somehow both tasks dont have trips anymore");

            if (!t1HasTrip)
                tasks.RemoveAt(t1Index);
            if (!t2HasTrip)
                tasks.RemoveAt(t2Index);

            return (overallCostDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }


        /// <summary>
        /// Accept simulated annealing iteration
        /// </summary>
        /// <param name="deltaScore">score change</param>
        /// <returns>accept/deny</returns>
        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > rnd.NextDouble();
        }

        internal override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (opt2, Config.VSP_LS_G_2OPT),
                //(removeTrip, Config.VSP_LS_G_REM_TRIP),
                //(changeChargeAction, Config.VSP_LS_G_CHANGE_CHARGE),
            ];

            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;
            while (currIts < Config.VSP_LS_G_ITERATIONS)
            {
                currIts++;
                if (currIts % Q == 0) T *= alpha;

                double r = rnd.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
            }

            return tasks.Select(taskHead =>
            {
                VehicleTask vehicleTask = taskHead.ToVehicleTask(vehicleType);
                double rc = vehicleTask.Cost;
                double reducedCost = vehicleTask.Cost;
                foreach (int coveredTripIndex in vehicleTask.Covers)
                {
                    reducedCost -= reducedCostsTrip[coveredTripIndex];
                }
                return (reducedCost, vehicleTask);
            }).ToList();
        }
    }
}
