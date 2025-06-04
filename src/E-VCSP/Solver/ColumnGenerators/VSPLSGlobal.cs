using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    public class VSPLSGlobal : VehicleShortestPath
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
        /// Attempt to do a 2opt operation; Select a rnd time in the day, swap all trip assignments after that point
        /// </summary>
        private LSOpResult opt2()
        {
            int t1Index = rnd.Next(tasks.Count);
            int t2Index = rnd.Next(tasks.Count);
            while (t1Index == t2Index) t2Index = rnd.Next(tasks.Count);
            LLNode t1Head = tasks[t1Index];
            LLNode t2Head = tasks[t2Index];

            // Select a rnd point in time 
            int time = rnd.Next(StartTime + Config.MIN_NODE_TIME, EndTime - Config.MIN_NODE_TIME);

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
                if (t1FirstAffected == null && (curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip) && curr.VehicleElement.StartTime >= time)
                {
                    t1FirstAffected = curr;
                }

                if (curr.NodeType == LLNodeType.Trip)
                {
                    if (t1FirstAffected == null) t1TripsBefore++;
                    else t1TripsAfter++;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null)
            {
                if (t2FirstAffected == null && (curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip) && curr.VehicleElement.StartTime >= time)
                {
                    t2FirstAffected = curr;
                }

                if (curr.NodeType == LLNodeType.Trip)
                {
                    if (t2FirstAffected == null) t2TripsBefore++;
                    else t2TripsAfter++;
                }
                curr = curr.Next;
            }

            if (t1TripsAfter == 0 && t2TripsAfter == 0)
            {
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
            if (t1FirstAffected.NodeType == LLNodeType.Depot && t2FirstAffected.NodeType == LLNodeType.Depot) return LSOpResult.Invalid;

            // Trip 1 with new end                    idle  dh    trip/depot
            LLNode t1AdditionTarget = t1FirstAffected.Prev!.Prev!.Prev!;
            LLNode t2AdditionTarget = t2FirstAffected.Prev!.Prev!.Prev!;


            (bool feasible, double costDiff, LLNode? dh, LLNode? idle) glue(LLNode head, LLNode additionTarget, LLNode firstAffected)
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
                Location from = additionTargetAsTrip?.Trip?.From ?? Depot;
                VEDeadhead? ved = null;
                if (arc.Deadhead.ChargingActions.FindIndex(x => x.ChargeLocation == from) != -1)
                {
                    ved = new VEDeadhead(
                        arc.Deadhead,
                        additionTarget.VehicleElement.EndTime,
                        vehicleType,
                        arc.Deadhead.ChargingActions.FindIndex(x => x.ChargeLocation == from),
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
                LLNode prevDh = additionTarget.Next!;
                LLNode prevIdle = firstAffected.Prev!;

                // Current state: (?) is onesided link
                // tat2at -> ..... -> ........ -> t2fa -> ....
                //                              ?
                //       /-? dh1    -> idle1 -/  
                //      /                        
                // tat1at -> prevdh -> previdle -> t1fa -> ...
                // Now going to compare costs of both solutions; charging costs + driving costs
                double costDiff = 0;

                (bool originalValid, double originalChargeCosts) = head.CalcSoCValues(vehicleType, false);
                if (!originalValid) throw new InvalidOperationException("Something went wrong in previous operations; the original state was not valid");
                costDiff -= originalChargeCosts;
                costDiff -= additionTarget.CostOfTail();

                // Change tail
                additionTarget.Next = dh;
                firstAffected.Prev = idle;

                (bool changeValid, double newChargeCosts) = head.CalcSoCValues(vehicleType, false);
                // Continue with calculation either way; otherwise revert might be skipped
                costDiff += originalChargeCosts;
                costDiff += additionTarget.CostOfTail();

                // Revert changes
                additionTarget.Next = prevDh;
                firstAffected.Prev = prevIdle;

                if (!changeValid) return (false, double.MinValue, null, null);
                else return (true, costDiff, dh, idle);
            }

            var res1 = glue(t1Head, t1AdditionTarget, t2FirstAffected);
            var res2 = glue(t2Head, t2AdditionTarget, t1FirstAffected);

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
                    //break;
                }
                curr = curr.Next;
            }
            curr = t2Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
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
        private LSOpResult moveRange()
        {
            int t1Index = rnd.Next(tasks.Count);
            int t2Index = rnd.Next(tasks.Count);
            while (t1Index == t2Index) t2Index = rnd.Next(tasks.Count);
            LLNode t1Head = tasks[t1Index];
            LLNode t2Head = tasks[t2Index];

            // Get trips from t2 in order to attempt to move to t1
            List<LLNode> t2trips = new();
            LLNode? curr = t2Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip) t2trips.Add(curr);
                curr = curr.Next;
            }

            if (t2trips.Count == 0) throw new InvalidDataException("No trips in t2; should be at least 1");

            int t2StartIndex = rnd.Next(t2trips.Count),
                t2EndIndex = rnd.Next(t2StartIndex, t2trips.Count);

            LLNode t2Start = t2trips[t2StartIndex];
            LLNode t2End = t2trips[t2EndIndex];


            // Check if connecting the remainder of t2 is feasible
            LLNode t2Prev = t2Start.Prev!.Prev!.Prev!;
            LLNode t2Next = t2End.Next!.Next!.Next!;
            VETrip? t2PrevAsTrip = t2Prev.NodeType == LLNodeType.Trip ? (VETrip)t2Prev.VehicleElement : null,
                    t2NextAsTrip = t2Next.NodeType == LLNodeType.Trip ? (VETrip)t2Next.VehicleElement : null;
            Arc? t2PrevTot2Next = adjFull[t2PrevAsTrip?.Trip?.Index ?? instance.DepotStartIndex][t2NextAsTrip?.Trip?.Index ?? instance.DepotEndIndex];

            if (t2PrevTot2Next == null)
            {
                // Cannot repair t2 if trips are removed;
                return LSOpResult.Invalid;
            }

            // Check if addition of t2 range into t1 is time-feasible;
            // Can be checked by finding the trip preceding t2S in t1,
            // then checking the trip coming after that to see if it is possible
            // to travel from t2E to it.
            LLNode t1Prev = t1Head; // depot can be preceding
            curr = t1Head;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Trip)
                {
                    // Already found last vehicle element preceding
                    if (curr.VehicleElement.EndTime > t2Start.VehicleElement.StartTime) break;
                    else t1Prev = curr;
                }
                curr = curr.Next;
            }

            LLNode t1Next = t1Prev.Next!.Next!.Next!;
            VETrip t2StartRangeAsTrip = (VETrip)t2Start.VehicleElement,
                   t2EndRangeAsTrip = (VETrip)t2End.VehicleElement;
            VETrip? t1PrevAsTrip = t1Prev.NodeType == LLNodeType.Trip ? (VETrip)t1Prev.VehicleElement : null,
                    t1NextAsTrip = t1Next.NodeType == LLNodeType.Trip ? (VETrip)t1Next.VehicleElement : null;

            Arc? t1PrevTot2Start = adjFull[t1PrevAsTrip?.Trip?.Index ?? instance.DepotStartIndex][t2StartRangeAsTrip.Trip.Index];
            Arc? t2EndTot1Next = adjFull[t2EndRangeAsTrip.Trip.Index][t1NextAsTrip?.Trip?.Index ?? instance.DepotEndIndex];

            if (t1PrevTot2Start == null || t2EndTot1Next == null)
            {
                // Cannot connect the two trip parts
                return LSOpResult.Invalid;
            }

            // Get new connecting pieces ready
            // Previous state: 
            // t1: ... -> t1Prev -(currDh1, currIdle1)> t1Next -> ...
            // t2: ... -> t2Prev -(currDh2Prev, currIdle2Prev)> t2Start -> ... -> t2End -(currDh2Next, currIdle2Next)> t2Next -> ...
            // Target state: 
            // t1: ... -> t1Prev -(newDh2Start, newIdle2Start)> t2Start -> ... -> t2End -(newDh2End, newIdle2End)> t1Next -> ...
            // t2: ... -> t2Prev -(newDh2, newIdle2)> t2Next -> ...

            // Copies of previous dh/idle
            LLNode currDh1 = t1Prev.Next!,
                currIdle1 = t1Next.Prev!,
                currDh2Prev = t2Prev.Next!,
                currIdle2Prev = t2Start.Prev!,
                currDh2Next = t2End.Next,
                currIdle2Next = t2Next.Prev!;

            // New glue
            // t1Prev -> t2Start
            VEDeadhead vedNewDh2Start = new(t1PrevTot2Start.Deadhead, t1Prev.VehicleElement.EndTime, vehicleType);
            LLNode newDh2Start = new()
            {
                NodeType = LLNodeType.Deadhead,
                Prev = t1Prev,
                SoCAtStart = t1Prev.SoCAtEnd,
                SoCAtEnd = t1Prev.SoCAtEnd + vedNewDh2Start.SoCDiff,
                VehicleElement = vedNewDh2Start,
            };
            LLNode newIdle2Start = newDh2Start.AddAfter(LLNodeType.Idle, new VEIdle(vedNewDh2Start.Deadhead.DeadheadTemplate.To, vedNewDh2Start.EndTime, t2Start.VehicleElement.StartTime, vehicleType));
            newIdle2Start.Next = t2Start;
            // t2End -> t1Next
            VEDeadhead vedNewDh2End = new(t2EndTot1Next.Deadhead, t2End.VehicleElement.EndTime, vehicleType);
            LLNode newDh2End = new()
            {
                NodeType = LLNodeType.Deadhead,
                Prev = t2End,
                SoCAtStart = t2End.SoCAtEnd,
                SoCAtEnd = t2End.SoCAtEnd + vedNewDh2End.SoCDiff,
                VehicleElement = vedNewDh2End,
            };
            LLNode newIdle2End = newDh2End.AddAfter(LLNodeType.Idle, new VEIdle(vedNewDh2End.Deadhead.DeadheadTemplate.To, vedNewDh2End.EndTime, t1Next.VehicleElement.StartTime, vehicleType));
            newIdle2End.Next = t1Next;
            // t2Prev -> t2Next
            VEDeadhead vedNewDh2 = new(t2PrevTot2Next.Deadhead, t2Prev.VehicleElement.EndTime, vehicleType);
            LLNode newDh2 = new()
            {
                NodeType = LLNodeType.Deadhead,
                Prev = t2Prev,
                SoCAtStart = t2Prev.SoCAtEnd,
                SoCAtEnd = t2Prev.SoCAtEnd + vedNewDh2.SoCDiff,
                VehicleElement = vedNewDh2,
            };
            LLNode newIdle2 = newDh2.AddAfter(LLNodeType.Idle, new VEIdle(vedNewDh2.Deadhead.DeadheadTemplate.To, vedNewDh2.EndTime, t2Next.VehicleElement.StartTime, vehicleType));
            newIdle2.Next = t2Next;


            // Find previous costs
            double costDiff = 0;
            (bool t1OriginalValid, double t1OriginalChargeCosts) = t1Head.CalcSoCValues(vehicleType, false);
            (bool t2OriginalValid, double t2OriginalChargeCosts) = t2Head.CalcSoCValues(vehicleType, false);
            if (!t1OriginalValid || !t2OriginalValid) throw new InvalidOperationException("Something went wrong in previous operations; the original state was not valid");
            costDiff -= t1OriginalChargeCosts;
            costDiff -= t1Head.CostOfTail();
            costDiff -= t2OriginalChargeCosts;
            costDiff -= t2Head.CostOfTail();

            // Perform operation 
            t1Prev.Next = newDh2Start;
            t1Next.Prev = newIdle2End;
            t2Start.Prev = newIdle2Start;
            t2End.Next = newDh2End;
            t2Prev.Next = newDh2;
            t2Next.Prev = newIdle2;

            // Add new costs
            (bool t1NewValid, double t1NewChargeCosts) = t1Head.CalcSoCValues(vehicleType, false);
            (bool t2NewValid, double t2NewChargeCosts) = t2Head.CalcSoCValues(vehicleType, false);
            costDiff += t1NewChargeCosts;
            costDiff += t1Head.CostOfTail();
            costDiff += t2NewChargeCosts;
            costDiff += t2Head.CostOfTail();

            // TODO: kan verwerkt worden in eerdere check
            bool t2Remove = t2Head.TailCount() <= 4;
            if (t2Remove)
            {
                costDiff -= tasks.Count > Config.MAX_VEHICLES
                    ? Config.MAX_VEHICLES_OVER_COST + Config.PULLOUT_COST // TODO: check if this cost is correct
                    : Config.PULLOUT_COST;
            }

            // Check acceptance
            if (!t1NewValid || !t2NewValid || !accept(costDiff))
            {
                // Restore original structue
                t1Prev.Next = currDh1;
                t1Next.Prev = currIdle1;
                t2Prev.Next = currDh2Prev;
                t2Start.Prev = currIdle2Prev;
                t2End.Next = currDh2Next;
                t2Next.Prev = currIdle2Next;

                (bool v1, _) = t1Head.CalcSoCValues(vehicleType, false);
                (bool v2, _) = t2Head.CalcSoCValues(vehicleType, false);
                if (!v1 || !v2) throw new InvalidDataException("heh");

                return LSOpResult.Decline;
            }

            if (t2Remove) tasks.RemoveAt(t2Index);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult changeChargeAction()
        {
            int tIndex = rnd.Next(tasks.Count);
            LLNode head = tasks[tIndex];
            LLNode? curr = head;
            List<LLNode> chargeTargets = [];
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Deadhead && ((VEDeadhead)curr.VehicleElement).Deadhead.ChargingActions.Count > 0)
                {
                    chargeTargets.Add(curr);
                }
                curr = curr.Next;
            }

            if (chargeTargets.Count == 0) return LSOpResult.Invalid;

            LLNode target = chargeTargets[rnd.Next(chargeTargets.Count)];
            VEDeadhead dh = (VEDeadhead)target.VehicleElement;

            // Chose new action
            int currAction = dh.SelectedAction;
            int newAction = currAction;
            while (newAction == currAction)
                newAction = rnd.Next(-1, dh.Deadhead.ChargingActions.Count);
            ChargingAction? action = newAction == -1 ? null : dh.Deadhead.ChargingActions[newAction];

            // Determine new charge
            int currChargeTime = dh.ChargeTime;
            int newChargeTime = action == null ? 0 : action.TimeAtLocation;
            int newTotalTime = action == null
                ? dh.Deadhead.DeadheadTemplate.Duration
                : action.DrivingTimeTo + action.DrivingTimeFrom + newChargeTime;

            double newChargeGained = 0;
            double newDrivingCost = dh.Deadhead.BaseDrivingCost;
            double currSoCDiff = dh.SoCDiff;
            double newSoCDiff = -dh.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage;
            if (action != null)
            {
                double SoCAtCharger = target.SoCAtStart - action.ChargeUsedTo;
                newChargeGained = action.ChargeLocation.ChargingCurves[vehicleType.Index]
                    .MaxChargeGained(SoCAtCharger, newChargeTime, false).SoCGained;
                newDrivingCost = action.DrivingCost;
                newSoCDiff = newChargeGained - (action.ChargeUsedFrom + action.ChargeUsedTo);
            }

            double costDiff = 0;

            costDiff -= dh.DrivingCost;
            costDiff += newDrivingCost;

            // Old SoC related costs
            (bool currFeasible, double currChargingCost) = head.CalcSoCValues(vehicleType, false);
            if (!currFeasible) throw new InvalidDataException("Current charging scheme wasn't valid");
            costDiff -= currChargingCost;

            // perform op
            dh.SelectedAction = newAction;
            dh.ChargeTime = newChargeTime;
            dh.SoCDiff = newSoCDiff;

            // New SoC related costs
            (bool newFeasible, double newChargingCost) = head.CalcSoCValues(vehicleType, false);
            costDiff += newChargingCost;

            // TODO: idle costs moeten eigenlijk ook meegenomen worden aangezien de dh tijd verandert

            if (!newFeasible || !accept(costDiff))
            {
                // Revert operation
                dh.SelectedAction = currAction;
                dh.ChargeTime = currChargeTime;
                dh.SoCDiff = currSoCDiff;
                (bool oldFeasible, _) = head.CalcSoCValues(vehicleType, false);
                if (!oldFeasible) throw new InvalidOperationException("hoe dan");

                return LSOpResult.Decline;
            }

            // Finalize operation with everything not needed for SoC calculations
            dh.DrivingCost = newDrivingCost;
            dh.SoCGained = newChargeGained;
            dh.EndTime = dh.StartTime + newTotalTime;
            target.Next!.VehicleElement.StartTime = dh.EndTime;

            return costDiff < 0 ? LSOpResult.Improvement : LSOpResult.Accept;
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

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (opt2, Config.VSP_LS_G_2OPT),
                (moveRange, Config.VSP_LS_G_MOVE_RANGE),
                (changeChargeAction, Config.VSP_LS_G_CHANGE_CHARGE),
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

                for (int i = 0; i < tasks.Count; i++)
                {
                    var t = tasks[i];
                    var tailcount = t.TailCount();
                    if (tailcount <= 4)
                    {
                        throw new InvalidOperationException("Invalid state");
                    }

                    (bool feasible, _) = t.CalcSoCValues(vehicleType, false);
                    if (!feasible)
                    {
                        throw new InvalidOperationException("Invalid state");
                    }
                }
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
