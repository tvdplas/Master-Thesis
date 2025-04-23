using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver
{
    internal class LLNode
    {
        internal required VehicleElement VehicleElement;
        internal LLNode? Prev;
        internal LLNode? Next;
        internal double SoCAtStart;
        internal double SoCAtEnd;

        /// <summary>
        /// Determine the problems in this and all following nodes in the task.
        /// </summary>
        /// <returns>Array of peaks &gt; maxSoC and valleys &lt; minSoC. Each peak / valley determined by maximum error at a charging location.</returns>
        internal (double chargingCost, List<double> peaks, List<double> valleys) SoCError()
        {
            double chargingCost = CalcSoCValues();
            List<double> peaks = new(), valleys = new();
            LLNode? curr = this;
            while (curr != null)
            {
                // Peaks / valleys may occur at charging stations
                if (curr.VehicleElement is VEDeadhead ved && ved.SelectedAction != -1)
                {
                    ChargingAction ca = ved.Deadhead.ChargingActions[ved.SelectedAction];
                    double SoCBeforeCharge = curr.SoCAtStart - ca.ChargeUsedTo;
                    if (SoCBeforeCharge < ShortestPathLS.vehicleType!.MinCharge)
                        valleys.Add(ShortestPathLS.vehicleType!.MinCharge - SoCBeforeCharge);

                    ChargingCurve cc = ca.ChargeLocation.ChargingCurves[ShortestPathLS.vehicleType!.Id];
                    var chargeResult = cc.MaxChargeGained(SoCBeforeCharge, ved.ChargeTime, true);
                    double SoCAfterCharge = SoCBeforeCharge + chargeResult.SoCGained;

                    if (SoCAfterCharge > ShortestPathLS.vehicleType!.MaxCharge)
                        peaks.Add(SoCBeforeCharge - ShortestPathLS.vehicleType!.MaxCharge);
                }

                // Valley may also occur at the final depot visit
                if (curr.Next == null)
                {
                    if (curr.SoCAtEnd < ShortestPathLS.vehicleType!.MinCharge)
                        valleys.Add(ShortestPathLS.vehicleType!.MinCharge - curr.SoCAtEnd);
                }

                curr = curr.Next;
            }

            return (chargingCost, peaks, valleys);
        }

        /// <summary>
        /// Set the SoC values of the rest of the trip. Uses <see cref="SoCAtEnd"/> of <c>this</c> as base for propegation. 
        /// </summary>
        /// <returns>Charge costs over the rest of the trip</returns>
        internal double CalcSoCValues()
        {
            LLNode? curr = this;

            double chargeCosts = 0;
            while (curr != null)
            {
                if (curr.Next == null) break;
                if (curr.Next.VehicleElement is VEDeadhead ved && ved.SelectedAction != -1)
                {
                    // Recalculate amount of SoC gained as charge gained may depend on charge at start of action
                    ChargingAction ca = ved.Deadhead.ChargingActions[ved.SelectedAction];
                    ChargingCurve cc = ca.ChargeLocation.ChargingCurves[ShortestPathLS.vehicleType!.Id];
                    var chargeResult = cc.MaxChargeGained(curr.Next.SoCAtStart - ca.ChargeUsedTo, ved.ChargeTime, true);
                    ved.ChargeCost = chargeResult.Cost;
                    chargeCosts += chargeResult.Cost;
                    double chargeSoCGained = chargeResult.SoCGained;
                    double SoCDiff = chargeSoCGained - (ca.ChargeUsedFrom + ca.ChargeUsedTo);
                    curr.Next.VehicleElement.SoCDiff = SoCDiff;
                }

                curr.Next.SoCAtStart = curr.SoCAtEnd;
                curr.Next.SoCAtEnd = curr.Next.SoCAtStart + curr.Next.VehicleElement.SoCDiff;
                curr = curr.Next;
            }

            return chargeCosts;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>Node that was removed</returns>
        /// <exception cref="InvalidOperationException"></exception>
        internal LLNode? RemoveAfter()
        {
            if (Next == null) throw new InvalidOperationException("Can't remove with no next");

            LLNode? oldNext = Next;
            LLNode? newNext = Next.Next;
            Next = newNext;
            if (newNext != null)
            {
                newNext.Prev = this;
            }

            return oldNext;
        }

        /// <summary>
        /// Removes <paramref name="count"/> nodes after <c>this</c>. Does <b>not</b> update structure of removed nodes.
        /// </summary>
        /// <param name="count">Amount of nodes to remove</param>
        /// <returns>List of removed nodes</returns>
        internal List<LLNode?> RemoveAfter(int count)
        {
            List<LLNode?> removedNodes = new(count);
            LLNode? curr = Next;
            for (int i = 0; i < count; i++)
            {
                removedNodes.Add(curr);
                curr = curr.Next;
            }
            this.Next = curr;
            if (curr != null) curr.Prev = this;

            return removedNodes;
        }

        internal LLNode AddAfter(LLNode node)
        {
            if (node == null) throw new ArgumentNullException("Cant explicitely add null node");

            if (this.VehicleElement.EndTime != node.VehicleElement.StartTime) throw new ArgumentException("Vehicle task is not continuous");

            // Set next correctly
            if (this.Next != null)
            {
                this.Next.Prev = node;
            }

            // Update ordering
            node.Next = this.Next;
            node.Prev = this;
            this.Next = node;

            return node;
        }

        internal LLNode AddAfter(VehicleElement ve)
        {
            return AddAfter(new LLNode() { VehicleElement = ve });
        }

        public override string ToString()
        {
            LLNode? curr = this;
            int prev = 0, next = 0;
            //while (curr != null) { prev++; curr = curr.Prev; }
            //curr = this;
            //while (curr != null) { prev++; curr = curr.Next; }

            return $"{VehicleElement} (<{prev}, >{next})";
        }
    }

    internal enum LSOpResult
    {
        Improvement = 0,
        Accept = 1,
        Decline = 2,
        Invalid = 3,
        Count,
    }

    internal class ShortestPathLS
    {
        private Instance instance;
        private GRBModel model;
        private List<double> reducedCostsTrips;

        private double T;
        private double alpha;
        private int Q;
        private List<List<Arc?>> adjFull = new();
        internal static VehicleType? vehicleType;

        private Random random = new();

        // Always has the form of depot -> dh -> idle -> (trip -> dh -> idle) * n -> depot w/ n >= 0
        // Depot are guaranteed to be before / after all other trips + dh time. 
        private LLNode? head = null;
        private List<int> activeTrips = new();
        private List<int> inactiveTrips = new();

        private LLNode InitVehicleTask()
        {
            Location? depot = instance.Locations.Find(x => x.IsDepot);
            Arc? depotDepotArc = adjFull[^2][^1];
            if (depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
            if (depotDepotArc == null) throw new InvalidDataException("No depot to depot arc found when constructing depot vehicleelements in LS ShortestPath");

            // Find starting and ending times for instance
            int startTime = int.MaxValue, endTime = int.MinValue;
            foreach (Trip t in instance.Trips)
            {
                // Depot to trip
                int minDepTime = t.StartTime - adjFull[^2][t.Index]!.Deadhead.DeadheadTemplate.Duration;
                startTime = Math.Min(minDepTime, startTime);

                // Trip to depot
                int maxArrTime = t.EndTime + adjFull[t.Index][^1]!.Deadhead.DeadheadTemplate.Duration;
                endTime = Math.Max(maxArrTime, endTime);
            }

            // Create depot nodes, connecting dh + idle time to ensure that there is always a feasible start / end to the vehicle task
            // Depot start
            LLNode depotStart = new LLNode()
            {
                VehicleElement = new VEDepot()
                {
                    StartTime = startTime - Config.MIN_NODE_TIME,
                    EndTime = startTime,
                    Location = depot,
                    SoCDiff = 0,
                    DrivingCost = 0,
                },
                SoCAtStart = vehicleType.StartCharge,
                SoCAtEnd = vehicleType.StartCharge,
            };

            // Depot to depot deadhead
            LLNode depotDepotDH = depotStart.AddAfter(new VEDeadhead()
            {
                Deadhead = depotDepotArc.Deadhead,
                StartTime = startTime,
                EndTime = startTime + depotDepotArc.Deadhead.DeadheadTemplate.Duration,
                DrivingCost = depotDepotArc.Deadhead.BaseDrivingCost,
                SoCDiff = -depotDepotArc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage
            });

            // Idle at ending depot
            int idleTime = endTime - depotDepotDH.VehicleElement.EndTime;
            LLNode idle = depotDepotDH.AddAfter(new VEIdle()
            {
                Location = depot,
                StartTime = depotDepotDH.VehicleElement.EndTime,
                EndTime = endTime,
                DrivingCost = idleTime * Config.IDLE_COST,
                SoCDiff = 0,
            });

            // Depot end; 
            LLNode depotEnd = idle.AddAfter(new VEDepot()
            {
                StartTime = endTime,
                EndTime = endTime + Config.MIN_NODE_TIME,
                Location = depot,
                DrivingCost = 0,
                SoCDiff = 0,
            });

            head = depotStart;
            head.CalcSoCValues();
            return head;
        }

        public ShortestPathLS(Instance instance, GRBModel model, List<List<Arc?>> adjFull)
        {
            this.instance = instance;
            this.model = model;
            this.adjFull = adjFull;
            Reset();
        }

        public void Reset()
        {
            vehicleType = instance.VehicleTypes[0];
            T = Config.LS_STARTING_T;
            alpha = Config.LS_COOLING_RATE;
            Q = (int)Math.Round(-Config.LS_ITERATIONS / (Math.Log(Config.LS_STARTING_T / Config.LS_ENDING_T) / Math.Log(alpha)));
            activeTrips = new();
            inactiveTrips = instance.Trips.Select(x => x.Index).ToList();
            reducedCostsTrips = new();

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCostsTrips.Add(constrs[i].Pi);
            }

            InitVehicleTask();
        }


        /// <summary>
        /// Adds random unused trip
        /// </summary>
        private LSOpResult addTrip()
        {
            // Select random trip
            int selectIndex = random.Next(inactiveTrips.Count);
            int selectedTrip = inactiveTrips[selectIndex];
            inactiveTrips[selectIndex] = inactiveTrips[^1];
            inactiveTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip t = instance.Trips[selectedTrip];

            // Find VE (trip / depot) that precedes selected trip
            LLNode? curr = head, prev = head;
            while (curr != null)
            {
                if ((curr.VehicleElement is VEDepot || curr.VehicleElement is VETrip) && curr.VehicleElement.EndTime <= t.StartTime)
                    prev = curr;
                curr = curr.Next;
            }
            if (prev == null) throw new Exception("No previous trip/depot found for addition target");

            // Find next target
            curr = prev.Next;
            LLNode? next = null;
            while (curr != null)
            {
                if (curr.VehicleElement is VEDepot || curr.VehicleElement is VETrip)
                {
                    next = curr;
                    break;
                }
                curr = curr.Next;
            }
            if (next == null) throw new Exception("No next trip / depot found for addition target");

            // Check driving time feasibility between prev -> t -> next
            int prevAdjIndex = prev.VehicleElement is VETrip prevt ? prevt.Trip.Index : instance.DepotStartIndex;
            int nextAdjIndex = next.VehicleElement is VETrip nextt ? nextt.Trip.Index : instance.DepotEndIndex;
            Arc? prevToTrip = adjFull[prevAdjIndex][t.Index];
            Arc? tripToNext = adjFull[t.Index][nextAdjIndex];
            if (prevToTrip == null || tripToNext == null)
            {
                // Time feasibility is in here already; results in null deadhead
                return LSOpResult.Invalid;
            }


            // Check SAA feasibiltiy
            double costDiff = 0;

            // Deadhead + idle removal.
            VEDeadhead dhToReplace = prev.Next.VehicleElement as VEDeadhead;
            VEIdle idleToReplace = prev.Next.Next.VehicleElement as VEIdle;
            costDiff -= dhToReplace!.DrivingCost;
            costDiff -= idleToReplace!.DrivingCost;

            // Added deadheads 
            costDiff += prevToTrip.Deadhead.BaseDrivingCost;
            costDiff += tripToNext.Deadhead.BaseDrivingCost;

            // idle times
            int prevToTripIdle = t.StartTime - (prev.VehicleElement.EndTime + prevToTrip.Deadhead.DeadheadTemplate.Duration);
            int tripToNextIdle = next.VehicleElement.StartTime - (t.EndTime + tripToNext.Deadhead.DeadheadTemplate.Duration);
            costDiff += prevToTripIdle * Config.IDLE_COST;
            costDiff += tripToNextIdle * Config.IDLE_COST;

            // reduced cost of trip
            costDiff -= reducedCostsTrips[t.Index];

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = prev!.SoCError();
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // Place elements in current task so SoC effects can be determined
            // old situation: prev -> dh -> idle -> next
            var removedNodes = prev.RemoveAfter(2);
            LLNode dh = removedNodes[0];
            LLNode idle = removedNodes[1];

            // Situation after remove: prev -> next
            // Target situation: prev -> dh1 -> idle1 -> t -> dh2 -> idle2 -> next
            LLNode dh1 = prev.AddAfter(new VEDeadhead()
            {
                DrivingCost = prevToTrip.Deadhead.BaseDrivingCost,
                Deadhead = prevToTrip.Deadhead,
                StartTime = prev.VehicleElement.EndTime,
                EndTime = prev.VehicleElement.EndTime + prevToTrip.Deadhead.DeadheadTemplate.Duration,
                SoCDiff = -prevToTrip.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage,
            });
            LLNode idle1 = dh1.AddAfter(new VEIdle()
            {
                Location = t.From,
                DrivingCost = 0,
                StartTime = dh1.VehicleElement.EndTime,
                EndTime = t.StartTime,
                SoCDiff = 0,
            });
            LLNode trip = idle1.AddAfter(new VETrip()
            {
                DrivingCost = reducedCostsTrips[t.Index],
                StartTime = t.StartTime,
                EndTime = t.EndTime,
                Trip = t,
                SoCDiff = -t.Distance * vehicleType!.DriveUsage,
            });
            LLNode dh2 = trip.AddAfter(new VEDeadhead()
            {
                DrivingCost = tripToNext.Deadhead.BaseDrivingCost,
                Deadhead = tripToNext.Deadhead,
                StartTime = t.EndTime,
                EndTime = t.EndTime + tripToNext.Deadhead.DeadheadTemplate.Duration,
                SoCDiff = -tripToNext.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage,
            });
            LLNode idle2 = dh2.AddAfter(new VEIdle()
            {
                Location = tripToNext.Deadhead.DeadheadTemplate.To,
                DrivingCost = 0,
                StartTime = dh2.VehicleElement.EndTime,
                EndTime = next.VehicleElement.StartTime,
                SoCDiff = -tripToNext.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage,
            });

            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = prev!.SoCError();
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // TODO: reduced cost of block 

            if (!accept(costDiff))
            {
                // revert changes in list; can simply be done by connecting prev back to dh and next to idle and resetting soc. 
                prev.Next = dh;
                next.Prev = idle;
                prev.CalcSoCValues();
                return LSOpResult.Decline;
            }

            // Finalize operation
            inactiveTrips.RemoveAt(inactiveTrips.Count - 1);
            activeTrips.Add(selectedTrip);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult removeTrip()
        {
            if (activeTrips.Count == 0) return LSOpResult.Invalid;

            // Select random trip for removal
            int selectIndex = random.Next(activeTrips.Count);
            int selectedTrip = activeTrips[selectIndex];
            activeTrips[selectIndex] = activeTrips[^1];
            activeTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip t = instance.Trips[selectedTrip];

            // Find the trip in the current ll
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VehicleElement is VETrip vet && vet.Trip == t)
                {
                    break;
                }
                curr = curr.Next;
            }
            if (curr == null) throw new InvalidOperationException("Could not find selected trip for removal");

            // Check if removal is time-feasible; should always be the case. 
            // Pre-removal: prev -> dh1 -> idle1 -> t -> dh2 -> idle2 -> next
            // Post-removal: prev -> dh -> idle -> next
            LLNode prev = curr.Prev.Prev.Prev;
            LLNode next = curr.Next.Next.Next;
            int prevIndex = prev.VehicleElement is VETrip prevvet ? prevvet.Trip.Index : instance.DepotStartIndex;
            int nextIndex = next.VehicleElement is VETrip nextvet ? nextvet.Trip.Index : instance.DepotEndIndex;

            Arc? arc = adjFull[prevIndex][nextIndex];
            if (arc == null)
            {
                return LSOpResult.Invalid;
            }

            // Check LS accept
            double costDiff = 0;

            // Old drive
            costDiff -= curr.Prev.Prev.VehicleElement.DrivingCost;
            costDiff -= curr.Prev.VehicleElement.DrivingCost;
            costDiff -= curr.VehicleElement.DrivingCost;
            costDiff -= curr.Next.VehicleElement.DrivingCost;
            costDiff -= curr.Next.Next.VehicleElement.DrivingCost;

            // New drive
            costDiff += arc.Deadhead.BaseDrivingCost;
            int idleTime = next.VehicleElement.StartTime - (prev.VehicleElement.EndTime + arc.Deadhead.DeadheadTemplate.Duration);
            costDiff += idleTime * Config.IDLE_COST;

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = prev!.SoCError();
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // perform op to get charge costs
            // Situation before: prev -> dh1 -> idle1 -> trip -> dh2 -> idle2 -> next
            var removedNodes = prev.RemoveAfter(5);
            LLNode dh1 = removedNodes[0];
            LLNode idle2 = removedNodes[4];
            // Situation after: prev -> next
            LLNode dh = prev.AddAfter(new VEDeadhead()
            {
                DrivingCost = arc.Deadhead.BaseDrivingCost,
                StartTime = prev.VehicleElement.EndTime,
                EndTime = prev.VehicleElement.EndTime + arc.Deadhead.DeadheadTemplate.Duration,
                Deadhead = arc.Deadhead,
                SoCDiff = -arc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage,
            });
            LLNode idle = dh.AddAfter(new VEIdle()
            {
                Location = arc.Deadhead.DeadheadTemplate.To,
                DrivingCost = idleTime * Config.IDLE_COST,
                StartTime = dh.VehicleElement.EndTime,
                EndTime = next.VehicleElement.StartTime,
                SoCDiff = 0,
            });
            // Situation after: prev -> dh -> idle -> next


            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = prev!.SoCError();
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;


            if (!accept(costDiff))
            {
                // revert op
                prev.Next = dh1;
                next.Prev = idle2;
                prev.CalcSoCValues();

                return LSOpResult.Decline;
            }

            // Finalize operation
            activeTrips.RemoveAt(activeTrips.Count - 1);
            inactiveTrips.Add(selectedTrip);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult changeChargeAction()
        {
            // List all trips that currently have charging actions available
            LLNode? curr = head;
            List<LLNode> chargeTargets = new();
            while (curr != null)
            {
                if (curr.VehicleElement is VEDeadhead vedh && vedh.Deadhead.ChargingActions.Count > 0)
                {
                    chargeTargets.Add(curr);
                }
                curr = curr.Next;
            }

            if (chargeTargets.Count == 0) return LSOpResult.Invalid;

            LLNode target = chargeTargets[random.Next(chargeTargets.Count)];
            VEDeadhead dh = (VEDeadhead)target.VehicleElement;

            // Chose new action
            int currAction = dh.SelectedAction;
            int newAction = currAction;
            while (newAction == currAction)
                newAction = random.Next(-1, dh.Deadhead.ChargingActions.Count);
            ChargingAction? action = newAction == -1 ? null : dh.Deadhead.ChargingActions[newAction];

            // Determine new charge
            int currChargeTime = dh.ChargeTime;
            int newChargeTime = action == null ? 0 : Math.Max(action.TimeAtLocation / 2, Config.MIN_CHARGE_TIME);
            int newTotalTime = action == null
                ? dh.Deadhead.DeadheadTemplate.Duration
                : action.DrivingTimeTo + action.DrivingTimeFrom + newChargeTime;

            double newChargeGained = 0;
            double newDrivingCost = dh.Deadhead.BaseDrivingCost;
            double currSoCDiff = dh.SoCDiff;
            double newSoCDiff = -dh.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage;
            if (action != null)
            {
                double SoCAtCharger = target.SoCAtStart - action.ChargeUsedTo;
                newChargeGained = action.ChargeLocation.ChargingCurves[vehicleType.Id]
                    .MaxChargeGained(SoCAtCharger, newChargeTime, true).SoCGained;
                newDrivingCost = action.DrivingCost;
                newSoCDiff = newChargeGained - (action.ChargeUsedFrom + action.ChargeUsedTo);
            }

            double costDiff = 0;

            costDiff -= dh.DrivingCost;
            costDiff += newDrivingCost;

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = target.Prev!.SoCError();
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // perform op
            dh.SelectedAction = newAction;
            dh.ChargeTime = newChargeTime;
            dh.SoCDiff = newSoCDiff;

            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = target.Prev!.SoCError();
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // TODO: idle costs moeten eigenlijk ook meegenomen worden aangezien de dh tijd verandert

            if (!accept(costDiff))
            {
                // Revert operation
                dh.SelectedAction = currAction;
                dh.ChargeTime = currChargeTime;
                dh.SoCDiff = currSoCDiff;
                target.Prev.CalcSoCValues();

                return LSOpResult.Decline;
            }

            // Finalize operation with everything not needed for SoC calculations
            dh.DrivingCost = newDrivingCost;
            dh.ChargeGained = newChargeGained;
            dh.EndTime = dh.StartTime + newTotalTime;
            target.Next.VehicleElement.StartTime = dh.EndTime;

            return costDiff < 0 ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult changeChargeTime(LLNode target, int chargeTimeDiff)
        {
            VEDeadhead ved = target.VehicleElement as VEDeadhead;
            ChargingAction action = ved.Deadhead.ChargingActions[ved.SelectedAction];

            int currChargeTime = ved.ChargeTime;
            int newChargeTime = ved.ChargeTime + chargeTimeDiff;
            double currSoCDiff = ved.SoCDiff;
            double SoCAtCharger = target.SoCAtStart - action.ChargeUsedTo;
            double newChargeGained = action.ChargeLocation.ChargingCurves[vehicleType.Id]
                .MaxChargeGained(SoCAtCharger, newChargeTime, true).SoCGained;
            double newSoCDiff = newChargeGained - (action.ChargeUsedFrom + action.ChargeUsedTo);

            double costDiff = 0;

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = target.Prev!.SoCError();
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            // perform op
            ved.ChargeTime = newChargeTime;
            ved.SoCDiff = newSoCDiff;

            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = target.Prev!.SoCError();
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * Config.LS_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * Config.LS_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * Config.LS_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * Config.LS_UNDERCHARGE_PENALTY_VAR) / T;

            if (!accept(costDiff))
            {
                // Revert operation
                ved.ChargeTime = currChargeTime;
                ved.SoCDiff = currSoCDiff;
                target.Prev.CalcSoCValues();

                return LSOpResult.Decline;
            }

            ved.ChargeGained = newChargeGained;
            ved.EndTime += chargeTimeDiff;
            target.Next.VehicleElement.StartTime = ved.EndTime;

            return costDiff < 0 ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult increaseChargeTime()
        {
            List<LLNode?> targets = new();
            // Find node which can be increased
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VehicleElement is VEDeadhead dh
                    && dh.SelectedAction != -1 &&
                    dh.ChargeTime < dh.Deadhead.ChargingActions[dh.SelectedAction].TimeAtLocation
                )
                {
                    targets.Add(curr);
                }

                curr = curr.Next;
            }

            if (targets.Count == 0) return LSOpResult.Invalid;

            LLNode target = targets[random.Next(targets.Count)];
            VEDeadhead ved = target.VehicleElement as VEDeadhead;
            int increase = random.Next(ved.Deadhead.ChargingActions[ved.SelectedAction].TimeAtLocation - ved.ChargeTime);
            return changeChargeTime(target, increase);
        }

        private LSOpResult decreaseChargeTime()
        {
            List<LLNode?> targets = new();
            // Find node which can be increased
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VehicleElement is VEDeadhead dh
                    && dh.SelectedAction != -1 &&
                    dh.ChargeTime > Config.MIN_CHARGE_TIME
                )
                {
                    targets.Add(curr);
                }

                curr = curr.Next;
            }

            if (targets.Count == 0) return LSOpResult.Invalid;

            LLNode target = targets[random.Next(targets.Count)];
            VEDeadhead ved = target.VehicleElement as VEDeadhead;
            int decrease = random.Next(ved.ChargeTime - Config.MIN_CHARGE_TIME);
            return changeChargeTime(target, -decrease);
        }

        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        internal (double reducedCost, VehicleTask vehicleTask) getVehicleTaskLS()
        {
            List<(Func<LSOpResult> operation, double chance)> operations = [
                (addTrip, Config.LS_ADD_TRIP),
                (removeTrip, Config.LS_REM_TRIP),
                (changeChargeAction, Config.LS_CHANGE_CHARGE),
                (increaseChargeTime, Config.LS_INC_CHARGE),
                (decreaseChargeTime, Config.LS_DEC_CHARGE),
            ];
            List<(string name, List<int> counts)> results = [
                ("addTrip", [.. new int[(int)LSOpResult.Count]]),
                ("removeTrip", [.. new int[(int)LSOpResult.Count]]),
                ("changeChargeAction", [.. new int[(int)LSOpResult.Count]]),
                ("increaseChargeTime", [.. new int[(int)LSOpResult.Count]]),
                ("decreaseChargeTime", [.. new int[(int)LSOpResult.Count]]),
            ];
            List<double> sums = [operations[0].chance];

            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;
            while (currIts < Config.LS_ITERATIONS)
            {
                currIts++;
                if (currIts % Q == 0) T *= alpha;

                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
                results[operationIndex].counts[(int)res]++;
            }

            var socError = head.SoCError();
            if (socError.peaks.Count > 0 || socError.valleys.Count > 0)
            {
                Console.WriteLine("SoC error found in finished vehicle task");
            }

            if (Config.CONSOLE_LS)
            {
                foreach (var result in results)
                {
                    Console.WriteLine($"Operation: {result.name} (total: {result.counts.Sum()})");
                    for (int i = 0; i < result.counts.Count; i++)
                    {
                        Console.WriteLine((LSOpResult)i + ": " + result.counts[i]);
                    }
                }
            }

            LLNode? curr = head;
            head.CalcSoCValues();
            List<VehicleElement> elements = new();
            while (curr != null)
            {
                VehicleElement ve = curr.VehicleElement;
                ve.StartSoCInTask = curr.SoCAtStart;
                ve.EndSoCInTask = curr.SoCAtEnd;
                if (ve is VETrip) ve.DrivingCost = 0;
                elements.Add(ve);

                if (curr.Next != null && curr.SoCAtEnd != curr.Next.SoCAtStart)
                {
                    Console.WriteLine("invalid");
                }

                curr = curr.Next;
            }

            if (Config.CONSOLE_LS) Console.WriteLine($"Vehicle task found with {elements.Count} elements");

            VehicleTask vehicleTask = new VehicleTask(elements);
            double reducedCost = vehicleTask.Cost;
            foreach (int coveredTripIndex in vehicleTask.Covers)
            {
                reducedCost -= reducedCostsTrips[coveredTripIndex];
                // TODO: blocks;
            }

            return (reducedCost, vehicleTask);
        }

    }
}
