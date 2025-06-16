using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;

namespace E_VCSP.Solver.ColumnGenerators
{
    public enum LSOpResult
    {
        Improvement = 0,
        Accept = 1,
        Decline = 2,
        Invalid = 3,
        Count,
    }

    public class LLNode
    {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion
        /// <summary>
        /// Vehicle element represented in node
        /// </summary>
        public required PartialVehicleElement VE;

        /// <summary>
        /// Previous vehicle element
        /// </summary>
        public LLNode? Prev;
        /// <summary>
        /// Next vehicle element
        /// </summary>
        public LLNode? Next;

        public LLNode()
        {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString()
        {
            return $"{DEBUG_INDEX}: {VE}";
        }

        #region find
        public LLNode? FindBefore(Func<LLNode, bool> predicate, bool first)
        {
            LLNode? curr = Prev;
            LLNode? res = null;
            while (curr != null)
            {
                if (predicate(curr))
                {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Prev;
            }
            return res;
        }

        public LLNode? FindFirstBefore(Func<LLNode, bool> predicate) => FindBefore(predicate, true);
        public LLNode? FindLastBefore(Func<LLNode, bool> predicate) => FindBefore(predicate, false);

        public LLNode? FindAfter(Func<LLNode, bool> predicate, bool first)
        {
            LLNode? curr = Next;
            LLNode? res = null;
            while (curr != null)
            {
                if (predicate(curr))
                {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Next;
            }
            return res;
        }
        public LLNode? FindFirstAfter(Func<LLNode, bool> predicate) => FindAfter(predicate, true);
        public LLNode? FindLastAfter(Func<LLNode, bool> predicate) => FindAfter(predicate, false);

        #endregion

        #region count

        public int Count(bool head)
        {
            LLNode? curr = this;
            int c = 0;
            while (curr != null)
            {
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
        public List<LLNode> RemoveAfter(int count)
        {
            List<LLNode> removedNodes = new(count);
            LLNode? curr = Next;
            for (int i = 0; i < count && curr != null; i++)
            {
                removedNodes.Add(curr);
                curr = curr.Next;
            }
            this.Next = curr;
            if (curr != null) curr.Prev = this;

            return removedNodes;
        }

        public LLNode AddAfter(LLNode node)
        {
            ArgumentNullException.ThrowIfNull(node);

            if (VE.EndTime != node.VE.StartTime)
                throw new InvalidOperationException("Vehicle task is not continuous");

            // Set next correctly
            if (Next != null)
            {
                Next.Prev = node;
            }

            // Update ordering
            node.Next = Next;
            node.Prev = this;
            this.Next = node;

            return node;
        }

        public LLNode AddAfter(PartialVehicleElement ve)
        {
            return AddAfter(new LLNode()
            {
                VE = ve,
            });
        }
        #endregion


        /// <summary>
        /// Determine charge and handover feasibility
        /// </summary>
        /// <param name="vt"></param>
        /// <returns></returns>
        public (
            bool handoversFeasible,
            bool chargeFeasible,
            double drivingCost,
            double chargingCost
        ) validateTail(VehicleType vt)
        {
            if (this.Prev != null) throw new Exception("Not on head");

            bool handoversFeasible = true;
            bool chargeFeasible = true;
            double drivingCost = 0;
            double chargingCost = 0;

            double currSoC = vt.StartSoC;
            int drivingSince = ((PVETravel)Next!.VE).DepartureTime;

            bool SoCInBounds() => currSoC <= vt.MaxSoC && currSoC >= vt.MinSoC;
            void performCharge(int chargeTime, Location chargeLocation)
            {
                // In bounds before charge
                chargeFeasible &= SoCInBounds();

                ChargingCurve cc = chargeLocation.ChargingCurves[vt.Index];
                ChargeResult cr = cc.MaxChargeGained(currSoC, chargeTime);
                currSoC += cr.SoCGained;
                chargingCost += cr.Cost;

                // In bounds after charge
                chargeFeasible &= SoCInBounds();
            }

            bool handoverValid(int currTime) => currTime - drivingSince <= Config.MAX_DRIVE_TIME;

            LLNode? curr = this.Next!; // First travel after depot head

            while (curr.Next != null) // end at depot
            {
                PartialVehicleElement VE = curr.VE;

                // Only update soc and driving costs; everything interesting is handled elsewhere
                if (VE.Type == PVEType.Travel)
                {
                    drivingCost += VE.DrivingCost;
                    currSoC += VE.SoCDiff;
                }
                else
                {
                    PVETravel prevTravel = (PVETravel)curr.Prev!.VE;
                    PVETravel nextTravel = (PVETravel)curr.Next!.VE;

                    // Start by checking whether we are allowed to hand over at the start;
                    // if so, update the time we are driving
                    if (VE.Type == PVEType.Trip)
                    {
                        // Handover must happen before / after / both

                        // Handover before
                        bool handoverPossibleBefore = VE.StartLocation!.HandoverAllowed && VE.StartTime - prevTravel.ArrivalTime >= VE.StartLocation!.MinHandoverTime;
                        if (handoverPossibleBefore)
                        {
                            handoversFeasible &= handoverValid(prevTravel.ArrivalTime);
                            drivingSince = VE.StartTime;
                        }

                        // Handover after
                        bool handoverPossibleAfter = VE.EndLocation!.HandoverAllowed && nextTravel.DepartureTime - VE.EndTime >= VE.EndLocation!.MinHandoverTime;
                        if (handoverPossibleAfter)
                        {
                            handoversFeasible &= handoverValid(VE.EndTime);
                            drivingSince = nextTravel.DepartureTime;
                        }
                    }
                    else
                    {
                        // Check if time allows for handover
                        bool handoverPossible = VE.StartLocation!.HandoverAllowed && nextTravel.DepartureTime - prevTravel.ArrivalTime >= VE.StartLocation!.MinHandoverTime;
                        if (handoverPossible)
                        {
                            handoversFeasible &= handoverValid(prevTravel.ArrivalTime);
                            drivingSince = nextTravel.DepartureTime;
                        }
                    }

                    // Then, check for charging and update SoC / costs
                    int chargeTimeBefore = 0, chargeTimeAfter = 0;
                    if (VE.StartLocation!.CanCharge)
                    {
                        chargeTimeBefore = !prevTravel.IdleAtStart ? prevTravel.IdleTime : 0;
                    }
                    if (VE.EndLocation!.CanCharge)
                    {
                        chargeTimeAfter = nextTravel.IdleAtStart ? nextTravel.IdleTime : 0;
                    }

                    // Only charging, not actually driving anything in between. If so, ignore any costs there, and only charge
                    bool onlyCharge =
                        chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter >= Config.MIN_CHARGE_TIME // min charging time is met
                        && (VE.Type == PVEType.HandoverDetour || VE.Type == PVEType.ChargeDetour) // Remains at same location
                        && VE.StartLocation!.CanCharge; // can actually charge

                    if (onlyCharge) performCharge(chargeTimeBefore + chargeTimeAfter, VE.StartLocation!);
                    else
                    {
                        // Possible charge before
                        if (chargeTimeBefore >= Config.MIN_CHARGE_TIME) performCharge(chargeTimeBefore, VE.StartLocation!);

                        // Perform element
                        drivingCost += VE.DrivingCost;
                        currSoC += VE.SoCDiff;

                        // Possible charge after
                        if (chargeTimeAfter >= Config.MIN_CHARGE_TIME) performCharge(chargeTimeAfter, VE.EndLocation!);
                    }
                }

                curr = curr.Next;
            }

            // Check if arrival at depot was valid
            handoversFeasible &= handoverValid(((PVETravel)curr.Prev!.VE).ArrivalTime);

            if (!handoversFeasible)
            {
                int x = 0;
            }

            // Check if SoC was valid
            chargeFeasible &= SoCInBounds();

            // Add night charging costsP
            chargingCost += Math.Min(0, vt.StartSoC - currSoC) * vt.Capacity / 100 * Config.KWH_COST;

            return (handoversFeasible, chargeFeasible, drivingCost, chargingCost);
        }

        public VehicleTask ToVehicleTask(VehicleType vt)
        {
            if (Prev != null) throw new InvalidOperationException("Not calling on head");


            List<VehicleElement> elements = [];
            if (this.Prev != null) throw new Exception("Not on head");

            double currSoC = vt.StartSoC;

            void performCharge(int startTime, int chargeTime, Location chargeLocation)
            {
                ChargingCurve cc = chargeLocation.ChargingCurves[vt.Index];
                ChargeResult cr = cc.MaxChargeGained(currSoC, chargeTime);
                elements.Add(new VECharge(chargeLocation, startTime, startTime + chargeTime, cr.SoCGained, cr.Cost) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + cr.SoCGained });
                currSoC += cr.SoCGained;
            }


            LLNode? curr = this; // First travel after depot head
            while (curr != null) // end at depot
            {
                PartialVehicleElement VE = curr.VE;

                if (VE.Type == PVEType.Depot)
                {
                    VEIdle vei;
                    if (curr.Prev == null)
                    {
                        vei = new(VE.StartLocation!, VE.StartTime, ((PVETravel)curr.Next!.VE).DepartureTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };

                    }
                    else
                    {
                        vei = new(VE.StartLocation!, ((PVETravel)curr.Prev!.VE).ArrivalTime, VE.EndTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };
                    }
                    elements.Add(vei);
                    currSoC += VE.SoCDiff;
                }
                else if (VE.Type == PVEType.Travel)
                {
                    PVETravel pvet = (PVETravel)VE;
                    VEDeadhead ved = new(pvet.DeadheadTemplate, pvet.DepartureTime, pvet.ArrivalTime, vt) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };
                    elements.Add(ved);
                    currSoC += VE.SoCDiff;
                }
                else
                {
                    PVETravel prevTravel = (PVETravel)curr.Prev!.VE;
                    PVETravel nextTravel = (PVETravel)curr.Next!.VE;

                    // Then, check for charging and update SoC / costs
                    int chargeTimeBefore = 0, chargeTimeAfter = 0;
                    if (VE.StartLocation!.CanCharge)
                    {
                        chargeTimeBefore = !prevTravel.IdleAtStart ? prevTravel.IdleTime : 0;
                    }
                    if (VE.EndLocation!.CanCharge)
                    {
                        chargeTimeAfter = nextTravel.IdleAtStart ? nextTravel.IdleTime : 0;
                    }

                    // Only charging, not actually driving anything in between. If so, ignore any costs there, and only charge
                    bool onlyCharge =
                        chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter >= Config.MIN_CHARGE_TIME // min charging time is met
                        && (VE.Type == PVEType.HandoverDetour || VE.Type == PVEType.ChargeDetour) // Remains at same location
                        && VE.StartLocation!.CanCharge; // can actually charge

                    if (onlyCharge) performCharge(prevTravel.ArrivalTime, chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter, VE.StartLocation!);
                    else
                    {
                        bool chargeBefore = chargeTimeBefore >= Config.MIN_CHARGE_TIME;
                        bool chargeAfter = chargeTimeAfter >= Config.MIN_CHARGE_TIME;

                        // Possible charge before
                        if (chargeBefore) performCharge(prevTravel.ArrivalTime, chargeTimeBefore, VE.StartLocation!);
                        else if (VE.Type == PVEType.Trip)
                        {
                            elements.Add(new VEIdle(VE.StartLocation, prevTravel.ArrivalTime, VE.StartTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC });
                        }

                        // Perform element
                        VehicleElement stop = VE.Type == PVEType.Trip
                            ? new VETrip(((PVETrip)VE).Trip, vt) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff }
                            : new VEIdle(
                                VE.StartLocation,
                                chargeBefore ? VE.StartTime : prevTravel.ArrivalTime,
                                chargeAfter ? VE.EndTime : nextTravel.DepartureTime
                            )
                            { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };
                        elements.Add(stop);
                        currSoC += VE.SoCDiff;

                        // Possible charge after
                        if (chargeAfter) performCharge(nextTravel.DepartureTime, chargeTimeAfter, VE.EndLocation!);
                        else if (VE.Type == PVEType.Trip)
                        {
                            elements.Add(new VEIdle(VE.EndLocation, VE.EndTime, nextTravel.DepartureTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC });
                        }
                    }
                }

                curr = curr.Next;
            }

            return new VehicleTask(elements) { vehicleType = vt };
        }
    }
}