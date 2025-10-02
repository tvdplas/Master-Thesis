
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;

namespace E_VCSP.Solver.ColumnGenerators {
    public struct VSPLSCostBreakdown {
        public double DrivingCost;
        public double ChargingCost;
        public double DualCosts;
        public double PenaltyCost;

        public double OverallCost(bool withPenalty) {
            if (withPenalty)
                return DrivingCost + ChargingCost + PenaltyCost - DualCosts;
            return DrivingCost + ChargingCost - DualCosts;
        }
    }

    public class VSPLSNode {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion
        /// <summary>
        /// Vehicle element represented in node
        /// </summary>
        public required PartialVehicleElement PVE;

        /// <summary>
        /// Previous vehicle element
        /// </summary>
        public VSPLSNode? Prev;
        /// <summary>
        /// Next vehicle element
        /// </summary>
        public VSPLSNode? Next;

        public VSPLSNode() {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString() {
            return $"{DEBUG_INDEX}: {PVE}";
        }

        #region find
        public VSPLSNode? FindBefore(Func<VSPLSNode, bool> predicate, bool first) {
            VSPLSNode? curr = Prev;
            VSPLSNode? res = null;
            while (curr != null) {
                if (predicate(curr)) {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Prev;
            }
            return res;
        }

        public VSPLSNode? FindFirstBefore(Func<VSPLSNode, bool> predicate) => FindBefore(predicate, true);
        public VSPLSNode? FindLastBefore(Func<VSPLSNode, bool> predicate) => FindBefore(predicate, false);

        public VSPLSNode? FindAfter(Func<VSPLSNode, bool> predicate, bool first) {
            VSPLSNode? curr = Next;
            VSPLSNode? res = null;
            while (curr != null) {
                if (predicate(curr)) {
                    if (first) return curr;
                    else res = curr;
                }
                curr = curr.Next;
            }
            return res;
        }
        public VSPLSNode? FindFirstAfter(Func<VSPLSNode, bool> predicate) => FindAfter(predicate, true);
        public VSPLSNode? FindLastAfter(Func<VSPLSNode, bool> predicate) => FindAfter(predicate, false);

        #endregion

        #region count

        public int Count(bool head) {
            VSPLSNode? curr = this;
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
        public List<VSPLSNode> RemoveAfter(int count) {
            List<VSPLSNode> removedNodes = new(count);
            VSPLSNode? curr = Next;
            for (int i = 0; i < count && curr != null; i++) {
                removedNodes.Add(curr);
                curr = curr.Next;
            }
            this.Next = curr;
            if (curr != null) curr.Prev = this;

            return removedNodes;
        }

        public VSPLSNode AddAfter(VSPLSNode node) {
            ArgumentNullException.ThrowIfNull(node);

            if (PVE.EndTime != node.PVE.StartTime)
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

        public VSPLSNode AddAfter(PartialVehicleElement ve) {
            return AddAfter(new VSPLSNode() {
                PVE = ve,
            });
        }
        #endregion

        /// <summary>
        /// Determine charge and handover feasibility
        /// </summary>
        /// <param name="vt"></param>
        /// <returns>Cost breakdown of the tail</returns>
        public VSPLSCostBreakdown validateTail(VehicleColumnGen vcg) {
            if (this.Prev != null) throw new Exception("Not on head");

            List<int> overSteeringTime = [];
            List<int> overCrewbaseTime = [];
            List<double> underCharge = [];

            double drivingCost = 0;
            double chargingCost = 0;
            double dualCosts = 0;

            double currSoC = vcg.vss.VehicleType.StartSoC;

            int lastHandoverChance = ((PVETravel)Next!.PVE).DepartureTime;
            int lastSignOnOffChance = ((PVETravel)Next!.PVE).DepartureTime;
            DescriptorHalf currentBlockStart = new DescriptorHalf(((PVETravel)Next!.PVE).StartLocation, ((PVETravel)Next!.PVE).DepartureTime);

            void performCharge(int chargeTime, Location chargeLocation) {
                // Check bounds before charge
                if (currSoC < vcg.vss.VehicleType.MinSoC) underCharge.Add(vcg.vss.VehicleType.MinSoC - currSoC);

                ChargingCurve cc = chargeLocation.ChargingCurves[vcg.vss.VehicleType.Index];
                ChargeResult cr = cc.MaxChargeGained(currSoC, chargeTime);
                currSoC += cr.SoCGained;
                chargingCost += cr.Cost;
            }

            void performIdle(Location l, int idleStartTime, int idleEndTime) {
                // See if handover can be performed
                bool handoverAllowed = l.HandoverAllowed && idleEndTime - idleStartTime >= l.MinHandoverTime;
                if (handoverAllowed) {
                    // Handle handover
                    int overTime = (idleStartTime - lastHandoverChance) - Constants.MAX_STEERING_TIME;
                    if (overTime > 0) overSteeringTime.Add(overTime);
                    lastHandoverChance = idleEndTime;

                    // Finalize block
                    Descriptor desc = currentBlockStart.AddEnd(l, idleStartTime);
                    dualCosts -= vcg.blockDualCosts.GetValueOrDefault(desc);

                    // initialize new block
                    currentBlockStart = new DescriptorHalf(l, idleEndTime);
                }

                // See if sign on / off can be performed
                bool signOnOffAllowed = l.CrewBase && idleEndTime - idleStartTime >= l.MinHandoverTime;
                if (signOnOffAllowed) {
                    int overTime = (idleStartTime - lastSignOnOffChance) - Constants.MAX_NO_HUB_TIME;
                    if (overTime > 0) overCrewbaseTime.Add(overTime);
                    lastSignOnOffChance = idleEndTime;
                }
            }

            VSPLSNode? curr = this.Next!; // First travel after depot head

            while (curr.Next != null) // end at depot
            {
                PartialVehicleElement VE = curr.PVE;

                // Only update soc and driving costs; everything interesting is handled elsewhere
                if (VE.Type == PVEType.Travel) {
                    drivingCost += VE.DrivingCost;
                    currSoC += VE.SoCDiff;
                }
                else {
                    PVETravel prevTravel = (PVETravel)curr.Prev!.PVE;
                    PVETravel nextTravel = (PVETravel)curr.Next!.PVE;

                    // Start by checking whether we are allowed to hand over at the start;
                    // if so, update the time we are driving
                    if (VE.Type == PVEType.Trip) {
                        performIdle(VE.StartLocation, VE.StartTime - prevTravel.ArrivalTime, VE.StartTime);
                        performIdle(VE.EndLocation, VE.EndTime, nextTravel.DepartureTime);
                        dualCosts -= vcg.tripDualCosts[((PVETrip)VE).Trip.Index];
                    }
                    else {
                        // Check if time allows for handover
                        performIdle(VE.StartLocation, prevTravel.ArrivalTime, nextTravel.DepartureTime);
                    }

                    // Then, check for charging and update SoC / costs
                    int chargeTimeBefore = 0, chargeTimeAfter = 0;
                    if (VE.StartLocation!.CanCharge) {
                        chargeTimeBefore = !prevTravel.IdleAtStart ? prevTravel.IdleTime : 0;
                    }
                    if (VE.EndLocation!.CanCharge) {
                        chargeTimeAfter = nextTravel.IdleAtStart ? nextTravel.IdleTime : 0;
                    }

                    // Only charging, not actually driving anything in between. If so, ignore any costs there, and only charge
                    bool onlyCharge =
                        chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter >= Constants.MIN_CHARGE_TIME // min charging time is met
                        && (VE.Type == PVEType.HandoverDetour || VE.Type == PVEType.ChargeDetour) // Remains at same location
                        && VE.StartLocation!.CanCharge; // can actually charge

                    if (onlyCharge) {
                        performCharge(chargeTimeBefore + chargeTimeAfter, VE.StartLocation!);
                    }
                    else {
                        // Possible charge before
                        if (chargeTimeBefore >= Constants.MIN_CHARGE_TIME) performCharge(chargeTimeBefore, VE.StartLocation!);

                        // Perform element
                        drivingCost += VE.DrivingCost;
                        currSoC += VE.SoCDiff;

                        // Possible charge after
                        if (chargeTimeAfter >= Constants.MIN_CHARGE_TIME) performCharge(chargeTimeAfter, VE.EndLocation!);
                    }
                }

                curr = curr.Next;
            }

            // Add night charging costs
            chargingCost += Math.Min(0, vcg.vss.VehicleType.StartSoC - currSoC) * vcg.vss.VehicleType.Capacity / 100 * Constants.KWH_COST;

            // Finalize all times / charge at the depot
            int arrivalTime = ((PVETravel)curr.Prev!.PVE).ArrivalTime;
            int handoverOvertime = (arrivalTime - lastHandoverChance) - Constants.MAX_STEERING_TIME;
            if (handoverOvertime > 0) overSteeringTime.Add(handoverOvertime);
            int crewbaseOvertime = (arrivalTime - lastSignOnOffChance) - Constants.MAX_NO_HUB_TIME;
            if (crewbaseOvertime > 0) overCrewbaseTime.Add(crewbaseOvertime);
            double SoCDeficit = vcg.vss.VehicleType.MinSoC - currSoC;
            if (SoCDeficit > 0) underCharge.Add(SoCDeficit);

            // End final block at depot
            Descriptor desc = currentBlockStart.AddEnd(curr.PVE.StartLocation, arrivalTime);
            dualCosts -= vcg.blockDualCosts.GetValueOrDefault(desc);

            double penaltyCost = 0;
            penaltyCost += overSteeringTime.Count * Config.VSP_LS_SHR_HNDVR_FIX;
            foreach (var x in overSteeringTime) penaltyCost += x * Config.VSP_LS_SHR_HNDVR_VAR;
            penaltyCost += overCrewbaseTime.Count * Config.VSP_LS_SHR_SGNOO_FIX;
            foreach (var x in overCrewbaseTime) penaltyCost += x * Config.VSP_LS_SHR_SGNOO_VAR;
            penaltyCost += underCharge.Count * Config.VSP_LS_SHR_CHGDF_FIX;
            foreach (var x in underCharge) penaltyCost += x * Config.VSP_LS_SHR_CHGDF_VAR;

            return new VSPLSCostBreakdown {
                ChargingCost = chargingCost,
                DualCosts = dualCosts,
                PenaltyCost = penaltyCost,
                DrivingCost = drivingCost,
            };
        }

        public VehicleTask? ToVehicleTask(VehicleColumnGen vcg, string source) {
            if (Prev != null) throw new InvalidOperationException("Not calling on head");
            VehicleType vt = vcg.vss.VehicleType;

            if (validateTail(vcg).PenaltyCost > 0) return null; // Cannot create an invalid vehicle task

            List<VehicleElement> elements = [];
            if (this.Prev != null) throw new Exception("Not on head");

            double currSoC = vt.StartSoC;

            void performCharge(int startTime, int chargeTime, Location chargeLocation) {
                ChargingCurve cc = chargeLocation.ChargingCurves[vt.Index];
                ChargeResult cr = cc.MaxChargeGained(currSoC, chargeTime);
                elements.Add(new VECharge(chargeLocation, startTime, startTime + chargeTime, cr.SoCGained, cr.Cost) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + cr.SoCGained });
                currSoC += cr.SoCGained;
            }

            VSPLSNode? curr = this; // First travel after depot head
            while (curr != null) // end at depot
            {
                PartialVehicleElement VE = curr.PVE;

                if (VE.Type == PVEType.Depot) {
                    // Do nothing
                }
                else if (VE.Type == PVEType.Travel) {
                    PVETravel pvet = (PVETravel)VE;
                    VEDeadhead ved = new(pvet.DeadheadTemplate, pvet.DepartureTime, pvet.ArrivalTime, vt) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };
                    elements.Add(ved);
                    currSoC += VE.SoCDiff;
                }
                else {
                    PVETravel prevTravel = (PVETravel)curr.Prev!.PVE;
                    PVETravel nextTravel = (PVETravel)curr.Next!.PVE;

                    // Then, check for charging and update SoC / costs
                    int chargeTimeBefore = 0, chargeTimeAfter = 0;
                    if (VE.StartLocation!.CanCharge) {
                        chargeTimeBefore = !prevTravel.IdleAtStart ? prevTravel.IdleTime : 0;
                    }
                    if (VE.EndLocation!.CanCharge) {
                        chargeTimeAfter = nextTravel.IdleAtStart ? nextTravel.IdleTime : 0;
                    }

                    // Only charging, not actually driving anything in between. If so, ignore any costs there, and only charge
                    bool onlyCharge =
                        chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter >= Constants.MIN_CHARGE_TIME // min charging time is met
                        && (VE.Type == PVEType.HandoverDetour || VE.Type == PVEType.ChargeDetour) // Remains at same location
                        && VE.StartLocation!.CanCharge; // can actually charge

                    if (onlyCharge) performCharge(prevTravel.ArrivalTime, chargeTimeBefore + (VE.EndTime - VE.StartTime) + chargeTimeAfter, VE.StartLocation!);
                    else {
                        bool chargeBefore = chargeTimeBefore >= Constants.MIN_CHARGE_TIME;
                        bool chargeAfter = chargeTimeAfter >= Constants.MIN_CHARGE_TIME;

                        // Possible charge before
                        if (chargeBefore) performCharge(prevTravel.ArrivalTime, chargeTimeBefore, VE.StartLocation!);
                        else if (VE.Type == PVEType.Trip) {
                            elements.Add(new VEIdle(VE.StartLocation, prevTravel.ArrivalTime, VE.StartTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC });
                        }

                        // Perform element
                        VehicleElement stop = VE.Type == PVEType.Trip
                            ? new VETrip(((PVETrip)VE).Trip, vt) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff }
                            : new VEIdle(
                                VE.StartLocation,
                                chargeBefore ? VE.StartTime : prevTravel.ArrivalTime,
                                chargeAfter ? VE.EndTime : nextTravel.DepartureTime
                            ) { StartSoCInTask = currSoC, EndSoCInTask = currSoC + VE.SoCDiff };
                        elements.Add(stop);
                        currSoC += VE.SoCDiff;

                        // Possible charge after
                        if (chargeAfter) performCharge(VE.EndTime, chargeTimeAfter, VE.EndLocation!);
                        else if (VE.Type == PVEType.Trip) {
                            elements.Add(new VEIdle(VE.EndLocation, VE.EndTime, nextTravel.DepartureTime) { StartSoCInTask = currSoC, EndSoCInTask = currSoC });
                        }
                    }
                }

                curr = curr.Next;
            }

            return new VehicleTask(elements) {
                vehicleType = vt,
                Source = source,
            };
        }
    }

    public class LSOperations {
        VehicleColumnGen vcg;
        VehicleSolutionState vss;
        private Random random = new();
        public string type;
        public double T;

        public LSOperations(VehicleColumnGen vcg, double T, string type) {
            this.vcg = vcg;
            this.vss = this.vcg.vss;
            this.T = T;
            this.type = type;
        }

        /// <summary>
        /// Adds a stop to the task
        /// </summary>
        /// <param name="ve"></param>
        /// <returns></returns>
        public LSOpResult addStop(VSPLSNode? head, PartialVehicleElement ve) {
            if (ve.Type == PVEType.Depot || ve.Type == PVEType.Travel)
                throw new InvalidOperationException("Are you sure?");

            // Find addition targetss
            VSPLSNode? prev = head!.FindLastAfter((node) => {
                PVEType type = node.PVE.Type;
                return type != PVEType.Travel && node.PVE.EndTime <= ve.StartTime;
            }) ?? head;
            VSPLSNode? next = prev!.FindFirstAfter((node) => {
                PVEType type = node.PVE.Type;
                return type != PVEType.Travel;
            }) ?? head;
            if (prev == null || next == null) throw new InvalidDataException("Could not find prev/next");
            if (next != prev!.Next!.Next) throw new InvalidDataException("Vlgm gaat hier nog iets mis");

            // See if travels can be made to connect prev -travel1> ve -travel2> next
            DeadheadTemplate? travel1Template = null;
            DeadheadTemplate? travel2Template = null;

            if (!Config.VSP_LS_SHR_EXPAND_AVT && prev.PVE.Type == PVEType.Trip && ve.Type == PVEType.Trip) {
                // Check possible deadheads instead of direct location transfer
                travel1Template = vss.AdjFull[((PVETrip)prev.PVE).Trip.Index][((PVETrip)ve).Trip.Index]?.DeadheadTemplate;
            }
            else {
                travel1Template = vss.LocationDHT[prev.PVE.EndLocation!.Index][ve.StartLocation!.Index];
            }

            if (!Config.VSP_LS_SHR_EXPAND_AVT && ve.Type == PVEType.Trip && next.PVE.Type == PVEType.Trip) {
                // Check possible deadheads instead of direct location transfer
                travel2Template = vss.AdjFull[((PVETrip)ve).Trip.Index][((PVETrip)next.PVE).Trip.Index]?.DeadheadTemplate;
            }
            else {
                travel2Template = vss.LocationDHT[ve.EndLocation!.Index][next.PVE.StartLocation!.Index];
            }

            // No travel possible
            if (travel1Template == null || travel2Template == null) return LSOpResult.Invalid;

            // No time for new travels
            if (travel1Template.Duration > ve.StartTime - prev.PVE.EndTime || travel2Template.Duration > next.PVE.StartTime - ve.EndTime) return LSOpResult.Invalid;

            // New travel is time feasible 
            // Will now check charge / handover time feasibility
            double costDiff = 0;

            // Costs lost due to previous travel
            VSPLSNode oldTravel = prev.Next!;

            // Old charge / driving costs
            var oldRes = head!.validateTail(vcg);
            if (!Config.VSP_LS_SHR_ALLOW_PENALTY && oldRes.PenaltyCost != 0) {
                throw new InvalidOperationException("Previous state not valid");
            }
            costDiff -= oldRes.OverallCost(Config.VSP_LS_SHR_ALLOW_PENALTY);

            // Add new part into mix
            VSPLSNode travel1 = new VSPLSNode() {
                PVE = new PVETravel(travel1Template, prev.PVE.EndTime, ve.StartTime, vss.VehicleType)
            };
            travel1.Prev = prev;
            VSPLSNode stop = travel1.AddAfter(ve);
            VSPLSNode travel2 = stop.AddAfter(new PVETravel(travel2Template, ve.EndTime, next.PVE.StartTime, vss.VehicleType));
            travel2.Next = next;

            prev.Next = travel1;
            next.Prev = travel2;
            var newRes = head!.validateTail(vcg);
            bool changeFeasible = Config.VSP_LS_SHR_ALLOW_PENALTY || newRes.PenaltyCost == 0;
            costDiff += newRes.OverallCost(Config.VSP_LS_SHR_ALLOW_PENALTY);

            bool decline = !LSShared.Accept(costDiff, T);

            if (!changeFeasible || decline) {
                // revert
                prev.Next = oldTravel;
                next.Prev = oldTravel;

                return decline ? LSOpResult.Decline : LSOpResult.Invalid;
            }

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        public LSOpResult removeStop(VSPLSNode? head, VSPLSNode node) {
            if (node.PVE.Type == PVEType.Depot || node.PVE.Type == PVEType.Travel)
                throw new InvalidOperationException("Are you sure?");

            // Find addition targets
            VSPLSNode prev = node.Prev!.Prev!;
            VSPLSNode next = node.Next!.Next!;

            // See if travels can be made to connect prev -travel1> ve -travel2> next
            DeadheadTemplate? travelTemplate = null;
            if (!Config.VSP_LS_SHR_EXPAND_AVT && prev.PVE.Type == PVEType.Trip && next.PVE.Type == PVEType.Trip) {
                // Check possible deadheads instead of direct location transfer
                travelTemplate = vss.AdjFull[((PVETrip)prev.PVE).Trip.Index][((PVETrip)next.PVE).Trip.Index]?.DeadheadTemplate;
            }
            else {
                travelTemplate = vss.LocationDHT[prev.PVE.EndLocation!.Index][next.PVE.StartLocation!.Index];
            }

            // No travel possible
            if (travelTemplate == null) return LSOpResult.Invalid;

            // No time for new travel
            if (travelTemplate.Duration > next.PVE.StartTime - prev.PVE.EndTime) return LSOpResult.Invalid;

            // New travel is time feasible 
            double costDiff = 0;

            // Previous travels
            VSPLSNode oldTravel1 = prev.Next!;
            VSPLSNode oldTravel2 = next.Prev!;

            // Old charge / driving costs
            var oldRes = head!.validateTail(vcg);
            if (!Config.VSP_LS_SHR_ALLOW_PENALTY && oldRes.PenaltyCost != 0) {
                throw new InvalidOperationException("old solution not feasible");
            }
            costDiff -= oldRes.OverallCost(Config.VSP_LS_SHR_ALLOW_PENALTY);

            // Add new part into mix
            VSPLSNode travel = new VSPLSNode() {
                PVE = new PVETravel(travelTemplate, prev.PVE.EndTime, next.PVE.StartTime, vss.VehicleType)
            };
            travel.Prev = prev;
            travel.Next = next;

            prev.Next = travel;
            next.Prev = travel;
            var newRes = head!.validateTail(vcg);
            bool changeFeasible = Config.VSP_LS_SHR_ALLOW_PENALTY || newRes.PenaltyCost == 0;
            costDiff += newRes.OverallCost(Config.VSP_LS_SHR_ALLOW_PENALTY);

            bool decline = !LSShared.Accept(costDiff, T);

            if (!changeFeasible || decline) {
                // revert
                prev.Next = oldTravel1;
                next.Prev = oldTravel2;

                return decline ? LSOpResult.Decline : LSOpResult.Invalid;
            }

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        /// <summary>
        /// adds random charging stop
        /// </summary>
        public LSOpResult addChargeStop(VSPLSNode? head) {
            // Select random trip
            int selectIndex = random.Next(vss.Instance.ChargingLocations.Count);
            Location selectedLocation = vss.Instance.ChargingLocations[selectIndex];

            List<(int start, int end)> times = new();
            VSPLSNode? curr = head;
            while (curr != null) {
                if (curr.PVE.Type == PVEType.Travel) {
                    if (!curr.Prev!.PVE.EndLocation.CanCharge && !curr.Next!.PVE.StartLocation.CanCharge) {
                        times.Add((curr.PVE.StartTime, curr.PVE.EndTime));
                    }
                }
                curr = curr.Next;
            }

            if (times.Count == 0) return LSOpResult.Invalid;

            var timeSlot = times[random.Next(times.Count)];
            var padding = Math.Max(0, (timeSlot.end - timeSlot.start - Constants.MIN_NODE_TIME) / 2);

            return addStop(head, new PVECharge(
                selectedLocation,
                Math.Min(timeSlot.start + padding, timeSlot.end),
                Math.Min(timeSlot.start + padding + Constants.MIN_NODE_TIME, timeSlot.end))
            );
        }

        /// <summary>
        /// Removes random charging stop
        /// </summary>
        /// <returns></returns>
        public LSOpResult removeChargeStop(VSPLSNode? head) {
            List<VSPLSNode> targets = new();

            VSPLSNode? curr = head;
            while (curr != null) {
                if (curr.PVE.Type == PVEType.ChargeDetour) targets.Add(curr);
                curr = curr.Next;
            }

            if (targets.Count == 0) return LSOpResult.Invalid;

            return removeStop(head, targets[random.Next(targets.Count)]);
        }
    }
}