using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;
using System.Collections;

namespace E_VCSP.Solver.ColumnGenerators {
    public enum ChargeTime {
        None,
        Source,
        Target,
        Detour,
    }

    public record VSPLabel {
        public required int Id;
        public int PrevNodeIndex; // Previously visited node index
        public int PrevId; // Label preceding this label 
        public int ChargeLocationIndex = -1; // Charge location used in this node
        public int SteeringTime = 0; // Time since a handover was visited
        public int LastHubTime = 0; // Time since a stop with hub capibility was visited
        public required ChargeTime ChargeTime; // Was charge at start / end of deadhead
        public BitArray CoveredTrips;

        public double CurrActualSoC; // SoC at the end of trip
        public int CurrSoCBin; // SoC bin used for domination

        public double CurrCosts;  // Actual costs incurred at the end of trip, including blocks already completed
        public DescriptorHalf CurrentBlockStart; // Origin block start
        public double MaxBlockSavings; // Min costs that can be made based on block reduced cost
        public double MinBlockSavings; // Max costs that can be made based on block reduced cost

        public static int SoCBin(double SoC) {
            return (int)Math.Round(SoC / 100 * Config.VSP_LB_SOC_BINS);
        }
    }

    public class VSPFront {
        private SortedList<int, VSPLabel> front = [];

        private List<int> sameCharge = new();


        /// <summary>
        /// Inserts label into front
        /// </summary>
        /// <param name="label">Label to be inserted</param>
        /// <returns>The difference in amount of items currently in the front; min = -#items in front + 1, max = 1</returns>
        public int Insert(VSPLabel label) {
            int l = 0, r = front.Count;

            // find first item that is larger than or equal to label CurrSoC
            while (l < r) {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.CurrSoCBin) r = m;
                else l = m + 1;
            }

            sameCharge.Clear();
            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++) {
                // Skip insertion if new label is dominated by an existing one
                VSPLabel existing = front.Values[i];
                if (existing.CurrCosts - existing.MinBlockSavings <= label.CurrCosts - label.MaxBlockSavings)
                    return 0;

                // If domination is not found however we are within epsilon range of the inserted label, 
                // add it to a seperate list of items we might remove if the new label dominates it
                if (front.Values[i].CurrSoCBin == label.CurrSoCBin) sameCharge.Add(i);
            }

            // Keep track of front size diff
            int diff = 0;

            // Remove labels within epsilon range if they are dominated by the new label
            for (int i = sameCharge.Count - 1; i >= 0; i--) {
                VSPLabel existing = front.Values[sameCharge[i]];
                if (existing.CurrCosts - existing.MaxBlockSavings >= label.CurrActualSoC - label.MinBlockSavings) {
                    front.RemoveAt(sameCharge[i]);
                    diff--;
                }
            }

            // Remove all items which are strictly dominated
            for (int i = l - 1; i >= 0 && i < front.Count; i--) {
                var existing = front.Values[i];
                if (existing.CurrCosts - existing.MaxBlockSavings >= label.CurrActualSoC - label.MinBlockSavings) {
                    front.RemoveAt(i);
                    diff--;
                }
            }

            // Lastly, insert new item. First path should never be taken
            if (front.ContainsKey(label.CurrSoCBin)) front[label.CurrSoCBin] = label;
            else {
                front[label.CurrSoCBin] = label;
                diff++;
            }
            return diff;
        }
        public int Count => front.Count;

        public void Clear() => front.Clear();

        public VSPLabel Pop() {
            int lastIndex = front.Count - 1;
            var label = front.Values[lastIndex];
            front.RemoveAt(lastIndex);
            return label;
        }
    }

    public class VSPLabeling : VehicleColumnGen {
        private int labelIdCounter = 0;
        private List<List<VSPLabel>> allLabels = [];
        private List<VSPFront> activeLabels = [];
        private readonly List<double> noDualCost = [0];

        private List<bool> blockedNodes = [];

        public VSPLabeling(VehicleSolutionState vss) : base(vss) {
            blockedNodes = [.. vss.Nodes.Select(x => false)];
        }
        private int addLabel(VSPLabel spl, int index) {
            allLabels[index].Add(spl);
            return activeLabels[index].Insert(spl);
        }

        public void ResetLabels() {
            labelIdCounter = 0;
            allLabels.Clear();
            activeLabels.Clear();

            for (int i = 0; i < vss.Nodes.Count; i++) {
                allLabels.Add([]);
                activeLabels.Add(new());
            }
        }

        private void runLabeling() {
            bool handoverAllowed(Location l, int idleTime) => l.HandoverAllowed && idleTime >= l.MinHandoverTime;
            bool inBounds(double SoC, int steeringTime, int hubTime) {
                return SoC >= vss.VehicleType.MinSoC
                    && steeringTime <= Constants.MAX_STEERING_TIME
                    && hubTime <= Constants.MAX_NO_HUB_TIME;
            }

            // Initial label at start of depot; note: no cost range yet, as 
            // we cannot be inside of a block yet
            int activeLabelCount = addLabel(new VSPLabel() {
                Id = labelIdCounter++,
                PrevId = -1,
                PrevNodeIndex = vss.Nodes.Count - 2,
                CurrCosts = Config.VH_PULLOUT_COST,
                MaxBlockSavings = 0,
                MinBlockSavings = 0,
                CurrActualSoC = vss.VehicleType.StartSoC,
                CurrSoCBin = VSPLabel.SoCBin(vss.VehicleType.StartSoC),
                ChargeTime = ChargeTime.None,
                CoveredTrips = new BitArray(vss.Instance.Trips.Count)
            }, vss.Nodes.Count - 2);

            Location depot = vss.Instance.Locations.Find(x => x.IsDepot)!;

            while (activeLabelCount > 0) {
                // Get label to expand
                VSPLabel? expandingLabel = null;
                int expandingLabelIndex = -1;
                for (int i = 0; i < activeLabels.Count; i++) {
                    if (activeLabels[i].Count > 0) {
                        expandingLabel = activeLabels[i].Pop();
                        expandingLabelIndex = i;
                        activeLabelCount--;
                        break;
                    }
                }
                if (expandingLabel == null) throw new InvalidDataException("Could not find active label to expand");

                for (int i = 0; i < vss.Adj[expandingLabelIndex].Count; i++) {
                    VSPArc arc = vss.Adj[expandingLabelIndex][i];

                    // Skip this node as it is already part of the primary solution
                    if (blockedNodes[arc.To.Index]) continue;

                    int targetIndex = arc.To.Index;
                    Trip? targetTrip = targetIndex < vss.Instance.Trips.Count ? vss.Instance.Trips[targetIndex] : null;

                    BitArray newCover;
                    if (targetTrip != null) {
                        newCover = new BitArray(expandingLabel.CoveredTrips);
                        newCover[targetTrip.Index] = true;
                    }
                    else {
                        newCover = expandingLabel.CoveredTrips;
                    }

                    void processNewLabel(
                        double SoC,
                        double cost,
                        int steeringTime,
                        int hubTime,
                        int chargeIndex,
                        ChargeTime chargeTime,
                        double minBlockSavings,
                        double maxBlockSavings,
                        DescriptorHalf blockDescriptorStart
                    ) {
                        SoC -= (targetTrip?.Distance ?? 0) * vss.VehicleType.DriveUsage;
                        steeringTime += targetTrip?.Duration ?? 0;
                        hubTime += targetTrip?.Duration ?? 0;
                        cost += (targetTrip?.Distance ?? 0) * Constants.VH_M_COST;
                        if (!inBounds(SoC, steeringTime, hubTime)) return;

                        activeLabelCount += addLabel(new VSPLabel() {
                            Id = labelIdCounter++,
                            PrevId = expandingLabel.Id,
                            PrevNodeIndex = expandingLabelIndex,
                            ChargeTime = chargeTime,
                            ChargeLocationIndex = chargeIndex,
                            CurrCosts = expandingLabel.CurrCosts + cost,
                            CurrActualSoC = SoC,
                            CurrSoCBin = VSPLabel.SoCBin(SoC),
                            SteeringTime = steeringTime,
                            LastHubTime = hubTime,
                            CoveredTrips = newCover,
                            CurrentBlockStart = blockDescriptorStart,
                            MaxBlockSavings = maxBlockSavings,
                            MinBlockSavings = minBlockSavings,
                        }, arc.To.Index);
                    }

                    // List of vehicle statstics achievable at start of trip.

                    bool depotStart = expandingLabelIndex >= vss.Instance.Trips.Count;
                    bool depotEnd = targetIndex >= vss.Instance.Trips.Count;
                    bool depotArc = depotStart || depotEnd;

                    // Direct trip
                    {
                        Location arcStartLocation = expandingLabelIndex < vss.Instance.Trips.Count
                            ? vss.Instance.Trips[expandingLabelIndex].EndLocation
                            : depot;
                        Location arcEndLocation = targetIndex < vss.Instance.Trips.Count
                            ? vss.Instance.Trips[targetIndex].StartLocation
                            : depot;

                        int idleTime = depotArc ? 0
                            : vss.Instance.Trips[targetIndex].StartTime - vss.Instance.Trips[expandingLabelIndex].EndTime - arc.DeadheadTemplate.Duration;
                        bool idleAtStart = true;
                        Location? idleLocation = arcStartLocation;
                        if (depotArc) {
                            // No idle anyways
                            idleLocation = depot;
                        }
                        else if (arcEndLocation.CanCharge && !arcStartLocation.CanCharge) {
                            idleLocation = arcEndLocation;
                            idleAtStart = false;
                        }
                        else if (arcEndLocation.HandoverAllowed && !arcStartLocation.HandoverAllowed) {
                            idleLocation = arcEndLocation;
                            idleAtStart = false;
                        }

                        bool canCharge = idleLocation.CanCharge && idleTime > Constants.MIN_CHARGE_TIME;

                        int steeringTime = expandingLabel.SteeringTime;
                        int hubTime = expandingLabel.LastHubTime;
                        double SoC = expandingLabel.CurrActualSoC;
                        double minBlockSavings = expandingLabel.MinBlockSavings;
                        double maxBlockSavings = expandingLabel.MaxBlockSavings;
                        DescriptorHalf blockDescriptorStart = expandingLabel.CurrentBlockStart;

                        // Costs of doing arc
                        double costs = arc.DeadheadTemplate.Distance * Constants.VH_M_COST;
                        if (targetIndex < vss.Instance.Trips.Count) costs -= tripDualCosts[targetIndex];

                        // ###### IDLE BEFORE DEADHEAD ######

                        // Short idle at start of arc
                        if (idleAtStart) {
                            // No handover possible
                            if (!handoverAllowed(idleLocation, idleTime)) {
                                steeringTime += idleTime;
                                hubTime += idleTime;
                            }
                            // Handover possible
                            else {
                                // Reset steering time
                                steeringTime = 0;
                                if (idleLocation.CrewBase) hubTime = 0;

                                // Finalize block dual costs by adding to actual costs
                                int locationIndex = idleLocation.Index;
                                int idleStartTime = arc.StartTime;
                                int idleEndTime = arc.StartTime + idleTime;
                                Descriptor descriptor = blockDescriptorStart.AddEnd(idleLocation, idleStartTime);
                                costs -= blockDualCosts.GetValueOrDefault(descriptor);

                                // Initialize new range
                                blockDescriptorStart = new DescriptorHalf(idleLocation, idleEndTime);
                                List<double> blockDuals = blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart, noDualCost);
                                minBlockSavings = blockDuals.Min();
                                maxBlockSavings = blockDuals.Max();
                            }

                            // Charging during idle
                            if (canCharge) {
                                var res = idleLocation!.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                                costs += res.Cost;
                                SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);
                            }
                            // Losing charge during idle
                            else if (!idleLocation.FreeIdle) {
                                double idleChargeUsed = idleTime * vss.VehicleType.IdleUsage;
                                SoC -= idleChargeUsed;
                            }
                        }

                        // Check for valid state
                        if (!inBounds(SoC, steeringTime, hubTime)) continue;

                        // ###### END IDLE BEFORE DEADHEAD ######

                        // ###### DEADHEAD #######
                        // Block descriptor init when coming from depot
                        if (depotStart && !depotEnd) {
                            blockDescriptorStart = new DescriptorHalf(depot, arc.StartTime);
                            List<double> blockDuals = blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart, noDualCost);
                            minBlockSavings = blockDuals.Min();
                            maxBlockSavings = blockDuals.Max();
                        }

                        // Perform actual deadhead
                        SoC -= arc.DeadheadTemplate.Distance * vss.VehicleType.DriveUsage;
                        steeringTime += arc.DeadheadTemplate.Duration;
                        hubTime += arc.DeadheadTemplate.Duration;

                        // Block descriptor finalization when entering depot
                        if (!depotStart && depotEnd) {
                            int vehicleEndTime = arc.StartTime + arc.DeadheadTemplate.Duration;
                            Descriptor descriptor = blockDescriptorStart.AddEnd(depot, vehicleEndTime);
                            costs -= blockDualCosts.GetValueOrDefault(descriptor);
                        }

                        // Check for valid state
                        if (!inBounds(SoC, steeringTime, hubTime)) continue;

                        // ###### END DEADHEAD ######

                        // ###### IDLE AFTER DEADHEAD ######

                        // check for idle; no idle if next stop is depot end
                        if (!idleAtStart && targetTrip != null) {
                            // handover allowed
                            if (handoverAllowed(targetTrip.StartLocation, idleTime)) {
                                // reset steering time
                                steeringTime = 0;
                                if (idleLocation.CrewBase) hubTime = 0;

                                // Finalize block dual costs by adding to actual costs
                                int idleStartTime = arc.StartTime + arc.DeadheadTemplate.Duration;
                                int idleEndTime = arc.EndTime;
                                Descriptor descriptor = blockDescriptorStart.AddEnd(targetTrip.StartLocation, idleStartTime);
                                costs -= blockDualCosts.GetValueOrDefault(descriptor);

                                // Initialize new range
                                blockDescriptorStart = new DescriptorHalf(targetTrip.StartLocation, idleEndTime);
                                List<double> blockDuals = blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart, noDualCost);
                                minBlockSavings = blockDuals.Min();
                                maxBlockSavings = blockDuals.Max();
                            }
                            else {
                                steeringTime += idleTime;
                                hubTime += idleTime;
                            }

                            // Charge during idle
                            if (canCharge) {
                                var res = idleLocation!.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                                costs += res.Cost;
                                SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);
                            }
                            // Losing charge during idle
                            else if (!idleLocation.FreeIdle) {
                                double idleChargeUsed = idleTime * vss.VehicleType.IdleUsage;
                                SoC -= idleChargeUsed;
                            }
                        }

                        if (!inBounds(SoC, steeringTime, hubTime)) continue;

                        // ###### END IDLE AFTER DEADHEAD ######
                        // trip itself is handled later

                        ChargeTime cl = idleAtStart
                            ? (canCharge ? ChargeTime.Source : ChargeTime.None)
                            : (canCharge ? ChargeTime.Target : ChargeTime.None);

                        processNewLabel(
                            SoC,
                            costs,
                            steeringTime,
                            hubTime,
                            canCharge ? idleLocation.Index : -1,
                            cl,
                            minBlockSavings,
                            maxBlockSavings,
                            blockDescriptorStart
                        );
                    }
                    // Check possible detours to charging locations for which the location is not already visited
                    for (int j = 0; j < vss.Instance.ChargingLocations.Count; j++) {
                        if (depotArc) continue;

                        Location candidateLocation = vss.Instance.ChargingLocations[j];
                        // Dont allow detour to already visited location
                        if (candidateLocation.Index == arc.DeadheadTemplate.StartLocation.Index ||
                            candidateLocation.Index == arc.DeadheadTemplate.EndLocation.Index) continue;

                        // Check if detour is possible
                        DeadheadTemplate? dht1 = vss.LocationDHT[arc.DeadheadTemplate.StartLocation.Index][candidateLocation.Index];
                        DeadheadTemplate? dht2 = vss.LocationDHT[candidateLocation.Index][arc.DeadheadTemplate.EndLocation.Index];

                        // Not possible to do deadhead 
                        if (dht1 == null || dht2 == null) continue;

                        Trip fromTrip = vss.Instance.Trips[expandingLabelIndex];
                        Trip toTrip = vss.Instance.Trips[targetIndex];
                        bool invalidFromTrip = dht1.FreqChangeOnly && (fromTrip.FrequencyChange == FrequencyChange.None || fromTrip.FrequencyChange == FrequencyChange.StartOfTrip);
                        bool invalidToTrip = dht2.FreqChangeOnly && (fromTrip.FrequencyChange == FrequencyChange.None || fromTrip.FrequencyChange == FrequencyChange.EndOfTrip);
                        if (invalidFromTrip || invalidToTrip) continue;

                        // No time to perform charge
                        int idleTime = (arc.EndTime - arc.StartTime) - (dht1.Duration + dht2.Duration);
                        if (idleTime < Constants.MIN_CHARGE_TIME) continue;

                        // There's time to perform the detour; calculate feasibility. 
                        // Always assume that detour is performed directly after previous label
                        int steeringTime = expandingLabel.SteeringTime;
                        int hubTime = expandingLabel.LastHubTime;
                        double SoC = expandingLabel.CurrActualSoC;
                        double costs = (dht1.Distance + dht2.Distance) * Constants.VH_M_COST;
                        double minBlockSavings = expandingLabel.MinBlockSavings;
                        double maxBlockSavings = expandingLabel.MaxBlockSavings;
                        DescriptorHalf blockDescriptorStart = expandingLabel.CurrentBlockStart;

                        // Perform first deadhead;
                        steeringTime += dht1.Duration;
                        hubTime += dht1.Duration;
                        SoC -= dht1.Distance * vss.VehicleType.DriveUsage;
                        if (!inBounds(SoC, steeringTime, hubTime)) continue;

                        // Check if we can reset steering/hub time now that we are at charging location
                        if (handoverAllowed(candidateLocation, idleTime)) {
                            steeringTime = 0;
                            if (candidateLocation.CrewBase) hubTime = 0;

                            // Finalize block dual costs by adding to actual costs
                            int idleStartTime = arc.StartTime + dht1.Duration;
                            int idleEndTime = arc.StartTime + dht1.Duration + idleTime;
                            Descriptor descriptor = blockDescriptorStart.AddEnd(candidateLocation, idleStartTime);
                            costs -= blockDualCosts.GetValueOrDefault(descriptor);

                            // Initialize new range
                            blockDescriptorStart = new DescriptorHalf(candidateLocation, idleEndTime);
                            List<double> blockDuals = blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart, noDualCost);
                            minBlockSavings = blockDuals.Min();
                            maxBlockSavings = blockDuals.Max();
                        }

                        // Perform charge at charging location  
                        var res = candidateLocation.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                        costs += res.Cost;
                        SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);

                        // Perform second deadhead
                        steeringTime += dht2.Duration;
                        hubTime += dht2.Duration;
                        SoC -= dht2.Distance * vss.VehicleType.DriveUsage;
                        if (!inBounds(SoC, steeringTime, hubTime)) continue;

                        processNewLabel(
                            SoC,
                            costs,
                            steeringTime,
                            hubTime,
                            candidateLocation.Index,
                            ChargeTime.Detour,
                            minBlockSavings,
                            maxBlockSavings,
                            blockDescriptorStart
                        );
                    }
                }
            }

            // Perform cost correction for charge at end
            foreach (var finalLabel in allLabels[^1]) {
                finalLabel.CurrCosts += Math.Max(0, vss.VehicleType.StartSoC - finalLabel.CurrActualSoC)
                    * vss.VehicleType.Capacity / 100 * Constants.KWH_COST;
            }
        }

        private List<(double reducedCost, VehicleTask vehicleTask)> extractTasks(string source, HashSet<BitArray> alreadyKnown) {

            // Backtrack in order to get path
            // indexes to nodes
            List<VSPLabel> feasibleEnds = allLabels[^1]
                .Where(x => {
                    if (alreadyKnown.Contains(x.CoveredTrips)) return false;
                    int c = 0;
                    for (int i = 0; i < x.CoveredTrips.Count; i++) if (x.CoveredTrips[i]) c++;
                    if (c < Config.VSP_LB_MIN_TRIPS) return false;
                    return true;
                })
                .OrderBy(x => x.CurrCosts).ToList();

            List<VSPLabel> validEnds = [];

            // First try to add fully disjoint labels
            BitArray currCover = new(vss.Instance.Trips.Count);

            for (int i = 0; i < feasibleEnds.Count && validEnds.Count < Config.VSP_LB_MAX_COLS; i++) {
                var label = feasibleEnds[i];
                if (alreadyKnown.Contains(label.CoveredTrips)) continue;

                BitArray ba = new(currCover);
                if (ba.And(label.CoveredTrips).HasAnySet()) continue;
                validEnds.Add(label);
                alreadyKnown.Add(label.CoveredTrips);
                currCover.Or(label.CoveredTrips);
            }
            // Then keep on filling with unique until max is reached / no more labels
            for (int i = 0; i < feasibleEnds.Count && validEnds.Count < Config.VSP_LB_MAX_COLS; i++) {
                var label = feasibleEnds[i];
                if (alreadyKnown.Contains(label.CoveredTrips)) continue;
                validEnds.Add(label);
                alreadyKnown.Add(label.CoveredTrips);
            }

            List<(double cost, VehicleTask vehicleTask)> results = [];

            for (int validEndIndex = 0; validEndIndex < validEnds.Count; validEndIndex++) {
                var validEnd = validEnds[validEndIndex];
                List<(int nodeIndex, VSPLabel label)> path = [(
                    allLabels.Count - 1,
                    validEnd
                )];
                double minCosts = path[0].label.CurrCosts;
                while (path[^1].label.PrevId != -1) {
                    var prev = allLabels[path[^1].label.PrevNodeIndex].Find(x => x.Id == path[^1].label.PrevId) ?? throw new Exception("Could not backtrace");
                    path.Add((path[^1].label.PrevNodeIndex, prev));
                }
                path.Reverse();

                List<VehicleElement> taskElements = [];
                for (int i = 1; i < path.Count - 1; i++) {
                    var curr = path[i];
                    var prev = path[i - 1];
                    Trip? trip = vss.Instance.Trips[curr.nodeIndex];

                    int currTime = taskElements.Count > 0
                        ? taskElements[^1].EndTime
                        : trip!.StartTime - vss.AdjFull[^2][curr.nodeIndex]!.DeadheadTemplate.Duration;
                    double CurrSoC = taskElements.Count > 0
                        ? taskElements[^1].EndSoCInTask
                        : vss.VehicleType.StartSoC;

                    // Depending on detour not: 
                    // (idle) -> dh -> (idle) -> trip; never charge when going to/from depot
                    // dh -> charge -> dh -> trip; 

                    if (curr.label.ChargeTime == ChargeTime.Detour) {
                        int fromIndex = prev.nodeIndex < vss.Instance.Trips.Count
                            ? vss.Instance.Trips[prev.nodeIndex].EndLocation.Index
                            : vss.Instance.Locations.FindIndex(x => x.IsDepot);
                        int toIndex = trip.StartLocation.Index;
                        DeadheadTemplate dht1 = vss.LocationDHT[fromIndex][curr.label.ChargeLocationIndex]!;
                        DeadheadTemplate dht2 = vss.LocationDHT[curr.label.ChargeLocationIndex][toIndex]!;

                        int idleTime = trip!.StartTime - dht1.Duration - dht2.Duration - currTime;

                        taskElements.Add(new VEDeadhead(dht1, currTime, currTime + dht1.Duration, vss.VehicleType) {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht1.Distance * vss.VehicleType.DriveUsage,
                        });

                        currTime += dht1.Duration;
                        CurrSoC -= dht1.Distance * vss.VehicleType.DriveUsage;

                        var chargeRes = vss.Instance.Locations[curr.label.ChargeLocationIndex]
                                            .ChargingCurves[vss.VehicleType.Index]
                                            .MaxChargeGained(CurrSoC, idleTime);

                        taskElements.Add(new VECharge(vss.Instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, chargeRes.SoCGained, chargeRes.Cost) {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = Math.Min(CurrSoC + chargeRes.SoCGained, vss.VehicleType.MaxSoC)
                        });

                        CurrSoC = Math.Min(CurrSoC + chargeRes.SoCGained, vss.VehicleType.MaxSoC);
                        currTime += idleTime;

                        taskElements.Add(new VEDeadhead(dht2, currTime, currTime + dht2.Duration, vss.VehicleType) {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht2.Distance * vss.VehicleType.DriveUsage,
                        });

                        currTime += dht2.Duration;
                        CurrSoC -= dht2.Distance * vss.VehicleType.DriveUsage;
                    }
                    else {
                        DeadheadTemplate dht = vss.AdjFull[prev.nodeIndex][curr.nodeIndex]!.DeadheadTemplate;
                        int idleTime = trip!.StartTime - dht.Duration - currTime;

                        // Charge before deadhead
                        if (curr.label.ChargeTime == ChargeTime.Source) {
                            var res = vss.Instance.Locations[curr.label.ChargeLocationIndex]
                                              .ChargingCurves[vss.VehicleType.Index]
                                              .MaxChargeGained(CurrSoC, idleTime);

                            taskElements.Add(new VECharge(vss.Instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost) {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = Math.Min(CurrSoC + res.SoCGained, vss.VehicleType.MaxSoC)
                            });

                            CurrSoC = Math.Min(CurrSoC + res.SoCGained, vss.VehicleType.MaxSoC);
                            currTime += idleTime;
                        }

                        // Deadhead
                        taskElements.Add(new VEDeadhead(dht, currTime, currTime + dht.Duration, vss.VehicleType) {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht.Distance * vss.VehicleType.DriveUsage,
                        });
                        currTime += dht.Duration;
                        CurrSoC -= dht.Distance * vss.VehicleType.DriveUsage;

                        // Charge after deadhead
                        if (curr.label.ChargeTime == ChargeTime.Target) {
                            var res = vss.Instance.Locations[curr.label.ChargeLocationIndex]
                                              .ChargingCurves[vss.VehicleType.Index]
                                              .MaxChargeGained(CurrSoC, idleTime);

                            taskElements.Add(new VECharge(vss.Instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost) {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = Math.Min(CurrSoC + res.SoCGained, vss.VehicleType.MaxSoC)
                            });

                            CurrSoC = Math.Min(CurrSoC + res.SoCGained, vss.VehicleType.MaxSoC);
                            currTime += idleTime;
                        }

                        if (currTime != trip.StartTime) {
                            int timeIndIdle = trip.StartTime - currTime;
                            taskElements.Add(new VEIdle(trip.StartLocation, currTime, trip.StartTime) {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = CurrSoC - timeIndIdle * vss.VehicleType.IdleUsage
                            });

                            CurrSoC -= timeIndIdle * vss.VehicleType.IdleUsage;
                        }
                    }

                    // add trip
                    if (curr.nodeIndex < vss.Instance.Trips.Count) {
                        double tripUsage = vss.VehicleType.DriveUsage * vss.Instance.Trips[curr.nodeIndex].Distance;
                        taskElements.Add(new VETrip(vss.Instance.Trips[curr.nodeIndex], vss.VehicleType) {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - tripUsage,
                        });

                        currTime = vss.Instance.Trips[curr.nodeIndex].EndTime;
                        CurrSoC -= tripUsage;
                    }
                }

                // Add final deadhead to depot
                DeadheadTemplate dhtToDepot = vss.AdjFull[path[^2].nodeIndex][^1]!.DeadheadTemplate;
                int endTime = taskElements[^1].EndTime;
                taskElements.Add(new VEDeadhead(dhtToDepot, endTime, endTime + dhtToDepot.Duration, vss.VehicleType) {
                    StartSoCInTask = taskElements[^1].EndSoCInTask,
                    EndSoCInTask = taskElements[^1].EndSoCInTask - dhtToDepot.Distance * vss.VehicleType.DriveUsage,
                });

                results.Add((minCosts, new VehicleTask(taskElements) {
                    vehicleType = vss.VehicleType,
                    Source = $"Labeling {source} - {validEndIndex}",
                }));
            }

            return results.Where(x => x.vehicleTask != null).ToList();
        }

        public List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks(bool isSubprocess, HashSet<BitArray> alreadyKnown) {
            ResetLabels();
            runLabeling();
            List<(double reducedCost, VehicleTask vehicleTask)> primaryTasks =
                extractTasks(isSubprocess ? "secondary" : "primary", alreadyKnown);

            if (isSubprocess) return primaryTasks;

            int numberSubprocesses = Math.Min(primaryTasks.Count, Config.VSP_LB_SEC_COL_COUNT);
            var secondaryTasks = new List<(double reducedCost, VehicleTask vehicleTask)>[numberSubprocesses];
            for (int i = 0; i < numberSubprocesses; i++) secondaryTasks[i] = [];

            Parallel.For(0, numberSubprocesses, (i) => {
                HashSet<BitArray> alreadyKnownLocal = new(alreadyKnown, new BitArrayComparer());
                var baseTask = primaryTasks[i];
                List<(double reducedCost, VehicleTask vehicleTask)> localTasks = [];

                var subprocess = new VSPLabeling(vss);
                subprocess.UpdateDualCosts(tripDualCosts, blockDualCosts, blockDualCostsByStart);

                for (int j = 0; j < Config.VSP_LB_SEC_COL_ATTEMPTS; j++) {
                    subprocess.ResetLabels();
                    for (int k = 0; k < subprocess.blockedNodes.Count; k++) subprocess.blockedNodes[i] = false;

                    // Determine blocked nodes;
                    int half = Config.VSP_LB_SEC_COL_ATTEMPTS / 2;
                    int maxBlocked = baseTask.vehicleTask.TripIndexCover.Count;
                    bool fromStart = j < (half + 1);
                    int totalBlocked = (int)((double)(j % (half + 1)) * maxBlocked / (half + 1));

                    int startBlock = j < (half + 1) ? 0 : totalBlocked;
                    int endBlock = j < (half + 1) ? totalBlocked : maxBlocked;

                    for (int k = startBlock; k < endBlock; k++)
                        subprocess.blockedNodes[baseTask.vehicleTask.TripIndexCover[k]] = true;

                    localTasks.AddRange(subprocess.GenerateVehicleTasks(true, alreadyKnownLocal));
                }

                secondaryTasks[i] = localTasks;
            });

            foreach (var st in secondaryTasks) primaryTasks.AddRange(st);
            return primaryTasks;
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks() =>
            GenerateVehicleTasks(false, new(new BitArrayComparer()));
    }
}
