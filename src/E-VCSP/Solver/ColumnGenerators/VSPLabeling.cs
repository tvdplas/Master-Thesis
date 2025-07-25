﻿using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;
using Gurobi;
using System.Collections;

namespace E_VCSP.Solver.ColumnGenerators {
    public enum ChargeTime {
        None,
        Source,
        Target,
        Detour,
    }

    public class VSPLabel {
        public static int ID_COUNTER = 0;
        public int Id;
        public int PrevNodeIndex; // Previously visited node index
        public int PrevId; // Label preceding this label 
        public int ChargeLocationIndex = -1; // Charge location used in this node
        public int SteeringTime = 0; // Time since a handover was visited
        public int LastHubTime = 0; // Time since a stop with hub capibility was visited
        public required ChargeTime ChargeTime; // Was charge at start / end of deadhead
        public BitArray CoveredTrips;
        public double CurrSoC; // SoC at the end of trip

        public double CurrCosts;  // Actual costs incurred at the end of trip, including blocks already completed

        public string CurrentBlockStart = ""; // Origin block start
        public double MaxBlockSavings; // Min costs that can be made based on block reduced cost
        public double MinBlockSavings; // Max costs that can be made based on block reduced cost

        public VSPLabel() {
            this.Id = ID_COUNTER++;
        }
    }

    public class VSPFront {
        private SortedList<double, VSPLabel> front = [];

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
                if (front.Keys[m] >= label.CurrSoC - Config.VSP_LB_CHARGE_EPSILON) r = m;
                else l = m + 1;
            }

            List<int> sameCharge = new();
            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++) {
                // Skip insertion if new label is dominated by an existing one
                VSPLabel existing = front.Values[i];
                if (existing.CurrCosts - existing.MinBlockSavings <= label.CurrCosts - label.MaxBlockSavings)
                    return 0;

                // If domination is not found however we are within epsilon range of the inserted label, 
                // add it to a seperate list of items we might remove if the new label dominates it
                if (Math.Abs(front.Values[i].CurrSoC - label.CurrSoC) < Config.VSP_LB_CHARGE_EPSILON) sameCharge.Add(i);
            }

            // Keep track of front size diff
            int diff = 0;

            // Remove labels within epsilon range if they are dominated by the new label
            for (int i = sameCharge.Count - 1; i >= 0; i--) {
                VSPLabel existing = front.Values[sameCharge[i]];
                if (existing.CurrCosts - existing.MaxBlockSavings >= label.CurrSoC - label.MinBlockSavings) {
                    front.RemoveAt(sameCharge[i]);
                    diff--;
                }
            }

            // Remove all items which are strictly dominated
            for (int i = l - 1; i >= 0 && i < front.Count; i--) {
                var existing = front.Values[i];
                if (existing.CurrCosts - existing.MaxBlockSavings >= label.CurrSoC - label.MinBlockSavings) {
                    front.RemoveAt(i);
                    diff--;
                }
            }

            // Lastly, insert new item. First path should never be taken
            if (front.ContainsKey(label.CurrSoC)) front[label.CurrSoC] = label;
            else {
                front[label.CurrSoC] = label;
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
        private List<List<VSPLabel>> allLabels = [];
        private List<VSPFront> activeLabels = [];

        private List<double> tripDualCosts = [];
        private Dictionary<string, double> blockDualCosts = [];
        private Dictionary<string, List<double>> blockDualCostsByStart = [];
        private List<bool> blockedNodes = [];

        public VSPLabeling(GRBModel model, VehicleSolutionState vss) : base(model, vss) { }
        private int addLabel(VSPLabel spl, int index) {
            allLabels[index].Add(spl);
            return activeLabels[index].Insert(spl);
        }

        private void reset() {
            allLabels.Clear();
            activeLabels.Clear();
            tripDualCosts.Clear();
            blockedNodes.Clear();
            blockDualCosts.Clear();
            blockDualCostsByStart.Clear();

            for (int i = 0; i < vss.Nodes.Count; i++) {
                allLabels.Add([]);
                activeLabels.Add(new());
                blockedNodes.Add(false);
            }

            for (int i = 0; i < vss.Instance.Trips.Count; i++) {
                tripDualCosts.Add(model.GetConstrByName("cover_trip_" + i).Pi);
            }

            var allConstrs = model.GetConstrs();
            foreach (var constr in allConstrs) {
                string name = constr.ConstrName;
                if (constr.ConstrName.StartsWith("cover_block_")) {
                    string descriptor = name.Split("_")[^1];
                    string descriptorStart = String.Join("#", descriptor.Split("#").Take(2));
                    blockDualCosts[descriptor] = constr.Pi;
                    if (blockDualCostsByStart.ContainsKey(descriptorStart)) blockDualCostsByStart[descriptorStart].Add(constr.Pi);
                    else blockDualCostsByStart[descriptorStart] = [constr.Pi];
                }
            }
        }

        private void runLabeling() {
            bool handoverAllowed(Location l, int idleTime) => l.HandoverAllowed && idleTime >= l.MinHandoverTime;

            // Initial label at start of depot; note: no cost range yet, as 
            // we cannot be inside of a block yet
            int activeLabelCount = addLabel(new VSPLabel() {
                PrevId = -1,
                PrevNodeIndex = vss.Nodes.Count - 2,
                CurrCosts = Config.VH_PULLOUT_COST,
                MaxBlockSavings = 0,
                MinBlockSavings = 0,
                CurrSoC = vss.VehicleType.StartSoC,
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

                    BitArray newCover = new BitArray(expandingLabel.CoveredTrips);
                    if (targetTrip != null) newCover[targetTrip.Index] = true;

                    // List of vehicle statstics achievable at start of trip.
                    List<(
                        double SoC,
                        double cost,
                        int steeringTime,
                        int hubTime,
                        int chargeIndex,
                        ChargeTime chargeTime,
                        double minBlockSavings,
                        double maxBlockSavings,
                        string blockDescriptorStart
                    )> statsAtTrip = [];

                    // Direct trip
                    {
                        bool depotStart = expandingLabelIndex >= vss.Instance.Trips.Count,
                             depotEnd = targetIndex >= vss.Instance.Trips.Count;

                        int idleTime = depotStart || depotEnd
                            ? 0
                            : vss.Instance.Trips[targetIndex].StartTime - vss.Instance.Trips[expandingLabelIndex].EndTime - arc.DeadheadTemplate.Duration;

                        int canCharge = 0;
                        if (idleTime > Config.MIN_CHARGE_TIME && !depotStart && vss.Instance.Trips[expandingLabelIndex].To.CanCharge) canCharge += 1;
                        if (idleTime > Config.MIN_CHARGE_TIME && !depotEnd && vss.Instance.Trips[targetIndex].From.CanCharge) canCharge += 2;

                        Location? chargeLocation = null;
                        if (canCharge > 0) {
                            chargeLocation = canCharge >= 2 ? vss.Instance.Trips[targetIndex].From : vss.Instance.Trips[expandingLabelIndex].To;
                        }

                        int steeringTime = expandingLabel.SteeringTime;
                        int hubTime = expandingLabel.LastHubTime;
                        double SoC = expandingLabel.CurrSoC;
                        double minBlockSavings = expandingLabel.MinBlockSavings;
                        double maxBlockSavings = expandingLabel.MaxBlockSavings;
                        string blockDescriptorStart = expandingLabel.CurrentBlockStart;

                        double costs = arc.DeadheadTemplate.Distance * Config.VH_M_COST;
                        if (targetIndex < vss.Instance.Trips.Count) costs -= tripDualCosts[targetIndex];

                        if (canCharge == 0) steeringTime += idleTime;

                        if (canCharge == 1) {
                            var res = chargeLocation!.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                            costs += res.Cost;
                            SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);

                            if (handoverAllowed(chargeLocation, idleTime)) {
                                steeringTime = 0;

                                // Finalize block dual costs by adding to actual costs
                                int locationIndex = chargeLocation.Index;
                                int idleStartTime = arc.StartTime;
                                int idleEndTime = arc.StartTime + idleTime;
                                string descriptor = blockDescriptorStart + $"#{locationIndex}#{idleStartTime}";
                                costs -= blockDualCosts.GetValueOrDefault(descriptor);

                                // Initialize new range
                                blockDescriptorStart = $"{locationIndex}#{idleEndTime}";
                                minBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Min();
                                maxBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Max();
                            }
                        }

                        // Initialize block descriptor start
                        if (depotStart && !depotEnd) {
                            blockDescriptorStart = $"{depot.Index}#{arc.StartTime}";
                            minBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Min();
                            maxBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Max();
                        }

                        SoC -= arc.DeadheadTemplate.Distance * vss.VehicleType.DriveUsage;
                        steeringTime += arc.DeadheadTemplate.Duration;
                        hubTime += arc.DeadheadTemplate.Duration;
                        if (SoC < vss.VehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        if (canCharge >= 2) {
                            var res = chargeLocation!.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                            costs += res.Cost;
                            SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);
                        }
                        if (targetTrip != null && handoverAllowed(targetTrip.From, idleTime)) {
                            steeringTime = 0;

                            // Finalize block dual costs by adding to actual costs
                            int locationIndex = targetTrip.From.Index;
                            int idleStartTime = arc.StartTime + arc.DeadheadTemplate.Duration;
                            int idleEndTime = arc.EndTime;
                            string descriptor = blockDescriptorStart + $"#{locationIndex}#{idleStartTime}";
                            costs -= blockDualCosts.GetValueOrDefault(descriptor);

                            // Initialize new range
                            blockDescriptorStart = $"{locationIndex}#{idleEndTime}";
                            minBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Min();
                            maxBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Max();
                        }
                        if (targetTrip != null && targetTrip.From.CrewHub && idleTime >= targetTrip.From.MinHandoverTime) {
                            hubTime = 0;
                        }

                        if (canCharge == 0 && targetTrip != null && !targetTrip.From.FreeIdle) {
                            // Idle is after adjust SoC
                            SoC -= idleTime * vss.VehicleType.IdleUsage;
                            if (SoC < vss.VehicleType.MinSoC) continue;
                        }

                        ChargeTime cl = canCharge switch {
                            0 => ChargeTime.None,
                            1 => ChargeTime.Source,
                            2 => ChargeTime.Target,
                            3 => ChargeTime.Target,
                            _ => throw new InvalidDataException("not possible")
                        };

                        statsAtTrip.Add((
                            SoC,
                            costs,
                            steeringTime,
                            hubTime,
                            chargeLocation?.Index ?? -1,
                            cl,
                            minBlockSavings,
                            maxBlockSavings,
                            blockDescriptorStart
                        ));
                    }
                    // Check possible detours to charging locations for which the location is not already visited
                    for (int j = 0; j < vss.Instance.ChargingLocations.Count; j++) {
                        Location candidateLocation = vss.Instance.ChargingLocations[j];
                        // Dont allow detour to already visited location
                        if (candidateLocation.Index == arc.DeadheadTemplate.From.Index ||
                            candidateLocation.Index == arc.DeadheadTemplate.To.Index) continue;

                        // Check if detour is possible
                        DeadheadTemplate? dht1 = vss.LocationDHT[arc.DeadheadTemplate.From.Index][candidateLocation.Index];
                        DeadheadTemplate? dht2 = vss.LocationDHT[candidateLocation.Index][arc.DeadheadTemplate.To.Index];

                        // Not possible to do deadhead 
                        if (dht1 == null || dht2 == null) continue;

                        // No time to perform charge
                        int idleTime = (arc.EndTime - arc.StartTime) - (dht1.Duration + dht2.Duration);
                        if (idleTime < Config.MIN_CHARGE_TIME) continue;

                        // There's time to perform the detour; calculate feasibility. 
                        // Always assume that detour is performed directly after previous label
                        int steeringTime = expandingLabel.SteeringTime;
                        int hubTime = expandingLabel.LastHubTime;
                        double SoC = expandingLabel.CurrSoC;
                        double costs = (dht1.Distance + dht2.Distance) * Config.VH_M_COST;
                        double minBlockSavings = expandingLabel.MinBlockSavings;
                        double maxBlockSavings = expandingLabel.MaxBlockSavings;
                        string blockDescriptorStart = expandingLabel.CurrentBlockStart;

                        // Perform first deadhead;
                        steeringTime += dht1.Duration;
                        hubTime += dht1.Duration;
                        SoC -= dht1.Distance * vss.VehicleType.DriveUsage;
                        if (SoC < vss.VehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        // Check if we can reset steering/hub time now that we are at charging location
                        if (handoverAllowed(candidateLocation, idleTime)) {
                            steeringTime = 0;

                            // Finalize block dual costs by adding to actual costs
                            int locationIndex = candidateLocation.Index;
                            int idleStartTime = arc.StartTime + dht1.Duration;
                            int idleEndTime = arc.StartTime + dht1.Duration + idleTime;
                            string descriptor = blockDescriptorStart + $"#{locationIndex}#{idleStartTime}";
                            costs -= blockDualCosts.GetValueOrDefault(descriptor);

                            // Initialize new range
                            blockDescriptorStart = $"{locationIndex}#{idleEndTime}";
                            minBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Min();
                            maxBlockSavings = (blockDualCostsByStart.GetValueOrDefault(blockDescriptorStart) ?? [0]).Max();
                        }
                        if (candidateLocation.CrewHub && idleTime >= candidateLocation.MinHandoverTime)
                            hubTime = 0;

                        // Perform charge at charging location  
                        var res = candidateLocation.ChargingCurves[vss.VehicleType.Index].MaxChargeGained(SoC, idleTime);
                        costs += res.Cost;
                        SoC = Math.Min(SoC + res.SoCGained, vss.VehicleType.MaxSoC);

                        // Perform second deadhead
                        steeringTime += dht2.Duration;
                        hubTime += dht2.Duration;
                        SoC -= dht2.Distance * vss.VehicleType.DriveUsage;
                        if (SoC < vss.VehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        statsAtTrip.Add((
                            SoC,
                            costs,
                            steeringTime,
                            hubTime,
                            candidateLocation.Index,
                            ChargeTime.Detour,
                            minBlockSavings,
                            maxBlockSavings,
                            blockDescriptorStart
                        ));
                    }

                    for (int j = 0; j < statsAtTrip.Count; j++) {
                        var stats = statsAtTrip[j];

                        stats.SoC -= (targetTrip?.Distance ?? 0) * vss.VehicleType.DriveUsage;
                        stats.steeringTime += targetTrip?.Duration ?? 0;
                        stats.hubTime += targetTrip?.Duration ?? 0;
                        stats.cost += (targetTrip?.Distance ?? 0) * Config.VH_M_COST;
                        if (stats.SoC < vss.VehicleType.MinSoC || stats.steeringTime >= Config.MAX_STEERING_TIME || stats.hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        activeLabelCount += addLabel(new VSPLabel() {
                            PrevId = expandingLabel.Id,
                            PrevNodeIndex = expandingLabelIndex,
                            ChargeTime = stats.chargeTime,
                            ChargeLocationIndex = stats.chargeIndex,
                            CurrCosts = expandingLabel.CurrCosts + stats.cost,
                            CurrSoC = stats.SoC,
                            SteeringTime = stats.steeringTime,
                            LastHubTime = stats.hubTime,
                            CoveredTrips = newCover,
                            CurrentBlockStart = stats.blockDescriptorStart,
                            MaxBlockSavings = stats.maxBlockSavings,
                            MinBlockSavings = stats.minBlockSavings,
                        }, arc.To.Index);
                    }
                }
            }

            // Perform cost correction for charge at end
            foreach (var finalLabel in allLabels[^1]) {
                finalLabel.CurrCosts += Math.Max(0, vss.VehicleType.StartSoC - finalLabel.CurrSoC)
                    * vss.VehicleType.Capacity / 100 * Config.KWH_COST;
            }
        }

        private List<(double reducedCost, VehicleTask vehicleTask)> extractTasks(string source, HashSet<BitArray> alreadyFound) {
            // Backtrack in order to get path
            // indexes to nodes
            List<VSPLabel> feasibleEnds = allLabels[^1]
                .Where(x => x.CurrCosts < 0 && !alreadyFound.Contains(x.CoveredTrips))
                .OrderBy(x => x.CurrCosts).ToList();

            List<VSPLabel> validEnds = [];
            // Make trying to add disjoint labels
            if (Config.VSP_LB_ATTEMPT_DISJOINT) {
                BitArray currCover = new(vss.Instance.Trips.Count);
                for (int i = 0; i < feasibleEnds.Count; i++) {
                    if (alreadyFound.Contains(feasibleEnds[i].CoveredTrips)) continue;
                    BitArray ba = new(currCover);
                    if (ba.And(feasibleEnds[i].CoveredTrips).HasAnySet()) continue;
                    validEnds.Add(feasibleEnds[i]);
                    alreadyFound.Add(feasibleEnds[i].CoveredTrips);
                    currCover.Or(feasibleEnds[i].CoveredTrips);
                }
            }
            // Keep on filling with unique until max is reached / no more labels
            for (int i = 0; i < feasibleEnds.Count && validEnds.Count < Config.VSP_LB_MAX_COLS; i++) {
                if (alreadyFound.Contains(feasibleEnds[i].CoveredTrips)) continue;
                validEnds.Add(feasibleEnds[i]);
                alreadyFound.Add(feasibleEnds[i].CoveredTrips);
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
                    // (charge) -> dh -> (charge) -> trip; never charge when going to/from depot
                    // dh -> charge -> dh -> trip; 

                    if (curr.label.ChargeTime == ChargeTime.Detour) {
                        int fromIndex = prev.nodeIndex < vss.Instance.Trips.Count
                            ? vss.Instance.Trips[prev.nodeIndex].To.Index
                            : vss.Instance.Locations.FindIndex(x => x.IsDepot);
                        int toIndex = trip.From.Index;
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
                            taskElements.Add(new VEIdle(trip.From, currTime, trip.StartTime) {
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
                    Source = $"Labeling {source} -  {validEndIndex}",
                }));
            }

            return results;
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks() {
            reset();

            if (model == null || model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            runLabeling();
            List<(double reducedCost, VehicleTask vehicleTask)> primaryTasks = extractTasks("primary", new());
            List<(double reducedCost, VehicleTask vehicleTask)> secondaryTasks = [];

            HashSet<BitArray> alreadyFound = new(new BitArrayComparer());
            foreach (var x in primaryTasks) alreadyFound.Add(x.vehicleTask.ToBitArray(vss.Instance.Trips.Count));

            for (int i = 0; i < Math.Min(primaryTasks.Count, Config.VSP_LB_SEC_COL_COUNT); i++) {
                var baseTask = primaryTasks[i];

                for (int j = 1; j < (Config.VSP_LB_SEC_COL_ATTEMPTS + 1); j++) {
                    reset();
                    for (int k = 0; k < (int)((double)j * baseTask.vehicleTask.TripCover.Count / (Config.VSP_LB_SEC_COL_ATTEMPTS + 1)); k++) {
                        blockedNodes[baseTask.vehicleTask.TripCover[k]] = true;
                    }
                    runLabeling();
                    var newExtracted = extractTasks("secondary", alreadyFound);
                    secondaryTasks.AddRange(newExtracted);
                    foreach (var x in newExtracted) alreadyFound.Add(x.vehicleTask.ToBitArray(vss.Instance.Trips.Count));
                }
            }

            primaryTasks.AddRange(secondaryTasks);
            return primaryTasks;
        }
    }
}
