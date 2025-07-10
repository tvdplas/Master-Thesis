using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Utils;
using Gurobi;
using System.Collections;

namespace E_VCSP.Solver.ColumnGenerators
{
    public enum ChargeTime
    {
        None,
        Source,
        Target,
        Detour,
    }

    public class VSPLabel
    {
        public static int ID_COUNTER = 0;
        public int Id;
        public int PrevNodeIndex; // Previously visited node index
        public int PrevId; // Label preceding this label 
        public int ChargeLocationIndex = -1; // Charge location used in this node
        public int SteeringTime = 0; // Time since a handover was visited
        public int LastHubTime = 0; // Time since a stop with hub capibility was visited
        public required ChargeTime ChargeTime; // Was charge at start / end of deadhead
        public BitArray CoveredTrips;
        public double CurrSoC; // SoC at start of node
        public double CurrCosts;  // Costs incurred at start of node

        public VSPLabel()
        {
            this.Id = ID_COUNTER++;
        }
    }

    public class VSPFront
    {
        private SortedList<double, VSPLabel> front = [];

        /// <summary>
        /// Inserts label into front
        /// </summary>
        /// <param name="label">Label to be inserted</param>
        /// <returns>The difference in amount of items currently in the front; min = -#items in front + 1, max = 1</returns>
        public int Insert(VSPLabel label)
        {
            int l = 0, r = front.Count;
            // find first item that is larger than or equal to label CurrSoC
            while (l < r)
            {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.CurrSoC - Config.VSP_LB_CHARGE_EPSILON) r = m;
                else l = m + 1;
            }

            List<int> sameCharge = new();
            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++)
            {
                // Skip if domination is found
                if (front.Values[i].CurrCosts <= label.CurrCosts) return 0;

                // If domination is not found however we are within epsilon range of the inserted label, 
                // add it to a seperate list of items we will be removing
                if (Math.Abs(front.Values[i].CurrSoC - label.CurrSoC) < Config.VSP_LB_CHARGE_EPSILON) sameCharge.Add(i);
            }

            // Keep track of front size diff
            int diff = 0;

            // Remove all labels in front in epsilon range from the inserted label, as none were dominating
            for (int i = sameCharge.Count - 1; i >= 0; i--)
            {
                front.RemoveAt(sameCharge[i]);
                diff--;
            }

            // Remove all items which are strictly dominated
            for (int i = l - 1; i >= 0 && i < front.Count; i--)
            {
                var b = front.Values[i];
                if (label.CurrCosts <= b.CurrCosts)
                {
                    front.RemoveAt(i);
                    diff--;
                }
            }

            // Lastly, insert new item. First path should never be taken
            if (front.ContainsKey(label.CurrSoC)) front[label.CurrSoC] = label;
            else
            {
                front[label.CurrSoC] = label;
                diff++;
            }
            return diff;
        }
        public int Count => front.Count;

        public void Clear() => front.Clear();

        public VSPLabel Pop()
        {
            int lastIndex = front.Count - 1;
            var label = front.Values[lastIndex];
            front.RemoveAt(lastIndex);
            return label;
        }
    }

    public class VSPLabeling : VehicleColumnGen
    {
        private List<List<VSPLabel>> allLabels = [];
        private List<VSPFront> activeLabels = [];
        private List<double> reducedCosts = [];

        private List<bool> blockedNodes = [];

        List<List<DeadheadTemplate?>> locationDHT = [];

        public VSPLabeling(
            GRBModel model,
            Instance instance,
            VehicleType vehicleType,
            List<EVSPNode> nodes,
            List<List<VSPArc?>> adjFull,
            List<List<VSPArc>> adj,
            List<List<DeadheadTemplate?>> locationDHT
        ) : base(
            model,
            instance,
            vehicleType,
            nodes,
            adjFull,
            adj
        )
        {
            this.locationDHT = locationDHT;
        }

        private int addLabel(VSPLabel spl, int index)
        {
            allLabels[index].Add(spl);
            return activeLabels[index].Insert(spl);
        }

        private void reset()
        {
            allLabels.Clear();
            activeLabels.Clear();
            reducedCosts.Clear();
            blockedNodes.Clear();

            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels.Add([]);
                activeLabels.Add(new());
                blockedNodes.Add(false);
            }

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCosts.Add(constrs[i].Pi);
            }
        }

        private void runLabeling()
        {
            int activeLabelCount = addLabel(new VSPLabel()
            {
                PrevId = -1,
                PrevNodeIndex = nodes.Count - 2,
                CurrCosts = 0,
                CurrSoC = vehicleType.StartSoC,
                ChargeTime = ChargeTime.None,
                CoveredTrips = new BitArray(instance.Trips.Count)
            }, nodes.Count - 2);

            while (activeLabelCount > 0)
            {
                // Get label to expand
                VSPLabel? expandingLabel = null;
                int expandingLabelIndex = -1;
                for (int i = 0; i < activeLabels.Count; i++)
                {
                    if (activeLabels[i].Count > 0)
                    {
                        expandingLabel = activeLabels[i].Pop();
                        expandingLabelIndex = i;
                        activeLabelCount--;
                        break;
                    }
                }
                if (expandingLabel == null) throw new InvalidDataException("Could not find active label to expand");

                // Try expand label
                for (int i = 0; i < adj[expandingLabelIndex].Count; i++)
                {
                    VSPArc arc = adj[expandingLabelIndex][i];

                    // Skip this node as it is already part of the primary solution
                    if (blockedNodes[arc.To.Index]) continue;

                    int targetIndex = arc.To.Index;
                    Trip? targetTrip = targetIndex < instance.Trips.Count ? instance.Trips[targetIndex] : null;

                    BitArray newCover = new BitArray(expandingLabel.CoveredTrips);
                    if (targetTrip != null) newCover[targetTrip.Index] = true;

                    // List of vehicle statstics achievable at start of trip.
                    List<(double SoC, double cost, int steeringTime, int hubTime, int chargeIndex, ChargeTime chargeTime)> statsAtTrip = [];

                    // Direct trip
                    {
                        bool depotStart = expandingLabelIndex >= instance.Trips.Count,
                             depotEnd = targetIndex >= instance.Trips.Count;

                        int idleTime = depotStart || depotEnd
                            ? 0
                            : instance.Trips[targetIndex].StartTime - instance.Trips[expandingLabelIndex].EndTime - arc.DeadheadTemplate.Duration;

                        int canCharge = 0;
                        if (idleTime > Config.MIN_CHARGE_TIME && !depotStart && instance.Trips[expandingLabelIndex].To.CanCharge) canCharge += 1;
                        if (idleTime > Config.MIN_CHARGE_TIME && !depotEnd && instance.Trips[targetIndex].From.CanCharge) canCharge += 2;

                        Location? chargeLocation = null;
                        if (canCharge > 0)
                        {
                            chargeLocation = canCharge >= 2 ? instance.Trips[targetIndex].From : instance.Trips[expandingLabelIndex].To;
                        }

                        int steeringTime = expandingLabel.SteeringTime;
                        int hubTime = expandingLabel.LastHubTime;
                        double SoC = expandingLabel.CurrSoC;
                        double costs = arc.DeadheadTemplate.Distance * Config.VH_M_COST;
                        if (targetIndex < instance.Trips.Count) costs -= reducedCosts[targetIndex];

                        if (canCharge == 0) steeringTime += idleTime;

                        if (canCharge == 1)
                        {
                            var res = chargeLocation!.ChargingCurves[vehicleType.Index].MaxChargeGained(SoC, idleTime);
                            costs += res.Cost;
                            SoC = Math.Min(SoC + res.SoCGained, vehicleType.MaxSoC);

                            if (chargeLocation.HandoverAllowed && idleTime >= Math.Max(chargeLocation.SignOffTime, chargeLocation.SignOnTime))
                            {
                                steeringTime = 0;
                            }
                        }

                        SoC -= arc.DeadheadTemplate.Distance * vehicleType.DriveUsage;
                        steeringTime += arc.DeadheadTemplate.Duration;
                        hubTime += arc.DeadheadTemplate.Duration;
                        if (SoC < vehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        if (canCharge >= 2)
                        {
                            var res = chargeLocation!.ChargingCurves[vehicleType.Index].MaxChargeGained(SoC, idleTime);
                            costs += res.Cost;
                            SoC = Math.Min(SoC + res.SoCGained, vehicleType.MaxSoC);
                        }
                        if (targetTrip != null && targetTrip.From.HandoverAllowed && idleTime >= targetTrip.From.MinHandoverTime)
                        {
                            steeringTime = 0;
                        }
                        if (targetTrip != null && targetTrip.From.CrewHub && idleTime >= targetTrip.From.MinHandoverTime)
                        {
                            hubTime = 0;
                        }

                        if (canCharge == 0 && targetTrip != null && !targetTrip.From.FreeIdle)
                        {
                            // Idle is after adjust SoC
                            SoC -= idleTime * vehicleType.IdleUsage;
                            if (SoC < vehicleType.MinSoC) continue;
                        }

                        ChargeTime cl = canCharge switch
                        {
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
                            cl
                        ));
                    }
                    // Check possible detours to charging locations for which the location is not already visited
                    for (int j = 0; j < instance.ChargingLocations.Count; j++)
                    {
                        Location candidateLocation = instance.ChargingLocations[j];
                        // Dont allow detour to already visited location
                        if (candidateLocation.Index == arc.DeadheadTemplate.From.Index ||
                            candidateLocation.Index == arc.DeadheadTemplate.To.Index) continue;

                        // Check if detour is possible
                        DeadheadTemplate? dht1 = locationDHT[arc.DeadheadTemplate.From.Index][candidateLocation.Index];
                        DeadheadTemplate? dht2 = locationDHT[candidateLocation.Index][arc.DeadheadTemplate.To.Index];

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

                        // Perform first deadhead;
                        steeringTime += dht1.Duration;
                        hubTime += dht1.Duration;
                        SoC -= dht1.Distance * vehicleType.DriveUsage;
                        if (SoC < vehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        // Check if we can reset steering/hub time now that we are at charging location
                        if (candidateLocation.HandoverAllowed && idleTime >= candidateLocation.MinHandoverTime)
                            steeringTime = 0;
                        if (candidateLocation.CrewHub && idleTime >= candidateLocation.MinHandoverTime)
                            hubTime = 0;

                        // Perform charge at charging location  
                        var res = candidateLocation.ChargingCurves[vehicleType.Index].MaxChargeGained(SoC, idleTime);
                        costs += res.Cost;
                        SoC = Math.Min(SoC + res.SoCGained, vehicleType.MaxSoC);

                        // Perform second deadhead
                        steeringTime += dht2.Duration;
                        hubTime += dht2.Duration;
                        SoC -= dht2.Distance * vehicleType.DriveUsage;
                        if (SoC < vehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME || hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        statsAtTrip.Add((
                            SoC,
                            costs,
                            steeringTime,
                            hubTime,
                            candidateLocation.Index,
                            ChargeTime.Detour
                        ));
                    }

                    for (int j = 0; j < statsAtTrip.Count; j++)
                    {
                        var stats = statsAtTrip[j];

                        stats.SoC -= (targetTrip?.Distance ?? 0) * vehicleType.DriveUsage;
                        stats.steeringTime += targetTrip?.Duration ?? 0;
                        stats.hubTime += targetTrip?.Duration ?? 0;
                        stats.cost += (targetTrip?.Distance ?? 0) * Config.VH_M_COST;
                        if (stats.SoC < vehicleType.MinSoC || stats.steeringTime >= Config.MAX_STEERING_TIME || stats.hubTime >= Config.MAX_NO_HUB_TIME) continue;

                        activeLabelCount += addLabel(new VSPLabel()
                        {
                            PrevId = expandingLabel.Id,
                            PrevNodeIndex = expandingLabelIndex,
                            ChargeTime = stats.chargeTime,
                            ChargeLocationIndex = stats.chargeIndex,
                            CurrCosts = expandingLabel.CurrCosts + stats.cost,
                            CurrSoC = stats.SoC,
                            SteeringTime = stats.steeringTime,
                            LastHubTime = stats.hubTime,
                            CoveredTrips = newCover,
                        }, arc.To.Index);
                    }
                }
            }
        }

        private List<(double reducedCost, VehicleTask vehicleTask)> extractTasks(string source, HashSet<BitArray> alreadyFound)
        {
            // Backtrack in order to get path
            // indexes to nodes
            List<VSPLabel> feasibleEnds = allLabels[^1]
                .Where(x => x.CurrCosts < 0 && !alreadyFound.Contains(x.CoveredTrips))
                .OrderBy(x => x.CurrCosts).ToList();

            List<VSPLabel> validEnds = [];
            // Make trying to add disjoint labels
            if (Config.VSP_LB_ATTEMPT_DISJOINT)
            {
                BitArray currCover = new(instance.Trips.Count);
                for (int i = 0; i < feasibleEnds.Count; i++)
                {
                    if (alreadyFound.Contains(feasibleEnds[i].CoveredTrips)) continue;
                    BitArray ba = new(currCover);
                    if (ba.And(feasibleEnds[i].CoveredTrips).HasAnySet()) continue;
                    validEnds.Add(feasibleEnds[i]);
                    alreadyFound.Add(feasibleEnds[i].CoveredTrips);
                    currCover.Or(feasibleEnds[i].CoveredTrips);
                }
            }
            // Keep on filling with unique until max is reached / no more labels
            for (int i = 0; i < feasibleEnds.Count && validEnds.Count < Config.VSP_LB_MAX_COLS; i++)
            {
                if (alreadyFound.Contains(feasibleEnds[i].CoveredTrips)) continue;
                validEnds.Add(feasibleEnds[i]);
                alreadyFound.Add(feasibleEnds[i].CoveredTrips);
            }

            List<(double cost, VehicleTask vehicleTask)> results = [];

            for (int validEndIndex = 0; validEndIndex < validEnds.Count; validEndIndex++)
            {
                var validEnd = validEnds[validEndIndex];
                List<(int nodeIndex, VSPLabel label)> path = [(
                    allLabels.Count - 1,
                    validEnd
                )];
                double minCosts = path[0].label.CurrCosts;
                while (path[^1].label.PrevId != -1)
                {
                    var prev = allLabels[path[^1].label.PrevNodeIndex].Find(x => x.Id == path[^1].label.PrevId) ?? throw new Exception("Could not backtrace");
                    path.Add((path[^1].label.PrevNodeIndex, prev));
                }
                path.Reverse();

                List<VehicleElement> taskElements = [];
                for (int i = 1; i < path.Count - 1; i++)
                {
                    var curr = path[i];
                    var prev = path[i - 1];
                    Trip? trip = instance.Trips[curr.nodeIndex];

                    int currTime = taskElements.Count > 0
                        ? taskElements[^1].EndTime
                        : trip!.StartTime - adjFull[^2][curr.nodeIndex]!.DeadheadTemplate.Duration;
                    double CurrSoC = taskElements.Count > 0
                        ? taskElements[^1].EndSoCInTask
                        : vehicleType.StartSoC;

                    // Depending on detour not: 
                    // (charge) -> dh -> (charge) -> trip; never charge when going to/from depot
                    // dh -> charge -> dh -> trip; 

                    if (curr.label.ChargeTime == ChargeTime.Detour)
                    {
                        int fromIndex = prev.nodeIndex < instance.Trips.Count
                            ? instance.Trips[prev.nodeIndex].To.Index
                            : instance.Locations.FindIndex(x => x.IsDepot);
                        int toIndex = trip.From.Index;
                        DeadheadTemplate dht1 = locationDHT[fromIndex][curr.label.ChargeLocationIndex]!;
                        DeadheadTemplate dht2 = locationDHT[curr.label.ChargeLocationIndex][toIndex]!;

                        int idleTime = trip!.StartTime - dht1.Duration - dht2.Duration - currTime;


                        taskElements.Add(new VEDeadhead(dht1, currTime, currTime + dht1.Duration, vehicleType)
                        {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht1.Distance * vehicleType.DriveUsage,
                        });

                        currTime += dht1.Duration;
                        CurrSoC -= dht1.Distance * vehicleType.DriveUsage;

                        var chargeRes = instance.Locations[curr.label.ChargeLocationIndex]
                                            .ChargingCurves[vehicleType.Index]
                                            .MaxChargeGained(CurrSoC, idleTime);

                        taskElements.Add(new VECharge(instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, chargeRes.SoCGained, chargeRes.Cost)
                        {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = Math.Min(CurrSoC + chargeRes.SoCGained, vehicleType.MaxSoC)
                        });

                        CurrSoC = Math.Min(CurrSoC + chargeRes.SoCGained, vehicleType.MaxSoC);
                        currTime += idleTime;

                        taskElements.Add(new VEDeadhead(dht2, currTime, currTime + dht2.Duration, vehicleType)
                        {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht2.Distance * vehicleType.DriveUsage,
                        });

                        currTime += dht2.Duration;
                        CurrSoC -= dht2.Distance * vehicleType.DriveUsage;
                    }
                    else
                    {
                        DeadheadTemplate dht = adjFull[prev.nodeIndex][curr.nodeIndex]!.DeadheadTemplate;
                        int idleTime = trip!.StartTime - dht.Duration - currTime;

                        // Charge before deadhead
                        if (curr.label.ChargeTime == ChargeTime.Source)
                        {
                            var res = instance.Locations[curr.label.ChargeLocationIndex]
                                              .ChargingCurves[vehicleType.Index]
                                              .MaxChargeGained(CurrSoC, idleTime);

                            taskElements.Add(new VECharge(instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost)
                            {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = Math.Min(CurrSoC + res.SoCGained, vehicleType.MaxSoC)
                            });

                            CurrSoC = Math.Min(CurrSoC + res.SoCGained, vehicleType.MaxSoC);
                            currTime += idleTime;
                        }

                        // Deadhead
                        taskElements.Add(new VEDeadhead(dht, currTime, currTime + dht.Duration, vehicleType)
                        {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - dht.Distance * vehicleType.DriveUsage,
                        });
                        currTime += dht.Duration;
                        CurrSoC -= dht.Distance * vehicleType.DriveUsage;


                        // Charge after deadhead
                        if (curr.label.ChargeTime == ChargeTime.Target)
                        {
                            var res = instance.Locations[curr.label.ChargeLocationIndex]
                                              .ChargingCurves[vehicleType.Index]
                                              .MaxChargeGained(CurrSoC, idleTime);

                            taskElements.Add(new VECharge(instance.Locations[curr.label.ChargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost)
                            {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = Math.Min(CurrSoC + res.SoCGained, vehicleType.MaxSoC)
                            });

                            CurrSoC = Math.Min(CurrSoC + res.SoCGained, vehicleType.MaxSoC);
                            currTime += idleTime;
                        }

                        if (currTime != trip.StartTime)
                        {
                            int timeIndIdle = trip.StartTime - currTime;
                            taskElements.Add(new VEIdle(trip.From, currTime, trip.StartTime)
                            {
                                StartSoCInTask = CurrSoC,
                                EndSoCInTask = CurrSoC - timeIndIdle * vehicleType.IdleUsage
                            });

                            CurrSoC -= timeIndIdle * vehicleType.IdleUsage;
                        }
                    }

                    // add trip
                    if (curr.nodeIndex < instance.Trips.Count)
                    {
                        double tripUsage = vehicleType.DriveUsage * instance.Trips[curr.nodeIndex].Distance;
                        taskElements.Add(new VETrip(instance.Trips[curr.nodeIndex], vehicleType)
                        {
                            StartSoCInTask = CurrSoC,
                            EndSoCInTask = CurrSoC - tripUsage,
                        });

                        currTime = instance.Trips[curr.nodeIndex].EndTime;
                        CurrSoC -= tripUsage;
                    }
                }

                // Add final deadhead to depot
                DeadheadTemplate dhtToDepot = adjFull[path[^2].nodeIndex][^1]!.DeadheadTemplate;
                int endTime = taskElements[^1].EndTime;
                taskElements.Add(new VEDeadhead(dhtToDepot, endTime, endTime + dhtToDepot.Duration, vehicleType)
                {
                    StartSoCInTask = taskElements[^1].EndSoCInTask,
                    EndSoCInTask = taskElements[^1].EndSoCInTask - dhtToDepot.Distance * vehicleType.DriveUsage,
                });

                results.Add((minCosts, new VehicleTask(taskElements)
                {
                    vehicleType = vehicleType,
                    Source = $"Labeling {source} -  {validEndIndex}",
                }));
            }

            return results;
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            reset();

            if (model == null || model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            runLabeling();
            List<(double reducedCost, VehicleTask vehicleTask)> primaryTasks = extractTasks("primary", new());
            List<(double reducedCost, VehicleTask vehicleTask)> secondaryTasks = [];

            HashSet<BitArray> alreadyFound = new(new BitArrayComparer());
            foreach (var x in primaryTasks) alreadyFound.Add(x.vehicleTask.ToBitArray(instance.Trips.Count));

            for (int i = 0; i < Math.Min(primaryTasks.Count, Config.VSP_LB_SEC_COL_COUNT); i++)
            {
                var baseTask = primaryTasks[i];

                for (int j = 1; j < (Config.VSP_LB_SEC_COL_ATTEMPTS + 1); j++)
                {
                    reset();
                    for (int k = 0; k < (int)((double)j * baseTask.vehicleTask.Covers.Count / (Config.VSP_LB_SEC_COL_ATTEMPTS + 1)); k++)
                    {
                        blockedNodes[baseTask.vehicleTask.Covers[k]] = true;
                    }
                    runLabeling();
                    var newExtracted = extractTasks("secondary", alreadyFound);
                    secondaryTasks.AddRange(newExtracted);
                    foreach (var x in newExtracted) alreadyFound.Add(x.vehicleTask.ToBitArray(instance.Trips.Count));
                }
            }

            primaryTasks.AddRange(secondaryTasks);
            return primaryTasks;
        }
    }
}
