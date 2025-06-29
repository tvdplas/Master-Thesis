using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    public class VSPLabel
    {
        public static int ID_COUNTER = 0;
        public int Id;
        public int PrevNodeIndex; // Previously visited node index
        public int PrevId; // Label preceding this label 
        public int ChargeLocationIndex = -1; // Charge location used in this node
        public int SteeringTime = 0;
        public bool ChargeAtSource = false; // Was charge at start / end of deadhead
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

    public class VSPLabeling : VehicleShortestPath
    {
        private List<List<VSPLabel>> allLabels = [];
        private List<VSPFront> activeLabels = [];
        private List<double> reducedCosts = [];

        public VSPLabeling(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<VSPArc?>> adjFull, List<List<VSPArc>> adj) : base(model, instance, vehicleType, nodes, adjFull, adj) { }

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

            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels.Add([]);
                activeLabels.Add(new());
            }

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCosts.Add(constrs[i].Pi);
            }
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            reset();

            if (Config.CONSOLE_LABELING) Console.WriteLine("Starting label correction");
            if (model == null || model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            int activeLabelCount = addLabel(new VSPLabel()
            {
                PrevId = -1,
                PrevNodeIndex = nodes.Count - 2,
                CurrCosts = 0,
                CurrSoC = vehicleType.StartSoC,
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
                    int targetIndex = arc.To.Index;
                    Trip? targetTrip = targetIndex < instance.Trips.Count ? instance.Trips[targetIndex] : null;

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
                    if (SoC < vehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME) continue;

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

                    if (canCharge == 0 && targetTrip != null && !targetTrip.From.FreeIdle)
                    {
                        // Idle is after adjust SoC
                        SoC -= idleTime * vehicleType.IdleUsage;
                        if (SoC < vehicleType.MinSoC) continue;
                    }

                    SoC -= (targetTrip?.Distance ?? 0) * vehicleType.DriveUsage;
                    steeringTime += targetTrip?.Duration ?? 0;
                    if (SoC < vehicleType.MinSoC || steeringTime >= Config.MAX_STEERING_TIME) continue;

                    List<(double newSoC, double cost, int chargingIndex)> options = [];
                    // TODO add additional charging detours
                    options.Add((
                        SoC,
                        expandingLabel.CurrCosts + costs,
                        chargeLocation?.Index ?? -1
                    ));

                    // For each label possibility, check if it is not already dominated at the target node. 
                    foreach (var option in options)
                    {
                        activeLabelCount += addLabel(new VSPLabel()
                        {
                            PrevId = expandingLabel.Id,
                            PrevNodeIndex = expandingLabelIndex,
                            ChargeAtSource = canCharge == 1,
                            ChargeLocationIndex = option.chargingIndex,
                            CurrCosts = option.cost,
                            CurrSoC = option.newSoC,
                            SteeringTime = steeringTime,
                        }, targetIndex);
                    }
                }
            }


            // Backtrack in order to get path
            // indexes to nodes
            var feasibleEnds = allLabels[^1]
                .Where(x => x.CurrCosts < 0)
                .OrderBy(x => x.CurrCosts).ToList();
            var validEnds = feasibleEnds.Take(Config.VSP_LB_MAX_COLS);

            List<(double cost, VehicleTask vehicleTask)> results = [];

            foreach (var validEnd in validEnds)
            {
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
                    DeadheadTemplate dht = adjFull[prev.nodeIndex][curr.nodeIndex]!.DeadheadTemplate;

                    int currTime = taskElements.Count > 0
                        ? taskElements[^1].EndTime
                        : trip!.StartTime - adjFull[^2][curr.nodeIndex]!.DeadheadTemplate.Duration;
                    double CurrSoC = taskElements.Count > 0
                        ? taskElements[^1].EndSoCInTask
                        : vehicleType.StartSoC;

                    // (charge) -> dh -> (charge) -> trip; never charge when going to/from depot

                    int idleTime = trip!.StartTime - dht.Duration - currTime;

                    // Charge before deadhead
                    if (curr.label.ChargeLocationIndex != -1 && curr.label.ChargeAtSource)
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
                    if (curr.label.ChargeLocationIndex != -1 && !curr.label.ChargeAtSource)
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

                results.Add((minCosts, new VehicleTask(taskElements) { vehicleType = vehicleType }));
            }

            return results;
        }
    }
}
