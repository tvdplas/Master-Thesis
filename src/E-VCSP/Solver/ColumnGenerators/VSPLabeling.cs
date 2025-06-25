using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    public record SPLabel(
        int prevIndex, // Previously visited node index
        int prevId, // Label preceding this label 
        int chargeLocationIndex, // Charge location used in this node
        bool chargeAtSource, // Was charge at start / end of deadhead
        double currSoC, // SoC at start of node
        double currCosts,  // Costs incurred at start of node
        int labelId
    );

    public class Front
    {
        private SortedList<double, SPLabel> front = [];

        /// <summary>
        /// Inserts label into front
        /// </summary>
        /// <param name="label">Label to be inserted</param>
        /// <returns>The difference in amount of items currently in the front; min = -#items in front + 1, max = 1</returns>
        public int Insert(SPLabel label)
        {
            int l = 0, r = front.Count;
            // find first item that is larger than or equal to label currSoC
            while (l < r)
            {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.currSoC - Config.VSP_LB_CHARGE_EPSILON) r = m;
                else l = m + 1;
            }

            List<int> sameCharge = new();
            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++)
            {
                // Skip if domination is found
                if (front.Values[i].currCosts <= label.currCosts) return 0;

                // If domination is not found however we are within epsilon range of the inserted label, 
                // add it to a seperate list of items we will be removing
                if (Math.Abs(front.Values[i].currSoC - label.currSoC) < Config.VSP_LB_CHARGE_EPSILON) sameCharge.Add(i);
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
                if (label.currCosts <= b.currCosts)
                {
                    front.RemoveAt(i);
                    diff--;
                }
            }

            // Lastly, insert new item. First path should never be taken
            if (front.ContainsKey(label.currSoC)) front[label.currSoC] = label;
            else
            {
                front[label.currSoC] = label;
                diff++;
            }
            return diff;
        }
        public int Count => front.Count;

        public void Clear() => front.Clear();

        public SPLabel Pop()
        {
            int lastIndex = front.Count - 1;
            var label = front.Values[lastIndex];
            front.RemoveAt(lastIndex);
            return label;
        }
    }

    public class VSPLabeling : VehicleShortestPath
    {
        private List<List<SPLabel>> allLabels = [];
        private List<Front> activeLabels = [];
        private List<double> reducedCosts = [];

        public VSPLabeling(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj) : base(model, instance, vehicleType, nodes, adjFull, adj) { }

        private int addLabel(SPLabel spl, int index)
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

            int labelId = 0;
            int activeLabelCount = addLabel(new SPLabel(nodes.Count - 2, -1, -1, false, vehicleType.StartSoC, 0, labelId++), nodes.Count - 2);

            while (activeLabelCount > 0)
            {
                // Get label to expand
                SPLabel? expandingLabel = null;
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
                    Arc arc = adj[expandingLabelIndex][i];
                    int targetIndex = arc.To.Index;
                    Trip? targetTrip = targetIndex < instance.Trips.Count ? instance.Trips[targetIndex] : null;

                    bool depotStart = expandingLabelIndex >= instance.Trips.Count,
                         depotEnd = targetIndex >= instance.Trips.Count;

                    int idleTime = depotStart || depotEnd
                        ? 0
                        : instance.Trips[targetIndex].StartTime - instance.Trips[expandingLabelIndex].EndTime - arc.Deadhead.DeadheadTemplate.Duration;

                    int canCharge = 0;
                    if (idleTime > 0 && !depotStart && instance.Trips[expandingLabelIndex].To.CanCharge) canCharge += 1;
                    if (idleTime > 0 && !depotEnd && instance.Trips[targetIndex].From.CanCharge) canCharge += 2;

                    Location? chargeLocation = null;
                    if (idleTime > 0 && canCharge > 0)
                    {
                        chargeLocation = canCharge > 2 ? instance.Trips[targetIndex].From : instance.Trips[expandingLabelIndex].To;
                    }

                    double SoC = expandingLabel.currSoC;
                    double costs = arc.Deadhead.DeadheadTemplate.Distance * Config.VH_M_COST;
                    if (targetIndex < instance.Trips.Count) costs -= reducedCosts[targetIndex];

                    if (canCharge == 1)
                    {
                        var res = chargeLocation!.ChargingCurves[vehicleType.Index].MaxChargeGained(SoC, idleTime);
                        costs += res.Cost;
                        SoC = Math.Min(SoC + res.SoCGained, vehicleType.MaxSoC);
                    }

                    SoC -= arc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage;
                    if (SoC < vehicleType.MinSoC) continue;

                    if (canCharge >= 2)
                    {
                        var res = chargeLocation!.ChargingCurves[vehicleType.Index].MaxChargeGained(SoC, idleTime);
                        costs += res.Cost;
                        SoC = Math.Min(SoC + res.SoCGained, vehicleType.MaxSoC);
                    }

                    SoC -= (targetTrip?.Distance ?? 0) * vehicleType.DriveUsage;
                    if (SoC < vehicleType.MinSoC) continue;

                    List<(double newSoC, double cost, int chargingIndex)> options = [];
                    // TODO add additional charging detours
                    options.Add((
                        SoC,
                        expandingLabel.currCosts + costs,
                        chargeLocation?.Index ?? -1
                    ));

                    // For each label possibility, check if it is not already dominated at the target node. 
                    foreach (var option in options)
                    {
                        activeLabelCount += addLabel(new SPLabel(
                            expandingLabelIndex,
                            expandingLabel.labelId,
                            option.chargingIndex,
                            canCharge == 1,
                            option.newSoC,
                            option.cost,
                            labelId++
                        ), targetIndex);
                    }
                }
            }


            // Backtrack in order to get path
            // indexes to nodes
            var feasibleEnds = allLabels[^1]
                .Where(x => x.currCosts < 0)
                .OrderBy(x => x.currCosts).ToList();
            var validEnds = feasibleEnds.Take(Config.VSP_LB_MAX_COLS);

            List<(double cost, VehicleTask vehicleTask)> results = [];

            foreach (var validEnd in validEnds)
            {
                List<(int nodeIndex, SPLabel label)> path = [(
                    allLabels.Count - 1,
                    validEnd
                )];
                double minCosts = path[0].label.currCosts;
                while (path[^1].Item2.prevId != -1)
                {
                    var prev = allLabels[path[^1].Item2.prevIndex].Find(x => x.labelId == path[^1].Item2.prevId) ?? throw new Exception("Could not backtrace");
                    path.Add((path[^1].Item2.prevIndex, prev));
                }
                path.Reverse();

                List<VehicleElement> taskElements = [];
                for (int i = 1; i < path.Count - 1; i++)
                {
                    var curr = path[i];
                    var prev = path[i - 1];
                    Trip? trip = instance.Trips[curr.nodeIndex];
                    DeadheadTemplate dht = adjFull[prev.nodeIndex][curr.nodeIndex]!.Deadhead.DeadheadTemplate;

                    int currTime = taskElements.Count > 0
                        ? taskElements[^1].EndTime
                        : trip!.StartTime - adjFull[^2][curr.nodeIndex]!.Deadhead.DeadheadTemplate.Duration;
                    double currSoC = taskElements.Count > 0
                        ? taskElements[^1].EndSoCInTask
                        : vehicleType.StartSoC;

                    // (charge) -> dh -> (charge) -> trip; never charge when going to/from depot

                    int idleTime = trip!.StartTime - dht.Duration - currTime;

                    // Charge before deadhead
                    if (curr.label.chargeLocationIndex != -1 && curr.label.chargeAtSource)
                    {
                        var res = instance.Locations[curr.label.chargeLocationIndex]
                                          .ChargingCurves[vehicleType.Index]
                                          .MaxChargeGained(currSoC, idleTime);

                        taskElements.Add(new VECharge(instance.Locations[curr.label.chargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost)
                        {
                            StartSoCInTask = currSoC,
                            EndSoCInTask = Math.Min(currSoC + res.SoCGained, vehicleType.MaxSoC)
                        });

                        currSoC = Math.Min(currSoC + res.SoCGained, vehicleType.MaxSoC);
                        currTime += idleTime;
                    }

                    // Deadhead
                    taskElements.Add(new VEDeadhead(dht, currTime, currTime + dht.Duration, vehicleType)
                    {
                        StartSoCInTask = currSoC,
                        EndSoCInTask = currSoC - dht.Distance * vehicleType.DriveUsage,
                    });
                    currTime += dht.Duration;
                    currSoC -= dht.Distance * vehicleType.DriveUsage;


                    // Charge after deadhead
                    if (curr.label.chargeLocationIndex != -1 && !curr.label.chargeAtSource)
                    {
                        var res = instance.Locations[curr.label.chargeLocationIndex]
                                          .ChargingCurves[vehicleType.Index]
                                          .MaxChargeGained(currSoC, idleTime);

                        taskElements.Add(new VECharge(instance.Locations[curr.label.chargeLocationIndex], currTime, currTime + idleTime, res.SoCGained, res.Cost)
                        {
                            StartSoCInTask = currSoC,
                            EndSoCInTask = Math.Min(currSoC + res.SoCGained, vehicleType.MaxSoC)
                        });

                        currSoC = Math.Min(currSoC + res.SoCGained, vehicleType.MaxSoC);
                        currTime += idleTime;
                    }

                    if (currTime != trip.StartTime)
                    {
                        // TODO: klopt niet helemaal maar goed
                        taskElements.Add(new VEIdle(trip.From, currTime, trip.StartTime)
                        {
                            StartSoCInTask = currSoC,
                            EndSoCInTask = currSoC
                        });
                    }

                    // add trip
                    if (curr.nodeIndex < instance.Trips.Count)
                    {
                        double tripUsage = vehicleType.DriveUsage * instance.Trips[curr.nodeIndex].Distance;
                        taskElements.Add(new VETrip(instance.Trips[curr.nodeIndex], vehicleType)
                        {
                            StartSoCInTask = currSoC,
                            EndSoCInTask = currSoC - tripUsage,
                        });

                        currTime = instance.Trips[curr.nodeIndex].EndTime;
                        currSoC -= tripUsage;
                    }
                }

                // Add final deadhead to depot
                DeadheadTemplate dhtToDepot = adjFull[path[^2].nodeIndex][^1]!.Deadhead.DeadheadTemplate;
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
