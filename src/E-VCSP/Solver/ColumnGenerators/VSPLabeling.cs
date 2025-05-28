using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal record SPLabel(
        int prevIndex, // Previously visited node index
        int prevId, // Label preceding this label 
        int prevChargeAction, // Charge action done at this node
        double currSoC, // SoC at start of node
        double currCosts,  // Costs incurred at start of node
        int labelId
    );

    internal class Front
    {
        private SortedList<double, SPLabel> front = [];

        /// <summary>
        /// Inserts label into front
        /// </summary>
        /// <param name="label">Label to be inserted</param>
        /// <returns>The difference in amount of items currently in the front; min = -#items in front + 1, max = 1</returns>
        internal int Insert(SPLabel label)
        {
            int l = 0, r = front.Count;
            // find first item that is larger than or equal to label currSoC
            while (l < r)
            {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.currSoC - Config.CHARGE_EPSILON) r = m;
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
                if (Math.Abs(front.Values[i].currSoC - label.currSoC) < Config.CHARGE_EPSILON) sameCharge.Add(i);
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
        internal int Count => front.Count;

        internal void Clear() => front.Clear();

        internal SPLabel Pop()
        {
            int lastIndex = front.Count - 1;
            var label = front.Values[lastIndex];
            front.RemoveAt(lastIndex);
            return label;
        }
    }

    internal class VSPLabeling : VehicleShortestPath
    {
        private List<List<SPLabel>> allLabels = [];
        private List<Front> activeLabels = [];
        private List<double> reducedCosts = [];

        internal VSPLabeling(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj) : base(model, instance, vehicleType, nodes, adjFull, adj) { }

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

        internal override (double reducedCost, VehicleTask vehicleTask)? GenerateVehicleTask()
        {
            reset();

            if (Config.CONSOLE_LABELING) Console.WriteLine("Starting label correction");
            if (model == null || model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            int labelId = 0;
            int activeLabelCount = addLabel(new SPLabel(nodes.Count - 2, -1, -1, vehicleType.StartCharge, 0, labelId++), nodes.Count - 2);

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

                    int tripDistance = expandingLabelIndex < instance.Trips.Count - 2 ? instance.Trips[expandingLabelIndex].Distance : 0;
                    double tripCost = expandingLabelIndex < instance.Trips.Count - 2 ? reducedCosts[expandingLabelIndex] : 0;
                    List<(double newSoC, double cost, int chargingIndex)> options = [];

                    if (arc.Deadhead.ChargingActions.Count == 0)
                    {
                        // Add label for driving deadhead directly
                        double directTravelSoC = expandingLabel.currSoC -
                            (tripDistance + arc.Deadhead.DeadheadTemplate.Distance) * vehicleType.DriveUsage;
                        if (directTravelSoC >= vehicleType.MinCharge)
                        {
                            options.Add((
                                directTravelSoC,
                                expandingLabel.currCosts - tripCost + arc.Deadhead.DeadheadTemplate.Distance * Config.M_COST,
                                -1
                            ));
                        }
                    }
                    else
                    {
                        // Add labels for each of the possible charging actions between the expanding trip and target trip
                        for (int j = 0; j < arc.Deadhead.ChargingActions.Count; j++)
                        {
                            var chargeAction = arc.Deadhead.ChargingActions[j];
                            double chargeAtStation = expandingLabel.currSoC - (tripDistance * vehicleType.DriveUsage + chargeAction.ChargeUsedTo);
                            if (chargeAtStation < vehicleType.MinCharge) continue; // Not feasible, not enough soc to reach charger

                            ChargingCurve cc = chargeAction.ChargeLocation.ChargingCurves[vehicleType.Index];
                            var maxCharge = cc.MaxChargeGained(chargeAtStation, chargeAction.TimeAtLocation, false);
                            double SoCAtNextTrip = chargeAtStation + maxCharge.SoCGained - chargeAction.ChargeUsedFrom;
                            if (SoCAtNextTrip < vehicleType.MinCharge) continue; // Not feasible, soc at next trip is too low

                            options.Add((
                                SoCAtNextTrip,
                                expandingLabel.currCosts - tripCost + chargeAction.DrivingCost + maxCharge.Cost,
                                j
                            ));
                        }
                    }

                    // For each label possibility, check if it is not already dominated at the target node. 
                    foreach (var option in options)
                    {
                        activeLabelCount += addLabel(new SPLabel(
                            expandingLabelIndex,
                            expandingLabel.labelId,
                            option.chargingIndex,
                            option.newSoC,
                            option.cost,
                            labelId++
                        ), targetIndex);
                    }
                }
            }


            // Backtrack in order to get path
            // indexes to nodes
            List<(int nodeIndex, SPLabel label)> path = [(allLabels.Count - 1, allLabels[^1].MinBy(x => x.currCosts) ?? throw new InvalidDataException("Could not find path to EOD depot"))];
            double minCosts = allLabels[^1].Min(x => x.currCosts);
            while (path[^1].Item2.prevId != -1)
            {
                var prev = allLabels[path[^1].Item2.prevIndex].Find(x => x.labelId == path[^1].Item2.prevId) ?? throw new Exception("Could not backtrace");
                path.Add((path[^1].Item2.prevIndex, prev));
            }

            path.Reverse();



            List<VehicleElement> taskElements = [
                new VEDepot(Depot, StartTime - Config.MIN_NODE_TIME, StartTime) {
                    StartSoCInTask = vehicleType.StartCharge,
                    EndSoCInTask = vehicleType.StartCharge,
                }
            ];
            for (int i = 0; i < path.Count - 1; i++)
            {
                int index = path[i].nodeIndex;
                int nextIndex = path[i + 1].nodeIndex;
                var node = nodes[index];

                int currTime = taskElements[^1].EndTime;
                double currSoC = taskElements[^1].EndSoCInTask ?? throw new InvalidDataException("Did not properly set soc in previous element");


                if (index < instance.Trips.Count)
                {
                    double tripUsage = vehicleType.DriveUsage * instance.Trips[index].Distance;
                    taskElements.Add(new VETrip(instance.Trips[index], vehicleType)
                    {
                        StartSoCInTask = currSoC,
                        EndSoCInTask = currSoC - tripUsage,
                    });


                    currTime = instance.Trips[index].EndTime;
                    currSoC -= tripUsage;
                }

                Deadhead dh = adjFull[index][nextIndex]?.Deadhead ?? throw new InvalidDataException("No deadhead corresponding to driven path found");
                int chargeActionIndex = path[i + 1].label.prevChargeAction;
                if (chargeActionIndex != -1)
                {
                    var chargeAction = dh.ChargingActions[chargeActionIndex];
                    double chargeAtStation = currSoC - chargeAction.ChargeUsedTo;

                    // Amount charged is always maximal
                    ChargingCurve cc = chargeAction.ChargeLocation.ChargingCurves[vehicleType.Index];
                    var maxCharge = cc.MaxChargeGained(chargeAtStation, chargeAction.TimeAtLocation, false);
                    double SoCAtNextTrip = chargeAtStation + maxCharge.SoCGained - chargeAction.ChargeUsedFrom;
                    int totalDuration = (int)maxCharge.TimeUsed + chargeAction.DrivingTimeTo + chargeAction.DrivingTimeFrom;


                    taskElements.Add(new VEDeadhead()
                    {
                        Deadhead = dh,
                        StartTime = currTime,
                        EndTime = currTime + totalDuration,
                        DrivingCost = chargeAction.DrivingCost,
                        ChargeCost = maxCharge.Cost,
                        ChargeGained = maxCharge.SoCGained,
                        ChargeTime = (int)maxCharge.TimeUsed,
                        EndSoCInTask = SoCAtNextTrip,
                        SelectedAction = chargeActionIndex,
                        StartSoCInTask = currSoC,
                        SoCDiff = SoCAtNextTrip - currSoC,
                    });

                    currTime += totalDuration;
                    currSoC = SoCAtNextTrip;
                }
                else
                {
                    double SoCDiff = -(dh.DeadheadTemplate.Distance * vehicleType.DriveUsage);
                    taskElements.Add(new VEDeadhead()
                    {
                        Deadhead = dh,
                        StartTime = currTime,
                        EndTime = currTime + dh.DeadheadTemplate.Duration,
                        DrivingCost = dh.BaseDrivingCost,
                        SelectedAction = -1,
                        StartSoCInTask = currSoC,
                        EndSoCInTask = currSoC + SoCDiff,
                        SoCDiff = SoCDiff,
                    });

                    currTime += dh.DeadheadTemplate.Duration;
                    currSoC += SoCDiff;
                }

                int endTime = nextIndex == instance.DepotEndIndex ? EndTime : instance.Trips[nextIndex].StartTime;
                Location location = nextIndex == instance.DepotEndIndex ? Depot : instance.Trips[nextIndex].From;
                VEIdle vei = new VEIdle(location, currTime, endTime);
                vei.StartSoCInTask = currSoC;
                vei.EndSoCInTask = currSoC + vei.SoCDiff;
                taskElements.Add(vei);
            }

            taskElements.Add(new VEDepot(Depot, EndTime, EndTime + Config.MIN_NODE_TIME));

            if (Config.CONSOLE_LABELING) Console.WriteLine($"Generated column with reduced cost of {minCosts}");

            return (minCosts, new VehicleTask(taskElements) { vehicleType = vehicleType });
        }
    }
}
