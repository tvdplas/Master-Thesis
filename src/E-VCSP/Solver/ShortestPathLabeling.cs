using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver
{
    internal record SPLabel(int prevIndex, int prevId, double currSoC, double costs, int id);

    internal class Front
    {
        private SortedList<double, SPLabel> front = [];

        internal void Insert(SPLabel label)
        {
            int l = 0, r = front.Count;
            // find first item that is larger than or equal to label currSoC
            while (l < r)
            {
                int m = (l + r) / 2;
                if (front.Keys[m] >= label.currSoC) r = m;
                else l = m + 1;
            }

            // Linear scan over remaining items, stop insertion once an item is found which dominates
            for (int i = l; i < front.Count; i++)
            {
                // Skip once domination is found
                if (front.Values[i].costs <= label.costs) return;
            }

            // Label is inserted, all dominated items need to be removed
            for (int i = l - 1; i >= 0 && i < front.Count; i--)
            {
                var b = front.Values[i];
                if (label.costs <= b.costs)
                {
                    front.RemoveAt(i);
                }
            }
            front[label.currSoC] = label;
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

    internal class ShortestPathLabeling
    {
        internal required GRBModel model;
        internal required Instance instance;

        internal required VehicleType vt;
        internal required List<EVSPNode> nodes = [];
        internal required List<List<Arc?>> adjFull = [];
        internal required List<List<Arc>> adj = [];

        private List<List<SPLabel>> allLabels = [];
        private List<Front> activeLabels = [];

        private (double minCosts, VehicleTask vehicleTask) getVehicleTaskLabeling()
        {

            // Initialize labels
            allLabels = [];
            activeLabels = [];
            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels.Add([]);
                activeLabels.Add(new());
            }

            if (Config.CONSOLE_LABELING) Console.WriteLine("Starting label correction");
            if (model == null || model.Status == GRB.Status.LOADED || model.Status == GRB.Status.INFEASIBLE)
                throw new InvalidOperationException("Can't find shortest path if model is in infeasible state");

            List<double> reducedCosts = [];
            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCosts.Add(constrs[i].Pi);
            }

            for (int i = 0; i < nodes.Count; i++)
            {
                allLabels[i].Clear();
                activeLabels[i].Clear();
            }

            int labelId = 0;
            void addLabel(SPLabel spl, int index)
            {
                allLabels[index].Add(spl);
                activeLabels[index].Insert(spl);
            }


            addLabel(new SPLabel(nodes.Count - 2, -1, vt.StartCharge, 0, labelId++), nodes.Count - 2);

            while (activeLabels.Sum(x => x.Count) > 0)
            {
                // Get label to expand
                SPLabel? l = null;
                int currIndex = -1;
                for (int i = 0; i < activeLabels.Count; i++) if (activeLabels[i].Count > 0)
                    {
                        l = activeLabels[i].Pop();
                        currIndex = i;
                        break;
                    }

                // Try to expand label
                for (int i = 0; i < adj[currIndex].Count; i++)
                {
                    Arc arc = adj[currIndex][i];
                    int targetIndex = arc.To.Index;

                    List<(double newSoC, double cost)> options = [];
                    int tripDistance = currIndex < instance.Trips.Count - 2 ? instance.Trips[currIndex].Distance : 0;
                    double tripCost = currIndex < instance.Trips.Count - 2 ? reducedCosts[currIndex] : 0;
                    double directTravelSoC = l!.currSoC -
                        ((tripDistance + arc.Deadhead.DeadheadTemplate.Distance) * vt.DriveUsage);
                    if (directTravelSoC >= vt.MinCharge)
                        options.Add((directTravelSoC, l.costs - tripCost + (arc.Deadhead.DeadheadTemplate.Distance * Config.M_COST)));

                    // TODO: dit moet VEEL efficienter
                    foreach (var chargeAction in arc.Deadhead.ChargingActions)
                    {
                        double chargeAtStation = l.currSoC - (tripDistance * vt.DriveUsage) - chargeAction.ChargeUsedTo;
                        if (chargeAtStation < vt.MinCharge) continue; // Not feasible

                        ChargingCurve cc = chargeAction.ChargeLocation.ChargingCurves[vt.Index];
                        var maxCharge = cc.MaxChargeGained(chargeAtStation, chargeAction.TimeAtLocation, false);

                        // Try some charging actions. 
                        double chargeRange = maxCharge.SoCGained;
                        for (int j = 0; j <= Config.DISCRETE_FACTOR; j++)
                        {
                            var res = cc.ChargeCosts(chargeAtStation, Math.Min(chargeAtStation + (j * chargeRange / Config.DISCRETE_FACTOR), 100.0), false);
                            double SoCAtNextTrip = chargeAtStation + res.SoCGained - chargeAction.ChargeUsedFrom;
                            if (SoCAtNextTrip < vt.MinCharge) continue;
                            options.Add((SoCAtNextTrip, l.costs - tripCost + chargeAction.DrivingCost + res.Cost));
                        }
                    }

                    // For each label possibility, check if it is not already dominated at the target node. 
                    foreach (var option in options)
                    {
                        addLabel(new SPLabel(currIndex, l.id, option.newSoC, option.cost, labelId++), targetIndex);
                    }
                }
            }

            if (Config.CONSOLE_LABELING) Console.WriteLine($"Total of {labelId} labels considered");

            // Backtrack in order to get path
            // indexes to nodes
            List<(int, SPLabel)> path = [(allLabels.Count - 1, allLabels[^1].MinBy(x => x.costs))];
            double minCosts = allLabels[^1].Min(x => x.costs);
            while (path[^1].Item2.prevId != -1)
            {
                var prev = allLabels[path[^1].Item2.prevIndex].Find(x => x.id == path[^1].Item2.prevId) ?? throw new Exception("waar is mn pad gebleven");
                path.Add((path[^1].Item2.prevIndex, prev));
            }

            path.Reverse();

            List<VehicleElement> taskElements = [];
            for (int i = 0; i < path.Count - 1; i++)
            {
                int index = path[i].Item1;
                int nextIndex = path[i + 1].Item1;
                var node = nodes[index];
                if (index < instance.Trips.Count)
                {
                    taskElements.Add(new VETrip()
                    {
                        Trip = instance.Trips[index],
                        EndTime = instance.Trips[index].EndTime,
                        StartTime = instance.Trips[index].StartTime,
                        DrivingCost = 0,
                        SoCDiff = vt.DriveUsage * instance.Trips[index].Duration,
                    });
                }

                int startTime, endTime;
                Deadhead dh = adjFull[index][nextIndex]?.Deadhead ?? throw new InvalidDataException("Huh waar is mn deadhead");

                if (index == instance.Trips.Count)
                {
                    startTime = instance.Trips[nextIndex].StartTime - dh.DeadheadTemplate.Duration;
                    endTime = instance.Trips[nextIndex].StartTime;
                }
                else if (index == instance.Trips.Count + 1)
                {
                    endTime = instance.Trips[index].EndTime;
                    startTime = instance.Trips[index].EndTime + dh.DeadheadTemplate.Duration;
                }
                else
                {
                    startTime = instance.Trips[index].StartTime;
                    endTime = instance.Trips[index].EndTime;
                }
                taskElements.Add(new VEDeadhead()
                {
                    Deadhead = dh,
                    StartTime = startTime,
                    EndTime = endTime,
                    DrivingCost = dh.BaseDrivingCost,
                    SoCDiff = -1, //TODO
                });
            }

            if (Config.CONSOLE_LABELING) Console.WriteLine($"Generated column with reduced cost of {minCosts}");

            return (minCosts, new VehicleTask(taskElements));
        }
    }
}
