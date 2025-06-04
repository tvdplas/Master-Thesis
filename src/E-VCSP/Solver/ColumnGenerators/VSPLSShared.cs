using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal enum LSOpResult
    {
        Improvement = 0,
        Accept = 1,
        Decline = 2,
        Invalid = 3,
        Count,
    }

    internal enum LLNodeType
    {
        Depot,
        Deadhead,
        Trip,
        Idle,
    }

    internal class LLNode
    {
        internal required LLNodeType NodeType; // Type of vehicle element in node
        internal required VehicleElement VehicleElement;
        internal LLNode? Prev;
        internal LLNode? Next;
        internal double SoCAtStart;
        internal double SoCAtEnd;
        internal int DebugIndex;
        internal static int DebugIndexCounter;

        public LLNode()
        {
            DebugIndex = DebugIndexCounter++;
        }

        public override string ToString()
        {
            return $"{DebugIndex}: {VehicleElement}";
        }

        internal int TailCount()
        {
            LLNode? curr = this;
            int c = 0;
            while (curr != null)
            {
                c++;
                curr = curr.Next;
            }
            return c;
        }

        internal VehicleTask ToVehicleTask(VehicleType vehicleType)
        {
            this.CalcSoCValues(vehicleType, false);
            List<VehicleElement> elements = [];
            LLNode? curr = this;
            while (curr != null)
            {
                VehicleElement ve = curr.VehicleElement;
                ve.StartSoCInTask = curr.SoCAtStart;
                ve.EndSoCInTask = curr.SoCAtEnd;
                if (ve is VETrip) ve.DrivingCost = 0;
                elements.Add(ve);

                if (curr.Next != null && curr.VehicleElement.EndTime != curr.Next.VehicleElement.StartTime)
                {
                    Console.WriteLine("Invalid; time discontionous");
                }

                if (curr.Next != null && curr.SoCAtEnd != curr.Next.SoCAtStart)
                {
                    Console.WriteLine("Invalid; soc discontinous");
                }

                if (ve.StartSoCInTask > vehicleType.MaxCharge || ve.StartSoCInTask < vehicleType.MinCharge ||
                    ve.EndSoCInTask > vehicleType.MaxCharge || ve.EndSoCInTask < vehicleType.MinCharge
                )
                {
                    Console.WriteLine("Invalid; soc out of vehicle bounds");
                }

                if (ve is VEDeadhead ved && (ved.SoCGained < 0 || ved.ChargeTime < 0))
                {
                    Console.WriteLine("Invalid; soc gained / charge time during deadhead are negative");
                }

                curr = curr.Next;
            }

            if (Config.CONSOLE_LS) Console.WriteLine($"Vehicle task found with {elements.Count} elements");

            return new VehicleTask(elements) { vehicleType = vehicleType };
        }

        /// <summary>
        /// Determine the problems in this and all following nodes in the task.
        /// </summary>
        /// <returns>Array of peaks &gt; maxSoC and valleys &lt; minSoC. Each peak / valley determined by maximum error at a charging location.</returns>
        internal (double chargingCost, List<double> peaks, List<double> valleys) SoCError(VehicleType vehicleType)
        {
            double chargingCost = CalcSoCValues(vehicleType).chargingCosts;
            List<double> peaks = [], valleys = [];
            LLNode? curr = this;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Deadhead)
                {
                    VEDeadhead ved = (VEDeadhead)curr.VehicleElement;
                    // Peaks / valleys may occur at charging stations
                    if (ved.SelectedAction != -1)
                    {
                        ChargingAction ca = ved.Deadhead.ChargingActions[ved.SelectedAction];
                        double SoCBeforeCharge = curr.SoCAtStart - ca.ChargeUsedTo;
                        if (SoCBeforeCharge < vehicleType.MinCharge)
                            valleys.Add(vehicleType.MinCharge - SoCBeforeCharge);

                        ChargingCurve cc = ca.ChargeLocation.ChargingCurves[vehicleType.Index];
                        var chargeResult = cc.MaxChargeGained(SoCBeforeCharge, ved.ChargeTime, true);
                        double SoCAfterCharge = SoCBeforeCharge + chargeResult.SoCGained;

                        if (SoCAfterCharge > vehicleType.MaxCharge)
                            peaks.Add(SoCAfterCharge - vehicleType.MaxCharge);
                    }
                }


                // Valley may also occur at the final depot visit
                if (curr.Next == null)
                {
                    if (curr.SoCAtEnd < vehicleType.MinCharge)
                        valleys.Add(vehicleType.MinCharge - curr.SoCAtEnd);
                }

                curr = curr.Next;
            }

            return (chargingCost, peaks, valleys);
        }

        /// <summary>
        /// Set the SoC values of the rest of the trip. Uses <see cref="SoCAtEnd"/> of <c>this</c> as base for propegation. 
        /// </summary>
        /// <returns>Charge costs over the rest of the trip</returns>
        internal (bool feasible, double chargingCosts) CalcSoCValues(VehicleType vehicleType, bool expand = true)
        {
            LLNode? curr = this;

            double chargeCosts = 0;
            while (curr != null)
            {
                if (!expand && (curr.SoCAtStart > vehicleType.MaxCharge || curr.SoCAtStart < vehicleType.MinCharge))
                {
                    // SoC at start of node is invalid
                    return (false, double.MinValue);
                }


                if (curr.NodeType == LLNodeType.Deadhead)
                {
                    VEDeadhead ved = (VEDeadhead)curr.VehicleElement;

                    if (ved.SelectedAction != -1)
                    {
                        // Recalculate amount of SoC gained as charge gained may depend on charge at start of action
                        ChargingAction ca = ved.Deadhead.ChargingActions[ved.SelectedAction];
                        ChargingCurve cc = ca.ChargeLocation.ChargingCurves[vehicleType.Index];
                        var chargeResult = cc.MaxChargeGained(curr.SoCAtStart - ca.ChargeUsedTo, ved.ChargeTime, expand);
                        ved.ChargeCost = chargeResult.Cost;
                        chargeCosts += chargeResult.Cost;
                        double chargeSoCGained = chargeResult.SoCGained;
                        double SoCDiff = chargeSoCGained - (ca.ChargeUsedFrom + ca.ChargeUsedTo);
                        curr.VehicleElement.SoCDiff = SoCDiff;
                    }
                }

                curr.SoCAtEnd = curr.SoCAtStart + curr.VehicleElement.SoCDiff;

                if (!expand && (curr.SoCAtEnd > vehicleType.MaxCharge || curr.SoCAtEnd < vehicleType.MinCharge))
                {
                    // SoC at end of node is invalid
                    return (false, double.MinValue);
                }

                if (curr.Next != null)
                {
                    curr.Next.SoCAtStart = curr.SoCAtEnd;
                    curr = curr.Next;
                }
                else
                {
                    // Ensure that curr is always depot at end of while
                    break;
                }
            }

            // Add charging costs for recharge at depot node
            chargeCosts += Math.Min(0, vehicleType.StartCharge - curr!.SoCAtEnd) * vehicleType.Capacity / 100 * Config.KWH_COST;

            return (true, chargeCosts);
        }

        /// <summary>
        /// Cost of tail
        /// </summary>
        /// <returns>Overall costs of tail</returns>
        internal double CostOfTail()
        {
            LLNode? curr = this.Next;

            double cost = 0;
            while (curr != null)
            {
                cost += curr.VehicleElement.DrivingCost;

                curr = curr.Next;
            }

            return cost;
        }

        /// <summary>
        /// Removes <paramref name="count"/> nodes after <c>this</c>. Does <b>not</b> update structure of removed nodes.
        /// </summary>
        /// <param name="count">Amount of nodes to remove</param>
        /// <returns>List of removed nodes</returns>
        internal List<LLNode> RemoveAfter(int count)
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

        internal LLNode AddAfter(LLNode node)
        {
            ArgumentNullException.ThrowIfNull(node);

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

        internal LLNode AddAfter(LLNodeType type, VehicleElement ve)
        {
            return AddAfter(new LLNode()
            {
                VehicleElement = ve,
                NodeType = type,
                SoCAtStart = this.SoCAtEnd,
                SoCAtEnd = this.SoCAtEnd + ve.SoCDiff,
            });
        }
    }
}
