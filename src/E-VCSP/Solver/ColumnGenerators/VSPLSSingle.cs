using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class VSPLSSingle : VehicleShortestPath
    {
        private List<double> reducedCostsTrips = [];

        private double T;
        private double alpha;
        private int Q;

        private readonly Random random = new();

        // Always has the form of depot -> dh -> idle -> (trip -> dh -> idle) * n -> depot w/ n >= 0
        // Depot are guaranteed to be before / after all other trips + dh time. 
        private LLNode? head = null;
        private List<int> activeTrips = [];
        private List<int> inactiveTrips = [];

        private double VSP_LS_S_OVERCHARGE_PENALTY_FIX;
        private double VSP_LS_S_OVERCHARGE_PENALTY_VAR;
        private double VSP_LS_S_UNDERCHARGE_PENALTY_FIX;
        private double VSP_LS_S_UNDERCHARGE_PENALTY_VAR;

        public VSPLSSingle(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj)
            : base(model, instance, vehicleType, nodes, adjFull, adj) { }

        public void Reset()
        {
            vehicleType = instance.VehicleTypes[0];
            T = Config.VSP_LS_S_STARTING_T;
            alpha = Config.VSP_LS_S_COOLING_RATE;
            Q = (int)Math.Round(-Config.VSP_LS_S_ITERATIONS / (Math.Log(Config.VSP_LS_S_STARTING_T / Config.VSP_LS_S_ENDING_T) / Math.Log(alpha)));
            activeTrips = [];
            inactiveTrips = [.. instance.Trips.Select(x => x.Index)];
            reducedCostsTrips = [];

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCostsTrips.Add(constrs[i].Pi);
            }

            // Local copies of penalties as they might be adjusted in order to correct for charge errors
            VSP_LS_S_OVERCHARGE_PENALTY_FIX = Config.VSP_LS_S_OVERCHARGE_PENALTY_FIX;
            VSP_LS_S_OVERCHARGE_PENALTY_VAR = Config.VSP_LS_S_OVERCHARGE_PENALTY_VAR;
            VSP_LS_S_UNDERCHARGE_PENALTY_FIX = Config.VSP_LS_S_UNDERCHARGE_PENALTY_FIX;
            VSP_LS_S_UNDERCHARGE_PENALTY_VAR = Config.VSP_LS_S_UNDERCHARGE_PENALTY_VAR;

            InitVehicleTask();
        }

        private LLNode InitVehicleTask()
        {
            Arc? depotDepotArc = adjFull[^2][^1];
            if (Depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
            if (depotDepotArc == null) throw new InvalidDataException("No depot to depot arc found when constructing depot vehicleelements in LS ShortestPath");

            // Create depot nodes, connecting dh + idle time to ensure that there is always a feasible start / end to the vehicle task
            LLNode depotStart = new()
            {
                NodeType = LLNodeType.Depot,
                VehicleElement = new VEDepot(Depot, StartTime - Config.MIN_NODE_TIME, StartTime),
                SoCAtStart = vehicleType.StartCharge,
                SoCAtEnd = vehicleType.StartCharge,
            };
            LLNode depotDepotDH = depotStart.AddAfter(LLNodeType.Deadhead, new VEDeadhead(depotDepotArc.Deadhead, StartTime, vehicleType));
            LLNode idle = depotDepotDH.AddAfter(LLNodeType.Idle, new VEIdle(Depot, depotDepotDH.VehicleElement.EndTime, EndTime, vehicleType));
            LLNode depotEnd = idle.AddAfter(LLNodeType.Depot, new VEDepot(Depot, EndTime, EndTime + Config.MIN_NODE_TIME));

            head = depotStart;
            head.CalcSoCValues(vehicleType);
            return head;
        }


        /// <summary>
        /// Adds random unused trip
        /// </summary>
        private LSOpResult addTrip()
        {
            // Select random trip
            int selectIndex = random.Next(inactiveTrips.Count);
            int selectedTrip = inactiveTrips[selectIndex];
            inactiveTrips[selectIndex] = inactiveTrips[^1];
            inactiveTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip t = instance.Trips[selectedTrip];

            // Find VE (trip / depot) that precedes selected trip
            LLNode? curr = head, prev = head;
            while (curr != null)
            {
                if ((curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip) && curr.VehicleElement.EndTime <= t.StartTime)
                    prev = curr;
                curr = curr.Next;
            }
            if (prev == null) throw new Exception("No previous trip/depot found for addition target");

            // Find next target
            curr = prev.Next;
            LLNode? next = null;
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Depot || curr.NodeType == LLNodeType.Trip)
                {
                    next = curr;
                    break;
                }
                curr = curr.Next;
            }
            if (next == null) throw new Exception("No next trip / depot found for addition target");

            // Check driving time feasibility between prev -> t -> next
            int prevAdjIndex = prev.VehicleElement is VETrip prevt ? prevt.Trip.Index : instance.DepotStartIndex;
            int nextAdjIndex = next.VehicleElement is VETrip nextt ? nextt.Trip.Index : instance.DepotEndIndex;
            Arc? prevToTrip = adjFull[prevAdjIndex][t.Index];
            Arc? tripToNext = adjFull[t.Index][nextAdjIndex];
            if (prevToTrip == null || tripToNext == null)
            {
                // Time feasibility is in here already; results in null deadhead
                return LSOpResult.Invalid;
            }


            // Check SAA feasibiltiy
            double costDiff = 0;

            // Deadhead + idle removal.
            VEDeadhead dhToReplace = (VEDeadhead)prev.Next!.VehicleElement;
            VEIdle idleToReplace = (VEIdle)prev.Next.Next!.VehicleElement;
            costDiff -= dhToReplace!.DrivingCost;
            costDiff -= idleToReplace!.DrivingCost;

            // Added deadheads 
            costDiff += prevToTrip.Deadhead.BaseDrivingCost;
            costDiff += tripToNext.Deadhead.BaseDrivingCost;

            // idle times
            int prevToTripIdle = t.StartTime - (prev.VehicleElement.EndTime + prevToTrip.Deadhead.DeadheadTemplate.Duration);
            int tripToNextIdle = next.VehicleElement.StartTime - (t.EndTime + tripToNext.Deadhead.DeadheadTemplate.Duration);
            costDiff += prevToTripIdle * Config.IDLE_COST;
            costDiff += tripToNextIdle * Config.IDLE_COST;

            // reduced cost of trip
            costDiff -= reducedCostsTrips[t.Index];

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = prev!.SoCError(vehicleType);
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * VSP_LS_S_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * VSP_LS_S_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;

            // Place elements in current task so SoC effects can be determined
            // old situation: prev -> dh -> idle -> next
            var removedNodes = prev.RemoveAfter(2);
            LLNode dh = removedNodes[0];
            LLNode idle = removedNodes[1];

            // Situation after remove: prev -> next
            // Target situation: prev -> dh1 -> idle1 -> t -> dh2 -> idle2 -> next
            LLNode dh1 = prev.AddAfter(LLNodeType.Deadhead, new VEDeadhead(prevToTrip.Deadhead, prev.VehicleElement.EndTime, vehicleType));
            LLNode idle1 = dh1.AddAfter(LLNodeType.Idle, new VEIdle(t.From, dh1.VehicleElement.EndTime, t.StartTime, vehicleType));
            LLNode trip = idle1.AddAfter(LLNodeType.Trip, new VETrip(t, vehicleType));
            LLNode dh2 = trip.AddAfter(LLNodeType.Deadhead, new VEDeadhead(tripToNext.Deadhead, t.EndTime, vehicleType));
            LLNode idle2 = dh2.AddAfter(LLNodeType.Idle, new VEIdle(tripToNext.Deadhead.DeadheadTemplate.To, dh2.VehicleElement.EndTime, next.VehicleElement.StartTime, vehicleType));

            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = prev!.SoCError(vehicleType);
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * VSP_LS_S_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * VSP_LS_S_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;

            // TODO: reduced cost of block 

            if (!accept(costDiff))
            {
                // revert changes in list; can simply be done by connecting prev back to dh and next to idle and resetting soc. 
                prev.Next = dh;
                next.Prev = idle;
                prev.CalcSoCValues(vehicleType);
                return LSOpResult.Decline;
            }

            // Finalize operation
            inactiveTrips.RemoveAt(inactiveTrips.Count - 1);
            activeTrips.Add(selectedTrip);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult removeTrip()
        {
            if (activeTrips.Count == 0) return LSOpResult.Invalid;

            // Select random trip for removal
            int selectIndex = random.Next(activeTrips.Count);
            int selectedTrip = activeTrips[selectIndex];
            activeTrips[selectIndex] = activeTrips[^1];
            activeTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip t = instance.Trips[selectedTrip];

            // Find the trip in the current ll
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VehicleElement is VETrip vet && vet.Trip == t)
                {
                    break;
                }
                curr = curr.Next;
            }
            if (curr == null) throw new InvalidOperationException("Could not find selected trip for removal");

            // Check if removal is time-feasible; should always be the case. 
            // Pre-removal: prev -> dh1 -> idle1 -> t -> dh2 -> idle2 -> next
            // Post-removal: prev -> dh -> idle -> next
            LLNode? prev = curr.Prev?.Prev?.Prev;
            LLNode? next = curr.Next?.Next?.Next;
            if (prev == null || next == null) throw new InvalidOperationException("Something went wrong");
            int prevIndex = prev!.VehicleElement is VETrip prevvet ? prevvet.Trip.Index : instance.DepotStartIndex;
            int nextIndex = next!.VehicleElement is VETrip nextvet ? nextvet.Trip.Index : instance.DepotEndIndex;

            Arc? arc = adjFull[prevIndex][nextIndex];
            if (arc == null)
            {
                return LSOpResult.Invalid;
            }

            // Check LS accept
            double costDiff = 0;

            // Old drive
            costDiff -= curr.Prev!.Prev!.VehicleElement.DrivingCost;
            costDiff -= curr.Prev.VehicleElement.DrivingCost;
            costDiff -= curr.VehicleElement.DrivingCost;
            costDiff -= curr.Next!.VehicleElement.DrivingCost;
            costDiff -= curr.Next!.Next!.VehicleElement.DrivingCost;

            // New drive
            costDiff += arc.Deadhead.BaseDrivingCost;
            int idleTime = next.VehicleElement.StartTime - (prev.VehicleElement.EndTime + arc.Deadhead.DeadheadTemplate.Duration);
            costDiff += idleTime * Config.IDLE_COST;

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = prev!.SoCError(vehicleType);
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * Config.VSP_LS_S_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * Config.VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * Config.VSP_LS_S_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * Config.VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;

            // perform op to get charge costs
            // Situation before: prev -> dh1 -> idle1 -> trip -> dh2 -> idle2 -> next
            var removedNodes = prev.RemoveAfter(5);
            LLNode dh1 = removedNodes[0];
            LLNode idle2 = removedNodes[4];
            // Situation after: prev -> next
            LLNode dh = prev.AddAfter(LLNodeType.Deadhead, new VEDeadhead(arc.Deadhead, prev.VehicleElement.EndTime, vehicleType));
            LLNode idle = dh.AddAfter(LLNodeType.Idle, new VEIdle(arc.Deadhead.DeadheadTemplate.To, dh.VehicleElement.EndTime, next.VehicleElement.StartTime, vehicleType));
            // Situation after: prev -> dh -> idle -> next


            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = prev!.SoCError(vehicleType);
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * VSP_LS_S_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * VSP_LS_S_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;


            if (!accept(costDiff))
            {
                // revert op
                prev.Next = dh1;
                next.Prev = idle2;
                prev.CalcSoCValues(vehicleType);

                return LSOpResult.Decline;
            }

            // Finalize operation
            activeTrips.RemoveAt(activeTrips.Count - 1);
            inactiveTrips.Add(selectedTrip);

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult changeChargeAction()
        {
            // List all trips that currently have charging actions available
            LLNode? curr = head;
            List<LLNode> chargeTargets = [];
            while (curr != null)
            {
                if (curr.NodeType == LLNodeType.Deadhead && ((VEDeadhead)curr.VehicleElement).Deadhead.ChargingActions.Count > 0)
                {
                    chargeTargets.Add(curr);
                }
                curr = curr.Next;
            }

            if (chargeTargets.Count == 0) return LSOpResult.Invalid;

            LLNode target = chargeTargets[random.Next(chargeTargets.Count)];
            VEDeadhead dh = (VEDeadhead)target.VehicleElement;

            // Chose new action
            int currAction = dh.SelectedAction;
            int newAction = currAction;
            while (newAction == currAction)
                newAction = random.Next(-1, dh.Deadhead.ChargingActions.Count);
            ChargingAction? action = newAction == -1 ? null : dh.Deadhead.ChargingActions[newAction];

            // Determine new charge
            int currChargeTime = dh.ChargeTime;
            int newChargeTime = action == null ? 0 : action.TimeAtLocation;
            int newTotalTime = action == null
                ? dh.Deadhead.DeadheadTemplate.Duration
                : action.DrivingTimeTo + action.DrivingTimeFrom + newChargeTime;

            double newChargeGained = 0;
            double newDrivingCost = dh.Deadhead.BaseDrivingCost;
            double currSoCDiff = dh.SoCDiff;
            double newSoCDiff = -dh.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage;
            if (action != null)
            {
                double SoCAtCharger = target.SoCAtStart - action.ChargeUsedTo;
                newChargeGained = action.ChargeLocation.ChargingCurves[vehicleType.Index]
                    .MaxChargeGained(SoCAtCharger, newChargeTime, true).SoCGained;
                newDrivingCost = action.DrivingCost;
                newSoCDiff = newChargeGained - (action.ChargeUsedFrom + action.ChargeUsedTo);
            }

            double costDiff = 0;

            costDiff -= dh.DrivingCost;
            costDiff += newDrivingCost;

            // Old SoC related costs
            (double currChargingCost, var currPeaks, var currValleys) = target.Prev!.SoCError(vehicleType);
            costDiff -= currChargingCost;
            costDiff -= (currPeaks.Count * VSP_LS_S_OVERCHARGE_PENALTY_FIX + currPeaks.Sum() * VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff -= (currValleys.Count * VSP_LS_S_UNDERCHARGE_PENALTY_FIX + currValleys.Sum() * VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;

            // perform op
            dh.SelectedAction = newAction;
            dh.ChargeTime = newChargeTime;
            dh.SoCDiff = newSoCDiff;

            // New SoC related costs
            (double newChargingCost, var newPeaks, var newValleys) = target.Prev!.SoCError(vehicleType);
            costDiff += newChargingCost;
            costDiff += (newPeaks.Count * VSP_LS_S_OVERCHARGE_PENALTY_FIX + newPeaks.Sum() * VSP_LS_S_OVERCHARGE_PENALTY_VAR) / T;
            costDiff += (newValleys.Count * VSP_LS_S_UNDERCHARGE_PENALTY_FIX + newValleys.Sum() * VSP_LS_S_UNDERCHARGE_PENALTY_VAR) / T;

            // TODO: idle costs moeten eigenlijk ook meegenomen worden aangezien de dh tijd verandert

            if (!accept(costDiff))
            {
                // Revert operation
                dh.SelectedAction = currAction;
                dh.ChargeTime = currChargeTime;
                dh.SoCDiff = currSoCDiff;
                target.Prev.CalcSoCValues(vehicleType);

                return LSOpResult.Decline;
            }

            // Finalize operation with everything not needed for SoC calculations
            dh.DrivingCost = newDrivingCost;
            dh.SoCGained = newChargeGained;
            dh.EndTime = dh.StartTime + newTotalTime;
            target.Next!.VehicleElement.StartTime = dh.EndTime;

            return costDiff < 0 ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        internal override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            Reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (addTrip, Config.VSP_LS_S_ADD_TRIP),
                (removeTrip, Config.VSP_LS_S_REM_TRIP),
                (changeChargeAction, Config.VSP_LS_S_CHANGE_CHARGE),
            ];
            List<(string name, List<int> counts)> results = [
                ("addTrip", [.. new int[(int)LSOpResult.Count]]),
                ("removeTrip", [.. new int[(int)LSOpResult.Count]]),
                ("changeChargeAction", [.. new int[(int)LSOpResult.Count]]),
                ("increaseChargeTime", [.. new int[(int)LSOpResult.Count]]),
                ("decreaseChargeTime", [.. new int[(int)LSOpResult.Count]]),
            ];
            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;
            while (currIts < Config.VSP_LS_S_ITERATIONS)
            {
                currIts++;
                if (currIts % Q == 0) T *= alpha;

                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
                results[operationIndex].counts[(int)res]++;
            }


            // Force SoC error to be resolved
            // Only use charge updating as to preserve existing trip structure
            List<(Func<LSOpResult> operation, double chance)> operationsCorrection = [
                (changeChargeAction, Config.VSP_LS_S_CHANGE_CHARGE),
            ];
            List<double> sumsCorrection = [operationsCorrection[0].chance];
            for (int i = 1; i < operationsCorrection.Count; i++)
                sumsCorrection.Add(sumsCorrection[i - 1] + operationsCorrection[i].chance);

            double currOvercharge = VSP_LS_S_OVERCHARGE_PENALTY_FIX;
            double currUndercharge = VSP_LS_S_UNDERCHARGE_PENALTY_FIX;
            VSP_LS_S_OVERCHARGE_PENALTY_FIX = 1_000_000_000_000;
            VSP_LS_S_UNDERCHARGE_PENALTY_FIX = 1_000_000_000_000;
            currIts = 0;
            var socError = head!.SoCError(vehicleType);
            while (currIts < Config.VSP_LS_S_ITERATIONS && (socError.peaks.Count > 0 || socError.valleys.Count > 0))
            {
                double r = random.NextDouble() * sumsCorrection[^1];
                int operationIndex = sumsCorrection.FindIndex(x => r <= x);
                var res = operationsCorrection[operationIndex].operation();

                currIts++;
                socError = head.SoCError(vehicleType);
            }
            VSP_LS_S_OVERCHARGE_PENALTY_FIX = currOvercharge;
            VSP_LS_S_UNDERCHARGE_PENALTY_FIX = currUndercharge;
            if (socError.peaks.Count > 0 || socError.valleys.Count > 0)
            {
                return [];
            }

            if (Config.CONSOLE_LS)
            {
                foreach (var (name, counts) in results)
                {
                    Console.WriteLine($"Operation: {name} (total: {counts.Sum()})");
                    for (int i = 0; i < counts.Count; i++)
                    {
                        Console.WriteLine((LSOpResult)i + ": " + counts[i]);
                    }
                }
            }


            VehicleTask vehicleTask = head!.ToVehicleTask(vehicleType);
            double reducedCost = vehicleTask.Cost;
            foreach (int coveredTripIndex in vehicleTask.Covers)
            {
                reducedCost -= reducedCostsTrips[coveredTripIndex];
            }
            return [(reducedCost, vehicleTask)];
        }
    }
}
