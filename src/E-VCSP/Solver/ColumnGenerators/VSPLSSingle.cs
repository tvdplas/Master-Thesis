using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    public class VSPLSSingle : VehicleShortestPath
    {
        private List<double> reducedCostsTrips = [];
        List<List<DeadheadTemplate?>> locationDHT = [];

        private double T;
        private double alpha;
        private int Q;

        private readonly Random random = new();

        // Always has the form of depot -> dh -> idle -> (trip -> dh -> idle) * n -> depot w/ n >= 0
        // Depot are guaranteed to be before / after all other trips + dh time. 
        private LLNode? head = null;
        private List<int> activeTrips = [];
        private List<int> inactiveTrips = [];

        public VSPLSSingle(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj, List<List<DeadheadTemplate?>> locationDHT)
            : base(model, instance, vehicleType, nodes, adjFull, adj)
        {
            this.locationDHT = locationDHT;
        }

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

            InitVehicleTask();
        }

        private void InitVehicleTask()
        {
            Arc arc = adjFull[instance.DepotStartIndex][instance.DepotEndIndex] ?? throw new InvalidDataException();
            DeadheadTemplate dht = arc.Deadhead.DeadheadTemplate;

            // Create depot nodes, connecting dh + idle time to ensure that there is always a feasible start / end to the vehicle task
            head = new LLNode()
            {
                VE = new PVEDepot(Depot, StartTime - Config.MIN_NODE_TIME, StartTime),
            };

            head.AddAfter(
                    new PVETravel(dht, StartTime, EndTime, vehicleType)
                )
                .AddAfter(
                    new PVEDepot(Depot, EndTime, EndTime + Config.MIN_NODE_TIME)
                );
        }

        /// <summary>
        /// Adds a stop to the task
        /// </summary>
        /// <param name="ve"></param>
        /// <param name="reducedCostsDiff"></param>
        /// <returns></returns>
        private LSOpResult addStop(PartialVehicleElement ve, double reducedCostsDiff)
        {
            if (ve.Type == PVEType.Depot || ve.Type == PVEType.Travel)
                throw new InvalidOperationException("Are you sure?");

            // Find addition targetss
            LLNode? prev = head!.FindLastAfter((node) =>
            {
                PVEType type = node.VE.Type;
                return type != PVEType.Travel && node.VE.EndTime <= ve.StartTime;
            }) ?? head;
            LLNode? next = prev!.FindFirstAfter((node) =>
            {
                PVEType type = node.VE.Type;
                return type != PVEType.Travel;
            }) ?? head;
            if (prev == null || next == null) throw new InvalidDataException("Could not find prev/next");
            if (next != prev!.Next!.Next) throw new InvalidDataException("Vlgm gaat hier nog iets mis");

            // See if travels can be made to connect prev -travel1> ve -travel2> next
            DeadheadTemplate? travel1Template = locationDHT[prev.VE.EndLocation!.Index][ve.StartLocation!.Index];
            DeadheadTemplate? travel2Template = locationDHT[ve.EndLocation!.Index][next.VE.StartLocation!.Index];

            // No travel possible
            if (travel1Template == null || travel2Template == null) return LSOpResult.Invalid;

            // No time for new travels
            if (travel1Template.Duration > ve.StartTime - prev.VE.EndTime || travel2Template.Duration > next.VE.StartTime - ve.EndTime) return LSOpResult.Invalid;

            // New travel is time feasible 
            // Will now check charge / handover time feasibility
            double costDiff = reducedCostsDiff;

            // Costs lost due to previous travel
            LLNode oldTravel = prev.Next!;

            // Old charge / driving costs
            var oldRes = head!.validateTail(vehicleType);
            if (!oldRes.handoversFeasible || !oldRes.chargeFeasible) throw new InvalidOperationException("you fucked up");
            costDiff -= oldRes.drivingCost;
            costDiff -= oldRes.chargingCost;

            // Add new part into mix
            LLNode travel1 = new LLNode()
            {
                VE = new PVETravel(travel1Template, prev.VE.EndTime, ve.StartTime, vehicleType)
            };
            travel1.Prev = prev;
            LLNode stop = travel1.AddAfter(ve);
            LLNode travel2 = stop.AddAfter(new PVETravel(travel2Template, ve.EndTime, next.VE.StartTime, vehicleType));
            travel2.Next = next;

            prev.Next = travel1;
            next.Prev = travel2;
            var newRes = head!.validateTail(vehicleType);
            bool changeFeasible = newRes.handoversFeasible && newRes.chargeFeasible;
            costDiff += newRes.drivingCost;
            costDiff += newRes.chargingCost;

            bool decline = !accept(costDiff);

            if (!changeFeasible || decline)
            {
                // revert
                prev.Next = oldTravel;
                next.Prev = oldTravel;

                return decline ? LSOpResult.Decline : LSOpResult.Invalid;
            }

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
        }

        private LSOpResult removeStop(LLNode node, double reducedCostsDiff)
        {
            if (node.VE.Type == PVEType.Depot || node.VE.Type == PVEType.Travel)
                throw new InvalidOperationException("Are you sure?");

            // Find addition targets
            LLNode prev = node.Prev!.Prev!;
            LLNode next = node.Next!.Next!;

            // See if travels can be made to connect prev -travel1> ve -travel2> next
            DeadheadTemplate? travelTemplate = locationDHT[prev.VE.EndLocation!.Index][next.VE.StartLocation!.Index];

            // No travel possible
            if (travelTemplate == null) return LSOpResult.Invalid;

            // No time for new travel
            if (travelTemplate.Duration > next.VE.StartTime - prev.VE.EndTime) return LSOpResult.Invalid;

            // New travel is time feasible 
            double costDiff = reducedCostsDiff;

            // Previous travels
            LLNode oldTravel1 = prev.Next!;
            LLNode oldTravel2 = next.Prev!;

            // Old charge / driving costs
            var oldRes = head!.validateTail(vehicleType);
            if (!oldRes.handoversFeasible || !oldRes.chargeFeasible) throw new InvalidOperationException("you fucked up");
            costDiff -= oldRes.drivingCost;
            costDiff -= oldRes.chargingCost;

            // Add new part into mix
            LLNode travel = new LLNode()
            {
                VE = new PVETravel(travelTemplate, prev.VE.EndTime, next.VE.StartTime, vehicleType)
            };
            travel.Prev = prev;
            travel.Next = next;

            prev.Next = travel;
            next.Prev = travel;
            var newRes = head!.validateTail(vehicleType);
            bool changeFeasible = newRes.handoversFeasible && newRes.chargeFeasible;
            costDiff += newRes.drivingCost;
            costDiff += newRes.chargingCost;

            bool decline = !accept(costDiff);

            if (!changeFeasible || decline)
            {
                // revert
                prev.Next = oldTravel1;
                next.Prev = oldTravel2;

                return decline ? LSOpResult.Decline : LSOpResult.Invalid;
            }

            return (costDiff < 0) ? LSOpResult.Improvement : LSOpResult.Accept;
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
            Trip trip = instance.Trips[selectedTrip];

            LSOpResult res = addStop(new PVETrip(trip, vehicleType), -reducedCostsTrips[trip.Index]);

            if (res == LSOpResult.Decline || res == LSOpResult.Invalid) return res;

            // Finalize operation
            inactiveTrips.RemoveAt(inactiveTrips.Count - 1);
            activeTrips.Add(selectedTrip);
            return res;
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
                if (curr.VE is PVETrip vet && vet.Trip == t)
                {
                    break;
                }
                curr = curr.Next;
            }
            if (curr == null) throw new InvalidOperationException("Could not find selected trip for removal");

            LSOpResult res = removeStop(curr, reducedCostsTrips[t.Index]);

            if (res == LSOpResult.Decline || res == LSOpResult.Invalid) return res;

            // Finalize operation
            activeTrips.RemoveAt(activeTrips.Count - 1);
            inactiveTrips.Add(selectedTrip);
            return res;
        }

        /// <summary>
        /// adds random charging stop
        /// </summary>
        private LSOpResult addChargeStop()
        {
            // Select random trip
            int selectIndex = random.Next(instance.ChargingLocations.Count);
            Location selectedLocation = instance.ChargingLocations[selectIndex];

            List<(int start, int end)> times = new();
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VE.Type == PVEType.Travel)
                {
                    if (!curr.Prev.VE.EndLocation.CanCharge && !curr.Next.VE.StartLocation.CanCharge)
                    {
                        times.Add((curr.VE.StartTime, curr.VE.EndTime));
                    }
                }
                curr = curr.Next;
            }

            if (times.Count == 0) return LSOpResult.Invalid;

            var timeSlot = times[random.Next(times.Count)];
            var padding = Math.Max(0, (timeSlot.end - timeSlot.start - Config.MIN_NODE_TIME) / 2);

            return addStop(new PVECharge(
                selectedLocation,
                Math.Min(timeSlot.start + padding, timeSlot.end),
                Math.Min(timeSlot.start + padding + Config.MIN_NODE_TIME, timeSlot.end)),
                0
            );
        }

        /// <summary>
        /// Removes random charging stop
        /// </summary>
        /// <returns></returns>
        private LSOpResult removeChargeStop()
        {
            List<LLNode> targets = new();

            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VE.Type == PVEType.ChargeDetour) targets.Add(curr);
                curr = curr.Next;
            }

            if (targets.Count == 0) return LSOpResult.Invalid;

            return removeStop(targets[random.Next(targets.Count)], 0);
        }

        /// <summary>
        /// adds random charging stop
        /// </summary>
        private LSOpResult addHandoverStop()
        {
            // Select random trip
            List<Location> handoverLocations = instance.Locations.Where(x => x.HandoverAllowed).ToList();
            int selectIndex = random.Next(handoverLocations.Count);
            Location selectedLocation = handoverLocations[selectIndex];

            List<(int start, int end)> times = new();
            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VE.Type == PVEType.Travel)
                {
                    if (!curr.Prev.VE.EndLocation.HandoverAllowed && !curr.Next.VE.StartLocation.HandoverAllowed)
                    {
                        times.Add((curr.VE.StartTime, curr.VE.EndTime));
                    }
                }
                curr = curr.Next;
            }

            if (times.Count == 0) return LSOpResult.Invalid;

            var timeSlot = times[random.Next(times.Count)];
            var padding = Math.Max(0, (timeSlot.end - timeSlot.start - selectedLocation.MinHandoverTime) / 2);

            return addStop(new PVEHandover(
                selectedLocation,
                Math.Min(timeSlot.start + padding, timeSlot.end),
                Math.Min(timeSlot.start + padding + selectedLocation.MinHandoverTime, timeSlot.end)),
                0
            );
        }

        /// <summary>
        /// Removes random charging stop
        /// </summary>
        /// <returns></returns>
        private LSOpResult removeHandoverStop()
        {
            List<LLNode> targets = new();

            LLNode? curr = head;
            while (curr != null)
            {
                if (curr.VE.Type == PVEType.ChargeDetour) targets.Add(curr);
                curr = curr.Next;
            }

            if (targets.Count == 0) return LSOpResult.Invalid;

            return removeStop(targets[random.Next(targets.Count)], 0);
        }

        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks()
        {
            Reset();

            List<(Func<LSOpResult> operation, double chance)> operations = [
                (addTrip, Config.VSP_LS_S_ADD_TRIP),
                (removeTrip, Config.VSP_LS_S_REM_TRIP),
                (addChargeStop, Config.VSP_LS_S_ADD_CHARGE),
                (removeChargeStop, Config.VSP_LS_S_ADD_CHARGE),
                (addHandoverStop, Config.VSP_LS_S_ADD_HNDVR),
                (removeHandoverStop, Config.VSP_LS_S_REMOVE_HNDVR),
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
