using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver.SolutionState;

namespace E_VCSP.Solver.ColumnGenerators {
    public class VSPLSSingle : VehicleColumnGen {
        private double T;
        private double alpha;
        private int Q;

        private readonly Random random = new();

        // Always has the form of depot -> dh -> idle -> (trip -> dh -> idle) * n -> depot w/ n >= 0
        // Depot are guaranteed to be before / after all other trips + dh time. 
        private VSPLSNode? head = null;
        private List<int> activeTrips = [];
        private List<int> inactiveTrips = [];
        private LSOperations ops;

        public VSPLSSingle(VehicleSolutionState vss) : base(vss) {
            Reset();
        }

        public void Reset() {
            T = Config.VSP_LS_S_STARTING_T;
            alpha = Config.VSP_LS_S_COOLING_RATE;
            Q = (int)Math.Round(-Config.VSP_LS_S_ITERATIONS / (Math.Log(Config.VSP_LS_S_STARTING_T / Config.VSP_LS_S_ENDING_T) / Math.Log(alpha)));
            ops = new(this, T, "LS Single");
            activeTrips = [];
            inactiveTrips = [.. vss.Instance.Trips.Select(x => x.Index)];

            InitVehicleTask();
        }

        private void InitVehicleTask() {
            VSPArc arc = vss.AdjFull[vss.Instance.DepotStartIndex][vss.Instance.DepotEndIndex] ?? throw new InvalidDataException();
            DeadheadTemplate dht = arc.DeadheadTemplate;

            // Create depot nodes, connecting dh + idle time to ensure that there is always a feasible start / end to the vehicle task
            head = new VSPLSNode() {
                PVE = new PVEDepot(vss.Depot, vss.StartTime - Constants.MIN_NODE_TIME, vss.StartTime),
            };

            head.AddAfter(new PVETravel(dht, vss.StartTime, vss.EndTime, vss.VehicleType))
                .AddAfter(new PVEDepot(vss.Depot, vss.EndTime, vss.EndTime + Constants.MIN_NODE_TIME));
        }

        /// <summary>
        /// Adds random unused trip
        /// </summary>
        private LSOpResult addTrip(VSPLSNode? head) {
            // Select random trip
            int selectIndex = random.Next(inactiveTrips.Count);
            int selectedTrip = inactiveTrips[selectIndex];
            inactiveTrips[selectIndex] = inactiveTrips[^1];
            inactiveTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip trip = vss.Instance.Trips[selectedTrip];

            LSOpResult res = ops.addStop(head, new PVETrip(trip, vss.VehicleType));

            if (res == LSOpResult.Decline || res == LSOpResult.Invalid) return res;

            // Finalize operation
            inactiveTrips.RemoveAt(inactiveTrips.Count - 1);
            activeTrips.Add(selectedTrip);
            return res;
        }

        private LSOpResult removeTrip(VSPLSNode? head) {
            if (activeTrips.Count == 0) return LSOpResult.Invalid;

            // Select random trip for removal
            int selectIndex = random.Next(activeTrips.Count);
            int selectedTrip = activeTrips[selectIndex];
            activeTrips[selectIndex] = activeTrips[^1];
            activeTrips[^1] = selectedTrip; // Order doesn't matter, simply preparing for removal
            Trip t = vss.Instance.Trips[selectedTrip];

            // Find the trip in the current ll
            VSPLSNode? curr = head;
            while (curr != null) {
                if (curr.PVE is PVETrip vet && vet.Trip == t) {
                    break;
                }
                curr = curr.Next;
            }
            if (curr == null) throw new InvalidOperationException("Could not find selected trip for removal");

            LSOpResult res = ops.removeStop(head, curr);

            if (res == LSOpResult.Decline || res == LSOpResult.Invalid) return res;

            // Finalize operation
            activeTrips.RemoveAt(activeTrips.Count - 1);
            inactiveTrips.Add(selectedTrip);
            return res;
        }

        private (double reducedCost, VehicleTask vehicleTask)? finalizeTask() {
            VehicleTask? vehicleTask = head!.ToVehicleTask(this, "LS Single");
            if (vehicleTask == null) return null;
            double reducedCost = vehicleTask.Cost;
            foreach (int coveredTripIndex in vehicleTask.TripIndexCover) {
                reducedCost -= tripDualCosts[coveredTripIndex];
            }
            return (reducedCost, vehicleTask);
        }

        public override List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks() {
            Reset();

            List<(Func<VSPLSNode?, LSOpResult> operation, double chance)> operations = [
                (addTrip, Config.VSP_LS_S_ADD_TRIP),
                (removeTrip, Config.VSP_LS_S_REM_TRIP),
                (ops.addChargeStop, Config.VSP_LS_S_ADD_CHARGE),
                (ops.removeChargeStop, Config.VSP_LS_S_ADD_CHARGE),
            ];
            List<double> sums = [operations[0].chance];
            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            List<(double reducedCost, VehicleTask vehicleTask)> generatedTasks = [];
            int itsPerColumn = (int)Config.VSP_LS_S_ITERATIONS / Config.VSP_LS_S_NUM_COLS;

            int currIts = 0;
            while (currIts < Config.VSP_LS_S_ITERATIONS) {
                currIts++;
                if (currIts % Q == 0) {
                    T *= alpha;
                    ops.T = T;
                }
                if (currIts % itsPerColumn == 0) {
                    var task = finalizeTask();
                    if (task != null) generatedTasks.Add(task.Value);
                }
                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation(head);
            }

            var finalTask = finalizeTask();
            if (finalTask != null) generatedTasks.Add(finalTask.Value);
            return generatedTasks;
        }
    }
}
