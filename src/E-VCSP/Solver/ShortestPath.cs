using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver
{
    internal class LLNode
    {
        internal required VehicleElement VehicleElement;
        internal LLNode? Prev;
        internal LLNode? Next;
        internal double SoCAtStart;
        internal double SoCAtEnd;

        void UpdateSoC(double SoCDelta, bool propegateForward = false)
        {
            SoCAtStart += SoCDelta;
            SoCAtEnd += SoCDelta;
        }

        internal LLNode AddAfter(LLNode node, double SoCDelta)
        {
            if (node == null) throw new ArgumentNullException("Cant explicitely add null node");

            if (this.VehicleElement.EndTime != node.VehicleElement.StartTime) throw new ArgumentException("Vehicle task is not continuous");

            // Set next correctly
            if (this.Next != null)
            {
                this.Next.Prev = node;
                this.Next.UpdateSoC(SoCDelta, true);
            }

            // Update ordering
            node.Next = this.Next;
            node.Prev = this;
            this.Next = node;

            // Set correct SoC in added node
            node.SoCAtStart = this.SoCAtEnd;
            node.SoCAtEnd = this.SoCAtEnd + SoCDelta;
            return node;
        }

        internal LLNode AddAfter(VehicleElement ve, double SoCDelta)
        {
            return AddAfter(new LLNode() { VehicleElement = ve }, SoCDelta);
        }
    }

    internal class ShortestPath
    {
        private Instance instance;
        private GRBModel model;

        private double T;
        private double alpha;
        private int Q;
        private List<List<Arc?>> adjFull = new();
        private VehicleType vehicleType;

        private Random random;

        private List<int> activeTrips;
        private List<int> inactiveTrips;
        private LLNode? head = null;

        public ShortestPath(Instance instance, GRBModel model, List<List<Arc?>> adjFull)
        {
            this.instance = instance;
            this.model = model;
            this.adjFull = adjFull;

            vehicleType = instance.VehicleTypes[0];
            double T = Config.LS_STARTING_T;
            double alpha = Config.LS_COOLING_RATE;
            double Q = -Config.LS_ITERATIONS / (Math.Log(Config.LS_STARTING_T / Config.LS_ENDING_T) / Math.Log(alpha));

            Location? depot = instance.Locations.Find(x => x.IsDepot);
            Arc? depotDepotArc = adjFull[^2][^1];
            if (depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
            if (depotDepotArc == null) throw new InvalidDataException("No depot to depot arc found when constructing depot vehicleelements in LS ShortestPath");

            // Find starting and ending times for instance
            int startTime = int.MaxValue, endTime = int.MinValue;
            foreach (Trip t in instance.Trips)
            {
                // Depot to trip
                int minDepTime = t.StartTime - adjFull[^2][t.Index]!.Deadhead.DeadheadTemplate.Duration;
                startTime = Math.Min(minDepTime, startTime);

                // Trip to depot
                int maxArrTime = t.StartTime - adjFull[t.Index][^1]!.Deadhead.DeadheadTemplate.Duration;
                endTime = Math.Min(maxArrTime, startTime);
            }

            // Create depot nodes, connecting dh + idle time to ensure that there is always a feasible start / end to the vehicle task
            // Depot start
            LLNode depotStart = new LLNode()
            {
                VehicleElement = new VEDepot()
                {
                    StartTime = startTime - Config.DUMMY_NODE_TIME,
                    EndTime = startTime,
                    Location = depot,
                },
                SoCAtStart = vehicleType.StartCharge,
                SoCAtEnd = vehicleType.StartCharge,
            };

            // Depot to depot deadhead
            LLNode depotDepotDH = depotStart.AddAfter(new VEDeadhead()
            {
                Deadhead = depotDepotArc.Deadhead,
                StartTime = startTime,
                EndTime = startTime + depotDepotArc.Deadhead.DeadheadTemplate.Duration,
            }, -depotDepotArc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage);

            // Idle at ending depot
            LLNode idle = depotDepotDH.AddAfter(new VEIdle() { StartTime = depotDepotDH.VehicleElement.EndTime, EndTime = endTime }, 0);

            // Depot end; 
            LLNode depotEnd = idle.AddAfter(new VEDepot() { StartTime = endTime, EndTime = endTime + Config.DUMMY_NODE_TIME, Location = depot }, 0);

            head = depotStart;

            activeTrips = new();
            inactiveTrips = instance.Trips.Select(x => x.Index).ToList();
            this.adjFull = adjFull;
        }

        public void Reset()
        {
            head = null;
            activeTrips = new();
            inactiveTrips = instance.Trips.Select(x => x.Index).ToList();
        }

        internal (double minCosts, VehicleTask vehicleTask) getVehicleTaskLS()
        {
            // LS Setup
            ;
            Random random = new();
            LLNode? head = null;

            List<int> activeTrips = new(), inActiveTrips = instance.Trips.Select(trip => trip.Index).ToList();

            void addTrip()
            {
                // Get new trip, set to active
                int selectIndex = random.Next(inActiveTrips.Count);
                int selectedTrip = inActiveTrips[selectIndex];
                inActiveTrips[selectIndex] = inActiveTrips[^1];
                inActiveTrips.RemoveAt(inActiveTrips.Count - 1);
                activeTrips.Add(selectedTrip);
                Trip t = instance.Trips[selectedTrip];

                if (head == null)
                {
                    // seperate case.
                }



                // Find trip preceding the new trip.
                // If no such trip exists, all other trips are later and we must start from the list head.
                LLNode? curr = head;
                Trip? precTrip = null;
                while (curr != null)
                {
                    if (curr.VehicleElement is VETrip currt && curr.VehicleElement.EndTime <= t.StartTime)
                    {
                        // Found trip that precedes our current trip; break;
                        precTrip = currt.Trip;
                        curr = curr.Next; // Skip to next deadhead for next phase
                        break;
                    }
                    curr = curr.Next;
                }
                if (precTrip == null)
                {
                    // No preceding trip; therefore it must come before all others
                    curr = head;
                }

                // Find the star

                // We will now attempt to link the trip into the current setup; no regard for SoC as we simply penalize.
                // 

            }

            List<(Action, double)> actions = new();

            actions.Add((addTrip, 1.0));


            bool accept(double deltaScore)
            {
                return Math.Exp(-deltaScore / T) > random.NextDouble();
            }



            accept(1);

            int currIts = 0;
            while (currIts < Config.LS_STARTING_T)
            {
                if (currIts % Q == 0)
                {
                    T *= alpha;
                    if (Config.CONSOLE_LS) Console.WriteLine("Decreased T to " + T);
                }
            }
        }

    }
}
