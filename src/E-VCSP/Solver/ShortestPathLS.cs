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

            if (propegateForward && Next != null) Next.UpdateSoC(SoCDelta, true);
        }

        internal LLNode? RemoveAfter()
        {
            if (Next == null) throw new InvalidOperationException("Can't remove with no next");

            // Find SoC diff in next to propegate
            double diff = -(Next.SoCAtEnd - Next.SoCAtStart);

            LLNode? newNext = Next.Next;
            Next = newNext;
            if (newNext != null)
            {
                newNext.Prev = this;
                newNext.UpdateSoC(diff, true);
            }

            return Next;
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

        public override string ToString()
        {
            LLNode? curr = this;
            int prev = 0, next = 0;
            while (curr != null) { prev++; curr = curr.Prev; }
            curr = this;
            while (curr != null) { prev++; curr = curr.Next; }

            return $"{VehicleElement} (<{prev}, >{next})";
        }
    }

    internal enum LSOpResult
    {
        Improvement = 0,
        Accept = 1,
        Decline = 2,
        Invalid = 3,
        Count,
    }

    internal class ShortestPathLS
    {
        private Instance instance;
        private GRBModel model;
        private List<double> reducedCostsTrips;

        private double T;
        private double alpha;
        private int Q;
        private List<List<Arc?>> adjFull = new();
        private VehicleType? vehicleType = null;

        private Random random = new();

        // Always has the form of depot -> dh -> idle -> (trip -> dh -> idle) * n -> depot w/ n >= 0
        // Depot are guaranteed to be before / after all other trips + dh time. 
        private LLNode? head = null;
        private List<int> activeTrips = new();
        private List<int> inactiveTrips = new();
        private double costs;


        private LLNode InitVehicleTask()
        {
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
                int maxArrTime = t.EndTime + adjFull[t.Index][^1]!.Deadhead.DeadheadTemplate.Duration;
                endTime = Math.Max(maxArrTime, endTime);
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
                    Cost = 0,
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
                Cost = depotDepotArc.Deadhead.BaseCost,
            }, -depotDepotArc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage);

            // Idle at ending depot
            int idleTime = endTime - depotDepotDH.VehicleElement.EndTime;
            LLNode idle = depotDepotDH.AddAfter(new VEIdle()
            {
                StartTime = depotDepotDH.VehicleElement.EndTime,
                EndTime = endTime,
                Cost = idleTime * Config.IDLE_COST
            }, 0);

            // Depot end; 
            LLNode depotEnd = idle.AddAfter(new VEDepot()
            {
                StartTime = endTime,
                EndTime = endTime + Config.DUMMY_NODE_TIME,
                Location = depot,
                Cost = 0
            }, 0);

            head = depotStart;
            return head;
        }

        public ShortestPathLS(Instance instance, GRBModel model, List<List<Arc?>> adjFull)
        {
            this.instance = instance;
            this.model = model;
            this.adjFull = adjFull;
            Reset();
        }

        public void Reset()
        {
            costs = 0;
            vehicleType = instance.VehicleTypes[0];
            T = Config.LS_STARTING_T;
            alpha = Config.LS_COOLING_RATE;
            Q = (int)Math.Round(-Config.LS_ITERATIONS / (Math.Log(Config.LS_STARTING_T / Config.LS_ENDING_T) / Math.Log(alpha)));
            activeTrips = new();
            inactiveTrips = instance.Trips.Select(x => x.Index).ToList();
            reducedCostsTrips = new();

            GRBConstr[] constrs = model.GetConstrs();
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                reducedCostsTrips.Add(constrs[i].Pi);
            }

            InitVehicleTask();
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
            LLNode? curr = head, prec = head;
            while (curr != null)
            {
                if ((curr.VehicleElement is VEDepot || curr.VehicleElement is VETrip) && curr.VehicleElement.EndTime <= t.StartTime)
                    prec = curr;
                curr = curr.Next;
            }
            if (prec == null) throw new Exception("HUH");

            // Find next target
            curr = prec.Next;
            LLNode? next = null;
            while (curr != null)
            {
                if (curr.VehicleElement is VEDepot || curr.VehicleElement is VETrip)
                {
                    next = curr;
                    break;
                }
                curr = curr.Next;
            }
            if (next == null) throw new Exception("HUH2");

            // Check driving time feasibility between prec -> t -> next
            int precAdjIndex = prec.VehicleElement is VETrip prect ? prect.Trip.Index : instance.DepotStartIndex;
            int nextAdjIndex = next.VehicleElement is VETrip nextt ? nextt.Trip.Index : instance.DepotEndIndex;

            Arc? precToTrip = adjFull[precAdjIndex][t.Index];
            Arc? tripToNext = adjFull[t.Index][nextAdjIndex];
            if (precToTrip == null || tripToNext == null)
            {
                // Time feasibility is in here already
                return LSOpResult.Invalid;
                //throw new InvalidDataException("No deadhead");
            }

            int precToTripIdle = t.StartTime - (prec.VehicleElement.EndTime + precToTrip.Deadhead.DeadheadTemplate.Duration);
            int tripToNextIdle = next.VehicleElement.StartTime - (t.EndTime + tripToNext.Deadhead.DeadheadTemplate.Duration);

            // Check if deadheads are time-feasible
            if (precToTripIdle < 0 || tripToNextIdle < 0)
            {
                return LSOpResult.Invalid; // no retries
            }

            // Check SAA feasibiltiy
            double costDiff = 0;
            // Deadhead + idle removal.
            VEDeadhead dhToReplace = prec.Next.VehicleElement as VEDeadhead;
            VEIdle idleToReplace = prec.Next.Next.VehicleElement as VEIdle;
            costDiff -= dhToReplace!.Cost;
            costDiff -= idleToReplace!.Cost;

            // Added deadheads / idle times
            costDiff += precToTrip.Deadhead.BaseCost;
            costDiff += tripToNext.Deadhead.BaseCost;
            // TODO: idle time moet echt wel kosten hebben ofzo
            costDiff += precToTripIdle * Config.IDLE_COST;
            costDiff += tripToNextIdle * Config.IDLE_COST;

            // reduced cost of trip
            costDiff -= reducedCostsTrips[t.Index];

            // TODO: reduced cost of block

            if (!accept(costDiff))
            {
                return LSOpResult.Decline;
            }

            // Perform actual operation 
            // Update active trip arrays
            inactiveTrips.RemoveAt(inactiveTrips.Count - 1);
            activeTrips.Add(selectedTrip);

            // old situation: prec -> dh -> idle -> next
            prec.Next.RemoveAfter(); // idle
            prec.RemoveAfter(); // dh

            // Situation after remove: prec -> next
            // Target situation: prec -> dh1 -> idle1 -> t -> dh2 -> idle2 -> next
            LLNode dh1 = prec.AddAfter(new VEDeadhead()
            {
                Cost = precToTrip.Deadhead.BaseCost,
                Deadhead = precToTrip.Deadhead,
                StartTime = prec.VehicleElement.EndTime,
                EndTime = prec.VehicleElement.EndTime + precToTrip.Deadhead.DeadheadTemplate.Duration,
            }, precToTrip.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage);
            LLNode idle1 = dh1.AddAfter(new VEIdle()
            {
                Cost = 0,
                StartTime = dh1.VehicleElement.EndTime,
                EndTime = t.StartTime,
            }, 0);
            LLNode trip = idle1.AddAfter(new VETrip()
            {
                Cost = reducedCostsTrips[t.Index],
                StartTime = t.StartTime,
                EndTime = t.EndTime,
                Trip = t,
            }, t.Distance * vehicleType!.DriveUsage);
            LLNode dh2 = trip.AddAfter(new VEDeadhead()
            {
                Cost = tripToNext.Deadhead.BaseCost,
                Deadhead = tripToNext.Deadhead,
                StartTime = t.EndTime,
                EndTime = t.EndTime + tripToNext.Deadhead.DeadheadTemplate.Duration,
            }, tripToNext.Deadhead.DeadheadTemplate.Distance * vehicleType!.DriveUsage);
            LLNode idle2 = dh2.AddAfter(new VEIdle()
            {
                Cost = 0,
                StartTime = dh2.VehicleElement.EndTime,
                EndTime = next.VehicleElement.StartTime,
            }, 0);

            if (idle2.Next != next) throw new InvalidOperationException("Something went wrong with insertion");

            if (costDiff < 0) return LSOpResult.Improvement;
            return LSOpResult.Accept;
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
            LLNode prev = curr.Prev.Prev.Prev;
            LLNode next = curr.Next.Next.Next;
            int prevIndex = prev.VehicleElement is VETrip prevvet ? prevvet.Trip.Index : instance.DepotStartIndex;
            int nextIndex = next.VehicleElement is VETrip nextvet ? nextvet.Trip.Index : instance.DepotEndIndex;

            Arc? arc = adjFull[prevIndex][nextIndex];
            if (arc == null)
            {
                return LSOpResult.Invalid;
                throw new InvalidDataException($"Could not find deadhead between {prevIndex} and {nextIndex}");
            }

            int idleTime = next.VehicleElement.StartTime - (prev.VehicleElement.EndTime + arc.Deadhead.DeadheadTemplate.Duration);
            if (idleTime < 0)
            {
                Console.WriteLine("(?) Removal operation invalid due to time constraints. Might be incorrect");
                return LSOpResult.Invalid;
            }

            // Check LS accept
            double scoreDiff = 0;
            scoreDiff -= curr.Prev.Prev.VehicleElement.Cost;
            scoreDiff -= curr.Prev.VehicleElement.Cost;
            scoreDiff -= curr.VehicleElement.Cost;
            scoreDiff -= curr.Next.VehicleElement.Cost;
            scoreDiff -= curr.Next.Next.VehicleElement.Cost;
            scoreDiff += arc.Deadhead.BaseCost;
            scoreDiff += idleTime * Config.IDLE_COST;

            if (!accept(scoreDiff))
            {
                return LSOpResult.Decline;
            }

            // Actually perform removal
            activeTrips.RemoveAt(activeTrips.Count - 1);
            inactiveTrips.Add(selectedTrip);

            for (int i = 0; i < 5; i++) prev.RemoveAfter();
            if (prev.Next != next) throw new InvalidOperationException("Invalid removals");

            LLNode dh = prev.AddAfter(new VEDeadhead()
            {
                Cost = arc.Deadhead.BaseCost,
                StartTime = prev.VehicleElement.EndTime,
                EndTime = prev.VehicleElement.EndTime + arc.Deadhead.DeadheadTemplate.Duration,
                Deadhead = arc.Deadhead,
            }, arc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage);

            LLNode idle = dh.AddAfter(new VEIdle()
            {
                Cost = idleTime * Config.IDLE_COST,
                StartTime = dh.VehicleElement.EndTime,
                EndTime = next.VehicleElement.StartTime,
            }, 0);
            if (idle.Next != next) throw new InvalidOperationException("Invalid addition of dh/idle");

            if (scoreDiff < 0) return LSOpResult.Improvement;
            return LSOpResult.Accept;
        }

        private bool accept(double deltaScore)
        {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }

        internal VehicleTask getVehicleTaskLS()
        {
            List<(Func<LSOpResult> operation, double chance)> operations = [
                (addTrip, 1.0),
                (removeTrip, 0.05),
            ];
            List<(string name, List<int> counts)> results = [
                ("addTrip", [.. new int[(int)LSOpResult.Count]]),
                ("removeTrip", [.. new int[(int)LSOpResult.Count]]),
            ];
            List<double> sums = [operations[0].chance];

            for (int i = 1; i < operations.Count; i++) sums.Add(sums[i - 1] + operations[i].chance);

            int currIts = 0;
            while (currIts < Config.LS_ITERATIONS)
            {
                currIts++;
                if (currIts % Q == 0) T *= alpha;

                double r = random.NextDouble() * sums[^1];
                int operationIndex = sums.FindIndex(x => r <= x);
                var res = operations[operationIndex].operation();
                results[operationIndex].counts[(int)res]++;
            }

            if (Config.CONSOLE_LS)
            {
                foreach (var result in results)
                {
                    Console.WriteLine($"Operation: {result.name} (total: {result.counts.Sum()})");
                    for (int i = 0; i < result.counts.Count; i++)
                    {
                        Console.WriteLine((LSOpResult)i + ": " + result.counts[i]);
                    }
                }
            }

            LLNode? curr = head;
            List<VehicleElement> elements = new();
            while (curr != null)
            {
                VehicleElement ve = curr.VehicleElement;
                if (ve is VETrip) ve.Cost = 0;
                elements.Add(ve);
                curr = curr.Next;
            }

            if (Config.CONSOLE_LS) Console.WriteLine($"Vehicle task found with {elements.Count} elements");

            return new VehicleTask(elements);
        }

    }
}
