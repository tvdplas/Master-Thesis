using Microsoft.Msagl.Drawing;
using Microsoft.Msagl.Layout.Layered;

namespace E_VCSP.Objects.Discrete
{
    public class DGraph
    {
        public DDepot DDepotStart;
        public DDepot DDepotEnd;
        public List<List<DTrip>> DTrips = new();
        public List<DDeadhead> DDeadheads = new();

        private double floorToDiscreteValue(double inp) => Math.Floor(inp / (100.0 / Config.DISCRETE_FACTOR)) * (100.0 / Config.DISCRETE_FACTOR);

        /// <summary>
        /// Create a discrete graph from an instance with current Discrete factor dictated by <see cref="Config.DISCRETE_FACTOR"/>
        /// </summary>
        /// <param name="instance">Instance to be transformed</param>
        /// <exception cref="InvalidDataException">Instance contains invalid data</exception>
        public DGraph(Instance instance)
        {
            // TODO: beter kiezen
            VehicleType vh = instance.VehicleTypes[0];

            // Create all discrete trips
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                DTrips.Add(new(Config.DISCRETE_FACTOR));
                for (int j = 0; j <= Config.DISCRETE_FACTOR; j++)
                {
                    double SoC = Math.Floor(j * 100.0 / Config.DISCRETE_FACTOR);

                    // Dont even add nodes whose charge is infeasible
                    if (SoC < vh.MinSoC) continue;

                    DTrips[i].Add(new DTrip()
                    {
                        Id = $"{instance.Trips[i].Id}@{SoC}%",
                        Trip = instance.Trips[i],
                        StartingSoC = SoC,
                    });
                }
            }

            foreach ((var dl1, var dl2) in DTrips.SelectMany(x => DTrips.Select(y => (x, y))))
            {
                foreach ((DTrip dt1, DTrip dt2) in dl1.SelectMany(x => dl2.Select(y => (x, y))))
                {
                    // Check each pair of discrete nodes
                    if (dt1.Trip.EndTime > dt2.Trip.StartTime) continue;
                    //Console.WriteLine($"dt1: {dt1.Trip.Id}, dt2: {dt2.Trip.Id}");

                    // Early return when the initial trip can't even be driven at the starting SoC.
                    double SoCusedInDT1 = dt1.Trip.Distance * vh.DriveUsage;
                    if (SoCusedInDT1 > dt1.StartingSoC) continue;

                    // Two possibilities: direct drive from dt1 to dt2 or drive via charging station. Both also need to be time-feasible
                    // First: check if direct drive is timefeasible
                    // If it's not, trips via a charging station aren't feasible either (or deadhead times are incorrect, but we assume that they are consistent).

                    // TODO: use deadheads instead of deadhead templates
                    DeadheadTemplate? dht_direct = instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == dt2.Trip.From);
                    if (dht_direct != null && dt1.Trip.EndTime + dht_direct.Duration <= dt2.Trip.StartTime)
                    {
                        // Check if charge is compatible between the two trips
                        double SoC = dt1.StartingSoC - SoCusedInDT1;
                        SoC -= dht_direct.Distance * vh.DriveUsage;
                        if (SoC > vh.MinSoC)
                        {
                            // May be compatible; Note that SoC < 0 may still be possible via indirect trip, therefore no early termination. 
                            SoC = floorToDiscreteValue(SoC);
                            if (SoC == dt2.StartingSoC)
                            {
                                // Determine costs of deadhead; only driving time is used
                                // TODO: idle wordt helemaal niet meegenomen
                                double drivingCost = dht_direct.Distance * Config.M_COST;
                                DDeadheads.Add(new()
                                {
                                    From = dt1,
                                    To = dt2,
                                    Costs = drivingCost,
                                    ChargeGained = 0,
                                    ChargeUsed = dt1.StartingSoC - SoC,
                                    DrivingTimes = [dht_direct.Duration],
                                    IdleTime = dt2.Trip.StartTime - (dt1.Trip.EndTime + dht_direct.Duration),
                                });
                                continue;
                            }
                        }
                    }

                    // Iterate over all charging locations, see whether we can find a valid match in here.
                    // TODO: misschien handig om over alles heen te itereren / duplicate deadheads to te voegen 
                    // om te checken of het goedkoper kan dmv idle time/charging
                    for (int i = 0; i < instance.ChargingLocations.Count; i++)
                    {
                        Location chargeLoc = instance.ChargingLocations[i];
                        // Check if detour via charging location is time-feasible.
                        DeadheadTemplate? dht_toCharge = instance.DeadheadTemplates.Find(dht => dht.From == dt1.Trip.To && dht.To == chargeLoc);
                        DeadheadTemplate? dht_fromCharge = instance.DeadheadTemplates.Find(dht => dht.From == chargeLoc && dht.To == dt2.Trip.From);

                        // No deadhead found / not feasible 
                        if (dht_toCharge == null || dht_fromCharge == null) continue;
                        int idleTime = dt2.Trip.StartTime - dt1.Trip.EndTime - dht_toCharge.Duration - dht_fromCharge.Duration;
                        if (idleTime < 0) continue;

                        // Deadhead feasible; see how much charge can be gained.
                        double SoCAtCharge = dt1.StartingSoC - SoCusedInDT1 - (dht_toCharge.Distance * vh.DriveUsage);
                        if (SoCAtCharge < vh.MinSoC) continue;

                        ChargingCurve cc = chargeLoc.ChargingCurves[vh.Index];
                        ChargeResult cr = cc.MaxChargeGained(SoCAtCharge, idleTime);

                        // Now, check if the charge is actually compatible; In order to do so, we need to check if we can use some partial charge such that our target SOC is reached.
                        double SoCDiff = dt2.StartingSoC - (SoCAtCharge - (dht_fromCharge.Distance * vh.DriveUsage));
                        double alpha = SoCDiff / cr.SoCGained;
                        if (alpha >= 0 && alpha <= 1)
                        {
                            // A feasible charge schedule was found; we now rerun the charge sequence in order to get costs
                            ChargeResult crUsed = cc.ChargeCosts(SoCAtCharge, SoCAtCharge + SoCDiff);

                            if (crUsed.TimeUsed < Config.MIN_CHARGE_TIME) continue;

                            // Add deadhead that takes into account 2x driving costs and charging costs
                            // TODO: add time-dependent prices, staat nu gewoon vast

                            double drivingCosts = Config.M_COST * (dht_toCharge.Distance + dht_fromCharge.Distance);
                            int idleTimeAfterCharge = idleTime - (int)Math.Ceiling(crUsed.TimeUsed);
                            DDeadheads.Add(new()
                            {
                                From = dt1,
                                To = dt2,
                                ChargeGained = crUsed.SoCGained,
                                ChargeUsed = SoCDiff,
                                ChargingTime = (int)Math.Ceiling(crUsed.TimeUsed),
                                Costs = drivingCosts + crUsed.Cost,
                                DrivingTimes = [dht_toCharge.Duration, dht_fromCharge.Duration],
                                IdleTime = idleTimeAfterCharge
                            });
                            continue;
                        }
                    }
                }
            }

            Location? depot = instance.Locations.Find(loc => loc.IsDepot);
            if (depot == null) throw new InvalidDataException("At least one location should be flagged as a depot.");
            DDepotStart = new()
            {
                Id = "start-" + depot.Id,
                Location = depot,
            };
            DDepotEnd = new()
            {
                Id = "end-" + depot.Id,
                Location = depot,
            };

            // From depot to trip
            // TODO: depot to trip and vice versa can also do charging actions
            foreach (DTrip dt in DTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = instance.DeadheadTemplates.Find(dh => dh.From == depot && dh.To == dt.Trip.From);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtTrip = vh.StartSoC - (dh.Distance * vh.DriveUsage);
                double discretizedSoC = floorToDiscreteValue(SoCAtTrip);
                if (discretizedSoC == dt.StartingSoC)
                {
                    double drivingCosts = dh.Distance * Config.M_COST;
                    double pullOutCosts = Config.PULLOUT_COST;
                    DDeadheads.Add(new()
                    {
                        From = DDepotStart,
                        To = dt,
                        Costs = drivingCosts + pullOutCosts,
                        ChargeGained = 0,
                        ChargeUsed = vh.StartSoC - SoCAtTrip,
                        ChargingTime = 0,
                        DrivingTimes = [dh.Duration],
                        IdleTime = 0,
                    });
                }
            }

            // From trip to depot
            foreach (DTrip dt in DTrips.SelectMany(dts => dts.Select(x => x)))
            {
                // Check if there is a deadhead from the depot to the starting location of the trip
                DeadheadTemplate? dh = instance.DeadheadTemplates.Find(dh => dh.From == dt.Trip.To && dh.To == depot);
                if (dh == null) throw new InvalidDataException("Can't go from the deadhead to a trip; This might be wrong");

                double SoCAtDepot = dt.StartingSoC - ((dt.Trip.Distance + dh.Distance) * vh.DriveUsage);
                if (SoCAtDepot >= 0)
                {
                    double drivingCosts = dh.Distance * Config.M_COST;
                    DDeadheads.Add(new()
                    {
                        From = dt,
                        To = DDepotEnd,
                        Costs = drivingCosts,
                        ChargeGained = 0,
                        ChargeUsed = dt.StartingSoC - (dt.Trip.Distance * vh.DriveUsage) - SoCAtDepot,
                        ChargingTime = 0,
                        DrivingTimes = [dh.Duration],
                        IdleTime = 0,
                    });
                }
            }
            Console.WriteLine("Transformation complete");
        }

        public Graph GenerateDiscreteGraph()
        {
            Console.WriteLine("Generating visual representation of graph");
            Graph graph = new();
            // Reset graph
            graph.LayoutAlgorithmSettings = new SugiyamaLayoutSettings()
            {
                LiftCrossEdges = true,
            };

            foreach (var dts in DTrips)
            {
                foreach (DTrip dt in dts)
                {
                    Node node = Formatting.GraphElement.TripNode(dt);
                    graph.AddNode(node);
                }
            }
            graph.AddNode(new Node(DDepotStart.Id.ToString()) { Label = { Text = "Depot" } });
            graph.AddNode(new Node(DDepotEnd.Id.ToString()) { Label = { Text = "Depot" } });

            foreach (var fdh in DDeadheads)
            {
                // Dont include edges even if their not show to save space
                Edge edge = Formatting.GraphElement.DeadheadEdge(fdh.From.Id.ToString(), fdh.To.Id.ToString(), fdh.Costs, graph);
                graph.AddPrecalculatedEdge(edge);
            }

            graph.LayerConstraints.RemoveAllConstraints();
            // Ensure that different charge same trips are above eachother
            for (int i = 0; i < DTrips.Count; i++)
            {
                for (int j = 0; j < DTrips[i].Count; j++)
                {
                    for (int k = j + 1; k < DTrips[i].Count; k++)
                    {
                        graph.LayerConstraints.AddUpDownVerticalConstraint(
                            graph.FindNode(DTrips[i][j].Id.ToString()),
                            graph.FindNode(DTrips[i][k].Id.ToString())
                        );
                    }
                }
            }

            // Depot start left of everything
            Node depotStartNode = graph.FindNode(DDepotStart.Id.ToString());
            for (int i = 0; i < DTrips.Count; i++)
            {
                for (int j = 0; j < DTrips[i].Count; j++)
                {
                    graph.LayerConstraints.AddLeftRightConstraint(
                        depotStartNode,
                        graph.FindNode(DTrips[i][j].Id.ToString())
                    );
                }
            }
            // Depot end right left of everything
            Node depotEndNode = graph.FindNode(DDepotEnd.Id.ToString());
            for (int i = 0; i < DTrips.Count; i++)
            {
                for (int j = 0; j < DTrips[i].Count; j++)
                {
                    graph.LayerConstraints.AddLeftRightConstraint(
                        graph.FindNode(DTrips[i][j].Id.ToString()),
                        depotEndNode
                    );
                }
            }

            // Time dependency between individual nodes
            foreach ((var dl1, var dl2) in DTrips.SelectMany(x => DTrips.Select(y => (x, y))))
            {
                foreach ((DTrip dt1, DTrip dt2) in dl1.SelectMany(x => dl2.Select(y => (x, y))))
                {
                    if (dt1.Trip.EndTime <= dt2.Trip.StartTime || dt1.Trip.StartTime < dt2.Trip.StartTime)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(
                            graph.FindNode(dt1.Id.ToString()),
                            graph.FindNode(dt2.Id.ToString())
                        );
                    }
                }
            }

            graph.Directed = true;
            Console.WriteLine("Graph generated");
            return graph;
        }
    }
}
