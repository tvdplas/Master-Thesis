using E_VCSP.Objects;
using Microsoft.Msagl.Drawing;

namespace E_VCSP.Solver
{
    internal abstract class Node
    {
        internal int Index;
    }

    internal class TripNode : Node
    {
        internal required Trip Trip;
    }

    internal class DepotNode : Node
    {
        internal required Location Depot;
    }

    internal class Arc
    {
        internal required Node From;
        internal required Node To;
        internal required LabelDeadhead Deadhead;
    }

    internal class ChargingAction
    {
        internal required Location ChargeLocation;
        internal double ChargeUsedTo;
        internal double ChargeUsedFrom;
        internal double DrivingCost;
        internal int DrivingTimeTo;
        internal int DrivingTimeFrom;
        internal int TimeAtLocation;
    }

    internal class LabelDeadhead
    {
        internal required DeadheadTemplate DeadheadTemplate;
        internal required List<ChargingAction> ChargingActions;
        internal double BaseCost;
    }


    internal class EVSPLabeling : Solver
    {
        private Instance instance;

        List<Node> nodes = new();
        List<List<Arc>> adj = new();

        List<List<DeadheadTemplate>> LocationMapping = new();

        public EVSPLabeling(Instance instance)
        {
            this.instance = instance;

            // Generate lookup table for deadhead templates 
            foreach (Location l1 in instance.Locations)
            {
                LocationMapping.Add(new());
                foreach (Location l2 in instance.Locations)
                {
                    DeadheadTemplate? dht = instance.DeadheadTemplates.Find((x) => x.From == l1 && x.To == l2);
                    if (dht == null) throw new InvalidDataException($"Location pair {l1} x {l2} did not have corresponding deadhead template");
                    LocationMapping[l1.Index].Add(dht);
                }
            }

            GenerateGraph();
        }

        private void GenerateGraph()
        {
            VehicleType vt = instance.VehicleTypes[0];

            // Transform trips into nodes, add depot start/end
            // Depot start = ^2, depot end = ^1
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                nodes.Add(new TripNode() { Index = i, Trip = instance.Trips[i] });
            }

            Location? depot = instance.Locations.Find(loc => loc.IsDepot);
            if (depot == null) throw new InvalidDataException("No depot found in location list");
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // Start depot
            nodes.Add(new DepotNode() { Index = nodes.Count, Depot = depot }); // End depot


            // Initialize adjacency lists
            for (int i = 0; i < nodes.Count; i++) adj.Add(new());

            // depot start arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate dht = LocationMapping[depot.Index][tn.Trip.From.Index];
                double baseCost = Config.PULLOUT_COST + (dht.Distance * Config.M_COST);

                // TODO: Charge directly after depot?
                adj[^2].Add(new Arc() { From = nodes[^2], To = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } });
            }
            // depot end arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn = (TripNode)nodes[i];
                DeadheadTemplate dht = LocationMapping[tn.Trip.To.Index][depot.Index];
                double baseCost = dht.Distance * Config.M_COST;
                // TODO: Charge directly before depot?
                adj[^1].Add(new Arc() { To = nodes[^1], From = tn, Deadhead = new() { ChargingActions = [], BaseCost = baseCost, DeadheadTemplate = dht } });
            }

            // Trip to trip arcs
            for (int i = 0; i < nodes.Count - 2; i++)
            {
                TripNode tn1 = (TripNode)nodes[i];

                for (int j = 0; j < nodes.Count - 2; j++)
                {
                    if (i == j) continue;

                    TripNode tn2 = (TripNode)nodes[j];
                    DeadheadTemplate dht = LocationMapping[tn1.Trip.To.Index][tn2.Trip.From.Index];

                    if (tn1.Trip.EndTime + dht.Duration > tn2.Trip.StartTime) continue; // Deadhead not time feasible

                    double baseCost = dht.Distance * Config.M_COST;
                    List<ChargingAction> chargingActions = new List<ChargingAction>();
                    foreach (Location chargeLocation in instance.ChargingLocations)
                    {
                        DeadheadTemplate dhtTo = LocationMapping[tn1.Trip.To.Index][chargeLocation.Index];
                        DeadheadTemplate dhtFrom = LocationMapping[chargeLocation.Index][tn2.Trip.From.Index];

                        int chargeTime = tn2.Trip.StartTime - (tn1.Trip.EndTime + dhtTo.Duration + dhtFrom.Duration);
                        if (chargeTime < Config.MIN_CHARGE_TIME) continue; // Charging at location not feasible

                        chargingActions.Add(new ChargingAction()
                        {
                            ChargeLocation = chargeLocation,
                            ChargeUsedTo = dhtTo.Distance * vt.DriveUsage,
                            ChargeUsedFrom = dhtFrom.Distance * vt.DriveUsage,
                            DrivingCost = (dhtTo.Distance + dhtFrom.Distance) * Config.M_COST,
                            DrivingTimeFrom = dhtFrom.Duration,
                            DrivingTimeTo = dhtTo.Duration,
                            TimeAtLocation = chargeTime,
                        });
                    }

                    adj[i].Add(new Arc() { To = tn2, From = tn1, Deadhead = new() { ChargingActions = chargingActions, BaseCost = baseCost, DeadheadTemplate = dht } });
                }
            }



            // Starting graph generation
            foreach (Trip t1 in instance.Trips)
            {
                foreach (Trip t2 in instance.Trips)
                {
                    // Never feasible
                    if (t1.EndTime > t2.StartTime) continue;

                    // Check all charging locations to see which ones are feasible and/or result in most charge gained
                    Deadhead? dh = null;
                    foreach (Location chargeLoc in ChargingLocations)
                    {
                        DeadheadTemplate? dhtToCharge = DeadheadTemplates.Find(x => x.From == t1.To && x.To == chargeLoc);
                        DeadheadTemplate? dhtFromCharge = DeadheadTemplates.Find(x => x.From == chargeLoc && x.To == t2.From);

                        // No templates found; deadhead might be possible, but we dont have the info to know.
                        if (dhtToCharge == null || dhtFromCharge == null) continue;

                        // Check feasibility TODO: possibly add turnaround time
                        int travelTime = dhtToCharge.Duration + dhtFromCharge.Duration;
                        int idleTime = t2.StartTime - t1.EndTime - travelTime;

                        // Not possible to perform travels
                        if (idleTime < 0) continue;

                        double maxCharge = idleTime / 3600 * chargeLoc.ChargePowerPerSpot;
                        if (dh == null || maxCharge > dh.MaxCharge)
                        {
                            dh = new Deadhead()
                            {
                                From = t1,
                                To = t2,
                                Templates = [dhtToCharge, dhtFromCharge],
                                Id = $"dh{Deadheads.Count}",
                                MaxCharge = maxCharge,
                            };
                        }
                    }

                    // No valid charging deadhead was found; this is either 
                    // - Because there was not enough time to go to a charging location
                    // - Or because there wasn't enough time to perform the trip from t1 to t2
                    // We will check for the first case; if that also fails, we know that we can't create a deadhead.
                    if (dh == null)
                    {
                        DeadheadTemplate? dht = DeadheadTemplates.Find(x => x.From == t1.To && x.To == t2.From);

                        // No deadhead between t1 and t2 possible, skip to next
                        if (dht == null) continue;

                        int idleTime = t2.StartTime - t1.EndTime - dht.Duration;

                        // No idle time possible, so no deadhead possible
                        if (idleTime < 0) continue;

                        // TODO: combined charging, but for now we assume one side will be optimal.
                        // Note that this should always be 0
                        double maxCharge = idleTime / 3600 * t1.From.ChargePowerPerSpot;
                        dh = new Deadhead()
                        {
                            From = t1,
                            To = t2,
                            Templates = [dht],
                            Id = $"dh{Deadheads.Count}",
                            MaxCharge = maxCharge,
                        };
                    }

                    if (dh != null) Deadheads.Add(dh);
                }
            }
        }

        internal override Graph GenerateSolutionGraph()
        {
            throw new NotImplementedException();
        }

        internal override bool Solve()
        {
            throw new NotImplementedException();
        }
    }
}
