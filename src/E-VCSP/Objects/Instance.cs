using E_VCSP.Parsing;

namespace E_VCSP.Objects
{
    internal class Instance
    {
        internal List<Location> ChargingLocations;
        internal List<Location> Locations;
        internal List<Trip> Trips;
        internal List<Deadhead> Deadheads;
        internal List<VehicleType> VehicleTypes;

        public List<DeadheadTemplate> DeadheadTemplates;

        internal Instance(string path)
        {
            // Load initial set of locations with chargers; may not be complete.
            Locations = new ParserLocations().Parse(path, []);
            ChargingLocations = [.. Locations];

            // Add additional location info based on crew properties
            new ParserCrew().Parse(path, Locations);

            // If no depot is found yet; let the first parsed location be the depot. 
            // TODO: dit werkt toevallig voor de Terschelling dataset, maar dit moet wel echt gefixt worden. 
            if (Locations.Find(loc => loc.IsDepot) == null) Locations[0].IsDepot = true;

            // Trips are parsed directly; Can add locations that were not previously known.
            Trips = new ParserTrips().Parse(path, Locations);

            // Vehicle types; can add charging curve info to locations
            VehicleTypes = new ParserVehicleTypes().Parse(path, Locations);

            // Deadheads between locations are given
            DeadheadTemplates = new ParserDeadheadTemplates().Parse(path, Locations);
            // Add symetric deadheads to model drives back
            var sym = DeadheadTemplates.Select(dh => new DeadheadTemplate
            {
                From = dh.To,
                To = dh.From,
                Distance = dh.Distance,
                Duration = dh.Duration,
                Id = $"dht-sym{dh.Id}"
            }).ToList();
            DeadheadTemplates.AddRange(sym);
            // Add self-edges to model waiting time
            foreach (Location loc in Locations)
            {
                DeadheadTemplates.Add(new DeadheadTemplate
                {
                    From = loc,
                    To = loc,
                    Distance = 0,
                    Duration = 300,// TODO: add actual modeled time
                    Id = $"dht-self{DeadheadTemplates.Count}"
                });
            }

            // Usign deadhead templates, determine feasible deadheads based on trips
            Deadheads = [];

            // Bonk no more deadheads
            if (true) return;
            foreach (Trip t1 in Trips)
            {
                foreach (Trip t2 in Trips)
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
    }
}
