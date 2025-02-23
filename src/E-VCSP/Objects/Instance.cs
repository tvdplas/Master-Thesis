using E_VCSP.Parsing;

namespace E_VCSP.Objects
{
    internal class Instance
    {
        internal List<Location> Locations;
        internal List<Trip> Trips;
        internal List<Deadhead> Deadheads;

        private List<DeadheadTemplate> deadheadTemplates;

        internal Instance(string path)
        {
            // Get all charger locations, keep copy in order to check for charging trips later.
            Locations = new ParserLocations().Parse(path, []);
            List<Location> chargingLocations = [.. Locations];

            // Trips are parsed directly
            Trips = new ParserTrips().Parse(path, Locations);

            // Deadheads between locations are given, additional self-edge deadheads are added
            // for easier formulations later
            deadheadTemplates = new ParserDeadheadTemplates().Parse(path, Locations);
            foreach (Location loc in Locations)
            {
                deadheadTemplates.Add(new DeadheadTemplate
                {
                    From = loc,
                    To = loc,
                    Distance = 0,
                    Duration = 0,
                    Id = $"dht-self{deadheadTemplates.Count}"
                });
            }

            // Usign deadhead templates, determine feasible deadheads based on trips
            Deadheads = [];
            foreach (Trip t1 in Trips)
            {
                foreach (Trip t2 in Trips)
                {
                    // Never feasible
                    if (t1.EndTime > t2.StartTime) continue;

                    // Check all charging locations to see which ones are feasible and/or result in most charge gained
                    Deadhead? dh = null;
                    foreach (Location chargeLoc in chargingLocations)
                    {
                        DeadheadTemplate? dhtToCharge = deadheadTemplates.Find(x => x.From == t1.To && x.To == chargeLoc);
                        DeadheadTemplate? dhtFromCharge = deadheadTemplates.Find(x => x.From == chargeLoc && x.To == t2.From);

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
                        DeadheadTemplate? dht = deadheadTemplates.Find(x => x.From == t1.To && x.To == t2.From);

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
