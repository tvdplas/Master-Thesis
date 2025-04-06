using E_VCSP.Parsing;

namespace E_VCSP.Objects
{
    internal class Instance
    {
        internal List<Location> ChargingLocations;
        internal List<Location> Locations;
        internal List<Trip> Trips;
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
        }
    }
}
