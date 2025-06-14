﻿using E_VCSP.Parsing;

namespace E_VCSP.Objects
{
    public class Instance
    {
        public List<Location> ChargingLocations;
        public List<Location> Locations;
        public List<Trip> Trips;
        public List<VehicleType> VehicleTypes;

        public int DepotStartIndex = -1;
        public int DepotEndIndex = -1;

        public List<DeadheadTemplate> DeadheadTemplates;

        public Instance(string path)
        {
            // Load initial set of locations with chargers; may not be complete.
            Locations = new ParserLocations().Parse(path, []) ?? [];
            ChargingLocations = [.. Locations];

            // Add additional location info based on crew properties
            new ParserCrew().Parse(path, Locations);

            // If no depot is found yet; let the first parsed location be the depot. 
            if (Locations.Find(loc => loc.IsDepot) == null)
            {
                var depotTarget = Locations.Find(x => x.CanCharge && x.BreakAllowed) ?? throw new InvalidDataException("Cannot find depot");
                depotTarget.IsDepot = true;
            }

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
                    Duration = 0,// TODO: add actual modeled time
                    Id = $"dht-self{DeadheadTemplates.Count}"
                });
            }

            DepotStartIndex = Trips.Count;
            DepotEndIndex = Trips.Count + 1;
        }
    }
}
