using E_VCSP.Parsing;

namespace E_VCSP.Objects.ParsedData {
    public class Instance {
        public string Path { get; private set; }
        public List<Location> ChargingLocations;
        public List<Location> Locations;
        public List<Trip> Trips;
        public List<VehicleType> VehicleTypes;

        public int DepotStartIndex = -1;
        public int DepotEndIndex = -1;

        public List<DeadheadTemplate> DeadheadTemplates;
        public List<DeadheadTemplate> ExtendedTemplates;

        public Dictionary<string, List<int>> StartDescriptorToTripIndex = [];
        public Dictionary<string, List<int>> EndDescriptorToTripIndex = [];

        public Instance(string path) {
            Path = path;

            // Load initial set of locations with chargers; may not be complete.
            Locations = new ParserLocations().Parse(path, []) ?? [];
            ChargingLocations = [.. Locations];

            // Add additional location info based on crew properties
            new ParserCrew().Parse(path, Locations);

            // If no depot is found yet; let the first parsed location be the depot. 
            if (Locations.Find(loc => loc.IsDepot) == null) {
                var depotTarget = Locations.Find(x => x.CanCharge && x.BreakAllowed) ?? throw new InvalidDataException("Cannot find depot");
                depotTarget.IsDepot = true;
            }

            // Trips are parsed directly; Can add locations that were not previously known.
            Trips = new ParserTrips().Parse(path, Locations);
            Trips.Sort((a, b) => a.EndTime.CompareTo(b.EndTime));
            for (int i = 0; i < Trips.Count; i++) {
                Trips[i].Index = i;
                Trips[i].Id = "t" + i;

                // Create trip lookup based on start location + time, and end location + time
                string startDesc = Utils.Descriptor.Create(Trips[i].StartLocation, Trips[i].StartTime);
                StartDescriptorToTripIndex.TryAdd(startDesc, []);
                StartDescriptorToTripIndex[startDesc].Add(i);

                string endDesc = Utils.Descriptor.Create(Trips[i].EndLocation, Trips[i].EndTime);
                EndDescriptorToTripIndex.TryAdd(endDesc, []);
                EndDescriptorToTripIndex[endDesc].Add(i);
            }

            // Vehicle types; can add charging curve info to locations
            VehicleTypes = new ParserVehicleTypes().Parse(path, Locations);

            // Deadheads between locations are given
            DeadheadTemplates = new ParserDeadheadTemplates().Parse(path, Locations);
            // Add symetric deadheads to model drives back
            var sym = DeadheadTemplates.Select(dh => new DeadheadTemplate {
                StartLocation = dh.EndLocation,
                EndLocation = dh.StartLocation,
                Distance = dh.Distance,
                Duration = dh.Duration,
                Id = $"dht-sym{dh.Id}"
            }).ToList();
            DeadheadTemplates.AddRange(sym);

            // Add self-edges to model waiting time
            foreach (Location loc in Locations) {
                DeadheadTemplates.Add(new DeadheadTemplate {
                    StartLocation = loc,
                    EndLocation = loc,
                    Distance = 0,
                    Duration = 0,
                    Id = $"dht-self{DeadheadTemplates.Count}"
                });
            }

            ExtendedTemplates = [.. DeadheadTemplates];
            // For each pair that was not yet included, find either a trip which does this route, or a route via the depot; 
            // take the minimum time / distance. 
            foreach (Location loc1 in Locations) {
                foreach (Location loc2 in Locations) {
                    if (ExtendedTemplates.Find(x => x.StartLocation == loc1 && x.EndLocation == loc2) != null) continue;

                    // Trip which has this route
                    Trip? trip = Trips.Find(x => x.StartLocation == loc1 && x.EndLocation == loc2);
                    int tripDistance = trip?.Distance ?? int.MaxValue;
                    int tripDuration = trip?.Duration ?? int.MaxValue;

                    // Via other point (probably depot)
                    Location? loc3 = Locations.Find(x =>
                        ExtendedTemplates.Find(y => y.StartLocation == loc1 && y.EndLocation == x) != null
                        && ExtendedTemplates.Find(y => y.StartLocation == x && y.EndLocation == loc2) != null
                    );
                    DeadheadTemplate? dht1 = ExtendedTemplates.Find(y => y.StartLocation == loc1 && y.EndLocation == loc3);
                    DeadheadTemplate? dht2 = ExtendedTemplates.Find(y => y.StartLocation == loc3 && y.EndLocation == loc2);

                    int detourDistance = (dht1 != null && dht2 != null) ? dht1.Distance + dht2.Distance : int.MaxValue;
                    int detourDuration = (dht1 != null && dht2 != null) ? dht1.Duration + dht2.Duration : int.MaxValue;

                    ExtendedTemplates.Add(new DeadheadTemplate() {
                        StartLocation = loc1,
                        EndLocation = loc2,
                        Duration = Math.Min(tripDuration, detourDuration),
                        Distance = Math.Min(tripDistance, detourDistance),
                        Id = $"dht-generated-{loc1}-{loc2}",
                    });
                }
            }

            DepotStartIndex = Trips.Count;
            DepotEndIndex = Trips.Count + 1;


            // Mark trips at start / of operations as frequency change
            Dictionary<(string line, int startLocationIndex), List<Trip>> byLineAndStart = [];
            foreach (Trip trip in Trips) {
                byLineAndStart.TryAdd((trip.Route, trip.StartLocation.Index), []);
                byLineAndStart[(trip.Route, trip.StartLocation.Index)].Add(trip);
            }

            // Sort and assign
            foreach (var key in byLineAndStart.Keys) {
                byLineAndStart[key].Sort((a, b) => a.StartTime.CompareTo(b.StartTime));
                byLineAndStart[key][0].FrequencyChange = FrequencyChange.StartOfTrip;
                byLineAndStart[key][^1].FrequencyChange =
                    byLineAndStart[key][^1].FrequencyChange != FrequencyChange.None
                        ? FrequencyChange.SingleTrip
                        : FrequencyChange.EndOfTrip;
            }
        }
    }
}
