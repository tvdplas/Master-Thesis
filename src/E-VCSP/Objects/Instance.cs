using E_VCSP.Parsing;

namespace E_VCSP.Objects
{
    internal class Instance
    {
        internal List<Location> locations;
        internal List<Trip> trips;
        internal List<Deadhead> deadheads;

        internal Instance(string path)
        {
            locations = new ParserLocations().Parse(path, []);
            trips = new ParserTrips().Parse(path, locations);
            deadheads = new ParserDeadheads().Parse(path, locations);
        }
    }
}
