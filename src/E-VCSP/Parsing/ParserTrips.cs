using E_VCSP.Objects;
using System.Globalization;

namespace E_VCSP.Parsing
{
    public class ParserTrips : ParserBase<Trip>
    {
        public ParserTrips()
        {
            filename = "trips.csv";
            attributeNameMapping = new()
            {
                ( "Route", "Route" ),
                ( "FromId", "From" ),
                ( "ToId", "To" ),
                ( "StartTime", "Start" ),
                ( "EndTime", "End" ),
                ( "Duration", "Duration" ),
                ( "Distance", "Distance" ),
                ( "MinimumLayover", "MinLay" ),
                ( "AllowedVehicleTypes", "VehGrp" ),
            };
        }

        public override Trip ParseSingle(int index, List<string> headers, List<string> line, Dictionary<string, int> attributeIndexMapping, List<Location> locations)
        {
            Location from = GetOrCreateLocation(line[attributeIndexMapping["FromId"]], locations);
            Location to = GetOrCreateLocation(line[attributeIndexMapping["ToId"]], locations);

            return new Trip()
            {
                Route = line[attributeIndexMapping["Route"]],
                From = from,
                To = to,
                StartTime = ParseTime(line[attributeIndexMapping["StartTime"]]),
                EndTime = ParseTime(line[attributeIndexMapping["EndTime"]]),
                Duration = ParseTime(line[attributeIndexMapping["Duration"]]),
                Distance = (int)(1000 * double.Parse(line[attributeIndexMapping["Distance"]], CultureInfo.InvariantCulture)),
                MinimumLayover = ParseTime(line[attributeIndexMapping["MinimumLayover"]]),
                AllowedVehicleTypes = line[attributeIndexMapping["AllowedVehicleTypes"]].Split(',').ToList(),
                Id = $"t{index}",
                Index = index,
            };
        }
    }
}
