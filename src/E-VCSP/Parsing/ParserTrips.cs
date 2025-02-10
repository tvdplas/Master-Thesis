using E_VCSP.Objects;
using System.Globalization;

namespace E_VCSP.Parsing
{
    internal class ParserTrips : ParserBase<Trip>
    {
        internal ParserTrips()
        {
            filename = "trips.csv";
            attributeNameMapping = new()
            {
                ( "Route", "Route" ),
                ( "FromLocation", "From" ),
                ( "ToLocation", "To" ),
                ( "StartTime", "Start" ),
                ( "EndTime", "End" ),
                ( "Duration", "Duration" ),
                ( "Distance", "Distance" ),
                ( "MinimumLayover", "MinLay" ),
                ( "AllowedVehicleTypes", "VehGrp" ),
            };
        }

        internal override Trip ParseSingle(int index, List<string> line, Dictionary<string, int> attributeIndexMapping)
        {
            return new Trip()
            {
                Route = line[attributeIndexMapping["Route"]],
                FromLocation = line[attributeIndexMapping["FromLocation"]],
                ToLocation = line[attributeIndexMapping["ToLocation"]],
                StartTime = ParseTime(line[attributeIndexMapping["StartTime"]]),
                EndTime = ParseTime(line[attributeIndexMapping["EndTime"]]),
                Duration = ParseTime(line[attributeIndexMapping["Duration"]]),
                Distance = (int)(1000 * double.Parse(line[attributeIndexMapping["Distance"]], CultureInfo.InvariantCulture)),
                MinimumLayover = ParseTime(line[attributeIndexMapping["MinimumLayover"]]),
                AllowedVehicleTypes = line[attributeIndexMapping["AllowedVehicleTypes"]].Split(',').ToList(),
                Id = $"t{index}",
            };
        }
    }
}
