using E_VCSP.Objects;
using System.Globalization;

namespace E_VCSP.Parsing
{
    internal class ParserDeadheadTemplates : ParserBase<DeadheadTemplate>
    {
        internal ParserDeadheadTemplates()
        {
            filename = "deadheads.csv";
            attributeNameMapping = new()
            {
                ( "FromId", "From" ),
                ( "ToId", "To" ),
                ( "Duration", "Time" ),
                ( "Distance", "Distance" ),
            };
        }

        internal override DeadheadTemplate ParseSingle(int index, List<string> line, Dictionary<string, int> attributeIndexMapping, List<Location> locations)
        {
            Location from = GetOrCreateLocation(line[attributeIndexMapping["FromId"]], locations);
            Location to = GetOrCreateLocation(line[attributeIndexMapping["ToId"]], locations);

            return new DeadheadTemplate()
            {
                From = from,
                To = to,
                Duration = ParseTime(line[attributeIndexMapping["Duration"]]) * 60,
                Distance = (int)(1000 * double.Parse(line[attributeIndexMapping["Distance"]], CultureInfo.InvariantCulture)),
                Id = $"dht{index}",
            };
        }
    }
}
