using E_VCSP.Objects.ParsedData;
using System.Globalization;

namespace E_VCSP.Parsing {
    public class ParserDeadheadTemplates : ParserBase<DeadheadTemplate> {
        public ParserDeadheadTemplates() {
            filename = "deadheads.csv";
            attributeNameMapping = new()
            {
                ( "FromId", "From" ),
                ( "ToId", "To" ),
                ( "Duration", "Time" ),
                ( "Distance", "Distance" ),
                ( "FreqChangeOnly", "FreqChangeOnly" ),
            };
        }

        public override DeadheadTemplate ParseSingle(int index, List<string> headers, List<string> line, Dictionary<string, int> attributeIndexMapping, List<Location> locations) {
            Location from = GetOrCreateLocation(line[attributeIndexMapping["FromId"]], locations);
            Location to = GetOrCreateLocation(line[attributeIndexMapping["ToId"]], locations);

            return new DeadheadTemplate() {
                StartLocation = from,
                EndLocation = to,
                Duration = ParseTime(line[attributeIndexMapping["Duration"]]) * 60,
                Distance = (int)(1000 * double.Parse(line[attributeIndexMapping["Distance"]], CultureInfo.InvariantCulture)),
                FreqChangeOnly = line[attributeIndexMapping["FreqChangeOnly"]].Equals("yes", StringComparison.InvariantCultureIgnoreCase),
                Id = $"dht{index}",
            };
        }
    }
}
