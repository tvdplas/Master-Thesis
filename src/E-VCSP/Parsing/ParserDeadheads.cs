using E_VCSP.Objects;
using System.Globalization;

namespace E_VCSP.Parsing
{
    internal class ParserDeadheads : ParserBase<Deadhead>
    {
        internal ParserDeadheads()
        {
            filename = "deadheads.csv";
            attributeNameMapping = new()
            {
                ( "FromLocation", "From" ),
                ( "ToLocation", "To" ),
                ( "Duration", "Time" ),
                ( "Distance", "Distance" ),
            };
        }

        internal override Deadhead ParseSingle(int index, List<string> line, Dictionary<string, int> attributeIndexMapping)
        {
            return new Deadhead()
            {
                FromLocation = line[attributeIndexMapping["FromLocation"]],
                ToLocation = line[attributeIndexMapping["ToLocation"]],
                Duration = ParseTime(line[attributeIndexMapping["Duration"]]),
                Distance = (int)(1000 * double.Parse(line[attributeIndexMapping["Distance"]], CultureInfo.InvariantCulture)),
                Id = $"dh{index}",
            };
        }
    }
}
