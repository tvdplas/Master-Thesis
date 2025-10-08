using E_VCSP.Objects.ParsedData;

namespace E_VCSP.Parsing {
    public class Empty { };
    public class ParserCrew : ParserBase<Empty> {
        public ParserCrew() {
            filename = "crew.csv";
            attributeNameMapping = new()
            {
                ( "Id", "Locatie" ),
                ( "HandoverAllowed", "Switch allowed" ),
                ( "SignOnTime", "Opstaptijd" ),
                ( "SignOffTime", "Afstaptijd" ),
                ( "BreakAllowed", "Break allowed" ),
                ( "BrutoNetto", "Brutto-netto" ),
            };
        }

        public override Empty ParseSingle(
            int index,
            List<string> headers,
            List<string> line,
            Dictionary<string, int> attributeIndexMapping,
            List<Location> locations
        ) {
            Location loc = GetOrCreateLocation(line[attributeIndexMapping["Id"]], locations);
            loc.HandoverAllowed = line[attributeIndexMapping["HandoverAllowed"]] == "1";
            loc.BreakAllowed = line[attributeIndexMapping["BreakAllowed"]] == "1";
            loc.BrutoNetto = ParseTime(line[attributeIndexMapping["BrutoNetto"]]);
            loc.SignOnTime = ParseTime(line[attributeIndexMapping["SignOnTime"]]);
            loc.SignOffTime = ParseTime(line[attributeIndexMapping["SignOffTime"]]);

            // Dummy to fit into parser framework
            return new();
        }
    }
}
