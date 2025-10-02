using E_VCSP.Objects.ParsedData;

namespace E_VCSP.Parsing {
    public class ParserLocations : ParserBase<Location> {
        public ParserLocations() {
            filename = "chargers.csv";
            attributeNameMapping = new()
            {
                ( "Id", "Locatie" ),
                ( "IsDepot", "Depot"),
                ( "ChargeSpots", "Aantal laadpunten" ),
                ( "ChargeTotalPower", "Vermogen (kW)" ),
            };
        }

        public override Location ParseSingle(int index, List<string> headers, List<string> line, Dictionary<string, int> attributeIndexMapping, List<Location> locations) {
            Location loc = GetOrCreateLocation(line[attributeIndexMapping["Id"]], locations);
            loc.ChargeSpots = int.Parse(line[attributeIndexMapping["ChargeSpots"]]);
            loc.IsDepot = line[attributeIndexMapping["IsDepot"]] == "1";
            loc.ChargeTotalPower = double.Parse(line[attributeIndexMapping["ChargeTotalPower"]]);
            loc.CanCharge = loc.ChargeSpots > 0 && loc.ChargeTotalPower > 0;
            return loc;
        }
    }
}
