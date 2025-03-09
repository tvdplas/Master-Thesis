
using E_VCSP.Objects;

namespace E_VCSP.Parsing
{
    internal class ParserLocations : ParserBase<Location>
    {
        internal ParserLocations()
        {
            filename = "chargers.csv";
            attributeNameMapping = new()
            {
                ( "Id", "Locatie" ),
                ( "ChargeSpots", "Aantal laadpunten" ),
                ( "ChargeTotalPower", "Vermogen (kW)" ),
            };
        }

        internal override Location ParseSingle(int index, List<string> line, Dictionary<string, int> attributeIndexMapping, List<Location> locations)
        {
            Location loc = GetOrCreateLocation(line[attributeIndexMapping["Id"]], locations);
            loc.ChargeSpots = int.Parse(line[attributeIndexMapping["ChargeSpots"]]);
            loc.ChargeTotalPower = double.Parse(line[attributeIndexMapping["ChargeTotalPower"]]);
            return loc;
        }
    }
}
