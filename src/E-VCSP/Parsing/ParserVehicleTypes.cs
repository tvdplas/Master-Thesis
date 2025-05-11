using E_VCSP.Objects;
using System.Text.RegularExpressions;

namespace E_VCSP.Parsing
{
    internal class ParserVehicleTypes : ParserBase<VehicleType>
    {
        internal ParserVehicleTypes()
        {
            filename = "vehicles.csv";
            attributeNameMapping = new()
            {
                ( "Id", "Type" ),
                ( "Capacity", "Capaciteit" ),
                ( "DriveUsage", "Verbruik (kWh/km)" ),
                ( "IdleUsage", "Verbruik bij stilstand (kWh per uur stilstand)" ),
                ( "MinCharge", "Laagste toegestane lading" ),
                ( "Available", "Aantal beschikbaar" ),
            };
        }

        internal override VehicleType ParseSingle(
            int index,
            List<string> headers,
            List<string> line,
            Dictionary<string, int> attributeIndexMapping,
            List<Location> locations
        )
        {
            double Capacity = double.Parse(line[attributeIndexMapping["Capacity"]]);

            // Rescale all usage patterns to percentage instead of direct KWh measures
            // Driveusage is also rescaled to per meter instead of per km
            double DriveUsage = double.Parse(line[attributeIndexMapping["DriveUsage"]]) / 1000 / Capacity * 100;
            double IdleUsage = double.Parse(line[attributeIndexMapping["IdleUsage"]]) / Capacity * 100;

            VehicleType vh = new()
            {
                Id = line[attributeIndexMapping["Id"]],
                Index = index,
                Capacity = Capacity,
                DriveUsage = DriveUsage,
                IdleUsage = IdleUsage,
                MinCharge = double.Parse(line[attributeIndexMapping["MinCharge"]]) * 100,
                MaxCharge = 1 * 100,
            };

            // Get the columns for max charging speed
            List<(int index, int low, int high)> chargingColumns = new();
            for (int i = 0; i < headers.Count; i++)
            {
                if (headers[i].StartsWith("Laadsnelheid"))
                {
                    // Get the digits before/after the "-"
                    var match = Regex.Match(headers[i], @"\((\d+)-(\d+)%\)");
                    int low = int.Parse(match.Groups[1].Value);
                    int high = int.Parse(match.Groups[2].Value);
                    chargingColumns.Add((i, low, high));
                }
            }

            foreach (Location loc in locations)
            {
                if (loc.CanCharge)
                {
                    var curveDefinition = chargingColumns.Select((val) =>
                    {
                        (int index, int low, int high) = val;
                        return (high, Math.Min(loc.ChargeTotalPower, double.Parse(line[index]) / 3600 / Capacity * 100));
                    }).ToList();
                    loc.ChargingCurves.Add(new(
                        curveDefinition,
                        Capacity
                    ));
                }
            }

            return vh;
        }
    }
}
