using E_VCSP.Objects;

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
                ( "ChargeSpeedUniform", "Laadsnelheid (0-100%) kWh" ),
                ( "Available", "Aantal beschikbaar" ),
            };
        }

        internal override VehicleType ParseSingle(
            int index,
            List<string> line,
            Dictionary<string, int> attributeIndexMapping,
            List<Location> locations
        )
        {
            double Capacity = double.Parse(line[attributeIndexMapping["Capacity"]]);

            // Rescale all usage patterns to percentage instead of direct KWh measures
            double DriveUsage = double.Parse(line[attributeIndexMapping["DriveUsage"]]) / Capacity * 100;
            double IdleUsage = double.Parse(line[attributeIndexMapping["IdleUsage"]]) / Capacity * 100;

            VehicleType vh = new()
            {
                Id = line[attributeIndexMapping["Id"]],
                Capacity = Capacity,
                DriveUsage = DriveUsage,
                IdleUsage = IdleUsage,
                MinCharge = double.Parse(line[attributeIndexMapping["MinCharge"]]) * 100,
                MaxCharge = 1 * 100,
            };

            if (attributeIndexMapping["ChargeSpeedUniform"] != -1)
            {
                // Set all charging locations to accept this vehicle
                foreach (Location loc in locations)
                {
                    if (loc.CanCharge)
                    {
                        // Normalized to percentage gained per second
                        loc.ChargingCurves[vh.Id] = new(
                            [(100, double.Parse(line[attributeIndexMapping["ChargeSpeedUniform"]]) / 3600 / Capacity * 100)],
                            Capacity
                        );
                    }
                }
            }

            return vh;
        }
    }
}
