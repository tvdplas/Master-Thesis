namespace E_VCSP.Objects
{
    // All values rescaled to percentages [0-100] except Capacity
    internal class VehicleType
    {
        internal required string Id;
        internal required int Index;
        internal double Capacity; // KWh
        internal double DriveUsage; // Usage per KM driven
        internal double IdleUsage; // Usage per second idle
        internal double MinCharge = 0; // Fraction of total capacity
        internal double MaxCharge = 100;
        internal double StartCharge = 100;
    }
}
