namespace E_VCSP.Objects
{
    internal class Trip
    {
        internal string Route;
        internal string FromLocation;
        internal string ToLocation;
        internal int StartTime; // unit: seconds
        internal int EndTime; // unit: seconds
        internal int Duration; // unit: seconds
        internal int Distance; // unit: meters
        internal int MinimumLayover; // unit: seconds
        internal List<string> AllowedVehicleTypes;
        internal string Id;
    }
}
