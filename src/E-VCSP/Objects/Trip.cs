namespace E_VCSP.Objects
{
    internal class Trip
    {
        internal required string Route;
        internal required Location From;
        internal required Location To;
        internal int StartTime; // unit: seconds
        internal int EndTime; // unit: seconds
        internal int Duration; // unit: seconds
        internal int Distance; // unit: meters
        internal int MinimumLayover; // unit: seconds
        internal required List<string> AllowedVehicleTypes;
        internal required string Id;

        public override string ToString()
        {
            return Id;
        }
        public string ToLongString(bool showTime)
        {
            return $"{From} -> {To}" + (showTime ? $"\n{Formatting.Time.HHMMSS(StartTime)} - {Formatting.Time.HHMMSS(EndTime)}" : "");
        }
    }
}
