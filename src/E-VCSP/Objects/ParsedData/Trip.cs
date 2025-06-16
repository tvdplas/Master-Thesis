namespace E_VCSP.Objects.ParsedData
{
    public class Trip
    {
        public required string Route;
        public required Location From;
        public required Location To;
        public int StartTime; // unit: seconds
        public int EndTime; // unit: seconds
        public int Duration; // unit: seconds
        public int Distance; // unit: meters
        public int MinimumLayover; // unit: seconds
        public required List<string> AllowedVehicleTypes;
        public required string Id;
        public required int Index = -1;

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
