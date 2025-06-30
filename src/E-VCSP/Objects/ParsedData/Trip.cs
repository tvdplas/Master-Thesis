using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData
{
    public class Trip
    {
        public string Route = "unknown";
        public Location From;
        public Location To;
        public int StartTime; // unit: seconds
        public int EndTime; // unit: seconds
        public int Duration; // unit: seconds
        public int Distance; // unit: meters
        public int MinimumLayover; // unit: seconds
        public List<string> AllowedVehicleTypes = [];
        [JsonInclude]
        public required string Id;
        public int Index = -1;

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
