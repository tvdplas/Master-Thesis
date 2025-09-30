using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData {
    /// <summary>
    /// Indicates if a frequency change occurs at the start or end of a trip.
    /// Start/End of operations is also considered a frequency change.
    /// </summary>
    public enum FrequencyChange {
        None, // Part of regular schedule
        EndOfTrip, // Schedule frequency changes after this trip
        StartOfTrip, // Schedule frequency changes before this trip
        SingleTrip, // Schedule frequency changes before and after this trip / trip is an outlier
    }

    public class Trip {
        public required string Route;
        public required Location StartLocation;
        public required Location EndLocation;
        public int StartTime; // unit: seconds
        public int EndTime; // unit: seconds
        public int Duration; // unit: seconds
        public int Distance; // unit: meters
        public int MinimumLayover; // unit: seconds
        public List<string> AllowedVehicleTypes = [];
        public FrequencyChange FrequencyChange = FrequencyChange.None;
        [JsonInclude]
        public required string Id;
        public int Index = -1;

        public override string ToString() {
            return Id;
        }
        public string ToLongString(bool showTime) {
            return $"{StartLocation} -> {EndLocation}" + (showTime ? $"\n{Formatting.Time.HHMMSS(StartTime)} - {Formatting.Time.HHMMSS(EndTime)}" : "");
        }
    }
}
