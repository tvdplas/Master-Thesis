using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData {
    /// <summary>
    /// Non-time bound deadhead.
    /// </summary>
    public class DeadheadTemplate {
        /// <summary>
        /// Starting location of template
        /// </summary>
        public required Location StartLocation;
        /// <summary>
        /// Ending location of template
        /// </summary>
        public required Location EndLocation;
        /// <summary>
        /// Driving time in seconds
        /// </summary>
        public int Duration;
        /// <summary>
        /// Driving distance in meters  
        /// </summary>
        public int Distance;
        /// <summary>
        /// Only allowed to be used when a line frequency is changing
        /// </summary>
        public bool FreqChangeOnly;
        /// <summary>
        /// Name/ID of the deadhead template
        /// </summary>
        [JsonInclude]
        public required string Id;

        public override string ToString() {
            return Id;
        }
    }
}
