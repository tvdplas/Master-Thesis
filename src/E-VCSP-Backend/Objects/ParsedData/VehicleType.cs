using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData {
    /// <summary>
    /// Vehicle descriptor; Only 1 in use per instance
    /// Everything except for capacity scaled for [0, 100] SoC range
    /// </summary>
    public class VehicleType {
        /// <summary>
        /// Data ID
        /// </summary>
        [JsonInclude]
        public required string Id;
        /// <summary>
        /// Index into vehicle type array
        /// </summary>
        public int Index;
        /// <summary>
        /// Total capacity in KWh
        /// </summary>
        public double Capacity;
        /// <summary>
        ///  Usage per M driven
        /// </summary>
        public double DriveUsage;
        /// <summary>
        /// Usage per second idle
        /// </summary>
        public double IdleUsage;
        /// <summary>
        /// Minimum SoC (range [0, 100])
        /// </summary>
        public double MinSoC = 0;
        /// <summary>
        /// Maximum SoC (range [0, 100])
        /// </summary>
        public double MaxSoC = 100;
        /// <summary>
        /// Charge at start of day (range [0, 100])
        /// </summary>
        public double StartSoC = 100;
    }
}
