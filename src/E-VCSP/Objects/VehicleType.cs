namespace E_VCSP.Objects
{
    /// <summary>
    /// Vehicle descriptor; Only 1 in use per instance
    /// Everything except for capacity scaled for [0, 100] SoC range
    /// </summary>
    internal class VehicleType
    {
        /// <summary>
        /// Data ID
        /// </summary>
        internal required string Id;
        /// <summary>
        /// Index into vehicle type array
        /// </summary>
        internal required int Index;
        /// <summary>
        /// Total capacity in KWh
        /// </summary>
        internal double Capacity;
        /// <summary>
        ///  Usage per M driven
        /// </summary>
        internal double DriveUsage;
        /// <summary>
        /// Usage per second idle
        /// </summary>
        internal double IdleUsage;
        /// <summary>
        /// Minimum SoC (range [0, 100])
        /// </summary>
        internal double MinCharge = 0;
        /// <summary>
        /// Maximum SoC (range [0, 100])
        /// </summary>
        internal double MaxCharge = 100;
        /// <summary>
        /// Charge at start of day (range [0, 100])
        /// </summary>
        internal double StartCharge = 100;
    }
}
