namespace E_VCSP.Objects
{
    /// <summary>
    /// Time-bound deadhead
    /// </summary>
    internal class Deadhead
    {
        internal List<DeadheadTemplate> Templates = [];
        internal required Trip From;
        internal required Trip To;
        internal required string Id;
        internal double MaxCharge;

        public int DrivingTime
        {
            get => Templates.Sum(t => t.Duration);
        }

        public int TotalTime
        {
            get => To.StartTime - From.EndTime;
        }

        public int IdleTime
        {
            get => TotalTime - DrivingTime;
        }

        public override string ToString()
        {
            return Id;
        }

        /// <summary>
        /// Returns a string formatted with the specified entries
        /// </summary>
        /// <param name="format">[f]: fromTrip, [t]: toTrip, [s]: total time, [d]: driving time, [c]: charging time, [m]: max charge, [i]: id</param>
        /// <returns></returns>
        public string ToFormattedString(string format)
        {
            return format
                .Replace("[f]", From.ToString())
                .Replace("[t]", To.ToString())
                .Replace("[s]", Formatting.Time.HHMMSS(TotalTime))
                .Replace("[d]", Formatting.Time.HHMMSS(DrivingTime))
                .Replace("[c]", Formatting.Time.HHMMSS(IdleTime))
                .Replace("[m]", $"{MaxCharge} KWh")
                .Replace("[i]", Id);
        }
    }
}
