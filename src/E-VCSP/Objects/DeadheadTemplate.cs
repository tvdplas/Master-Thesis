namespace E_VCSP.Objects
{
    /// <summary>
    /// Non-time bound deadhead.
    /// </summary>
    public class DeadheadTemplate
    {
        public required Location From;
        public required Location To;
        public int Duration;
        public int Distance;
        public required string Id;

        public override string ToString()
        {
            return Id;
        }
        public string ToLongString(bool showInfo)
        {
            return $"{From} -> {To}" + (showInfo ? $"\n{Formatting.Time.HHMMSS(Duration)} ({Formatting.Distance.KM(Distance)})" : "");
        }
    }
}
