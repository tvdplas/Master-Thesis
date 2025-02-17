namespace E_VCSP.Objects
{
    internal class Deadhead
    {
        internal required Location From;
        internal required Location To;
        internal int Duration;
        internal int Distance;
        internal required string Id;

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
