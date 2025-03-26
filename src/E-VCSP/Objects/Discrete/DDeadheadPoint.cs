namespace E_VCSP.Objects.Discrete
{
    internal abstract class DDeadheadPoint
    {
        internal string Id = "";
    }
    internal class DTrip : DDeadheadPoint
    {
        internal required Trip Trip;
        internal double StartingSoC;
    }
    internal class DDepot : DDeadheadPoint
    {
        internal required Location Location;
    }
}
