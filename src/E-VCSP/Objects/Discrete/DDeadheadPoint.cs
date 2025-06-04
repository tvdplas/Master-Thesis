namespace E_VCSP.Objects.Discrete
{
    public abstract class DDeadheadPoint
    {
        public string Id = "";
    }
    public class DTrip : DDeadheadPoint
    {
        public required Trip Trip;
        public double StartingSoC;
    }
    public class DDepot : DDeadheadPoint
    {
        public required Location Location;
    }
}
