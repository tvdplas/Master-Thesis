namespace E_VCSP.Objects.Discrete
{
    public class DDeadhead
    {
        public required DDeadheadPoint From;
        public required DDeadheadPoint To;
        public List<int> DrivingTimes = new();
        public int IdleTime;
        public int ChargingTime;
        public double ChargeGained;
        public double ChargeUsed;
        public double Costs;
    }
}
