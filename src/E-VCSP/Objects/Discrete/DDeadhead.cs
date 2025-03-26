namespace E_VCSP.Objects.Discrete
{
    internal class DDeadhead
    {
        internal required DDeadheadPoint From;
        internal required DDeadheadPoint To;
        internal List<int> DrivingTimes = new();
        internal int IdleTime;
        internal int ChargingTime;
        internal double ChargeGained;
        internal double ChargeUsed;
        internal double Costs;
    }
}
