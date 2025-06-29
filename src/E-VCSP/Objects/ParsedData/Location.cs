namespace E_VCSP.Objects.ParsedData
{
    public class Location
    {
        public required string Id;
        public int Index = -1;

        public double ChargeTotalPower = 0; // KWh
        public int ChargeSpots = 0;
        public List<ChargingCurve> ChargingCurves = new(); // indexed by vehicle id

        public int SignOnTime = 0;
        public int SignOffTime = 0;
        public bool BreakAllowed = false;
        public bool HandoverAllowed = false;
        public int BrutoNetto = 0; // Difference in time between crew member downtime and actual counted break time

        public bool IsDepot = false;

        public int MinHandoverTime => Math.Max(SignOffTime, SignOnTime);

        public bool FreeIdle => BreakAllowed || CanCharge;

        public double ChargePowerPerSpot => ChargeSpots > 0 ? ChargeTotalPower / ChargeSpots : 0;

        public bool CanCharge => ChargePowerPerSpot > 0;

        public override string ToString()
        {
            return Id;
        }
    }
}
