namespace E_VCSP.Objects
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

        public double ChargePowerPerSpot
        {
            get
            {
                return ChargeSpots > 0 ? ChargeTotalPower / ChargeSpots : 0;
            }
        }

        public bool CanCharge
        {
            get
            {
                return ChargePowerPerSpot > 0;
            }
        }

        public override string ToString()
        {
            return Id;
        }
    }
}
