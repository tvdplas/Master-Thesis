namespace E_VCSP.Objects
{
    internal class Location
    {
        internal required string Id;

        internal double ChargeTotalPower = 0; // KWh
        internal int ChargeSpots = 0;
        internal Dictionary<string, ChargingCurve> ChargingCurves = new() { };

        internal int SignOnTime = 0;
        internal int SignOffTime = 0;
        internal bool BreakAllowed = false;
        internal bool HandoverAllowed = false;
        internal int BrutoNetto = 0; // Difference in time between crew member downtime and actual counted break time

        internal bool IsDepot = false;

        internal double ChargePowerPerSpot
        {
            get
            {
                return ChargeSpots > 0 ? ChargeTotalPower / ChargeSpots : 0;
            }
        }

        internal bool CanCharge
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
