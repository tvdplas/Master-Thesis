using System.Text.Json.Serialization;

namespace E_VCSP.Objects.ParsedData
{
    public class Location
    {
        [JsonInclude]
        public required string Id;
        [JsonIgnore]
        public int Index = -1;
        [JsonIgnore]
        public double ChargeTotalPower = 0; // KWh
        [JsonIgnore]
        public int ChargeSpots = 0;
        [JsonIgnore]
        public List<ChargingCurve> ChargingCurves = new(); // indexed by vehicle id
        [JsonIgnore]
        public int SignOnTime = 0;
        [JsonIgnore]
        public int SignOffTime = 0;
        [JsonIgnore]
        public bool BreakAllowed = false;
        [JsonIgnore]
        public bool HandoverAllowed = false;
        [JsonIgnore]
        public int BrutoNetto = 0; // Difference in time between crew member downtime and actual counted break time

        [JsonIgnore]
        public bool IsDepot = false;

        [JsonIgnore]
        public bool CrewHub => true || SignOnTime > 0 || SignOffTime > 0 || CanCharge;

        [JsonIgnore]
        // TODO: gaat dit wel goed?
        public int MinHandoverTime => Math.Max(SignOffTime, SignOnTime);
        [JsonIgnore]
        public bool FreeIdle => BreakAllowed || CanCharge;
        [JsonIgnore]
        public double ChargePowerPerSpot => ChargeSpots > 0 ? ChargeTotalPower / ChargeSpots : 0;
        [JsonIgnore]
        public bool CanCharge => ChargePowerPerSpot > 0;

        public override string ToString()
        {
            return Id;
        }
    }
}
