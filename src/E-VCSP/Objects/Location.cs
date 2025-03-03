﻿namespace E_VCSP.Objects
{
    internal class Location
    {
        internal required string Id;
        internal double ChargeTotalPower = 0;
        internal int ChargeSpots = 0;

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
