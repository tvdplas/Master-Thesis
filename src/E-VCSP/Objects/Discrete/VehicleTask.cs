using E_VCSP.Solver;
using System.Collections;

namespace E_VCSP.Objects.Discrete
{

    internal class VehicleElement
    {
        internal double DrivingCost = double.MinValue;
        internal double SoCDiff = double.MinValue;
        internal int StartTime = int.MinValue;
        internal int EndTime = int.MinValue;

        // Only filled in once element is part of finalized task.
        internal double? StartSoCInTask;
        internal double? EndSoCInTask;
    }

    internal class VETrip : VehicleElement
    {
        internal Trip Trip;

        public VETrip(Trip trip, VehicleType vt)
        {
            Trip = trip;
            DrivingCost = trip.Distance * Config.M_COST;
            SoCDiff = -trip.Distance * vt.DriveUsage;
            StartTime = trip.StartTime;
            EndTime = trip.EndTime;
        }

        public override string ToString()
        {
            return $"VE Trip {Trip.Id}";
        }
    }
    internal class VEDeadhead : VehicleElement
    {
        internal required Deadhead Deadhead;

        internal int SelectedAction = -1;
        internal int ChargeTime = 0;
        internal double ChargeGained = 0;
        internal double ChargeCost = 0;



        public override string ToString()
        {
            return $"VE DH {Deadhead.DeadheadTemplate.Id}";
        }
    }
    internal class VEIdle : VehicleElement
    {
        internal Location Location;

        public VEIdle(Location location, int startTime, int endTime)
        {
            Location = location;
            StartTime = startTime;
            EndTime = endTime;
            DrivingCost = (endTime - startTime) * Config.IDLE_COST;
            SoCDiff = 0; //todo
        }

        public override string ToString()
        {
            return $"VE Idle {Location.Id}";
        }
    }
    internal class VEDepot : VehicleElement
    {
        internal Location Location;

        public VEDepot(Location location, int startTime, int endTime)
        {
            Location = location;
            StartTime = startTime;
            EndTime = endTime;
            DrivingCost = 0;
            SoCDiff = 0;
        }

        public override string ToString()
        {
            return $"VE Depot {Location.Id}";
        }
    }

    internal class VehicleTask
    {
        internal required VehicleType vehicleType;
        internal List<int> Covers;
        internal List<VehicleElement> Elements;
        internal int Index = -1;
        internal double Cost
        {
            get
            {
                double cost = Elements.Sum(e =>
                {
                    double c = e.DrivingCost;

                    if (e is VEDeadhead ved && ved.ChargeGained > 0)
                        c += ved.ChargeCost;

                    return c;
                });

                if (Elements[^1].EndSoCInTask != null)
                    cost += Math.Min(0, vehicleType.StartCharge - (double)Elements[^1]!.EndSoCInTask!) * vehicleType.Capacity / 100 * Config.KWH_COST;

                return cost;
            }
        }

        internal BitArray ToBitArray(int tripCount)
        {
            BitArray ba = new(tripCount);
            for (int i = 0; i < Covers.Count; i++) ba[Covers[i]] = true;
            return ba;
        }

        internal VehicleTask(List<VehicleElement> elements)
        {
            Elements = elements;
            Covers = [.. elements.Where(e => e is VETrip).Select(e => ((VETrip)e).Trip.Index)];
        }
    }
}
