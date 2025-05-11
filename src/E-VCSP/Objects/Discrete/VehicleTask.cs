using E_VCSP.Solver;
using System.Collections;

namespace E_VCSP.Objects.Discrete
{

    internal class VehicleElement
    {
        internal required double DrivingCost;
        internal required double SoCDiff;
        internal required int StartTime;
        internal required int EndTime;

        // Only filled in once element is part of finalized task.
        internal double? StartSoCInTask;
        internal double? EndSoCInTask;
    }

    internal class VETrip : VehicleElement
    {
        internal required Trip Trip;

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
        internal required Location Location;

        public override string ToString()
        {
            return $"VE Idle {Location.Id}";
        }
    }
    internal class VEDepot : VehicleElement
    {
        internal required Location Location;

        public override string ToString()
        {
            return $"VE Depot {Location.Id}";
        }
    }

    internal class VehicleTask
    {
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
                    cost += Math.Min(0, ShortestPathLS.vehicleType.StartCharge - (double)Elements[^1]!.EndSoCInTask!) * ShortestPathLS.vehicleType.Capacity / 100 * Config.KWH_COST;

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
