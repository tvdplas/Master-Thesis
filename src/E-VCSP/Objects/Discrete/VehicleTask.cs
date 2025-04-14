using E_VCSP.Solver;

namespace E_VCSP.Objects.Discrete
{

    internal class VehicleElement
    {
        internal required double Cost;
        internal required int StartTime;
        internal required int EndTime;
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
        internal required LabelDeadhead Deadhead;

        public override string ToString()
        {
            return $"VE DH {Deadhead.DeadheadTemplate.Id}";
        }
    }
    internal class VEIdle : VehicleElement
    {
        public override string ToString()
        {
            return $"VE Idle";
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
        internal double Cost
        {
            get
            {
                return Elements.Sum(e => e.Cost);
            }
        }

        internal VehicleTask(List<VehicleElement> elements)
        {
            Elements = elements;
            Covers = elements.Where(e => e is VETrip).Select(e => ((VETrip)e).Trip.Index).ToList();
        }
    }
}
