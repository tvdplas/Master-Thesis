using E_VCSP.Solver;

namespace E_VCSP.Objects.Discrete
{

    internal class VehicleElement
    {
        internal int StartTime;
        internal int EndTime;
    }

    internal class VETrip : VehicleElement
    {
        internal required Trip Trip;
    }
    internal class VEDeadhead : VehicleElement
    {
        internal required LabelDeadhead Deadhead;
    }
    internal class VEIdle : VehicleElement { }

    internal class VehicleTask
    {
        internal List<int> Covers;
        internal List<VehicleElement> Elements;
        internal double Cost
        {
            get; private set;
        }

        internal VehicleTask(List<VehicleElement> elements)
        {
            Elements = elements;
            Covers = elements.Where(e => e is VETrip).Select(e => ((VETrip)e).Trip.Index).ToList();
            Cost = 0;
            foreach (var element in elements)
            {
                if (element is VETrip trip)
                {
                    Cost += trip.Trip.Distance * Config.M_COST;
                }
                if (element is VEDeadhead deadhead)
                {
                    Cost += deadhead.Deadhead.BaseCost;
                }
            }
        }
    }
}
