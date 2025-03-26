namespace E_VCSP.Objects.Discrete
{

    internal class DVehicleElement
    {
        internal int StartTime;
        internal int EndTime;
    }

    internal class DVETrip : DVehicleElement
    {
        internal required DTrip DTrip;
    }
    internal class DVEDeadhead : DVehicleElement
    {
        internal required DDeadhead DDeadhead;
    }
    internal class DVEIdle : DVehicleElement { }

    internal class DVehicleTask
    {
        internal List<string> Covers;
        internal List<DVehicleElement> Elements;
        internal double Cost;

        internal DVehicleTask(List<DVehicleElement> elements)
        {
            Elements = elements;
            Covers = elements.Where(e => e is DVETrip).Select(e => ((DVETrip)e).DTrip.Trip.Id).ToList();
            Cost = 0;
            foreach (DVehicleElement dve in elements)
            {
                if (dve is DVETrip || dve is DVEIdle) continue;
                if (dve is DVEDeadhead dvedh)
                {
                    Cost += dvedh.DDeadhead.Costs;
                }
            }
        }
    }
}
