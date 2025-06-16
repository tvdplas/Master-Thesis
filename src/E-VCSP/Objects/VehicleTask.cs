using E_VCSP.Objects.ParsedData;
using System.Collections;

namespace E_VCSP.Objects
{
    public enum VEType
    {
        /// <summary>
        /// Vehicle traveling a deadhead
        /// </summary>
        Deadhead,
        /// <summary>
        /// Vehicle idle at certain location
        /// </summary>
        Idle,
        /// <summary>
        /// Vehicle driving trip
        /// </summary>
        Trip,
        /// <summary>
        /// Vehicle charging at location
        /// </summary>
        Charge
    }

    /// <summary>
    /// Vehicle element which has not yet been finalized
    /// </summary>
    public class VehicleElement
    {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion

        /// <summary>
        /// Type of element
        /// </summary>
        public VEType Type;
        /// <summary>
        /// Starting time of element
        /// </summary>
        public int StartTime;
        /// <summary>
        /// Ending time of element
        /// </summary>
        public int EndTime;
        /// <summary>
        /// SoC usage throughout this element
        /// </summary>
        public double SoCDiff;
        /// <summary>
        /// Overall operational cost of element
        /// </summary>
        public double Cost;
        /// <summary>
        /// Location of vehicle at start of element
        /// </summary>
        public Location? StartLocation;
        /// <summary>
        /// Location of vehicle at end of deadhead
        /// </summary>
        public Location? EndLocation;

        public required double StartSoCInTask = double.MinValue;
        public required double EndSoCInTask = double.MinValue;

        public VehicleElement()
        {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString()
        {
            return $"[{DEBUG_INDEX.ToString()}]";
        }
    }

    public class VETrip : VehicleElement
    {
        public Trip Trip;

        public VETrip(Trip trip, VehicleType vt) : base()
        {
            Type = VEType.Trip;
            Trip = trip;
            Cost = trip.Distance * Config.VH_M_COST;
            SoCDiff = -trip.Distance * vt.DriveUsage;
            StartTime = trip.StartTime;
            EndTime = trip.EndTime;
            StartLocation = trip.From;
            EndLocation = trip.To;
        }

        public override string ToString()
        {
            return $"{base.ToString()} VE Trip {Trip.Id}";
        }
    }

    public class VEDeadhead : VehicleElement
    {
        public DeadheadTemplate DeadheadTemplate;

        public VEDeadhead(DeadheadTemplate dht, int startTime, int endTime, VehicleType vt) : base()
        {
            Type = VEType.Deadhead;
            StartTime = startTime;
            DeadheadTemplate = dht;
            EndTime = startTime + dht.Duration;
            StartLocation = dht.From;
            EndLocation = dht.To;

            Cost = dht.Distance * Config.VH_M_COST;
            SoCDiff = -(dht.Distance * vt.DriveUsage);
        }

        public override string ToString()
        {
            return $"${base.ToString()} VE DH {DeadheadTemplate.Id}";
        }
    }

    public class VEIdle : VehicleElement
    {
        public VEIdle(Location location, int startTime, int endTime) : base()
        {
            Type = VEType.Idle;
            Cost = 0;
            SoCDiff = 0;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString()
        {
            return $"{base.ToString()} VE Handover {StartLocation!.Id}";
        }
    }

    public class VECharge : VehicleElement
    {
        public VECharge(Location location, int startTime, int endTime, double chargeGained, double cost)
        {
            Type = VEType.Charge;
            Cost = cost;
            SoCDiff = chargeGained;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString()
        {
            return $"{base.ToString()} VE Charge {StartLocation!.Id} ({SoCDiff}%)";
        }
    }



    public class VehicleTask
    {
        public required VehicleType vehicleType;
        public List<int> Covers;
        public List<VehicleElement> Elements;
        public int Index = -1;
        public double Cost
        {
            get
            {
                // Driving / idle / charging costs throughout the day
                double cost = Elements.Sum(e => e.Cost);

                // Costs of overnight recharge
                cost += Math.Min(0, vehicleType.StartSoC - (double)Elements[^1]!.EndSoCInTask!) * vehicleType.Capacity / 100 * Config.KWH_COST;

                return cost;
            }
        }

        public BitArray ToBitArray(int tripCount)
        {
            BitArray ba = new(tripCount);
            for (int i = 0; i < Covers.Count; i++) ba[Covers[i]] = true;
            return ba;
        }

        public VehicleTask(List<VehicleElement> elements)
        {
            Elements = elements;
            Covers = [.. elements.Where(e => e.Type == VEType.Trip).Select(e => ((VETrip)e).Trip.Index)];
        }
    }
}
