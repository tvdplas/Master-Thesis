using E_VCSP.Objects.ParsedData;

namespace E_VCSP.Objects {
    public enum PVEType {
        /// <summary>
        /// Vehicle travel + idle in given time. Not considered a stop.
        /// </summary>
        Travel,
        /// <summary>
        /// Depot at start / end of day. Used as dummy.
        /// </summary>
        Depot,
        /// <summary>
        /// Vehicle visiting a certain location in order to facilitate handover
        /// </summary>
        HandoverDetour,
        /// <summary>
        /// Vehicle driving trip
        /// </summary>
        Trip,
        /// <summary>
        /// Vehicle charging at certain location
        /// </summary>
        ChargeDetour
    }

    /// <summary>
    /// Vehicle element which has not yet been finalized
    /// </summary>
    public class PartialVehicleElement {
        #region debug
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion

        /// <summary>
        /// Type of element
        /// </summary>
        public PVEType Type;
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
        /// Cost of operating this element. Does not include charging
        /// </summary>
        public double DrivingCost;
        /// <summary>
        /// Location of vehicle at start of element
        /// </summary>
        public Location StartLocation;
        /// <summary>
        /// Location of vehicle at end of deadhead
        /// </summary>
        public Location EndLocation;

        public PartialVehicleElement() {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString() {
            return $"[{DEBUG_INDEX.ToString()}]";
        }
    }

    public class PVETrip : PartialVehicleElement {
        public Trip Trip;

        public PVETrip(Trip trip, VehicleType vt) : base() {
            Type = PVEType.Trip;
            Trip = trip;
            DrivingCost = trip.Distance * Config.VH_M_COST;
            SoCDiff = -trip.Distance * vt.DriveUsage;
            StartTime = trip.StartTime;
            EndTime = trip.EndTime;
            StartLocation = trip.From;
            EndLocation = trip.To;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Trip {Trip.Id}";
        }
    }

    public class PVETravel : PartialVehicleElement {
        public DeadheadTemplate DeadheadTemplate;

        /// <summary>
        /// Determines whether the idle period is at the beginning or end of the travel;
        /// tries to minimize block length
        /// </summary>
        public bool IdleAtStart {
            get {
                bool fromHandover = DeadheadTemplate.From.HandoverAllowed;
                bool toHandover = DeadheadTemplate.To.HandoverAllowed;
                return fromHandover || !fromHandover && !toHandover;
            }
        }
        public int IdleTime => EndTime - StartTime - DeadheadTemplate.Duration;

        /// <summary>
        /// Time at which vehicle arrives after driving deadhead
        /// </summary>
        public int ArrivalTime => IdleAtStart ? EndTime : EndTime - IdleTime;
        /// <summary>
        /// Time at which vehicle departs in order to drive deadhead
        /// </summary>
        public int DepartureTime => IdleAtStart ? StartTime + IdleTime : StartTime;

        public PVETravel(DeadheadTemplate dht, int startTime, int endTime, VehicleType vt) : base() {
            Type = PVEType.Travel;
            StartTime = startTime;
            DeadheadTemplate = dht;
            EndTime = endTime;
            StartLocation = dht.From;
            EndLocation = dht.To;

            Location idleLocation = IdleAtStart ? DeadheadTemplate.From : DeadheadTemplate.To;
            bool idleAtGarage = idleLocation.BreakAllowed || idleLocation.IsDepot || idleLocation.CanCharge;
            double idleCost = idleAtGarage ? 0 : Config.VH_IDLE_COST * IdleTime;
            double idleSoCDiff = idleAtGarage ? 0 : -vt.IdleUsage * IdleTime;

            DrivingCost = dht.Distance * Config.VH_M_COST + idleCost;
            SoCDiff = -(dht.Distance * vt.DriveUsage) + idleSoCDiff;
        }

        public override string ToString() {
            return $"{base.ToString()} VE DH {DeadheadTemplate.Id}";
        }
    }

    public class PVEHandover : PartialVehicleElement {
        public PVEHandover(Location location, int startTime, int endTime) : base() {
            if (!location.HandoverAllowed)
                throw new InvalidOperationException("Handover visit planned at location without handover possibility");

            Type = PVEType.HandoverDetour;
            DrivingCost = 0;
            SoCDiff = 0;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Handover {StartLocation!.Id}";
        }
    }

    public class PVEDepot : PartialVehicleElement {
        public PVEDepot(Location location, int startTime, int endTime) : base() {
            Type = PVEType.Depot;
            DrivingCost = 0;
            SoCDiff = 0;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Depot {StartLocation!.Id}";
        }
    }

    public class PVECharge : PartialVehicleElement {
        public PVECharge(Location location, int startTime, int endTime) {
            Type = PVEType.ChargeDetour;
            DrivingCost = 0;
            SoCDiff = 0;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Charge {StartLocation!.Id} ({SoCDiff}%)";
        }
    }
}
