using E_VCSP.Objects.ParsedData;
using System.Collections;
using System.Text.Json.Serialization;

namespace E_VCSP.Objects {
    public enum VEType {
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

    #region Elements
    /// <summary>
    /// Vehicle element which has not yet been finalized
    /// </summary>
    [JsonPolymorphic(TypeDiscriminatorPropertyName = "$type")]
    [JsonDerivedType(typeof(VEIdle), "idle")]
    [JsonDerivedType(typeof(VEDeadhead), "deadhead")]
    [JsonDerivedType(typeof(VETrip), "trip")]
    [JsonDerivedType(typeof(VECharge), "charge")]
    public class VehicleElement {
        #region debug
        [JsonInclude]
        public int DEBUG_INDEX;
        public static int DEBUG_INDEX_COUNTER;
        #endregion

        /// <summary>
        /// Type of element
        /// </summary>
        [JsonInclude]
        public VEType Type;
        /// <summary>
        /// Starting time of element
        /// </summary>
        [JsonInclude]
        public int StartTime;
        /// <summary>
        /// Ending time of element
        /// </summary>
        [JsonInclude]
        public int EndTime;
        /// <summary>
        /// SoC usage throughout this element
        /// </summary>
        [JsonInclude]
        public double SoCDiff;
        /// <summary>
        /// Overall operational cost of element
        /// </summary>
        [JsonInclude]
        public double Cost;
        /// <summary>
        /// Location of vehicle at start of element
        /// </summary>
        [JsonInclude]
        public Location? StartLocation;
        /// <summary>
        /// Location of vehicle at end of deadhead
        [JsonInclude]
        public Location? EndLocation;

        [JsonInclude]
        public required double StartSoCInTask = double.MinValue;
        [JsonInclude]
        public required double EndSoCInTask = double.MinValue;

        /// <summary>
        /// Identifies a element as coming from postprocessing; SoC might be lower than actual
        /// </summary>
        [JsonInclude]
        public bool Postprocessed = false;

        public VehicleElement() {
            DEBUG_INDEX = DEBUG_INDEX_COUNTER++;
        }

        public override string ToString() {
            return $"[{DEBUG_INDEX.ToString()}]";
        }
    }

    public class VETrip : VehicleElement {
        [JsonInclude]
        public Trip Trip;

        [JsonConstructor]
        public VETrip(
            VEType type,
            int startTime,
            int endTime,
            double soCDiff,
            double cost,
            Location? startLocation,
            Location? endLocation,
            double startSoCInTask,
            double endSoCInTask,
            bool postprocessed,
            int dEBUG_INDEX,
            Trip trip
        ) : base() {
            Type = type;
            StartTime = startTime;
            EndTime = endTime;
            SoCDiff = soCDiff;
            Cost = cost;
            StartLocation = startLocation;
            EndLocation = endLocation;
            StartSoCInTask = startSoCInTask;
            EndSoCInTask = endSoCInTask;
            Postprocessed = postprocessed;
            DEBUG_INDEX = dEBUG_INDEX;
            Trip = trip;
        }


        public VETrip(Trip trip, VehicleType vt) : base() {
            Type = VEType.Trip;
            Trip = trip;
            Cost = trip.Distance * Constants.VH_M_COST;
            SoCDiff = -trip.Distance * vt.DriveUsage;
            StartTime = trip.StartTime;
            EndTime = trip.EndTime;
            StartLocation = trip.StartLocation;
            EndLocation = trip.EndLocation;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Trip {Trip.Id}";
        }
    }

    public class VEDeadhead : VehicleElement {
        [JsonInclude]
        public DeadheadTemplate DeadheadTemplate;

        [JsonConstructor]
        public VEDeadhead(
            VEType type,
            int startTime,
            int endTime,
            double soCDiff,
            double cost,
            Location? startLocation,
            Location? endLocation,
            double startSoCInTask,
            double endSoCInTask,
            bool postprocessed,
            int dEBUG_INDEX,
            DeadheadTemplate deadheadTemplate
        ) : base() {
            Type = type;
            StartTime = startTime;
            EndTime = endTime;
            SoCDiff = soCDiff;
            Cost = cost;
            StartLocation = startLocation;
            EndLocation = endLocation;
            StartSoCInTask = startSoCInTask;
            EndSoCInTask = endSoCInTask;
            Postprocessed = postprocessed;
            DEBUG_INDEX = dEBUG_INDEX;
            DeadheadTemplate = deadheadTemplate;
        }

        public VEDeadhead(DeadheadTemplate dht, int startTime, int endTime, VehicleType vt) : base() {
            Type = VEType.Deadhead;
            StartTime = startTime;
            DeadheadTemplate = dht;
            EndTime = startTime + dht.Duration;
            StartLocation = dht.StartLocation;
            EndLocation = dht.EndLocation;

            Cost = dht.Distance * Constants.VH_M_COST;
            SoCDiff = -(dht.Distance * vt.DriveUsage);
        }

        public override string ToString() {
            return $"${base.ToString()} VE DH {DeadheadTemplate.Id}";
        }
    }

    public class VEIdle : VehicleElement {
        [JsonConstructor]
        public VEIdle(
            VEType type,
            int startTime,
            int endTime,
            double soCDiff,
            double cost,
            Location? startLocation,
            Location? endLocation,
            double startSoCInTask,
            double endSoCInTask,
            bool postprocessed,
            int dEBUG_INDEX
        ) : base() {
            Type = type;
            StartTime = startTime;
            EndTime = endTime;
            SoCDiff = soCDiff;
            Cost = cost;
            StartLocation = startLocation;
            EndLocation = endLocation;
            StartSoCInTask = startSoCInTask;
            EndSoCInTask = endSoCInTask;
            Postprocessed = postprocessed;
            DEBUG_INDEX = dEBUG_INDEX;
        }

        public VEIdle(Location location, int startTime, int endTime) : base() {
            Type = VEType.Idle;
            Cost = 0;
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

    public class VECharge : VehicleElement {
        [JsonConstructor]
        public VECharge(
            VEType type,
            int startTime,
            int endTime,
            double soCDiff,
            double cost,
            Location? startLocation,
            Location? endLocation,
            double startSoCInTask,
            double endSoCInTask,
            bool postprocessed,
            int dEBUG_INDEX
        ) : base() {
            Type = type;
            StartTime = startTime;
            EndTime = endTime;
            SoCDiff = soCDiff;
            Cost = cost;
            StartLocation = startLocation;
            EndLocation = endLocation;
            StartSoCInTask = startSoCInTask;
            EndSoCInTask = endSoCInTask;
            Postprocessed = postprocessed;
            DEBUG_INDEX = dEBUG_INDEX;
        }
        public VECharge(Location location, int startTime, int endTime, double chargeGained, double cost) {
            Type = VEType.Charge;
            Cost = cost;
            SoCDiff = chargeGained;
            StartTime = startTime;
            EndTime = endTime;
            StartLocation = location;
            EndLocation = location;
        }

        public override string ToString() {
            return $"{base.ToString()} VE Charge {StartLocation!.Id} ({SoCDiff}%)";
        }
    }
    #endregion

    public class VehicleTask {
        /// <summary>
        /// Task is part of initial covering of trips
        /// </summary>
        public bool IsUnit = false;
        /// <summary>
        /// Column generation source which produced this column
        /// </summary>
        [JsonInclude]
        public required string Source;
        /// <summary>
        /// Vehicle type used in task
        /// </summary>
        [JsonInclude]
        public required VehicleType vehicleType;
        /// <summary>
        /// Trip indexes covered by the task
        /// </summary>
        [JsonInclude]
        public List<int> TripIndexCover;
        /// <summary>
        /// Block indexes covered by the task
        /// </summary>
        [JsonInclude]
        public List<int> BlockIndexCover;
        /// <summary>
        /// Block descriptors covered by task 
        /// </summary>
        public List<string> BlockDescriptorCover = [];
        /// <summary>
        /// Elements used in task
        /// </summary>
        [JsonInclude]
        public List<VehicleElement> Elements;
        /// <summary>
        /// Index of task in solution state list
        /// </summary>
        [JsonInclude]
        public int Index = -1;

        private double cachedCost = -1;
        /// <summary>
        /// Reset cost calculation; only use when manually modifying elements
        /// </summary>
        public void InvalidateCostCache() => cachedCost = -1;
        /// <summary>
        /// Cost of the task; includes driving, SoC and fixed costs
        /// </summary>
        public double Cost {
            get {
                if (cachedCost != -1) return cachedCost;

                // Driving / idle / charging costs throughout the day
                double cost = Elements.Sum(e => e.Cost);

                // Costs of overnight recharge
                cost += Math.Max(0, vehicleType.StartSoC - (double)Elements[^1]!.EndSoCInTask!) * vehicleType.Capacity / 100 * Constants.KWH_COST;

                // Cost of just using vehicle
                cost += Config.VH_PULLOUT_COST;

                cachedCost = cost;
                return cost;
            }
        }

        /// <summary>
        /// Returns a bitmask b where b[i] == 1 iff trip i is covered by this task
        /// </summary>
        /// <param name="tripCount">Total number of available trips</param>
        public BitArray ToTripBitArray(int tripCount) {
            BitArray ba = new(tripCount);
            foreach (int i in TripIndexCover) ba[i] = true;
            return ba;
        }

        /// <summary>
        /// Returns a bitmask b where b[i] == 1 iff block i is covered by this task
        /// </summary>
        /// <param name="tripCount">Total number of available blocks</param>
        public BitArray ToBlockBitArray(int blockCount) {
            BitArray ba = new(blockCount);
            foreach (int i in BlockIndexCover) ba[i] = true;
            return ba;
        }

        public VehicleTask(List<VehicleElement> elements) {
            Elements = elements;
            TripIndexCover = [];
            BlockIndexCover = [];
            RecalcTripCovers();
        }

        public void RecalcTripCovers() {
            TripIndexCover = [.. Elements.Where(e => e.Type == VEType.Trip).Select(e => ((VETrip)e).Trip.Index)];
            InvalidateCostCache();
        }
    }
}
