using E_VCSP.Solver;
using System.Collections;

namespace E_VCSP.Objects.Discrete
{

    public class VehicleElement
    {
        public double DrivingCost = double.MinValue;
        public double SoCDiff = double.MinValue;
        public int StartTime = int.MinValue;
        public int EndTime = int.MinValue;

        // Only filled in once element is part of finalized task.
        public double? StartSoCInTask;
        public double? EndSoCInTask;
    }

    public class VETrip : VehicleElement
    {
        public Trip Trip;

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
    public class VEDeadhead : VehicleElement
    {
        public Deadhead Deadhead;

        public int SelectedAction = -1;
        public int ChargeTime = 0;
        public double SoCGained = 0;
        public double ChargeCost = 0;

        public VEDeadhead(Deadhead dh, int startTime, VehicleType vt)
        {
            Deadhead = dh;
            StartTime = startTime;
            EndTime = startTime + dh.DeadheadTemplate.Duration;
            DrivingCost = dh.BaseDrivingCost;
            SoCDiff = -dh.DeadheadTemplate.Distance * vt.DriveUsage;
        }

        public VEDeadhead(Deadhead dh, int startTime, VehicleType vt, int selectedAction, double startSoC)
        {
            Deadhead = dh;
            StartTime = startTime;
            SelectedAction = selectedAction;

            // Determine charge; only allow fully charging at location
            var ca = dh.ChargingActions[selectedAction];
            EndTime = startTime + ca.DrivingTimeFrom + ca.DrivingTimeTo + ca.TimeAtLocation;
            DrivingCost = ca.DrivingCost;
            ChargeTime = ca.TimeAtLocation;

            var chargeResult = ca.ChargeLocation.ChargingCurves[vt.Index].MaxChargeGained(startSoC - ca.ChargeUsedTo, ca.TimeAtLocation, false);
            SoCDiff = chargeResult.SoCGained - ca.ChargeUsedTo - ca.ChargeUsedFrom;
            SoCGained = chargeResult.SoCGained;
            ChargeCost = chargeResult.Cost;
        }

        public override string ToString()
        {
            return $"VE DH {Deadhead.DeadheadTemplate.Id}";
        }
    }
    public class VEIdle : VehicleElement
    {
        public Location Location;

        public VEIdle(Location location, int startTime, int endTime, VehicleType vt)
        {

            Location = location;
            StartTime = startTime;
            EndTime = endTime;

            bool isGarage = true || location.CanCharge || location.BreakAllowed; // no need to use SoC at garage
            DrivingCost = isGarage ? 0 : (endTime - startTime) * Config.IDLE_COST;
            SoCDiff = isGarage ? 0 : -(endTime - startTime) * vt.IdleUsage;
        }

        public override string ToString()
        {
            return $"VE Idle {Location.Id}";
        }
    }
    public class VEDepot : VehicleElement
    {
        public Location Location;

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
                double cost = Elements.Sum(e =>
                {
                    double c = e.DrivingCost;

                    if (e is VEDeadhead ved && ved.SoCGained > 0)
                        c += ved.ChargeCost;

                    return c;
                });

                if (Elements[^1].EndSoCInTask != null)
                    cost += Math.Min(0, vehicleType.StartCharge - (double)Elements[^1]!.EndSoCInTask!) * vehicleType.Capacity / 100 * Config.KWH_COST;

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
            Covers = [.. elements.Where(e => e is VETrip).Select(e => ((VETrip)e).Trip.Index)];
        }
    }
}
