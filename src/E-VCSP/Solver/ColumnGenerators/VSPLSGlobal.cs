using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver.ColumnGenerators
{
    internal class VSPLSGlobal : VehicleShortestPath
    {
        public VSPLSGlobal(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj) : base(model, instance, vehicleType, nodes, adjFull, adj)
        {
        }

        private List<LLNode> routes = new();


        private void reset()
        {
            routes = new();

            // Generate initial set of routes
            for (int i = 0; i < instance.Trips.Count; i++)
            {
                Trip t = instance.Trips[i];
                if (Depot == null) throw new InvalidDataException("No depot found when constructing depot vehicleelements in LS ShortestPath");
                Arc depotTripArc = adjFull[instance.DepotStartIndex][i] ?? throw new InvalidDataException("No depot trip arc found in initial LSGLobal");
                Arc tripDepotArc = adjFull[i][instance.DepotEndIndex] ?? throw new InvalidDataException("No trip depot arc found in initial LSGLobal");

                // Create depot -> dh1 -> idle1 -> trip -> dh2 -> idle2 -> depot
                LLNode depotStart = new()
                {
                    NodeType = LLNodeType.Depot,
                    VehicleElement = new VEDepot(Depot, StartTime - Config.MIN_NODE_TIME, StartTime),
                    SoCAtStart = vehicleType.StartCharge,
                    SoCAtEnd = vehicleType.StartCharge,
                };

                // Depot to depot deadhead
                LLNode dh1 = depotStart.AddAfter(LLNodeType.Deadhead, new VEDeadhead()
                {
                    Deadhead = depotTripArc.Deadhead,
                    StartTime = StartTime,
                    EndTime = StartTime + depotTripArc.Deadhead.DeadheadTemplate.Duration,
                    DrivingCost = depotTripArc.Deadhead.BaseDrivingCost,
                    SoCDiff = -depotTripArc.Deadhead.DeadheadTemplate.Distance * vehicleType.DriveUsage
                });

                LLNode idle1 = dh1.AddAfter(LLNodeType.Idle, new VEIdle(t.From, dh1.VehicleElement.EndTime, t.StartTime));
                LLNode trip = idle1.AddAfter(LLNodeType.Trip, new VETrip(t, vehicleType));


                // Idle at ending depot
                int idleTime = EndTime - depotDepotDH.VehicleElement.EndTime;
                LLNode idle = depotDepotDH.AddAfter(LLNodeType.Idle, new VEIdle()
                {
                    Location = Depot,
                    StartTime = depotDepotDH.VehicleElement.EndTime,
                    EndTime = EndTime,
                    DrivingCost = idleTime * Config.IDLE_COST,
                    SoCDiff = 0,
                });

                // Depot end; 
                LLNode depotEnd = idle.AddAfter(LLNodeType.Depot, new VEDepot()
                {
                    StartTime = EndTime,
                    EndTime = EndTime + Config.MIN_NODE_TIME,
                    Location = Depot,
                    DrivingCost = 0,
                    SoCDiff = 0,
                });
            }
        }

        internal override (double reducedCost, VehicleTask vehicleTask)? GenerateVehicleTask()
        {
            return (0, new([]) { vehicleType = vehicleType });
        }
    }
}
