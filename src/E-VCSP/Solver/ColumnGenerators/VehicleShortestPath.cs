using E_VCSP.Objects;
using Gurobi;

namespace E_VCSP.Solver
{
    public abstract class VehicleShortestPath
    {
        // Information about current instance / model state
        public GRBModel model;
        public Instance instance;
        public VehicleType vehicleType;
        public List<EVSPNode> nodes = [];
        public List<List<Arc?>> adjFull = [];
        public List<List<Arc>> adj = [];

        public int StartTime = int.MaxValue;
        public int EndTime = int.MinValue;

        public Location Depot;

        protected VehicleShortestPath(GRBModel model, Instance instance, VehicleType vehicleType, List<EVSPNode> nodes, List<List<Arc?>> adjFull, List<List<Arc>> adj)
        {
            this.model = model;
            this.instance = instance;
            this.vehicleType = vehicleType;
            this.nodes = nodes;
            this.adjFull = adjFull;
            this.adj = adj;

            foreach (Trip t in instance.Trips)
            {
                // Depot to trip
                int minDepTime = t.StartTime - adjFull[instance.DepotStartIndex][t.Index]!.Deadhead.DeadheadTemplate.Duration;
                StartTime = Math.Min(minDepTime, StartTime);

                // Trip to depot
                int maxArrTime = t.EndTime + adjFull[t.Index][instance.DepotEndIndex]!.Deadhead.DeadheadTemplate.Duration;
                EndTime = Math.Max(maxArrTime, EndTime);
            }

            Depot = instance.Locations.Find(x => x.IsDepot) ?? throw new InvalidDataException("No depot found");
        }

        public abstract List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks();
    }
}
