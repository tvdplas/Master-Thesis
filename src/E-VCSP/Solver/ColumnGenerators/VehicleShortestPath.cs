using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;

namespace E_VCSP.Solver
{
    internal abstract class VehicleShortestPath
    {
        // Information about current instance / model state
        internal GRBModel model;
        internal Instance instance;
        internal VehicleType vehicleType;
        internal List<EVSPNode> nodes = [];
        internal List<List<Arc?>> adjFull = [];
        internal List<List<Arc>> adj = [];

        internal int StartTime = int.MaxValue;
        internal int EndTime = int.MinValue;

        internal Location Depot;

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

        internal abstract (double reducedCost, VehicleTask vehicleTask)? GenerateVehicleTask();
    }
}
