using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using Gurobi;

namespace E_VCSP.Solver {
    public abstract class VehicleColumnGen(GRBModel model, VehicleSolutionState vss) {
        // Information about current instance / model state
        public GRBModel model = model;
        public VehicleSolutionState vss = vss;

        public abstract List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks();
    }
}
