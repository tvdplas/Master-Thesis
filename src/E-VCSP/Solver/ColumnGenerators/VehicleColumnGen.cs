using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;

namespace E_VCSP.Solver {
    public abstract class VehicleColumnGen(VehicleSolutionState vss) {
        // Information about current instance state
        public VehicleSolutionState vss = vss;
        public List<double> tripDualCosts = [];
        public Dictionary<string, double> blockDualCosts = [];
        public Dictionary<string, List<double>> blockDualCostsByStart = [];

        public void UpdateDualCosts(
            List<double> newTripDualCosts,
            Dictionary<string, double> newBlockDualCosts,
            Dictionary<string, List<double>> newBlockDualCostsByStart
        ) {
            tripDualCosts = newTripDualCosts;
            blockDualCosts = newBlockDualCosts;
            blockDualCostsByStart = newBlockDualCostsByStart;
        }

        public abstract List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks();
    }
}
