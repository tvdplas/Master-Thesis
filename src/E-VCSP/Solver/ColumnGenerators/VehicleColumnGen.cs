using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;

namespace E_VCSP.Solver {
    public abstract class VehicleColumnGen(VehicleSolutionState vss) {
        // Information about current instance state
        public VehicleSolutionState vss = vss;
        internal List<double> tripDualCosts = [];
        internal Dictionary<string, double> blockDualCosts = [];
        internal Dictionary<string, List<double>> blockDualCostsByStart = [];

        public void updateDualCosts(
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
