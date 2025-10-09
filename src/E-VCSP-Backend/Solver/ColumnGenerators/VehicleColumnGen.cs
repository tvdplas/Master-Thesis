using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using E_VCSP.Utils;

namespace E_VCSP.Solver {
    public abstract class VehicleColumnGen(VehicleSolutionState vss) {
        // Information about current instance state
        public VehicleSolutionState vss = vss;
        public List<double> tripDualCosts = [];
        public Dictionary<Descriptor, double> blockDualCosts = [];
        public Dictionary<DescriptorHalf, List<double>> blockDualCostsByStart = [];

        public void UpdateDualCosts(
            List<double> newTripDualCosts,
            Dictionary<Descriptor, double> newBlockDualCosts,
            Dictionary<DescriptorHalf, List<double>> newBlockDualCostsByStart
        ) {
            tripDualCosts = newTripDualCosts;
            blockDualCosts = newBlockDualCosts;
            blockDualCostsByStart = newBlockDualCostsByStart;
        }

        public abstract List<(double reducedCost, VehicleTask vehicleTask)> GenerateVehicleTasks();
    }
}
