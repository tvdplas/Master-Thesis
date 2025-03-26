using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.Discrete;
using Gurobi;
using Microsoft.Msagl.Drawing;

namespace E_VCSP.Solver
{
    internal class EVSPDiscreteCG : Solver
    {
        internal DGraph DGraph;
        internal List<DVehicleTask> DVehicleTasks = new();

        private Instance instance;
        private Dictionary<string, DVehicleTask> varTaskMapping = new();
        private GRBModel model;

        internal EVSPDiscreteCG(Instance instance)
        {
            DGraph = new(instance);

            // Create base set of columns
            foreach (var tripSoCs in DGraph.DTrips)
            {
                bool added = false;
                for (int i = 0; i < tripSoCs.Count && !added; i++)
                {
                    DTrip dt = tripSoCs[i];

                    // Find deadheads from/to depot; 
                    DDeadhead? dhDepotTrip = DGraph.DDeadheads.Find(dh => dh.From == DGraph.DDepotStart && dh.To == dt);
                    DDeadhead? dhTripDepot = DGraph.DDeadheads.Find(dh => dh.From == dt && dh.To == DGraph.DDepotEnd);

                    if (dhDepotTrip == null || dhTripDepot == null) continue;
                    DVehicleTask dvt = new([
                        new DVEDeadhead() { DDeadhead = dhDepotTrip, StartTime = dt.Trip.StartTime - dhDepotTrip.DrivingTimes[0], EndTime = dt.Trip.StartTime },
                        new DVETrip() { DTrip = dt, StartTime = dt.Trip.StartTime, EndTime = dt.Trip.EndTime },
                        new DVEDeadhead() { DDeadhead = dhTripDepot, StartTime = dt.Trip.EndTime, EndTime = dt.Trip.EndTime + dhTripDepot.DrivingTimes[0] },
                    ]);
                    added = true;
                }

                if (!added) throw new InvalidDataException("Could not create initial set of columns covering all trips.");
            }
        }

        internal override Graph GenerateSolutionGraph()
        {
            throw new NotImplementedException();
        }

        internal override bool Solve()
        {
            GRBEnv env = new();
            env.LogToConsole = 1;
            env.LogFile = "evsp_discrete.log";

            model = new(env);
            model.SetCallback(new CustomGRBCallback());

            for (int i = 0; i < DVehicleTasks.Count; i++)
            {
                string name = $"vt_{i}";
                GRBVar v = model.AddVar(0, 1, DVehicleTasks[i].Cost, GRB.BINARY, $"vt_{i}");
                varTaskMapping[name] = DVehicleTasks[i];
            }

            // Add cover constraint for each of the trips
            foreach (Trip t in instance.Trips)
            {
                GRBLinExpr expr = new();
                for (int i = 0; i < DVehicleTasks.Count; i++)
                {
                    if (DVehicleTasks[i].Covers.Contains(t.Id)) expr.AddTerm(1, model.GetVarByName($"vt_{i}"));
                }
                model.AddConstr(expr >= 1, "cover_" + t.Id);
            }

            model.Optimize();
            return true;
        }
    }
}
