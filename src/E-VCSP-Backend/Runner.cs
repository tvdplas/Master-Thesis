using E_VCSP;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver;
using E_VCSP.Solver.SolutionState;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Reflection;

namespace E_VCSP_Backend {
    [DynamicallyAccessedMembers(DynamicallyAccessedMemberTypes.NonPublicMethods)]
    public class Runner
    {
        public string ActiveFolder = "No folder selected";

        public Instance? Instance;
        public VehicleSolutionState? vss;
        public CrewSolutionState? css;

        public EVSPCG? VSPSolver;
        public CSPCG? CSPSolver;
        public EVCSPCGLagrange? IntegratedSolver;

        public Dictionary<string, Action> Experiments;

        public Runner()
        {
            Experiments = [];

            // Add all methods starting with "exp" to the Experiments list
            var expMethods = this.GetType()
                .GetMethods(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
                .Where(m => m.Name.StartsWith("exp") && m.GetParameters().Length == 0);

            foreach (var method in expMethods)
            {
                // Create a delegate for each method and add to the list
                Experiments.Add(method.Name, (Action)Delegate.CreateDelegate(typeof(Action), this, method));
            }
        }

        public void Reload(string runDescription = "No Description")
        {
            if (ActiveFolder == "No folder selected") return;

            Instance = new(ActiveFolder);
            vss = new(Instance, Instance.VehicleTypes[0]);
            css = new(Instance, []);

            VSPSolver = new(vss);
            CSPSolver = new(css);
            IntegratedSolver = new EVCSPCGLagrange(vss, css);

            string timestamp = DateTime.Now.ToString("yy-MM-dd-HH.mm.ss");
            Constants.RUN_LOG_FOLDER = $"./runs/{timestamp} {runDescription}/";
            Directory.CreateDirectory(Constants.RUN_LOG_FOLDER);


            Console.WriteLine($"Instance reloaded. Current config state dumped to {Constants.RUN_LOG_FOLDER + "config.txt"}");
        }

        public bool VSPFromInstance()
        {
            Reload("VSP From Instance");
            if (VSPSolver == null) return false;

            bool success = VSPSolver.Solve();

            if (success) Console.WriteLine("Solver finished successfully.");
            else Console.WriteLine("Solver did not find a solution or was cancelled.");

            return success;
        }

        public bool CSPFromInstance()
        {
            if (Instance == null || vss == null || vss.SelectedTasks.Count == 0) return false;

            if (css == null || css.Blocks.Count == 0 || vss.SelectedTasks.Count > 0)
            {
                Console.WriteLine("Solving based on VSP solution");
                css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
            }
            else
            {
                Console.WriteLine("Resetting existing CSP solution");
                css.ResetFromBlocks();
            }
            Console.WriteLine($"Total of {css.Blocks.Count} blocks being considered");
            CSPSolver = new CSPCG(css);

            bool success = CSPSolver.Solve();

            if (success) Console.WriteLine("Solution found");
            else Console.WriteLine("No solution/cancelled.");
            return success;
        }

        public bool VCSPFromInstance()
        {
            Reload("Integrated From Instance");
            if (IntegratedSolver == null) return false;

            bool success = IntegratedSolver.Solve();
            if (success) Console.WriteLine("Solver finished successfully.");
            else Console.WriteLine("Solver did not find a solution or was cancelled.");
            return success;
        }

        #region Experiments

        void expVSPSecondaryColumns()
        {
            const int attempts = 32;
            const int subdivisions = 16;

            Reload("VSP Secondary Columns");
            if (vss == null || VSPSolver == null) return;

            Config.VH_OVER_MAX_COST = 0;
            Config.VSP_SOLVER_TIMEOUT_SEC = 900;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# attempts;# subdivs;value;#unique cols;mipgap;runtime");


            for (int i = 0; i <= attempts; i = Math.Max(1, i * 2))
            {
                Config.VSP_LB_SEC_COL_ATTEMPTS = i;
                for (int j = 4; j <= subdivisions; j += 4)
                {
                    Config.VSP_LB_SEC_COL_COUNT = i;

                    vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                    VSPSolver = new EVSPCG(vss);
                    bool success = VSPSolver.Solve();
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{j};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime}");
                }
            }
        }

        void expVSPTargetVehicles()
        {
            int minMaxVehicles = 8;
            int maxMaxVehicles = 17;

            Reload("VSP Target Vehicles");
            if (vss == null || VSPSolver == null) return;

            Config.VSP_SOLVER_TIMEOUT_SEC = 900;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}max vh;value;cols;mipgap;runtime");

            for (int i = minMaxVehicles; i <= maxMaxVehicles; i++)
            {
                Config.MAX_VEHICLES = i;

                vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                VSPSolver = new EVSPCG(vss);

                bool success = VSPSolver.Solve();
                Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime}");
            }
        }

        void expCSPSecondaryColumns()
        {
            Reload("CSP Secondary Columns");
            // Get vsp solution
            Config.VSP_SOLVER_TIMEOUT_SEC = 900;
            Config.VSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.VSP_LB_SEC_COL_COUNT = 4;
            vss = new(Instance!, Instance!.VehicleTypes[0]);
            VSPSolver = new EVSPCG(vss);
            bool success = VSPSolver.Solve();

            const int attempts = 16;
            const int subdivisions = 16;

            for (int i = 0; i <= attempts; i = Math.Max(1, i * 2))
            {
                Config.CSP_LB_SEC_COL_ATTEMPTS = i;
                for (int j = 4; j <= subdivisions; j += 4)
                {
                    Config.CSP_LB_SEC_COL_COUNT = i;
                    css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
                    CSPSolver = new CSPCG(css);
                    success = CSPSolver.Solve();
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{j};{css.Costs()};{css.Duties.Count};{CSPSolver.model!.MIPGap};{CSPSolver.model.Runtime}");
                }
            }
        }

        #endregion
    }
}
