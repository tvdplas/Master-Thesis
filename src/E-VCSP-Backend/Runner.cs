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
    public class Runner {
        public string ActiveFolder = "No folder selected";

        public Instance? Instance;
        public VehicleSolutionState? vss;
        public CrewSolutionState? css;

        public EVSPCG? VSPSolver;
        public CSPCG? CSPSolver;
        public EVCSPCGLagrange? IntegratedSolver;

        public Dictionary<string, Action> Experiments;

        public Runner() {
            Experiments = [];

            // Add all methods starting with "exp" to the Experiments list
            var expMethods = this.GetType()
                .GetMethods(BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
                .Where(m => m.Name.StartsWith("exp") && m.GetParameters().Length == 0);

            foreach (var method in expMethods) {
                // Create a delegate for each method and add to the list
                Experiments.Add(method.Name, (Action)Delegate.CreateDelegate(typeof(Action), this, method));
            }
        }

        public void Reload(string runDescription = "No Description") {
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
            Config.Dump();
            Console.WriteLine($"Instance reloaded. Current config state dumped to {Constants.RUN_LOG_FOLDER + "config.txt"}");
        }

        public bool VSPFromInstance() {
            Reload("VSP From Instance");
            if (VSPSolver == null) return false;

            bool success = VSPSolver.Solve();

            if (success) Console.WriteLine("Solver finished successfully.");
            else Console.WriteLine("Solver did not find a solution or was cancelled.");

            return success;
        }

        public bool CSPFromInstance() {
            if (Instance == null || vss == null || vss.SelectedTasks.Count == 0) return false;

            if (css == null || css.Blocks.Count == 0 || vss.SelectedTasks.Count > 0) {
                Console.WriteLine("Solving based on VSP solution");
                css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
            }
            else {
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

        public bool VCSPFromInstance() {
            Reload("Integrated From Instance");
            if (IntegratedSolver == null) return false;

            bool success = IntegratedSolver.Solve();
            if (success) Console.WriteLine("Solver finished successfully.");
            else Console.WriteLine("Solver did not find a solution or was cancelled.");
            return success;
        }

        #region Experiments

        void expVSPSecondaryColumns() {
            const int attempts = 32;
            const int subdivisions = 16;

            Reload("VSP Secondary Columns");
            if (vss == null || VSPSolver == null) return;

            Config.VH_OVER_MAX_COST = 0;
            Config.VSP_SOLVER_TIMEOUT_SEC = 600;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# attempts;# subdivs;value;#unique cols;mipgap;runtime");


            for (int i = 0; i <= attempts; i = Math.Max(1, i * 2)) {
                Config.VSP_LB_SEC_COL_ATTEMPTS = i;
                for (int j = 4; j <= subdivisions; j += 4) {
                    Config.VSP_LB_SEC_COL_COUNT = i;

                    vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                    VSPSolver = new EVSPCG(vss);
                    bool success = VSPSolver.Solve();
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{j};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime}");
                }
            }
        }

        void expVSPTargetVehicles() {
            int minMaxVehicles = 8;
            int maxMaxVehicles = 17;
            Config.VH_OVER_MAX_COST = 5000;

            Reload("VSP Target Vehicles");
            if (vss == null || VSPSolver == null) return;

            Config.VSP_SOLVER_TIMEOUT_SEC = 900;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}max vh;value;cols;mipgap;runtime");

            for (int i = minMaxVehicles; i <= maxMaxVehicles; i++) {
                Config.MAX_VEHICLES = i;

                vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                VSPSolver = new EVSPCG(vss);

                bool success = VSPSolver.Solve();
                Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime}");
            }
        }

        void expCSPSecondaryColumns() {
            Reload("CSP Secondary Columns");
            Console.WriteLine($"{Config.CNSL_OVERRIDE}# attempts;# subdivs;value;#unique cols;mipgap;mip runtime;total runtime");

            // Get vsp solution
            Config.VSP_SOLVER_TIMEOUT_SEC = 900;
            Config.VSP_LB_SEC_COL_ATTEMPTS = 4;
            Config.VSP_LB_SEC_COL_COUNT = 4;
            // Unfair testing for CSP
            Config.CR_SINGLE_SHIFT_COST = 10_000;
            Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
            Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;

            vss = new(Instance!, Instance!.VehicleTypes[0]);
            VSPSolver = new EVSPCG(vss);
            bool success = VSPSolver.Solve();

            const int attempts = 32;
            const int subdivisions = 16;

            for (int i = 0; i <= attempts; i = Math.Max(1, i * 2)) {
                Config.CSP_LB_SEC_COL_ATTEMPTS = i;
                for (int j = 4; j <= subdivisions; j += 4) {
                    Config.CSP_LB_SEC_COL_COUNT = i;
                    css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
                    CSPSolver = new CSPCG(css);
                    Stopwatch sw = Stopwatch.StartNew();
                    success = CSPSolver.Solve();
                    sw.Stop();
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{j};{css.Costs()};{css.Duties.Count};{CSPSolver.model!.MIPGap};{CSPSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
                }
            }
        }

        void expVSPGlobalLS() {
            Reload("VSP Global LS");

            Config.VSP_SOLVER_HEURISTIC_FRAC = 0.25;
            Config.VSP_SOLVER_TIMEOUT_SEC = 300;
            Config.VSP_LB_WEIGHT = 0;
            Config.VSP_LS_G_WEIGHT = 1;
            Config.VSP_INSTANCES_PER_IT = 20;

            int attempts = 5;
            List<int> rounds = [100 / Config.VSP_INSTANCES_PER_IT, 200 / Config.VSP_INSTANCES_PER_IT, 500 / Config.VSP_INSTANCES_PER_IT, 1000 / Config.VSP_INSTANCES_PER_IT];
            List<int> maxIts = [50_000, 100_000, 500_000, 1_000_000, 5_000_000];

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# rounds;# its;best direct;best ilp;#unique cols;mipgap;ilp runtime;total runtime");

            foreach (var r in rounds) {
                foreach (var i in maxIts) {
                    for (int x = 0; x < attempts; x++) {
                        Config.VSP_MAX_COL_GEN_ITS = r;
                        Config.VSP_OPT_IT_THRESHOLD = r;
                        Config.VSP_LS_G_ITERATIONS = i;

                        vss = new(vss!.Instance, vss.Instance.VehicleTypes[0]);
                        VSPSolver = new EVSPCG(vss);
                        Stopwatch sw = Stopwatch.StartNew();
                        bool success = VSPSolver.Solve();
                        sw.Stop();
                        Console.WriteLine($"{Config.CNSL_OVERRIDE}{r};{i};{VSPSolver.bestKnownSolutionValue};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
                    }
                }
            }
        }

        void expVSPSingleLS() {
            Reload("VSP Single LS");

            Config.VSP_SOLVER_HEURISTIC_FRAC = 0.25;
            Config.VSP_SOLVER_TIMEOUT_SEC = 300;
            Config.VSP_LB_WEIGHT = 0;
            Config.VSP_LS_S_WEIGHT = 1;
            Config.VSP_INSTANCES_PER_IT = 1;
            Config.VSP_LS_S_NUM_COLS = 16;
            Config.VH_OVER_MAX_COST = 0;

            int attempts = 5;
            List<int> rounds = [100, 200, 500, 1000, 2000, 5000];
            List<int> maxIts = [1_000, 10_000, 50_000, 100_000, 500_000];

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# rounds;# its;best ilp;#unique cols;mipgap;ilp runtime;total runtime");

            foreach (var r in rounds) {
                foreach (var i in maxIts) {
                    for (int x = 0; x < attempts; x++) {
                        Config.VSP_MAX_COL_GEN_ITS = r;
                        Config.VSP_OPT_IT_THRESHOLD = r;
                        Config.VSP_LS_S_ITERATIONS = i;

                        vss = new(vss!.Instance, vss.Instance.VehicleTypes[0]);
                        VSPSolver = new EVSPCG(vss);
                        Stopwatch sw = Stopwatch.StartNew();
                        bool success = VSPSolver.Solve();
                        sw.Stop();
                        Console.WriteLine($"{Config.CNSL_OVERRIDE}{r};{i};{vss.Costs()};{vss.Tasks.Count};{VSPSolver.model!.MIPGap};{VSPSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
                    }
                }
            }
        }

        void expVCSPRounds() {
            Reload("VCSP Rounds");

            Config.VSP_SOLVER_TIMEOUT_SEC = 600;
            Config.CSP_SOLVER_TIMEOUT_SEC = 600;
            Config.VCSP_SOLVER_TIMEOUT_SEC = 900;
            Config.VSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.VSP_LB_SEC_COL_COUNT = 8;
            Config.CSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.CSP_LB_SEC_COL_COUNT = 8;
            Config.VCSP_MAX_TASKS_DURING = 100000000;
            Config.VCSP_MAX_DUTIES_DURING = 100000000;
            Config.LAGRANGE_DISRUPT_ROUNDS = 0; // nondeterministic
            Config.LAGRANGE_N = 50;
            Config.LAGRANGE_PI_END = 0.0001;

            // Unfair testing for CSP
            Config.CR_SINGLE_SHIFT_COST = 10_000;
            Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
            Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;

            const int maxRounds = 20;
            Console.WriteLine($"{Config.CNSL_OVERRIDE}# rounds;" +
                $"seq vsp value;seq csp value;int vsp value;int csp value;" +
                $"seq vsp cols;seq csp cols;int vsp cols;int csp cols;" +
                $"seq vsp selected cols;seq csp selected cols;int vsp selected cols;int csp selected cols;" +
                $"mipgap;mip runtime;total runtime");

            // Initialize a cover that we will reuse for all experiments
            vss = new(Instance!, Instance!.VehicleTypes[0]);
            css = new(Instance, []);
            IntegratedSolver = new(vss, css);
            IntegratedSolver.initializeCover();
            List<VehicleTask> baseVTCover = [.. vss.SelectedTasks];
            List<(CrewDuty, int)> baseCDCover = [.. css.SelectedDuties];
            Console.WriteLine($"{Config.CNSL_OVERRIDE}-1;" +
                $"{vss.Costs()};{css.Costs()};-;-;" +
                $"{vss.Tasks.Count};{css.Duties.Count};-;-;" +
                $"{vss.SelectedTasks.Count};{css.SelectedDuties.Count};-;-;" +
                $"-;-;-");

            Config.VCSP_ROUNDS = 0;
            IntegratedSolver = new(vss, css);
            IntegratedSolver.Solve();
            Console.WriteLine($"{Config.CNSL_OVERRIDE}0;" +
                $"-;-;{IntegratedSolver.vss.Costs()};{IntegratedSolver.css.Costs()};" +
                $"-;-;{IntegratedSolver.vss.Tasks.Count};{IntegratedSolver.css.Duties.Count};" +
                $"-;-;{IntegratedSolver.vss.SelectedTasks.Count};{IntegratedSolver.css.SelectedDuties.Count};" +
                $"{IntegratedSolver.model!.MIPGap};{IntegratedSolver.model.Runtime};-");


            for (int i = 1; i <= maxRounds; i += 1) {
                Config.VCSP_ROUNDS = i;
                // Unselect previous ILP
                IntegratedSolver.vss.SelectedTasks = [.. baseVTCover];
                IntegratedSolver.css.SelectedDuties = [.. baseCDCover];
                Stopwatch sw = Stopwatch.StartNew();
                IntegratedSolver.DoRound(i, false);
                IntegratedSolver.SolveILP();
                sw.Stop();
                Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};" +
                    $"-;-;{IntegratedSolver.vss.Costs()};{IntegratedSolver.css.Costs()};" +
                    $"-;-;{IntegratedSolver.vss.Tasks.Count};{IntegratedSolver.css.Duties.Count};" +
                    $"-;-;{IntegratedSolver.vss.SelectedTasks.Count};{IntegratedSolver.css.SelectedDuties.Count};" +
                    $"{IntegratedSolver.model!.MIPGap};{IntegratedSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
            }
        }

        void expVCSPDisruption() {
            Reload("VCSP Disruption");

            const int attempts = 5;
            const int normalRounds = 20;
            const int disruptionRounds = 5;

            Config.VSP_SOLVER_TIMEOUT_SEC = 600;
            Config.CSP_SOLVER_TIMEOUT_SEC = 600;
            Config.VCSP_SOLVER_TIMEOUT_SEC = 900;
            Config.VSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.VSP_LB_SEC_COL_COUNT = 8;
            Config.CSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.CSP_LB_SEC_COL_COUNT = 8;
            Config.VCSP_MAX_TASKS_DURING = 100000000;
            Config.VCSP_MAX_DUTIES_DURING = 100000000;
            Config.LAGRANGE_DISRUPT_ROUNDS = 0; // nondeterministic
            Config.LAGRANGE_N = 50;
            Config.LAGRANGE_PI_END = 0.0001;

            // Unfair testing for CSP
            Config.CR_SINGLE_SHIFT_COST = 10_000;
            Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
            Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;

            // Initialize a cover that we will reuse for all experiments
            vss = new(Instance!, Instance!.VehicleTypes[0]);
            css = new(Instance, []);
            IntegratedSolver = new(vss, css);
            IntegratedSolver.Initialize();
            for (int i = 0; i < normalRounds; i++) IntegratedSolver.DoRound(i, false);
            var basevss = vss;
            var basecss = css;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# rounds;" +
                $"int vsp value;int csp value;" +
                $"int vsp cols;int csp cols;" +
                $"int vsp selected cols;int csp selected cols;" +
                $"mipgap;mip runtime;total runtime");

            for (int i = 0; i <= disruptionRounds; i++) {
                for (int j = 0; j < attempts; j++) {
                    if (i == 0 && j > 0) continue;
                    // Unselect previous ILP
                    IntegratedSolver = new(new(basevss), new(basecss));
                    IntegratedSolver.Initialize();
                    Stopwatch sw = Stopwatch.StartNew();
                    for (int r = 0; r < i; r++) IntegratedSolver.DoRound(i + r, true);
                    IntegratedSolver.SolveILP();
                    sw.Stop();
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};" +
                        $"{IntegratedSolver.vss.Costs()};{IntegratedSolver.css.Costs()};" +
                        $"{IntegratedSolver.vss.Tasks.Count};{IntegratedSolver.css.Duties.Count};" +
                        $"{IntegratedSolver.vss.SelectedTasks.Count};{IntegratedSolver.css.SelectedDuties.Count};" +
                        $"{IntegratedSolver.model!.MIPGap};{IntegratedSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
                }
            }
        }

        void expNothing() { }
        #endregion
    }
}
