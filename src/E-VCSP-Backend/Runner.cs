using E_VCSP;
using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver;
using E_VCSP.Solver.SolutionState;
using E_VCSP_Backend.Formatting;
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

        public void CreateLogFolder(string description) {
            string timestamp = DateTime.Now.ToString("yy-MM-dd-HH.mm.ss");
            Constants.RUN_LOG_FOLDER = $"./runs/{timestamp} {description}/";
            Directory.CreateDirectory(Constants.RUN_LOG_FOLDER);
            Config.Dump();
            Console.WriteLine($"Instance reloaded. Current config state dumped to {Constants.RUN_LOG_FOLDER + "config.txt"}");
        }

        public void Reload(string runDescription = "No Description", bool createNewFolder = true) {
            if (ActiveFolder == "No folder selected") return;

            Instance = new(ActiveFolder);
            vss = new(Instance, Instance.VehicleTypes[0]);
            css = new(Instance, []);

            VSPSolver = new(vss);
            CSPSolver = new(css);
            IntegratedSolver = new EVCSPCGLagrange(vss, css);

            if (createNewFolder) CreateLogFolder(runDescription);
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

        void expVCSPDisruption10() {
            Reload("VCSP Disruption");

            const int attempts = 10;
            const int normalRounds = 10;
            const int disruptionRounds = 10;

            Config.VSP_SOLVER_TIMEOUT_SEC = 600;
            Config.CSP_SOLVER_TIMEOUT_SEC = 600;
            Config.VCSP_SOLVER_TIMEOUT_SEC = 600;
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

        void expVCSPDisruption20() {
            Reload("VCSP Disruption");

            const int attempts = 10;
            const int normalRounds = 20;
            const int disruptionRounds = 10;

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

        void expVCSPMaxCols() {
            Reload("VCSP Max cols v2");

            int attempts = 5;
            List<int> maxCols = [500, 2500, 5000, 10_000, 25_000];

            Config.VSP_SOLVER_TIMEOUT_SEC = 600;
            Config.CSP_SOLVER_TIMEOUT_SEC = 600;
            Config.VCSP_SOLVER_TIMEOUT_SEC = 600;
            Config.VSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.VSP_LB_SEC_COL_COUNT = 8;
            Config.CSP_LB_SEC_COL_ATTEMPTS = 16;
            Config.CSP_LB_SEC_COL_COUNT = 8;
            Config.VCSP_ROUNDS = 5;
            Config.LAGRANGE_DISRUPT_ROUNDS = 0; // nondeterministic
            Config.LAGRANGE_N = 50;
            Config.LAGRANGE_PI_END = 0.0001;

            // Unfair testing for CSP
            Config.CR_SINGLE_SHIFT_COST = 10_000;
            Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
            Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;

            vss = new(Instance!, Instance!.VehicleTypes[0]);
            css = new(Instance, []);
            IntegratedSolver = new(vss, css);
            IntegratedSolver.Initialize();
            var basevss = vss;
            var basecss = css;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}max vh cols;max cr cols" +
                $"int vsp value;int csp value;" +
                $"int vsp cols;int csp cols;" +
                $"int vsp selected cols;int csp selected cols;" +
                $"mipgap;mip runtime;total runtime");

            foreach (int maxvhcols in maxCols) {
                foreach (int maxcrcols in maxCols) {
                    for (int i = 0; i < attempts; i++) {
                        Config.VCSP_MAX_TASKS_DURING = maxvhcols;
                        Config.VCSP_MAX_DUTIES_DURING = maxcrcols;

                        IntegratedSolver = new(new(basevss), new(basecss));
                        Stopwatch sw = Stopwatch.StartNew();
                        IntegratedSolver.Solve();
                        sw.Stop();
                        Console.WriteLine($"{Config.CNSL_OVERRIDE}{maxvhcols};{maxcrcols};" +
                            $"{IntegratedSolver.vss.Costs()};{IntegratedSolver.css.Costs()};" +
                            $"{IntegratedSolver.vss.Tasks.Count};{IntegratedSolver.css.Duties.Count};" +
                            $"{IntegratedSolver.vss.SelectedTasks.Count};{IntegratedSolver.css.SelectedDuties.Count};" +
                            $"{IntegratedSolver.model!.MIPGap};{IntegratedSolver.model.Runtime};{sw.Elapsed.TotalSeconds}");
                    }
                }
            }
        }

        void expFindBestSeq() {
            List<string> filepaths = ["../../data/terschelling", "../../data/leiden-1-2", "../../data/leiden-3-4-14", "../../data/leiden"];
            List<int> Ns = [4, 8, 16];
            List<int> Ms = [4, 8];
            Config.VSP_INSTANCES_PER_IT = 20;
            List<int> lsRounds = [100 / Config.VSP_INSTANCES_PER_IT, 1000 / Config.VSP_INSTANCES_PER_IT];
            const int lsmaxIts = 500_000;

            for (int fpi = 0; fpi < filepaths.Count; fpi++) {
                string filepath = filepaths[fpi];
                ActiveFolder = filepath;
                Reload("FindBestSeq", fpi == 0);
                if (fpi == 0) {
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}" +
                        $"instance;sec n;sec m;ls rounds;ls maxIts;" +
                        $"seq vsp value;seq csp value;" +
                        $"seq vsp cols;seq csp cols;" +
                        $"seq vsp selected cols;seq csp selected cols;seq csp selected single;" +
                        $"vsp mipgap;vsp mip runtime;vsp total runtime;" +
                        $"csp mipgap;csp mip runtime;csp total runtime"
                    );
                }

                foreach (var N in Ns) {
                    foreach (var M in Ms) {
                        foreach (var lsRound in lsRounds) {
                            Config.VSP_SOLVER_TIMEOUT_SEC = 300;
                            Config.CSP_SOLVER_TIMEOUT_SEC = 300;
                            Config.VSP_LB_SEC_COL_ATTEMPTS = N;
                            Config.VSP_LB_SEC_COL_COUNT = M;
                            Config.CSP_LB_SEC_COL_ATTEMPTS = N;
                            Config.CSP_LB_SEC_COL_COUNT = M;
                            Config.VSP_LS_G_ITERATIONS = lsmaxIts;
                            Config.VSP_OPT_IT_THRESHOLD = Math.Max(10, lsRound + 10);
                            Config.VSP_OPERATION_SEQUENCE = String.Join("", Enumerable.Range(0, lsRound).Select(x => "2"));
                            Config.VSP_MAX_COL_GEN_ITS = Math.Max(200, lsRound * 2);
                            Config.CR_SINGLE_SHIFT_COST = 1000;

                            if (fpi < 3) {
                                Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
                                Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;
                            }
                            else {
                                Config.CR_MAX_SHORT_IDLE_TIME = 30 * 60;
                                Config.CR_MAX_BREAK_TIME = 60 * 60;
                            }

                            vss = new(Instance!, Instance!.VehicleTypes[0]);
                            VSPSolver = new EVSPCG(vss);
                            Stopwatch sw = Stopwatch.StartNew();
                            VSPSolver.Solve();
                            sw.Stop();
                            double elapsedSecsVSP = sw.Elapsed.TotalSeconds;

                            css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
                            CSPSolver = new CSPCG(css);
                            sw.Restart();
                            CSPSolver.Solve();
                            sw.Stop();
                            double elapsedSecsCSP = sw.Elapsed.TotalSeconds;

                            Console.WriteLine($"{Config.CNSL_OVERRIDE}" +
                                $"{filepath};{N};{M};{lsRound};{lsmaxIts};" +
                                $"{vss.Costs()};{css.Costs()};" +
                                $"{vss.Tasks.Count};{css.Duties.Count};" +
                                $"{vss.SelectedTasks.Count};{css.SelectedDuties.Count};{css.SelectedDuties.Count(x => x.duty.Type == DutyType.Single)};" +
                                $"{VSPSolver.model.MIPGap};{VSPSolver.model.Runtime};{elapsedSecsVSP};" +
                                $"{CSPSolver.model.MIPGap};{CSPSolver.model.Runtime};{elapsedSecsCSP}");
                        }
                    }
                }
            }
        }

        void expFindBestIntegrated() {
            CreateLogFolder("Best seq-int");
            Directory.CreateDirectory(Constants.RUN_LOG_FOLDER + "rosters/seq");
            Directory.CreateDirectory(Constants.RUN_LOG_FOLDER + "rosters/int");
            string seqFile = Constants.RUN_LOG_FOLDER + "seq-raw-data.csv";
            string intFile = Constants.RUN_LOG_FOLDER + "int-raw-data.csv";
            void writeToSeq(string s) => File.AppendAllText(seqFile, s + "\n");
            void writeToInt(string s) => File.AppendAllText(intFile, s + "\n");
            writeToSeq("instance;sec n;sec m;ls rounds;ls maxIts;" +
                $"seq vsp cols;seq csp cols;" +
                $"seq vsp selected cols;seq csp selected cols;seq csp selected single;" +
                $"seq vsp value;seq csp value;" +
                $"seq vsp pullout;seq vsp tripdriving;seq vsp tripkwh;seq vsp dhdriving;seq vsp dhkwh;seq vsp idlekwh;seq vsp batterydeg;" +
                $"seq csp blockhours;seq csp blockdrivinghours;seq csp breakhours;" +
                $"seq vsp mip runtime;seq vsp mip gap;seq vsp runtime;" +
                $"seq csp mip runtime;seq csp mip gap;seq csp runtime"
            );
            writeToInt("instance;sec n;sec m;ls rounds;ls maxIts;" +
                $"int vsp cols;int csp cols;" +
                $"int vsp selected cols;int csp selected cols;int csp selected single;" +
                $"int vsp value;int csp value;" +
                $"int vsp pullout;int vsp tripdriving;int vsp tripkwh;int vsp dhdriving;int vsp dhkwh;int vsp idlekwh;int vsp batterydeg;" +
                $"int csp blockhours;int csp blockdrivinghours;int csp breakhours;" +
                $"int mip runtime;int mip gap;int total runtime"
            );
            Stopwatch sw = new();

            Console.WriteLine(Config.CNSL_OVERRIDE + "Starting best seq-int experiment; writing results to " + seqFile + " and " + intFile);
            Console.WriteLine(Config.CNSL_OVERRIDE + "Experiment updates are written to this file.");

            List<string> filepaths = ["../../data/terschelling", "../../data/leiden-3-4-14", "../../data/leiden-1-2", "../../data/leiden"];

            // Param sets 
            List<int> Ns = [4, 8, 16];
            List<int> Ms = [4, 8];
            Config.VSP_INSTANCES_PER_IT = 20;
            List<int> lsRounds = [0, 100 / Config.VSP_INSTANCES_PER_IT, 1000 / Config.VSP_INSTANCES_PER_IT];
            const int lsmaxIts = 500_000;

            // Total number times integrated is attempted using seq solution
            const int integratedAttempts = 5;

            List<(VehicleSolutionState vss, CrewSolutionState css, int N, int M, int lsRounds)> bestSequentialSolutions = [];

            for (int fpi = 0; fpi < filepaths.Count; fpi++) {
                string filepath = filepaths[fpi];
                ActiveFolder = filepath;

                if (fpi < 3) {
                    Config.CR_MAX_SHORT_IDLE_TIME = 4 * 60 * 60;
                    Config.CR_MAX_BREAK_TIME = 4 * 60 * 60;
                }
                else {
                    Config.CR_MAX_SHORT_IDLE_TIME = 30 * 60;
                    Config.CR_MAX_BREAK_TIME = 60 * 60;
                }
                Reload("FindBestInt" + filepath, false);
                Console.WriteLine(Config.CNSL_OVERRIDE + "Starting sequential rounds for " + filepath);
                foreach (var N in Ns) {
                    foreach (var M in Ms) {
                        foreach (var lsRound in lsRounds) {
                            Config.VSP_SOLVER_TIMEOUT_SEC = 300;
                            Config.CSP_SOLVER_TIMEOUT_SEC = 300;
                            Config.VSP_LB_SEC_COL_ATTEMPTS = N;
                            Config.VSP_LB_SEC_COL_COUNT = M;
                            Config.CSP_LB_SEC_COL_ATTEMPTS = N;
                            Config.CSP_LB_SEC_COL_COUNT = M;
                            Config.VSP_LS_G_ITERATIONS = lsmaxIts;
                            Config.VSP_OPT_IT_THRESHOLD = Math.Max(10, lsRound + 10);
                            Config.VSP_OPERATION_SEQUENCE = String.Join("", Enumerable.Range(0, lsRound).Select(x => "2"));
                            Config.VSP_MAX_COL_GEN_ITS = Math.Max(1000, lsRound * 2);
                            Config.CSP_MAX_COL_GEN_ITS = 1000;
                            Config.CR_SINGLE_SHIFT_COST = 1000;

                            vss = new(Instance!, Instance!.VehicleTypes[0]);
                            VSPSolver = new EVSPCG(vss);
                            sw.Restart();
                            VSPSolver.Solve();
                            sw.Stop();
                            double elapsedSecsVSP = sw.Elapsed.TotalSeconds;

                            css = new(Instance, Block.FromVehicleTasks(vss.SelectedTasks));
                            CSPSolver = new CSPCG(css);
                            sw.Restart();
                            CSPSolver.Solve();
                            sw.Stop();
                            double elapsedSecsCSP = sw.Elapsed.TotalSeconds;
                            Console.WriteLine(Config.CNSL_OVERRIDE + $"Finished seq round {filepath};{N};{M};{lsRound};{lsmaxIts}: {vss.Costs()} + {css.Costs()} = {vss.Costs() + css.Costs()}");

                            // Write sequential result to disk
                            var vsscf = vss.CostFactors();
                            var csscf = css.CostFactors();
                            writeToSeq($"{filepath};{N};{M};{lsRound};{lsmaxIts};" +
                                $"{vss.Tasks.Count};{css.Duties.Count};" +
                                $"{vss.SelectedTasks.Count};{css.SelectedDuties.Count};{css.SelectedDuties.Count(x => x.duty.Type == DutyType.Single)};" +
                                $"{vss.Costs()};{css.Costs()};" +
                                $"{vsscf.pullout};{vsscf.tripdriving};{vsscf.tripkwh};{vsscf.dhdriving};{vsscf.dhkwh};{vsscf.idlekwh};{vsscf.batterydeg};" +
                                $"{csscf.blockHours};{csscf.blockDrivingHours};{csscf.breakHours};" +
                                $"{VSPSolver.model.Runtime};{VSPSolver.model.MIPGap};{elapsedSecsVSP};" +
                                $"{CSPSolver.model.Runtime};{CSPSolver.model.MIPGap};{elapsedSecsCSP}"
                            );

                            // Save best vss/css for integrated runs in memory; Only dump rosternode views to files
                            if (bestSequentialSolutions.Count == fpi) {
                                bestSequentialSolutions.Add((new(vss), new(css), N, M, lsRound));
                            }
                            else if (vss.Costs() + css.Costs() < bestSequentialSolutions[fpi].vss.Costs() + bestSequentialSolutions[fpi].css.Costs()) {
                                bestSequentialSolutions[fpi] = (new(vss), new(css), N, M, lsRound);
                            }

                            Roster vhRoster = new() {
                                Comment = vss.CostFactors().ToString(),
                                RosterNodes = SolutionGraph.GenerateVehicleTaskGraph(vss.SelectedTasks),
                                Type = 0
                            };
                            Roster blockRoster = new() {
                                Comment = "Blocks based on vehicle tasks; " + vss.CostFactors().ToString(),
                                RosterNodes = SolutionGraph.GenerateBlockGraph(vss.SelectedTasks.Select(Block.FromVehicleTask).ToList()),
                                Type = 1
                            };
                            Roster crRoster = new() {
                                Comment = css.CostFactors().ToString(),
                                RosterNodes = SolutionGraph.GenerateCrewDutyGraph(css.SelectedDuties),
                                Type = 2
                            };

                            string instanceName = $"{filepath.Split("/")[^1]}-n{N}-m{M}-lsr{lsRound}";
                            vhRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/seq/{instanceName}");
                            blockRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/seq/{instanceName}");
                            crRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/seq/{instanceName}");
                        }
                    }
                }

                var bestSeqSol = bestSequentialSolutions[fpi];

                Console.WriteLine(Config.CNSL_OVERRIDE + "Starting integrated attempts");
                Console.WriteLine(Config.CNSL_OVERRIDE + $"Loading sequential solution with value {bestSeqSol.vss.Costs()} + {bestSeqSol.css.Costs()} = {bestSeqSol.vss.Costs() + bestSeqSol.css.Costs()}");
                for (int attempt = 0; attempt < integratedAttempts; attempt++) {
                    Config.VSP_SOLVER_TIMEOUT_SEC = 300;
                    Config.CSP_SOLVER_TIMEOUT_SEC = 300;
                    Config.VSP_LB_SEC_COL_ATTEMPTS = 16;
                    Config.VSP_LB_SEC_COL_COUNT = bestSeqSol.M;
                    Config.CSP_LB_SEC_COL_ATTEMPTS = 16;
                    Config.CSP_LB_SEC_COL_COUNT = bestSeqSol.M;
                    Config.VSP_LS_G_ITERATIONS = lsmaxIts;
                    Config.VSP_OPT_IT_THRESHOLD = Math.Max(10, bestSeqSol.lsRounds + 10);
                    Config.VSP_OPERATION_SEQUENCE = String.Join("", Enumerable.Range(0, bestSeqSol.lsRounds).Select(x => "2"));
                    Config.VSP_MAX_COL_GEN_ITS = Math.Max(1000, bestSeqSol.lsRounds * 2);
                    Config.CSP_MAX_COL_GEN_ITS = 1000;
                    Config.CR_SINGLE_SHIFT_COST = 1000;
                    Config.VCSP_SOLVER_TIMEOUT_SEC = 1800;
                    Config.VCSP_MAX_TASKS_DURING = 100_000;
                    Config.VCSP_MAX_DUTIES_DURING = 100_000;
                    Config.VCSP_ROUNDS = 15;
                    Config.LAGRANGE_DISRUPT_ROUNDS = 5;

                    VehicleSolutionState intvss = new(bestSeqSol.vss);
                    CrewSolutionState intcss = new(bestSeqSol.css);

                    IntegratedSolver = new(intvss, intcss);
                    sw.Restart();
                    IntegratedSolver.Solve();
                    sw.Stop();

                    Console.WriteLine(Config.CNSL_OVERRIDE + $"Finished int round {filepath};{bestSeqSol.N};{bestSeqSol.M};{bestSeqSol.lsRounds};{lsmaxIts}: {IntegratedSolver.vss.Costs()} + {IntegratedSolver.css.Costs()} = {IntegratedSolver.vss.Costs() + IntegratedSolver.css.Costs()}");


                    var vsscf = IntegratedSolver.vss.CostFactors();
                    var csscf = IntegratedSolver.css.CostFactors();
                    writeToInt($"{filepath};{bestSeqSol.N};{bestSeqSol.M};{bestSeqSol.lsRounds};{lsmaxIts};" +
                        $"{IntegratedSolver.vss.Tasks.Count};{IntegratedSolver.css.Duties.Count};" +
                        $"{IntegratedSolver.vss.SelectedTasks.Count};{IntegratedSolver.css.SelectedDuties.Count};{IntegratedSolver.css.SelectedDuties.Count(x => x.duty.Type == DutyType.Single)};" +
                        $"{IntegratedSolver.vss.Costs()};{IntegratedSolver.css.Costs()};" +
                        $"{vsscf.pullout};{vsscf.tripdriving};{vsscf.tripkwh};{vsscf.dhdriving};{vsscf.dhkwh};{vsscf.idlekwh};{vsscf.batterydeg};" +
                        $"{csscf.blockHours};{csscf.blockDrivingHours};{csscf.breakHours};" +
                        $"{IntegratedSolver.model.Runtime};{IntegratedSolver.model.MIPGap};{sw.Elapsed.TotalSeconds}"
                    );

                    Roster vhRoster = new() {
                        Comment = intvss.CostFactors().ToString(),
                        RosterNodes = SolutionGraph.GenerateVehicleTaskGraph(intvss.SelectedTasks),
                        Type = 0
                    };
                    Roster blockRoster = new() {
                        Comment = "Blocks based on vehicle tasks; " + intvss.CostFactors().ToString(),
                        RosterNodes = SolutionGraph.GenerateBlockGraph(intvss.SelectedTasks.Select(Block.FromVehicleTask).ToList()),
                        Type = 1
                    };
                    Roster crRoster = new() {
                        Comment = intcss.CostFactors().ToString(),
                        RosterNodes = SolutionGraph.GenerateCrewDutyGraph(intcss.SelectedDuties),
                        Type = 2
                    };

                    string instanceName = $"{filepath.Split("/")[^1]}-n{bestSeqSol.N}-m{bestSeqSol.M}-lsr{bestSeqSol.lsRounds}-a{attempt}";
                    vhRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/int/{instanceName}");
                    blockRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/int/{instanceName}");
                    crRoster.Dump(Constants.RUN_LOG_FOLDER + $"rosters/int/{instanceName}");

                }
            }
        }

        void expNothing() { }
        #endregion

    }
}