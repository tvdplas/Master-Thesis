
using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver;
using E_VCSP.Solver.SolutionState;
using System.Reflection;
using System.Text;
using System.Text.Json;

namespace E_VCSP {
    public partial class MainView : Form {
        private readonly Dictionary<string, Control> controlsMap = [];
        public string activeFolder = "No folder selected";

        Instance? instance;
        VehicleSolutionState? vss;
        CrewSolutionState? css;

        EVSPCG? vspSolver;
        CSPCG? cspSolver;
        EVCSPCGLagrange? integratedSolver;

        RosterDisplay rd = new();

        List<Action> experiments = [];

        bool working = false;
        int __view = 0; // 0: vehicles, 1: blocks, 2: duties, 3: general
        int view {
            get {
                return __view;
            }
            set {
                string v = value == 0 ? "vehicles" : value == 1 ? "blocks" : "duties";
                Console.WriteLine("View set to " + v);
                __view = value;
            }
        }

        void none() {
            Console.WriteLine("Nothing happened.");
        }

        async void vspSecondaryColumns() {
            Console.WriteLine($"{Config.CNSL_OVERRIDE}Starting experiment: vsp secondary columns");

            const int attempts = 16;
            const int subdivisions = 16;

            reload(); // Ensure instance and solver are ready
            if (vss == null || vspSolver == null) return;

            Config.VSP_SOLVER_TIMEOUT_SEC = 900;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}# attempts;# subdivs;value;#unique cols;mipgap;runtime");


            for (int i = 0; i <= attempts; i = Math.Max(1, i * 2)) {
                Config.VSP_LB_SEC_COL_ATTEMPTS = i;
                for (int j = 0; j <= subdivisions; j += 4) {
                    Config.VSP_LB_SEC_COL_COUNT = i;

                    vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                    vspSolver = new EVSPCG(vss);

                    ctSource = new();
                    var token = ctSource.Token;
                    bool success = false;
                    success = await Task.Run(() => vspSolver.Solve(token), token);
                    ctSource?.Dispose(); // Dispose the source
                    ctSource = null;
                    Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{j};{vss.Costs()};{vss.Tasks.Count};{vspSolver.model.MIPGap};{vspSolver.model.Runtime}");
                }
            }

            vspTargetVehicles();
        }

        async void vspTargetVehicles() {
            Console.WriteLine($"{Config.CNSL_OVERRIDE}Starting experiment: vsp target vehicles");

            int minMaxVehicles = 8;
            int maxMaxVehicles = 17;

            reload(); // Ensure instance and solver are ready
            if (vss == null || vspSolver == null) return;

            Config.VSP_SOLVER_TIMEOUT_SEC = 900;

            Console.WriteLine($"{Config.CNSL_OVERRIDE}max vh;value;cols;mipgap;runtime");

            for (int i = minMaxVehicles; i <= maxMaxVehicles; i++) {
                Config.MAX_VEHICLES = i;

                vss = new(vss.Instance, vss.Instance.VehicleTypes[0]);
                vspSolver = new EVSPCG(vss);

                ctSource = new();
                var token = ctSource.Token;
                bool success = false;
                success = await Task.Run(() => vspSolver.Solve(token), token);
                ctSource?.Dispose(); // Dispose the source
                ctSource = null;
                Console.WriteLine($"{Config.CNSL_OVERRIDE}{i};{vss.Costs()};{vss.Tasks.Count};{vspSolver.model.MIPGap};{vspSolver.model.Runtime}");
            }
        }

        public MainView() {
            InitializeComponent();
            splitContainer.Panel1.Controls.Add(rd);

            experiments.Add(none);
            experiments.Add(vspSecondaryColumns);
            experiments.Add(vspTargetVehicles);

            // Add all experiments to comboBox1
            comboBox1.Items.Clear();
            foreach (var exp in experiments) {
                comboBox1.Items.Add(exp.Method.Name);
            }

            Console.SetOut(new ConsoleIntercept(consoleView));
            Console.WriteLine("Program started");

            activeFolderLabel.Text = activeFolder;
            GenerateConfigUI();
        }

        // Add this event handler for the runExperiment button
        private void runExperiment_Click(object sender, EventArgs e) {
            if (comboBox1.SelectedIndex >= 0 && comboBox1.SelectedIndex < experiments.Count) {
                experiments[comboBox1.SelectedIndex]();
            }
        }

        private void GenerateConfigUI() {
            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            Panel scrollablePanel = new() {
                AutoScroll = true,
                BorderStyle = BorderStyle.FixedSingle,
                Location = new System.Drawing.Point(10, 135),
                Size = new System.Drawing.Size(280, 405),
                Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Bottom,
                Padding = new Padding(0, 0, 0, 100),
            };
            configPanel.Controls.Add(scrollablePanel);

            int yOffset = 10;
            foreach (FieldInfo field in fields) {
                Label label = new() {
                    Text = field.Name.Replace('_', ' ').ToLower(),
                    AutoSize = true,
                    Location = new System.Drawing.Point(10, yOffset)
                };
                scrollablePanel.Controls.Add(label);

                if (field.FieldType == typeof(Header)) {
                    yOffset += 30;
                    continue;
                }

                Control inputControl;
                if (field.FieldType == typeof(int) || field.FieldType == typeof(double) || field.FieldType == typeof(string)) {
                    TextBox textBox = new() {
                        Text = field.GetValue(null)?.ToString()?.Replace('_', ' ').ToLower(),
                        Location = new System.Drawing.Point(150, yOffset),
                        Width = 100
                    };
                    textBox.TextChanged += (sender, e) => UpdateString(field, textBox.Text ?? "");
                    inputControl = textBox;
                }
                else if (field.FieldType.IsEnum) {
                    ComboBox comboBox = new() {
                        DropDownStyle = ComboBoxStyle.DropDownList,
                        Location = new System.Drawing.Point(150, yOffset),
                        Width = 100
                    };
                    comboBox.Items.AddRange(Enum.GetNames(field.FieldType));
                    comboBox.SelectedItem = field.GetValue(null)?.ToString();
                    comboBox.SelectedIndexChanged += (sender, e) => {
                        field.SetValue(null, Enum.Parse(field.FieldType, comboBox.SelectedItem?.ToString() ?? ""));
                        Console.WriteLine($"Update {field.Name} to value {comboBox.SelectedItem?.ToString() ?? ""}");
                    };
                    inputControl = comboBox;
                }
                else if (field.FieldType == typeof(bool)) {
                    CheckBox check = new() {
                        Checked = (bool)(field.GetValue(null) ?? false),
                        Location = new System.Drawing.Point(150, yOffset),
                    };
                    check.CheckedChanged += (_, _) => {
                        field.SetValue(null, check.Checked);
                        Console.WriteLine($"Update {field.Name} to {check.Checked}");
                    };

                    inputControl = check;
                }
                else {
                    continue;
                }

                scrollablePanel.Controls.Add(inputControl);
                controlsMap[field.Name] = inputControl;
                yOffset += 30;
            }
        }

        private static void UpdateString(FieldInfo field, string text) {
            if (field.FieldType == typeof(string)) {
                field.SetValue(null, text);
                Console.WriteLine($"Update {field.Name} to value {text}");
            }

            if (field.FieldType == typeof(int) && int.TryParse(text, out int i)) {
                field.SetValue(null, i);
                Console.WriteLine($"Update {field.Name} to value {i}");
            }

            if (field.FieldType == typeof(double) && double.TryParse(text, out double d)) {
                field.SetValue(null, d);
                Console.WriteLine($"Update {field.Name} to value {d}");
            }
        }

        private void loadButtonClick(object sender, EventArgs e) {
            var res = loadFolderBrowser.ShowDialog();
            if (res == DialogResult.OK) {
                activeFolder = loadFolderBrowser.SelectedPath;
                activeFolderLabel.Text = activeFolder.Split("\\").Last();
                Console.WriteLine($"Selected folder: {activeFolder}");
            }
        }

        private async void solveVSPClick(object sender, EventArgs e) {
            reload(); // Ensure instance and solver are ready
            if (vspSolver == null) return;

            working = true;
            solveVSPButton.Enabled = false;
            ctSource = new();
            var token = ctSource.Token;

            bool success = false;
            try {
                success = await Task.Run(() => vspSolver.Solve(token), token);

                if (success) {
                    view = 0;
                    rd.UpdateRosterNodes(SolutionGraph.GenerateVehicleTaskGraph(vss!.SelectedTasks));
                    viewToggleButton.Enabled = true;
                    solveCSPButton.Enabled = true;
                    Console.WriteLine("Solver finished successfully.");
                }
                else {
                    Console.WriteLine("Solver did not find a solution or was cancelled.");
                }
            }
            catch (OperationCanceledException) {
                Console.WriteLine("Solver operation was cancelled.");
            }
            finally {
                working = false;
                solveVSPButton.Enabled = true;
                ctSource?.Dispose(); // Dispose the source
                ctSource = null;
            }
        }

        private async void solveEVCSPClick(object sender, EventArgs e) {
            reload(); // Ensure instance and solver are ready
            if (integratedSolver == null) return;

            working = true;
            solveEVCSPButton.Enabled = false;
            ctSource = new CancellationTokenSource();
            var token = ctSource.Token;

            bool success = false;
            try {
                success = await Task.Run(() => integratedSolver.Solve(token), token);

                if (success) {
                    view = 0;
                    rd.UpdateRosterNodes(SolutionGraph.GenerateVehicleTaskGraph(integratedSolver.vss.SelectedTasks));
                    viewToggleButton.Enabled = true;
                    solveCSPButton.Enabled = true;
                    Console.WriteLine("Solver finished successfully.");
                }
                else {
                    Console.WriteLine("Solver did not find a solution or was cancelled.");
                }
            }
            catch (OperationCanceledException) {
                Console.WriteLine("Solver operation was cancelled.");
            }
            finally {
                working = false;
                solveEVCSPButton.Enabled = true;
                ctSource?.Dispose(); // Dispose the source
                ctSource = null;
            }
        }

        private void reload() {
            if (activeFolder == "No folder selected") return;

            solveVSPButton.Enabled = true;
            solveEVCSPButton.Enabled = true;
            viewToggleButton.Enabled = false;
            solveCSPButton.Enabled = false;

            instance = new(activeFolder);
            vss = new(instance, instance.VehicleTypes[0]);
            css = new(instance, []);

            vspSolver = new(vss);
            integratedSolver = new EVCSPCGLagrange(vss, css);

            string timestamp = DateTime.Now.ToString("yy-MM-dd HH.mm.ss");
            Constants.RUN_LOG_FOLDER = $"./runs/{timestamp}/";
            Directory.CreateDirectory(Constants.RUN_LOG_FOLDER);

            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
            StringBuilder configDump = new();
            foreach (FieldInfo field in fields)
                configDump.AppendLine($"{field.Name}: {field.GetValue(null)}");
            File.WriteAllText(Constants.RUN_LOG_FOLDER + "config.txt", configDump.ToString());
            Console.WriteLine($"Instance reloaded. Current config state dumped to {Constants.RUN_LOG_FOLDER + "config.txt"}");
        }

        private void toggleGraphView(object sender, EventArgs e) {
            if (instance == null || working) return;
            view = (view + 1) % 3;

            List<List<RosterNode>> newNodes = [];
            if (view == 0 && vss != null && vss.SelectedTasks.Count > 0)
                newNodes = SolutionGraph.GenerateVehicleTaskGraph(vss.SelectedTasks);
            else if (view == 1 && vss != null && vss.SelectedTasks.Count > 0)
                newNodes = SolutionGraph.GenerateBlockGraph(vss.SelectedTasks.Select(x => Block.FromVehicleTask(x)).ToList());
            else if (view == 2 && css != null && css.SelectedDuties.Count > 0)
                newNodes = SolutionGraph.GenerateCrewDutyGraph(css.SelectedDuties);

            rd.UpdateRosterNodes(newNodes);
        }

        private async void solveCSPClick(object sender, EventArgs e) {
            if (instance == null || vss == null || vss.SelectedTasks.Count == 0) return;

            if (css == null || css.Blocks.Count == 0 || vss.SelectedTasks.Count > 0) {
                Console.WriteLine("Solving based on VSP solution");
                css = new(instance, Block.FromVehicleTasks(vss.SelectedTasks));
            }
            else {
                Console.WriteLine("Resetting existing CSP solution");
                css.ResetFromBlocks();
            }
            Console.WriteLine($"Total of {css.Blocks.Count} blocks being considered");
            cspSolver = new CSPCG(css);

            working = true;
            solveCSPButton.Enabled = false;
            ctSource = new CancellationTokenSource();
            var token = ctSource.Token;

            bool success = false;
            try {
                success = await Task.Run(() => cspSolver.Solve(token), token);

                if (success) {
                    Console.WriteLine("CSP Solver finished successfully.");
                    view = 2;
                    rd.UpdateRosterNodes(SolutionGraph.GenerateCrewDutyGraph(css.SelectedDuties));
                }
                else Console.WriteLine("No solution/cancelled.");
            }
            catch (OperationCanceledException) {
                Console.WriteLine("Solver operation was cancelled.");
            }
            finally {

                working = false;
                solveCSPButton.Enabled = true;
                ctSource?.Dispose(); // Dispose the source
                ctSource = null;
            }
        }

        private void loadEVSPResultClick(object sender, EventArgs e) {
            var res = loadResultDialog.ShowDialog();
            if (res == DialogResult.OK) {
                Console.WriteLine("Starting load of " + loadResultDialog.FileName);
                var dump = JsonSerializer.Deserialize<VehicleSolutionStateDump>(File.ReadAllText(loadResultDialog.FileName), new JsonSerializerOptions());
                if (dump == null) throw new InvalidDataException("Dump not valid");

                activeFolder = dump.path;
                reload();

                vss = new(instance!, instance!.VehicleTypes[0]);
                vss.LoadFromDump(dump);
                view = 0;
                rd.UpdateRosterNodes(SolutionGraph.GenerateVehicleTaskGraph(vss!.SelectedTasks));
                viewToggleButton.Enabled = true;
                solveCSPButton.Enabled = true;
                Console.WriteLine("Loaded " + loadResultDialog.FileName);
            }
        }

        private void loadCSPResultButton_Click(object sender, EventArgs e) {
            var res = loadResultDialog.ShowDialog();
            if (res == DialogResult.OK) {
                Console.WriteLine("Starting load of " + loadResultDialog.FileName);
                var dump = JsonSerializer.Deserialize<CrewSolutionStateDump>(File.ReadAllText(loadResultDialog.FileName), new JsonSerializerOptions());
                if (dump == null) throw new InvalidDataException("Dump not valid");

                activeFolder = dump.path;
                reload();

                css = new(instance!, []);
                css.LoadFromDump(dump);
                view = 2;
                rd.UpdateRosterNodes(SolutionGraph.GenerateCrewDutyGraph(css!.SelectedDuties));
                viewToggleButton.Enabled = true;
                solveCSPButton.Enabled = true;
                Console.WriteLine("Loaded " + loadResultDialog.FileName);
            }
        }

        private void loadEVCSPButton_Click(object sender, EventArgs e) {
            Console.WriteLine("Use the other two buttons, only here for symmetry");
        }
    }
}
