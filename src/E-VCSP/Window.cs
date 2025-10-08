
using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver.SolutionState;
using E_VCSP_Backend;
using System.Reflection;
using System.Text.Json;

namespace E_VCSP {
    public partial class MainView : Form {
        Runner runner = new();

        private readonly Dictionary<string, Control> controlsMap = [];
        RosterDisplay rd = new();

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

        public MainView() {
            InitializeComponent();
            splitContainer.Panel1.Controls.Add(rd);

            // Add all experiments to comboBox1
            comboBox1.Items.Clear();
            foreach (var exp in runner.Experiments) {
                comboBox1.Items.Add(exp.Method.Name);
            }

            Console.SetOut(new ConsoleIntercept(consoleView));
            Console.WriteLine("Program started");

            activeFolderLabel.Text = runner.ActiveFolder;
            GenerateConfigUI();
        }

        private void runExperiment_Click(object sender, EventArgs e) {
            if (comboBox1.SelectedIndex >= 0 && comboBox1.SelectedIndex < runner.Experiments.Count) {
                Config.GLOBAL_CONSOLE_KILL = true;
                Console.WriteLine($"{Config.CNSL_OVERRIDE}Starting experiment: {runner.Experiments[comboBox1.SelectedIndex].Method.Name}");
                runner.Experiments[comboBox1.SelectedIndex]();
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

        private void loadInstanceClick(object sender, EventArgs e) {
            var res = loadFolderBrowser.ShowDialog();
            if (res == DialogResult.OK) {
                runner.ActiveFolder = loadFolderBrowser.SelectedPath;
                activeFolderLabel.Text = runner.ActiveFolder.Split("\\").Last();
                Console.WriteLine($"Selected folder: {runner.ActiveFolder}");
            }
        }

        #region Solve from instance
        private async void solveVSPClick(object sender, EventArgs e) {
            working = true;
            solveVSPButton.Enabled = false;
            solveCSPButton.Enabled = false;
            solveEVCSPButton.Enabled = false;

            bool success = await runner.VSPFromInstance();
            if (success) {
                view = 0;
                rd.UpdateRosterNodes(
                    SolutionGraph.GenerateVehicleTaskGraph(runner.vss!.SelectedTasks)
                );
                solveCSPButton.Enabled = true;
            }

            working = false;
            solveVSPButton.Enabled = true;
            solveEVCSPButton.Enabled = true;
        }
        private async void solveCSPClick(object sender, EventArgs e) {
            working = true;
            solveVSPButton.Enabled = false;
            solveCSPButton.Enabled = false;
            solveEVCSPButton.Enabled = false;

            bool success = await Task.Run(() => runner.CSPFromInstance());

            if (success) {
                view = 2;
                rd.UpdateRosterNodes(SolutionGraph.GenerateCrewDutyGraph(runner.css!.SelectedDuties));
            }

            working = false;
            solveVSPButton.Enabled = true;
            solveCSPButton.Enabled = true;
            solveEVCSPButton.Enabled = true;
        }

        private async void solveEVCSPClick(object sender, EventArgs e) {
            working = true;
            solveVSPButton.Enabled = false;
            solveCSPButton.Enabled = false;
            solveEVCSPButton.Enabled = false;

            bool success = await Task.Run(() => runner.VCSPFromInstance());
            if (success) {
                view = 0;
                rd.UpdateRosterNodes(SolutionGraph.GenerateVehicleTaskGraph(runner.vss!.SelectedTasks));
            }

            working = false;
            solveVSPButton.Enabled = true;
            solveCSPButton.Enabled = true;
            solveEVCSPButton.Enabled = true;
        }
        #endregion

        private void toggleGraphView(object sender, EventArgs e) {
            if (runner.Instance == null) return;
            view = (view + 1) % 3;

            List<List<RosterNode>> newNodes = [];
            if (view == 0 && runner.vss != null && runner.vss.SelectedTasks.Count > 0)
                newNodes = SolutionGraph.GenerateVehicleTaskGraph(runner.vss.SelectedTasks);
            else if (view == 1 && runner.vss != null && runner.vss.SelectedTasks.Count > 0)
                newNodes = SolutionGraph.GenerateBlockGraph(runner.vss.SelectedTasks.Select(x => Block.FromVehicleTask(x)).ToList());
            else if (view == 2 && runner.css != null && runner.css.SelectedDuties.Count > 0)
                newNodes = SolutionGraph.GenerateCrewDutyGraph(runner.css.SelectedDuties);

            rd.UpdateRosterNodes(newNodes);
        }

        private void loadEVSPResultClick(object sender, EventArgs e) {
            var res = loadResultDialog.ShowDialog();
            if (res == DialogResult.OK) {
                Console.WriteLine("Starting load of " + loadResultDialog.FileName);
                var dump = JsonSerializer.Deserialize<VehicleSolutionStateDump>(File.ReadAllText(loadResultDialog.FileName), new JsonSerializerOptions());
                if (dump == null) throw new InvalidDataException("Dump not valid");

                runner.ActiveFolder = dump.path;
                runner.Reload();

                runner.vss = new(runner.Instance!, runner.Instance!.VehicleTypes[0]);
                runner.vss.LoadFromDump(dump);
                view = 0;
                rd.UpdateRosterNodes(SolutionGraph.GenerateVehicleTaskGraph(runner.vss!.SelectedTasks));
                viewToggleButton.Enabled = true;
                solveCSPButton.Enabled = true;
                Console.WriteLine("Loaded " + loadResultDialog.FileName);
            }
        }

        private void loadCSPResultClick(object sender, EventArgs e) {
            var res = loadResultDialog.ShowDialog();
            if (res == DialogResult.OK) {
                Console.WriteLine("Starting load of " + loadResultDialog.FileName);
                var dump = JsonSerializer.Deserialize<CrewSolutionStateDump>(File.ReadAllText(loadResultDialog.FileName), new JsonSerializerOptions());
                if (dump == null) throw new InvalidDataException("Dump not valid");

                runner.ActiveFolder = dump.path;
                runner.Reload();

                runner.css = new(runner.Instance!, []);
                runner.css.LoadFromDump(dump);
                view = 2;
                rd.UpdateRosterNodes(SolutionGraph.GenerateCrewDutyGraph(runner.css!.SelectedDuties));
                viewToggleButton.Enabled = true;
                solveCSPButton.Enabled = true;
                Console.WriteLine("Loaded " + loadResultDialog.FileName);
            }
        }
    }
}
