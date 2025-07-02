
using E_VCSP.Formatting;
using E_VCSP.Objects.ParsedData;
using E_VCSP.Solver;
using System.Reflection;
using System.Text;
using System.Text.Json;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        private readonly Dictionary<string, Control> controlsMap = [];
        public string activeFolder = "No folder selected";

        Instance? instance;

        EVSPCG? vspSolver;
        Solver.Solver? cspSolver;

        bool working = false;
        bool blockView = false;


        public MainView()
        {
            InitializeComponent();
            Console.SetOut(new ConsoleIntercept(textBox1));
            Console.WriteLine("Program started");

            activeFolderLabel.Text = activeFolder;
            GenerateConfigUI();
        }

        private void GenerateConfigUI()
        {
            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            Panel scrollablePanel = new()
            {
                AutoScroll = true,
                BorderStyle = BorderStyle.FixedSingle,
                Location = new System.Drawing.Point(10, 105),
                Size = new System.Drawing.Size(280, 435),
                Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Bottom,
                Padding = new Padding(0, 0, 0, 100),
            };
            panel1.Controls.Add(scrollablePanel);

            int yOffset = 10;
            foreach (FieldInfo field in fields)
            {
                Label label = new()
                {
                    Text = field.Name.Replace('_', ' ').ToLower(),
                    AutoSize = true,
                    Location = new System.Drawing.Point(10, yOffset)
                };
                scrollablePanel.Controls.Add(label);

                if (field.FieldType == typeof(Header))
                {
                    yOffset += 30;
                    continue;
                }

                Control inputControl;
                if (field.FieldType == typeof(int) || field.FieldType == typeof(double) || field.FieldType == typeof(string))
                {
                    TextBox textBox = new()
                    {
                        Text = field.GetValue(null)?.ToString()?.Replace('_', ' ').ToLower(),
                        Location = new System.Drawing.Point(150, yOffset),
                        Width = 100
                    };
                    textBox.TextChanged += (sender, e) => UpdateString(field, textBox.Text ?? "");
                    inputControl = textBox;
                }
                else if (field.FieldType.IsEnum)
                {
                    ComboBox comboBox = new()
                    {
                        DropDownStyle = ComboBoxStyle.DropDownList,
                        Location = new System.Drawing.Point(150, yOffset),
                        Width = 100
                    };
                    comboBox.Items.AddRange(Enum.GetNames(field.FieldType));
                    comboBox.SelectedItem = field.GetValue(null)?.ToString();
                    comboBox.SelectedIndexChanged += (sender, e) =>
                        field.SetValue(null, Enum.Parse(field.FieldType, comboBox.SelectedItem?.ToString() ?? ""));
                    inputControl = comboBox;
                }
                else if (field.FieldType == typeof(bool))
                {
                    CheckBox check = new()
                    {
                        Checked = (bool)(field.GetValue(null) ?? false),
                        Location = new System.Drawing.Point(150, yOffset),
                    };
                    check.CheckedChanged += (_, _) => field.SetValue(null, check.Checked);
                    inputControl = check;
                }
                else
                {
                    continue;
                }

                scrollablePanel.Controls.Add(inputControl);
                controlsMap[field.Name] = inputControl;
                yOffset += 30;
            }

            Button applyButton = new()
            {
                Text = "Apply Changes",
                Location = new System.Drawing.Point(10, yOffset + 10),
                Width = 200
            };
            applyButton.Click += (sender, e) => reload();
            scrollablePanel.Controls.Add(applyButton);
        }

        private static void UpdateString(FieldInfo field, string text)
        {
            if (field.FieldType == typeof(string))
                field.SetValue(null, text);

            if (field.FieldType == typeof(int) && int.TryParse(text, out int i))
                field.SetValue(null, i);

            if (field.FieldType == typeof(double) && double.TryParse(text, out double d))
                field.SetValue(null, d);
        }

        private void loadButtonClick(object sender, EventArgs e)
        {
            var res = loadFolderBrowser.ShowDialog();
            if (res == DialogResult.OK)
            {
                activeFolder = loadFolderBrowser.SelectedPath;
                activeFolderLabel.Text = activeFolder.Split("\\").Last();
                Console.WriteLine($"Loaded folder: {activeFolder}");
            }
            reload();
        }

        private void stopButtonClick(object sender, EventArgs e)
        {
            if (cancellationTokenSource != null && !cancellationTokenSource.IsCancellationRequested)
            {
                Console.WriteLine("Stopping");
                cancellationTokenSource.Cancel();
                stopButton.Enabled = false; // Disable stop button immediately
            }
        }

        private async void solveVSPClick(object sender, EventArgs e)
        {
            reload(); // Ensure instance and solver are ready
            if (vspSolver == null) return;

            working = true;
            solveVSPButton.Enabled = false;
            stopButton.Enabled = true;
            cancellationTokenSource = new CancellationTokenSource();
            var token = cancellationTokenSource.Token;

            bool success = false;
            try
            {
                success = await Task.Run(() => vspSolver.Solve(token), token);

                if (success)
                {
                    graphViewer.Graph = vspSolver.GenerateSolutionGraph(blockView);
                    viewToggleButton.Enabled = true;
                    solveCSPButton.Enabled = true;
                    Console.WriteLine("Solver finished successfully.");
                }
                else
                {
                    Console.WriteLine("Solver did not find a solution or was cancelled.");
                }
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("Solver operation was cancelled.");
            }
            finally
            {
                working = false;
                solveVSPButton.Enabled = true;
                stopButton.Enabled = false;
                cancellationTokenSource?.Dispose(); // Dispose the source
                cancellationTokenSource = null;
            }
        }

        private void reload()
        {
            if (activeFolder == "No folder selected") return;

            solveVSPButton.Enabled = true;
            stopButton.Enabled = false;
            viewToggleButton.Enabled = false;
            solveCSPButton.Enabled = false;

            instance = new(activeFolder);
            vspSolver = new EVSPCG(instance);
            Microsoft.Msagl.Drawing.Graph g = new();
            g.AddNode("No graph to show");
            graphViewer.Graph = g;

            string timestamp = DateTime.Now.ToString("yy-MM-dd HH.mm.ss");
            Config.RUN_LOG_FOLDER = $"./runs/{timestamp}/";
            Directory.CreateDirectory(Config.RUN_LOG_FOLDER);

            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
            StringBuilder configDump = new();
            foreach (FieldInfo field in fields)
                configDump.AppendLine($"{field.Name}: {field.GetValue(null)}");
            File.WriteAllText(Config.RUN_LOG_FOLDER + "config.txt", configDump.ToString());
            Console.WriteLine($"Instance reloaded. Current config state dumped to {Config.RUN_LOG_FOLDER + "config.txt"}");
        }

        private void toggleGraphView(object sender, EventArgs e)
        {
            if (vspSolver == null || working) return;
            blockView = !blockView;
            graphViewer.Graph = vspSolver.GenerateSolutionGraph(blockView);
        }

        private async void solveCSPClick(object sender, EventArgs e)
        {
            if (instance == null || instance.Blocks.Count == 0) return;

            cspSolver = new CSPCG(instance);

            working = true;
            solveCSPButton.Enabled = false;
            stopButton.Enabled = true;
            cancellationTokenSource = new CancellationTokenSource();
            var token = cancellationTokenSource.Token;

            bool success = false;
            try
            {
                success = await Task.Run(() => cspSolver.Solve(token), token);

                if (success)
                {
                    graphViewer.Graph = cspSolver.GenerateSolutionGraph(blockView);
                    Console.WriteLine("CSP Solver finished successfully.");
                }
                else
                {
                    Console.WriteLine("Solver did not find a solution or was cancelled.");
                }
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("Solver operation was cancelled.");
            }
            finally
            {
                working = false;
                solveCSPButton.Enabled = true;
                stopButton.Enabled = false;
                cancellationTokenSource?.Dispose(); // Dispose the source
                cancellationTokenSource = null;
            }
        }

        private void loadEVSPResultClick(object sender, EventArgs e)
        {
            var res = openFileDialog1.ShowDialog();
            if (res == DialogResult.OK)
            {
                Console.WriteLine("Starting load of " + openFileDialog1.FileName);
                var dump = JsonSerializer.Deserialize<EVSPCGDump>(File.ReadAllText(openFileDialog1.FileName), new JsonSerializerOptions());
                if (dump == null) throw new InvalidDataException("Dump not valid");

                activeFolder = dump.path;
                reload();

                vspSolver!.LoadFromDump(dump);
                graphViewer.Graph = vspSolver.GenerateSolutionGraph(blockView);
                viewToggleButton.Enabled = true;
                solveCSPButton.Enabled = true;
                Console.WriteLine("Loaded " + openFileDialog1.FileName);
            }
        }
    }
}
