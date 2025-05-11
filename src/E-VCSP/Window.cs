
using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver;
using System.Reflection;
using System.Text;
using MethodInvoker = System.Windows.Forms.MethodInvoker;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        private readonly Dictionary<string, Control> controlsMap = [];
        public string activeFolder = "No folder selected";

        Instance? instance;
        Solver.Solver? solver;

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
                Location = new System.Drawing.Point(10, 70),
                Size = new System.Drawing.Size(280, 470),
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
                reload();
            }
        }

        private void stopButtonClick(object sender, EventArgs e) // <<< Add this method
        {
            if (cancellationTokenSource != null && !cancellationTokenSource.IsCancellationRequested)
            {
                Console.WriteLine("Stop button clicked. Requesting cancellation...");
                cancellationTokenSource.Cancel();
                stopButton.Enabled = false; // Disable stop button immediately
            }
        }

        private async void solveButtonClick(object sender, EventArgs e) // <<< Change to async void
        {
            reload(); // Ensure instance and solver are ready
            if (solver == null) return;

            solveButton.Enabled = false;
            stopButton.Enabled = true;
            cancellationTokenSource = new CancellationTokenSource();
            var token = cancellationTokenSource.Token;

            bool success = false;
            try
            {
                // Run the solver on a background thread
                success = await Task.Run(() => solver.Solve(token), token); // <<< Pass token

                if (success)
                {
                    // Update UI thread safely
                    graphViewer.Invoke((System.Windows.Forms.MethodInvoker)delegate
                    {
                        graphViewer.Graph = solver.GenerateSolutionGraph();
                    });
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
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred during solving: {ex.Message}");
                // Optionally show more details or log the exception
            }
            finally
            {
                // Ensure UI is updated even if errors occur
                if (this.IsHandleCreated) // Check if the form handle still exists
                {
                    this.Invoke((MethodInvoker)delegate
                    {
                        solveButton.Enabled = true;
                        stopButton.Enabled = false;
                        cancellationTokenSource?.Dispose(); // Dispose the source
                        cancellationTokenSource = null;
                    });
                }
            }
        }

        private void reload()
        {
            if (activeFolder == "No folder selected") return;

            instance = new(activeFolder);
            solver = Config.USE_COLUMN_GENERATION ? new EVSPCG(instance) : new EVSPDiscrete(instance);
            if (instance.Trips.Count * Config.DISCRETE_FACTOR < Config.MAX_NODES_FOR_SHOWN && solver is EVSPDiscrete sd)
            {
                graphViewer.Graph = sd.DGraph.GenerateDiscreteGraph();
            }
            else
            {
                Microsoft.Msagl.Drawing.Graph g = new();
                g.AddNode("Graph not shown");
                graphViewer.Graph = g;
            }

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
    }
}
