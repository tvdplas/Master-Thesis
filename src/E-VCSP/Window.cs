
using E_VCSP.Formatting;
using E_VCSP.Objects;
using E_VCSP.Solver;
using System.Reflection;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        private Dictionary<string, Control> controlsMap = new();
        public string activeFolder = "No folder selected";

        Instance? instance;
        Solver.Solver? solver;

        public MainView()
        {
            InitializeComponent();
            Console.SetOut(new TextBoxWriter(textBox1));
            Console.WriteLine("Program started");

            activeFolderLabel.Text = activeFolder;
            GenerateConfigUI();
        }

        private void GenerateConfigUI()
        {
            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            Panel scrollablePanel = new Panel
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
                System.Windows.Forms.Label label = new System.Windows.Forms.Label
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
                    TextBox textBox = new TextBox
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
                    ComboBox comboBox = new ComboBox
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
                    CheckBox check = new CheckBox
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

            Button applyButton = new Button
            {
                Text = "Apply Changes",
                Location = new System.Drawing.Point(10, yOffset + 10),
                Width = 200
            };
            applyButton.Click += (sender, e) => reload();
            scrollablePanel.Controls.Add(applyButton);
        }

        private void UpdateString(FieldInfo field, string text)
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

        private void solveButtonClick(object sender, EventArgs e)
        {
            solve();
        }

        private void solve()
        {
            reload();
            if (solver == null) return;

            if (solver.Solve())
            {
                graphViewer.Graph = solver.GenerateSolutionGraph();
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
            Console.WriteLine("Instance reloaded");
        }
    }
}
