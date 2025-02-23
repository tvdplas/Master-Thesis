
using E_VCSP.Objects;
using Microsoft.Msagl.Drawing;
using Microsoft.Msagl.Layout.Layered;
using System.Reflection;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        private Dictionary<string, Control> controlsMap = new();
        public string activeFolder = "No folder selected";

        Graph? graph;
        Instance? instance;

        public MainView()
        {
            InitializeComponent();
            activeFolderLabel.Text = activeFolder;
            GenerateConfigUI();
        }

        private void GenerateConfigUI()
        {
            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            int yOffset = 40;
            foreach (FieldInfo field in fields)
            {
                System.Windows.Forms.Label label = new System.Windows.Forms.Label
                {
                    Text = field.Name.Replace('_', ' ').ToLower(),
                    AutoSize = true,
                    Location = new System.Drawing.Point(10, yOffset)
                };
                Controls.Add(label);

                Control inputControl;

                if (field.FieldType == typeof(int)
                || field.FieldType == typeof(double)
                || field.FieldType == typeof(string))
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

                Controls.Add(inputControl);
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
            Controls.Add(applyButton);
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
                reload();
            }
        }

        private void reload()
        {
            if (activeFolder == "No folder selected") return;

            instance = new Instance(activeFolder);
            refreshGraph();
        }

        private void refreshGraph()
        {
            if (instance == null)
            {
                throw new InvalidOperationException("Cannot refresh graph if no instance is loaded");
            }

            // Reset graph
            graph = new Microsoft.Msagl.Drawing.Graph("graph");
            graph.LayoutAlgorithmSettings = new SugiyamaLayoutSettings()
            {
                LiftCrossEdges = true,
            };

            foreach (Trip t in instance.Trips)
            {
                Node node = Formatting.GraphElement.TripNode(t);
                graph.AddNode(node);
            }
            foreach (Deadhead dh in instance.Deadheads)
            {
                // Dont include edges even if their not show to save space
                if (!Formatting.GraphElement.ShouldHideEdge(dh) || Config.EDGE_INCLUDE_NOT_SHOWN)
                {
                    Edge edge = Formatting.GraphElement.DeadheadEdge(dh, graph);
                    graph.AddPrecalculatedEdge(edge);
                }
            }

            graph.LayerConstraints.RemoveAllConstraints();

            for (int i = 0; i < instance.Trips.Count; i++)
            {
                Trip t1 = instance.Trips[i];
                for (int j = 0; j < instance.Trips.Count; j++)
                {
                    Trip t2 = instance.Trips[j];

                    if (t1.EndTime <= t2.StartTime)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(graph.FindNode(t1.Id), graph.FindNode(t2.Id));
                    }
                }
            }

            graph.Directed = true;
            graphViewer.Graph = graph;
        }
    }
}
