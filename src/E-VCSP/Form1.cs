
using E_VCSP.Objects;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Drawing;
using Microsoft.Msagl.Layout.Layered;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        public string activeFolder = "No folder selected";

        Graph? graph;
        Instance? instance;

        public MainView()
        {
            InitializeComponent();
        }

        private void loadButtonClick(object sender, EventArgs e)
        {
            var res = loadFolderBrowser.ShowDialog();
            if (res == DialogResult.OK)
            {
                activeFolder = loadFolderBrowser.SelectedPath;
                activeFolderLabel.Text = activeFolder.Split("\\").Last();
                instance = new Instance(activeFolder);
                refreshGraph();
            }
        }

        private void refreshGraph()
        {
            if (instance == null)
            {
                throw new InvalidOperationException("Cannot refresh graph if no instance is loaded");
            }

            // Reset graph
            graph = new Microsoft.Msagl.Drawing.Graph("graph");

            List<Node> tripNodes = new();
            // Trips
            foreach (Trip t in instance.trips)
            {
                Node node = new Node(t.Id)
                {
                    Attr = {
                        LabelMargin = 5,
                    },
                    NodeBoundaryDelegate = (Node node) =>
                    {
                        double width = t.Duration / 10;  // Custom width
                        double height = 40; // Custom height

                        // Create a rounded rectangle as an example
                        return CurveFactory.CreateRectangleWithRoundedCorners(
                            width,
                            height,
                            0.3,
                            0.3,
                            new Microsoft.Msagl.Core.Geometry.Point(0, 0)
                        );
                    },
                    LabelText = t.ToLongString(true),
                };

                tripNodes.Add(node);
                graph.AddNode(node);
            }

            graph.LayoutAlgorithmSettings = new SugiyamaLayoutSettings()
            {
                LiftCrossEdges = true,
            };

            for (int i = 0; i < instance.trips.Count; i++)
            {
                Trip t1 = instance.trips[i];
                for (int j = 0; j < instance.trips.Count; j++)
                {
                    if (i == j) continue;

                    Trip t2 = instance.trips[j];

                    var dh = instance.deadheads
                        .Where(dh => dh.From == t1.To && dh.To == t2.From)
                        .FirstOrDefault();

                    var dhPossible =
                        t2.StartTime - t1.EndTime >= 0 && t2.StartTime - t1.EndTime <= 0.5 * 60 * 60 && (t1.To == t2.From || (dh != null && t1.EndTime + dh.Duration <= t2.StartTime));


                    if (!dhPossible)
                    {
                        // No deadhead possible
                        continue;
                    }
                    else
                    {
                        Edge edge = new Edge(tripNodes[i], tripNodes[j], ConnectionToGraph.Connected);
                        edge.Attr.AddStyle(Style.Rounded);
                        tripNodes[i].AddOutEdge(edge);
                    }
                }
            }

            graph.LayerConstraints.RemoveAllConstraints();
            for (int i = 0; i < instance.trips.Count; i++)
            {
                Trip t1 = instance.trips[i];
                for (int j = 0; j < instance.trips.Count; j++)
                {
                    Trip t2 = instance.trips[j];

                    if (t1.EndTime <= t2.StartTime)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(tripNodes[i], tripNodes[j]);
                    }
                }
            }

            graph.Directed = true;
            graphViewer.Graph = graph;
        }
    }
}
