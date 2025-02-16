
using E_VCSP.Objects;
using E_VCSP.Parsing;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Drawing;
using Microsoft.Msagl.Layout.Layered;

namespace E_VCSP
{
    public partial class MainView : Form
    {
        public string activeFolder = "No folder selected";


        Microsoft.Msagl.Drawing.Graph graph;

        List<Trip> trips;
        List<Deadhead> deadheads;

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
                parseAll();
                refreshGraph();
            }
        }

        private string formatTime(int seconds, bool showHours = true)
        {
            string res = "";
            res += showHours ? (seconds / 3600).ToString("00") : "";
            res += ":" + ((seconds % 3600) / 60).ToString("00");
            res += ":" + (seconds % 60).ToString("00");
            return res;
        }

        private void parseAll()
        {
            // Clear previous data
            // No data to clear yet except objects

            // Parse new content
            trips = new ParserTrips().Parse(activeFolder);
            deadheads = new ParserDeadheads().Parse(activeFolder);
        }

        private void refreshGraph()
        {
            // Reset graph
            graph = new Microsoft.Msagl.Drawing.Graph("graph");

            List<Node> tripNodes = new();
            // Trips
            foreach (Trip t in trips)
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
                    LabelText = $"{t.FromLocation} -> {t.ToLocation}\n{formatTime(t.StartTime)}-{formatTime(t.EndTime)}",
                };

                tripNodes.Add(node);
                graph.AddNode(node);
            }

            graph.LayoutAlgorithmSettings = new SugiyamaLayoutSettings()
            {
                LiftCrossEdges = true,
            };

            for (int i = 0; i < trips.Count; i++)
            {
                Trip t1 = trips[i];
                for (int j = 0; j < trips.Count; j++)
                {
                    if (i == j) continue;

                    Trip t2 = trips[j];

                    var dh = deadheads
                        .Where(dh => dh.FromLocation == t1.ToLocation && dh.ToLocation == t2.FromLocation)
                        .FirstOrDefault();

                    if (dh == null)
                    {
                        // No deadhead possible
                        continue;
                    }

                    // feasible and not unreasonable long wait
                    if (t1.EndTime + dh.Duration <= t2.StartTime && t2.StartTime - t1.EndTime < 10_000)
                    {
                        tripNodes[i].AddOutEdge(new Edge(tripNodes[i], tripNodes[j], ConnectionToGraph.Connected));
                    }
                }
            }

            graph.LayerConstraints.RemoveAllConstraints();
            for (int i = 0; i < trips.Count; i++)
            {
                Trip t1 = trips[i];
                for (int j = 0; j < trips.Count; j++)
                {
                    Trip t2 = trips[j];

                    if (t1.EndTime <= t2.StartTime)
                    {
                        graph.LayerConstraints.AddLeftRightConstraint(tripNodes[i], tripNodes[j]);
                    }
                }
            }

            graph.Directed = true;
            // Enable graph in viewer
            graphViewer.Graph = graph;
        }
    }
}
