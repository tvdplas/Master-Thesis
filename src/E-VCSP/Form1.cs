
using E_VCSP.Objects;
using E_VCSP.Parsing;
using Microsoft.Msagl.Drawing;

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







            //Microsoft.Msagl.GraphViewerGdi.GViewer viewer = new Microsoft.Msagl.GraphViewerGdi.GViewer();
            //Microsoft.Msagl.Drawing.Graph graph = new Microsoft.Msagl.Drawing.Graph("graph");

            ////create the graph content 
            //graph.AddEdge("A", "B");
            //graph.AddEdge("B", "C");
            //graph.AddEdge("A", "C").Attr.Color = Microsoft.Msagl.Drawing.Color.Green;
            //graph.FindNode("A").Attr.FillColor = Microsoft.Msagl.Drawing.Color.Magenta;
            //graph.FindNode("B").Attr.FillColor = Microsoft.Msagl.Drawing.Color.MistyRose;
            //Microsoft.Msagl.Drawing.Node c = graph.FindNode("C");
            //c.Attr.FillColor = Microsoft.Msagl.Drawing.Color.PaleGreen;
            //c.Attr.Shape = Microsoft.Msagl.Drawing.Shape.Diamond;
            ////bind the graph to the viewer 
            //viewer.Graph = graph;

            //SuspendLayout();
            //viewer.Dock = System.Windows.Forms.DockStyle.Fill;
            //Controls.Add(viewer);
            //ResumeLayout();
            //associate the viewer with the form 
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
                tripNodes.Add(graph.AddNode(t.Id));
            }

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
                    if (t1.EndTime + dh.Duration <= t2.StartTime && t2.StartTime - t1.EndTime < 7200)
                    {
                        tripNodes[i].AddOutEdge(new Edge(tripNodes[i], tripNodes[j], ConnectionToGraph.Connected));
                    }
                }
            }

            // Enable graph in viewer
            graphViewer.Graph = graph;
        }
    }
}
