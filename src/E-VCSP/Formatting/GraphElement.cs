using E_VCSP.Solver;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;


namespace E_VCSP.Formatting
{
    internal static class GraphElement
    {
        internal static Node ScheduleNode(int startTime, int endTime, string content, Color color, int offset)
        {
            Node node = new Node($"{startTime}-{endTime}-{content}")
            {
                Attr = {
                    LabelMargin = 5,
                    FillColor = color,
                },
                Label = {

                    FontSize = Math.Min(13, Math.Max(4, (endTime - startTime) * (Config.NODE_MIN_WIDTH / 12000.0))),
                },
                UserData = (startTime, endTime),
                NodeBoundaryDelegate = (Node node) =>
                {
                    // We assume that the minimum amount of time represented is 5 min
                    // So everything must scale according to this 
                    double widthPerSecond = Config.NODE_MIN_WIDTH / 300.0;
                    double width = (endTime - startTime) * widthPerSecond;
                    double height = Config.NODE_HEIGHT;

                    return CurveFactory.CreateRectangleWithRoundedCorners(
                        width,
                        height,
                        0.3,
                        0.3,
                        new Microsoft.Msagl.Core.Geometry.Point(0, 0)
                    );
                },
                LabelText = $"{content}\n{Formatting.Time.HHMMSS(startTime)}-{Formatting.Time.HHMMSS(endTime)}",
            };

            return node;
        }
        internal static Node TripNode(DiscreteTrip t)
        {
            Node node = new Node(t.Id.ToString())
            {
                Attr = {
                    LabelMargin = 5,
                },
                DrawNodeDelegate = (Node _node, object _g) =>
                {
                    return !Config.NODE_SHOWN;
                },
                NodeBoundaryDelegate = (Node node) =>
                {
                    double width = Math.Max(t.Trip.Duration * Config.NODE_WIDTH_SCALAR, Config.NODE_MIN_WIDTH);
                    double height = Config.NODE_HEIGHT;

                    return CurveFactory.CreateRectangleWithRoundedCorners(
                        width,
                        height,
                        0.3,
                        0.3,
                        new Microsoft.Msagl.Core.Geometry.Point(0, 0)
                    );
                },
                LabelText = $"{t.StartingSoC}% {Config.NODE_LABEL switch
                {
                    GraphElementDisplay.Id => t.ToString(),
                    GraphElementDisplay.Trips => t.Trip.ToLongString(false),
                    GraphElementDisplay.TripsAndDetails => t.Trip.ToLongString(true),
                    _ => "",
                }}"
            };

            return node;
        }
        internal static Edge DeadheadEdge(string from, string to, double cost, Graph graph)
        {
            Edge edge = new Edge(graph.FindNode(from), graph.FindNode(to), ConnectionToGraph.Connected);
            return edge;
        }
    }
}
