using E_VCSP.Objects.Discrete;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;


namespace E_VCSP.Formatting
{
    internal static class GraphElement
    {
        static Random r = new();

        internal static Node? ScheduleNode(int startTime, int endTime, string content, Color color)
        {
            // No node displayed
            if ((endTime - startTime) / Config.MIN_NODE_TIME * Config.MIN_NODE_WIDTH < 5)
            {
                return null;
            }
            // Node with no text
            else if (endTime - startTime < Config.MIN_NODE_TIME)
            {
                return new Node($"{startTime}-{endTime}-{content}-{r.Next()}")
                {
                    Attr = {
                        FillColor = color,
                    },
                    UserData = (startTime, endTime),
                    NodeBoundaryDelegate = (Node node) =>
                    {
                        // We assume that the minimum amount of time represented is 5 min
                        // So everything must scale according to this 
                        double widthPerSecond = Config.MIN_NODE_WIDTH / 300.0;
                        double width = (endTime - startTime) * widthPerSecond;
                        double height = Config.NODE_HEIGHT;

                        return CurveFactory.CreateRectangle(width, height, new(0, 0));
                    },
                };
            }
            // Normal node
            else
            {
                return new Node($"{startTime}-{endTime}-{content}-{r.Next()}")
                {
                    Attr = {
                    LabelMargin = 5,
                    FillColor = color,
                },
                    Label = {
                    FontSize = content.Length > 0 ? Math.Min(13, Math.Max(4, (endTime - startTime) * (Config.MIN_NODE_WIDTH / 12000.0))) : 12,
                },
                    LabelText = $"{content}\n{Formatting.Time.HHMMSS(startTime)}-{Formatting.Time.HHMMSS(endTime)}",
                    UserData = (startTime, endTime),
                    NodeBoundaryDelegate = (Node node) =>
                    {
                        // We assume that the minimum amount of time represented is 5 min
                        // So everything must scale according to this 
                        double widthPerSecond = Config.MIN_NODE_WIDTH / 300.0;
                        double width = (endTime - startTime) * widthPerSecond;
                        double height = Config.NODE_HEIGHT;

                        return CurveFactory.CreateRectangle(width, height, new(0, 0));
                    },
                };
            }
        }
        internal static Node TripNode(DTrip t)
        {
            Node node = new Node(t.Id.ToString())
            {
                Attr = {
                    LabelMargin = 5,
                },
                NodeBoundaryDelegate = (Node node) =>
                {
                    double widthPerSecond = Config.MIN_NODE_WIDTH / 300.0;
                    double width = t.Trip.Duration * widthPerSecond;
                    double height = Config.NODE_HEIGHT;

                    return CurveFactory.CreateRectangleWithRoundedCorners(
                        width,
                        height,
                        0.3,
                        0.3,
                        new Microsoft.Msagl.Core.Geometry.Point(0, 0)
                    );
                },
                LabelText = t.StartingSoC + t.Trip.ToLongString(true)
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
