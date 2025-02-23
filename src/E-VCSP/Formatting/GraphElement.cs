using E_VCSP.Objects;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Drawing;
using Color = Microsoft.Msagl.Drawing.Color;

namespace E_VCSP.Formatting
{
    internal static class GraphElement
    {
        internal static Node TripNode(Trip t)
        {
            Node node = new Node(t.Id)
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
                    double width = Math.Max(t.Duration * Config.NODE_WIDTH_SCALAR, Config.NODE_MIN_WIDTH);  // Custom width
                    double height = Config.NODE_HEIGHT; // Custom height

                    // Create a rounded rectangle as an example
                    return CurveFactory.CreateRectangleWithRoundedCorners(
                        width,
                        height,
                        0.3,
                        0.3,
                        new Microsoft.Msagl.Core.Geometry.Point(0, 0)
                    );
                },
                LabelText = Config.NODE_LABEL switch
                {
                    GraphElementDisplay.Id => t.ToString(),
                    GraphElementDisplay.Trips => t.ToLongString(false),
                    GraphElementDisplay.TripsAndDetails => t.ToLongString(true),
                    _ => "",
                }
            };

            return node;
        }

        internal static bool ShouldHideEdge(Deadhead dh)
        {
            if (!Config.EDGE_SHOWN) return true;

            if (dh.To.StartTime - dh.From.EndTime > Config.EDGE_MAX_TIME_SHOWN) return true;
            if (dh.To.StartTime - dh.From.EndTime < Config.EDGE_MIN_TIME_SHOWN) return true;

            if (Config.EDGE_CHARGE_SHOWN == ChargeDisplay.ChargeOnly && dh.MaxCharge == 0) return true;

            return false;
        }
        internal static Edge DeadheadEdge(Deadhead dh, Graph graph)
        {
            Edge edge = new Edge(graph.FindNode(dh.From.Id), graph.FindNode(dh.To.Id), ConnectionToGraph.Connected)
            {
                DrawEdgeDelegate = (_, _) =>
                {
                    return ShouldHideEdge(dh);
                },
                LabelText = Config.EDGE_LABEL switch
                {
                    GraphElementDisplay.Id => dh.ToFormattedString("[i]"),
                    GraphElementDisplay.Trips => dh.ToFormattedString("[f] -> [t]"),
                    GraphElementDisplay.Details => dh.ToFormattedString("[s] / [m]"),
                    GraphElementDisplay.TripsAndDetails => dh.ToFormattedString("[f] -> [t]\n[s] / [m]"),
                    _ => ""
                },
                Attr =
                {
                    Color = Config.EDGE_CHARGE_SHOWN == ChargeDisplay.ChargeHighlighted
                        ? (dh.MaxCharge > 0 ? Color.Black : Color.Gray)
                        : Color.Black
                }
            };
            return edge;
        }
    }
}
