namespace E_VCSP
{
    internal class Header { }
    internal enum GraphElementDisplay
    {
        None,
        Id,
        Trips,
        Details,
        TripsAndDetails,
    }

    internal enum ChargeDisplay
    {
        ChargeOnly,
        ChargeHighlighted,
        All,
    }

    internal static class Config
    {
        // Graph display
        // General layout
        internal static Header GRAPH_LAYOUT = new();
        internal static bool GRAPH_SHOWN = true;
        internal static int MAX_NODES_FOR_SHOWN = 150;
        internal static bool FIX_TIME_AXIS = true;

        // Nodes
        internal static Header NODES = new();
        internal static bool NODE_SHOWN = true;
        internal static GraphElementDisplay NODE_LABEL = GraphElementDisplay.TripsAndDetails;
        internal static int NODE_MIN_WIDTH = 75;
        internal static double NODE_WIDTH_SCALAR = 0.1;
        internal static int NODE_HEIGHT = 40;

        // Edges
        internal static Header EDGES = new();
        internal static bool EDGE_SHOWN = true;
        internal static GraphElementDisplay EDGE_LABEL = GraphElementDisplay.TripsAndDetails;
        internal static int EDGE_MIN_TIME_SHOWN = 0;
        internal static int EDGE_MAX_TIME_SHOWN = 1 * 60 * 60;
        internal static ChargeDisplay EDGE_CHARGE_SHOWN = ChargeDisplay.ChargeHighlighted;
        internal static bool EDGE_INCLUDE_NOT_SHOWN = false;

        internal static Header DATA = new();
        internal static double KWH_COST = 0.32;
        internal static double M_COST = 0.00001; // TODO: actual value
        internal static double PULLOUT_COST = 10; // Costs to deploy a vehicle at the beginning of the day.
        internal static int MAX_VEHICLES = 6; // Maximum number of vehicles that can be used in the solution.
        internal static int DISCRETE_FACTOR = 5;

        internal static Header SOLVER = new();
        internal static bool DETERMINE_IIS = false;
    }
}
