namespace E_VCSP
{
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
        internal static bool FIX_TIME_AXIS = true;

        // Nodes
        internal static bool NODE_SHOWN = true;
        internal static GraphElementDisplay NODE_LABEL = GraphElementDisplay.TripsAndDetails;
        internal static int NODE_MIN_WIDTH = 150;
        internal static double NODE_WIDTH_SCALAR = 0.1;
        internal static int NODE_HEIGHT = 40;

        // Edges
        internal static bool EDGE_SHOWN = true;
        internal static GraphElementDisplay EDGE_LABEL = GraphElementDisplay.TripsAndDetails;
        internal static int EDGE_MIN_TIME_SHOWN = 0;
        internal static int EDGE_MAX_TIME_SHOWN = 1 * 60 * 60;
        internal static ChargeDisplay EDGE_CHARGE_SHOWN = ChargeDisplay.ChargeHighlighted;
        internal static bool EDGE_INCLUDE_NOT_SHOWN = false;

        internal static double KWH_COST = 0.32;
        internal static double KM_COST = 0.01; // TODO: actual value
        internal static double DEPLOY_COST = 10; // Costs to deploy a vehicle at the beginning of the day.
    }
}
