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
        internal static int MAX_NODES_FOR_SHOWN = 150;

        // Nodes
        internal static Header NODES = new();
        internal static int NODE_MIN_WIDTH = 75;
        internal static int NODE_HEIGHT = 40;

        internal static Header DATA = new();
        internal static double KWH_COST = 0.32;
        internal static double M_COST = 0.00001; // TODO: actual value
        internal static double PULLOUT_COST = 10; // Costs to deploy a vehicle at the beginning of the day.
        internal static int MAX_VEHICLES = 6; // Maximum number of vehicles that can be used in the solution.
        internal static int DISCRETE_FACTOR = 5;
        internal static int MIN_CHARGE_TIME = 300;

        internal static Header SOLVER = new();
        internal static bool DETERMINE_IIS = false;
    }
}
