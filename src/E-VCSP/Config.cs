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
        internal static Header PROGRAM = new();
        internal static string RUN_LOG_FOLDER = "";
        internal static int THREADS = 1;

        // Graph display
        internal static Header DEBUG = new();
        internal static bool CONSOLE_GUROBI = false;
        internal static bool CONSOLE_LABELING = false;
        internal static bool CONSOLE_LS = false;
        internal static bool CONSOLE_COVER = false;

        // General layout
        internal static Header GRAPH_LAYOUT = new();
        internal static int MAX_NODES_FOR_SHOWN = 150;

        // Nodes
        internal static Header NODES = new();
        internal static int MIN_NODE_TIME = 300;
        internal static int MIN_NODE_WIDTH = 75;
        internal static int NODE_HEIGHT = 40;

        internal static Header DATA = new();
        internal static double KWH_COST = 0.32;
        internal static double M_COST = 0.00001; // TODO: actual value
        internal static double PULLOUT_COST = 10; // Costs to deploy a vehicle at the beginning of the day.
        internal static double IDLE_COST = 0; // Cost per unit of time for being idle.
        internal static int MAX_VEHICLES = 6; // Maximum number of vehicles that can be used in the solution.
        internal static int MAX_VEHICLES_OVER_COST = 1000; // Cost per vehicle of going over.
        internal static int DISCRETE_FACTOR = 5;
        internal static int MIN_CHARGE_TIME = 300;

        internal static Header SOLVER = new();
        internal static int SOLVER_TIMEOUT_SEC = 300;
        internal static bool DETERMINE_IIS = false;
        internal static bool ALLOW_VH_OVERCOVER = true; // determines >= in constraint
        internal static bool ALLOW_VH_SLACK_FINAL_SOLVE = true; // Allows more vehicles to be used than available during final solve
        internal static bool USE_COLUMN_GENERATION = true;
        internal static int MAX_COL_GEN_ITS = 5000;
        internal static double VSP_LS_S_CHANCE = 0;
        internal static double CHARGE_EPSILON = 0.5; // Charge values within epsilon of eachother will be considered the same
        internal static int OPT_IT_THRESHOLD = 10; // Amount of columns sequentially generated without rc before cg is stopped

        internal static Header VSP_LS_SINGLE = new();
        internal static double VSP_LS_S_STARTING_T = 2;
        internal static double VSP_LS_S_ENDING_T = 0.01;
        internal static double VSP_LS_S_COOLING_RATE = 0.95;
        internal static double VSP_LS_S_ITERATIONS = 10_000;
        internal static double VSP_LS_S_OVERCHARGE_PENALTY_FIX = 5;
        internal static double VSP_LS_S_OVERCHARGE_PENALTY_VAR = 0.5;
        internal static double VSP_LS_S_UNDERCHARGE_PENALTY_FIX = 5;
        internal static double VSP_LS_S_UNDERCHARGE_PENALTY_VAR = 0.5;
        internal static double VSP_LS_S_ADD_TRIP = 15;
        internal static double VSP_LS_S_REM_TRIP = 1;
        internal static double VSP_LS_S_CHANGE_CHARGE = 5;

        internal static Header VSP_LS_GLOBAL = new();
        internal static double VSP_LS_G_STARTING_T = 2;
        internal static double VSP_LS_G_ENDING_T = 0.01;
        internal static double VSP_LS_G_COOLING_RATE = 0.95;
        internal static double VSP_LS_G_ITERATIONS = 10_000;
        internal static double VSP_LS_G_2OPT = 15;
        internal static double VSP_LS_G_MOVE_RANGE = 15;
        internal static double VSP_LS_G_CHANGE_CHARGE = 15;
    }
}
