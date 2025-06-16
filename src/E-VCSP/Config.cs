namespace E_VCSP
{
    public class Header { }
    public enum GraphElementDisplay
    {
        None,
        Id,
        Trips,
        Details,
        TripsAndDetails,
    }

    public enum ChargeDisplay
    {
        ChargeOnly,
        ChargeHighlighted,
        All,
    }

    public static class Config
    {
        public static Header PROGRAM = new();
        public static string RUN_LOG_FOLDER = "";

        // Graph display
        public static Header DEBUG = new();
        public static bool CONSOLE_GUROBI = false;
        public static bool CONSOLE_LABELING = false;
        public static bool CONSOLE_LS = false;
        public static bool CONSOLE_COVER = false;

        // General layout
        public static Header GRAPH_LAYOUT = new();
        public static int MAX_NODES_FOR_SHOWN = 150;

        // Nodes
        public static Header NODES = new();
        public static int MIN_NODE_TIME = 300;
        public static int MIN_NODE_WIDTH = 75;
        public static int NODE_HEIGHT = 40;

        public static Header DATA = new();
        public static double KWH_COST = 0.32; // based on national average price
        public static int MIN_CHARGE_TIME = 300;
        public static int MAX_VEHICLES = 5; // Maximum number of vehicles that can be used in the solution.
        public static double VH_M_COST = 0.00005; // verified at qbuzz
        public static double VH_OVER_MAX_COST = 100000; // Cost per vehicle of going over. only used as penalty
        public static double VH_PULLOUT_COST = 200; // Costs to deploy a vehicle at the beginning of the day. verified at qbuzz
        public static double VH_IDLE_COST = 0; // Cost per unit of time for being idle.
        public static int MAX_DRIVE_TIME = 60 * 60 * 4; // conitnous driving time
        public static double CREW_SHIFT_COST = 200; // Minimum price to pay a crew member
        public static double CREW_HOURLY_COST = 65; // hourly cost of employee. based on qbuzz data
        public static double CREW_BROKEN_SHIFT_COST = 20; // additional surcharge for broken shift. based on qbuzz data
        public static int DISCRETE_FACTOR = 5;

        public static Header VSP = new();
        public static bool VSP_USE_CG = true;
        public static bool VSP_DETERMINE_IIS = false;
        public static int VSP_SOLVER_TIMEOUT_SEC = 300;
        public static bool VSP_ALLOW_OVERCOVER = true; // determines >= in constraint
        public static bool VSP_ALLOW_SLACK_FINAL_SOLVE = true; // Allows more vehicles to be used than available during final solve

        public static Header VSP_CG = new();
        public static int VSP_COLS_PER_IT = 10;
        public static int VSP_MAX_COL_GEN_ITS = 100;
        public static int VSP_OPT_IT_THRESHOLD = 10; // Amount of columns sequentially generated without rc before cg is stopped

        public static Header VSP_CG_LABELING = new();
        public static double VSP_LB_WEIGHT = 0;
        public static double VSP_LB_CHARGE_EPSILON = 0.5; // Charge values within epsilon of eachother will be considered the same

        public static Header VSP_CG_LS_SINGLE = new();
        public static double VSP_LS_SINGLE_WEIGHT = 20;
        public static double VSP_LS_S_STARTING_T = 25;
        public static double VSP_LS_S_ENDING_T = 0.01;
        public static double VSP_LS_S_COOLING_RATE = 0.95;
        public static double VSP_LS_S_ITERATIONS = 100_000;
        public static double VSP_LS_S_ADD_TRIP = 20;
        public static double VSP_LS_S_REM_TRIP = 2;
        public static double VSP_LS_S_ADD_CHARGE = 10;
        public static double VSP_LS_S_REMOVE_CHARGE = 1;
        public static double VSP_LS_S_ADD_HNDVR = 5;
        public static double VSP_LS_S_REMOVE_HNDVR = 0.5;

        public static Header VSP_CG_LS_GLOBAL = new();
        public static double VSP_LS_GLOBAL_WEIGHT = 0;
        public static double VSP_LS_G_STARTING_T = 50;
        public static double VSP_LS_G_ENDING_T = 0.01;
        public static double VSP_LS_G_COOLING_RATE = 0.98;
        public static double VSP_LS_G_ITERATIONS = 100_000;
        public static double VSP_LS_G_2OPT = 15;
        public static double VSP_LS_G_MOVE_RANGE = 15;
        public static double VSP_LS_G_CHANGE_CHARGE = 15;

        public static Header CSP = new();
        public static bool CSP_DETERMINE_IIS = false;
        public static int CSP_SOLVER_TIMEOUT_SEC = 300;
        public static bool CSP_ALLOW_OVERCOVER = true; // determines >= in constraint
    }
}
