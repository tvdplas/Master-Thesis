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
        public static int THREADS = 1;

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
        public static double KWH_COST = 0.32;
        public static double M_COST = 0.00001; // TODO: actual value
        public static double PULLOUT_COST = 10; // Costs to deploy a vehicle at the beginning of the day.
        public static double IDLE_COST = 0; // Cost per unit of time for being idle.
        public static int MAX_DRIVE_TIME = 60 * 60 * 4; // conitnous driving time
        public static int MAX_VEHICLES = 6; // Maximum number of vehicles that can be used in the solution.
        public static int MAX_VEHICLES_OVER_COST = 1000; // Cost per vehicle of going over.
        public static int DISCRETE_FACTOR = 5;
        public static int MIN_CHARGE_TIME = 300;

        public static Header SOLVER = new();
        public static int SOLVER_TIMEOUT_SEC = 300;
        public static bool DETERMINE_IIS = false;
        public static bool ALLOW_VH_OVERCOVER = true; // determines >= in constraint
        public static bool ALLOW_VH_SLACK_FINAL_SOLVE = true; // Allows more vehicles to be used than available during final solve
        public static bool USE_COLUMN_GENERATION = true;
        public static int MAX_COL_GEN_ITS = 5000;
        public static double VSP_LS_S_CHANCE = 0;
        public static int OPT_IT_THRESHOLD = 10; // Amount of columns sequentially generated without rc before cg is stopped

        public static Header VSP_LABELING = new();
        public static double VSP_LB_WEIGHT = 0;
        public static double VSP_LB_CHARGE_EPSILON = 0.5; // Charge values within epsilon of eachother will be considered the same

        public static Header VSP_LS_SINGLE = new();
        public static double VSP_LS_SINGLE_WEIGHT = 20;
        public static double VSP_LS_S_STARTING_T = 2;
        public static double VSP_LS_S_ENDING_T = 0.01;
        public static double VSP_LS_S_COOLING_RATE = 0.95;
        public static double VSP_LS_S_ITERATIONS = 10_000;
        public static double VSP_LS_S_ADD_TRIP = 20;
        public static double VSP_LS_S_REM_TRIP = 2;
        public static double VSP_LS_S_ADD_CHARGE = 10;
        public static double VSP_LS_S_REMOVE_CHARGE = 1;
        public static double VSP_LS_S_ADD_HNDVR = 5;
        public static double VSP_LS_S_REMOVE_HNDVR = 0.5;

        public static Header VSP_LS_GLOBAL = new();
        public static double VSP_LS_GLOBAL_WEIGHT = 0;
        public static double VSP_LS_G_STARTING_T = 50;
        public static double VSP_LS_G_ENDING_T = 0.01;
        public static double VSP_LS_G_COOLING_RATE = 0.98;
        public static double VSP_LS_G_ITERATIONS = 100_000;
        public static double VSP_LS_G_2OPT = 15;
        public static double VSP_LS_G_MOVE_RANGE = 15;
        public static double VSP_LS_G_CHANGE_CHARGE = 15;
    }
}
