namespace E_VCSP {
    public class Header { }
    public static class Config {
        // Graph display
        public static Header DEBUG = new();
        public static bool CONSOLE_GUROBI = false;
        public static bool DUMP_VSP = false;
        public static bool DUMP_CSP = false;
        public static int THREAD_COUNT = 10;

        public static Header SOLVER_HINTS = new();
        public static double VH_PULLOUT_COST = 200; // Costs to deploy a vehicle at the beginning of the day. verified at qbuzz
        public static double CR_SHIFT_COST = 100; // Minimum price to pay a crew member
        public static double CR_SINGLE_SHIFT_COST = 1_000; // penalty for use of initial shifts
        public static int MAX_VEHICLES = 18; // Maximum number of vehicles that can be used in the solution.
        public static double VH_OVER_MAX_COST = 5000; // Cost per vehicle of going over. only used as penalty
        public static int MAX_DUTIES = 38; // Maximum number of vehicles that can be used in the solution.
        public static double CR_OVER_MAX_COST = 2000; // Cost per vehicle of going over. only used as penalty

        public static Header VSP = new();
        public static bool VSP_DETERMINE_IIS = false;
        public static int VSP_SOLVER_TIMEOUT_SEC = 60;
        public static bool VSP_ALLOW_OVERCOVER = true; // determines >= in constraint
        public static bool VSP_ALLOW_SLACK_FINAL_SOLVE = true; // Allows more vehicles to be used than available during final solve
        public static int VSP_PRE_DIRECT_TIME = 0;

        public static Header VSP_CG = new();
        public static int VSP_INSTANCES_PER_IT = 1;
        public static int VSP_MAX_COL_GEN_ITS = 25;
        public static int VSP_OPT_IT_THRESHOLD = 1000; // Amount of columns sequentially generated without rc before cg is stopped
        public static string VSP_OPERATION_SEQUENCE = "-"; // Defines an initial operation sequence

        public static Header VSP_LABELING = new();
        public static double VSP_LB_WEIGHT = 1;
        public static int VSP_LB_MAX_COLS = 5;
        public static int VSP_LB_MIN_TRIPS = 3; // min number of trips in a single vehicle task
        public static bool VSP_LB_ATTEMPT_DISJOINT = true;
        public static double VSP_LB_CHARGE_EPSILON = 0.5; // Charge values within epsilon of eachother will be considered the same
        public static double VSP_LB_SEC_COL_COUNT = 5; // Number of primary columns to generate secondary columns for 
        public static double VSP_LB_SEC_COL_ATTEMPTS = 4; // Number of attempts per secondary column

        public static Header VSP_LS_SHARED_PENALTIES = new();
        public static bool VSP_LS_SHR_ALLOW_PENALTY = true;
        public static double VSP_LS_SHR_HNDVR_FIX = 5000;
        public static double VSP_LS_SHR_HNDVR_VAR = 10;
        public static double VSP_LS_SHR_SGNOO_FIX = 5000;
        public static double VSP_LS_SHR_SGNOO_VAR = 10;
        public static double VSP_LS_SHR_CHGDF_FIX = 5000;
        public static double VSP_LS_SHR_CHGDF_VAR = 100;

        public static Header VSP_LS_SINGLE = new();
        public static double VSP_LS_S_WEIGHT = 0;
        public static double VSP_LS_S_STARTING_T = 50;
        public static double VSP_LS_S_ENDING_T = 0.01;
        public static double VSP_LS_S_COOLING_RATE = 0.95;
        public static double VSP_LS_S_ITERATIONS = 500_000;
        public static int VSP_LS_S_NUM_COLS = 5;
        public static double VSP_LS_S_ADD_TRIP = 20;
        public static double VSP_LS_S_REM_TRIP = 2;
        public static double VSP_LS_S_ADD_CHARGE = 10;
        public static double VSP_LS_S_REMOVE_CHARGE = 1;

        public static Header VSP_CG_LS_GLOBAL = new();
        public static double VSP_LS_G_WEIGHT = 0;
        public static double VSP_LS_G_STARTING_T = 50;
        public static double VSP_LS_G_ENDING_T = 0.01;
        public static double VSP_LS_G_COOLING_RATE = 0.98;
        public static double VSP_LS_G_ITERATIONS = 10_000_000;
        public static int VSP_LS_G_NUM_COLS = 10;
        public static double VSP_LS_G_2OPT = 20;
        public static double VSP_LS_G_MOVE_RANGE = 20;
        public static double VSP_LS_G_ADD_CHARGE = 5;
        public static double VSP_LS_G_REMOVE_CHARGE = 1;

        public static Header CSP = new();
        public static bool CSP_DETERMINE_IIS = false;
        public static int CSP_SOLVER_TIMEOUT_SEC = 300;
        public static int CSP_OPT_IT_THRESHOLD = 10; // Amount of columns sequentially generated without rc before cg is stopped
        public static bool CSP_ALLOW_OVERCOVER = true; // determines >= in constraint
        public static int CSP_INSTANCES_PER_IT = 1;
        public static int CSP_MAX_COL_GEN_ITS = 100;

        public static Header CSP_LABELING = new();
        public static double CSP_LABELING_WEIGHT = 0;
        public static int CSP_LB_MAX_LABELS_IN_END = 100_000;
        public static int CSP_LB_MAX_COLS = 50;
        public static bool CSP_LB_ATTEMPT_DISJOINT = true; // try to find disjoint paths; if none can be found, return any arbitrary path
        public static double CSP_LB_SEC_COL_COUNT = 0; // Number of primary columns to generate secondary columns for 
        public static double CSP_LB_SEC_COL_ATTEMPTS = 2; // Number of attempts per secondary column

        public static Header CSP_LS_GLOBAL = new();
        public static double CSP_LS_GLOBAL_WEIGHT = 0;
        public static double CSP_LS_G_STARTING_T = 50;
        public static double CSP_LS_G_ENDING_T = 0.01;
        public static double CSP_LS_G_COOLING_RATE = 0.98;
        public static double CSP_LS_G_ITERATIONS = 10_000_000;
        public static double CSP_LS_G_SWAP_TAILS = 5;
        public static double CSP_LS_G_MOVE_RANGE = 2;
        public static double CSP_LS_G_MOVE_SINGLE = 5;
        public static double CSP_LS_G_CREWHUB_PENALTY = 10_000;

        public static Header VCSP = new();
        public static int VCSP_SOLVER_TIMEOUT_SEC = 300;
        public static int VCSP_ROUNDS = 10;
        public static int VCSP_VH_ITS_INIT = 20;
        public static int VCSP_VH_ITS_ROUND = 10;
        public static int VCSP_CR_ITS_INIT = 20;
        public static int VCSP_CR_ITS_ROUND = 20;
        public static int VCSP_VH_INSTANCES = 1;
        public static int VCSP_CR_INSTANCES = 5;
        public static int VCSP_MAX_TASKS_DURING = 20000;
        public static int VCSP_MAX_DUTIES_DURING = 20000;
        public static bool VCSP_VH_CSTR_SLACK = true;
        public static bool VCSP_CR_MAX_CSTR_SLACK = true;
        public static bool VCSP_CR_OTH_CSTR_SLACK = false;
        public static double VSCP_BLOCK_ADD_CHANCE = 0.1;
        public static bool VCSP_NONNEGATIVE_RC_VSP = true;
        public static bool VCSP_NONNEGATIVE_RC_CSP = true;

        public static Header LAGRANGE = new();
        public static double LAGRANGE_PI_START = 1; // (0, 2]
        public static double LAGRANGE_PI_END = 0.001; // (0, 2]
        public static double LANGRANGE_THRS = 0.01; // % diff between two sequential solutions to be considered "done"
        public static int LANGRANGE_THRS_SEQ = 5; // number of sequential rounds within threshold to be considered done
        public static int LANGRANGE_MAX_ROUNDS = 5000; // number of sequential rounds within threshold to be considered done
        public static int LAGRANGE_DISRUPTION_ROUNDS = 1;
        public static double LAGRANGE_DISRUPT_LWR = 0.9;
        public static double LAGRANGE_DISRUPT_UPR = 1.1;
    }
}
