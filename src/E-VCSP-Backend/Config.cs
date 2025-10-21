
using System.Reflection;
using System.Text;

namespace E_VCSP {
    public class Header { }
    public static class Config {
        // Graph display
        public static Header DEBUG = new();
        public static bool GLOBAL_CONSOLE_KILL = false;
        public static string CNSL_OVERRIDE = "[CNSL]";
        public static bool CONSOLE_GUROBI = false;
        public static bool DUMP_VSP = false;
        public static bool DUMP_CSP = false;
        public static int THREAD_COUNT = Environment.ProcessorCount - 1;

        public static Header GUROBI = new();
        public static bool GRB_EXTEND_TIME = false;
        public static int GRB_MAX_EXTENDED_TIME = 60 * 60 * 2;
        public static double GRB_TARGET_GAP = 0.005;

        public static Header SOLVER_HINTS = new();
        public static double VH_PULLOUT_COST = 200; // Costs to deploy a vehicle at the beginning of the day. verified at qbuzz
        public static double CR_SHIFT_COST = 100; // Minimum price to pay a crew member
        public static double CR_SINGLE_SHIFT_COST = 1_000; // penalty for use of initial shifts
        public static int MAX_VEHICLES = 99; // Maximum number of vehicles that can be used in the solution.
        public static double VH_OVER_MAX_COST = 0; // Cost per vehicle of going over. only used as penalty
        public static int MAX_DUTIES = 99; // Maximum number of vehicles that can be used in the solution.
        public static double CR_OVER_MAX_COST = 0; // Cost per vehicle of going over. only used as penalty

        public static Header VSP = new();
        public static bool VSP_DETERMINE_IIS = false;
        public static bool VSP_POSTPROCESS = false;
        public static int VSP_SOLVER_TIMEOUT_SEC = 450;
        public static double VSP_SOLVER_HEURISTIC_FRAC = 0.25;
        public static bool VSP_ALLOW_OVERCOVER = true; // determines >= in constraint
        public static bool VSP_ALLOW_SLACK_FINAL_SOLVE = true; // Allows more vehicles to be used than available during final solve
        public static int VSP_PRE_DIRECT_TIME = 0;
        public static bool VSP_ALLOW_UNKNOWN_DHTS = true; // allow the use of estimated deadheads during vsp solving

        public static Header VSP_CG = new();
        public static int VSP_INSTANCES_PER_IT = 1;
        public static int VSP_MAX_COL_GEN_ITS = 250;
        public static int VSP_OPT_IT_THRESHOLD = 2; // Amount of columns sequentially generated without rc before cg is stopped
        public static string VSP_OPERATION_SEQUENCE = "-"; // Defines an initial operation sequence

        public static Header VSP_LABELING = new();
        public static double VSP_LB_WEIGHT = 1;
        public static int VSP_LB_MAX_COLS = 50;
        public static int VSP_LB_MIN_TRIPS = 3; // min number of trips in a single vehicle task
        public static int VSP_LB_SOC_BINS = 101; // Charge values within epsilon of eachother will be considered the same
        public static int VSP_LB_SEC_COL_COUNT = 16; // Number of primary columns to generate secondary columns for 
        public static int VSP_LB_SEC_COL_ATTEMPTS = 4; // Number of attempts per secondary column

        public static Header VSP_LS_SHARED = new();
        public static bool VSP_LS_SHR_EXPAND_AVT = true;
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
        public static double VSP_LS_G_2OPT = 20;
        public static double VSP_LS_G_MOVE_RANGE = 20;
        public static double VSP_LS_G_ADD_CHARGE = 5;
        public static double VSP_LS_G_REMOVE_CHARGE = 1;

        public static Header CSP = new();
        public static bool CSP_DETERMINE_IIS = false;
        public static int CSP_SOLVER_TIMEOUT_SEC = 300;
        public static int CSP_OPT_IT_THRESHOLD = 2; // Amount of columns sequentially generated without rc before cg is stopped
        public static bool CSP_ALLOW_OVERCOVER = true; // determines >= in constraint
        public static int CSP_INSTANCES_PER_IT = 1;
        public static int CSP_MAX_COL_GEN_ITS = 1000;
        public static int CR_MIN_BREAK_TIME = 15 * 60; // Min break length
        public static int CR_MAX_BREAK_TIME = 60 * 60; // Max break length
        public static int CR_MIN_SHORT_IDLE_TIME = 0; // Min idle time which is not part of a split shift
        public static int CR_MAX_SHORT_IDLE_TIME = 30 * 60; // Max idle time which is not part of a split shift
        public static int CR_MIN_LONG_IDLE_TIME = (int)(1.5 * 60 * 60); // Min idle time which is part of a split shift
        public static int CR_MAX_LONG_IDLE_TIME = 5 * 60 * 60; // Max idle time which is part of a split shift

        public static Header CSP_LABELING = new();
        public static double CSP_LABELING_WEIGHT = 0;
        public static int CSP_LB_MAX_LABELS_IN_END = 100_000;
        public static int CSP_LB_MAX_COLS = 50;
        public static int CSP_LB_SEC_COL_COUNT = 9; // Number of primary columns to generate secondary columns for 
        public static double CSP_LB_SEC_COL_ATTEMPTS = 10; // Number of attempts per secondary column

        public static Header CSP_LS_GLOBAL = new();
        public static double CSP_LS_GLOBAL_WEIGHT = 0;
        public static double CSP_LS_G_STARTING_T = 50;
        public static double CSP_LS_G_ENDING_T = 0.01;
        public static double CSP_LS_G_COOLING_RATE = 0.98;
        public static double CSP_LS_G_ITERATIONS = 10_000_000;
        public static double CSP_LS_G_SWAP_TAILS = 5;
        public static double CSP_LS_G_MOVE_RANGE = 2;
        public static double CSP_LS_G_MOVE_SINGLE = 5;
        public static double CSP_LS_G_CREWHUB_PENALTY = 100;
        public static double CSP_LS_G_TIME_PENALTY = 1;
        public static double CSP_LS_G_STEER_PENALTY = 1;
        public static double CSP_LS_G_BREAK_PENALTY = 10;

        public static Header VCSP = new();
        public static int VCSP_SOLVER_TIMEOUT_SEC = 900;
        public static int VCSP_SOLVE_PER_ROUNDS = -1;       // Solves the ILP every X rounds, -1 to only do at end
        public static int VCSP_SOLVE_PER_DIST_ROUNDS = -1;  // Solves the ILP every X disruption rounds, -1 to only do at end
        public static int VCSP_ROUNDS = 10;
        public static int VCSP_VH_ITS_INIT = 10;
        public static int VCSP_VH_ITS_ROUND = 10;
        public static int VCSP_CR_ITS_INIT = 10;
        public static int VCSP_CR_ITS_ROUND = 10;
        public static int VCSP_MAX_TASKS_DURING = 40000;
        public static int VCSP_MAX_DUTIES_DURING = 40000;
        public static bool VCSP_VH_CSTR_SLACK = true;
        public static bool VCSP_CR_MAX_CSTR_SLACK = true;
        public static bool VCSP_CR_OTH_CSTR_SLACK = false;
        public static double VCSP_BLOCK_ADD_CHANCE = 0.1;
        public static bool VCSP_NONNEGATIVE_RC_VSP = true;
        public static bool VCSP_NONNEGATIVE_RC_CSP = true;
        public static bool VCSP_SLACK_IN_FINAL_OBJ = false;

        public static Header LAGRANGE = new();
        public static double LAGRANGE_PI_START = 2;
        public static double LAGRANGE_PI_END = 0.001; // (0, 2]
        public static double LAGRANGE_PI_COOLING = 0.5; // [0, 1)
        public static double LAGRANGE_N = 30;
        public static int LAGRANGE_DISRUPT_ROUNDS = 2;
        public static double LAGRANGE_DISRUPT_LWR = 0.8;
        public static double LAGRANGE_DISRUPT_UPR = 1.20;

        public static void Dump() {
            Type configType = typeof(Config);
            FieldInfo[] fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);
            StringBuilder configDump = new();
            foreach (FieldInfo field in fields)
                configDump.AppendLine($"{field.Name}: {field.GetValue(null)}");
            File.WriteAllText(Constants.RUN_LOG_FOLDER + "config.txt", configDump.ToString());
        }

        public static void LoadPartialDump(string dumpFilePath) {
            if (!File.Exists(dumpFilePath))
                return;

            var lines = File.ReadAllLines(dumpFilePath);
            Type configType = typeof(Config);
            var fields = configType.GetFields(BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic);

            foreach (var line in lines) {
                var parts = line.Split(':', 2);
                if (parts.Length != 2)
                    continue;

                string fieldName = parts[0].Trim();
                string valueStr = parts[1].Trim();

                var field = fields.FirstOrDefault(f => f.Name == fieldName);
                if (field == null || field.FieldType == typeof(Header))
                    continue;

                try {
                    object value = null;
                    var type = field.FieldType;
                    if (type == typeof(int))
                        value = int.Parse(valueStr);
                    else if (type == typeof(double))
                        value = double.Parse(valueStr);
                    else if (type == typeof(bool))
                        value = bool.Parse(valueStr);
                    else if (type == typeof(string))
                        value = valueStr;
                    else
                        continue;

                    field.SetValue(null, value);
                }
                catch {
                    // Ignore parse errors
                }
            }
        }
    }
}
