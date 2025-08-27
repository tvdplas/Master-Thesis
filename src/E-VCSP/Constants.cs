namespace E_VCSP {
    public static class Constants {
        public static int MIN_NODE_TIME = 300;

        // Constraint names
        public static string CSTR_MAX_VEHICLES = "max_vehicles";
        public static string CSTR_MAX_DUTIES = "max_duties";
        public static string CSTR_TRIP_COVER = "cover_trip_"; // + trip index
        public static string CSTR_BLOCK_COVER = "cover_block_"; // + block descriptor
        public static string CSTR_CR_AVG_TIME = "cr_overall_limited_average_length";
        public static string CSTR_CR_LONG_DUTIES = "cr_overall_no_excessive_length";
        public static string CSTR_CR_BROKEN_DUTIES = "cr_overall_max_broken";
        public static string CSTR_CR_BETWEEN_DUTIES = "cr_overall_max_between";

        // Driving/Charging
        public static double KWH_COST = 0.32; // based on national average price
        public static int MIN_CHARGE_TIME = 300;
        public static double VH_M_COST = 0.00005; // verified at 
        public static int MAX_STEERING_TIME = 60 * 60 * 4; // conitnous driving time
        public static int MAX_NO_HUB_TIME = 60 * 60 * 8; // conitnous time without visiting crew hub
        public static double VH_IDLE_COST = 0; // Cost per unit of time for being idle.
        public static double CR_HOURLY_COST = 65; // hourly cost of employee. based on qbuzz data
        public static double CR_BROKEN_SHIFT_COST = 20; // additional surcharge for broken shift. based on qbuzz data
        public static double CR_MAX_BROKEN_SHIFTS = 0.3;
        public static double CR_MAX_BETWEEN_SHIFTS = 0.1;
        public static double CR_MAX_OVER_LONG_DUTY = 0.15;
        public static int CR_TARGET_SHIFT_LENGTH = 8 * 60 * 60;
        public static int CR_LONG_SHIFT_LENGTH = (int)(8.5 * 60 * 60);
        public static int CR_MAX_SHIFT_LENGTH = 9 * 60 * 60;
        public static int CR_MIN_BREAK_TIME = 15 * 60; // Min break length
        public static int CR_MAX_BREAK_TIME = 1 * 60 * 60; // Max break length
        public static int CR_MIN_SHORT_IDLE_TIME = 0; // Min idle time which is not part of a split shift
        public static int CR_MAX_SHORT_IDLE_TIME = 15 * 60; // Max idle time which is not part of a split shift
        public static int CR_MIN_LONG_IDLE_TIME = 2 * 60 * 60; // Min idle time which is part of a split shift
        public static int CR_MAX_LONG_IDLE_TIME = 5 * 60 * 60; // Max idle time which is part of a split shift
    }
}
