namespace E_VCSP {
    public static class Constants {
        public static int MIN_NODE_TIME = 300;

        public static string CSTR_MAX_VEHICLES = "max_vehicles";
        public static string CSTR_TRIP_COVER = "cover_trip_"; // + trip index
        public static string CSTR_BLOCK_COVER = "cover_block_"; // + block descriptor
        public static string CSTR_CR_AVG_TIME = "cr_overall_limited_average_length";
        public static string CSTR_CR_LONG_DUTIES = "cr_overall_no_excessive_length";
        public static string CSTR_CR_BROKEN_DUTIES = "cr_overall_max_broken";
        public static string CSTR_CR_BETWEEN_DUTIES = "cr_overall_max_between";
    }
}
