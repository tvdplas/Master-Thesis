namespace E_VCSP.Formatting {
    public static class Distance {
        public static string KM(int meters, int decimals = 1) {
            return $"{Math.Round(meters / 1000.0, decimals)} km";
        }
    }
}
