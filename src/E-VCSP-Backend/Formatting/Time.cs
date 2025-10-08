namespace E_VCSP.Formatting {
    public static class Time {
        public static string HHMMSS(int seconds) {
            string res = "";
            res += (seconds / 3600).ToString("00");
            res += ":" + (seconds % 3600 / 60).ToString("00");
            res += ":" + (seconds % 60).ToString("00");
            return res;
        }
    }
}
