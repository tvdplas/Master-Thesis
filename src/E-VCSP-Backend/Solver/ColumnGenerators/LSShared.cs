namespace E_VCSP.Solver.ColumnGenerators {
    public enum LSOpResult {
        Improvement = 0,
        Accept = 1,
        Decline = 2,
        Invalid = 3,
        Count,
    }

    internal static class LSShared {
        public static Random random = new();

        public static bool Accept(double deltaScore, double T) {
            if (deltaScore < 0) return true;
            return Math.Exp(-deltaScore / T) > random.NextDouble();
        }
    }
}
