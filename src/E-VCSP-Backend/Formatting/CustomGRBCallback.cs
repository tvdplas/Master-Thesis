using Gurobi;

namespace E_VCSP.Formatting {
    class CustomGRBCallback : GRBCallback {
        double timeLimit;
        public CustomGRBCallback(GRBModel model) {
            timeLimit = model.Get(GRB.DoubleParam.TimeLimit);
        }

        protected override void Callback() {
            if (where == GRB.Callback.MIP || where == GRB.Callback.MIPNODE) {
                if (!Config.GRB_EXTEND_TIME) return;
                double runtime = GetDoubleInfo(GRB.Callback.RUNTIME);
                double best = (where == GRB.Callback.MIP)
                                ? GetDoubleInfo(GRB.Callback.MIP_OBJBST)
                                : GetDoubleInfo(GRB.Callback.MIPNODE_OBJBST);
                double bound = (where == GRB.Callback.MIP)
                                ? GetDoubleInfo(GRB.Callback.MIP_OBJBND)
                                : GetDoubleInfo(GRB.Callback.MIPNODE_OBJBND);
                if (double.IsInfinity(best) || double.IsInfinity(bound)) return;


                double gap = Math.Abs(best - bound) / Math.Max(1e-10, Math.Abs(best));

                if (runtime >= timeLimit - 60.0 && gap > Config.GRB_TARGET_GAP && timeLimit < Config.GRB_MAX_EXTENDED_TIME) {
                    timeLimit += 60.0;
                    Set(GRB.DoubleParam.TimeLimit, timeLimit);
                    Console.WriteLine($"Current gap {(gap * 100).ToString("0.##")}% > target gap {Config.GRB_TARGET_GAP * 100}%, updating timelimit to {(int)timeLimit}s");
                }
            }

            if (where == GRB.Callback.MESSAGE && Config.CONSOLE_GUROBI) {
                string text = GetStringInfo(GRB.Callback.MSG_STRING);
                text = text.Substring(0, text.Length - 1); // remove \n which is baked into grb text
                Console.WriteLine("[GRB] " + text);
            }
        }
    }
}
