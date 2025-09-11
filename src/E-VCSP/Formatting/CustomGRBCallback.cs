using Gurobi;

namespace E_VCSP.Formatting {
    class CustomGRBCallback : GRBCallback {
        protected override void Callback() {
            if (where == GRB.Callback.MESSAGE && Config.CONSOLE_GUROBI) {
                string text = GetStringInfo(GRB.Callback.MSG_STRING);
                text.Substring(0, text.Length - 1);
                Console.WriteLine(text);
            }
        }
    }
}
