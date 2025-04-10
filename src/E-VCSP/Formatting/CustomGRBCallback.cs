using Gurobi;

namespace E_VCSP.Formatting
{
    class CustomGRBCallback : GRBCallback
    {
        protected override void Callback()
        {
            if (where == GRB.Callback.MESSAGE && Config.CONSOLE_GUROBI)
            {
                String text = GetStringInfo(GRB.Callback.MSG_STRING);
                Console.WriteLine(text);
            }
        }
    }
}
