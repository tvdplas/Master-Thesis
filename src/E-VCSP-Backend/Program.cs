namespace E_VCSP_Backend {
    internal class Program {
        public static async Task Main(string[] args) {
            foreach (var arg in args) Console.WriteLine(arg);

            Runner runner = new Runner();
            if (args.Length == 0) {
                Console.WriteLine("Invalid arguments provided. Uses -h for help");
                return;
            }

            if (args[0] == "-h" || args[0] == "h") {
                Console.WriteLine("List experiments: evcsp -l");
                Console.WriteLine("Run normal: evcsp [instance] [vsp/seq/int] [config?]");
                Console.WriteLine("Run experiment: evcsp [instance] [expname] [config?]");
                return;
            }

            if (args[0] == "-l" || args[0] == "l") {
                Console.WriteLine("List of available experiments:");
                foreach (var exp in runner.Experiments) {
                    Console.WriteLine($"\t{exp.Method.Name.Substring("exp".Length)}");
                }
                return;
            }

            if (args.Length == 1)
                Console.WriteLine("Invalid arguments provided. Uses -h for help");

            runner.ActiveFolder = args[0];

            if (args.Length == 3) {
                E_VCSP.Config.LoadPartialDump(args[2]);
            }

            if (args[1] == "vsp") {
                bool x = await runner.VSPFromInstance();
                Console.WriteLine(x);
            }
            else if (args[1] == "seq") {
                await runner.VSPFromInstance();
                await runner.CSPFromInstance();
            }
            else if (args[1] == "int") {
                await runner.VCSPFromInstance();
            }
            else {
                Action? task = null;
                foreach (var exp in runner.Experiments) {
                    if ("exp" + args[1] == exp.Method.Name) {
                        task = exp;
                    }
                }
                if (task == null) {
                    Console.WriteLine("Unknown experiment. Aborting.");
                    return;
                }
                else task();
            }
        }
    }
}
