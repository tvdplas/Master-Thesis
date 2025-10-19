using E_VCSP;
using E_VCSP.Formatting;

namespace E_VCSP_Backend {
    internal class Program {
        public static void Main(string[] args) {
            Console.SetOut(new ConsoleIntercept());
            Console.WriteLine("Arguments: " + String.Join(" ", args));

            Runner runner = new Runner();
            if (args.Length == 0) {
                Console.WriteLine("Invalid arguments provided. Uses -h for help");
                return;
            }

            if (args[0] == "-h" || args[0] == "h") {
                Console.WriteLine("List experiments: evcsp -l");
                Console.WriteLine("Run normal: evcsp [vsp/seq/int] [instance] [configfile?]");
                Console.WriteLine("Run experiment: evcsp [expname] [instance] [configfile?]");
                return;
            }

            if (args[0] == "-l" || args[0] == "l") {
                Console.WriteLine("List of available experiments:");
                foreach (var exp in runner.Experiments.Keys) {
                    Console.WriteLine($"\t{exp.Substring("exp".Length)}");
                }
                return;
            }

            if (args.Length == 1) {
                Console.WriteLine("Invalid arguments provided. Uses -h for help");
                return;
            }

            runner.ActiveFolder = args[1];

            if (args.Length == 3) {
                E_VCSP.Config.LoadPartialDump(args[2]);
            }

            if (args[0] == "vsp") {
                runner.VSPFromInstance();
            }
            else if (args[0] == "seq") {
                runner.VSPFromInstance();
                runner.CSPFromInstance();
            }
            else if (args[0] == "int") {
                runner.VCSPFromInstance();
            }
            else {
                List<(string, Action)> actions = [];
                var exps = args[0].Split(",");
                foreach (string s in exps) {
                    Action? action = runner.Experiments!.GetValueOrDefault("exp" + s, null);
                    if (action != null) actions.Add((s, action));
                    else Console.WriteLine("Experiment " + s + " not recognized.");
                }

                foreach ((string name, Action action) in actions) {
                    runner.ActiveFolder = args[1];
                    Console.WriteLine(Config.CNSL_OVERRIDE + "Running experiment " + name);
                    Config.GLOBAL_CONSOLE_KILL = true;
                    action();
                }
            }
        }
    }
}
