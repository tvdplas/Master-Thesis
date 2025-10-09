using System.Text;

namespace E_VCSP.Formatting {
    public class ConsoleIntercept : TextWriter {
        private bool _isNewLine = true; // Track if we are starting a new line

        public ConsoleIntercept() { }

        public override void Write(char value) {
            Write(Config.CNSL_OVERRIDE + value.ToString());
        }

        public string? DetermineOutput(string? value) {
            if (value == null) value = string.Empty;
            if (Config.GLOBAL_CONSOLE_KILL && (value != Environment.NewLine && !value.StartsWith(Config.CNSL_OVERRIDE)))
                return null;

            if (value.StartsWith(Config.CNSL_OVERRIDE)) {
                value = value.Substring(Config.CNSL_OVERRIDE.Length);
            }

            if (value.Contains("Set parameter") || value.Contains("Academic"))
                value = "";

            StringBuilder output = new StringBuilder();

            foreach (char c in value) {
                if (_isNewLine) // Only prepend timestamp at the start of a new line
                {
                    if (Config.GLOBAL_CONSOLE_KILL && Environment.NewLine.Contains(c)) continue;
                    if (!Config.GLOBAL_CONSOLE_KILL) output.Append(DateTime.Now.ToString("[HH:mm:ss]: "));
                    _isNewLine = false;
                }

                output.Append(c);

                if (c == '\n') // Mark new line when a newline character is written
                {
                    _isNewLine = true;
                }
            }

            return output.ToString();
        }

        public void FlushToDisk(string output) {
            if (Constants.RUN_LOG_FOLDER != "") {
                File.AppendAllText(Constants.RUN_LOG_FOLDER + "log.txt", output);
            }
        }

        public override void Write(string? value) {
            string? output = DetermineOutput(value);
            if (output != null) {
                FlushToDisk(output);
                Console.OpenStandardOutput().Write(output.Select(x => (byte)x).ToArray());
            }
        }

        public override Encoding Encoding => Encoding.UTF8;
    }
}
