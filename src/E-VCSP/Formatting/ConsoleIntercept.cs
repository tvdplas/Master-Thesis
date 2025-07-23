using System.Text;

namespace E_VCSP.Formatting {
    public class ConsoleIntercept : TextWriter {
        private readonly TextBox _textBox;
        private bool _isNewLine = true; // Track if we are starting a new line

        public ConsoleIntercept(TextBox textBox) {
            _textBox = textBox;
        }

        public override void Write(char value) {

            Write(value.ToString());
        }

        public override void Write(string? value) {
            if (value == null) value = string.Empty;

            StringBuilder output = new StringBuilder();

            foreach (char c in value) {
                if (_isNewLine) // Only prepend timestamp at the start of a new line
                {
                    output.Append(DateTime.Now.ToString("[HH:mm:ss]: "));
                    _isNewLine = false;
                }

                output.Append(c);

                if (c == '\n') // Mark new line when a newline character is written
                {
                    _isNewLine = true;
                }
            }

            if (Config.RUN_LOG_FOLDER != "") {
                File.AppendAllText(Config.RUN_LOG_FOLDER + "log.txt", output.ToString());
            }
            AppendTextToTextBox(output.ToString());
        }

        private void AppendTextToTextBox(string text) {
            if (_textBox.InvokeRequired) {
                _textBox.Invoke(new Action(() => {
                    _textBox.AppendText(text);
                    _textBox.ScrollToCaret();
                }));
            }
            else {
                _textBox.AppendText(text);
                _textBox.ScrollToCaret();
            }
        }

        public override Encoding Encoding => Encoding.UTF8;
    }
}
