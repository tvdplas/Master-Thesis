namespace E_VCSP.Formatting {
    internal class TextboxConsoleIntercept : ConsoleIntercept {
        private readonly TextBox _textBox;

        public TextboxConsoleIntercept(TextBox textBox) {
            _textBox = textBox;
        }

        public override void Write(string? value) {
            string? output = DetermineOutput(value);
            if (output != null) {
                FlushToDisk(output);
                AppendTextToTextBox(output);
            }
        }

        private void AppendTextToTextBox(string text) {
            if (_textBox == null) return;

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
    }
}
