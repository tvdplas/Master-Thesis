using System.Threading;

namespace E_VCSP
{
    partial class MainView
    {
        private System.ComponentModel.IContainer components = null;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        private void InitializeComponent() {
            loadButton = new Button();
            loadFolderBrowser = new FolderBrowserDialog();
            activeFolderLabel = new Label();
            consoleView = new TextBox();
            configPanel = new Panel();
            comboBox1 = new ComboBox();
            runExperiment = new Button();
            solveEVCSPButton = new Button();
            loadCSPResultButton = new Button();
            loadEVSPResultButton = new Button();
            solveCSPButton = new Button();
            viewToggleButton = new Button();
            solveVSPButton = new Button();
            button2 = new Button();
            button3 = new Button();
            button4 = new Button();
            button5 = new Button();
            label1 = new Label();
            splitContainer = new SplitContainer();
            loadResultDialog = new OpenFileDialog();
            configPanel.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)splitContainer).BeginInit();
            splitContainer.Panel2.SuspendLayout();
            splitContainer.SuspendLayout();
            SuspendLayout();
            // 
            // loadButton
            // 
            loadButton.Location = new Point(4, 4);
            loadButton.Name = "loadButton";
            loadButton.Size = new Size(75, 23);
            loadButton.TabIndex = 0;
            loadButton.Text = "Load data";
            loadButton.UseVisualStyleBackColor = true;
            loadButton.Click += loadInstanceClick;
            // 
            // activeFolderLabel
            // 
            activeFolderLabel.AutoSize = true;
            activeFolderLabel.Location = new Point(83, 8);
            activeFolderLabel.Name = "activeFolderLabel";
            activeFolderLabel.Size = new Size(0, 15);
            activeFolderLabel.TabIndex = 1;
            // 
            // consoleView
            // 
            consoleView.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            consoleView.BackColor = Color.Black;
            consoleView.Font = new Font("Consolas", 10F);
            consoleView.ForeColor = Color.White;
            consoleView.Location = new Point(0, 0);
            consoleView.MaxLength = 1000000000;
            consoleView.Multiline = true;
            consoleView.Name = "consoleView";
            consoleView.ReadOnly = true;
            consoleView.ScrollBars = ScrollBars.Both;
            consoleView.Size = new Size(408, 558);
            consoleView.TabIndex = 4;
            // 
            // configPanel
            // 
            configPanel.Controls.Add(comboBox1);
            configPanel.Controls.Add(runExperiment);
            configPanel.Controls.Add(solveEVCSPButton);
            configPanel.Controls.Add(loadCSPResultButton);
            configPanel.Controls.Add(loadEVSPResultButton);
            configPanel.Controls.Add(solveCSPButton);
            configPanel.Controls.Add(viewToggleButton);
            configPanel.Controls.Add(solveVSPButton);
            configPanel.Controls.Add(loadButton);
            configPanel.Controls.Add(activeFolderLabel);
            configPanel.Dock = DockStyle.Left;
            configPanel.Location = new Point(0, 0);
            configPanel.Name = "configPanel";
            configPanel.Size = new Size(304, 558);
            configPanel.TabIndex = 1;
            // 
            // comboBox1
            // 
            comboBox1.FormattingEnabled = true;
            comboBox1.Location = new Point(4, 107);
            comboBox1.Name = "comboBox1";
            comboBox1.Size = new Size(136, 23);
            comboBox1.TabIndex = 10;
            // 
            // runExperiment
            // 
            runExperiment.Location = new Point(146, 107);
            runExperiment.Name = "runExperiment";
            runExperiment.Size = new Size(152, 23);
            runExperiment.TabIndex = 9;
            runExperiment.Text = "Run experiment";
            runExperiment.UseVisualStyleBackColor = true;
            runExperiment.Click += runExperiment_Click;
            // 
            // solveEVCSPButton
            // 
            solveEVCSPButton.Location = new Point(146, 81);
            solveEVCSPButton.Name = "solveEVCSPButton";
            solveEVCSPButton.Size = new Size(152, 23);
            solveEVCSPButton.TabIndex = 8;
            solveEVCSPButton.Text = "Solve EVCSP";
            solveEVCSPButton.UseVisualStyleBackColor = true;
            solveEVCSPButton.Click += solveEVCSPClick;
            // 
            // loadCSPResultButton
            // 
            loadCSPResultButton.Location = new Point(4, 55);
            loadCSPResultButton.Name = "loadCSPResultButton";
            loadCSPResultButton.Size = new Size(136, 23);
            loadCSPResultButton.TabIndex = 7;
            loadCSPResultButton.Text = "Load CSP result";
            loadCSPResultButton.UseVisualStyleBackColor = true;
            loadCSPResultButton.Click += loadCSPResultClick;
            // 
            // loadEVSPResultButton
            // 
            loadEVSPResultButton.Location = new Point(4, 29);
            loadEVSPResultButton.Name = "loadEVSPResultButton";
            loadEVSPResultButton.Size = new Size(136, 23);
            loadEVSPResultButton.TabIndex = 6;
            loadEVSPResultButton.Text = "Load EVSP result";
            loadEVSPResultButton.UseVisualStyleBackColor = true;
            loadEVSPResultButton.Click += loadEVSPResultClick;
            // 
            // solveCSPButton
            // 
            solveCSPButton.Location = new Point(146, 55);
            solveCSPButton.Name = "solveCSPButton";
            solveCSPButton.Size = new Size(152, 23);
            solveCSPButton.TabIndex = 5;
            solveCSPButton.Text = "Solve CSP";
            solveCSPButton.UseVisualStyleBackColor = true;
            solveCSPButton.Click += solveCSPClick;
            // 
            // viewToggleButton
            // 
            viewToggleButton.Location = new Point(4, 81);
            viewToggleButton.Name = "viewToggleButton";
            viewToggleButton.Size = new Size(136, 23);
            viewToggleButton.TabIndex = 4;
            viewToggleButton.Text = "Toggle Graph View";
            viewToggleButton.UseVisualStyleBackColor = true;
            viewToggleButton.Click += toggleGraphView;
            // 
            // solveVSPButton
            // 
            solveVSPButton.Location = new Point(146, 29);
            solveVSPButton.Name = "solveVSPButton";
            solveVSPButton.Size = new Size(152, 23);
            solveVSPButton.TabIndex = 2;
            solveVSPButton.Text = "Solve EVSP";
            solveVSPButton.UseVisualStyleBackColor = true;
            solveVSPButton.Click += solveVSPClick;
            // 
            // button2
            // 
            button2.Location = new Point(166, 33);
            button2.Name = "button2";
            button2.Size = new Size(135, 23);
            button2.TabIndex = 4;
            button2.Text = "Toggle Graph View";
            button2.UseVisualStyleBackColor = true;
            // 
            // button3
            // 
            button3.Location = new Point(3, 33);
            button3.Name = "button3";
            button3.Size = new Size(75, 23);
            button3.TabIndex = 2;
            button3.Text = "Solve";
            button3.UseVisualStyleBackColor = true;
            // 
            // button4
            // 
            button4.Enabled = false;
            button4.Location = new Point(85, 33);
            button4.Name = "button4";
            button4.Size = new Size(75, 23);
            button4.TabIndex = 3;
            button4.Text = "Stop";
            button4.UseVisualStyleBackColor = true;
            // 
            // button5
            // 
            button5.Location = new Point(4, 4);
            button5.Name = "button5";
            button5.Size = new Size(75, 23);
            button5.TabIndex = 0;
            button5.Text = "Load data";
            button5.UseVisualStyleBackColor = true;
            // 
            // label1
            // 
            label1.AutoSize = true;
            label1.Location = new Point(83, 8);
            label1.Name = "label1";
            label1.Size = new Size(0, 15);
            label1.TabIndex = 1;
            // 
            // splitContainer
            // 
            splitContainer.Dock = DockStyle.Fill;
            splitContainer.Location = new Point(304, 0);
            splitContainer.Name = "splitContainer";
            // 
            // splitContainer.Panel2
            // 
            splitContainer.Panel2.Controls.Add(consoleView);
            splitContainer.Size = new Size(860, 558);
            splitContainer.SplitterDistance = 445;
            splitContainer.TabIndex = 0;
            // 
            // loadResultDialog
            // 
            loadResultDialog.FileName = "openFileDialog1";
            // 
            // MainView
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1164, 558);
            Controls.Add(splitContainer);
            Controls.Add(configPanel);
            Name = "MainView";
            Text = "E-VCSP Visualisation";
            configPanel.ResumeLayout(false);
            configPanel.PerformLayout();
            splitContainer.Panel2.ResumeLayout(false);
            splitContainer.Panel2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)splitContainer).EndInit();
            splitContainer.ResumeLayout(false);
            ResumeLayout(false);
        }

        #endregion

        private Button loadButton;
        private FolderBrowserDialog loadFolderBrowser;
        private Label activeFolderLabel;
        private TextBox consoleView;
        private Panel configPanel; // Panel for the Button and FolderBrowserDialog
        private SplitContainer splitContainer; // SplitContainer for the TextBox and GraphViewer
        private Button solveVSPButton;
        private Button viewToggleButton;
        private Button button2;
        private Button button3;
        private Button button4;
        private Button button5;
        private Label label1;
        private Button loadEVSPResultButton;
        private Button loadCSPResultButton;
        private Button solveCSPButton;
        private OpenFileDialog loadResultDialog;
        private Button solveEVCSPButton;
        private ComboBox comboBox1;
        private Button runExperiment;
    }
}
