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

        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainView));
            loadButton = new Button();
            loadFolderBrowser = new FolderBrowserDialog();
            activeFolderLabel = new Label();
            graphViewer = new Microsoft.Msagl.GraphViewerGdi.GViewer();
            textBox1 = new TextBox();
            panel1 = new Panel();
            solveCSPButton = new Button();
            viewToggleButton = new Button();
            solveVSPButton = new Button();
            stopButton = new Button();
            button2 = new Button();
            button3 = new Button();
            button4 = new Button();
            button5 = new Button();
            label1 = new Label();
            splitContainer1 = new SplitContainer();
            panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)splitContainer1).BeginInit();
            splitContainer1.Panel1.SuspendLayout();
            splitContainer1.Panel2.SuspendLayout();
            splitContainer1.SuspendLayout();
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
            loadButton.Click += loadButtonClick;
            // 
            // activeFolderLabel
            // 
            activeFolderLabel.AutoSize = true;
            activeFolderLabel.Location = new Point(83, 8);
            activeFolderLabel.Name = "activeFolderLabel";
            activeFolderLabel.Size = new Size(0, 15);
            activeFolderLabel.TabIndex = 1;
            // 
            // graphViewer
            // 
            graphViewer.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            graphViewer.ArrowheadLength = 10D;
            graphViewer.AsyncLayout = false;
            graphViewer.AutoScroll = true;
            graphViewer.AutoSize = true;
            graphViewer.BackwardEnabled = false;
            graphViewer.BuildHitTree = true;
            graphViewer.CurrentLayoutMethod = Microsoft.Msagl.GraphViewerGdi.LayoutMethod.UseSettingsOfTheGraph;
            graphViewer.EdgeInsertButtonVisible = false;
            graphViewer.FileName = "";
            graphViewer.ForwardEnabled = false;
            graphViewer.Graph = null;
            graphViewer.IncrementalDraggingModeAlways = false;
            graphViewer.InsertingEdge = false;
            graphViewer.LayoutAlgorithmSettingsButtonVisible = false;
            graphViewer.LayoutEditingEnabled = true;
            graphViewer.Location = new Point(0, 0);
            graphViewer.LooseOffsetForRouting = 0.25D;
            graphViewer.MouseHitDistance = 0.05D;
            graphViewer.Name = "graphViewer";
            graphViewer.NavigationVisible = true;
            graphViewer.NeedToCalculateLayout = true;
            graphViewer.OffsetForRelaxingInRouting = 0.6D;
            graphViewer.PaddingForEdgeRouting = 8D;
            graphViewer.PanButtonPressed = false;
            graphViewer.SaveAsImageEnabled = true;
            graphViewer.SaveAsMsaglEnabled = true;
            graphViewer.SaveButtonVisible = true;
            graphViewer.SaveGraphButtonVisible = true;
            graphViewer.SaveInVectorFormatEnabled = true;
            graphViewer.Size = new Size(443, 534);
            graphViewer.TabIndex = 3;
            graphViewer.TightOffsetForRouting = 0.125D;
            graphViewer.ToolBarIsVisible = true;
            graphViewer.Transform = (Microsoft.Msagl.Core.Geometry.Curves.PlaneTransformation)resources.GetObject("graphViewer.Transform");
            graphViewer.UndoRedoButtonsVisible = true;
            graphViewer.WindowZoomButtonPressed = false;
            graphViewer.ZoomF = 1D;
            graphViewer.ZoomWindowThreshold = 0.05D;
            // 
            // textBox1
            // 
            textBox1.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
            textBox1.BackColor = Color.Black;
            textBox1.Font = new Font("Consolas", 10F);
            textBox1.ForeColor = Color.White;
            textBox1.Location = new Point(0, 0);
            textBox1.MaxLength = 1000000000;
            textBox1.Multiline = true;
            textBox1.Name = "textBox1";
            textBox1.ReadOnly = true;
            textBox1.ScrollBars = ScrollBars.Both;
            textBox1.Size = new Size(408, 558);
            textBox1.TabIndex = 4;
            // 
            // panel1
            // 
            panel1.Controls.Add(solveCSPButton);
            panel1.Controls.Add(viewToggleButton);
            panel1.Controls.Add(solveVSPButton);
            panel1.Controls.Add(stopButton);
            panel1.Controls.Add(loadButton);
            panel1.Controls.Add(activeFolderLabel);
            panel1.Dock = DockStyle.Left;
            panel1.Location = new Point(0, 0);
            panel1.Name = "panel1";
            panel1.Size = new Size(304, 558);
            panel1.TabIndex = 1;
            // 
            // solveCSPButton
            // 
            solveCSPButton.Location = new Point(146, 33);
            solveCSPButton.Name = "solveCSPButton";
            solveCSPButton.Size = new Size(152, 23);
            solveCSPButton.TabIndex = 5;
            solveCSPButton.Text = "Solve CSP";
            solveCSPButton.UseVisualStyleBackColor = true;
            solveCSPButton.Click += solveCSPClick;
            // 
            // viewToggleButton
            // 
            viewToggleButton.Enabled = false;
            viewToggleButton.Location = new Point(146, 62);
            viewToggleButton.Name = "viewToggleButton";
            viewToggleButton.Size = new Size(155, 23);
            viewToggleButton.TabIndex = 4;
            viewToggleButton.Text = "Toggle Graph View";
            viewToggleButton.UseVisualStyleBackColor = true;
            viewToggleButton.Click += toggleGraphView;
            // 
            // solveVSPButton
            // 
            solveVSPButton.Location = new Point(4, 33);
            solveVSPButton.Name = "solveVSPButton";
            solveVSPButton.Size = new Size(136, 23);
            solveVSPButton.TabIndex = 2;
            solveVSPButton.Text = "Solve EVSP";
            solveVSPButton.UseVisualStyleBackColor = true;
            solveVSPButton.Click += solveVSPClick;
            // 
            // stopButton
            // 
            stopButton.Enabled = false;
            stopButton.Location = new Point(4, 62);
            stopButton.Name = "stopButton";
            stopButton.Size = new Size(136, 23);
            stopButton.TabIndex = 3;
            stopButton.Text = "Stop";
            stopButton.UseVisualStyleBackColor = true;
            stopButton.Click += stopButtonClick;
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
            // splitContainer1
            // 
            splitContainer1.Dock = DockStyle.Fill;
            splitContainer1.Location = new Point(304, 0);
            splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            splitContainer1.Panel1.Controls.Add(graphViewer);
            // 
            // splitContainer1.Panel2
            // 
            splitContainer1.Panel2.Controls.Add(textBox1);
            splitContainer1.Size = new Size(860, 558);
            splitContainer1.SplitterDistance = 445;
            splitContainer1.TabIndex = 0;
            // 
            // MainView
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(1164, 558);
            Controls.Add(splitContainer1);
            Controls.Add(panel1);
            Name = "MainView";
            Text = "E-VCSP Visualisation";
            panel1.ResumeLayout(false);
            panel1.PerformLayout();
            splitContainer1.Panel1.ResumeLayout(false);
            splitContainer1.Panel1.PerformLayout();
            splitContainer1.Panel2.ResumeLayout(false);
            splitContainer1.Panel2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)splitContainer1).EndInit();
            splitContainer1.ResumeLayout(false);
            ResumeLayout(false);
        }

        #endregion

        private Button loadButton;
        private FolderBrowserDialog loadFolderBrowser;
        private Label activeFolderLabel;
        private Microsoft.Msagl.GraphViewerGdi.GViewer graphViewer;
        private TextBox textBox1;
        private Panel panel1; // Panel for the Button and FolderBrowserDialog
        private SplitContainer splitContainer1; // SplitContainer for the TextBox and GraphViewer
        private Button solveVSPButton;
        private Button stopButton;
        private CancellationTokenSource cancellationTokenSource;
        private Button viewToggleButton;
        private Button button2;
        private Button button3;
        private Button button4;
        private Button button5;
        private Label label1;
        private Button solveCSPButton;
    }
}
