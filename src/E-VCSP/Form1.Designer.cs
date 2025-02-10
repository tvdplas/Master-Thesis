namespace E_VCSP
{
    partial class MainView
    {
        /// <summary>
        ///  Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        ///  Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        ///  Required method for Designer support - do not modify
        ///  the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainView));
            loadButton = new Button();
            loadFolderBrowser = new FolderBrowserDialog();
            activeFolderLabel = new Label();
            graphViewer = new Microsoft.Msagl.GraphViewerGdi.GViewer();
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
            // loadFolderBrowser
            // 
            loadFolderBrowser.Description = "Select a folder containing E-VCSP input data";
            loadFolderBrowser.InitialDirectory = "./";
            loadFolderBrowser.UseDescriptionForTitle = true;
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
            graphViewer.ArrowheadLength = 10D;
            graphViewer.AsyncLayout = false;
            graphViewer.AutoScroll = true;
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
            graphViewer.LayoutEditingEnabled = false;
            graphViewer.Location = new Point(4, 33);
            graphViewer.LooseOffsetForRouting = 0.25D;
            graphViewer.MouseHitDistance = 0.05D;
            graphViewer.Name = "graphViewer";
            graphViewer.NavigationVisible = false;
            graphViewer.NeedToCalculateLayout = true;
            graphViewer.OffsetForRelaxingInRouting = 0.6D;
            graphViewer.PaddingForEdgeRouting = 8D;
            graphViewer.PanButtonPressed = true;
            graphViewer.SaveAsImageEnabled = false;
            graphViewer.SaveAsMsaglEnabled = false;
            graphViewer.SaveButtonVisible = true;
            graphViewer.SaveGraphButtonVisible = true;
            graphViewer.SaveInVectorFormatEnabled = true;
            graphViewer.Size = new Size(784, 405);
            graphViewer.TabIndex = 2;
            graphViewer.TightOffsetForRouting = 0.125D;
            graphViewer.ToolBarIsVisible = true;
            graphViewer.Transform = (Microsoft.Msagl.Core.Geometry.Curves.PlaneTransformation)resources.GetObject("graphViewer.Transform");
            graphViewer.UndoRedoButtonsVisible = false;
            graphViewer.WindowZoomButtonPressed = false;
            graphViewer.ZoomF = 1D;
            graphViewer.ZoomWindowThreshold = 0.05D;
            // 
            // MainView
            // 
            AutoScaleDimensions = new SizeF(7F, 15F);
            AutoScaleMode = AutoScaleMode.Font;
            ClientSize = new Size(800, 450);
            Controls.Add(graphViewer);
            Controls.Add(activeFolderLabel);
            Controls.Add(loadButton);
            Name = "MainView";
            Text = "E-VCSP Visualisation";
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion
        private Button loadButton;
        private FolderBrowserDialog loadFolderBrowser;
        private Label activeFolderLabel;
        private Microsoft.Msagl.GraphViewerGdi.GViewer graphViewer;
    }
}
