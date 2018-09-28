using System.Windows.Forms;

namespace SW2URDF
{
    partial class TreeMerge
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
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
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.LoadedTreeView = new System.Windows.Forms.TreeView();
            this.ExistingTreeView = new System.Windows.Forms.TreeView();
            this.TitleLabel = new System.Windows.Forms.Label();
            this.DescriptionLabel = new System.Windows.Forms.Label();
            this.CoordinatesExistingComboBox = new System.Windows.Forms.ComboBox();
            this.AxisExistingComboBox = new System.Windows.Forms.ComboBox();
            this.CoordinatesExistingLabel = new System.Windows.Forms.Label();
            this.AxisExistingLabel = new System.Windows.Forms.Label();
            this.TypeExistingLabel = new System.Windows.Forms.Label();
            this.TypeExistingComboBox = new System.Windows.Forms.ComboBox();
            this.UpdateExistingButton = new System.Windows.Forms.Button();
            this.panel1 = new System.Windows.Forms.Panel();
            this.ExistingConfigurationEditLabel = new System.Windows.Forms.Label();
            this.JointNameExistingLabel = new System.Windows.Forms.Label();
            this.LinkNameExistingLabel = new System.Windows.Forms.Label();
            this.JointNameExistingTextBox = new System.Windows.Forms.TextBox();
            this.LinkNameExistingTextBox = new System.Windows.Forms.TextBox();
            this.ResetExistingButton = new System.Windows.Forms.Button();
            this.panel2 = new System.Windows.Forms.Panel();
            this.checkBox6 = new System.Windows.Forms.CheckBox();
            this.JointNameLoadedField = new System.Windows.Forms.Label();
            this.TypeLoadedField = new System.Windows.Forms.Label();
            this.AxisLoadedField = new System.Windows.Forms.Label();
            this.CoordinatesLoadedField = new System.Windows.Forms.Label();
            this.LoadedConfigurationEditLabel = new System.Windows.Forms.Label();
            this.JointNameLoadedLabel = new System.Windows.Forms.Label();
            this.LinkNameLoadedLabel = new System.Windows.Forms.Label();
            this.LinkNameLoadedTextBox = new System.Windows.Forms.TextBox();
            this.ResetLoadedButton = new System.Windows.Forms.Button();
            this.UpdateLoadedButton = new System.Windows.Forms.Button();
            this.TypeLoadedLabel = new System.Windows.Forms.Label();
            this.AxisLoadedLabel = new System.Windows.Forms.Label();
            this.CoordinatesLoadedLabel = new System.Windows.Forms.Label();
            this.ExistingConfigurationLabel = new System.Windows.Forms.Label();
            this.LoadedConfigurationLabel = new System.Windows.Forms.Label();
            this.MergeButton = new System.Windows.Forms.Button();
            this.CancelMergeButton = new System.Windows.Forms.Button();
            this.backgroundWorker1 = new System.ComponentModel.BackgroundWorker();
            this.MassInertiaExistingRadio = new System.Windows.Forms.RadioButton();
            this.MassInertiaLoadedRadio = new System.Windows.Forms.RadioButton();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.VisualExistingRadio = new System.Windows.Forms.RadioButton();
            this.VisualLoadedRadio = new System.Windows.Forms.RadioButton();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.JointKinematicsExistingRadio = new System.Windows.Forms.RadioButton();
            this.JointKinematicsLoadedRadio = new System.Windows.Forms.RadioButton();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.JointOtherExistingRadio = new System.Windows.Forms.RadioButton();
            this.JointOtherLoadedRadio = new System.Windows.Forms.RadioButton();
            this.panel1.SuspendLayout();
            this.panel2.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.SuspendLayout();
            // 
            // LoadedTreeView
            // 
            this.LoadedTreeView.AllowDrop = true;
            this.LoadedTreeView.Location = new System.Drawing.Point(480, 112);
            this.LoadedTreeView.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.LoadedTreeView.Name = "LoadedTreeView";
            this.LoadedTreeView.Size = new System.Drawing.Size(427, 665);
            this.LoadedTreeView.TabIndex = 0;
            this.LoadedTreeView.ItemDrag += new System.Windows.Forms.ItemDragEventHandler(this.LoadedTreeItemDrag);
            this.LoadedTreeView.AfterSelect += new System.Windows.Forms.TreeViewEventHandler(this.TreeViewLoadedAfterSelect);
            this.LoadedTreeView.DragDrop += new System.Windows.Forms.DragEventHandler(this.LoadedTreeDragDrop);
            this.LoadedTreeView.DragEnter += new System.Windows.Forms.DragEventHandler(this.LoadedTreeDragEnter);
            this.LoadedTreeView.DragOver += new System.Windows.Forms.DragEventHandler(this.LoadedTreeDragOver);
            // 
            // ExistingTreeView
            // 
            this.ExistingTreeView.AllowDrop = true;
            this.ExistingTreeView.Location = new System.Drawing.Point(18, 112);
            this.ExistingTreeView.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.ExistingTreeView.Name = "ExistingTreeView";
            this.ExistingTreeView.Size = new System.Drawing.Size(427, 665);
            this.ExistingTreeView.TabIndex = 1;
            this.ExistingTreeView.ItemDrag += new System.Windows.Forms.ItemDragEventHandler(this.ExistingTreeItemDrag);
            this.ExistingTreeView.AfterSelect += new System.Windows.Forms.TreeViewEventHandler(this.TreeViewExistingAfterSelect);
            this.ExistingTreeView.DragDrop += new System.Windows.Forms.DragEventHandler(this.ExistingTreeDragDrop);
            this.ExistingTreeView.DragEnter += new System.Windows.Forms.DragEventHandler(this.ExistingTreeDragEnter);
            this.ExistingTreeView.DragOver += new System.Windows.Forms.DragEventHandler(this.ExistingTreeDragOver);
            // 
            // TitleLabel
            // 
            this.TitleLabel.AutoSize = true;
            this.TitleLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.TitleLabel.Location = new System.Drawing.Point(11, 8);
            this.TitleLabel.Name = "TitleLabel";
            this.TitleLabel.Size = new System.Drawing.Size(302, 20);
            this.TitleLabel.TabIndex = 2;
            this.TitleLabel.Text = "URDF Configuration Manual Merge";
            // 
            // DescriptionLabel
            // 
            this.DescriptionLabel.AutoSize = true;
            this.DescriptionLabel.Location = new System.Drawing.Point(40, 40);
            this.DescriptionLabel.Name = "DescriptionLabel";
            this.DescriptionLabel.Size = new System.Drawing.Size(432, 17);
            this.DescriptionLabel.TabIndex = 3;
            this.DescriptionLabel.Text = "Drag and drop to re-order tree or rename links to match each other";
            // 
            // CoordinatesExistingComboBox
            // 
            this.CoordinatesExistingComboBox.FormattingEnabled = true;
            this.CoordinatesExistingComboBox.Location = new System.Drawing.Point(133, 160);
            this.CoordinatesExistingComboBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.CoordinatesExistingComboBox.Name = "CoordinatesExistingComboBox";
            this.CoordinatesExistingComboBox.Size = new System.Drawing.Size(374, 24);
            this.CoordinatesExistingComboBox.TabIndex = 4;
            // 
            // AxisExistingComboBox
            // 
            this.AxisExistingComboBox.FormattingEnabled = true;
            this.AxisExistingComboBox.Location = new System.Drawing.Point(133, 200);
            this.AxisExistingComboBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.AxisExistingComboBox.Name = "AxisExistingComboBox";
            this.AxisExistingComboBox.Size = new System.Drawing.Size(374, 24);
            this.AxisExistingComboBox.TabIndex = 5;
            // 
            // CoordinatesExistingLabel
            // 
            this.CoordinatesExistingLabel.AutoSize = true;
            this.CoordinatesExistingLabel.Location = new System.Drawing.Point(39, 160);
            this.CoordinatesExistingLabel.Name = "CoordinatesExistingLabel";
            this.CoordinatesExistingLabel.Size = new System.Drawing.Size(84, 17);
            this.CoordinatesExistingLabel.TabIndex = 6;
            this.CoordinatesExistingLabel.Text = "Coordinates";
            this.CoordinatesExistingLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // AxisExistingLabel
            // 
            this.AxisExistingLabel.AutoSize = true;
            this.AxisExistingLabel.Location = new System.Drawing.Point(90, 200);
            this.AxisExistingLabel.Name = "AxisExistingLabel";
            this.AxisExistingLabel.Size = new System.Drawing.Size(33, 17);
            this.AxisExistingLabel.TabIndex = 7;
            this.AxisExistingLabel.Text = "Axis";
            this.AxisExistingLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // TypeExistingLabel
            // 
            this.TypeExistingLabel.AutoSize = true;
            this.TypeExistingLabel.Location = new System.Drawing.Point(85, 242);
            this.TypeExistingLabel.Name = "TypeExistingLabel";
            this.TypeExistingLabel.Size = new System.Drawing.Size(40, 17);
            this.TypeExistingLabel.TabIndex = 8;
            this.TypeExistingLabel.Text = "Type";
            this.TypeExistingLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // TypeExistingComboBox
            // 
            this.TypeExistingComboBox.FormattingEnabled = true;
            this.TypeExistingComboBox.Location = new System.Drawing.Point(133, 240);
            this.TypeExistingComboBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.TypeExistingComboBox.Name = "TypeExistingComboBox";
            this.TypeExistingComboBox.Size = new System.Drawing.Size(374, 24);
            this.TypeExistingComboBox.TabIndex = 9;
            // 
            // UpdateExistingButton
            // 
            this.UpdateExistingButton.Location = new System.Drawing.Point(409, 288);
            this.UpdateExistingButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.UpdateExistingButton.Name = "UpdateExistingButton";
            this.UpdateExistingButton.Size = new System.Drawing.Size(98, 30);
            this.UpdateExistingButton.TabIndex = 10;
            this.UpdateExistingButton.Text = "Update";
            this.UpdateExistingButton.UseVisualStyleBackColor = true;
            this.UpdateExistingButton.Click += new System.EventHandler(this.UpdateExistingButtonClick);
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.ExistingConfigurationEditLabel);
            this.panel1.Controls.Add(this.JointNameExistingLabel);
            this.panel1.Controls.Add(this.LinkNameExistingLabel);
            this.panel1.Controls.Add(this.JointNameExistingTextBox);
            this.panel1.Controls.Add(this.LinkNameExistingTextBox);
            this.panel1.Controls.Add(this.ResetExistingButton);
            this.panel1.Controls.Add(this.UpdateExistingButton);
            this.panel1.Controls.Add(this.TypeExistingComboBox);
            this.panel1.Controls.Add(this.TypeExistingLabel);
            this.panel1.Controls.Add(this.AxisExistingLabel);
            this.panel1.Controls.Add(this.CoordinatesExistingLabel);
            this.panel1.Controls.Add(this.AxisExistingComboBox);
            this.panel1.Controls.Add(this.CoordinatesExistingComboBox);
            this.panel1.Location = new System.Drawing.Point(942, 112);
            this.panel1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(524, 328);
            this.panel1.TabIndex = 18;
            // 
            // ExistingConfigurationEditLabel
            // 
            this.ExistingConfigurationEditLabel.AutoSize = true;
            this.ExistingConfigurationEditLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ExistingConfigurationEditLabel.Location = new System.Drawing.Point(15, 13);
            this.ExistingConfigurationEditLabel.Name = "ExistingConfigurationEditLabel";
            this.ExistingConfigurationEditLabel.Size = new System.Drawing.Size(211, 17);
            this.ExistingConfigurationEditLabel.TabIndex = 16;
            this.ExistingConfigurationEditLabel.Text = "Existing Configuration (Left)";
            // 
            // JointNameExistingLabel
            // 
            this.JointNameExistingLabel.AutoSize = true;
            this.JointNameExistingLabel.Location = new System.Drawing.Point(44, 88);
            this.JointNameExistingLabel.Name = "JointNameExistingLabel";
            this.JointNameExistingLabel.Size = new System.Drawing.Size(79, 17);
            this.JointNameExistingLabel.TabIndex = 15;
            this.JointNameExistingLabel.Text = "Joint Name";
            this.JointNameExistingLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // LinkNameExistingLabel
            // 
            this.LinkNameExistingLabel.AutoSize = true;
            this.LinkNameExistingLabel.Location = new System.Drawing.Point(49, 48);
            this.LinkNameExistingLabel.Name = "LinkNameExistingLabel";
            this.LinkNameExistingLabel.Size = new System.Drawing.Size(75, 17);
            this.LinkNameExistingLabel.TabIndex = 14;
            this.LinkNameExistingLabel.Text = "Link Name";
            this.LinkNameExistingLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // JointNameExistingTextBox
            // 
            this.JointNameExistingTextBox.Location = new System.Drawing.Point(133, 88);
            this.JointNameExistingTextBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.JointNameExistingTextBox.Name = "JointNameExistingTextBox";
            this.JointNameExistingTextBox.Size = new System.Drawing.Size(374, 22);
            this.JointNameExistingTextBox.TabIndex = 13;
            // 
            // LinkNameExistingTextBox
            // 
            this.LinkNameExistingTextBox.Location = new System.Drawing.Point(133, 48);
            this.LinkNameExistingTextBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.LinkNameExistingTextBox.Name = "LinkNameExistingTextBox";
            this.LinkNameExistingTextBox.Size = new System.Drawing.Size(374, 22);
            this.LinkNameExistingTextBox.TabIndex = 12;
            // 
            // ResetExistingButton
            // 
            this.ResetExistingButton.Location = new System.Drawing.Point(18, 288);
            this.ResetExistingButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.ResetExistingButton.Name = "ResetExistingButton";
            this.ResetExistingButton.Size = new System.Drawing.Size(98, 30);
            this.ResetExistingButton.TabIndex = 11;
            this.ResetExistingButton.Text = "Reset";
            this.ResetExistingButton.UseVisualStyleBackColor = true;
            this.ResetExistingButton.Click += new System.EventHandler(this.ResetExistingButtonClick);
            // 
            // panel2
            // 
            this.panel2.Controls.Add(this.checkBox6);
            this.panel2.Controls.Add(this.JointNameLoadedField);
            this.panel2.Controls.Add(this.TypeLoadedField);
            this.panel2.Controls.Add(this.AxisLoadedField);
            this.panel2.Controls.Add(this.CoordinatesLoadedField);
            this.panel2.Controls.Add(this.LoadedConfigurationEditLabel);
            this.panel2.Controls.Add(this.JointNameLoadedLabel);
            this.panel2.Controls.Add(this.LinkNameLoadedLabel);
            this.panel2.Controls.Add(this.LinkNameLoadedTextBox);
            this.panel2.Controls.Add(this.ResetLoadedButton);
            this.panel2.Controls.Add(this.UpdateLoadedButton);
            this.panel2.Controls.Add(this.TypeLoadedLabel);
            this.panel2.Controls.Add(this.AxisLoadedLabel);
            this.panel2.Controls.Add(this.CoordinatesLoadedLabel);
            this.panel2.Location = new System.Drawing.Point(942, 448);
            this.panel2.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.panel2.Name = "panel2";
            this.panel2.Size = new System.Drawing.Size(524, 328);
            this.panel2.TabIndex = 19;
            // 
            // checkBox6
            // 
            this.checkBox6.AutoSize = true;
            this.checkBox6.Location = new System.Drawing.Point(1850, 135);
            this.checkBox6.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.checkBox6.Name = "checkBox6";
            this.checkBox6.Size = new System.Drawing.Size(18, 17);
            this.checkBox6.TabIndex = 28;
            this.checkBox6.UseVisualStyleBackColor = true;
            // 
            // JointNameLoadedField
            // 
            this.JointNameLoadedField.AutoSize = true;
            this.JointNameLoadedField.Location = new System.Drawing.Point(133, 88);
            this.JointNameLoadedField.Name = "JointNameLoadedField";
            this.JointNameLoadedField.Size = new System.Drawing.Size(79, 17);
            this.JointNameLoadedField.TabIndex = 20;
            this.JointNameLoadedField.Text = "Joint Name";
            // 
            // TypeLoadedField
            // 
            this.TypeLoadedField.AutoSize = true;
            this.TypeLoadedField.Location = new System.Drawing.Point(133, 240);
            this.TypeLoadedField.Name = "TypeLoadedField";
            this.TypeLoadedField.Size = new System.Drawing.Size(74, 17);
            this.TypeLoadedField.TabIndex = 19;
            this.TypeLoadedField.Text = "Joint Type";
            // 
            // AxisLoadedField
            // 
            this.AxisLoadedField.AutoSize = true;
            this.AxisLoadedField.Location = new System.Drawing.Point(133, 200);
            this.AxisLoadedField.Name = "AxisLoadedField";
            this.AxisLoadedField.Size = new System.Drawing.Size(103, 17);
            this.AxisLoadedField.TabIndex = 18;
            this.AxisLoadedField.Text = "Reference Axis";
            // 
            // CoordinatesLoadedField
            // 
            this.CoordinatesLoadedField.AutoSize = true;
            this.CoordinatesLoadedField.Location = new System.Drawing.Point(133, 160);
            this.CoordinatesLoadedField.Name = "CoordinatesLoadedField";
            this.CoordinatesLoadedField.Size = new System.Drawing.Size(197, 17);
            this.CoordinatesLoadedField.TabIndex = 17;
            this.CoordinatesLoadedField.Text = "Reference Coordinate System";
            // 
            // LoadedConfigurationEditLabel
            // 
            this.LoadedConfigurationEditLabel.AutoSize = true;
            this.LoadedConfigurationEditLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LoadedConfigurationEditLabel.Location = new System.Drawing.Point(15, 13);
            this.LoadedConfigurationEditLabel.Name = "LoadedConfigurationEditLabel";
            this.LoadedConfigurationEditLabel.Size = new System.Drawing.Size(219, 17);
            this.LoadedConfigurationEditLabel.TabIndex = 16;
            this.LoadedConfigurationEditLabel.Text = "Loaded Configuration (Right)";
            // 
            // JointNameLoadedLabel
            // 
            this.JointNameLoadedLabel.AutoSize = true;
            this.JointNameLoadedLabel.Location = new System.Drawing.Point(44, 88);
            this.JointNameLoadedLabel.Name = "JointNameLoadedLabel";
            this.JointNameLoadedLabel.Size = new System.Drawing.Size(79, 17);
            this.JointNameLoadedLabel.TabIndex = 15;
            this.JointNameLoadedLabel.Text = "Joint Name";
            this.JointNameLoadedLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // LinkNameLoadedLabel
            // 
            this.LinkNameLoadedLabel.AutoSize = true;
            this.LinkNameLoadedLabel.Location = new System.Drawing.Point(49, 46);
            this.LinkNameLoadedLabel.Name = "LinkNameLoadedLabel";
            this.LinkNameLoadedLabel.Size = new System.Drawing.Size(75, 17);
            this.LinkNameLoadedLabel.TabIndex = 14;
            this.LinkNameLoadedLabel.Text = "Link Name";
            this.LinkNameLoadedLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // LinkNameLoadedTextBox
            // 
            this.LinkNameLoadedTextBox.Location = new System.Drawing.Point(133, 48);
            this.LinkNameLoadedTextBox.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.LinkNameLoadedTextBox.Name = "LinkNameLoadedTextBox";
            this.LinkNameLoadedTextBox.Size = new System.Drawing.Size(374, 22);
            this.LinkNameLoadedTextBox.TabIndex = 12;
            // 
            // ResetLoadedButton
            // 
            this.ResetLoadedButton.Location = new System.Drawing.Point(18, 288);
            this.ResetLoadedButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.ResetLoadedButton.Name = "ResetLoadedButton";
            this.ResetLoadedButton.Size = new System.Drawing.Size(98, 30);
            this.ResetLoadedButton.TabIndex = 11;
            this.ResetLoadedButton.Text = "Reset";
            this.ResetLoadedButton.UseVisualStyleBackColor = true;
            this.ResetLoadedButton.Click += new System.EventHandler(this.ResetLoadedButtonClick);
            // 
            // UpdateLoadedButton
            // 
            this.UpdateLoadedButton.Location = new System.Drawing.Point(409, 288);
            this.UpdateLoadedButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.UpdateLoadedButton.Name = "UpdateLoadedButton";
            this.UpdateLoadedButton.Size = new System.Drawing.Size(98, 30);
            this.UpdateLoadedButton.TabIndex = 10;
            this.UpdateLoadedButton.Text = "Update";
            this.UpdateLoadedButton.UseVisualStyleBackColor = true;
            this.UpdateLoadedButton.Click += new System.EventHandler(this.UpdateLoadedButtonClick);
            // 
            // TypeLoadedLabel
            // 
            this.TypeLoadedLabel.AutoSize = true;
            this.TypeLoadedLabel.Location = new System.Drawing.Point(85, 240);
            this.TypeLoadedLabel.Name = "TypeLoadedLabel";
            this.TypeLoadedLabel.Size = new System.Drawing.Size(40, 17);
            this.TypeLoadedLabel.TabIndex = 8;
            this.TypeLoadedLabel.Text = "Type";
            this.TypeLoadedLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // AxisLoadedLabel
            // 
            this.AxisLoadedLabel.AutoSize = true;
            this.AxisLoadedLabel.Location = new System.Drawing.Point(90, 200);
            this.AxisLoadedLabel.Name = "AxisLoadedLabel";
            this.AxisLoadedLabel.Size = new System.Drawing.Size(33, 17);
            this.AxisLoadedLabel.TabIndex = 7;
            this.AxisLoadedLabel.Text = "Axis";
            this.AxisLoadedLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // CoordinatesLoadedLabel
            // 
            this.CoordinatesLoadedLabel.AutoSize = true;
            this.CoordinatesLoadedLabel.Location = new System.Drawing.Point(39, 160);
            this.CoordinatesLoadedLabel.Name = "CoordinatesLoadedLabel";
            this.CoordinatesLoadedLabel.Size = new System.Drawing.Size(84, 17);
            this.CoordinatesLoadedLabel.TabIndex = 6;
            this.CoordinatesLoadedLabel.Text = "Coordinates";
            this.CoordinatesLoadedLabel.TextAlign = System.Drawing.ContentAlignment.TopRight;
            // 
            // ExistingConfigurationLabel
            // 
            this.ExistingConfigurationLabel.AutoSize = true;
            this.ExistingConfigurationLabel.Location = new System.Drawing.Point(20, 80);
            this.ExistingConfigurationLabel.Name = "ExistingConfigurationLabel";
            this.ExistingConfigurationLabel.Size = new System.Drawing.Size(144, 17);
            this.ExistingConfigurationLabel.TabIndex = 20;
            this.ExistingConfigurationLabel.Text = "Existing Configuration";
            // 
            // LoadedConfigurationLabel
            // 
            this.LoadedConfigurationLabel.AutoSize = true;
            this.LoadedConfigurationLabel.Location = new System.Drawing.Point(476, 80);
            this.LoadedConfigurationLabel.Name = "LoadedConfigurationLabel";
            this.LoadedConfigurationLabel.Size = new System.Drawing.Size(144, 17);
            this.LoadedConfigurationLabel.TabIndex = 21;
            this.LoadedConfigurationLabel.Text = "Loaded Configuration";
            // 
            // MergeButton
            // 
            this.MergeButton.Location = new System.Drawing.Point(1351, 858);
            this.MergeButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.MergeButton.Name = "MergeButton";
            this.MergeButton.Size = new System.Drawing.Size(98, 30);
            this.MergeButton.TabIndex = 17;
            this.MergeButton.Text = "Merge";
            this.MergeButton.UseVisualStyleBackColor = true;
            this.MergeButton.Click += new System.EventHandler(this.MergeButtonClick);
            // 
            // CancelMergeButton
            // 
            this.CancelMergeButton.Location = new System.Drawing.Point(960, 858);
            this.CancelMergeButton.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.CancelMergeButton.Name = "CancelMergeButton";
            this.CancelMergeButton.Size = new System.Drawing.Size(98, 30);
            this.CancelMergeButton.TabIndex = 22;
            this.CancelMergeButton.Text = "Cancel";
            this.CancelMergeButton.UseVisualStyleBackColor = true;
            this.CancelMergeButton.Click += new System.EventHandler(this.CancelMergeButtonClick);
            // 
            // MassInertiaExistingRadio
            // 
            this.MassInertiaExistingRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.MassInertiaExistingRadio.AutoSize = true;
            this.MassInertiaExistingRadio.Location = new System.Drawing.Point(22, 28);
            this.MassInertiaExistingRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.MassInertiaExistingRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.MassInertiaExistingRadio.Name = "MassInertiaExistingRadio";
            this.MassInertiaExistingRadio.Size = new System.Drawing.Size(116, 27);
            this.MassInertiaExistingRadio.TabIndex = 62;
            this.MassInertiaExistingRadio.TabStop = true;
            this.MassInertiaExistingRadio.Text = "Existing Values";
            this.MassInertiaExistingRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.MassInertiaExistingRadio.UseVisualStyleBackColor = true;
            // 
            // MassInertiaLoadedRadio
            // 
            this.MassInertiaLoadedRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.MassInertiaLoadedRadio.AutoSize = true;
            this.MassInertiaLoadedRadio.Checked = true;
            this.MassInertiaLoadedRadio.Location = new System.Drawing.Point(22, 60);
            this.MassInertiaLoadedRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.MassInertiaLoadedRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.MassInertiaLoadedRadio.Name = "MassInertiaLoadedRadio";
            this.MassInertiaLoadedRadio.Size = new System.Drawing.Size(116, 27);
            this.MassInertiaLoadedRadio.TabIndex = 66;
            this.MassInertiaLoadedRadio.TabStop = true;
            this.MassInertiaLoadedRadio.Text = "Loaded Values";
            this.MassInertiaLoadedRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.MassInertiaLoadedRadio.UseVisualStyleBackColor = true;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.MassInertiaExistingRadio);
            this.groupBox1.Controls.Add(this.MassInertiaLoadedRadio);
            this.groupBox1.Location = new System.Drawing.Point(36, 792);
            this.groupBox1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Size = new System.Drawing.Size(160, 96);
            this.groupBox1.TabIndex = 70;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Mass and Inertia";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.VisualExistingRadio);
            this.groupBox2.Controls.Add(this.VisualLoadedRadio);
            this.groupBox2.Location = new System.Drawing.Point(267, 792);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Size = new System.Drawing.Size(160, 96);
            this.groupBox2.TabIndex = 71;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Visual Properties";
            // 
            // VisualExistingRadio
            // 
            this.VisualExistingRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.VisualExistingRadio.AutoSize = true;
            this.VisualExistingRadio.Location = new System.Drawing.Point(22, 28);
            this.VisualExistingRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.VisualExistingRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.VisualExistingRadio.Name = "VisualExistingRadio";
            this.VisualExistingRadio.Size = new System.Drawing.Size(116, 27);
            this.VisualExistingRadio.TabIndex = 62;
            this.VisualExistingRadio.TabStop = true;
            this.VisualExistingRadio.Text = "Existing Values";
            this.VisualExistingRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.VisualExistingRadio.UseVisualStyleBackColor = true;
            // 
            // VisualLoadedRadio
            // 
            this.VisualLoadedRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.VisualLoadedRadio.AutoSize = true;
            this.VisualLoadedRadio.Checked = true;
            this.VisualLoadedRadio.Location = new System.Drawing.Point(22, 60);
            this.VisualLoadedRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.VisualLoadedRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.VisualLoadedRadio.Name = "VisualLoadedRadio";
            this.VisualLoadedRadio.Size = new System.Drawing.Size(116, 27);
            this.VisualLoadedRadio.TabIndex = 66;
            this.VisualLoadedRadio.TabStop = true;
            this.VisualLoadedRadio.Text = "Loaded Values";
            this.VisualLoadedRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.VisualLoadedRadio.UseVisualStyleBackColor = true;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.JointKinematicsExistingRadio);
            this.groupBox3.Controls.Add(this.JointKinematicsLoadedRadio);
            this.groupBox3.Location = new System.Drawing.Point(498, 792);
            this.groupBox3.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Size = new System.Drawing.Size(160, 96);
            this.groupBox3.TabIndex = 72;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Joint Kinematics";
            // 
            // JointKinematicsExistingRadio
            // 
            this.JointKinematicsExistingRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.JointKinematicsExistingRadio.AutoSize = true;
            this.JointKinematicsExistingRadio.Checked = true;
            this.JointKinematicsExistingRadio.Location = new System.Drawing.Point(22, 28);
            this.JointKinematicsExistingRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.JointKinematicsExistingRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.JointKinematicsExistingRadio.Name = "JointKinematicsExistingRadio";
            this.JointKinematicsExistingRadio.Size = new System.Drawing.Size(116, 27);
            this.JointKinematicsExistingRadio.TabIndex = 62;
            this.JointKinematicsExistingRadio.TabStop = true;
            this.JointKinematicsExistingRadio.Text = "Existing Values";
            this.JointKinematicsExistingRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.JointKinematicsExistingRadio.UseVisualStyleBackColor = true;
            // 
            // JointKinematicsLoadedRadio
            // 
            this.JointKinematicsLoadedRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.JointKinematicsLoadedRadio.AutoSize = true;
            this.JointKinematicsLoadedRadio.Location = new System.Drawing.Point(22, 60);
            this.JointKinematicsLoadedRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.JointKinematicsLoadedRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.JointKinematicsLoadedRadio.Name = "JointKinematicsLoadedRadio";
            this.JointKinematicsLoadedRadio.Size = new System.Drawing.Size(116, 27);
            this.JointKinematicsLoadedRadio.TabIndex = 66;
            this.JointKinematicsLoadedRadio.TabStop = true;
            this.JointKinematicsLoadedRadio.Text = "Loaded Values";
            this.JointKinematicsLoadedRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.JointKinematicsLoadedRadio.UseVisualStyleBackColor = true;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.JointOtherExistingRadio);
            this.groupBox4.Controls.Add(this.JointOtherLoadedRadio);
            this.groupBox4.Location = new System.Drawing.Point(729, 792);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Size = new System.Drawing.Size(160, 96);
            this.groupBox4.TabIndex = 73;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "Other Joint Values";
            // 
            // JointOtherExistingRadio
            // 
            this.JointOtherExistingRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.JointOtherExistingRadio.AutoSize = true;
            this.JointOtherExistingRadio.Location = new System.Drawing.Point(22, 28);
            this.JointOtherExistingRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.JointOtherExistingRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.JointOtherExistingRadio.Name = "JointOtherExistingRadio";
            this.JointOtherExistingRadio.Size = new System.Drawing.Size(116, 27);
            this.JointOtherExistingRadio.TabIndex = 62;
            this.JointOtherExistingRadio.TabStop = true;
            this.JointOtherExistingRadio.Text = "Existing Values";
            this.JointOtherExistingRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.JointOtherExistingRadio.UseVisualStyleBackColor = true;
            // 
            // JointOtherLoadedRadio
            // 
            this.JointOtherLoadedRadio.Appearance = System.Windows.Forms.Appearance.Button;
            this.JointOtherLoadedRadio.AutoSize = true;
            this.JointOtherLoadedRadio.Checked = true;
            this.JointOtherLoadedRadio.Location = new System.Drawing.Point(22, 60);
            this.JointOtherLoadedRadio.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.JointOtherLoadedRadio.MinimumSize = new System.Drawing.Size(116, 0);
            this.JointOtherLoadedRadio.Name = "JointOtherLoadedRadio";
            this.JointOtherLoadedRadio.Size = new System.Drawing.Size(116, 27);
            this.JointOtherLoadedRadio.TabIndex = 66;
            this.JointOtherLoadedRadio.TabStop = true;
            this.JointOtherLoadedRadio.Text = "Loaded Values";
            this.JointOtherLoadedRadio.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.JointOtherLoadedRadio.UseVisualStyleBackColor = true;
            // 
            // TreeMerge
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1483, 899);
            this.Controls.Add(this.groupBox4);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.CancelMergeButton);
            this.Controls.Add(this.MergeButton);
            this.Controls.Add(this.LoadedConfigurationLabel);
            this.Controls.Add(this.ExistingConfigurationLabel);
            this.Controls.Add(this.panel2);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.DescriptionLabel);
            this.Controls.Add(this.TitleLabel);
            this.Controls.Add(this.ExistingTreeView);
            this.Controls.Add(this.LoadedTreeView);
            this.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Name = "TreeMerge";
            this.Text = "TreeMerge";
            this.Load += new System.EventHandler(this.TreeMerge_Load);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            this.panel2.ResumeLayout(false);
            this.panel2.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }
        #endregion

        private System.Windows.Forms.TreeView LoadedTreeView;
        private System.Windows.Forms.TreeView ExistingTreeView;
        private System.Windows.Forms.Label TitleLabel;
        private System.Windows.Forms.Label DescriptionLabel;
        private System.Windows.Forms.ComboBox CoordinatesExistingComboBox;
        private System.Windows.Forms.ComboBox AxisExistingComboBox;
        private System.Windows.Forms.Label CoordinatesExistingLabel;
        private System.Windows.Forms.Label AxisExistingLabel;
        private System.Windows.Forms.Label TypeExistingLabel;
        private System.Windows.Forms.ComboBox TypeExistingComboBox;
        private System.Windows.Forms.Button UpdateExistingButton;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label ExistingConfigurationEditLabel;
        private System.Windows.Forms.Label JointNameExistingLabel;
        private System.Windows.Forms.Label LinkNameExistingLabel;
        private System.Windows.Forms.TextBox JointNameExistingTextBox;
        private System.Windows.Forms.TextBox LinkNameExistingTextBox;
        private System.Windows.Forms.Button ResetExistingButton;
        private System.Windows.Forms.Panel panel2;
        private System.Windows.Forms.Label LoadedConfigurationEditLabel;
        private System.Windows.Forms.Label JointNameLoadedLabel;
        private System.Windows.Forms.Label LinkNameLoadedLabel;
        private System.Windows.Forms.TextBox LinkNameLoadedTextBox;
        private System.Windows.Forms.Button ResetLoadedButton;
        private System.Windows.Forms.Button UpdateLoadedButton;
        private System.Windows.Forms.Label TypeLoadedLabel;
        private System.Windows.Forms.Label AxisLoadedLabel;
        private System.Windows.Forms.Label CoordinatesLoadedLabel;
        private System.Windows.Forms.Label ExistingConfigurationLabel;
        private System.Windows.Forms.Label LoadedConfigurationLabel;
        private System.Windows.Forms.Button MergeButton;
        private System.Windows.Forms.Button CancelMergeButton;
        private System.Windows.Forms.CheckBox checkBox6;
        private System.Windows.Forms.Label JointNameLoadedField;
        private System.Windows.Forms.Label TypeLoadedField;
        private System.Windows.Forms.Label AxisLoadedField;
        private System.Windows.Forms.Label CoordinatesLoadedField;
        private System.ComponentModel.BackgroundWorker backgroundWorker1;
        private System.Windows.Forms.RadioButton MassInertiaExistingRadio;
        private System.Windows.Forms.RadioButton MassInertiaLoadedRadio;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.RadioButton VisualExistingRadio;
        private System.Windows.Forms.RadioButton VisualLoadedRadio;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.RadioButton JointKinematicsExistingRadio;
        private System.Windows.Forms.RadioButton JointKinematicsLoadedRadio;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.RadioButton JointOtherExistingRadio;
        private System.Windows.Forms.RadioButton JointOtherLoadedRadio;
    }
}