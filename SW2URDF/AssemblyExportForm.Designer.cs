namespace SW2URDF
{
    partial class AssemblyExportForm
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
            this.button_link_next = new System.Windows.Forms.Button();
            this.button_link_cancel = new System.Windows.Forms.Button();
            this.checkedListBox1 = new System.Windows.Forms.CheckedListBox();
            this.label1 = new System.Windows.Forms.Label();
            this.panel_joint = new System.Windows.Forms.Panel();
            this.panel_mesh = new System.Windows.Forms.Panel();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.trackBar_mesh_visual = new System.Windows.Forms.TrackBar();
            this.trackBar_mesh_collision = new System.Windows.Forms.TrackBar();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.button_mesh_cancel = new System.Windows.Forms.Button();
            this.button_mesh_previous = new System.Windows.Forms.Button();
            this.button_mesh_finish = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.button_joint_cancel = new System.Windows.Forms.Button();
            this.button_joint_previous = new System.Windows.Forms.Button();
            this.button_joint_next = new System.Windows.Forms.Button();
            this.button_select = new System.Windows.Forms.Button();
            this.button_deselect = new System.Windows.Forms.Button();
            this.panel_joint.SuspendLayout();
            this.panel_mesh.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_mesh_visual)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_mesh_collision)).BeginInit();
            this.SuspendLayout();
            // 
            // button_link_next
            // 
            this.button_link_next.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button_link_next.Location = new System.Drawing.Point(545, 479);
            this.button_link_next.Name = "button_link_next";
            this.button_link_next.Size = new System.Drawing.Size(75, 23);
            this.button_link_next.TabIndex = 0;
            this.button_link_next.Text = "Next";
            this.button_link_next.UseVisualStyleBackColor = true;
            this.button_link_next.Click += new System.EventHandler(this.button_link_next_Click);
            // 
            // button_link_cancel
            // 
            this.button_link_cancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.button_link_cancel.Location = new System.Drawing.Point(20, 479);
            this.button_link_cancel.Name = "button_link_cancel";
            this.button_link_cancel.Size = new System.Drawing.Size(75, 23);
            this.button_link_cancel.TabIndex = 1;
            this.button_link_cancel.Text = "Cancel";
            this.button_link_cancel.UseVisualStyleBackColor = true;
            this.button_link_cancel.Click += new System.EventHandler(this.button_link_cancel_Click);
            // 
            // checkedListBox1
            // 
            this.checkedListBox1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.checkedListBox1.FormattingEnabled = true;
            this.checkedListBox1.Location = new System.Drawing.Point(20, 110);
            this.checkedListBox1.MaximumSize = new System.Drawing.Size(10000, 10000);
            this.checkedListBox1.Name = "checkedListBox1";
            this.checkedListBox1.Size = new System.Drawing.Size(600, 349);
            this.checkedListBox1.TabIndex = 2;
            this.checkedListBox1.SelectedIndexChanged += new System.EventHandler(this.checkedListBox1_SelectedIndexChanged);
            // 
            // label1
            // 
            this.label1.Location = new System.Drawing.Point(20, 26);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(464, 50);
            this.label1.TabIndex = 3;
            this.label1.Text = "Choose which parts and assemblies will be converted into independent links. Items" +
                " not selected will be grouped with the higher lever assembly.";
            // 
            // panel_joint
            // 
            this.panel_joint.Controls.Add(this.panel_mesh);
            this.panel_joint.Controls.Add(this.label2);
            this.panel_joint.Controls.Add(this.groupBox1);
            this.panel_joint.Controls.Add(this.button_joint_cancel);
            this.panel_joint.Controls.Add(this.button_joint_previous);
            this.panel_joint.Controls.Add(this.button_joint_next);
            this.panel_joint.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel_joint.Location = new System.Drawing.Point(0, 0);
            this.panel_joint.Name = "panel_joint";
            this.panel_joint.Size = new System.Drawing.Size(640, 512);
            this.panel_joint.TabIndex = 4;
            this.panel_joint.Visible = false;
            // 
            // panel_mesh
            // 
            this.panel_mesh.Controls.Add(this.label6);
            this.panel_mesh.Controls.Add(this.label5);
            this.panel_mesh.Controls.Add(this.groupBox2);
            this.panel_mesh.Controls.Add(this.trackBar_mesh_visual);
            this.panel_mesh.Controls.Add(this.trackBar_mesh_collision);
            this.panel_mesh.Controls.Add(this.label4);
            this.panel_mesh.Controls.Add(this.label3);
            this.panel_mesh.Controls.Add(this.button_mesh_cancel);
            this.panel_mesh.Controls.Add(this.button_mesh_previous);
            this.panel_mesh.Controls.Add(this.button_mesh_finish);
            this.panel_mesh.Dock = System.Windows.Forms.DockStyle.Fill;
            this.panel_mesh.Location = new System.Drawing.Point(0, 0);
            this.panel_mesh.Name = "panel_mesh";
            this.panel_mesh.Size = new System.Drawing.Size(640, 512);
            this.panel_mesh.TabIndex = 5;
            this.panel_mesh.Visible = false;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(588, 62);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(13, 13);
            this.label6.TabIndex = 9;
            this.label6.Text = "0";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(289, 62);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(13, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "0";
            // 
            // groupBox2
            // 
            this.groupBox2.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox2.AutoSize = true;
            this.groupBox2.Location = new System.Drawing.Point(20, 113);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(600, 354);
            this.groupBox2.TabIndex = 7;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Mesh Configurations";
            // 
            // trackBar_mesh_visual
            // 
            this.trackBar_mesh_visual.Location = new System.Drawing.Point(329, 62);
            this.trackBar_mesh_visual.Name = "trackBar_mesh_visual";
            this.trackBar_mesh_visual.Size = new System.Drawing.Size(253, 45);
            this.trackBar_mesh_visual.TabIndex = 6;
            this.trackBar_mesh_visual.Scroll += new System.EventHandler(this.trackBar_mesh_visual_Scroll);
            // 
            // trackBar_mesh_collision
            // 
            this.trackBar_mesh_collision.Location = new System.Drawing.Point(30, 62);
            this.trackBar_mesh_collision.Name = "trackBar_mesh_collision";
            this.trackBar_mesh_collision.Size = new System.Drawing.Size(253, 45);
            this.trackBar_mesh_collision.TabIndex = 5;
            this.trackBar_mesh_collision.Scroll += new System.EventHandler(this.trackBar_mesh_collision_Scroll);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(329, 43);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(243, 13);
            this.label4.TabIndex = 4;
            this.label4.Text = "Maximum number of triangles for each visual mesh";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(30, 43);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(253, 13);
            this.label3.TabIndex = 3;
            this.label3.Text = "Maximum number of triangles for each collision mesh";
            // 
            // button_mesh_cancel
            // 
            this.button_mesh_cancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.button_mesh_cancel.Location = new System.Drawing.Point(20, 479);
            this.button_mesh_cancel.Name = "button_mesh_cancel";
            this.button_mesh_cancel.Size = new System.Drawing.Size(75, 23);
            this.button_mesh_cancel.TabIndex = 2;
            this.button_mesh_cancel.Text = "Cancel";
            this.button_mesh_cancel.UseVisualStyleBackColor = true;
            this.button_mesh_cancel.Click += new System.EventHandler(this.button_mesh_cancel_Click);
            // 
            // button_mesh_previous
            // 
            this.button_mesh_previous.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button_mesh_previous.Location = new System.Drawing.Point(450, 479);
            this.button_mesh_previous.Name = "button_mesh_previous";
            this.button_mesh_previous.Size = new System.Drawing.Size(75, 23);
            this.button_mesh_previous.TabIndex = 1;
            this.button_mesh_previous.Text = "Previous";
            this.button_mesh_previous.UseVisualStyleBackColor = true;
            this.button_mesh_previous.Click += new System.EventHandler(this.button_mesh_previous_Click);
            // 
            // button_mesh_finish
            // 
            this.button_mesh_finish.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button_mesh_finish.Location = new System.Drawing.Point(545, 479);
            this.button_mesh_finish.Name = "button_mesh_finish";
            this.button_mesh_finish.Size = new System.Drawing.Size(75, 23);
            this.button_mesh_finish.TabIndex = 0;
            this.button_mesh_finish.Text = "Finish";
            this.button_mesh_finish.UseVisualStyleBackColor = true;
            this.button_mesh_finish.Click += new System.EventHandler(this.button_mesh_finish_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(27, 26);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(160, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Configure custom joint properties";
            // 
            // groupBox1
            // 
            this.groupBox1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox1.AutoSize = true;
            this.groupBox1.Location = new System.Drawing.Point(20, 80);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(600, 378);
            this.groupBox1.TabIndex = 3;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Joint Configurations";
            this.groupBox1.Enter += new System.EventHandler(this.groupBox1_Enter);
            // 
            // button_joint_cancel
            // 
            this.button_joint_cancel.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.button_joint_cancel.Location = new System.Drawing.Point(20, 479);
            this.button_joint_cancel.Name = "button_joint_cancel";
            this.button_joint_cancel.Size = new System.Drawing.Size(75, 23);
            this.button_joint_cancel.TabIndex = 2;
            this.button_joint_cancel.Text = "Cancel";
            this.button_joint_cancel.UseVisualStyleBackColor = true;
            this.button_joint_cancel.Click += new System.EventHandler(this.button_joint_cancel_Click);
            // 
            // button_joint_previous
            // 
            this.button_joint_previous.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button_joint_previous.Location = new System.Drawing.Point(450, 479);
            this.button_joint_previous.Name = "button_joint_previous";
            this.button_joint_previous.Size = new System.Drawing.Size(75, 23);
            this.button_joint_previous.TabIndex = 1;
            this.button_joint_previous.Text = "Previous";
            this.button_joint_previous.UseVisualStyleBackColor = true;
            this.button_joint_previous.Click += new System.EventHandler(this.button_joint_previous_Click);
            // 
            // button_joint_next
            // 
            this.button_joint_next.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.button_joint_next.Location = new System.Drawing.Point(545, 479);
            this.button_joint_next.Name = "button_joint_next";
            this.button_joint_next.Size = new System.Drawing.Size(75, 23);
            this.button_joint_next.TabIndex = 0;
            this.button_joint_next.Text = "Next";
            this.button_joint_next.UseVisualStyleBackColor = true;
            this.button_joint_next.Click += new System.EventHandler(this.button_joint_next_Click);
            // 
            // button_select
            // 
            this.button_select.Location = new System.Drawing.Point(20, 81);
            this.button_select.Name = "button_select";
            this.button_select.Size = new System.Drawing.Size(75, 23);
            this.button_select.TabIndex = 5;
            this.button_select.Text = "Select All";
            this.button_select.UseVisualStyleBackColor = true;
            this.button_select.Click += new System.EventHandler(this.button_select_Click);
            // 
            // button_deselect
            // 
            this.button_deselect.Location = new System.Drawing.Point(101, 81);
            this.button_deselect.Name = "button_deselect";
            this.button_deselect.Size = new System.Drawing.Size(75, 23);
            this.button_deselect.TabIndex = 6;
            this.button_deselect.Text = "Deselect All";
            this.button_deselect.UseVisualStyleBackColor = true;
            this.button_deselect.Click += new System.EventHandler(this.button_deselect_Click);
            // 
            // AssemblyExportForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(640, 512);
            this.Controls.Add(this.panel_joint);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.checkedListBox1);
            this.Controls.Add(this.button_link_cancel);
            this.Controls.Add(this.button_link_next);
            this.Controls.Add(this.button_deselect);
            this.Controls.Add(this.button_select);
            this.Name = "AssemblyExportForm";
            this.Text = "SolidWorks Assembly to URDF Exporter";
            this.Load += new System.EventHandler(this.AssemblyExportForm_Load);
            this.panel_joint.ResumeLayout(false);
            this.panel_joint.PerformLayout();
            this.panel_mesh.ResumeLayout(false);
            this.panel_mesh.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_mesh_visual)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar_mesh_collision)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button button_link_next;
        private System.Windows.Forms.Button button_link_cancel;
        private System.Windows.Forms.CheckedListBox checkedListBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Panel panel_joint;
        private System.Windows.Forms.Button button_joint_cancel;
        private System.Windows.Forms.Button button_joint_previous;
        private System.Windows.Forms.Button button_joint_next;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Panel panel_mesh;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TrackBar trackBar_mesh_visual;
        private System.Windows.Forms.TrackBar trackBar_mesh_collision;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Button button_mesh_cancel;
        private System.Windows.Forms.Button button_mesh_previous;
        private System.Windows.Forms.Button button_mesh_finish;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Button button_select;
        private System.Windows.Forms.Button button_deselect;
    }
}