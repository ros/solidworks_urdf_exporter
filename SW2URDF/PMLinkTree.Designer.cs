namespace SW2URDF
{
    partial class PMLinkTree
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
            this.treeView_linkTree = new System.Windows.Forms.TreeView();
            this.SuspendLayout();
            // 
            // treeView_linkTree
            // 
            this.treeView_linkTree.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom)
                        | System.Windows.Forms.AnchorStyles.Left)
                        | System.Windows.Forms.AnchorStyles.Right)));
            this.treeView_linkTree.Location = new System.Drawing.Point(12, 12);
            this.treeView_linkTree.Name = "treeView_linkTree";
            this.treeView_linkTree.Size = new System.Drawing.Size(259, 538);
            this.treeView_linkTree.TabIndex = 0;
            this.treeView_linkTree.AfterSelect += new System.Windows.Forms.TreeViewEventHandler(this.treeView_linkTree_AfterSelect);
            // 
            // PMLinkTree
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(283, 562);
            this.Controls.Add(this.treeView_linkTree);
            this.Name = "PMLinkTree";
            this.ResumeLayout(false);

        }
        public System.Windows.Forms.TreeView getTree()
        {
            return treeView_linkTree;
        }


        #endregion

        private System.Windows.Forms.TreeView treeView_linkTree;
    }
}