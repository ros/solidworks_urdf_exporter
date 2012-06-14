using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using SolidWorks.Interop.sldworks;

namespace SW2URDF
{
    public partial class AssemblyExportForm : Form
    {
        public SW2URDFExporter Exporter;
        public AssemblyExportForm(ISldWorks iSwApp)
        {
            InitializeComponent();
            Exporter = new SW2URDFExporter(iSwApp);
        }

        //Joint form configuration controls
        private void AssemblyExportForm_Load(object sender, EventArgs e)
        {
            Exporter.getRobotFromActiveModel();
            fillLinkTreeView();
            //for (int i = 0; i < mAssyExporter.mLinks.Count; i++)
            //{
            //    checkedListBox1.Items.Add(mAssyExporter.mLinks.ElementAt(i).name);
            //}
        }

        private void button_link_next_Click(object sender, EventArgs e)
        {
            //for (int i = 0; i < checkedListBox1.Items.Count; i++)
            //{
                
            //}


            panel_links.Visible = true;
        }

        private void button_link_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void checkedListBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        //Joint form configuration controls

        private void button_joint_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void button_joint_previous_Click(object sender, EventArgs e)
        {
            panel_joint.Visible = false;
        }

        private void button_joint_next_Click(object sender, EventArgs e)
        {
            panel_mesh.Visible = true;
        }

        //Mesh form configuration controls
        private void button_mesh_finish_Click(object sender, EventArgs e)
        {

            this.Close();
        }

        private void button_mesh_previous_Click(object sender, EventArgs e)
        {
            panel_mesh.Visible = false;
        }

        private void button_mesh_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void trackBar_mesh_collision_Scroll(object sender, EventArgs e)
        {

        }

        private void trackBar_mesh_visual_Scroll(object sender, EventArgs e)
        {

        }

        private void button_select_Click(object sender, EventArgs e)
        {
            //for (int i = 0; i < checkedListBox1.Items.Count; i++)
            //{
            //    checkedListBox1.SetItemChecked(i, true);
            //}
        }

        private void button_deselect_Click(object sender, EventArgs e)
        {
            //for (int i = 0; i < checkedListBox1.Items.Count; i++)
            //{
            //    checkedListBox1.SetItemChecked(i, false);
            //}
        }

        private void button_links_previous_Click(object sender, EventArgs e)
        {
            panel_links.Visible = false;
        }

        private void button_links_next_Click(object sender, EventArgs e)
        {
            panel_joint.Visible = true;
        }

        private void button_links_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void treeView_linktree_AfterSelect(object sender, TreeViewEventArgs e)
        {

        }

        private void button_promote_parent_Click(object sender, EventArgs e)
        {
            //treeView_linktree.SelectedNode
        }

        private void button_change_parent_Click(object sender, EventArgs e)
        {

        }

        private void button_delete_link_Click(object sender, EventArgs e)
        {

        }
        public void fillLinkTreeView()
        {
            TreeNode baseNode = new TreeNode();
            link baseLink = Exporter.mRobot.getBaseLink();
            baseNode.Name = baseLink.name;
            foreach (link child in baseLink.Children)
            {
                baseNode.Nodes.Add(createTreeNodeFromLink(child));
            }
            treeView_linktree.Nodes.Add(baseNode);

        }
        public TreeNode createTreeNodeFromLink(link Link)
        {
            TreeNode node = new TreeNode();
            node.Name = Link.name;
            foreach (link child in Link.Children)
            {
                node.Nodes.Add(createTreeNodeFromLink(child));
            }
            return node;
        }







    }
}