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
        private int NodeCount, FolderCount;
        private string NodeMap;
        TreeNode sourceNode;
        private StringBuilder NewNodeMap = new StringBuilder(128);
        public SW2URDFExporter Exporter;
        public AssemblyExportForm(ISldWorks iSwApp)
        {
            InitializeComponent();
            Exporter = new SW2URDFExporter(iSwApp);
        }

        //Joint form configuration controls
        private void AssemblyExportForm_Load(object sender, EventArgs e)
        {
            Exporter.createRobotFromActiveModel();
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

        #region microsoft code
        private void treeView_linktree_ItemDrag(object sender, System.Windows.Forms.ItemDragEventArgs e)
        {
            DoDragDrop(e.Item, DragDropEffects.Move);

        }
        private void treeView_linktree_DragOver(object sender, System.Windows.Forms.DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = treeView_linktree.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            treeView_linktree.SelectedNode = treeView_linktree.GetNodeAt(targetPoint);
        }
        private void treeView_linktree_DragEnter(object sender, DragEventArgs e)
        {
            e.Effect = e.AllowedEffect;
        }
        private void treeView_linktree_DragDrop(object sender, System.Windows.Forms.DragEventArgs e)
        {
            // Retrieve the client coordinates of the drop location.
            Point targetPoint = treeView_linktree.PointToClient(new Point(e.X, e.Y));

            // Retrieve the node at the drop location.
            LinkNode targetNode = (LinkNode)treeView_linktree.GetNodeAt(targetPoint);

            // Retrieve the node that was dragged.
            LinkNode draggedNode = (LinkNode)e.Data.GetData(typeof(LinkNode));          
            if (targetNode != null)
            {
                draggedNode.Remove();
                targetNode.Nodes.Add(draggedNode);
                targetNode.Expand();
            }
        }
        #endregion



        public void fillLinkTreeView()
        {
            LinkNode baseNode = new LinkNode();
            link baseLink = Exporter.mRobot.BaseLink;
            baseNode.Name = baseLink.name;
            baseNode.Text = baseLink.name;
            baseNode.Link = baseLink;
            foreach (link child in baseLink.Children)
            {
                baseNode.Nodes.Add(createLinkNodeFromLink(child));
            }
            treeView_linktree.Nodes.Add(baseNode);
            treeView_linktree.ExpandAll();

        }
        public LinkNode createLinkNodeFromLink(link Link)
        {
            LinkNode node = new LinkNode();
            node.Name = Link.name;
            node.Text = Link.name;
            node.Link = Link;
            foreach (link child in Link.Children)
            {
                node.Nodes.Add(createLinkNodeFromLink(child));
            }
            node.Link.Children.Clear(); // Need to erase the children from the embedded link because they may be rearranged later.
            return node;
        }
        public robot createRobotFromTreeView()
        {
            robot Robot = new robot();
            Robot.name = Robot.BaseLink.name;
            foreach (LinkNode node in treeView_linktree.Nodes)
            {
                if (node.Parent == null)
                {
                    Robot.BaseLink = createLinkFromLinkNode(node);
                }
            }
            return Robot;
        }
        public link createLinkFromLinkNode(LinkNode node)
        {
            link Link = new link();
            Link = node.Link;
            foreach (LinkNode child in node.Nodes)
            {
                Link.Children.Add(createLinkFromLinkNode(child)); // Recreates the children of each embedded link
            }
            return Link;
        }
    }
    public class LinkNode : TreeNode
    {
        public link Link
        { get; set; }
    }
}