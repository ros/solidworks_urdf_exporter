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

        }

        private void button_link_next_Click(object sender, EventArgs e)
        {
            Exporter.mRobot = createRobotFromTreeView();
            //panel_links.Visible = true;
        }

        private void button_link_cancel_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void groupBox1_Enter(object sender, EventArgs e)
        {

        }

        //Joint form configuration controls

        private void button_select_Click(object sender, EventArgs e)
        {

        }

        private void button_deselect_Click(object sender, EventArgs e)
        {

        }


        #region Dragging and Dropping Links
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
            e.Effect = DragDropEffects.Move;
        }
        private void treeView_linktree_DragEnter(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = treeView_linktree.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            treeView_linktree.SelectedNode = treeView_linktree.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }
        private void treeView_linktree_DragDrop(object sender, System.Windows.Forms.DragEventArgs e)
        {
            // Retrieve the client coordinates of the drop location.
            Point targetPoint = treeView_linktree.PointToClient(new Point(e.X, e.Y));

            // Retrieve the node at the drop location.
            LinkNode targetNode = (LinkNode)treeView_linktree.GetNodeAt(targetPoint);

            // Retrieve the node that was dragged.
            LinkNode draggedNode = (LinkNode)e.Data.GetData(typeof(LinkNode));
            if (draggedNode == null)
            {
                LinkItem item = (LinkItem)e.Data.GetData(typeof(LinkItem));
                if (item == null)
                {
                    return;
                }
                draggedNode = LinkItemToLinkNode(item);
                listBox_deleted.Items.Remove(item);
            }

            // If dragging a node closer to root level onto a target node further, do parent swapping kung fu
            if (draggedNode.Level < targetNode.Level)
            {
                LinkNode newParent = targetNode;
                LinkNode newChild = draggedNode;
                LinkNode sameGrandparent = (LinkNode)draggedNode.Parent;
                newParent.Remove();
                newChild.Remove();
                newParent.Nodes.Add(newChild);
                foreach (LinkNode node in newChild.Nodes)
                {
                    LinkNode newNode = node;
                    newParent.Nodes.Add(newNode);
                    newChild.Nodes.Remove(node);
                }
                if (sameGrandparent == null)
                {
                    treeView_linktree.Nodes.Add(newParent);
                }
                else
                {
                    sameGrandparent.Nodes.Add(newParent);
                }
            }
            draggedNode.Remove();
            if (targetNode == null)
            {
                targetNode = (LinkNode)treeView_linktree.TopNode;
                
                if (targetNode == null)
                {
                    treeView_linktree.Nodes.Add(draggedNode);                    
                }
                else
                {
                    targetNode.Nodes.Add(draggedNode);
                }
            }
            else
            {
                targetNode.Nodes.Add(draggedNode);   
            }
            targetNode.Expand();
        }
        private void listBox_deleted_DragOver(object sender, System.Windows.Forms.DragEventArgs e)
        {
            e.Effect = DragDropEffects.Move;
        }
        private void listBox_deleted_DragEnter(object sender, DragEventArgs e)
        {
            e.Effect = DragDropEffects.Move;
        }
        private void listBox_deleted_DragDrop(object sender, System.Windows.Forms.DragEventArgs e)
        {
            LinkNode draggedNode = (LinkNode)e.Data.GetData(typeof(LinkNode));
            if (draggedNode != null)
            {
                draggedNode.Remove();
                listBox_deleted.Items.Add(LinkNodeToLinkItem(draggedNode));
            }
        }
        private void listBox_deleted_MouseDown(object sender, MouseEventArgs e)
        {
            this.listBox_deleted.DoDragDrop(this.listBox_deleted.SelectedItem, DragDropEffects.Move);
        }
        

        public LinkItem LinkNodeToLinkItem(LinkNode node)
        {
            LinkItem item = new LinkItem();
            item.Name = node.Name;
            item.Link = node.Link;
            item.Text = node.Text;
            return item;
        }
        public LinkNode LinkItemToLinkNode(LinkItem item)
        {
            LinkNode node = new LinkNode();
            node.Name = item.Name;
            node.Link = item.Link;
            node.Text = item.Text;
            return node;
        }
        #endregion

        #region Robot<->Tree
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
        #endregion

        private void textBox_inertial_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_inertial_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixx_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixy_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_ixz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_iyy_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_iyz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_izz_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_mass_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_visual_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void comboBox_materials_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_red_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_green_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_blue_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void domainUpDown_alpha_SelectedItemChanged(object sender, EventArgs e)
        {

        }

        private void textBox_texture_TextChanged(object sender, EventArgs e)
        {

        }

        private void button_texturebrowse_Click(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_x_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_y_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_z_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_roll_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_pitch_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_collision_origin_yaw_TextChanged(object sender, EventArgs e)
        {

        }

        private void radioButton3_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void radioButton4_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void textBox_name_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox_save_as_TextChanged(object sender, EventArgs e)
        {

        }

        private void button_savename_browse_Click(object sender, EventArgs e)
        {

        }
    }

    #region Derived classes
    public class LinkNode : TreeNode
    {
        public link Link
        { get; set; }
    }
    public class LinkItem : ListViewItem
    {
        public link Link
        { get; set; }
    }
    #endregion
}