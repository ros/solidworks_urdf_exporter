using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.IO;
using System.Windows.Forms;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;
using SolidWorksTools;
using SolidWorksTools.File;
using System.Xml;
using System.Xml.Serialization;

namespace SW2URDF
{

    public partial class AssemblyExportForm : Form
    {
        ISldWorks swApp;
        ModelDoc2 ActiveSWModel;
        private StringBuilder NewNodeMap = new StringBuilder(128);
        public SW2URDFExporter Exporter;
        LinkNode previouslySelectedNode;
        public bool AutoUpdatingForm;

        LinkNode BaseNode;

        public AssemblyExportForm(ISldWorks iSwApp, LinkNode node)
        {
            InitializeComponent();
            swApp = iSwApp;
            BaseNode = node;
            ActiveSWModel = swApp.ActiveDoc;
            Exporter = new SW2URDFExporter(iSwApp);
            AutoUpdatingForm = false;
        }

        //Joint form configuration controls
        private void AssemblyExportForm_Load(object sender, EventArgs e)
        {
            fillJointTree();
        }

        private void button_joint_next_Click(object sender, EventArgs e)
        {
            if (!(previouslySelectedNode == null || previouslySelectedNode.Link.Joint == null))
            {
                 saveJointDataFromPropertyBoxes(previouslySelectedNode.Link.Joint);
            }
            previouslySelectedNode = null; // Need to clear this for the link properties page
            //treeView_linkProperties.Nodes.Clear();
            //Exporter.mRobot = createRobotFromTreeView(treeView_jointtree);
            //fillTreeViewFromRobot(Exporter.mRobot, treeView_linkProperties);
            foreach (LinkNode node in treeView_jointtree.Nodes)
            {
                treeView_jointtree.Nodes.Remove(node);
                BaseNode.Nodes.Add(node);
            }
            changeAllNodeFont(BaseNode, new System.Drawing.Font(treeView_jointtree.Font, FontStyle.Regular));
            fillLinkTree();
            panel_link_properties.Visible = true;
            this.Focus();
        }

        private void button_joint_cancel_Click(object sender, EventArgs e)
        {
            if (previouslySelectedNode != null)
            {
                saveJointDataFromPropertyBoxes(previouslySelectedNode.Link.Joint);
            }
            foreach(LinkNode node in treeView_jointtree.Nodes)
            {
                treeView_jointtree.Nodes.Remove(node);
                BaseNode.Nodes.Add(node);
            }
            Exporter.saveConfigTree(BaseNode, true);
            this.Close();
        }
        private void button_links_cancel_Click(object sender, EventArgs e)
        {
            if (previouslySelectedNode != null)
            {
                saveLinkDataFromPropertyBoxes(previouslySelectedNode.Link);
            }
            Exporter.saveConfigTree(BaseNode, true);
            this.Close();
        }

        private void button_links_previous_Click(object sender, EventArgs e)
        {
            LinkNode node = (LinkNode)treeView_linkProperties.SelectedNode;
            if (node != null)
            {
                saveLinkDataFromPropertyBoxes(node.Link);
            }
            previouslySelectedNode = null;
            //treeView_jointtree.Nodes.Clear();
            //Exporter.mRobot = createRobotFromTreeView(treeView_linkProperties);
            //fillTreeViewFromRobot(Exporter.mRobot, treeView_jointtree);
            changeAllNodeFont(BaseNode, new System.Drawing.Font(treeView_jointtree.Font, FontStyle.Regular));
            fillJointTree();
            panel_link_properties.Visible = false;
        }

        private void button_links_finish_Click(object sender, EventArgs e)
        {
            Exporter.saveConfigTree(BaseNode, false);
            SaveFileDialog saveFileDialog1 = new SaveFileDialog();
            saveFileDialog1.RestoreDirectory = true;
            saveFileDialog1.InitialDirectory = Exporter.mSavePath;
            saveFileDialog1.FileName = Exporter.mPackageName;
            if (saveFileDialog1.ShowDialog() == DialogResult.OK)
            {
                Exporter.mSavePath = Path.GetDirectoryName(saveFileDialog1.FileName);
                Exporter.mPackageName = Path.GetFileName(saveFileDialog1.FileName);
                LinkNode node = (LinkNode)treeView_linkProperties.SelectedNode;
                if (node != null)
                {
                    saveLinkDataFromPropertyBoxes(node.Link);
                }
                Exporter.mRobot = createRobotFromTreeView(treeView_linkProperties);

                Exporter.exportRobot();
                this.Close();
            }
        }
        
        private void treeView_linkProperties_AfterSelect(object sender, TreeViewEventArgs e)
        {
            Font fontRegular = new Font(treeView_jointtree.Font, FontStyle.Regular);
            Font fontBold = new Font(treeView_jointtree.Font, FontStyle.Bold);
            if (previouslySelectedNode != null)
            {
                saveLinkDataFromPropertyBoxes(previouslySelectedNode.Link);
                previouslySelectedNode.NodeFont = fontRegular;
            }
            LinkNode node = (LinkNode)e.Node;
            node.NodeFont = fontBold;
            node.Text = node.Text;
            ActiveSWModel.ClearSelection2(true);    
            SelectionMgr manager = ActiveSWModel.SelectionManager;




            SelectData data = manager.CreateSelectData();
            data.Mark = -1;
            if (node.Link.SWComponent != null)
            {
                node.Link.SWComponent.Select4(false, data, false);
            }
            else
            {
                foreach (Component2 component in node.Link.SWcomponents)
                {
                    component.Select4(true, data, false);
                }
            }
            fillLinkPropertyBoxes(node.Link);
            treeView_linkProperties.Focus();
            previouslySelectedNode = node;
        }

        #region Robot<->Tree

        //From the link, this method fills the property boxes on the Link Properties page
        public void fillLinkPropertyBoxes(link Link)
        {

            textBox_collision_origin_x.Text = Link.Collision.Origin.X.ToString("G5");
            textBox_collision_origin_y.Text = Link.Collision.Origin.Y.ToString("G5");
            textBox_collision_origin_z.Text = Link.Collision.Origin.Z.ToString("G5");
            textBox_collision_origin_roll.Text = Link.Collision.Origin.Roll.ToString("G5");
            textBox_collision_origin_pitch.Text = Link.Collision.Origin.Pitch.ToString("G5");
            textBox_collision_origin_yaw.Text = Link.Collision.Origin.Yaw.ToString("G5");

            textBox_visual_origin_x.Text = Link.Visual.Origin.X.ToString("G5");
            textBox_visual_origin_y.Text = Link.Visual.Origin.Y.ToString("G5");
            textBox_visual_origin_z.Text = Link.Visual.Origin.Z.ToString("G5");
            textBox_visual_origin_roll.Text = Link.Visual.Origin.Roll.ToString("G5");
            textBox_visual_origin_pitch.Text = Link.Visual.Origin.Pitch.ToString("G5");
            textBox_visual_origin_yaw.Text = Link.Visual.Origin.Yaw.ToString("G5");

            textBox_inertial_origin_x.Text = Link.Inertial.Origin.X.ToString("G5");
            textBox_inertial_origin_y.Text = Link.Inertial.Origin.Y.ToString("G5");
            textBox_inertial_origin_z.Text = Link.Inertial.Origin.Z.ToString("G5");
            textBox_inertial_origin_roll.Text = Link.Inertial.Origin.Roll.ToString("G5");
            textBox_inertial_origin_pitch.Text = Link.Inertial.Origin.Pitch.ToString("G5");
            textBox_inertial_origin_yaw.Text = Link.Inertial.Origin.Yaw.ToString("G5");

            textBox_mass.Text = Link.Inertial.Mass.Value.ToString("G5");

            textBox_ixx.Text = Link.Inertial.Inertia.Ixx.ToString("G5");
            textBox_ixy.Text = Link.Inertial.Inertia.Ixy.ToString("G5");
            textBox_ixz.Text = Link.Inertial.Inertia.Ixz.ToString("G5");
            textBox_iyy.Text = Link.Inertial.Inertia.Iyy.ToString("G5");
            textBox_iyz.Text = Link.Inertial.Inertia.Iyz.ToString("G5");
            textBox_izz.Text = Link.Inertial.Inertia.Izz.ToString("G5");

            comboBox_materials.Text = Link.Visual.Material.name;
            textBox_texture.Text = Link.Visual.Material.Texture.wFilename;

            domainUpDown_red.Text = Link.Visual.Material.Color.Red.ToString("G5");
            domainUpDown_green.Text = Link.Visual.Material.Color.Green.ToString("G5");
            domainUpDown_blue.Text = Link.Visual.Material.Color.Blue.ToString("G5");
            domainUpDown_alpha.Text = Link.Visual.Material.Color.Alpha.ToString("G5");

            radioButton_fine.Checked = Link.STLQualityFine;
            radioButton_course.Checked = !Link.STLQualityFine;
        }

        //Fills the property boxes on the joint properties page
        public void fillJointPropertyBoxes(joint Joint)
        {
            AutoUpdatingForm = true;
            if (Joint == null) //For the base_link or if none is selected
            {
                textBox_joint_name.Text = "";
                comboBox_joint_type.Text = "";

                label_parent.Text = "";
                label_child.Text = "";

                textBox_joint_x.Text = "";
                textBox_joint_y.Text = "";
                textBox_joint_z.Text = "";
                textBox_joint_roll.Text = "";
                textBox_joint_pitch.Text = "";
                textBox_joint_yaw.Text = "";

                textBox_axis_x.Text = "";
                textBox_axis_y.Text = "";
                textBox_axis_z.Text = "";

                textBox_limit_lower.Text = "";
                textBox_limit_upper.Text = "";
                textBox_limit_effort.Text = "";
                textBox_limit_velocity.Text = "";

                textBox_calibration_rising.Text = "";
                textBox_calibration_falling.Text = "";

                textBox_friction.Text = "";
                textBox_damping.Text = "";

                textBox_soft_lower.Text = "";
                textBox_soft_upper.Text = "";
                textBox_k_position.Text = "";
                textBox_k_velocity.Text = "";

                label_lower_limit.Text = "lower";
                label_limit_upper.Text = "upper";
                label_effort.Text = "effort";
                label_velocity.Text = "velocity";
                label_friction.Text = "friction";
                label_damping.Text = "damping";
                label_soft_lower.Text = "soft lower limit";
                label_soft_upper.Text = "soft upper limit";
                label_kposition.Text = "k position";
                label_kvelocity.Text = "k velocity";
            }
            else
            {
                textBox_joint_name.Text = Joint.name;
                comboBox_joint_type.Text = Joint.type;

                label_parent.Text = Joint.Parent.name;
                label_child.Text = Joint.Child.name;

                //G5: Maximum decimal places to use (not counting exponential notation) is 5

                
                textBox_joint_x.Text = Joint.Origin.X.ToString("G5");
                textBox_joint_y.Text = Joint.Origin.Y.ToString("G5");
                textBox_joint_z.Text = Joint.Origin.Z.ToString("G5");
                textBox_joint_roll.Text = Joint.Origin.Roll.ToString("G5");
                textBox_joint_pitch.Text = Joint.Origin.Pitch.ToString("G5");
                textBox_joint_yaw.Text = Joint.Origin.Yaw.ToString("G5");

                textBox_axis_x.Text = Joint.Axis.X.ToString("G5");
                textBox_axis_y.Text = Joint.Axis.Y.ToString("G5");
                textBox_axis_z.Text = Joint.Axis.Z.ToString("G5");

                textBox_limit_lower.Text = Joint.Limit.lower.ToString("G5");
                textBox_limit_upper.Text = Joint.Limit.upper.ToString("G5");
                textBox_limit_effort.Text = Joint.Limit.effort.ToString("G5");
                textBox_limit_velocity.Text = Joint.Limit.effort.ToString("G5");

                textBox_calibration_rising.Text = Joint.Calibration.rising.ToString("G5");
                textBox_calibration_falling.Text = Joint.Calibration.falling.ToString("G5");

                textBox_friction.Text = Joint.Dynamics.friction.ToString("G5");
                textBox_damping.Text = Joint.Dynamics.damping.ToString("G5");

                textBox_soft_lower.Text = Joint.Safety.soft_lower.ToString("G5");
                textBox_soft_upper.Text = Joint.Safety.soft_upper.ToString("G5");
                textBox_k_position.Text = Joint.Safety.k_position.ToString("G5");
                textBox_k_velocity.Text = Joint.Safety.k_velocity.ToString("G5");

                if (Joint.type == "revolute" || Joint.type == "continuous")
                {
                    label_lower_limit.Text = "lower (rad)";
                    label_limit_upper.Text = "upper (rad)";
                    label_effort.Text = "effort (N-m)";
                    label_velocity.Text = "velocity (rad/s)";
                    label_friction.Text = "friction (N-m)";
                    label_damping.Text = "damping (N-m-s/rad)";
                    label_soft_lower.Text = "soft lower limit (rad)";
                    label_soft_upper.Text = "soft upper limit (rad)";
                    label_kposition.Text = "k position";
                    label_kvelocity.Text = "k velocity";
                }
                else if (Joint.type == "prismatic")
                {
                    label_lower_limit.Text = "lower (m)";
                    label_limit_upper.Text = "upper (m)";
                    label_effort.Text = "effort (N)";
                    label_velocity.Text = "velocity (m/s)";
                    label_friction.Text = "friction (N)";
                    label_damping.Text = "damping (N-s/m)";
                    label_soft_lower.Text = "soft lower limit (m)";
                    label_soft_upper.Text = "soft upper limit (m)";
                    label_kposition.Text = "k position";
                    label_kvelocity.Text = "k velocity";
                }
                else
                {
                    label_lower_limit.Text = "lower";
                    label_limit_upper.Text = "upper";
                    label_effort.Text = "effort";
                    label_velocity.Text = "velocity";
                    label_friction.Text = "friction";
                    label_damping.Text = "damping";
                    label_soft_lower.Text = "soft lower limit";
                    label_soft_upper.Text = "soft upper limit";
                    label_kposition.Text = "k position";
                    label_kvelocity.Text = "k velocity";
                }
                comboBox_origin.Items.Clear();
                string[] originNames = Exporter.findOrigins();
                comboBox_origin.Items.AddRange(originNames);
                comboBox_axis.Items.Clear();
                string[] axesNames = Exporter.findAxes();
                comboBox_axis.Items.AddRange(axesNames);
                comboBox_origin.SelectedIndex = comboBox_origin.FindStringExact(Joint.CoordinateSystemName);
                if (Joint.AxisName != "")
                {
                    comboBox_axis.SelectedIndex = comboBox_axis.FindStringExact(Joint.AxisName);
                }
                AutoUpdatingForm = false;
            }
        }

        //Converts the text boxes back into values for the link
        public void saveLinkDataFromPropertyBoxes(link Link)
        {
            double value;
            Link.Inertial.Origin.X = (Double.TryParse(textBox_inertial_origin_x.Text, out value)) ? value : 0;
            Link.Inertial.Origin.Y = (Double.TryParse(textBox_inertial_origin_y.Text, out value)) ? value : 0;
            Link.Inertial.Origin.Z = (Double.TryParse(textBox_inertial_origin_z.Text, out value)) ? value : 0;
            Link.Inertial.Origin.Roll = (Double.TryParse(textBox_inertial_origin_roll.Text, out value)) ? value : 0;
            Link.Inertial.Origin.Pitch = (Double.TryParse(textBox_inertial_origin_pitch.Text, out value)) ? value : 0;
            Link.Inertial.Origin.Yaw = (Double.TryParse(textBox_inertial_origin_yaw.Text, out value)) ? value : 0;

            Link.Visual.Origin.X = (Double.TryParse(textBox_visual_origin_x.Text, out value)) ? value : 0;
            Link.Visual.Origin.Y = (Double.TryParse(textBox_visual_origin_y.Text, out value)) ? value : 0;
            Link.Visual.Origin.Z = (Double.TryParse(textBox_visual_origin_z.Text, out value)) ? value : 0;
            Link.Visual.Origin.Roll = (Double.TryParse(textBox_visual_origin_roll.Text, out value)) ? value : 0;
            Link.Visual.Origin.Pitch = (Double.TryParse(textBox_visual_origin_pitch.Text, out value)) ? value : 0;
            Link.Visual.Origin.Yaw = (Double.TryParse(textBox_visual_origin_yaw.Text, out value)) ? value : 0;

            Link.Collision.Origin.X = (Double.TryParse(textBox_collision_origin_x.Text, out value)) ? value : 0;
            Link.Collision.Origin.Y = (Double.TryParse(textBox_collision_origin_y.Text, out value)) ? value : 0;
            Link.Collision.Origin.Z = (Double.TryParse(textBox_collision_origin_z.Text, out value)) ? value : 0;
            Link.Collision.Origin.Roll = (Double.TryParse(textBox_collision_origin_roll.Text, out value)) ? value : 0;
            Link.Collision.Origin.Pitch = (Double.TryParse(textBox_collision_origin_pitch.Text, out value)) ? value : 0;
            Link.Collision.Origin.Yaw = (Double.TryParse(textBox_collision_origin_yaw.Text, out value)) ? value : 0;

            Link.Inertial.Mass.Value = (Double.TryParse(textBox_mass.Text, out value)) ? value : 0;

            Link.Inertial.Inertia.Ixx = (Double.TryParse(textBox_ixx.Text, out value)) ? value : 0;
            Link.Inertial.Inertia.Ixy = (Double.TryParse(textBox_ixy.Text, out value)) ? value : 0;
            Link.Inertial.Inertia.Ixz = (Double.TryParse(textBox_ixz.Text, out value)) ? value : 0;
            Link.Inertial.Inertia.Iyy = (Double.TryParse(textBox_iyy.Text, out value)) ? value : 0;
            Link.Inertial.Inertia.Iyz = (Double.TryParse(textBox_iyz.Text, out value)) ? value : 0;
            Link.Inertial.Inertia.Izz = (Double.TryParse(textBox_izz.Text, out value)) ? value : 0;

            Link.Visual.Material.name = comboBox_materials.Text;
            Link.Visual.Material.Texture.wFilename = textBox_texture.Text;

            Link.Visual.Material.Color.Red = (Double.TryParse(domainUpDown_red.Text, out value)) ? value : 0;
            Link.Visual.Material.Color.Green = (Double.TryParse(domainUpDown_green.Text, out value)) ? value : 0;
            Link.Visual.Material.Color.Blue = (Double.TryParse(domainUpDown_blue.Text, out value)) ? value : 0;
            Link.Visual.Material.Color.Alpha = (Double.TryParse(domainUpDown_alpha.Text, out value)) ? value : 0;

            Link.STLQualityFine = radioButton_fine.Checked;
        }

        //Saves data from text boxes back into a joint
        public void saveJointDataFromPropertyBoxes(joint Joint)
        {
            double value = 0;

            Exporter.mRobot.BaseLink.Inertial.Origin.X = (Double.TryParse(textBox_inertial_origin_x.Text, out value)) ? value : 0;
            Joint.name = textBox_joint_name.Text;
            Joint.type = comboBox_joint_type.Text;
            Joint.Parent.name = label_parent.Text;
            Joint.Child.name = label_child.Text;

            Joint.CoordinateSystemName = comboBox_origin.Text;
            Joint.AxisName = comboBox_axis.Text;

            Joint.Origin.X = (Double.TryParse(textBox_joint_x.Text, out value)) ? value : 0;
            Joint.Origin.Y = (Double.TryParse(textBox_joint_y.Text, out value)) ? value : 0;
            Joint.Origin.Z = (Double.TryParse(textBox_joint_z.Text, out value)) ? value : 0;
            Joint.Origin.Roll = (Double.TryParse(textBox_joint_roll.Text, out value)) ? value : 0;
            Joint.Origin.Pitch = (Double.TryParse(textBox_joint_pitch.Text, out value)) ? value : 0;
            Joint.Origin.Yaw = (Double.TryParse(textBox_joint_yaw.Text, out value)) ? value : 0;

            Joint.Axis.X = (Double.TryParse(textBox_axis_x.Text, out value)) ? value : 0;
            Joint.Axis.Y = (Double.TryParse(textBox_axis_y.Text, out value)) ? value : 0;
            Joint.Axis.Z = (Double.TryParse(textBox_axis_z.Text, out value)) ? value : 0;

            Joint.Limit.lower = (Double.TryParse(textBox_limit_lower.Text, out value)) ? value : 0;
            Joint.Limit.upper = (Double.TryParse(textBox_limit_upper.Text, out value)) ? value : 0;
            Joint.Limit.effort = (Double.TryParse(textBox_limit_effort.Text, out value)) ? value : 0;
            Joint.Limit.velocity = (Double.TryParse(textBox_limit_velocity.Text, out value)) ? value : 0;

            Joint.Calibration.rising = (Double.TryParse(textBox_calibration_rising.Text, out value)) ? value : 0;
            Joint.Calibration.falling = (Double.TryParse(textBox_calibration_falling.Text, out value)) ? value : 0;

            Joint.Dynamics.friction = (Double.TryParse(textBox_friction.Text, out value)) ? value : 0;
            Joint.Dynamics.damping = (Double.TryParse(textBox_damping.Text, out value)) ? value : 0;

            Joint.Safety.soft_lower = (Double.TryParse(textBox_soft_lower.Text, out value)) ? value : 0;
            Joint.Safety.soft_upper = (Double.TryParse(textBox_soft_upper.Text, out value)) ? value : 0;
            Joint.Safety.k_position = (Double.TryParse(textBox_k_position.Text, out value)) ? value : 0;
            Joint.Safety.k_velocity = (Double.TryParse(textBox_k_velocity.Text, out value)) ? value : 0;
        }

        //Fills either TreeView from the URDF robot
        public void fillTreeViewFromRobot(robot Robot, TreeView tree)
        {
            
            LinkNode baseNode = new LinkNode();
            link baseLink = Robot.BaseLink;
            baseNode.Name = baseLink.name;
            baseNode.Text = baseLink.name;
            baseNode.Link = baseLink;
            baseNode.isBaseNode = true;
            baseNode.linkName = baseLink.name;
            baseNode.Components = baseLink.SWcomponents;
            baseNode.coordsysName = "Origin_global";
            baseNode.isIncomplete = false;
            
            foreach (link child in baseLink.Children)
            {
                baseNode.Nodes.Add(createLinkNodeFromLink(child));
            }
            tree.Nodes.Add(baseNode);
            tree.ExpandAll();
        }

        //Fills specifically the joint TreeView. This is used by the Property Manager Page because it does not have access to the tree directly
        public void fillJointTree()
        {
            treeView_jointtree.Nodes.Clear();
            foreach(LinkNode node in BaseNode.Nodes)
            {
                BaseNode.Nodes.Remove(node);
                treeView_jointtree.Nodes.Add(node);
                updateNodeText(node, true);
            }
            treeView_jointtree.ExpandAll();
        }

        public void fillLinkTree()
        {
            treeView_linkProperties.Nodes.Clear();
            treeView_linkProperties.Nodes.Add(BaseNode);
            updateNodeText(BaseNode, false);
            treeView_linkProperties.ExpandAll();
        }

        public void updateNodeText(LinkNode node, bool useJointName)
        {
            if (useJointName)
            {
                node.Text = node.Link.Joint.name;
            }
            else
            {
                node.Text = node.Link.name;
            }
            foreach(LinkNode child in node.Nodes)
            {
                updateNodeText(child, useJointName);
            }
        }

        //Converts a Link to a LinkNode
        public LinkNode createLinkNodeFromLink(link Link)
        {
            LinkNode node = new LinkNode();
            node.Name = Link.name;
            node.Text = Link.name;
            node.Link = Link;
            node.isBaseNode = false;
            node.linkName = Link.name;
            node.jointName = Link.Joint.name;
            node.Components = Link.SWcomponents;
            node.coordsysName = Link.Joint.CoordinateSystemName;
            node.axisName = Link.Joint.AxisName;
            node.jointType = Link.Joint.type;
            node.isIncomplete = false;

            foreach (link child in Link.Children)
            {
                node.Nodes.Add(createLinkNodeFromLink(child));
            }
            node.Link.Children.Clear(); // Need to erase the children from the embedded link because they may be rearranged later.


            return node;
        }

        //Converts a TreeView back into a robot
        public robot createRobotFromTreeView(TreeView tree)
        {
            //TODO: This needs to properly handle the new differences between the trees.
            robot Robot = Exporter.mRobot;
            Robot.BaseLink = createLinkFromLinkNode((LinkNode)tree.Nodes[0]);
            Robot.name = Exporter.mRobot.name;
            return Robot;
        }

        //Converts a LinkNode into a Link
        public link createLinkFromLinkNode(LinkNode node)
        {
            link Link = node.Link;
            Link.Children.Clear();
            foreach (LinkNode child in node.Nodes)
            {

                link childLink = createLinkFromLinkNode(child);
                Link.Children.Add(childLink); // Recreates the children of each embedded link

            }
            return Link;
        }


        #endregion

        #region Link Properties Controls Handlers

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
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            openFileDialog1.RestoreDirectory = true;
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {
                textBox_texture.Text = openFileDialog1.FileName;
            }
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

        #endregion

        #region Joint Properties Controls Handlers
        private void label_damping_Click(object sender, EventArgs e)
        {

        }

        public void changeAllNodeFont(LinkNode node, Font font)
        {
            node.NodeFont = font;
            foreach (LinkNode child in node.Nodes)
            {
                changeAllNodeFont(child, font);
            }
        }

        private void treeView_jointtree_AfterSelect(object sender, TreeViewEventArgs e)
        {
            Font fontRegular = new Font(treeView_jointtree.Font, FontStyle.Regular);
            Font fontBold = new Font(treeView_jointtree.Font, FontStyle.Bold);
            if (previouslySelectedNode != null && !previouslySelectedNode.isBaseNode)
            {
                saveJointDataFromPropertyBoxes(previouslySelectedNode.Link.Joint);
            }
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.NodeFont = fontRegular;
            }
            LinkNode node = (LinkNode)e.Node;
            ActiveSWModel.ClearSelection2(true);
            SelectionMgr manager = ActiveSWModel.SelectionManager;

            SelectData data = manager.CreateSelectData();
            data.Mark = -1;
            if (node.Link.SWComponent != null)
            {
                node.Link.SWComponent.Select4(false, data, false);
            }
            else
            {
                foreach (Component2 component in node.Link.SWcomponents)
                {
                    component.Select4(true, data, false);
                }
            }
            node.NodeFont = fontBold;
            node.Text = node.Text;
            fillJointPropertyBoxes(node.Link.Joint);
            previouslySelectedNode = node;
        }

        private void comboBox_axis_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (!AutoUpdatingForm)
            {
                if (!(comboBox_origin.Text == "" || comboBox_axis.Text == ""))
                {
                    double[] Axis = Exporter.estimateAxis(comboBox_axis.Text);
                    Exporter.localizeAxis(Axis, comboBox_origin.Text);
                    textBox_axis_x.Text = Axis[0].ToString("G5");
                    textBox_axis_y.Text = Axis[1].ToString("G5");
                    textBox_axis_z.Text = Axis[2].ToString("G5");
                }
            }
        }
        #endregion
    }

}