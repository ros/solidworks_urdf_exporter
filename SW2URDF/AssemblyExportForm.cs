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

            while (treeView_jointtree.Nodes.Count > 0)
            {
                LinkNode node = (LinkNode)treeView_jointtree.Nodes[0];
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
            while (treeView_jointtree.Nodes.Count > 0)
            {
                LinkNode node = (LinkNode)treeView_jointtree.Nodes[0];
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
            Link.Collision.Origin.fillBoxes(textBox_collision_origin_x, 
                                            textBox_collision_origin_y, 
                                            textBox_collision_origin_z, 
                                            textBox_collision_origin_roll, 
                                            textBox_collision_origin_pitch, 
                                            textBox_collision_origin_yaw, 
                                            "G5" );

            Link.Visual.Origin.fillBoxes(textBox_visual_origin_x, 
                                         textBox_visual_origin_y, 
                                         textBox_visual_origin_z, 
                                         textBox_visual_origin_roll, 
                                         textBox_visual_origin_pitch, 
                                         textBox_visual_origin_yaw, 
                                         "G5" );

            Link.Inertial.Origin.fillBoxes(textBox_inertial_origin_x, 
                                           textBox_inertial_origin_y, 
                                           textBox_inertial_origin_z, 
                                           textBox_inertial_origin_roll, 
                                           textBox_inertial_origin_pitch, 
                                           textBox_inertial_origin_yaw,
                                           "G5" );

            Link.Inertial.Mass.fillBoxes(textBox_mass, "G5");

            Link.Inertial.Inertia.fillBoxes(textBox_ixx, 
                                            textBox_ixy, 
                                            textBox_ixz, 
                                            textBox_iyy, 
                                            textBox_iyz, 
                                            textBox_izz, 
                                            "G5" );

            Link.Visual.Material.fillBoxes(comboBox_materials, "G5");
            textBox_texture.Text = Link.Visual.Material.Texture.wFilename;

            Link.Visual.Material.Color.fillBoxes(domainUpDown_red, 
                                                 domainUpDown_green, 
                                                 domainUpDown_blue, 
                                                 domainUpDown_alpha, 
                                                 "G5");

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
                Joint.fillBoxes(textBox_joint_name, comboBox_joint_type);
                Joint.Parent.fillBoxes(label_parent);
                Joint.Child.fillBoxes(label_child);

                //G5: Maximum decimal places to use (not counting exponential notation) is 5

                Joint.Origin.fillBoxes(textBox_joint_x, 
                                       textBox_joint_y, 
                                       textBox_joint_z, 
                                       textBox_joint_roll, 
                                       textBox_joint_pitch, 
                                       textBox_joint_yaw, 
                                       "G5");

                Joint.Axis.fillBoxes(textBox_axis_x, textBox_axis_y, textBox_axis_z, "G5");

                if (Joint.Limit != null)
                {
                    Joint.Limit.fillBoxes(textBox_limit_lower, 
                                          textBox_limit_upper, 
                                          textBox_limit_effort, 
                                          textBox_limit_velocity, 
                                          "G5");
                }

                if (Joint.Calibration != null)
                {
                    Joint.Calibration.fillBoxes(textBox_calibration_rising, 
                                                textBox_calibration_falling, 
                                                "G5");
                }

                if (Joint.Dynamics != null)
                {
                    Joint.Dynamics.fillBoxes(textBox_damping, 
                                             textBox_friction, 
                                             "G5");
                }

                if (Joint.Safety != null)
                {
                    Joint.Safety.fillBoxes(textBox_soft_lower, 
                                           textBox_soft_upper, 
                                           textBox_k_position, 
                                           textBox_k_velocity, 
                                           "G5");
                }

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
            Link.Inertial.Origin.update(textBox_inertial_origin_x, 
                                        textBox_inertial_origin_y, 
                                        textBox_inertial_origin_z, 
                                        textBox_inertial_origin_roll, 
                                        textBox_inertial_origin_pitch, 
                                        textBox_inertial_origin_yaw);

            Link.Visual.Origin.update(textBox_visual_origin_x, 
                                      textBox_visual_origin_y, 
                                      textBox_visual_origin_z, 
                                      textBox_visual_origin_roll, 
                                      textBox_visual_origin_pitch, 
                                      textBox_visual_origin_yaw);

            Link.Collision.Origin.update(textBox_collision_origin_x, 
                                         textBox_collision_origin_y, 
                                         textBox_collision_origin_z, 
                                         textBox_collision_origin_roll, 
                                         textBox_collision_origin_pitch, 
                                         textBox_collision_origin_yaw);

            Link.Inertial.Mass.update(textBox_mass);

            Link.Inertial.Inertia.update(textBox_ixx, 
                                         textBox_ixy, 
                                         textBox_ixz, 
                                         textBox_iyy, 
                                         textBox_iyz, 
                                         textBox_izz);

            Link.Visual.Material.name = comboBox_materials.Text;
            Link.Visual.Material.Texture.wFilename = textBox_texture.Text;

            Link.Visual.Material.Color.update(domainUpDown_red, 
                                              domainUpDown_green, 
                                              domainUpDown_blue, 
                                              domainUpDown_alpha);
 
            Link.STLQualityFine = radioButton_fine.Checked;
        }

        //Saves data from text boxes back into a joint
        public void saveJointDataFromPropertyBoxes(joint Joint)
        {
            Joint.update(textBox_joint_name, comboBox_joint_type);

            Joint.Parent.update(label_parent);
            Joint.Child.update(label_child);

            Joint.CoordinateSystemName = comboBox_origin.Text;
            Joint.AxisName = comboBox_axis.Text;

            Joint.Origin.update(textBox_joint_x, 
                                textBox_joint_y, 
                                textBox_joint_z, 
                                textBox_joint_roll, 
                                textBox_joint_pitch, 
                                textBox_joint_yaw);

            Joint.Axis.update(textBox_axis_x, 
                              textBox_axis_y, 
                              textBox_axis_z);

            if (textBox_limit_lower.Text == "" && textBox_limit_upper.Text == "" && textBox_limit_effort.Text == "" && textBox_limit_lower.Text == "")
            {
                if (Joint.type == "prismatic" || Joint.type == "revolute")
                {
                    if (Joint.Limit == null)
                    {
                        Joint.Limit = new limit();
                    }
                    else
                    {
                        Joint.Limit.effort = 0;
                        Joint.Limit.velocity = 0;
                    }
                }
                else
                {
                    Joint.Limit = null;
                }
            }
            else
            {
                if (Joint.Limit == null)
                {
                    Joint.Limit = new limit();
                }
                Joint.Limit.update(textBox_limit_lower, 
                                   textBox_limit_upper, 
                                   textBox_limit_effort, 
                                   textBox_limit_velocity);
            }

            if (textBox_calibration_rising.Text == "" && textBox_calibration_falling.Text == "")
            {
                Joint.Calibration = null;
            }
            else
            {
                if (Joint.Calibration == null)
                {
                    Joint.Calibration = new calibration();
                }
                Joint.Calibration.update(textBox_calibration_rising, 
                                         textBox_calibration_falling);
            }

            if (textBox_friction.Text == "" && textBox_damping.Text == "")
            {
                Joint.Dynamics = null;
            }
            else
            {
                if (Joint.Dynamics == null)
                {
                    Joint.Dynamics = new dynamics();
                }
                Joint.Dynamics.update(textBox_damping, 
                                      textBox_friction);
            }

            if (textBox_soft_lower.Text == "" && textBox_soft_upper.Text == "" && textBox_k_position.Text == "" && textBox_k_velocity.Text == "")
            {
                Joint.Safety = null;
            }
            else
            {
                if (Joint.Safety == null)
                {
                    Joint.Safety = new safety_controller();
                }
                Joint.Safety.update(textBox_soft_lower, 
                                    textBox_soft_upper, 
                                    textBox_k_position, 
                                    textBox_k_velocity);

            }
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

        //Fills specifically the joint TreeView
        public void fillJointTree()
        {
            treeView_jointtree.Nodes.Clear();

            while (BaseNode.Nodes.Count > 0)
            {
                LinkNode node = (LinkNode)BaseNode.FirstNode;
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