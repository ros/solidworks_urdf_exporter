using System;
using System.Drawing;
using System.Text;
using System.IO;
using System.Windows.Forms;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System.Xml.Serialization;

namespace SW2URDF
{

    public partial class AssemblyExportForm : Form
    {
        ISldWorks swApp;
        ModelDoc2 ActiveSWModel;
        private StringBuilder NewNodeMap = new StringBuilder(128);
        public URDFExporter Exporter;
        LinkNode previouslySelectedNode;
        public bool AutoUpdatingForm;
        Control[] jointBoxes;
        Control[] linkBoxes;
        LinkNode BaseNode;
        public AttributeDef saveConfigurationAttributeDef;

        public AssemblyExportForm(ISldWorks iSwApp, LinkNode node)
        {
            InitializeComponent();
            swApp = iSwApp;
            BaseNode = node;
            ActiveSWModel = swApp.ActiveDoc;
            Exporter = new URDFExporter(iSwApp);
            AutoUpdatingForm = false;


            jointBoxes = new Control[] {
                textBox_joint_name, comboBox_axis, comboBox_joint_type,
                textBox_axis_x, textBox_axis_y, textBox_axis_z,
                textBox_joint_x, textBox_joint_y, textBox_joint_z, textBox_joint_pitch, textBox_joint_roll, textBox_joint_yaw,
                textBox_limit_lower, textBox_limit_upper, textBox_limit_effort, textBox_limit_velocity,
                textBox_damping, textBox_friction,
                textBox_calibration_falling, textBox_calibration_rising,
                textBox_soft_lower, textBox_soft_upper, textBox_k_position, textBox_k_velocity
            };
            linkBoxes = new Control[] {
                textBox_inertial_origin_x, textBox_inertial_origin_y, textBox_inertial_origin_z, textBox_inertial_origin_roll, textBox_inertial_origin_pitch, textBox_inertial_origin_yaw,
                textBox_visual_origin_x, textBox_visual_origin_y, textBox_visual_origin_z, textBox_visual_origin_roll, textBox_visual_origin_pitch, textBox_visual_origin_yaw,
                textBox_collision_origin_x, textBox_collision_origin_y, textBox_collision_origin_z, textBox_collision_origin_roll, textBox_collision_origin_pitch, textBox_collision_origin_yaw,
                textBox_ixx, textBox_ixy, textBox_ixz, textBox_iyy, textBox_iyz, textBox_izz,
                textBox_mass, 
                domainUpDown_red, domainUpDown_green, domainUpDown_blue, domainUpDown_alpha,
                comboBox_materials,
                textBox_texture
            };

            saveConfigurationAttributeDef = iSwApp.DefineAttribute("URDF Export Configuration");
            int Options = 0;

            saveConfigurationAttributeDef.AddParameter("data", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("name", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("date", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("exporterVersion", (int)swParamType_e.swParamTypeDouble, 1.0, Options);
            saveConfigurationAttributeDef.Register();
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
            saveConfigTree(ActiveSWModel, BaseNode, true);
            this.Close();
        }

        private void button_links_cancel_Click(object sender, EventArgs e)
        {
            if (previouslySelectedNode != null)
            {
                saveLinkDataFromPropertyBoxes(previouslySelectedNode.Link);
            }
            saveConfigTree(ActiveSWModel, BaseNode, true);
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
            saveConfigTree(ActiveSWModel, BaseNode, false);
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