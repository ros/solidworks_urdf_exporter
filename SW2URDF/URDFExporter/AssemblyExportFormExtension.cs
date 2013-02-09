using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using System.Xml.Serialization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace SW2URDF
{
    //This source file contains all the non-handler methods for the assembly export form, the ones that are helpers.
    public partial class AssemblyExportForm : Form
    {
                //From the link, this method fills the property boxes on the Link Properties page
        public void fillLinkPropertyBoxes(link Link)
        {
            fillBlank(linkBoxes);
            if (!Link.isFixedFrame)
            {
                Link.Collision.Origin.fillBoxes(textBox_collision_origin_x,
                                                textBox_collision_origin_y,
                                                textBox_collision_origin_z,
                                                textBox_collision_origin_roll,
                                                textBox_collision_origin_pitch,
                                                textBox_collision_origin_yaw,
                                                "G5");

                Link.Visual.Origin.fillBoxes(textBox_visual_origin_x,
                                             textBox_visual_origin_y,
                                             textBox_visual_origin_z,
                                             textBox_visual_origin_roll,
                                             textBox_visual_origin_pitch,
                                             textBox_visual_origin_yaw,
                                             "G5");

                Link.Inertial.Origin.fillBoxes(textBox_inertial_origin_x,
                                               textBox_inertial_origin_y,
                                               textBox_inertial_origin_z,
                                               textBox_inertial_origin_roll,
                                               textBox_inertial_origin_pitch,
                                               textBox_inertial_origin_yaw,
                                               "G5");

                Link.Inertial.Mass.fillBoxes(textBox_mass, "G5");

                Link.Inertial.Inertia.fillBoxes(textBox_ixx,
                                                textBox_ixy,
                                                textBox_ixz,
                                                textBox_iyy,
                                                textBox_iyz,
                                                textBox_izz,
                                                "G5");

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
        }

        //Fills the property boxes on the joint properties page
        public void fillJointPropertyBoxes(joint Joint)
        {
            fillBlank(jointBoxes);
            AutoUpdatingForm = true;
            if (Joint != null) //For the base_link or if none is selected
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
            }

            if (Joint != null && (Joint.type == "revolute" || Joint.type == "continuous"))
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
            else if (Joint != null && Joint.type == "prismatic")
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

        public void fillBlank(Control[] boxes)
        {
            foreach (Control box in boxes)
            {
                box.Text = "";
            }
        }

        //Converts the text boxes back into values for the link
        public void saveLinkDataFromPropertyBoxes(link Link)
        {
            if (!Link.isFixedFrame)
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

            if (textBox_limit_lower.Text == "" && textBox_limit_upper.Text == "" && textBox_limit_effort.Text == "" && textBox_limit_velocity.Text == "")
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
                Joint.Limit.setValues(textBox_limit_lower, 
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
                Joint.Calibration.setValues(textBox_calibration_rising, 
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
                Joint.Dynamics.setValues(textBox_damping, 
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
                Joint.Safety.setValues(textBox_soft_lower, 
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


        public void saveConfigTree(ModelDoc2 model, LinkNode BaseNode, bool warnUser)
        {
            Object[] objects = model.FeatureManager.GetFeatures(true);
            string oldData = "";
            Parameter param;
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        param = att.GetParameter("data");
                        oldData = param.GetStringValue();
                    }
                }
            }


            //moveComponentsToFolder((LinkNode)tree.Nodes[0]);
            Common.retrieveSWComponentPIDs(model, BaseNode);
            SerialNode sNode = new SerialNode(BaseNode);

            StringWriter stringWriter;
            XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
            stringWriter = new StringWriter();
            serializer.Serialize(stringWriter, sNode);
            stringWriter.Flush();
            stringWriter.Close();

            string newData = stringWriter.ToString();
            if (oldData != newData)
            {
                if (!warnUser || (warnUser && MessageBox.Show("The configuration has changed, would you like to save?", "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes))
                {
                    int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = createSWSaveAttribute(swApp, "URDF Export Configuration");
                    param = saveExporterAttribute.GetParameter("data");
                    param.SetStringValue2(stringWriter.ToString(), ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("name");
                    param.SetStringValue2("config1", ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("date");
                    param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("exporterVersion");
                    param.SetStringValue2("1.1", ConfigurationOptions, "");
                }
            }
        }

        private SolidWorks.Interop.sldworks.Attribute createSWSaveAttribute(ISldWorks iSwApp, string name)
        {
            int Options = 0;
            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            ModelDoc2 ActiveSWModel = iSwApp.ActiveDoc;
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == name)
                    {
                        return att;
                    }
                }
            }

            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = saveConfigurationAttributeDef.CreateInstance5(ActiveSWModel, null, "URDF Export Configuration", Options, ConfigurationOptions);
            return saveExporterAttribute;
        }

        public void changeAllNodeFont(LinkNode node, Font font)
        {
            node.NodeFont = font;
            foreach (LinkNode child in node.Nodes)
            {
                changeAllNodeFont(child, font);
            }
        }
    }
}
