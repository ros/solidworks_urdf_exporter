/*
Copyright (c) 2015 Stephen Brawner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
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
        public void FillLinkPropertyBoxes(Link Link)
        {
            FillBlank(linkBoxes);
            if (!Link.isFixedFrame)
            {
                Link.Visual.Origin.FillBoxes(textBox_visual_origin_x,
                                             textBox_visual_origin_y,
                                             textBox_visual_origin_z,
                                             textBox_visual_origin_roll,
                                             textBox_visual_origin_pitch,
                                             textBox_visual_origin_yaw,
                                             "G5");

                Link.Inertial.Origin.FillBoxes(textBox_inertial_origin_x,
                                               textBox_inertial_origin_y,
                                               textBox_inertial_origin_z,
                                               textBox_inertial_origin_roll,
                                               textBox_inertial_origin_pitch,
                                               textBox_inertial_origin_yaw,
                                               "G5");

                Link.Inertial.Mass.FillBoxes(textBox_mass, "G5");

                Link.Inertial.Inertia.FillBoxes(textBox_ixx,
                                                textBox_ixy,
                                                textBox_ixz,
                                                textBox_iyy,
                                                textBox_iyz,
                                                textBox_izz,
                                                "G5");

                Link.Visual.Material.FillBoxes(comboBox_materials, "G5");
                textBox_texture.Text = Link.Visual.Material.Texture.wFilename;

                Link.Visual.Material.Color.FillBoxes(domainUpDown_red,
                                                     domainUpDown_green,
                                                     domainUpDown_blue,
                                                     domainUpDown_alpha,
                                                     "G5");

                radioButton_fine.Checked = Link.STLQualityFine;
                radioButton_course.Checked = !Link.STLQualityFine;
            }
        }

        //Fills the property boxes on the joint properties page
        public void FillJointPropertyBoxes(Joint Joint)
        {
            FillBlank(jointBoxes);
            AutoUpdatingForm = true;
            if (Joint != null) //For the base_link or if none is selected
            {
                Joint.FillBoxes(textBox_joint_name, comboBox_joint_type);
                Joint.Parent.FillBoxes(label_parent);
                Joint.Child.FillBoxes(label_child);

                //G5: Maximum decimal places to use (not counting exponential notation) is 5

                Joint.Origin.FillBoxes(textBox_joint_x,
                                       textBox_joint_y,
                                       textBox_joint_z,
                                       textBox_joint_roll,
                                       textBox_joint_pitch,
                                       textBox_joint_yaw,
                                       "G5");

                Joint.Axis.FillBoxes(textBox_axis_x, textBox_axis_y, textBox_axis_z, "G5");

                if (Joint.Limit != null)
                {
                    Joint.Limit.FillBoxes(textBox_limit_lower,
                                          textBox_limit_upper,
                                          textBox_limit_effort,
                                          textBox_limit_velocity,
                                          "G5");
                }

                if (Joint.Calibration != null)
                {
                    Joint.Calibration.FillBoxes(textBox_calibration_rising,
                                                textBox_calibration_falling,
                                                "G5");
                }

                if (Joint.Dynamics != null)
                {
                    Joint.Dynamics.FillBoxes(textBox_damping,
                                             textBox_friction,
                                             "G5");
                }

                if (Joint.Safety != null)
                {
                    Joint.Safety.FillBoxes(textBox_soft_lower,
                                           textBox_soft_upper,
                                           textBox_k_position,
                                           textBox_k_velocity,
                                           "G5");
                }
            }

            if (Joint != null && (Joint.Type == "revolute" || Joint.Type == "continuous"))
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
            else if (Joint != null && Joint.Type == "prismatic")
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
            List<string> originNames = Exporter.FindRefGeoNames("CoordSys");
            comboBox_origin.Items.AddRange(originNames.ToArray());
            comboBox_axis.Items.Clear();
            List<string> axesNames = Exporter.FindRefGeoNames("RefAxis");
            comboBox_axis.Items.AddRange(axesNames.ToArray());
            comboBox_origin.SelectedIndex = comboBox_origin.FindStringExact(Joint.CoordinateSystemName);
            if (String.IsNullOrWhiteSpace(Joint.AxisName))
            {
                comboBox_axis.SelectedIndex = comboBox_axis.FindStringExact(Joint.AxisName);
            }
            AutoUpdatingForm = false;
        }

        public void FillBlank(Control[] boxes)
        {
            foreach (Control box in boxes)
            {
                box.Text = "";
            }
        }

        //Converts the text boxes back into values for the link
        public void SaveLinkDataFromPropertyBoxes(Link Link)
        {
            if (!Link.isFixedFrame)
            {
                Link.Inertial.Origin.Update(textBox_inertial_origin_x,
                                            textBox_inertial_origin_y,
                                            textBox_inertial_origin_z,
                                            textBox_inertial_origin_roll,
                                            textBox_inertial_origin_pitch,
                                            textBox_inertial_origin_yaw);

                Link.Visual.Origin.Update(textBox_visual_origin_x,
                                          textBox_visual_origin_y,
                                          textBox_visual_origin_z,
                                          textBox_visual_origin_roll,
                                          textBox_visual_origin_pitch,
                                          textBox_visual_origin_yaw);

                Link.Inertial.Mass.Update(textBox_mass);

                Link.Inertial.Inertia.Update(textBox_ixx,
                                             textBox_ixy,
                                             textBox_ixz,
                                             textBox_iyy,
                                             textBox_iyz,
                                             textBox_izz);

                Link.Visual.Material.Name = comboBox_materials.Text;
                Link.Visual.Material.Texture.wFilename = textBox_texture.Text;

                Link.Visual.Material.Color.Update(domainUpDown_red,
                                                  domainUpDown_green,
                                                  domainUpDown_blue,
                                                  domainUpDown_alpha);

                Link.STLQualityFine = radioButton_fine.Checked;
            }
        }

        //Saves data from text boxes back into a joint
        public void SaveJointDataFromPropertyBoxes(Joint Joint)
        {
            Joint.Update(textBox_joint_name, comboBox_joint_type);

            Joint.Parent.Update(label_parent);
            Joint.Child.Update(label_child);

            Joint.CoordinateSystemName = comboBox_origin.Text;
            Joint.AxisName = comboBox_axis.Text;

            Joint.Origin.Update(textBox_joint_x,
                                textBox_joint_y,
                                textBox_joint_z,
                                textBox_joint_roll,
                                textBox_joint_pitch,
                                textBox_joint_yaw);

            Joint.Axis.Update(textBox_axis_x,
                              textBox_axis_y,
                              textBox_axis_z);

            if (String.IsNullOrWhiteSpace(textBox_limit_lower.Text) && String.IsNullOrWhiteSpace(textBox_limit_upper.Text) && String.IsNullOrWhiteSpace(textBox_limit_effort.Text) && String.IsNullOrWhiteSpace(textBox_limit_velocity.Text))
            {
                if (Joint.Type == "prismatic" || Joint.Type == "revolute")
                {
                    if (Joint.Limit == null)
                    {
                        Joint.Limit = new Limit();
                    }
                    else
                    {
                        Joint.Limit.Effort = 0;
                        Joint.Limit.Velocity = 0;
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
                    Joint.Limit = new Limit();
                }
                Joint.Limit.SetValues(textBox_limit_lower,
                                   textBox_limit_upper,
                                   textBox_limit_effort,
                                   textBox_limit_velocity);
            }

            if (String.IsNullOrWhiteSpace(textBox_calibration_rising.Text) && String.IsNullOrWhiteSpace(textBox_calibration_falling.Text))
            {
                Joint.Calibration = null;
            }
            else
            {
                if (Joint.Calibration == null)
                {
                    Joint.Calibration = new Calibration();
                }
                Joint.Calibration.SetValues(textBox_calibration_rising,
                                         textBox_calibration_falling);
            }

            if (String.IsNullOrWhiteSpace(textBox_friction.Text) && String.IsNullOrWhiteSpace(textBox_damping.Text))
            {
                Joint.Dynamics = null;
            }
            else
            {
                if (Joint.Dynamics == null)
                {
                    Joint.Dynamics = new Dynamics();
                }
                Joint.Dynamics.SetValues(textBox_damping,
                                      textBox_friction);
            }

            if (String.IsNullOrWhiteSpace(textBox_soft_lower.Text) && String.IsNullOrWhiteSpace(textBox_soft_upper.Text) && String.IsNullOrWhiteSpace(textBox_k_position.Text) && String.IsNullOrWhiteSpace(textBox_k_velocity.Text))
            {
                Joint.Safety = null;
            }
            else
            {
                if (Joint.Safety == null)
                {
                    Joint.Safety = new SafetyController();
                }
                Joint.Safety.SetValues(textBox_soft_lower,
                                    textBox_soft_upper,
                                    textBox_k_position,
                                    textBox_k_velocity);
            }
        }

        //Fills either TreeView from the URDF robot
        public void FillTreeViewFromRobot(Robot Robot, TreeView tree)
        {
            LinkNode baseNode = new LinkNode();
            Link baseLink = Robot.BaseLink;
            baseNode.Name = baseLink.Name;
            baseNode.Text = baseLink.Name;
            baseNode.Link = baseLink;
            baseNode.IsBaseNode = true;
            baseNode.LinkName = baseLink.Name;
            baseNode.Components = baseLink.SWcomponents;
            baseNode.CoordsysName = "Origin_global";
            baseNode.IsIncomplete = false;

            foreach (Link child in baseLink.Children)
            {
                baseNode.Nodes.Add(CreateLinkNodeFromLink(child));
            }
            tree.Nodes.Add(baseNode);
            tree.ExpandAll();
        }

        //Fills specifically the joint TreeView
        public void FillJointTree()
        {
            treeView_jointtree.Nodes.Clear();

            while (BaseNode.Nodes.Count > 0)
            {
                LinkNode node = (LinkNode)BaseNode.FirstNode;
                BaseNode.Nodes.Remove(node);
                treeView_jointtree.Nodes.Add(node);
                UpdateNodeText(node, true);
            }
            treeView_jointtree.ExpandAll();
        }

        public void FillLinkTree()
        {
            treeView_linkProperties.Nodes.Clear();
            treeView_linkProperties.Nodes.Add(BaseNode);
            UpdateNodeText(BaseNode, false);
            treeView_linkProperties.ExpandAll();
        }

        public void UpdateNodeText(LinkNode node, bool useJointName)
        {
            if (useJointName)
            {
                node.Text = node.Link.Joint.Name;
            }
            else
            {
                node.Text = node.Link.Name;
            }
            foreach (LinkNode child in node.Nodes)
            {
                UpdateNodeText(child, useJointName);
            }
        }

        //Converts a Link to a LinkNode
        public LinkNode CreateLinkNodeFromLink(Link Link)
        {
            LinkNode node = new LinkNode
            {
                Name = Link.Name,
                Text = Link.Name,
                Link = Link,
                IsBaseNode = false,
                LinkName = Link.Name,
                JointName = Link.Joint.Name,
                Components = Link.SWcomponents,
                CoordsysName = Link.Joint.CoordinateSystemName,
                AxisName = Link.Joint.AxisName,
                JointType = Link.Joint.Type,
                IsIncomplete = false
            };

            foreach (Link child in Link.Children)
            {
                node.Nodes.Add(CreateLinkNodeFromLink(child));
            }
            node.Link.Children.Clear(); // Need to erase the children from the embedded link because they may be rearranged later.

            return node;
        }

        //Converts a TreeView back into a robot
        public Robot CreateRobotFromTreeView(TreeView tree)
        {
            //TODO: This needs to properly handle the new differences between the trees.
            Robot Robot = Exporter.URDFRobot;
            Robot.BaseLink = CreateLinkFromLinkNode((LinkNode)tree.Nodes[0]);
            Robot.Name = Exporter.URDFRobot.Name;
            return Robot;
        }

        //Converts a LinkNode into a Link
        public Link CreateLinkFromLinkNode(LinkNode node)
        {
            Link Link = node.Link;
            Link.Children.Clear();
            foreach (LinkNode child in node.Nodes)
            {
                Link childLink = CreateLinkFromLinkNode(child);
                Link.Children.Add(childLink); // Recreates the children of each embedded link
            }
            return Link;
        }

        public void SaveConfigTree(ModelDoc2 model, LinkNode BaseNode, bool warnUser)
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
            Common.RetrieveSWComponentPIDs(model, BaseNode);
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
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = CreateSWSaveAttribute(swApp, "URDF Export Configuration");
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

        private SolidWorks.Interop.sldworks.Attribute CreateSWSaveAttribute(ISldWorks iSwApp, string name)
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

        public void ChangeAllNodeFont(LinkNode node, Font font)
        {
            node.NodeFont = font;
            foreach (LinkNode child in node.Nodes)
            {
                ChangeAllNodeFont(child, font);
            }
        }
    }
}