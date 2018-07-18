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
using System.Xml;
using System.Xml.Serialization;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using System.Runtime.Serialization.Formatters.Soap;
using System.Text;

namespace SW2URDF
{
    public partial class URDFExporterPM : PropertyManagerPage2Handler9
    {
        public static readonly string CONFIGURATION_VERSION = "1.3";
        public static readonly double SOAP_MIN_VERSION = 1.3;
        public AttributeDef saveConfigurationAttributeDef;

        private bool AskUserConfigurationSave(bool warnUser, string newData, string oldData, string previousVersion)
        {
            bool success = (oldData != newData);
            if (oldData != newData)
            {
                if (previousVersion != CONFIGURATION_VERSION)
                {
                    if (MessageBox.Show("The configuration has changed, would you like to save and " +
                    "update the configuration to the latest version?",
                    "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes)
                    {
                        success = true;
                    }
                }
                else if (warnUser)
                {
                    if (MessageBox.Show("The configuration has changed, would you like to save?",
                    "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes)
                    {
                        success = true;
                    }
                }
                else
                {
                    success = true;
                }
            }
            return success;
        }

        private void SaveConfigTreeSoap(ModelDoc2 model, LinkNode BaseNode, bool warnUser)
        {
            Object[] objects = model.FeatureManager.GetFeatures(true);
            string oldData = "";
            string previousVersion = "";
            Parameter param;
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        param = att.GetParameter("data");
                        oldData = param.GetStringValue();
                        param = att.GetParameter("exporterVersion");
                        previousVersion = param.GetStringValue();
                    }
                }
            }
            //moveComponentsToFolder((LinkNode)tree.Nodes[0]);
            RetrieveSWComponentPIDs(BaseNode);

            StringWriter stringWriter = new StringWriter();
            String newData = "";
            using (MemoryStream stream = new MemoryStream())
            {
                SoapFormatter formatter = new SoapFormatter();
                formatter.Serialize(stream, BaseNode);
                stream.Flush();
                newData = Encoding.UTF8.GetString(stream.GetBuffer(), 0, (int)stream.Position);
            }

            bool userConfirms = AskUserConfigurationSave(warnUser, newData, oldData, previousVersion);

            if (userConfirms)
            {
                int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
                SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                    CreateSWSaveAttribute("URDF Export Configuration");
                param = saveExporterAttribute.GetParameter("data");
                param.SetStringValue2(stringWriter.ToString(), ConfigurationOptions, "");
                param = saveExporterAttribute.GetParameter("name");
                param.SetStringValue2("config1", ConfigurationOptions, "");
                param = saveExporterAttribute.GetParameter("date");
                param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
                param = saveExporterAttribute.GetParameter("exporterVersion");
                param.SetStringValue2("1.3", ConfigurationOptions, "");
            }
        }

        public void SaveConfigTreeXML(ModelDoc2 model, LinkNode BaseNode, bool warnUser)
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
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        param = att.GetParameter("data");
                        oldData = param.GetStringValue();
                    }
                }
            }
            //moveComponentsToFolder((LinkNode)tree.Nodes[0]);
            RetrieveSWComponentPIDs(BaseNode);
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
                if (!warnUser ||
                    (warnUser &&
                    MessageBox.Show("The configuration has changed, would you like to save?",
                    "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes))
                {
                    int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                        CreateSWSaveAttribute("URDF Export Configuration");
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

        //As nodes are created and destroyed, this menu gets called a lot. It basically just
        // adds the context menu (right-click menu) to the node
        public void AddDocMenu(LinkNode node)
        {
            node.ContextMenuStrip = docMenu;
            foreach (LinkNode child in node.Nodes)
            {
                AddDocMenu(child);
            }
        }

        // Gets all the features in the SolidWorks model doc that match the specific feature name,
        // and updates the specified combobox.
        private void UpdateComboBoxFromFeatures(
            PropertyManagerPageCombobox box, string featureName)
        {
            List<string> featureNames = Exporter.FindRefGeoNames(featureName);
            FillComboBox(box, featureNames);
        }

        // Populates the combo box with feature names
        private void FillComboBox(PropertyManagerPageCombobox box, List<string> featureNames)
        {
            box.Clear();
            box.AddItems("Automatically Generate");
            foreach (string name in featureNames)
            {
                box.AddItems(name);
            }
        }

        // Finds the specified item in a combobox and sets the box to it. I'm not sure why I
        // couldn't do this with a foreach loop or even a for loop, but there is no way to get
        // the current number of items in the menu
        private void SelectComboBox(PropertyManagerPageCombobox box, string item)
        {
            short i = 0;
            string itemtext = "nothing";
            box.CurrentSelection = 0;

            // Cycles through the menu items until it finds what its looking for, it finds
            // blank strings, or itemtext is null
            while (String.IsNullOrWhiteSpace(itemtext) && itemtext != item)
            {
                // Gets the item text at index in a pull-down menu. No way to now how many
                // items are in the combobox
                itemtext = box.get_ItemText(i);
                if (itemtext == item)
                {
                    box.CurrentSelection = i;
                }
                i++;
            }
        }

        // Adds an asterix to the node text if it is incomplete (not currently used)
        private void UpdateNodeNames(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                UpdateNodeNames(node);
            }
        }

        // Adds an asterix to the node text if it is incomplete (not currently used)
        private void UpdateNodeNames(LinkNode node)
        {
            if (node.IsIncomplete)
            {
                node.Text = node.LinkName + "*";
            }
            foreach (LinkNode child in node.Nodes)
            {
                UpdateNodeNames(child);
            }
        }

        // Determines how many nodes need to be built, and they are added to the current node
        private void CreateNewNodes(LinkNode CurrentlySelectedNode)
        {
            int nodesToBuild = (int)PMNumberBoxChildCount.Value - CurrentlySelectedNode.Nodes.Count;
            CreateNewNodes(CurrentlySelectedNode, nodesToBuild);
        }

        // Adds the number of empty nodes to the currently active node
        private void CreateNewNodes(LinkNode currentNode, int number)
        {
            for (int i = 0; i < number; i++)
            {
                LinkNode node = CreateEmptyNode(currentNode);
                currentNode.Nodes.Add(node);
            }
            for (int i = 0; i < -number; i++)
            {
                currentNode.Nodes.RemoveAt(currentNode.Nodes.Count - 1);
            }
            int itemsCount = Common.GetCount(Tree.Nodes);
            int itemHeight = 1 + itemsCount * Tree.ItemHeight;
            int min = 163;
            int max = 600;

            int height = MathOps.Envelope(itemHeight, min, max);
            Tree.Height = height;
            PMTree.Height = height;
            currentNode.ExpandAll();
        }

        // When a new node is selected or another node is found that needs to be visited, this
        // method saves the previously active node and fills in the property mananger with the new one
        public void SwitchActiveNodes(LinkNode node)
        {
            SaveActiveNode();

            Font fontRegular = new Font(Tree.Font, FontStyle.Regular);
            Font fontBold = new Font(Tree.Font, FontStyle.Bold);
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.NodeFont = fontRegular;
            }
            FillPropertyManager(node);

            //If this flag is set to true, it prevents this method from getting called again when
            // changing the selected node
            automaticallySwitched = true;

            //Change the selected node to the argument node. This highlights the newly activated node
            Tree.SelectedNode = node;

            node.NodeFont = fontBold;
            node.Text = node.Text;
            previouslySelectedNode = node;
            CheckNodeComplete(node);
        }

        // This method runs through first the child nodes of the selected node to see if there are
        // more to visit then it runs through the nodes top to bottom to find the next to visit.
        // Returns the node if one is found otherwise it returns null.
        public LinkNode FindNextLinkToVisit(TreeView tree)
        {
            // First check if SelectedNode has any nodes to visit
            if (tree.SelectedNode != null)
            {
                LinkNode nodeToReturn = FindNextLinkToVisit((LinkNode)tree.SelectedNode);
                if (nodeToReturn != null)
                {
                    return nodeToReturn;
                }
            }

            // Now run through tree to see if any other nodes need to be visited
            return FindNextLinkToVisit((LinkNode)tree.Nodes[0]);
        }

        // Finds the next incomplete node and returns that
        public LinkNode FindNextLinkToVisit(LinkNode nodeToCheck)
        {
            if (nodeToCheck.Link.isIncomplete)
            {
                return nodeToCheck;
            }
            foreach (LinkNode node in nodeToCheck.Nodes)
            {
                return FindNextLinkToVisit(node);
            }
            return null;
        }

        //Sets the node's isIncomplete flag if the node has key items that need to be completed
        public void CheckNodeComplete(LinkNode node)
        {
            node.WhyIncomplete = "";
            node.IsIncomplete = false;
            if (String.IsNullOrWhiteSpace(node.LinkName))
            {
                node.IsIncomplete = true;
                node.WhyIncomplete += "        Link name is empty. Fill in a unique link name\r\n";
            }
            if (node.Nodes.Count > 0 && node.Components.Count == 0)
            {
                node.IsIncomplete = true;
                node.WhyIncomplete +=
                    "        Links with children cannot be empty. Select its associated components\r\n";
            }
            if (node.Components.Count == 0 && node.CoordsysName == "Automatically Generate")
            {
                node.IsIncomplete = true;
                node.WhyIncomplete +=
                    "        The origin reference coordinate system cannot be automatically generated\r\n";
                node.WhyIncomplete +=
                    "        without components. Either select an origin or at least one component.";
            }
            if (String.IsNullOrWhiteSpace(node.JointName) && !node.IsBaseNode)
            {
                node.IsIncomplete = true;
                node.WhyIncomplete += "        Joint name is empty. Fill in a unique joint name\r\n";
            }
        }

        //Recursive function to iterate though nodes and build a message containing those that are incomplete
        public string CheckNodesComplete(LinkNode node, string incompleteNodes)
        {
            // Determine if the node is incomplete
            CheckNodeComplete(node);
            if (node.IsIncomplete)
            {
                //Building the message
                incompleteNodes += "    '" + node.Text + "':\r\n" + node.WhyIncomplete + "\r\n\r\n";
            }
            // Cycle through the rest of the nodes
            foreach (LinkNode child in node.Nodes)
            {
                incompleteNodes = CheckNodesComplete(child, incompleteNodes);
            }
            return incompleteNodes;
        }

        //Finds all the nodes in a TreeView that need to be completed before exporting
        public bool CheckNodesComplete(TreeView tree)
        {
            //Calls the recursive function starting with the base_link node and retrieves a string
            // identifying the incomplete nodes
            string incompleteNodes = CheckNodesComplete((LinkNode)tree.Nodes[0], "");
            if (!String.IsNullOrWhiteSpace(incompleteNodes))
            {
                MessageBox.Show(
                    "The following nodes are incomplete. You need to fix them before continuing.\r\n\r\n" + incompleteNodes);
                return false;
            }
            return true;
        }

        // When the selected node is changed, the previously active node needs to be saved
        public void SaveActiveNode()
        {
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.LinkName = PMTextBoxLinkName.Text;
                if (!previouslySelectedNode.IsBaseNode)
                {
                    previouslySelectedNode.JointName = PMTextBoxJointName.Text;
                    previouslySelectedNode.AxisName = PMComboBoxAxes.get_ItemText(-1);
                    previouslySelectedNode.CoordsysName = PMComboBoxCoordSys.get_ItemText(-1);
                    previouslySelectedNode.JointType = PMComboBoxJointType.get_ItemText(-1);
                }
                else
                {
                    previouslySelectedNode.CoordsysName =
                        PMComboBoxGlobalCoordsys.get_ItemText(-1);
                }
                Common.GetSelectedComponents(
                    ActiveSWModel, previouslySelectedNode.Components, PMSelection.Mark);
            }
        }

        //Creates an Empty node when children are added to a link
        public LinkNode CreateEmptyNode(LinkNode Parent)
        {
            LinkNode node = new LinkNode();

            if (Parent == null)             //For the base_link node
            {
                node.LinkName = "base_link";
                node.AxisName = "";
                node.CoordsysName = "Automatically Generate";
                node.Components = new List<Component2>();
                node.IsBaseNode = true;
                node.IsIncomplete = true;
            }
            else
            {
                node.IsBaseNode = false;
                node.LinkName = "Empty_Link";
                node.AxisName = "Automatically Generate";
                node.CoordsysName = "Automatically Generate";
                node.JointType = "Automatically Detect";
                node.Components = new List<Component2>();
                node.IsBaseNode = false;
                node.IsIncomplete = true;
            }
            node.Name = node.LinkName;
            node.Text = node.LinkName;
            node.ContextMenuStrip = docMenu;
            return node;
        }

        //Sets all the controls in the Property Manager from the Selected Node
        public void FillPropertyManager(LinkNode node)
        {
            PMTextBoxLinkName.Text = node.LinkName;
            PMNumberBoxChildCount.Value = node.Nodes.Count;

            //Selecting the associated link components
            Common.SelectComponents(ActiveSWModel, node.Components, true, PMSelection.Mark);

            //Setting joint properties
            if (!node.IsBaseNode && node.Parent != null)
            {
                //Combobox needs to be blanked before de-activating
                SelectComboBox(PMComboBoxGlobalCoordsys, "");

                //Labels need to be activated before changing them
                EnableControls(!node.IsBaseNode);
                PMTextBoxJointName.Text = node.JointName;
                PMLabelParentLink.Caption = node.Parent.Name;

                UpdateComboBoxFromFeatures(PMComboBoxCoordSys, "CoordSys");
                //checkTransforms(ActiveSWModel);

                UpdateComboBoxFromFeatures(PMComboBoxAxes, "RefAxis");
                PMComboBoxAxes.AddItems("None");
                SelectComboBox(PMComboBoxCoordSys, node.CoordsysName);
                SelectComboBox(PMComboBoxAxes, node.AxisName);
                SelectComboBox(PMComboBoxJointType, node.JointType);
            }
            else
            {
                //Labels and text box have be blanked before de-activating them
                PMLabelParentLink.Caption = " ";
                SelectComboBox(PMComboBoxCoordSys, "");
                SelectComboBox(PMComboBoxAxes, "");
                SelectComboBox(PMComboBoxJointType, "");

                //Activate controls before changing them
                EnableControls(!node.IsBaseNode);
                UpdateComboBoxFromFeatures(PMComboBoxGlobalCoordsys, "CoordSys");
                SelectComboBox(PMComboBoxGlobalCoordsys, node.CoordsysName);
            }
        }

        //Takes care of activating/deactivating the drop down menus, lables and text box for
        // joint configuration. Generally these are deactivated for the base node
        private void EnableControls(bool enableJoints)
        {
            PropertyManagerPageControl[] pmJointControls =
                new PropertyManagerPageControl[] { (PropertyManagerPageControl)PMTextBoxJointName,
                                                    (PropertyManagerPageControl)PMLabelJointName,
                                                    (PropertyManagerPageControl)PMComboBoxCoordSys,
                                                    (PropertyManagerPageControl)PMLabelCoordSys,
                                                    (PropertyManagerPageControl)PMComboBoxAxes,
                                                    (PropertyManagerPageControl)PMLabelAxes,
                                                    (PropertyManagerPageControl)PMComboBoxJointType,
                                                    (PropertyManagerPageControl)PMLabelJointType };

            PropertyManagerPageControl[] pmGlobalOriginControls = new PropertyManagerPageControl[] {
                (PropertyManagerPageControl)PMComboBoxGlobalCoordsys,
                (PropertyManagerPageControl)PMLabelGlobalCoordsys};

            PropertyManagerPageControl[] pmJointOriginControls = new PropertyManagerPageControl[] {
                (PropertyManagerPageControl)PMComboBoxCoordSys,
                (PropertyManagerPageControl)PMLabelCoordSys};

            foreach (PropertyManagerPageControl control in pmGlobalOriginControls)
            {
                // Make the global origin controls visible when no joint controls are needed
                control.Visible = !enableJoints;
                control.Enabled = !enableJoints;
            }
            foreach (PropertyManagerPageControl control in pmJointOriginControls)
            {
                control.Visible = enableJoints;
                control.Enabled = enableJoints;
            }
            foreach (PropertyManagerPageControl control in pmJointControls)
            {
                control.Enabled = enableJoints;
                control.Visible = enableJoints;
            }
        }

        //Only allows components to be selected for the PMPage selection box
        private void SetComponentFilters()
        {
            swSelectType_e[] filters = new swSelectType_e[1];
            filters[0] = swSelectType_e.swSelCOMPONENTS;
            object filterObj = null;
            filterObj = filters;
            PMSelection.SetSelectionFilters(filterObj);
        }

        // This removes the component only filters so that the export tool can select sketches,
        // sketch items etc while the PMPage is active and items are added to the selection box.
        // Because the PMPage closes before selections need to occur, this method is no longer used.
        private void SetGeneralFilters()
        {
            swSelectType_e[] filters = new swSelectType_e[15];
            filters[0] = swSelectType_e.swSelCOMPONENTS;
            filters[1] = swSelectType_e.swSelEXTSKETCHPOINTS;
            filters[2] = swSelectType_e.swSelEXTSKETCHSEGS;
            filters[3] = swSelectType_e.swSelSKETCHES;
            filters[4] = swSelectType_e.swSelSKETCHPOINTS;
            filters[5] = swSelectType_e.swSelSKETCHSEGS;
            filters[6] = swSelectType_e.swSelCOORDSYS;
            filters[7] = swSelectType_e.swSelDATUMAXES;
            filters[8] = swSelectType_e.swSelDATUMPOINTS;
            filters[9] = swSelectType_e.swSelCONNECTIONPOINTS;
            filters[10] = swSelectType_e.swSelFRAMEPOINT;
            filters[11] = swSelectType_e.swSelMIDPOINTS;
            filters[12] = swSelectType_e.swSelROUTEPOINTS;
            filters[13] = swSelectType_e.swSelSKETCHPOINTFEAT;
            filters[14] = swSelectType_e.swSelVERTICES;

            object filterObj = null;

            filterObj = filters;
            PMSelection.SetSelectionFilters(filterObj);
        }

        //Populates the TreeView with the organized links from the robot
        public void FillTreeViewFromRobot(Robot robot)
        {
            Tree.Nodes.Clear();
            LinkNode baseNode = new LinkNode();
            Link baseLink = robot.BaseLink;
            baseNode.Name = baseLink.Name;
            baseNode.Text = baseLink.Name;
            baseNode.Link = baseLink;
            baseNode.ContextMenuStrip = docMenu;

            foreach (Link child in baseLink.Children)
            {
                baseNode.Nodes.Add(CreateLinkNodeFromLink(child));
            }
            Tree.Nodes.Add(baseNode);
            Tree.ExpandAll();
        }

        // Similar to the AssemblyExportForm method. It creates a LinkNode from a Link object
        public LinkNode CreateLinkNodeFromLink(Link Link)
        {
            LinkNode node = new LinkNode();
            node.Name = Link.Name;
            node.Text = Link.Name;
            node.Link = Link;
            node.ContextMenuStrip = docMenu;

            foreach (Link child in Link.Children)
            {
                node.Nodes.Add(CreateLinkNodeFromLink(child));
            }

            // Need to erase the children from the embedded link because they may be rearranged later.
            node.Link.Children.Clear();
            return node;
        }

        public void LoadConfigTree()
        {
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            string data = "";
            double configVersion = 0.0;
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        Parameter param = att.GetParameter("data");
                        data = param.GetStringValue();
                        logger.Info("URDF Configuration found\n" + data);

                        param = att.GetParameter("exporterVersion");
                        configVersion = Double.TryParse(param.GetStringValue(), out configVersion) ? configVersion : 0.0;
                    }
                }
            }

            LinkNode basenode = null;
            if (configVersion >= SOAP_MIN_VERSION)
            {
                basenode = LoadConfigFromStringSoap(data);
            }
            else
            {
                basenode = LoadConfigFromStringXML(data);
            }

            if (basenode == null)
            {
                logger.Info("Starting new configuration");
                basenode = CreateEmptyNode(null);
            }
            AddDocMenu(basenode);

            Tree.Nodes.Clear();
            Tree.Nodes.Add(basenode);
            Tree.ExpandAll();
            Tree.SelectedNode = Tree.Nodes[0];
        }

        private LinkNode LoadConfigFromStringXML(string data)
        {
            LinkNode baseNode = null;
            if (!String.IsNullOrWhiteSpace(data))
            {
                XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
                XmlTextReader textReader = new XmlTextReader(new StringReader(data));
                SerialNode node = (SerialNode)serializer.Deserialize(textReader);
                baseNode = new LinkNode(node);
                Common.LoadSWComponents(ActiveSWModel, baseNode);
                textReader.Close();
            }
            return baseNode;
        }

        private LinkNode LoadConfigFromStringSoap(string data)
        {
            LinkNode baseNode = null;
            if (!String.IsNullOrWhiteSpace(data))
            {
                using (MemoryStream stream = new MemoryStream(Encoding.UTF8.GetBytes(data)))
                {
                    SoapFormatter formatter = new SoapFormatter();
                    baseNode = (LinkNode)formatter.Deserialize(stream);
                }

                Common.LoadSWComponents(ActiveSWModel, baseNode);
            }
            return baseNode;
        }

        public void RetrieveSWComponentPIDs(LinkNode node)
        {
            if (node.Components != null)
            {
                node.ComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in node.Components)
                {
                    byte[] PID = ActiveSWModel.Extension.GetPersistReference3(comp);
                    node.ComponentPIDs.Add(PID);
                }
            }
            foreach (LinkNode child in node.Nodes)
            {
                RetrieveSWComponentPIDs(child);
            }
        }

        public void RetrieveSWComponentPIDs(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                RetrieveSWComponentPIDs(node);
            }
        }

        public void MoveComponentsToFolder(LinkNode node)
        {
            bool needToCreateFolder = true;
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                if (feat.Name == "URDF Export Items")
                {
                    needToCreateFolder = false;
                }
            }
            ActiveSWModel.ClearSelection2(true);
            ActiveSWModel.Extension.SelectByID2(
                "Origin_global", "COORDSYS", 0, 0, 0, true, 0, null, 0);
            if (needToCreateFolder)
            {
                Feature folderFeature =
                    ActiveSWModel.FeatureManager.InsertFeatureTreeFolder2(
                        (int)swFeatureTreeFolderType_e.swFeatureTreeFolder_Containing);
                folderFeature.Name = "URDF Export Items";
            }
            ActiveSWModel.Extension.SelectByID2
                ("URDF Reference", "SKETCH", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
            ActiveSWModel.Extension.SelectByID2
                ("URDF Export Configuration", "ATTRIBUTE", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
            SelectFeatures(node);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
        }

        public void SelectFeatures(LinkNode node)
        {
            ActiveSWModel.Extension.SelectByID2(
                node.CoordsysName, "COORDSYS", 0, 0, 0, true, -1, null, 0);
            if (node.AxisName != "None")
            {
                ActiveSWModel.Extension.SelectByID2(
                    node.AxisName, "AXIS", 0, 0, 0, true, -1, null, 0);
            }
            foreach (LinkNode child in node.Nodes)
            {
                SelectFeatures(child);
            }
        }

        public void CheckIfLinkNamesAreUnique(LinkNode node, string linkName, List<string> conflict)
        {
            if (node.LinkName == linkName)
            {
                conflict.Add(node.LinkName);
            }

            foreach (LinkNode child in node.Nodes)
            {
                CheckIfLinkNamesAreUnique(child, linkName, conflict);
            }
        }

        public void CheckIfJointNamesAreUnique(LinkNode node, string jointName, List<string> conflict)
        {
            if (node.JointName == jointName)
            {
                conflict.Add(node.LinkName);
            }
            foreach (LinkNode child in node.Nodes)
            {
                CheckIfLinkNamesAreUnique(child, jointName, conflict);
            }
        }

        public bool CheckIfNamesAreUnique(LinkNode node)
        {
            List<List<string>> linkConflicts = new List<List<string>>();
            List<List<string>> jointConflicts = new List<List<string>>();
            CheckIfLinkNamesAreUnique(node, node, linkConflicts);
            CheckIfJointNamesAreUnique(node, node, jointConflicts);

            string message = "\r\nPlease fix these errors before proceeding.";
            string specificErrors = "";
            bool displayInitialMessage = true;
            bool linkNamesInConflict = false;
            foreach (List<string> conflict in linkConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors +=
                            "The following links have LINK names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                }
            }
            displayInitialMessage = true;
            foreach (List<string> conflict in jointConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors +=
                            "The following links have JOINT names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                }
            }
            if (linkNamesInConflict)
            {
                MessageBox.Show(specificErrors + message);
                return false;
            }
            return true;
        }

        public void CheckIfLinkNamesAreUnique(
            LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentNode with all the other nodes
            CheckIfLinkNamesAreUnique(basenode, currentNode.LinkName, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }
            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (LinkNode child in currentNode.Nodes)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts
                // list of lists.
                CheckIfLinkNamesAreUnique(basenode, child, conflicts);
            }
        }

        public void CheckIfJointNamesAreUnique(
            LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentNode with all the other nodes
            CheckIfJointNamesAreUnique(basenode, currentNode.JointName, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (conflict.Count > 0 && existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }

            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (LinkNode child in currentNode.Nodes)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts
                // list of lists.
                CheckIfJointNamesAreUnique(basenode, child, conflicts);
            }
        }

        private SolidWorks.Interop.sldworks.Attribute CreateSWSaveAttribute(string name)
        {
            int Options = 0;
            if (saveConfigurationAttributeDef == null)
            {
                saveConfigurationAttributeDef = swApp.DefineAttribute("URDF Export Configuration");

                saveConfigurationAttributeDef.AddParameter(
                    "data", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter(
                    "name", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter(
                    "date", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter(
                    "exporterVersion", (int)swParamType_e.swParamTypeDouble, 1.0, Options);
                saveConfigurationAttributeDef.Register();
            }

            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            ModelDoc2 modeldoc = swApp.ActiveDoc;
            Object[] objects = modeldoc.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == name)
                    {
                        return att;
                    }
                }
            }
            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                saveConfigurationAttributeDef.CreateInstance5(
                    ActiveSWModel, null, "URDF Export Configuration", Options, ConfigurationOptions);
            return saveExporterAttribute;
        }
    }
}