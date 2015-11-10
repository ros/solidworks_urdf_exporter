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
using System.IO;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using System.Drawing;

namespace SW2URDF
{
    public partial class URDFExporterPM : PropertyManagerPage2Handler9
    {

        public AttributeDef saveConfigurationAttributeDef;

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
            retrieveSWComponentPIDs(BaseNode);
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
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = createSWSaveAttribute("URDF Export Configuration");
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
 


        //As nodes are created and destroyed, this menu gets called a lot. It basically just adds the context menu (right-click menu)
        // to the node
        public void addDocMenu(LinkNode node)
        {
            node.ContextMenuStrip = docMenu;
            foreach (LinkNode child in node.Nodes)
            {
                addDocMenu(child);
            }
        }

        // Gets all the features in the SolidWorks model doc that match the specific feature name, and updates the specified combobox.
        private void updateComboBoxFromFeatures(PropertyManagerPageCombobox box, string featureName)
        {
            List<string> featureNames = Exporter.FindRefGeoNames(featureName);
            fillComboBox(box, featureNames);
        }

        // Populates the combo box with feature names
        private void fillComboBox(PropertyManagerPageCombobox box, List<string> featureNames)
        {
            box.Clear();
            box.AddItems("Automatically Generate");
            foreach (string name in featureNames)
            {
                box.AddItems(name);
            }
        }

        // Finds the specified item in a combobox and sets the box to it. I'm not sure why I couldn't do this with a foreach loop
        // or even a for loop, but there is no way to get the current number of items in the menu
        private void selectComboBox(PropertyManagerPageCombobox box, string item)
        {
            short i = 0;
            string itemtext = "nothing";
            box.CurrentSelection = 0;

            // Cycles through the menu items until it finds what its looking for, it finds blank strings, or itemtext is null
            while (itemtext != null && itemtext != "" && itemtext != item)
            {
                // Gets the item text at index in a pull-down menu. No way to now how many items are in the combobox
                itemtext = box.get_ItemText(i);
                if (itemtext == item)
                {
                    box.CurrentSelection = i;
                }
                i++;
            }
        }

        // Adds an asterix to the node text if it is incomplete (not currently used)
        private void updateNodeNames(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                updateNodeNames(node);
            }
        }

        // Adds an asterix to the node text if it is incomplete (not currently used)
        private void updateNodeNames(LinkNode node)
        {
            if (node.isIncomplete)
            {
                node.Text = node.linkName + "*";
            }
            foreach (LinkNode child in node.Nodes)
            {
                updateNodeNames(child);
            }
        }

        // Determines how many nodes need to be built, and they are added to the current node
        private void createNewNodes(LinkNode CurrentlySelectedNode)
        {
            int nodesToBuild = (int)pm_NumberBox_ChildCount.Value - CurrentlySelectedNode.Nodes.Count;
            createNewNodes(CurrentlySelectedNode, nodesToBuild);
        }

        // Adds the number of empty nodes to the currently active node
        private void createNewNodes(LinkNode currentNode, int number)
        {
            for (int i = 0; i < number; i++)
            {
                LinkNode node = createEmptyNode(currentNode);
                currentNode.Nodes.Add(node);
            }
            for (int i = 0; i < -number; i++)
            {
                currentNode.Nodes.RemoveAt(currentNode.Nodes.Count - 1);
            }
            int itemsCount = Common.getCount(tree.Nodes);
            int itemHeight = 1 + itemsCount * tree.ItemHeight;
            int min = 163;
            int max = 600;

            int height = ops.envelope(itemHeight, min, max);
            tree.Height = height;
            pm_tree.Height = height;
            currentNode.ExpandAll();
        }

        // When a new node is selected or another node is found that needs to be visited, this method saves the previously
        // active node and fills in the property mananger with the new one
        public void switchActiveNodes(LinkNode node)
        {
            saveActiveNode();

            Font fontRegular = new Font(tree.Font, FontStyle.Regular);
            Font fontBold = new Font(tree.Font, FontStyle.Bold);
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.NodeFont = fontRegular;
            }
            fillPropertyManager(node);

            //If this flag is set to true, it prevents this method from getting called again when changing the selected node
            automaticallySwitched = true;

            //Change the selected node to the argument node. This highlights the newly activated node
            tree.SelectedNode = node;

            node.NodeFont = fontBold;
            node.Text = node.Text;
            previouslySelectedNode = node;
            checkNodeComplete(node);
        }

        // This method runs through first the child nodes of the selected node to see if there are more to visit
        // then it runs through the nodes top to bottom to find the next to visit. Returns the node if one is found
        // otherwise it returns null.
        public LinkNode findNextLinkToVisit(System.Windows.Forms.TreeView tree)
        {
            // First check if SelectedNode has any nodes to visit
            if (tree.SelectedNode != null)
            {
                LinkNode nodeToReturn = findNextLinkToVisit((LinkNode)tree.SelectedNode);
                if (nodeToReturn != null)
                {
                    return nodeToReturn;
                }
            }

            // Now run through tree to see if any other nodes need to be visited
            return findNextLinkToVisit((LinkNode)tree.Nodes[0]);
        }

        // Finds the next incomplete node and returns that
        public LinkNode findNextLinkToVisit(LinkNode nodeToCheck)
        {
            if (nodeToCheck.Link.isIncomplete)
            {
                return nodeToCheck;
            }
            foreach (LinkNode node in nodeToCheck.Nodes)
            {
                return findNextLinkToVisit(node);
            }
            return null;
        }

        //Sets the node's isIncomplete flag if the node has key items that need to be completed
        public void checkNodeComplete(LinkNode node)
        {
            node.whyIncomplete = "";
            node.isIncomplete = false;
            if (node.linkName.Equals(""))
            {
                node.isIncomplete = true;
                node.whyIncomplete += "        Link name is empty. Fill in a unique link name\r\n";
            }
            if (node.Nodes.Count > 0 && node.Components.Count == 0)
            {
                node.isIncomplete = true;
                node.whyIncomplete += "        Links with children cannot be empty. Select its associated components\r\n";
            }
            if (node.Components.Count == 0 && node.coordsysName == "Automatically Generate")
            {
                node.isIncomplete = true;
                node.whyIncomplete += "        The origin reference coordinate system cannot be automatically generated\r\n";
                node.whyIncomplete += "        without components. Either select an origin or at least one component.";
            }
            if (node.jointName == "" && !node.isBaseNode)
            {
                node.isIncomplete = true;
                node.whyIncomplete += "        Joint name is empty. Fill in a unique joint name\r\n";
            }
        }

        //Recursive function to iterate though nodes and build a message containing those that are incomplete
        public string checkNodesComplete(LinkNode node, string incompleteNodes)
        {
            // Determine if the node is incomplete
            checkNodeComplete(node);
            if (node.isIncomplete)
            {
                incompleteNodes += "    '" + node.Text + "':\r\n" + node.whyIncomplete + "\r\n\r\n"; //Building the message
            }
            // Cycle through the rest of the nodes
            foreach (LinkNode child in node.Nodes)
            {
                incompleteNodes = checkNodesComplete(child, incompleteNodes);
            }
            return incompleteNodes;
        }

        //Finds all the nodes in a TreeView that need to be completed before exporting
        public bool checkNodesComplete(TreeView tree)
        {
            //Calls the recursive function starting with the base_link node and retrieves a string identifying the incomplete nodes
            string incompleteNodes = checkNodesComplete((LinkNode)tree.Nodes[0], "");
            if (incompleteNodes != "")
            {
                MessageBox.Show("The following nodes are incomplete. You need to fix them before continuing.\r\n\r\n" + incompleteNodes);
                return false;
            }
            return true;
        }

        // When the selected node is changed, the previously active node needs to be saved
        public void saveActiveNode()
        {
            if (previouslySelectedNode != null)
            {
                previouslySelectedNode.linkName = pm_TextBox_LinkName.Text;
                if (!previouslySelectedNode.isBaseNode)
                {
                    previouslySelectedNode.jointName = pm_TextBox_JointName.Text;
                    previouslySelectedNode.axisName = pm_ComboBox_Axes.get_ItemText(-1);
                    previouslySelectedNode.coordsysName = pm_ComboBox_CoordSys.get_ItemText(-1);
                    previouslySelectedNode.jointType = pm_ComboBox_JointType.get_ItemText(-1);
                }
                else
                {
                    previouslySelectedNode.coordsysName = pm_ComboBox_GlobalCoordsys.get_ItemText(-1);
                }
                Common.getSelectedComponents(ActiveSWModel, previouslySelectedNode.Components, pm_Selection.Mark);
            }
        }

        //Creates an Empty node when children are added to a link
        public LinkNode createEmptyNode(LinkNode Parent)
        {
            LinkNode node = new LinkNode();

            if (Parent == null)             //For the base_link node
            {
                node.linkName = "base_link";
                node.axisName = "";
                node.coordsysName = "Automatically Generate";
                node.Components = new List<Component2>();
                node.isBaseNode = true;
                node.isIncomplete = true;
            }
            else
            {
                node.isBaseNode = false;
                node.linkName = "Empty_Link";
                node.axisName = "Automatically Generate";
                node.coordsysName = "Automatically Generate";
                node.jointType = "Automatically Detect";
                node.Components = new List<Component2>();
                node.isBaseNode = false;
                node.isIncomplete = true;
            }
            node.Name = node.linkName;
            node.Text = node.linkName;
            node.ContextMenuStrip = docMenu;
            return node;
        }

        //Sets all the controls in the Property Manager from the Selected Node
        public void fillPropertyManager(LinkNode node)
        {
            pm_TextBox_LinkName.Text = node.linkName;
            pm_NumberBox_ChildCount.Value = node.Nodes.Count;

            //Selecting the associated link components
            Common.selectComponents(ActiveSWModel, node.Components, true, pm_Selection.Mark);

            //Setting joint properties
            if (!node.isBaseNode && node.Parent != null)
            {
                //Combobox needs to be blanked before de-activating
                selectComboBox(pm_ComboBox_GlobalCoordsys, "");

                //Labels need to be activated before changing them
                enableControls(!node.isBaseNode);
                pm_TextBox_JointName.Text = node.jointName;
                pm_Label_ParentLink.Caption = node.Parent.Name;

                updateComboBoxFromFeatures(pm_ComboBox_CoordSys, "CoordSys");
                //checkTransforms(ActiveSWModel);

                updateComboBoxFromFeatures(pm_ComboBox_Axes, "RefAxis");
                pm_ComboBox_Axes.AddItems("None");
                selectComboBox(pm_ComboBox_CoordSys, node.coordsysName);
                selectComboBox(pm_ComboBox_Axes, node.axisName);
                selectComboBox(pm_ComboBox_JointType, node.jointType);
            }
            else
            {
                //Labels and text box have be blanked before de-activating them
                pm_Label_ParentLink.Caption = " ";
                selectComboBox(pm_ComboBox_CoordSys, "");
                selectComboBox(pm_ComboBox_Axes, "");
                selectComboBox(pm_ComboBox_JointType, "");

                //Activate controls before changing them
                enableControls(!node.isBaseNode);
                updateComboBoxFromFeatures(pm_ComboBox_GlobalCoordsys, "CoordSys");
                selectComboBox(pm_ComboBox_GlobalCoordsys, node.coordsysName);
            }
        }

        //Takes care of activating/deactivating the drop down menus, lables and text box for joint configuration
        //Generally these are deactivated for the base node
        private void enableControls(bool enableJoints)
        {
            PropertyManagerPageControl[] pm_joint_controls = new PropertyManagerPageControl[] { (PropertyManagerPageControl)pm_TextBox_JointName, 
                                                                                          (PropertyManagerPageControl)pm_Label_JointName, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_CoordSys, 
                                                                                          (PropertyManagerPageControl)pm_Label_CoordSys, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_Axes, 
                                                                                          (PropertyManagerPageControl)pm_Label_Axes, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_JointType, 
                                                                                          (PropertyManagerPageControl)pm_Label_JointType };

            PropertyManagerPageControl[] pm_GlobalOrigin_controls = new PropertyManagerPageControl[] { (PropertyManagerPageControl)pm_ComboBox_GlobalCoordsys, 
                                                                                                       (PropertyManagerPageControl)pm_Label_GlobalCoordsys};

            PropertyManagerPageControl[] pm_JointOrigin_controls = new PropertyManagerPageControl[] { (PropertyManagerPageControl)pm_ComboBox_CoordSys, 
                                                                                                       (PropertyManagerPageControl)pm_Label_CoordSys};

            foreach (PropertyManagerPageControl control in pm_GlobalOrigin_controls)
            {
                control.Visible = !enableJoints; // Make the global origin controls visible when no joint controls are needed
                control.Enabled = !enableJoints;
            }
            foreach (PropertyManagerPageControl control in pm_JointOrigin_controls)
            {
                control.Visible = enableJoints;
                control.Enabled = enableJoints;
            }
            foreach (PropertyManagerPageControl control in pm_joint_controls)
            {
                control.Enabled = enableJoints;
                control.Visible = enableJoints;
            }
        }

        //Only allows components to be selected for the PMPage selection box
        void setComponentFilters()
        {
            swSelectType_e[] filters = new swSelectType_e[1];
            filters[0] = swSelectType_e.swSelCOMPONENTS;
            object filterObj = null;
            filterObj = filters;
            pm_Selection.SetSelectionFilters(filterObj);
        }

        // This removes the component only filters so that the export tool can select sketches, sketch items etc while the PMPage is active
        // and items are added to the selection box. 
        // Because the PMPage closes before selections need to occur, this method is no longer used. 
        void setGeneralFilters()
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
            pm_Selection.SetSelectionFilters(filterObj);
        }

        //Populates the TreeView with the organized links from the robot
        public void fillTreeViewFromRobot(robot robot)
        {
            tree.Nodes.Clear();
            LinkNode baseNode = new LinkNode();
            link baseLink = robot.BaseLink;
            baseNode.Name = baseLink.name;
            baseNode.Text = baseLink.name;
            baseNode.Link = baseLink;
            baseNode.ContextMenuStrip = docMenu;

            foreach (link child in baseLink.Children)
            {
                baseNode.Nodes.Add(createLinkNodeFromLink(child));
            }
            tree.Nodes.Add(baseNode);
            tree.ExpandAll();
        }

        // Similar to the AssemblyExportForm method. It creates a LinkNode from a Link object
        public LinkNode createLinkNodeFromLink(link Link)
        {
            LinkNode node = new LinkNode();
            node.Name = Link.name;
            node.Text = Link.name;
            node.Link = Link;
            node.ContextMenuStrip = docMenu;

            foreach (link child in Link.Children)
            {
                node.Nodes.Add(createLinkNodeFromLink(child));
            }
            node.Link.Children.Clear(); // Need to erase the children from the embedded link because they may be rearranged later.
            return node;
        }

        // Calls the Exporter loadConfigTree method and then populates the tree with the loaded config
        public void loadConfigTree()
        {
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            string data = "";
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        Parameter param = att.GetParameter("data");
                        data = param.GetStringValue();
                    }
                }

            }
            LinkNode basenode = null;
            if (!data.Equals(""))
            {
                XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
                XmlTextReader textReader = new XmlTextReader(new StringReader(data));
                SerialNode node = (SerialNode)serializer.Deserialize(textReader);
                basenode = new LinkNode(node);
                Common.loadSWComponents(ActiveSWModel, basenode);
                textReader.Close();
            }

            if (basenode == null)
            {
                basenode = createEmptyNode(null);
            }
            addDocMenu(basenode);

            tree.Nodes.Clear();
            tree.Nodes.Add(basenode);
            tree.ExpandAll();
            tree.SelectedNode = tree.Nodes[0];
        }

        public void retrieveSWComponentPIDs(LinkNode node)
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
                retrieveSWComponentPIDs(child);
            }
        }

        public void retrieveSWComponentPIDs(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                retrieveSWComponentPIDs(node);
            }
        }


        public void moveComponentsToFolder(LinkNode node)
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
            ActiveSWModel.Extension.SelectByID2("Origin_global", "COORDSYS", 0, 0, 0, true, 0, null, 0);
            if (needToCreateFolder)
            {
                Feature folderFeature = ActiveSWModel.FeatureManager.InsertFeatureTreeFolder2((int)swFeatureTreeFolderType_e.swFeatureTreeFolder_Containing);
                folderFeature.Name = "URDF Export Items";
            }
            ActiveSWModel.Extension.SelectByID2("URDF Reference", "SKETCH", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
            ActiveSWModel.Extension.SelectByID2("URDF Export Configuration", "ATTRIBUTE", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
            selectFeatures(node);
            ActiveSWModel.FeatureManager.MoveToFolder("URDF Export Items", "", false);
        }

        public void selectFeatures(LinkNode node)
        {
            ActiveSWModel.Extension.SelectByID2(node.coordsysName, "COORDSYS", 0, 0, 0, true, -1, null, 0);
            if (node.axisName != "None")
            {
                ActiveSWModel.Extension.SelectByID2(node.axisName, "AXIS", 0, 0, 0, true, -1, null, 0);
            }
            foreach (LinkNode child in node.Nodes)
            {
                selectFeatures(child);
            }
        }

        public void checkIfLinkNamesAreUnique(LinkNode node, string linkName, List<string> conflict)
        {
            if (node.linkName == linkName)
            {
                conflict.Add(node.linkName);
            }

            foreach (LinkNode child in node.Nodes)
            {
                checkIfLinkNamesAreUnique(child, linkName, conflict);
            }
        }

        public void checkIfJointNamesAreUnique(LinkNode node, string jointName, List<string> conflict)
        {
            if (node.jointName == jointName)
            {
                conflict.Add(node.linkName);
            }
            foreach (LinkNode child in node.Nodes)
            {
                checkIfLinkNamesAreUnique(child, jointName, conflict);
            }
        }

        public bool checkIfNamesAreUnique(LinkNode node)
        {
            List<List<string>> linkConflicts = new List<List<string>>();
            List<List<string>> jointConflicts = new List<List<string>>();
            checkIfLinkNamesAreUnique(node, node, linkConflicts);
            checkIfJointNamesAreUnique(node, node, jointConflicts);

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
                        specificErrors += "The following links have LINK names that conflict:\r\n\r\n";
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
                        specificErrors += "The following links have JOINT names that conflict:\r\n\r\n";
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

        public void checkIfLinkNamesAreUnique(LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentNode with all the other nodes
            checkIfLinkNamesAreUnique(basenode, currentNode.linkName, conflict);
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
                //Proceeds recursively through the children nodes and adds to the conflicts list of lists.
                checkIfLinkNamesAreUnique(basenode, child, conflicts);
            }
        }

        public void checkIfJointNamesAreUnique(LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentNode with all the other nodes
            checkIfJointNamesAreUnique(basenode, currentNode.jointName, conflict);
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
                //Proceeds recursively through the children nodes and adds to the conflicts list of lists.
                checkIfJointNamesAreUnique(basenode, child, conflicts);
            }
        }


        private SolidWorks.Interop.sldworks.Attribute createSWSaveAttribute(string name)
        {
            int Options = 0;
            if (saveConfigurationAttributeDef == null)
            {
                saveConfigurationAttributeDef = swApp.DefineAttribute("URDF Export Configuration");

                saveConfigurationAttributeDef.AddParameter("data", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter("name", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter("date", (int)swParamType_e.swParamTypeString, 0, Options);
                saveConfigurationAttributeDef.AddParameter("exporterVersion", (int)swParamType_e.swParamTypeDouble, 1.0, Options);
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
    }
}
