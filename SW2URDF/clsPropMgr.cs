using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using System.Xml.Serialization;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace SW2URDF
{

    [ComVisibleAttribute(true)]

    [Serializable]
    public class clsPropMgr : PropertyManagerPage2Handler9
    {
        #region class variables
        public SldWorks swApp;
        public ModelDoc2 ActiveSWModel;

        public SW2URDFExporter Exporter;
        public LinkNode previouslySelectedNode;
        public link previouslySelectedLink;
        public List<link> linksToVisit;
        public LinkNode rightClickedNode;
        private ContextMenuStrip docMenu;
        //General objects required for the PropertyManager page

        PropertyManagerPage2 pm_Page;
        PropertyManagerPageGroup pm_Group;

        PropertyManagerPageSelectionbox pm_Selection;
        PropertyManagerPageButton pm_Button_export;
        PropertyManagerPageTextbox pm_TextBox_LinkName;
        PropertyManagerPageTextbox pm_TextBox_JointName;
        PropertyManagerPageNumberbox pm_NumberBox_ChildCount;
        PropertyManagerPageCombobox pm_ComboBox_Axes;
        PropertyManagerPageCombobox pm_ComboBox_CoordSys;
        PropertyManagerPageCombobox pm_ComboBox_JointType;

        PropertyManagerPageLabel pm_Label_LinkName;
        PropertyManagerPageLabel pm_Label_JointName;
        PropertyManagerPageLabel pm_Label_Selection;
        PropertyManagerPageLabel pm_Label_ChildCount;
        PropertyManagerPageLabel pm_Label_ParentLink;
        PropertyManagerPageLabel pm_Label_ParentLinkLabel;
        PropertyManagerPageLabel pm_Label_Axes;
        PropertyManagerPageLabel pm_Label_CoordSys;
        PropertyManagerPageLabel pm_Label_JointType;

        PropertyManagerPageWindowFromHandle pm_tree;
        public TreeView tree
        { get; set; }
        bool automaticallySwitched = false;

        //Each object in the page needs a unique ID

        const int GroupID = 1;
        const int TextBox_LinkNameID = 2;
        const int SelectionID = 3;
        const int ComboID = 4;
        const int ListID = 5;
        const int Button_save_ID = 6;
        const int NumBox_ChildCount_ID = 7;
        const int Label_LinkName_ID = 8;
        const int Label_Selection_ID = 9;
        const int Label_ChildCount_ID = 10;
        const int Label_ParentLink_ID = 11;
        const int Label_ParentLinkLabel_ID = 12;
        const int TextBox_JointNameID = 13;
        const int Label_JointName_ID = 14;
        const int dotNet_tree = 16;
        const int Button_export_ID = 17;
        const int ComboBox_Axes_ID = 18;
        const int ComboBox_CoordSys_ID = 19;
        const int Label_Axes_ID = 20;
        const int Label_CoordSys_ID = 21;
        const int ComboBox_JointType_ID = 22;
        const int Label_JointType_ID = 23;
        #endregion

        public void Show()
        {
            pm_Page.Show2(0);
        }

        //The following runs when a new instance of the class is created
        public clsPropMgr(SldWorks swAppPtr)
        {
            swApp = swAppPtr;
            ActiveSWModel = swApp.ActiveDoc;

            Exporter = new SW2URDFExporter(swApp);
            Exporter.mRobot = new robot();
            Exporter.mRobot.name = ActiveSWModel.GetTitle();

            linksToVisit = new List<link>();
            docMenu = new ContextMenuStrip();

            string PageTitle = null;
            string caption = null;
            string tip = null;
            long options = 0;
            int longerrors = 0;
            int controlType = 0;
            int alignment = 0;
            string[] listItems = new string[4];

            ActiveSWModel.ShowConfiguration2("URDF Export");


            #region Create and instantiate components of PM page
            //Set the variables for the page
            PageTitle = "URDF Exporter";
            //options = (int)swPropertyManagerButtonTypes_e.swPropertyManager_OkayButton + (int)swPropertyManagerButtonTypes_e.swPropertyManager_CancelButton + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_LockedPage + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_PushpinButton;
            options = (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_OkayButton + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_CancelButton + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_HandleKeystrokes;

            //Create the PropertyManager page
            pm_Page = (PropertyManagerPage2)swApp.CreatePropertyManagerPage(PageTitle, (int)options, this, ref longerrors);

            //Make sure that the page was created properly
            if (longerrors == (int)swPropertyManagerPageStatus_e.swPropertyManagerPage_Okay)
            {
                setupPropertyManagerPage(ref caption, ref tip, ref options, ref controlType, ref alignment);
            }

            else
            {
                //If the page is not created
                System.Windows.Forms.MessageBox.Show("An error occurred while attempting to create the " + "PropertyManager Page");
            }

            #endregion
        }

        #region Property Manager Methods

        // Finds all the RefAxis and Coordsys currently in the SolidWorks model doc and adds them to the appropriate pull down menus
        private void updateComboBoxes()
        {
            List<Feature> axes = new List<Feature>();
            List<Feature> coordsys = new List<Feature>();

            object[] features;
            features = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Feature feat in features)
            {
                if (feat.GetTypeName2() == "RefAxis")
                {
                    axes.Add(feat);
                }
                else if (feat.GetTypeName2() == "CoordSys")
                {
                    coordsys.Add(feat);
                }
            }

            fillComboBox(pm_ComboBox_CoordSys, coordsys);
            fillComboBox(pm_ComboBox_Axes, axes);
            pm_ComboBox_Axes.AddItems("None");
        }

        // Populates the combo box with feature names
        private void fillComboBox(PropertyManagerPageCombobox box, List<Feature> features)
        {
            box.Clear();
            box.AddItems("Automatically Generate");

            foreach (Feature feat in features)
            {
                box.AddItems(feat.Name);
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
            node.isIncomplete = (node.linkName.Equals("Empty_Link") ||
                                 node.linkName.Equals("") ||
                                 node.Components.Count == 0 ||
                                 (node.jointName == "" && !node.isBaseNode));
        }

        //Recursive function to iterate though nodes and build a message containing those that are incomplete
        public string checkNodesComplete(LinkNode node, string incompleteNodes)
        {
            // Determine if the node is incomplete
            checkNodeComplete(node);
            if (node.isIncomplete)
            {
                incompleteNodes += "    " + node.Text + "\r\n"; //Building the message
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
                Exporter.getSelectedComponents(previouslySelectedNode.Components, pm_Selection.Mark);
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
            Exporter.selectComponents(node.Components, true, pm_Selection.Mark);

            //Setting joint properties
            if (!node.isBaseNode && node.Parent != null)
            {
                //Labels need to be activated before changing them
                enableJointControls(!node.isBaseNode);
                pm_TextBox_JointName.Text = node.jointName;
                pm_Label_ParentLink.Caption = node.Parent.Name;

                updateComboBoxes();
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
                enableJointControls(!node.isBaseNode);
            }
        }

        //Takes care of activating/deactivating the drop down menus, lables and text box for joint configuration
        //Generally these are deactivated for the base node
        private void enableJointControls(bool enabled)
        {
            PropertyManagerPageControl[] pm_controls = new PropertyManagerPageControl[] { (PropertyManagerPageControl)pm_TextBox_JointName, 
                                                                                          (PropertyManagerPageControl)pm_Label_JointName, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_CoordSys, 
                                                                                          (PropertyManagerPageControl)pm_Label_CoordSys, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_Axes, 
                                                                                          (PropertyManagerPageControl)pm_Label_Axes, 
                                                                                          (PropertyManagerPageControl)pm_ComboBox_JointType, 
                                                                                          (PropertyManagerPageControl)pm_Label_JointType };

            foreach (PropertyManagerPageControl control in pm_controls)
            {
                control.Enabled = enabled;
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
            LinkNode basenode = Exporter.loadConfigTree();
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
        #endregion

        #region Implemented Property Manager Page Handler Methods

        void IPropertyManagerPage2Handler9.AfterActivation()
        {
            //Turns the selection box blue so that selected components are added to the PMPage selection box
            pm_Selection.SetSelectionFocus();
        }

        // Called when a PropertyManagerPageButton is pressed. In our case, that's only the export button for now
        void IPropertyManagerPage2Handler9.OnButtonPress(int Id)
        {
            if (Id == Button_export_ID) //If the export button was pressed
            {
                saveActiveNode();
                if (Exporter.checkIfNamesAreUnique((LinkNode)tree.Nodes[0]) && checkNodesComplete(tree)) // Only if everything is A-OK, then do we proceed.
                {
                    pm_Page.Close(true); //It saves automatically when sending Okay as true;
                    AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;

                    //This call can be a real sink of processing time if the model is large. Unfortunately there isn't a way around it I believe.
                    int result = assy.ResolveAllLightWeightComponents(true);

                    // If the user confirms to resolve the components and they are successfully resolved we can continue
                    if (result == (int)swComponentResolveStatus_e.swResolveOk)
                    {
                        // Builds the links and joints from the PMPage configuration
                        Exporter.createRobotFromTreeView(tree);
                        AssemblyExportForm exportForm = new AssemblyExportForm(swApp);
                        exportForm.Exporter = Exporter;
                        // Fills the TreeView on the joints page of the exporter
                        exportForm.fillJointTreeFromRobot(Exporter.mRobot);
                        exportForm.Show();
                    }
                    else if (result == (int)swComponentResolveStatus_e.swResolveError || result == (int)swComponentResolveStatus_e.swResolveNotPerformed)
                    {
                        MessageBox.Show("Resolving components failed. In order to export to URDF, this tool needs all components to be resolved. Try resolving lightweight components manually before attempting to export again");
                    }
                    else if (result == (int)swComponentResolveStatus_e.swResolveAbortedByUser)
                    {
                        MessageBox.Show("In order to export to URDF, this tool needs all components to be resolved. You can resolve them manually or try exporting again");
                    }
                }
            }

        }

        void IPropertyManagerPage2Handler9.OnClose(int Reason)
        {
            if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Cancel)
            {
                saveActiveNode();
            }

            else if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Okay)
            {
                saveActiveNode();
                Exporter.saveConfigTree(tree, false);
            }
        }

        void IPropertyManagerPage2Handler9.OnGainedFocus(int Id)
        {
        }


        bool IPropertyManagerPage2Handler9.OnHelp()
        {
            return true;
        }

        bool IPropertyManagerPage2Handler9.OnKeystroke(int Wparam, int Message, int Lparam, int Id)
        {
            if (Wparam == (int)Keys.Enter)
            {
                return true;
            }
            return false;
        }

        void IPropertyManagerPage2Handler9.OnLostFocus(int Id)
        {
            Debug.Print("Control box " + Id + " has lost focus");
        }

        void IPropertyManagerPage2Handler9.OnNumberboxChanged(int Id, double Value)
        {
            if (Id == NumBox_ChildCount_ID)
            {
                LinkNode node = (LinkNode)tree.SelectedNode;
                createNewNodes(node);
                //updateNodeNames((LinkNode)tree.Nodes[0]);
            }
        }

        void IPropertyManagerPage2Handler9.OnSelectionboxFocusChanged(int Id)
        {
            Debug.Print("The focus has moved to selection box " + Id);
        }

        void IPropertyManagerPage2Handler9.OnSelectionboxListChanged(int Id, int Count)
        {
            // Move focus to next selection box if right-mouse button pressed
            pm_Page.SetCursor((int)swPropertyManagerPageCursors_e.swPropertyManagerPageCursors_Advance);
        }

        bool IPropertyManagerPage2Handler9.OnSubmitSelection(int Id, object Selection, int SelType, ref string ItemText)
        {
            // This method must return true for selections to occur
            return true;
        }

        void IPropertyManagerPage2Handler9.OnTextboxChanged(int Id, string Text)
        {
            if (Id == TextBox_LinkNameID)
            {
                LinkNode node = (LinkNode)tree.SelectedNode;
                node.Text = pm_TextBox_LinkName.Text;
                node.Name = pm_TextBox_LinkName.Text;
            }
        }

        int IPropertyManagerPage2Handler9.OnWindowFromHandleControlCreated(int Id, bool Status)
        {
            return 0;
        }


        #endregion

        #region TreeView handler methods
        // Upon selection of a node, the node displayed on the PMPage is saved and the selected one is then set
        private void tree_AfterSelect(object sender, TreeViewEventArgs e)
        {
            if (!automaticallySwitched && e.Node != null)
            {
                switchActiveNodes((LinkNode)e.Node);
            }
            automaticallySwitched = false;
        }

        // Captures which node was right clicked
        private void tree_NodeMouseClick(object sender, TreeNodeMouseClickEventArgs e)
        {
            rightClickedNode = (LinkNode)e.Node;
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

        //When a keyboard key is pressed on the tree
        void tree_KeyDown(object sender, KeyEventArgs e)
        {
            if (rightClickedNode.IsEditing)
            {
                if (e.KeyCode == Keys.Enter)
                {
                    rightClickedNode.EndEdit(false);
                }
                else if (e.KeyCode == Keys.Escape)
                {
                    rightClickedNode.EndEdit(true);
                }
            }
        }

        // The callback for the configuration page context menu 'Add Child' option
        void addChild_Click(object sender, EventArgs e)
        {
            createNewNodes(rightClickedNode, 1);
        }

        // The callback for the configuration page context menu 'Remove Child' option
        void removeChild_Click(object sender, EventArgs e)
        {
            LinkNode parent = (LinkNode)rightClickedNode.Parent;
            parent.Nodes.Remove(rightClickedNode);
        }

        // The callback for the configuration page context menu 'Rename Child' option
        // This isn't really working right now, so the option was deactivated from the context menu
        void renameChild_Click(object sender, EventArgs e)
        {
            tree.SelectedNode = rightClickedNode;
            tree.LabelEdit = true;
            rightClickedNode.BeginEdit();
            pm_Page.SetFocus(dotNet_tree);
        }

        private void tree_ItemDrag(object sender, System.Windows.Forms.ItemDragEventArgs e)
        {
            tree.DoDragDrop(e.Item, DragDropEffects.Move);

        }
        private void tree_DragOver(object sender, System.Windows.Forms.DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = tree.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            tree.SelectedNode = tree.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }
        private void tree_DragEnter(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = tree.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            tree.SelectedNode = tree.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }
        private void tree_DragDrop(object sender, System.Windows.Forms.DragEventArgs e)
        {
            // Retrieve the client coordinates of the drop location.
            Point targetPoint = tree.PointToClient(new Point(e.X, e.Y));

            // Retrieve the node at the drop location.
            LinkNode targetNode = (LinkNode)tree.GetNodeAt(targetPoint);

            LinkNode draggedNode;
            // Retrieve the node that was dragged.

            //Retrieve the node/item that was dragged
            if ((LinkNode)e.Data.GetData(typeof(LinkNode)) != null)
            {
                draggedNode = (LinkNode)e.Data.GetData(typeof(LinkNode));
            }
            else
            {
                return;
            }

            // If the node is picked up and dragged back on to itself, please don't crash
            if (draggedNode == targetNode)
            {
                return;
            }

            // If the it was dropped into the box itself, but not onto an actual node
            if (targetNode == null)
            {
                // If for some reason the tree is empty
                if (tree.Nodes.Count == 0)
                {
                    draggedNode.Remove();
                    tree.Nodes.Add(draggedNode);
                    tree.ExpandAll();
                    return;
                }
                else
                {
                    targetNode = (LinkNode)tree.TopNode;
                    draggedNode.Remove();
                    targetNode.Nodes.Add(draggedNode);
                    targetNode.ExpandAll();
                    return;
                }
            }
            else
            {
                // If dragging a node closer onto its ancestor do parent swapping kungfu
                if (draggedNode.Level < targetNode.Level)
                {
                    int level_diff = targetNode.Level - draggedNode.Level;
                    LinkNode ancestor_node = targetNode;
                    for (int i = 0; i < level_diff; i++)
                    {
                        //Ascend up the target's node (new parent) parent tree the level difference to get the ancestoral node that is at the same level
                        //as the dragged node (the new child)
                        ancestor_node = (LinkNode)ancestor_node.Parent;
                    }
                    // If the dragged node is in the same line as the target node, then the real kungfu begins
                    if (ancestor_node == draggedNode)
                    {
                        LinkNode newParent = targetNode;
                        LinkNode newChild = draggedNode;
                        LinkNode sameGrandparent = (LinkNode)draggedNode.Parent;
                        newParent.Remove(); // Remove the target node from the tree
                        newChild.Remove();  // Remove the dragged node from the tree
                        newParent.Nodes.Add(newChild); // 
                        if (sameGrandparent == null)
                        {
                            tree.Nodes.Add(newParent);
                        }
                        else
                        {
                            sameGrandparent.Nodes.Add(newParent);
                        }
                    }
                }
                draggedNode.Remove();
                targetNode.Nodes.Add(draggedNode);
                targetNode.ExpandAll();
            }
        }
        #endregion

        //A method that sets up the Property Manager Page
        private void setupPropertyManagerPage(ref string caption, ref string tip, ref long options, ref int controlType, ref int alignment)
        {
            //Begin adding the controls to the page
            //Create the group box
            caption = "Configure and Organize Links";
            options = (int)swAddGroupBoxOptions_e.swGroupBoxOptions_Visible + (int)swAddGroupBoxOptions_e.swGroupBoxOptions_Expanded;
            pm_Group = (PropertyManagerPageGroup)pm_Page.AddGroupBox(GroupID, caption, (int)options);

            //Create the parent link label (static)
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Parent Link";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_Label_ParentLinkLabel = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

            //Create the parent link name label, the one that is updated
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_Label_ParentLink = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

            //Create the link name text box label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Link Name";
            tip = "Enter the name of the link";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_Label_LinkName = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, tip);

            //Create the link name text box
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Textbox;
            caption = "base_link";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            tip = "Enter the name of the link";
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_TextBox_LinkName = (PropertyManagerPageTextbox)pm_Group.AddControl(TextBox_LinkNameID, (short)(controlType), caption, (short)alignment, (int)options, tip);


            //Create the joint name text box label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Joint Name";
            tip = "Enter the name of the joint";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_Label_JointName = (PropertyManagerPageLabel)pm_Group.AddControl(Label_JointName_ID, (short)controlType, caption, (short)alignment, (int)options, tip);

            //Create the joint name text box
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Textbox;
            caption = "";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            tip = "Enter the name of the joint";
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_TextBox_JointName = (PropertyManagerPageTextbox)pm_Group.AddControl(TextBox_LinkNameID, (short)(controlType), caption, (short)alignment, (int)options, tip);

            //Create the ref coordinate sys label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Reference Coordinate System";
            tip = "Select the reference coordinate system for the joint origin";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_Label_CoordSys = (PropertyManagerPageLabel)pm_Group.AddControl(Label_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, tip);


            // Create pull down menu for Coordinate systems
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
            caption = "Reference Coordinate System Name";
            tip = "Select the reference coordinate system for the joint origin";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_ComboBox_CoordSys = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, tip);
            pm_ComboBox_CoordSys.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;

            //Create the ref axis label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Reference Axis";
            tip = "Select the reference axis for the joint";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_Label_Axes = (PropertyManagerPageLabel)pm_Group.AddControl(Label_Axes_ID, (short)controlType, caption, (short)alignment, (int)options, tip);


            // Create pull down menu for axes
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
            caption = "Reference Axis Name";
            tip = "Select the reference axis for the joint";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_ComboBox_Axes = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, tip);
            pm_ComboBox_Axes.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;

            //Create the joint type label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Joint Type";
            tip = "Select the joint type";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_Label_JointType = (PropertyManagerPageLabel)pm_Group.AddControl(Label_Axes_ID, (short)controlType, caption, (short)alignment, (int)options, tip);


            // Create pull down menu for joint type
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
            caption = "Joint type";
            tip = "Select the joint type";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            options = (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_ComboBox_JointType = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, tip);
            pm_ComboBox_JointType.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;
            pm_ComboBox_JointType.AddItems(new string[] { "Automatically Detect", "continuous", "revolute", "prismatic", "fixed" });


            //Create the selection box label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Link Components";
            tip = "Select components associated with this link";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_Label_Selection = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, tip);

            //Create selection box
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Selectionbox;
            caption = "Link Components";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            tip = "Select components associated with this link";
            pm_Selection = (PropertyManagerPageSelectionbox)pm_Group.AddControl(SelectionID, (short)controlType, caption, (short)alignment, (int)options, tip);

            swSelectType_e[] filters = new swSelectType_e[1];
            filters[0] = swSelectType_e.swSelCOMPONENTS;
            object filterObj = null;
            filterObj = filters;

            pm_Selection.AllowSelectInMultipleBoxes = true;
            pm_Selection.SingleEntityOnly = false;
            pm_Selection.AllowMultipleSelectOfSameEntity = false;
            pm_Selection.Height = 50;
            pm_Selection.SetSelectionFilters(filterObj);

            //Create the number box label
            //Create the link name text box label
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
            caption = "Number of child links";
            tip = "Enter the number of child links and they will be automatically added";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_Label_ChildCount = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, tip);

            //Create the number box
            controlType = (int)swPropertyManagerPageControlType_e.swControlType_Numberbox;
            caption = "";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
            tip = "Enter the number of child links and they will be automatically added";
            options = (int)swAddControlOptions_e.swControlOptions_Enabled + (int)swAddControlOptions_e.swControlOptions_Visible;
            pm_NumberBox_ChildCount = pm_Group.AddControl(NumBox_ChildCount_ID, (short)controlType, caption, (short)alignment, (int)options, tip);
            pm_NumberBox_ChildCount.SetRange2((int)swNumberboxUnitType_e.swNumberBox_UnitlessInteger, 0, int.MaxValue, true, 1, 1, 1);
            pm_NumberBox_ChildCount.Value = 0;

            //pm_Button_save = pm_Group.AddControl(Button_save_ID, (short)swPropertyManagerPageControlType_e.swControlType_Button, "Build Link", 0, (int)options, "");
            pm_Button_export = pm_Group.AddControl(Button_export_ID, (short)swPropertyManagerPageControlType_e.swControlType_Button, "Preview and Export...", 0, (int)options, "");


            controlType = (int)swPropertyManagerPageControlType_e.swControlType_WindowFromHandle;
            caption = "Link Tree";
            alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
            options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
            pm_tree = pm_Page.AddControl(dotNet_tree, (short)swPropertyManagerPageControlType_e.swControlType_WindowFromHandle, caption, 0, (int)options, "");
            pm_tree.Height = 600;
            tree = new TreeView();
            tree.Height = 600;
            tree.Visible = true;
            tree.AfterSelect += new System.Windows.Forms.TreeViewEventHandler(tree_AfterSelect);
            tree.NodeMouseClick += new System.Windows.Forms.TreeNodeMouseClickEventHandler(tree_NodeMouseClick);
            tree.KeyDown += new System.Windows.Forms.KeyEventHandler(tree_KeyDown);
            tree.DragDrop += new DragEventHandler(tree_DragDrop);
            tree.DragOver += new DragEventHandler(tree_DragOver);
            tree.DragEnter += new DragEventHandler(tree_DragEnter);
            tree.ItemDrag += new ItemDragEventHandler(tree_ItemDrag);
            tree.AllowDrop = true;
            pm_tree.SetWindowHandlex64(tree.Handle.ToInt64());




            ToolStripMenuItem addChild = new ToolStripMenuItem();
            ToolStripMenuItem removeChild = new ToolStripMenuItem();
            //ToolStripMenuItem renameChild = new ToolStripMenuItem();
            addChild.Text = "Add Child Link";
            addChild.Click += new System.EventHandler(this.addChild_Click);

            removeChild.Text = "Remove";
            removeChild.Click += new System.EventHandler(this.removeChild_Click);
            //renameChild.Text = "Rename";
            //renameChild.Click += new System.EventHandler(this.renameChild_Click);
           //docMenu.Items.AddRange(new ToolStripMenuItem[] { addChild, removeChild, renameChild });
            docMenu.Items.AddRange(new ToolStripMenuItem[] { addChild, removeChild});
            LinkNode node = createEmptyNode(null);
            node.ContextMenuStrip = docMenu;
            tree.Nodes.Add(node);
            tree.SelectedNode = tree.Nodes[0];
            pm_Selection.SetSelectionFocus();
            pm_Page.SetFocus(dotNet_tree);
            //updateNodeNames(tree);
        }


        #region Not implemented handler methods
        // These methods are still active. The exceptions that are thrown only cause the debugger to pause. Comment out the exception
        // if you choose not to implement it, but it gets regularly called anyway
        void IPropertyManagerPage2Handler9.OnCheckboxCheck(int Id, bool Checked)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnComboboxEditChanged(int Id, string Text)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnComboboxSelectionChanged(int Id, int Item)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnGroupCheck(int Id, bool Checked)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnGroupExpand(int Id, bool Expanded)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnListboxSelectionChanged(int Id, int Item)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnNextPage()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnOptionCheck(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnPopupMenuItem(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnPopupMenuItemUpdate(int Id, ref int retval)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnPreview()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnPreviousPage()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnRedo()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnSelectionboxCalloutCreated(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnSelectionboxCalloutDestroyed(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }


        void IPropertyManagerPage2Handler9.OnSliderPositionChanged(int Id, double Value)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnSliderTrackingCompleted(int Id, double Value)
        {

            throw new Exception("The method or operation is not implemented.");

        }
        bool IPropertyManagerPage2Handler9.OnTabClicked(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }


        void IPropertyManagerPage2Handler9.OnUndo()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnWhatsNew()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnListboxRMBUp(int Id, int PosX, int PosY)
        {
            throw new Exception("The method or operation is not implemented.");
        }

        void IPropertyManagerPage2Handler9.OnNumberBoxTrackingCompleted(int Id, double Value)
        {
            //throw new Exception("The method or operation is not implemented.");
        }
        void IPropertyManagerPage2Handler9.AfterClose()
        {
            //throw new Exception("The method or operation is not implemented.");
        }

        int IPropertyManagerPage2Handler9.OnActiveXControlCreated(int Id, bool Status)
        {
            throw new Exception("The method or operation is not implemented.");
        }
        #endregion


    }

}