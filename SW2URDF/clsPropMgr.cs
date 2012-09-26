using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using System;
using System.Runtime.InteropServices;
using System.Diagnostics;
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
        PropertyManagerPageControl pm_Control;
        PropertyManagerPageGroup pm_Group;

        PropertyManagerPageSelectionbox pm_Selection;
        PropertyManagerPageButton pm_Button_save;
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

        public void Show()
        {
            pm_Page.Show2(0);
        }

        //The following runs when a new instance

        //of the class is created

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
            PageTitle = "Comps";
            options = (int)swPropertyManagerButtonTypes_e.swPropertyManager_OkayButton + (int)swPropertyManagerButtonTypes_e.swPropertyManager_CancelButton + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_LockedPage + (int)swPropertyManagerPageOptions_e.swPropertyManagerOptions_PushpinButton;

            //Create the PropertyManager page
            pm_Page = (PropertyManagerPage2)swApp.CreatePropertyManagerPage(PageTitle, (int)options, this, ref longerrors);

            //Make sure that the page was created properly
            if (longerrors == (int)swPropertyManagerPageStatus_e.swPropertyManagerPage_Okay)
            {
                //Begin adding the controls to the page
                //Create the group box
                caption = "URDF Exporter: Organize Links";
                options = (int)swAddGroupBoxOptions_e.swGroupBoxOptions_Visible + (int)swAddGroupBoxOptions_e.swGroupBoxOptions_Expanded;
                pm_Group = (PropertyManagerPageGroup)pm_Page.AddGroupBox(GroupID, caption, (int)options);

                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Parent Link";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_ParentLinkLabel = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create the link name text box label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_ParentLink = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create the link name text box label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Link Name";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_LinkName = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

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
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_Label_JointName = (PropertyManagerPageLabel)pm_Group.AddControl(Label_JointName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create the joint name text box
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Textbox;
                caption = "";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                tip = "Enter the name of the link";
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_TextBox_JointName = (PropertyManagerPageTextbox)pm_Group.AddControl(TextBox_LinkNameID, (short)(controlType), caption, (short)alignment, (int)options, tip);

                //Create the ref coordinate sys label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Reference Coordinate System";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_Label_CoordSys = (PropertyManagerPageLabel)pm_Group.AddControl(Label_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, "");


                // Create pull down menu for Coordinate systems
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
                caption = "Reference Coordinate System Name";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_ComboBox_CoordSys = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, "");
                pm_ComboBox_CoordSys.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;

                //Create the ref axis label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Reference Axis";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_Label_Axes = (PropertyManagerPageLabel)pm_Group.AddControl(Label_Axes_ID, (short)controlType, caption, (short)alignment, (int)options, "");


                // Create pull down menu for axes
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
                caption = "Reference Axis Name";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_ComboBox_Axes = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, "");
                pm_ComboBox_Axes.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;

                //Create the joint type label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Joint Type";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_Label_JointType = (PropertyManagerPageLabel)pm_Group.AddControl(Label_Axes_ID, (short)controlType, caption, (short)alignment, (int)options, "");


                // Create pull down menu for joint type
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Combobox;
                caption = "Joint type";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                options = (int)swAddControlOptions_e.swControlOptions_Visible;
                pm_ComboBox_JointType = (PropertyManagerPageCombobox)pm_Group.AddControl(ComboBox_CoordSys_ID, (short)controlType, caption, (short)alignment, (int)options, "");
                pm_ComboBox_JointType.Style = (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_EditBoxReadOnly + (int)swPropMgrPageComboBoxStyle_e.swPropMgrPageComboBoxStyle_Sorted;
                pm_ComboBox_JointType.AddItems(new string[] { "Automatically Detect", "continuous", "revolute", "prismatic", "fixed" });


                //Create the selection box label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Selected Link Components";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_Selection = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create selection box
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Selectionbox;
                caption = "Link Components";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                tip = "Select components to group into a single link";
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
                caption = "Number of child links to create";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_ChildCount = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create the number box
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Numberbox;
                caption = "";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_Indent;
                tip = "Enter the number of child links that will be created";
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
                pm_tree.SetWindowHandlex64(tree.Handle.ToInt64());




                ToolStripMenuItem addChild = new ToolStripMenuItem();
                ToolStripMenuItem removeChild = new ToolStripMenuItem();
                ToolStripMenuItem renameChild = new ToolStripMenuItem();
                addChild.Text = "Add Child Link";
                addChild.Click += new System.EventHandler(this.addChild_Click);

                removeChild.Text = "Remove";
                removeChild.Click += new System.EventHandler(this.removeChild_Click);
                renameChild.Text = "Rename";
                renameChild.Click += new System.EventHandler(this.renameChild_Click);
                docMenu.Items.AddRange(new ToolStripMenuItem[] { addChild, removeChild, renameChild });

                LinkNode node = createEmptyNode(null);
                node.ContextMenuStrip = docMenu;
                tree.Nodes.Add(node);
                tree.SelectedNode = tree.Nodes[0];
                pm_Selection.SetSelectionFocus();
                updateNodeNames(tree);
            }

            else
            {
                //If the page is not created
                System.Windows.Forms.MessageBox.Show("An error occurred while attempting to create the " + "PropertyManager Page");
            }

            #endregion
        }

        #region IPropertyManagerPage2Handler9 Members
        void addChild_Click(object sender, EventArgs e)
        {
            createNewNodes(rightClickedNode, 1);
        }
        void removeChild_Click(object sender, EventArgs e)
        {
            LinkNode parent = (LinkNode)rightClickedNode.Parent;
            parent.Nodes.Remove(rightClickedNode);
        }

        void renameChild_Click(object sender, EventArgs e)
        {
            rightClickedNode.BeginEdit();
        }

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
        void IPropertyManagerPage2Handler9.AfterActivation()
        {
            pm_Selection.SetSelectionFocus();
            //throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.AfterClose()
        {

        }

        int IPropertyManagerPage2Handler9.OnActiveXControlCreated(int Id, bool Status)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnButtonPress(int Id)
        {
            if (Id == Button_export_ID)
            {
                saveActiveNode();
                if (Exporter.checkIfNamesAreUnique((LinkNode)tree.Nodes[0]))
                {
                    pm_Page.Close(true); //It saves automatically when sending Okay as true;
                    Exporter.createRobotFromTreeView(tree);
                    AssemblyExportForm exportForm = new AssemblyExportForm(swApp);
                    exportForm.Exporter = Exporter;
                    exportForm.fillLinkTreeFromRobot(Exporter.mRobot);
                    exportForm.Show();
                }
            }

        }

        private void updatePM()
        {
            LinkNode nextNode = findNextLinkToVisit(tree);
            if (nextNode != null)
            {
                switchActiveNodes(nextNode);
                ActiveSWModel.ClearSelection2(true);
                setComponentFilters();
            }

        }

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

        private void fillComboBox(PropertyManagerPageCombobox box, List<Feature> features)
        {
            box.Clear();
            box.AddItems("Automatically Generate");
            foreach (Feature feat in features)
            {
                box.AddItems(feat.Name);
            }
        }

        private void selectComboBox(PropertyManagerPageCombobox box, string item)
        {
            short i = 0;
            string itemtext = "nothing";
            box.CurrentSelection = 0;
            while (itemtext != null && itemtext != "" && itemtext != item)
            {
                itemtext = box.get_ItemText(i);
                if (itemtext == item)
                {
                    box.CurrentSelection = i;
                }
                i++;
            }
        }
        private void updateNodeNames(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                updateNodeNames(node);
            }
        }

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
        private void createNewLinks(LinkNode CurrentlySelectedNode)
        {
            int linksToBuild = (int)pm_NumberBox_ChildCount.Value - CurrentlySelectedNode.Nodes.Count;
            createNewNodes(CurrentlySelectedNode, linksToBuild);
        }

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

        public void switchActiveNodes(LinkNode node)
        {
            saveActiveNode();
            fillPropertyManager(node);
            automaticallySwitched = true;
            tree.SelectedNode = node;
            previouslySelectedNode = node;
            checkNodeComplete(node);
            updateNodeNames(tree);
        }

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
            foreach (LinkNode nodeToCheck in tree.Nodes)
            {
                LinkNode nodeToReturn = findNextLinkToVisit(nodeToCheck);
                if (nodeToReturn != null)
                {
                    return nodeToReturn;
                }
            }

            //Otherwise we're done
            return null;

        }
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

        public void checkNodeComplete(LinkNode node)
        {
            node.isIncomplete = (node.linkName.Equals("Empty_Link") ||
                                 node.linkName.Equals("") ||
                                 node.components.Count == 0 ||
                                 (node.jointName == "" && !node.isBaseNode));
        }
        public void saveActiveLink()
        {
            if (previouslySelectedLink != null)
            {
                previouslySelectedLink.name = pm_TextBox_LinkName.Text;
                previouslySelectedLink.uniqueName = pm_TextBox_LinkName.Text;

                pm_Control = (PropertyManagerPageControl)pm_TextBox_JointName;
                if (pm_Control.Enabled)
                {
                    previouslySelectedLink.Joint.name = pm_TextBox_JointName.Text;
                }
                SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
                previouslySelectedLink.SWcomponents.Clear();
                for (int i = 0; i < selectionManager.GetSelectedObjectCount2(pm_Selection.Mark); i++)
                {
                    object obj = selectionManager.GetSelectedObject6(i + 1, pm_Selection.Mark);
                    Component2 comp = (Component2)obj;
                    if (comp != null)
                    {
                        previouslySelectedLink.SWcomponents.Add(comp);
                    }
                }
                if (previouslySelectedLink.Joint != null)
                {
                    // -1 is to get the currently selected text

                    previouslySelectedLink.Joint.AxisName = pm_ComboBox_Axes.get_ItemText(-1);
                    previouslySelectedLink.Joint.CoordinateSystemName = pm_ComboBox_CoordSys.get_ItemText(-1);
                }
            }
        }
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
                SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
                previouslySelectedNode.components.Clear();
                for (int i = 0; i < selectionManager.GetSelectedObjectCount2(pm_Selection.Mark); i++)
                {
                    object obj = selectionManager.GetSelectedObject6(i + 1, pm_Selection.Mark);
                    Component2 comp = (Component2)obj;
                    if (comp != null)
                    {
                        previouslySelectedNode.components.Add(comp);
                    }
                }
            }

        }

        public LinkNode createEmptyNode(LinkNode Parent)
        {
            LinkNode node = new LinkNode();
            if (Parent == null)
            {
                node.linkName = "base_link";
                node.axisName = "";
                node.coordsysName = "Automatically Generate";
                node.components = new List<Component2>();
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
                node.components = new List<Component2>();
                node.isBaseNode = false;
                node.isIncomplete = true;
            }
            node.Name = node.linkName;
            node.Text = node.linkName;
            node.ContextMenuStrip = docMenu;
            return node;
        }

        public void fillPropertyManager(LinkNode node)
        {
            pm_TextBox_LinkName.Text = node.linkName;
            pm_NumberBox_ChildCount.Value = node.Nodes.Count;
            ActiveSWModel.ClearSelection2(true);
            pm_Selection.SetSelectionFocus();
            foreach (Component2 component in node.components)
            {
                SelectData data = default(SelectData);
                SelectionMgr manager = ActiveSWModel.SelectionManager;
                data = manager.CreateSelectData();
                data.Mark = pm_Selection.Mark;

                component.Select4(true, data, false);
            }

            if (!node.isBaseNode && node.Parent != null)
            {
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
                pm_Label_ParentLink.Caption = " ";
                selectComboBox(pm_ComboBox_CoordSys, "");
                selectComboBox(pm_ComboBox_Axes, "");
                selectComboBox(pm_ComboBox_JointType, "");
                enableJointControls(!node.isBaseNode);
            }
        }

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

        void setComponentFilters()
        {
            swSelectType_e[] filters = new swSelectType_e[1];
            filters[0] = swSelectType_e.swSelCOMPONENTS;
            object filterObj = null;
            filterObj = filters;
            pm_Selection.SetSelectionFilters(filterObj);
        }

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

        void IPropertyManagerPage2Handler9.OnCheckboxCheck(int Id, bool Checked)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnClose(int Reason)
        {

            if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Cancel)
            {

                if (MessageBox.Show("Do you want to save your changes to the export configuration?", "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes)
                {
                    saveActiveNode();
                    Exporter.saveConfigTree(tree);
                }

            }

            else if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Okay)
            {
                saveActiveNode();
                Exporter.saveConfigTree(tree);
            }

        }



        void IPropertyManagerPage2Handler9.OnComboboxEditChanged(int Id, string Text)
        {

            //throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnComboboxSelectionChanged(int Id, int Item)
        {

            //throw new Exception("The method or operation is not implemented.");

        }



        void IPropertyManagerPage2Handler9.OnGainedFocus(int Id)
        {

            //short[] varArray = null;

            //Debug.Print("Control box " + Id + " has gained focus");

            //varArray = (short[])pm_List.GetSelectedItems();

            //pm_Combo.CurrentSelection = varArray[0];

        }

        void IPropertyManagerPage2Handler9.OnGroupCheck(int Id, bool Checked)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnGroupExpand(int Id, bool Expanded)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnHelp()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnKeystroke(int Wparam, int Message, int Lparam, int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnListboxSelectionChanged(int Id, int Item)
        {

            //throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnLostFocus(int Id)
        {

            Debug.Print("Control box " + Id + " has lost focus");

        }

        bool IPropertyManagerPage2Handler9.OnNextPage()
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnNumberboxChanged(int Id, double Value)
        {
            if (Id == NumBox_ChildCount_ID)
            {
                LinkNode node = (LinkNode)tree.SelectedNode;
                createNewLinks(node);
                updateNodeNames((LinkNode)tree.Nodes[0]);
            }
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

        void IPropertyManagerPage2Handler9.OnSelectionboxFocusChanged(int Id)
        {

            Debug.Print("The focus has moved to selection box " + Id);

        }

        void IPropertyManagerPage2Handler9.OnSelectionboxListChanged(int Id, int Count)
        {

            // Move focus to next selection box if right-mouse button pressed

            pm_Page.SetCursor((int)swPropertyManagerPageCursors_e.swPropertyManagerPageCursors_Advance);

            Debug.Print("The list in selection box " + Id + " has changed");

        }



        void IPropertyManagerPage2Handler9.OnSliderPositionChanged(int Id, double Value)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnSliderTrackingCompleted(int Id, double Value)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        bool IPropertyManagerPage2Handler9.OnSubmitSelection(int Id, object Selection, int SelType, ref string ItemText)
        {

            // This method must return true for selections to occur

            return true;

        }

        bool IPropertyManagerPage2Handler9.OnTabClicked(int Id)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnTextboxChanged(int Id, string Text)
        {
            if (Id == TextBox_LinkNameID)
            {
                LinkNode node = (LinkNode)tree.SelectedNode;
                node.Text = pm_TextBox_LinkName.Text;
            }
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

        int IPropertyManagerPage2Handler9.OnWindowFromHandleControlCreated(int Id, bool Status)
        {
            //throw new Exception("The method or operation is not implemented.");
            return 0;
        }

        void IPropertyManagerPage2Handler9.OnNumberBoxTrackingCompleted(int Id, double Value)
        {
            //throw new Exception("The method or operation is not implemented.");
        }

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

        private void tree_AfterSelect(object sender, TreeViewEventArgs e)
        {
            if (!automaticallySwitched && e.Node != null)
            {
                switchActiveNodes((LinkNode)e.Node);
            }
            automaticallySwitched = false;
        }

        private void tree_NodeMouseClick(object sender, TreeNodeMouseClickEventArgs e)
        {
            rightClickedNode = (LinkNode)e.Node;
        }

        private bool doesLinkNeedRebuildingFromComponents(link Link)
        {
            SelectionMgr manager = ActiveSWModel.SelectionManager;
            int count = manager.GetSelectedObjectCount2(pm_Selection.Mark);
            List<Component2> components = new List<Component2>();
            for (int i = 0; i < count; i++)
            {
                object obj = manager.GetSelectedObject6(i + 1, pm_Selection.Mark);
                Component2 comp = (Component2)obj;
                if (comp != null)
                {
                    components.Add(comp);
                }
            }

            foreach (Component2 comp in components)
            {
                if (!Link.SWcomponents.Contains(comp))
                {
                    return true;
                }

            }
            if (Link.SWcomponents.Count == 0)
            {
                return true;
            }
            return false;
        }

        public void loadConfigTree()
        {

            LinkNode basenode = Exporter.loadConfigTree();
            if (basenode != null)
            {
                tree.Nodes.Clear();
                tree.Nodes.Add(basenode);
                tree.ExpandAll();
                tree.SelectedNode = tree.Nodes[0];
            }
        }



        #endregion



    }

}