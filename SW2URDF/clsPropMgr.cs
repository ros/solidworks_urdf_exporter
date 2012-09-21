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
        public link previouslySelectedLink;
        public List<link> linksToVisit;
        public LinkNode rightClickedNode;
        private ContextMenuStrip docMenu;
        //General objects required for the PropertyManager page

        PropertyManagerPage2 pm_Page;
        PropertyManagerPageControl pm_Control;
        PropertyManagerPageGroup pm_Group;

        PropertyManagerPageSelectionbox pm_Selection;
        PropertyManagerPageButton pm_Button;
        PropertyManagerPageTextbox pm_TextBox_LinkName;
        PropertyManagerPageTextbox pm_TextBox_JointName;
        PropertyManagerPageNumberbox pm_NumberBox_ChildCount;

        PropertyManagerPageLabel pm_Label_LinkName;
        PropertyManagerPageLabel pm_Label_JointName;
        PropertyManagerPageLabel pm_Label_Selection;
        PropertyManagerPageLabel pm_Label_ChildCount;
        PropertyManagerPageLabel pm_Label_ParentLink;
        PropertyManagerPageLabel pm_Label_ParentLinkLabel;

        PropertyManagerPageWindowFromHandle pm_tree;
        TreeView tree;
        bool automaticallySwitched = false;

        //Each object in the page needs a unique ID

        const int GroupID = 1;
        const int TextBox_LinkNameID = 2;
        const int SelectionID = 3;
        const int ComboID = 4;
        const int ListID = 5;
        const int ButtonID = 6;
        const int NumBox_ChildCount_ID = 7;
        const int Label_LinkName_ID = 8;
        const int Label_Selection_ID = 9;
        const int Label_ChildCount_ID = 10;
        const int Label_ParentLink_ID = 11;
        const int Label_ParentLinkLabel_ID = 12;
        const int TextBox_JointNameID = 13;
        const int Label_JointName_ID = 14;
        const int dotNet_tree = 16;

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



                //Create the selection box label
                controlType = (int)swPropertyManagerPageControlType_e.swControlType_Label;
                caption = "Selected Link Components";
                alignment = (int)swPropertyManagerPageControlLeftAlign_e.swControlAlign_LeftEdge;
                options = (int)swAddControlOptions_e.swControlOptions_Visible + (int)swAddControlOptions_e.swControlOptions_Enabled;
                pm_Label_Selection = (PropertyManagerPageLabel)pm_Group.AddControl(Label_LinkName_ID, (short)controlType, caption, (short)alignment, (int)options, "");

                //Create selection boxe
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
                
                pm_Button = pm_Group.AddControl(ButtonID, (short)swPropertyManagerPageControlType_e.swControlType_Button, "Create Link", 0, (int)options, "");


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

                LinkNode node = new LinkNode();
                node.Link = new link();
                node.Link.name = "base_link";
                node.Link.uniqueName = "base_link";
                node.Text = node.Link.name;
                node.Name = node.Link.name;

                
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
                node.ContextMenuStrip = docMenu;
                
                tree.Nodes.Add(node);
                tree.SelectedNode = tree.Nodes[0];
                pm_Selection.SetSelectionFocus();

                
                //
            }

            else
            {
                //If the page is not created
                System.Windows.Forms.MessageBox.Show("An error occurred while attempting to create the " + "PropertyManager Page");
            }
        }

        #region IPropertyManagerPage2Handler9 Members
        void addChild_Click(object sender, EventArgs e)
        {
            createNewLinks(rightClickedNode, 1);
        }
        void removeChild_Click(object sender, EventArgs e)
        {
            createNewLinks(rightClickedNode, -1);
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

            //throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.AfterClose()
        {
            Exporter.saveExporter();
            StringWriter stringWriter;
            XmlSerializer serializer = new XmlSerializer(typeof(SW2URDFExporter));
            stringWriter = new StringWriter();
            serializer.Serialize(stringWriter, Exporter);
            stringWriter.Flush();
            stringWriter.Close();

            int Options = 0;
            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = Exporter.saveConfigurationAttributeDef.CreateInstance5(ActiveSWModel, null, "URDF Export Configuration", Options, ConfigurationOptions);
            Parameter param = saveExporterAttribute.GetParameter("data");
            param.SetStringValue2(stringWriter.ToString(), ConfigurationOptions, "");
            param = saveExporterAttribute.GetParameter("name");
            param.SetStringValue2("config1", ConfigurationOptions, "");
            param = saveExporterAttribute.GetParameter("date");
            param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
        }

        int IPropertyManagerPage2Handler9.OnActiveXControlCreated(int Id, bool Status)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnButtonPress(int Id)

        {
            if (Id == ButtonID)
            {
                if (pm_Selection.ItemCount > 0 ||
                    MessageBox.Show("No components have been selected, do you want to create an empty link?", "Create empty link?", MessageBoxButtons.YesNo) == DialogResult.Yes)
                {
                    setGeneralFilters();
                    createNewLinks(buildNode());
                    updatePM();
                }
            }
            if (Id == NumBox_ChildCount_ID)
            {
                int c = 1;
            }
        }

        private void updatePM()
        {
            LinkNode nextNode = findNextLinkToVisit(tree);
            if (nextNode != null)
            {
                switchActiveLinks(nextNode);
                ActiveSWModel.ClearSelection2(true);
                setComponentFilters();
            }
            else
            {
                Exporter.mRobot = createRobotFromTreeView();
                AssemblyExportForm export = new AssemblyExportForm(swApp);
                export.Exporter = Exporter;
                export.fillLinkTreeFromRobot(Exporter.mRobot);
                export.Show();
                pm_Page.Close(true);
            }
        }

        private void createNewLinks(LinkNode CurrentlySelectedNode)
        {
            int linksToBuild = (int)pm_NumberBox_ChildCount.Value - CurrentlySelectedNode.Nodes.Count;
            createNewLinks(CurrentlySelectedNode, linksToBuild);
        }

        private void createNewLinks(LinkNode currentNode, int number)
        {
            for (int i = 0; i < number; i++)
            {
                LinkNode node = createEmptyNode(currentNode.Link);
                currentNode.Nodes.Add(node);
            }
            for (int i = 0; i < -number; i++)
            {
                currentNode.Link.Children.RemoveAt(currentNode.Link.Children.Count - 1);
            }
        }

        private LinkNode buildNode()
        {

            LinkNode CurrentlySelectedNode = (LinkNode)tree.SelectedNode;

            
            SelectionMgr manager = ActiveSWModel.SelectionManager;
            int count = manager.GetSelectedObjectCount2(pm_Selection.Mark);
            List<Component2> components = new List<Component2>();
            for (int i = 0; i < count; i++)
            {
                object obj = manager.GetSelectedObject6(i+1, pm_Selection.Mark);
                Component2 comp = (Component2)obj;
                if (comp != null)
                {
                    components.Add(comp);
                }
            }
            LinkNode parentNode = (LinkNode)CurrentlySelectedNode.Parent;
            if (parentNode == null)
            {
                Exporter.createBaseLinkFromComponents(components, pm_TextBox_LinkName.Text);
                CurrentlySelectedNode.Link = Exporter.mRobot.BaseLink;
            }
            else
            {
                CurrentlySelectedNode.Link = Exporter.createLinkFromComponents(parentNode.Link, components, pm_TextBox_LinkName.Text, pm_TextBox_JointName.Text);
            }
            CurrentlySelectedNode.Text = CurrentlySelectedNode.Link.name;
            CurrentlySelectedNode.Name = CurrentlySelectedNode.Link.name;
            return CurrentlySelectedNode;
        }

        public void switchActiveLinks(LinkNode node)
        {
            saveActiveLink();
            fillPropertyManager(node);
            automaticallySwitched = true;
            tree.SelectedNode = node;
            previouslySelectedLink = node.Link;
        }

        public LinkNode findNextLinkToVisit(System.Windows.Forms.TreeView tree)
        {
            if (tree.SelectedNode != null)
            {
                foreach (LinkNode node in tree.SelectedNode.Nodes)
                {
                    if (!isLinkComplete(node))
                    {
                        return node;
                    }
                }
            }
            return findNextLinkToVisit((LinkNode)tree.Nodes[0]);
        }
        public LinkNode findNextLinkToVisit(LinkNode nodeToCheck)
        {
            if (!isLinkComplete(nodeToCheck))
            {
                return nodeToCheck;
            }
            foreach (LinkNode node in nodeToCheck.Nodes)
            {
                findNextLinkToVisit(node);
            }
            return null;
        }

        public bool isLinkComplete(LinkNode node)
        {
            return !(node.Link.Equals("Empty_Link") || node.Link.SWcomponents.Count == 0 || (node.Link.Joint != null && node.Link.Joint.Equals("")));
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
                    object obj = selectionManager.GetSelectedObject6(i+1, pm_Selection.Mark);
                    Component2 comp = (Component2)obj;
                    if (comp != null)
                    {
                        previouslySelectedLink.SWcomponents.Add(comp);
                    }
                }
            }
        }

        public LinkNode createEmptyNode(link Parent)
        {
            LinkNode node = new LinkNode();
            link Link = new link();
            Link.name = "Empty_Link";
            Link.uniqueName = "Empty_Link";
            Link.Joint = new joint();
            Link.Joint.Parent.name = Parent.uniqueName;

            node.Name = Link.name;
            node.Text = Link.name;
            node.Link = Link;

            node.ContextMenuStrip = docMenu;
            return node;
        }

        public void fillPropertyManager(LinkNode node)
        {         
            pm_TextBox_LinkName.Text = node.Link.uniqueName;
            pm_NumberBox_ChildCount.Value = node.Nodes.Count;
            ActiveSWModel.ClearSelection2(true);
            pm_Selection.SetSelectionFocus();
            foreach (Component2 component in node.Link.SWcomponents)
            {
                SelectData data = default(SelectData);
                SelectionMgr manager = ActiveSWModel.SelectionManager;
                data = manager.CreateSelectData();
                data.Mark = pm_Selection.Mark;
                
                component.Select4(true, data, false);
            }

            pm_Control = (PropertyManagerPageControl)pm_TextBox_JointName;
            pm_Control.Enabled = (node.Link.Joint != null);

            pm_Control = (PropertyManagerPageControl)pm_Label_JointName;
            pm_Control.Enabled = (node.Link.Joint != null);

            if (node.Link.Joint != null)
            {
                pm_TextBox_JointName.Text = node.Link.Joint.name;
                pm_Label_ParentLink.Caption = node.Link.Joint.Parent.name;
            }
            else
            {
                pm_Label_ParentLink.Caption = " ";
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

                //Do something when the cancel button is clicked

            }

            else if (Reason == (int)swPropertyManagerPageCloseReasons_e.swPropertyManagerPageClose_Okay)
            {

                //Do something else when the OK button is clicked

            }

        }

        void IPropertyManagerPage2Handler9.OnComboboxEditChanged(int Id, string Text)
        {

            throw new Exception("The method or operation is not implemented.");

        }

        void IPropertyManagerPage2Handler9.OnComboboxSelectionChanged(int Id, int Item)
        {

            throw new Exception("The method or operation is not implemented.");

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
            LinkNode node = (LinkNode)tree.SelectedNode;
            createNewLinks(node);
            
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

            //throw new Exception("The method or operation is not implemented.");

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

            foreach (LinkNode node in tree.Nodes)
            {
                if (node.Level == 0)
                {

                    link BaseLink = createLinkFromLinkNode(node);
                    Robot.BaseLink = BaseLink;
                }
            }
            Robot.name = Exporter.mRobot.name;
            return Robot;
        }

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

        private void tree_AfterSelect(object sender, TreeViewEventArgs e)
        {
            if (!automaticallySwitched && e.Node != null)
            {
                switchActiveLinks((LinkNode)e.Node);
            }
            automaticallySwitched = false;
        }

        private void tree_NodeMouseClick(object sender, TreeNodeMouseClickEventArgs e)
        {
            rightClickedNode = (LinkNode)e.Node;
        }


        #endregion



    }

}