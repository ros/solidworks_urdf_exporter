using log4net;
using SW2URDF.URDF;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace SW2URDF
{
    public partial class TreeMerge : Form
    {
        private static readonly ILog logger = Logger.GetLogger();
        private static string KEY_NAME = "name";
        private static string KEY_JOINT_NAME = "joint_name";
        private static string KEY_COORDINATES = "coordinates";
        private static string KEY_AXIS_NAME = "axis";
        private static string KEY_TYPE_NAME = "type";

        private LinkNode previouslySelectedExisting;
        private LinkNode previouslySelectedLoaded;

        private readonly Dictionary<string, Control> ExistingEditingControls;
        private readonly Dictionary<string, Control> LoadedEditingControls;

        private readonly Dictionary<LinkNode, LinkNode> ExistingToLoaded;
        private readonly Dictionary<LinkNode, LinkNode> LoadedToExisting;

        public Link MergedLink { get; private set; }
        public TreeMergedEventHandler MergeCompleted;

        public TreeMerge()
        {
            InitializeComponent();

            ExistingEditingControls = new Dictionary<string, Control>()
            {
                { KEY_NAME, LinkNameExistingTextBox},
                { KEY_JOINT_NAME, JointNameExistingTextBox},
                { KEY_COORDINATES, CoordinatesExistingComboBox},
                { KEY_AXIS_NAME, AxisExistingComboBox},
                { KEY_TYPE_NAME, TypeExistingComboBox}
            };

            LoadedEditingControls = new Dictionary<string, Control>()
            {
                { KEY_NAME, LinkNameLoadedTextBox},
                { KEY_JOINT_NAME, JointNameLoadedField},
                { KEY_COORDINATES, CoordinatesLoadedField},
                { KEY_AXIS_NAME, AxisLoadedField},
                { KEY_TYPE_NAME, TypeLoadedField}
            };

            ExistingToLoaded = new Dictionary<LinkNode, LinkNode>();
            LoadedToExisting = new Dictionary<LinkNode, LinkNode>();
        }

        private void TreeMerge_Load(object sender, EventArgs e)
        {
        }

        private void BuildCorrespondances()
        {
            ExistingTreeView.Sort();
            LoadedTreeView.Sort();

            ExistingToLoaded.Clear();
            LoadedToExisting.Clear();
            AddNodesToLookups((LinkNode)ExistingTreeView.Nodes[0], (LinkNode)LoadedTreeView.Nodes[0]);
        }

        private void AddNodesToLookups(LinkNode existing, LinkNode loaded)
        {
            ExistingToLoaded[existing] = loaded;
            LoadedToExisting[loaded] = existing;

            for (int i = 0, j = 0; i < existing.Nodes.Count && j < loaded.Nodes.Count; i++, j++)
            {
                int compareResult = string.Compare(existing.Nodes[i].Name, loaded.Nodes[i].Name);
                if (compareResult < 0)
                {
                    // Existing's name is before Loaded in sorted order. On the next go around
                    // let i advance but not j
                    j--;
                }
                else if (compareResult == 0)
                {
                    AddNodesToLookups((LinkNode)existing.Nodes[i], (LinkNode)loaded.Nodes[j]);
                }
                else
                {
                    // Loaded is before existing's in sorted order. On the next go around
                    // let j advance but not i
                    i--;
                }
            }
        }

        public void SetTrees(LinkNode existingNode, LinkNode loadedNode)
        {
            ExistingTreeView.Nodes.Clear();
            LoadedTreeView.Nodes.Clear();

            ExistingTreeView.Nodes.Add(existingNode);
            LoadedTreeView.Nodes.Add(loadedNode);

            ExistingTreeView.ExpandAll();
            ExistingTreeView.SelectedNode = existingNode;
            FillExistingControls(existingNode);

            LoadedTreeView.ExpandAll();
            LoadedTreeView.SelectedNode = loadedNode;
            FillLoadedControls(loadedNode);

            previouslySelectedExisting = existingNode;
            previouslySelectedLoaded = loadedNode;
        }

        public void FillPullDowns(List<string> refCoordinates, List<string> refAxes)
        {
            CoordinatesExistingComboBox.Items.AddRange(refCoordinates.ToArray());
            AxisExistingComboBox.Items.AddRange(refAxes.ToArray());
            TypeExistingComboBox.Items.AddRange(new string[]
            {
                "continuous",
                "fixed",
                "floating",
                "planar",
                "prismatic",
                "revolute"
            });
        }

        private void SaveNode(LinkNode node, Dictionary<string, Control> controls)
        {
            node.Name = controls[KEY_NAME].Text;
            node.Text = node.Name;
            node.Link.Name = node.Name;

            if (!node.IsBaseNode)
            {
                node.Link.Joint.Name = controls[KEY_JOINT_NAME].Text;
                node.Link.Joint.CoordinateSystemName = controls[KEY_COORDINATES].Text;
                node.Link.Joint.AxisName = controls[KEY_AXIS_NAME].Text;
                node.Link.Joint.Type = controls[KEY_TYPE_NAME].Text;
            }
        }

        private void FillExistingControls(LinkNode node)
        {
            if (node == null)
            {
                return;
            }
            ExistingEditingControls[KEY_NAME].Text = node.Link.Name;

            bool showJointControls = !node.IsBaseNode;
            ExistingEditingControls[KEY_JOINT_NAME].Visible = showJointControls;
            ExistingEditingControls[KEY_COORDINATES].Visible = showJointControls;
            ExistingEditingControls[KEY_AXIS_NAME].Visible = showJointControls;
            ExistingEditingControls[KEY_TYPE_NAME].Visible = showJointControls;

            if (!node.IsBaseNode)
            {
                ComboBox coordinatesComboBox = (ComboBox)ExistingEditingControls[KEY_COORDINATES];
                ComboBox axisComboBox = (ComboBox)ExistingEditingControls[KEY_AXIS_NAME];
                ComboBox typeComboBox = (ComboBox)ExistingEditingControls[KEY_TYPE_NAME];

                coordinatesComboBox.Visible = true;
                axisComboBox.Visible = true;
                typeComboBox.Visible = true;

                ExistingEditingControls[KEY_JOINT_NAME].Text = node.Link.Joint.Name;
                coordinatesComboBox.SelectedIndex = coordinatesComboBox.FindStringExact(node.Link.Joint.CoordinateSystemName);
                axisComboBox.SelectedIndex = axisComboBox.FindStringExact(node.Link.Joint.AxisName);
                typeComboBox.SelectedIndex = typeComboBox.FindStringExact(node.Link.Joint.Type);
            }
        }

        private void FillLoadedControls(LinkNode node)
        {
            if (node == null)
            {
                return;
            }
            LoadedEditingControls[KEY_NAME].Text = node.Link.Name;

            bool showJointControls = !node.IsBaseNode;
            LoadedEditingControls[KEY_JOINT_NAME].Visible = showJointControls;
            LoadedEditingControls[KEY_COORDINATES].Visible = showJointControls;
            LoadedEditingControls[KEY_AXIS_NAME].Visible = showJointControls;
            LoadedEditingControls[KEY_TYPE_NAME].Visible = showJointControls;

            if (!node.IsBaseNode)
            {
                LoadedEditingControls[KEY_JOINT_NAME].Text = node.Link.Joint.Name;
                LoadedEditingControls[KEY_COORDINATES].Text = node.Link.Joint.CoordinateSystemName;
                LoadedEditingControls[KEY_AXIS_NAME].Text = node.Link.Joint.AxisName;
                LoadedEditingControls[KEY_TYPE_NAME].Text = node.Link.Joint.Type;
            }
        }

        private LinkNode GetCorrespondingLinkNode(LinkNode node)
        {
            if (ExistingToLoaded.ContainsKey(node) && LoadedToExisting.ContainsKey(node))
            {
                throw new Exception("This will not work");
            }
            if (ExistingToLoaded.ContainsKey(node))
            {
                return ExistingToLoaded[node];
            }
            if (LoadedToExisting.ContainsKey(node))
            {
                return LoadedToExisting[node];
            }
            return null;
        }

        private void TreeViewExistingAfterSelect(object sender, TreeViewEventArgs e)
        {
            // Don't process if selection is chosen automatically (stops infinite loop)
            if (e.Action == TreeViewAction.ByKeyboard || e.Action == TreeViewAction.ByMouse)
            {
                LinkNode existing = (LinkNode)e.Node;

                SaveNode(previouslySelectedExisting, ExistingEditingControls);
                FillExistingControls(existing);
                previouslySelectedExisting = existing;

                Font fontRegular = new Font(ExistingTreeView.Font, FontStyle.Regular);
                Font fontBold = new Font(ExistingTreeView.Font, FontStyle.Bold);

                if (previouslySelectedExisting != null)
                {
                    previouslySelectedExisting.NodeFont = fontRegular;
                }
                existing.NodeFont = fontBold;
            }
        }

        private void TreeViewLoadedAfterSelect(object sender, TreeViewEventArgs e)
        {
            // Don't process if selection is chosen automatically (stops infinite loop)
            if (e.Action == TreeViewAction.ByKeyboard || e.Action == TreeViewAction.ByMouse)
            {
                LinkNode loaded = (LinkNode)e.Node;
                SaveNode(previouslySelectedLoaded, LoadedEditingControls);
                FillLoadedControls(loaded);
                previouslySelectedLoaded = loaded;

                Font fontRegular = new Font(ExistingTreeView.Font, FontStyle.Regular);
                Font fontBold = new Font(ExistingTreeView.Font, FontStyle.Bold);

                if (previouslySelectedLoaded != null)
                {
                    previouslySelectedLoaded.NodeFont = fontRegular;
                }
                loaded.NodeFont = fontBold;
            }
        }

        private void ResetExistingButtonClick(object sender, EventArgs e)
        {
            FillExistingControls((LinkNode)ExistingTreeView.SelectedNode);
        }

        private void UpdateExistingButtonClick(object sender, EventArgs e)
        {
            SaveNode((LinkNode)ExistingTreeView.SelectedNode, ExistingEditingControls);
        }

        private void ResetLoadedButtonClick(object sender, EventArgs e)
        {
            FillLoadedControls((LinkNode)LoadedTreeView.SelectedNode);
        }

        private void UpdateLoadedButtonClick(object sender, EventArgs e)
        {
            SaveNode((LinkNode)LoadedTreeView.SelectedNode, LoadedEditingControls);
        }

        private void CancelMergeButtonClick(object sender, EventArgs e)
        {
            if (MessageBox.Show("You will lose any changes you have made on this form, would you like to continue?",
                "Cancel process?", MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                Close();
            }
        }

        private void UpdateLinkFromLinkNode(Link link, LinkNode linkNode)
        {
            link.Children.Clear();
            foreach (LinkNode child in linkNode.Nodes)
            {
                UpdateLinkFromLinkNode(child.Link, child);
                link.Children.Add(child.Link);
            }
        }

        private void MergeButtonClick(object sender, EventArgs e)
        {
            LinkNode existing = (LinkNode)ExistingTreeView.Nodes[0].Clone();
            LinkNode loaded = (LinkNode)LoadedTreeView.Nodes[0].Clone();

            // The tree structure is contained in the TreeView not the links
            UpdateLinkFromLinkNode(existing.Link, existing);
            UpdateLinkFromLinkNode(loaded.Link, loaded);

            string msg = TreeMergeHelper.ValidateMerge(existing.Link, loaded.Link,
                    keepInertial: MassInertiaExistingRadio.Checked,
                    keepVisual: VisualExistingRadio.Checked,
                    keepJointKinematics: JointKinematicsExistingRadio.Checked,
                    keepOtherJointValues: JointOtherExistingRadio.Checked);

            if (!string.IsNullOrWhiteSpace(msg))
            {
                MessageBox.Show("Merging URDF trees failed with the following message. Please fix and try again.\r\n\r\n" + msg);
                return;
            }

            if (MessageBox.Show("This will modify the configuration stored in this assembly document?",
                "Finish Merge?", MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                MergedLink = TreeMergeHelper.MergeInfoFromExternalLink(
                    current: existing.Link,
                    external: loaded.Link,
                    keepInertial: MassInertiaExistingRadio.Checked,
                    keepVisual: VisualExistingRadio.Checked,
                    keepJointKinematics: JointKinematicsExistingRadio.Checked,
                    keepOtherJointValues: JointOtherExistingRadio.Checked);

                if (MergedLink != null)
                {
                    Close();
                }
                MergeCompleted(this, new TreeMergedEventArgs(MergedLink != null, MergedLink));
            }
        }

        public void ExistingTreeDragDrop(object sender, DragEventArgs e)
        {
            DoDragDrop(ExistingTreeView, e);
        }

        public void ExistingTreeDragOver(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = ExistingTreeView.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            ExistingTreeView.SelectedNode = ExistingTreeView.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }

        public void ExistingTreeDragEnter(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = ExistingTreeView.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            ExistingTreeView.SelectedNode = ExistingTreeView.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }

        public void ExistingTreeItemDrag(object sender, ItemDragEventArgs e)
        {
            ExistingTreeView.DoDragDrop(e.Item, DragDropEffects.Move | DragDropEffects.Scroll);
        }

        private void LoadedTreeItemDrag(object sender, ItemDragEventArgs e)
        {
            LoadedTreeView.DoDragDrop(e.Item, DragDropEffects.Move | DragDropEffects.Scroll);
        }

        private void LoadedTreeDragEnter(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = LoadedTreeView.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            LoadedTreeView.SelectedNode = LoadedTreeView.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }

        private void LoadedTreeDragOver(object sender, DragEventArgs e)
        {
            // Retrieve the client coordinates of the mouse position.
            Point targetPoint = LoadedTreeView.PointToClient(new Point(e.X, e.Y));

            // Select the node at the mouse position.
            LoadedTreeView.SelectedNode = LoadedTreeView.GetNodeAt(targetPoint);
            e.Effect = DragDropEffects.Move;
        }

        private void LoadedTreeDragDrop(object sender, DragEventArgs e)
        {
            DoDragDrop(LoadedTreeView, e);
        }

        private bool IsAncestor(TreeNode linkNode, TreeNode possibleAncestor)
        {
            if (linkNode.Parent == null)
            {
                return false;
            }
            if (linkNode.Parent == possibleAncestor)
            {
                return true;
            }
            return IsAncestor(linkNode.Parent, possibleAncestor);
        }

        private void MoveNode(TreeView treeView, TreeNode node, TreeNode target)
        {
            TreeNode cloned = (TreeNode)node.Clone();
            node.Remove();
            TreeNodeCollection nodes = (target == null) ? treeView.Nodes : target.Nodes;
            nodes.Add(node);
            treeView.ExpandAll();
        }

        private void MoveNodeTo(TreeView treeView, TreeNode draggedNode, TreeNode targetNode)
        {
            // Check if the move is valid, if not then we won't do anything
            if (draggedNode == null || draggedNode == targetNode || draggedNode.TreeView != treeView)
            {
                return;
            }

            // If the it was dropped into the box itself, but not onto an actual node
            // Move to the end of the children to allow reordering
            if (targetNode == null)
            {
                if (draggedNode.Parent != null)
                {
                    MoveNode(treeView, draggedNode, draggedNode.Parent);
                }
                return;
            }

            // If dropping this node on the target will sever the tree, we need to reconnect it
            if (IsAncestor(targetNode, possibleAncestor: draggedNode))
            {
                // Skipping for now because this is causing a crash. Recommend doing this in two steps manually
                //foreach (TreeNode child in draggedNode.Nodes)
                //{
                //    if (child != null)
                //    {
                //        MoveNode(treeView, child, draggedNode.Parent);
                //    }
                //}

                //MoveNode(treeView, draggedNode, targetNode);
                return;
            }

            MoveNode(treeView, draggedNode, targetNode);
        }

        private void DoDragDrop(TreeView treeView, DragEventArgs e)
        {
            // Retrieve the client coordinates of the drop location.
            Point point = treeView.PointToClient(new Point(e.X, e.Y));

            // Retrieve the node at the drop location.
            TreeNode targetNode = treeView.GetNodeAt(point);

            TreeNode draggedNode = (LinkNode)e.Data.GetData(typeof(LinkNode));
            MoveNodeTo(treeView, draggedNode, targetNode);
        }
    }

    public delegate void TreeMergedEventHandler(object sender, TreeMergedEventArgs eventArgs);

    public class TreeMergedEventArgs : EventArgs
    {
        public readonly bool Success;
        public readonly Link MergedLink;

        public TreeMergedEventArgs(bool success, Link mergedLink)
        {
            Success = success;
            MergedLink = mergedLink;
        }
    }
}