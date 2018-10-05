using log4net;
using SW2URDF.URDF;
using SW2URDF.URDFMerge;
using System;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Threading;

namespace SW2URDF.UI
{
    /// <summary>
    /// Interaction logic for TreeMergeWPF.xaml
    /// </summary>
    public partial class TreeMergeWPF : Window
    {
        private static readonly ILog logger = Logger.GetLogger();

        public event EventHandler<TreeMergedEventArgs> TreeMerged = delegate { };

        private static readonly int MAX_LABEL_CHARACTER_WIDTH = 40;
        private static readonly int MAX_BUTTON_CHARACTER_WIDTH = 20;
        private static readonly string DEFAULT_COORDINATE_SYSTEM_TEXT = "Select Coordinate System";
        private static readonly string DEFAULT_AXIS_TEXT = "Select Reference Axis";
        private static readonly string DEFAULT_JOINT_TYPE_TEXT = "Select Joint Type";

        private readonly string CSVFileName;
        private readonly string AssemblyName;

        private readonly URDFTreeCorrespondance TreeCorrespondance;

        public TreeMergeWPF(List<string> coordinateSystems, List<string> referenceAxes, string csvFileName, string assemblyName)
        {
            Dispatcher.UnhandledException += AppDispatcherUnhandledException;

            CSVFileName = csvFileName;
            AssemblyName = assemblyName;

            InitializeComponent();
            ConfigureMenus(coordinateSystems, referenceAxes);
            ConfigureLabels();

            TreeCorrespondance = new URDFTreeCorrespondance();
        }

        private void AppDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            logger.Error("Exception encountered in TreeMerge form", e.Exception);
            MessageBox.Show("There was a problem with the TreeMerge form: \n\"" +
                e.Exception.Message + "\"\nEmail your maintainer with the log file found at " +
                Logger.GetFileName());
            e.Handled = true;
        }

        public void SetTrees(LinkNode existingNode, LinkNode loadedNode)
        {
            ExistingTreeView.SetTree(existingNode);
            LoadedTreeView.SetTree(loadedNode);

            ExistingTreeView.TreeModified += TreeViewModified;
            LoadedTreeView.TreeModified += TreeViewModified;

            ExistingTreeView.SelectedItemChanged += OnTreeItemClick;
            LoadedTreeView.SelectedItemChanged += OnTreeItemClick;

            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedTreeView);
        }

        private void TreeViewModified(object sender, TreeModifiedEventArgs e)
        {
            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedTreeView);
        }

        private void CancelClick(object sender, EventArgs e)
        {
            if (MessageBox.Show("Do you wish to cancel? Any changes you have made will not be saved",
                "Cancel Merge?", MessageBoxButton.YesNo) == MessageBoxResult.Yes)
            {
                Close();
            }
        }

        private void MergeClick(object sender, EventArgs e)
        {
            if (MessageBox.Show("Do you wish to merge these configuration trees? The configuration in the assembly" +
                " will be overwritten.",
                "Confirm Merge", MessageBoxButton.YesNo) != MessageBoxResult.Yes)
            {
                return;
            }

            TreeMerger merger = new TreeMerger(MassInertiaLoadedButton.IsChecked.Value,
                                                         VisualLoadedButton.IsChecked.Value,
                                                         JointKinematicsLoadedButton.IsChecked.Value,
                                                         OtherJointLoadedButton.IsChecked.Value);
            URDFTreeView result = merger.Merge(ExistingTreeView, LoadedTreeView);

            if (result != null)
            {
                TreeMergedEventArgs mergedArgs = new TreeMergedEventArgs(result, true, merger);
                TreeMerged(this, mergedArgs);
            }
            else
            {
                TreeMerged(this, new TreeMergedEventArgs());
            }
            Close();
        }

        private void FillExistingLinkProperties(Link link, bool isBaseLink)
        {
            ExistingLinkNameTextBox.Text = link.Name;

            if (isBaseLink)
            {
                ExistingJointNameTextBox.Text = "";
                ExistingJointNameTextBox.Visibility = Visibility.Hidden;
                ExistingCoordinatesMenu.Visibility = Visibility.Hidden;
                ExistingAxisMenu.Visibility = Visibility.Hidden;
                ExistingJointTypeMenu.Visibility = Visibility.Hidden;
            }
            else
            {
                ExistingJointNameTextBox.Text = link.Joint.Name;
                SetDropdownContextMenu(ExistingCoordinatesMenu, link.Joint.CoordinateSystemName, DEFAULT_COORDINATE_SYSTEM_TEXT);
                SetDropdownContextMenu(ExistingAxisMenu, link.Joint.AxisName, DEFAULT_AXIS_TEXT);
                SetDropdownContextMenu(ExistingJointTypeMenu, link.Joint.Type, DEFAULT_JOINT_TYPE_TEXT);
            }
        }

        private void FillLoadedLinkProperties(Link link, bool isBaseLink)
        {
            LoadedLinkNameTextBox.Text = link.Name;

            if (isBaseLink)
            {
                LoadedJointNameTextLabel.Content = null;
                LoadedCoordinateSystemTextLabel.Content = null;
                LoadedAxisTextLabel.Content = null;
                LoadedJointTypeTextLabel.Content = null;
            }
            else
            {
                LoadedJointNameTextLabel.Content = new TextBlock { Text = link.Name };
                LoadedCoordinateSystemTextLabel.Content = new TextBlock { Text = link.Joint.CoordinateSystemName };
                LoadedAxisTextLabel.Content = new TextBlock { Text = link.Joint.AxisName };
                LoadedJointTypeTextLabel.Content = new TextBlock { Text = link.Joint.Type };
            }
        }

        private void OnTreeItemClick(object sender, RoutedEventArgs e)
        {
            TreeView tree = (TreeView)sender;
            if (tree.SelectedItem == null)
            {
                return;
            }

            TreeViewItem selectedItem = (TreeViewItem)tree.SelectedItem;
            Link link = (Link)selectedItem.Tag;

            // The base link does not have a joint associated with it, so we'll hide the joint
            // controls
            bool isBaseLink = selectedItem.Parent.GetType() == typeof(TreeView);

            if (tree == ExistingTreeView)
            {
                FillExistingLinkProperties(link, isBaseLink);
            }
            else if (tree == LoadedTreeView)
            {
                FillLoadedLinkProperties(link, isBaseLink);
            }

            TreeViewItem corresponding = TreeCorrespondance.GetCorrespondingTreeViewItem(selectedItem);
            if (corresponding != null)
            {
                corresponding.IsSelected = true;
            }
        }

        private void SetDropdownContextMenu(Button button, string name, string defaultText)
        {
            button.Visibility = Visibility.Visible;
            if (name == null)
            {
                return;
            }

            TextBlock buttonText = (TextBlock)button.Content;

            foreach (MenuItem item in button.ContextMenu.Items)
            {
                TextBlock header = (TextBlock)item.Header;
                if (header.Text == name)
                {
                    item.IsChecked = true;
                    buttonText.Text = name;
                    return;
                }
            }

            logger.Error("Item " + name + " was not found in the dropdown for " + button.Name);
            buttonText.Text = defaultText;
        }

        private string ShortenStringForLabel(string text, int numCharacters)
        {
            string result = text;
            if (text.Length > numCharacters)
            {
                string extension = Path.GetExtension(text);
                int numToKeep = numCharacters - "...".Length - extension.Length;
                result = text.Substring(0, numToKeep) + "..." + extension;
            }
            return result;
        }

        private TextBlock BuildTextBlock(string boldBit, string regularBit)
        {
            TextBlock block = new TextBlock();
            block.Inlines.Add(new Bold(new Run(boldBit)));
            block.Inlines.Add(regularBit);
            return block;
        }

        private void ConfigureLabels()
        {
            string longAssemblyName = ShortenStringForLabel(AssemblyName, MAX_LABEL_CHARACTER_WIDTH);
            string shortAssemblyName = ShortenStringForLabel(AssemblyName, MAX_BUTTON_CHARACTER_WIDTH);

            string longCSVFilename = ShortenStringForLabel(CSVFileName, MAX_LABEL_CHARACTER_WIDTH);
            string shortCSVFilename = ShortenStringForLabel(CSVFileName, MAX_BUTTON_CHARACTER_WIDTH);

            ExistingTreeLabel.Content = BuildTextBlock("Configuration from Assembly: ", longAssemblyName);
            ExistingTreeLabel.ToolTip =
                new TextBlock { Text = "Configuration from Assembly: " + AssemblyName };

            LoadedTreeLabel.Content = BuildTextBlock("Configuration from CSV: ", longCSVFilename);
            LoadedTreeLabel.ToolTip =
                new TextBlock { Text = "Configuration from CSV: " + CSVFileName };

            MassInertiaExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            MassInertiaExistingButton.ToolTip =
                new TextBlock { Text = "Use Mass and Inertia properties loaded from: " + AssemblyName };

            VisualExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            VisualExistingButton.ToolTip =
                new TextBlock { Text = "Use Mesh and Material properties loaded from: " + AssemblyName };

            JointKinematicsExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            JointKinematicsExistingButton.ToolTip =
                new TextBlock { Text = "Use Joint Kinematic properties loaded from: " + AssemblyName };

            OtherJointExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            OtherJointExistingButton.ToolTip = new TextBlock
            {
                Text =
                "Use Limits, Dynamics, Calibration and Safety Controller values loaded from: " +
                AssemblyName
            };

            MassInertiaLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            MassInertiaLoadedButton.ToolTip =
                new TextBlock { Text = "Use Mass and Inertia properties loaded from: " + CSVFileName };

            VisualLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            VisualLoadedButton.ToolTip =
                new TextBlock { Text = "Use Mesh and Material properties loaded from: " + CSVFileName };

            JointKinematicsLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            JointKinematicsLoadedButton.ToolTip =
                new TextBlock { Text = "Use Joint Kinematic properties loaded from: " + CSVFileName };

            OtherJointLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            OtherJointLoadedButton.ToolTip = new TextBlock
            {
                Text = "Use Limits, Dynamics, Calibration and Safety Controller values loaded from: " + CSVFileName
            };
        }

        private bool IsValidDrop(TreeView tree, TreeViewItem package, DragEventArgs e)
        {
            if (package == null)
            {
                return false;
            }

            if (package == e.Source)
            {
                return false;
            }

            if (!tree.IsAncestorOf(package))
            {
                // if dropping into the wrong tree, abort!
                return false;
            }

            return true;
        }

        private bool IsValidDrop(TreeViewItem target, DragEventArgs e)
        {
            if (!(e.Data.GetData(typeof(TreeViewItem)) is TreeViewItem package))
            {
                return false;
            }

            if (target == package)
            {
                return false;
            }

            if (!target.IsAncestorOf(package) && !package.IsAncestorOf(target))
            {
                // If the target and the package aren't in the same tree, then no bueno
                return false;
            }
            return true;
        }

        /// <summary>
        /// A drag and drop feature is not simple to implement for a tree. There are several considerations about how the
        /// tree gets reordered when you drag a tree node to another tree node. Part of the difficulty is that there
        /// can be only one root node, so it has to be predictable which one that will be.
        ///
        /// This logic has two cases, either you're moving the node down the tree to one of its direct descendents,
        /// or you are adding it to another unrelated node.
        ///
        /// For the first case, the package node and all its descendents up to the target will be brought with it, and
        /// attached to the target. The target will then be attached to the package's parent.
        ///
        /// For the second case, it's the package is simply removed from its parent and attached to the target. With all
        /// the descendents brought with it.
        /// </summary>
        /// <param name="treeView"></param>
        /// <param name="target"></param>
        /// <param name="package"></param>
        private void ProcessDragDropOnItem(TreeView treeView, TreeViewItem target, TreeViewItem package, int position = -1)
        {
            // The parent of the package could be either a TreeView or TreeViewItem
            ItemsControl packageParent = (ItemsControl)package.Parent;

            // Clear background because DragLeave won't be activated
            target.Background = null;
            if (package.IsAncestorOf(target))
            {
                // You are now creating a hole in the tree, to resolve, we'll promote
                // the target to a child of the package's parent, and then add the package
                // to the target's children. We already know the target's parent is a TreeViewItem
                // so the cast is fine
                TreeViewItem targetParent = (TreeViewItem)target.Parent;

                targetParent.Items.Remove(target);
                packageParent.Items.Add(target);

                // The package and its remaing branches are then added to the target
                packageParent.Items.Remove(package);
            }
            else
            {
                // The simplest of cases. We can just remove the package from it's previous parent
                // and add it to its new parent.
                packageParent.Items.Remove(package);
            }

            if (position < 0)
            {
                // If the package was dropped onto a tree node, add to the end of its children
                target.Items.Add(package);
            }
            else
            {
                // If the package was dropped outside of a node, insert it in its appropriate position
                target.Items.Insert(position, package);
            }
        }

        private bool IsPointToSideOfElement(TreeViewItem item, Point pointOnElement)
        {
            pointOnElement.X = 1;
            IInputElement result = item.InputHitTest(pointOnElement);
            return result != null;
        }

        /// <summary>
        /// When reordering elements, we want to check which one is to the side of the point we drop it at.
        /// We'll put the new one just below it.
        /// </summary>
        /// <param name="items"></param>
        /// <param name="e"></param>
        /// <returns></returns>
        private TreeViewItem GetItemToSideOfPoint(TreeView tree, DragEventArgs e)
        {
            List<TreeViewItem> flattened = URDFTreeCorrespondance.FlattenTreeView(tree);

            TreeViewItem previous = null;

            foreach (TreeViewItem item in flattened)
            {
                Point pointOnElement = e.GetPosition(item);
                if (pointOnElement.Y < 0)
                {
                    // We went passed it, return the previous one
                    return previous;
                }

                if (IsPointToSideOfElement(item, pointOnElement))
                {
                    return item;
                }

                previous = item;
            }
            return null;
        }

        /// <summary>
        /// If they don't drop the package directly on an item, and instead drop it on the tree
        /// then that's how things get reordered.
        /// </summary>
        /// <param name="tree"></param>
        /// <param name="package"></param>
        /// <param name="e"></param>
        private void ProcessDragDropOnTree(TreeView tree, TreeViewItem package, DragEventArgs e)
        {
            TreeViewItem closest = GetItemToSideOfPoint(tree, e);

            // If no closest item was found, or if it didn't pass the qualifications then skip
            if (closest == null)
            {
                return;
            }

            if (closest.Items.Count > 0)
            {
                // If they drop it inbetween a parent and its first child, then that means they
                // want to set it as the closest's first item.
                ProcessDragDropOnItem(tree, closest, package, 0);
            }
            else
            {
                // If the closest was found, then add it to its parent at the appropriate index
                TreeViewItem parent = (TreeViewItem)closest.Parent;
                int closestIndex = parent.Items.IndexOf(closest);
                ProcessDragDropOnItem(tree, parent, package, closestIndex + 1);
            }
        }

        private void TreeViewDrop(object sender, DragEventArgs e)
        {
            TreeView tree = (TreeView)sender;
            TreeViewItem package = e.Data.GetData(typeof(TreeViewItem)) as TreeViewItem;

            if (!IsValidDrop(tree, package, e))
            {
                return;
            }

            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                // Dropping onto a Tree node
                ProcessDragDropOnItem(tree, (TreeViewItem)e.Source, package);
            }
            else if (e.Source.GetType() == typeof(TreeView))
            {
                // Dropping outside of a node will reorder nodes
                ProcessDragDropOnTree(tree, package, e);
            }

            // Items have been reordered probably. Rebuild the correspondance.
            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedTreeView);
        }

        /// <summary>
        /// This is how we control how TreeViewItems are highlighted when someone drags over them
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void TreeViewItemDragEnter(object sender, DragEventArgs e)
        {
            if (IsValidDrop((TreeViewItem)sender, e))
            {
                TreeViewItem target = (TreeViewItem)e.Source;
                target.Background = SystemColors.ActiveBorderBrush;
            }
        }

        /// <summary>
        /// This disables the highlight when dragging over leaves this element
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void TreeViewItemDragLeave(object sender, DragEventArgs e)
        {
            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                TreeViewItem target = (TreeViewItem)e.Source;
                target.Background = null;
            }
        }

        private TreeViewItem BuildTreeViewItem(LinkNode node)
        {
            TreeViewItem item = new TreeViewItem
            {
                Tag = node.Link,
                IsExpanded = true,
                AllowDrop = true,
                Name = node.Name,
                Header = node.Name,
            };

            item.DragEnter += TreeViewItemDragEnter;
            item.DragLeave += TreeViewItemDragLeave;

            foreach (LinkNode child in node.Nodes)
            {
                item.Items.Add(BuildTreeViewItem(child));
            }

            return item;
        }

        private void ConfigureMenus(List<string> coordinateSystems, List<string> referenceAxes)
        {
            SetMenu(ExistingCoordinatesMenu, coordinateSystems);
            SetMenu(ExistingAxisMenu, referenceAxes);
            SetMenu(ExistingJointTypeMenu, Joint.AVAILABLE_TYPES);
        }

        private void SetMenu(Button button, List<string> menuContents)
        {
            button.ContextMenu.Items.Clear();
            bool isFirst = true;
            foreach (string menuItemLabel in menuContents)
            {
                MenuItem menuItem = new MenuItem
                {
                    Header = new TextBlock { Text = menuItemLabel },
                    IsCheckable = true,
                    IsChecked = isFirst,
                };
                isFirst = false;

                menuItem.Checked += MenuItemChecked;
                button.ContextMenu.Items.Add(menuItem);
            }
        }

        private void MenuClick(object sender, RoutedEventArgs e)
        {
            (sender as Button).ContextMenu.IsEnabled = true;
            (sender as Button).ContextMenu.PlacementTarget = (sender as Button);
            (sender as Button).ContextMenu.Placement = System.Windows.Controls.Primitives.PlacementMode.Bottom;
            (sender as Button).ContextMenu.IsOpen = true;
        }

        private void MenuItemChecked(object sender, RoutedEventArgs e)
        {
            MenuItem menuItem = sender as MenuItem;
            logger.Info("Parent type " + menuItem.Parent.GetType());
            ContextMenu contextMenuParent = menuItem.Parent as ContextMenu;
            foreach (MenuItem item in contextMenuParent.Items)
            {
                if (item != sender)
                {
                    logger.Info("Unchecking " + item.Header);
                    item.IsChecked = false;
                }
            }

            // During the InitializeComponents, this callback fires, but things aren't fully setup
            if (!(contextMenuParent.PlacementTarget is Button button))
            {
                return;
            }
            if (!(menuItem.Header is TextBlock menuItemText))
            {
                return;
            }
            button.Content = new TextBlock
            {
                Text = menuItemText.Text,
            };
        }
    }
}