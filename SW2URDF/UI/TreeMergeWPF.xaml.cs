using log4net;
using SW2URDF.URDF;
using SW2URDF.URDFMerge;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Threading;

namespace SW2URDF.UI
{
    /// <summary>
    /// Interaction logic for TreeMergeWPF.xaml
    /// </summary>
    public partial class TreeMergeWPF : Window
    {
        private static readonly ILog logger = Logger.GetLogger();

        private static readonly int MAX_LABEL_CHARACTER_WIDTH = 40;
        private static readonly int MAX_BUTTON_CHARACTER_WIDTH = 20;
        private static readonly string DEFAULT_COORDINATE_SYSTEM_TEXT = "Select Coordinate System";
        private static readonly string DEFAULT_AXIS_TEXT = "Select Reference Axis";
        private static readonly string DEFAULT_JOINT_TYPE_TEXT = "Select Joint Type";

        private readonly string CSVFileName;
        private readonly string AssemblyName;

        private readonly URDFTreeCorrespondance TreeCorrespondance;

        /// <summary>
        /// Property key (since this is a read-only DP) for the IsPossibleDropTarget property.
        /// </summary>
        private static readonly DependencyPropertyKey IsPossibleDropTargetKey =
                                    DependencyProperty.RegisterAttachedReadOnly(
                                        "IsPossibleDropTarget",
                                        typeof(bool),
                                        typeof(TreeMergeWPF),
                                        new FrameworkPropertyMetadata(null,
                                            new CoerceValueCallback(CalculateIsPossibleDropTarget)));

        /// <summary>
        /// Coercion method which calculates the IsPossibleDropTarget property.
        /// </summary>
        private static object CalculateIsPossibleDropTarget(DependencyObject item, object value)
        {
            //if ((item == _currentItem) && (_dropPossible))
            return true;
            //else
            return false;
        }

        /// <summary>
        /// Dependency Property IsPossibleDropTarget.
        /// Is true if the TreeViewItem is a possible drop target (i.e., if it would receive
        /// the OnDrop event if the mouse button is released right now).
        /// </summary>
        public static readonly DependencyProperty IsPossibleDropTargetProperty = IsPossibleDropTargetKey.DependencyProperty;

        public TreeMergeWPF(List<string> coordinateSystems, List<string> referenceAxes, string csvFileName, string assemblyName)
        {
            Dispatcher.UnhandledException += App_DispatcherUnhandledException;

            CSVFileName = csvFileName;
            AssemblyName = assemblyName;

            InitializeComponent();
            ConfigureMenus(coordinateSystems, referenceAxes);
            ConfigureLabels();

            TreeCorrespondance = new URDFTreeCorrespondance();
        }

        private void App_DispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            logger.Error("Exception encountered in TreeMerge form", e.Exception);
            MessageBox.Show("There was a problem with the TreeMerge form: \n\"" +
                e.Exception.Message + "\"\nEmail your maintainer with the log file found at " +
                Logger.GetFileName());
            e.Handled = true;
        }

        public void SetTrees(LinkNode existingNode, LinkNode loadedNode)
        {
            ExistingTreeView.Items.Clear();
            LoadedTreeView.Items.Clear();

            TreeViewItem existing = BuildTreeViewItem(existingNode);
            TreeViewItem loaded = BuildTreeViewItem(loadedNode);

            ExistingTreeView.MouseMove += TreeViewMouseMove;
            ExistingTreeView.Drop += TreeViewDrop;

            ExistingTreeView.Items.Add(existing);
            LoadedTreeView.Items.Add(loaded);

            ExistingTreeView.AllowDrop = true;
            LoadedTreeView.AllowDrop = true;

            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedTreeView);
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
            LoadedTreeLabel.Content = BuildTextBlock("Configuration from CSV: ", longCSVFilename);

            MassInertiaExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            VisualExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            JointKinematicsExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            OtherJointExistingButton.Content = new TextBlock { Text = shortAssemblyName };

            MassInertiaLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            VisualLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            JointKinematicsLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            OtherJointLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
        }

        /// <summary>
        /// This function checks if there will be a hole created when the package is removed from
        /// it's parent during a drag and drop operation. This happens when the package is
        /// dropped on a target that is direct descendent in the tree branch
        /// </summary>
        /// <param name="target"></param>
        /// <param name="package"></param>
        /// <returns></returns>
        private bool IsTargetDescendent(TreeViewItem target, TreeViewItem package)
        {
            // If these are the same thing, then target is not a descendent
            if (target == package || target.Parent == null)
            {
                return false;
            }

            // If the parent of the target is a TreeView and not another Item, then we're done.
            if (target.Parent.GetType() != typeof(TreeViewItem))
            {
                return false;
            }

            // If the target's parent is the package, then yes, it's a descendent
            if (target.Parent == package)
            {
                return true;
            }

            // Recur up the tree from the target. If the target's parent is a descendent
            // then the target is a descendent.
            return IsTargetDescendent((TreeViewItem)target.Parent, package);
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
            if (IsTargetDescendent(target, package))
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
                target.Items.Add(package);
            }
            else
            {
                target.Items.Insert(position, package);
            }
        }

        private bool IsPointToSideOfElement(TreeViewItem item, Point pointOnElement)
        {
            // You would think PointFromScreen would not mutate the point, but noooo

            //Point pointOnElement = e.GetPosition(item);//new Point(pointOnScreen.X, pointOnScreen.Y);

            // Translate screen point to the element's coordinate frame
            //pointOnElement = item.PointFromScreen(pointOnScreen);

            // Set the
            pointOnElement.X = 1;
            IInputElement result = item.InputHitTest(pointOnElement);
            return result != null;
        }

        private TreeViewItem GetItemToSideOfPoint(List<TreeViewItem> items, DragEventArgs e)
        {
            TreeViewItem previous = null;

            foreach (TreeViewItem item in items)
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

        private TreeViewItem GetItemClosestToPoint(TreeView tree, DragEventArgs e)
        {
            UIElement first = (UIElement)tree.Items[0];
            //FrameworkElement.ActualHeightProperty
            double treeItemHeight = first.DesiredSize.Height;

            // Move the drop point up half a box width because we want to use the middle of
            // the element to decide whether to drop above or below it.
            //dropPointOnScreen.Offset(0.0, -treeItemHeight / 2.0);

            // Call recursive method to find the item
            List<TreeViewItem> flattened = URDFTreeCorrespondance.FlattenTreeView(tree);
            return GetItemToSideOfPoint(flattened, e);
        }

        private void ProcessDragDropOnTree(TreeView tree, TreeViewItem package, DragEventArgs e)
        {
            TreeViewItem closest = GetItemClosestToPoint(tree, e);

            // If no closest item was found, or if it didn't pass the qualifications then skip
            if (closest == null)
            {
                return;
            }

            if (closest.Items.Count > 0)
            {
                ProcessDragDropOnItem(tree, closest, package, 0);
            }
            else
            {
                TreeViewItem parent = (TreeViewItem)closest.Parent;
                int closestIndex = parent.Items.IndexOf(closest);
                ProcessDragDropOnItem(tree, parent, package, closestIndex + 1);
            }
        }

        private void TreeViewDrop(object sender, DragEventArgs e)
        {
            TreeViewItem package = e.Data.GetData(typeof(TreeViewItem)) as TreeViewItem;
            if (package != null & package != e.Source)
            {
                // Dropping onto a Tree node
                if (e.Source.GetType() == typeof(TreeViewItem))
                {
                    ProcessDragDropOnItem((TreeView)sender, (TreeViewItem)e.Source, package);
                }
                else if (e.Source.GetType() == typeof(TreeView))
                {
                    // Dropping outside of a node will reorder nodes
                    TreeView tree = (TreeView)e.Source;

                    Point point = e.GetPosition(this);
                    Point pointSender = e.GetPosition((IInputElement)sender);
                    Point pointOnTree = e.GetPosition(tree);

                    Point pointOnScreen = tree.PointToScreen(point);
                    ProcessDragDropOnTree(tree, package, e);
                }
                else
                {
                    logger.Warn("Unhandled drop target " + e.Source.GetType());
                }
            }
            // Items have been reordered probably. Rebuild the correspondance.
            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedTreeView);
        }

        private void TreeViewItemDragEnter(object sender, DragEventArgs e)
        {
            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                TreeViewItem target = (TreeViewItem)e.Source;
                target.Background = SystemColors.ActiveBorderBrush;
            }
        }

        private void TreeViewItemDragLeave(object sender, DragEventArgs e)
        {
            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                TreeViewItem target = (TreeViewItem)e.Source;
                target.Background = null;
            }
        }

        private void TreeViewMouseMove(object sender, MouseEventArgs e)
        {
            TreeView treeView = sender as TreeView;
            if (e.MouseDevice.LeftButton == MouseButtonState.Pressed)
            {
                DependencyObject dependencyObject = treeView.InputHitTest(e.GetPosition(treeView)) as DependencyObject;
                //Point downPos = e.GetPosition(null);

                if (treeView.SelectedValue != null)
                {
                    //TreeViewItem treeviewItem = e.Source as TreeViewItem;
                    DragDrop.DoDragDrop(treeView, treeView.SelectedValue, DragDropEffects.Move);
                    e.Handled = true;
                }
            }
        }

        private void TreeViewClick(object sender, MouseButtonEventArgs e)
        {
            TreeView treeView = sender as TreeView;
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