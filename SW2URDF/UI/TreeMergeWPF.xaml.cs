using log4net;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace SW2URDF.UI
{
    /// <summary>
    /// Interaction logic for TreeMergeWPF.xaml
    /// </summary>
    public partial class TreeMergeWPF : Window
    {
        private static readonly ILog logger = Logger.GetLogger();

        public TreeMergeWPF(List<string> coordinateSystems, List<string> referenceAxes)
        {
            InitializeComponent();
            ConfigureMenus(coordinateSystems, referenceAxes);
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
        }

        private void ProcessDragDrop(TreeView treeView, TreeViewItem target, TreeViewItem package)
        {
            if (package.Parent == treeView)
            {
            }
            else if (package.Parent.GetType() == typeof(TreeViewItem))
            {
                TreeViewItem packageParent = (TreeViewItem)package.Parent;

                packageParent.Items.Remove(package);

                target.Items.Add(package);
            }
            else
            {
                logger.Warn("Unhandled package parent " + package.Parent.GetType());
            }
        }

        private void TreeViewDrop(object sender, DragEventArgs e)
        {
            TreeViewItem package = e.Data.GetData(typeof(TreeViewItem)) as TreeViewItem;
            if (package != null & package != e.Source)
            {
                if (e.Source.GetType() == typeof(TreeViewItem))
                {
                    ProcessDragDrop((TreeView)sender, (TreeViewItem)e.Source, package);
                }
                else if (e.Source.GetType() == typeof(TreeView))
                {
                }
                else
                {
                    logger.Warn("Unhandled drop target " + e.Source.GetType());
                }
            }
        }

        private void TreeViewMouseMove(object sender, MouseEventArgs e)
        {
            TreeView treeView = sender as TreeView;
            if (e.MouseDevice.LeftButton == MouseButtonState.Pressed)
            {
                DependencyObject dependencyObject = treeView.InputHitTest(e.GetPosition(treeView)) as DependencyObject;

                if (treeView.SelectedValue != null)
                {
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
        }

        private void SetMenu(Button button, List<string> menuContents)
        {
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
            Button button = contextMenuParent.PlacementTarget as Button;
            TextBlock menuItemText = menuItem.Header as TextBlock;
            if (menuItemText == null)
            {
                logger.Info("MenuItemText is null here");
                return;
            }
            button.Content = new TextBlock
            {
                Text = menuItemText.Text,
            };
        }
    }
}
