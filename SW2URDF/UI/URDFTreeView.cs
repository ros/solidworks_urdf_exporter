using SW2URDF.URDF;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace SW2URDF.UI
{
    public class URDFTreeView : TreeView
    {
        public event EventHandler<TreeModifiedEventArgs> TreeModified;

        //public new event RoutedEventHandler SelectedTreeItemChanged;

        public URDFTreeView()
        {
            //base.SelectedItemChanged += BaseItemChanged;
        }

        private void BaseItemChanged(object sender, RoutedPropertyChangedEventArgs<object> e)
        {
            //SelectedItemChanged(sender, e);
        }

        public void SetTree(LinkNode node)
        {
            Items.Clear();

            TreeViewItem item = BuildTreeViewItem(node);
            MouseMove += TreeMouseMove;
            Drop += TreeViewDrop;
            Items.Add(item);
            AllowDrop = true;
        }

        public List<TreeViewItem> Flatten()
        {
            return Flatten(Items);
        }

        private static List<TreeViewItem> Flatten(ItemCollection items)
        {
            List<TreeViewItem> list = new List<TreeViewItem>();
            foreach (TreeViewItem item in items)
            {
                list.Add(item);
                list.AddRange(Flatten(item.Items));
            }
            return list;
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

            item.DragEnter += ItemDragEnter;
            item.DragLeave += ItemDragLeave;

            foreach (LinkNode child in node.Nodes)
            {
                item.Items.Add(BuildTreeViewItem(child));
            }

            return item;
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
                target.Items.Add(package);
            }
            else
            {
                target.Items.Insert(position, package);
            }

            TreeModified(this, new TreeModifiedEventArgs { Tree = this });
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
        private TreeViewItem GetItemToSideOfPoint(URDFTreeView tree, DragEventArgs e)
        {
            List<TreeViewItem> flattened = tree.Flatten();

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
            TreeViewItem closest = GetItemToSideOfPoint((URDFTreeView)tree, e);

            // If no closest item was found, or if it didn't pass the qualifications then skip
            if (closest == null)
            {
                return;
            }

            if (closest.Items.Count > 0)
            {
                // If they drop it inbetween a parent and its first child, then that means they
                // want to put set it as the closest's first item.
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

            TreeModified(this, new TreeModifiedEventArgs { Tree = this });
        }

        /// <summary>
        /// This is how we control how TreeViewItems are highlighted when someone drags over them
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void ItemDragEnter(object sender, DragEventArgs e)
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
        private void ItemDragLeave(object sender, DragEventArgs e)
        {
            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                TreeViewItem target = (TreeViewItem)e.Source;
                target.Background = null;
            }
        }

        private void TreeMouseMove(object sender, MouseEventArgs e)
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
    }
}