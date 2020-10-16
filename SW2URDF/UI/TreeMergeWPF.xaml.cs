using log4net;
using SW2URDF.URDF;
using SW2URDF.URDFExport.CSV;
using SW2URDF.URDFExport.URDFMerge;
using SW2URDF.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.IO;
using System.Linq;
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

        private const int MAX_LABEL_CHARACTER_WIDTH = 40;
        private const int MAX_BUTTON_CHARACTER_WIDTH = 20;

        private readonly string CSVFileName;
        private readonly string AssemblyName;

        private readonly URDFTreeCorrespondance TreeCorrespondance;

        private readonly Link ExistingBaseLink;
        private readonly List<Link> LoadedCSVLinks;
        private readonly HashSet<string> LoadedCSVLinkNames;
        private Link SelectedLink;
        public ObservableCollection<KeyValuePair<string, object>> SelectedLinkProperties { get; }

        public TreeMergeWPF(Link existingLink, List<Link> loadedLinks, string csvFileName, string assemblyName)
        {
            DataContext = this;
            Resources["SelectedLinkProperties"] = SelectedLinkProperties;
            Dispatcher.UnhandledException += AppDispatcherUnhandledException;

            CSVFileName = csvFileName;
            AssemblyName = assemblyName;

            InitializeComponent();
            ConfigureLabels();

            ExistingBaseLink = existingLink;
            LoadedCSVLinks = new List<Link>(loadedLinks);
            LoadedCSVLinkNames = new HashSet<string>();
            TreeCorrespondance = new URDFTreeCorrespondance();
            SelectedLinkProperties = new ObservableCollection<KeyValuePair<string, object>>();

            PropertiesListView.DataContext = SelectedLinkProperties;

            ExistingTreeView.SelectedItemChanged += OnTreeItemClick;
            MergeAndUpdate();
        }

        private void AppDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            logger.Error("Exception encountered in TreeMerge form", e.Exception);
            MessageBox.Show("There was a problem with the TreeMerge form: \n\"" +
                e.Exception.Message + "\"\nEmail your maintainer with the log file found at " +
                Logger.GetFileName());
            e.Handled = true;
        }

        /// <summary>
        /// This method performs a merge between the Loaded links and the existing configuration
        /// based on the names of the loaded links. It also updates the form to match the most
        /// updated merge
        /// </summary>
        /// <returns></returns>
        private TreeMerger MergeAndUpdate()
        {
            // Setup merge to start with a fresh link,
            ExistingTreeView.SetTree(ExistingBaseLink.Clone());
            LoadedCSVLinkNames.Clear();
            LoadedCSVLinkNames.UnionWith(LoadedCSVLinks.Select(link => link.Name));

            // Update correspondance with the most up-to-date names as well as the
            // appropriate list boxes
            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedCSVLinks,
                out List<Link> matched, out List<Link> unmatched);
            UpdateList(MatchingLoadedLinks, matched);
            UpdateList(UnmatchedLoadedLinks, unmatched);

            // Perform merge
            TreeMerger merger = new TreeMerger(MassInertiaLoadedButton.IsChecked.Value,
                                                        VisualLoadedButton.IsChecked.Value,
                                                        JointKinematicsLoadedButton.IsChecked.Value,
                                                        OtherJointLoadedButton.IsChecked.Value);

            Link mergedRoot = merger.Merge(ExistingTreeView, TreeCorrespondance);

            // Update Form Tree
            ExistingTreeView.SetTree(mergedRoot);

            return merger;
        }

        private void UpdateList(ListBox listBox, List<Link> unmatched)
        {
            listBox.Items.Clear();
            foreach (Link link in unmatched)
            {
                ListBoxItem item = new ListBoxItem
                {
                    Tag = link,
                    Name = link.Name,
                    Content = new TextBlock { Text = link.Name }
                };
                item.Selected += OnListBoxItemClick;
                listBox.Items.Add(item);
            }
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
            TreeMerger merger = MergeAndUpdate();
            if (UnmatchedLoadedLinks.Items.Count > 0)
            {
                IEnumerable<string> unmatchedLinkNames =
                    UnmatchedLoadedLinks.Items
                                        .Cast<ListBoxItem>()
                                        .Select(item => ((Link)item.Tag).Name);

                string unmatchedLinksStr = string.Join("\r\n", unmatchedLinkNames);

                string message = "The follow links loaded from the CSV " + CSVFileName + " have not " +
                    "been matched with links in the assembly configuration, would you like to " +
                    "continue?\r\n\r\n" + unmatchedLinksStr;

                MessageBoxResult result =
                    MessageBox.Show(message, "Merge with unmatched links?", MessageBoxButton.YesNo);
                if (result != MessageBoxResult.Yes)
                {
                    return;
                }
            }

            TreeMergedEventArgs mergedArgs = new TreeMergedEventArgs(ExistingTreeView, true, merger, CSVFileName);
            TreeMerged(this, mergedArgs);

            Close();
        }

        /// <summary>
        /// When an item in any of the list boxes or treeview is selected, the box and properties list view
        /// need to be populated. The link name TextBox is directly set, while the the properties ListView is
        /// bound to the SelectedLinkProperties dictionary
        /// </summary>
        /// <param name="link"></param>
        private void FillSelectedLinkBoxes(Link link)
        {
            SelectedLinkName.Text = link.Name;

            OrderedDictionary dictionary = new OrderedDictionary();
            link.AppendToCSVDictionary(new List<string>(), dictionary);

            SelectedLinkProperties.Clear();
            foreach (DictionaryEntry entry in dictionary)
            {
                string context = (string)entry.Key;
                string columnName = (string)ContextToColumns.Dictionary[context];
                SelectedLinkProperties.Add(new KeyValuePair<string, object>(columnName, entry.Value));
            }
        }

        private void ClearLinkBoxes()
        {
            SelectedLinkName.Text = null;
            SelectedLinkProperties.Clear();
        }

        private void UnselectOtherBoxes(object boxWithSelection)
        {
            TreeViewItem selectedTreeItem = (ExistingTreeView.SelectedItem as TreeViewItem);
            if (boxWithSelection != ExistingTreeView && selectedTreeItem != null)
            {
                selectedTreeItem.IsSelected = false;
            }

            ListBoxItem previouslySelected = (MatchingLoadedLinks.SelectedItem as ListBoxItem);
            if (boxWithSelection != MatchingLoadedLinks && previouslySelected != null)
            {
                previouslySelected.IsSelected = false;
            }

            previouslySelected = (UnmatchedLoadedLinks.SelectedItem as ListBoxItem);
            if (boxWithSelection != UnmatchedLoadedLinks && previouslySelected != null)
            {
                previouslySelected.IsSelected = false;
            }
        }

        private void OnListBoxItemClick(object sender, RoutedEventArgs e)
        {
            UnselectOtherBoxes(sender);

            ListBoxItem selectedItem = (sender as ListBoxItem);
            Link selectedLink = (Link)selectedItem.Tag;
            ProcessItemClick(sender, selectedLink, true);
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

            ProcessItemClick(sender, link, false);
        }

        private void ProcessItemClick(object sender, Link link, bool isLoadedFromCSV)
        {
            UnselectOtherBoxes(sender);
            SelectedLinkName.IsReadOnly = !isLoadedFromCSV;
            SelectedLink = link;
            FillSelectedLinkBoxes(link);

            string labelText = (isLoadedFromCSV) ? "Properties loaded from CSV" : "Preview of Merged Properties";
            PropertiesLoadedLabel.Content = new TextBlock { Text = labelText };
        }

        private void OnUpdateButtonClick(object sender, RoutedEventArgs e)
        {
            string updatedName = SelectedLinkName.Text;
            HashSet<string> existingNames = new HashSet<string>(LoadedCSVLinkNames);
            existingNames.Remove(SelectedLink.Name);

            if (string.IsNullOrWhiteSpace(updatedName))
            {
                SelectedLinkName.ToolTip = new ToolTip
                {
                    Content = "Link name cannot be empty",
                    IsOpen = true,
                    StaysOpen = false,
                };
                return;
            }

            if (existingNames.Contains(updatedName))
            {
                SelectedLinkName.ToolTip = new ToolTip
                {
                    Content = "\"" + updatedName + "\" already exists",
                    IsOpen = true,
                    StaysOpen = false,
                };
                return;
            }
            SelectedLinkName.ToolTip = null;
            SelectedLink.Name = updatedName;
            MergeAndUpdate();
        }

        private void OnResetButtonClick(object sender, RoutedEventArgs e)
        {
            FillSelectedLinkBoxes(SelectedLink);
        }

        private void OnRadioButtonClick(object sender, RoutedEventArgs e)
        {
            MergeAndUpdate();
            ClearLinkBoxes();
            SelectedLink = null;
        }

        private static string ShortenStringForLabel(string text, int numCharacters)
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

        private static TextBlock BuildTextBlock(string boldBit, string regularBit)
        {
            TextBlock block = new TextBlock();
            block.Inlines.Add(new Bold(new Run(boldBit)));
            block.Inlines.Add(regularBit);
            return block;
        }

        private void ConfigureLabels()
        {
            string shortAssemblyName = ShortenStringForLabel(AssemblyName, MAX_BUTTON_CHARACTER_WIDTH);

            string longCSVFilename = ShortenStringForLabel(CSVFileName, MAX_LABEL_CHARACTER_WIDTH);
            string shortCSVFilename = ShortenStringForLabel(CSVFileName, MAX_BUTTON_CHARACTER_WIDTH);

            MatchedListLabel.Content = BuildTextBlock("Matching Links from CSV: ", longCSVFilename);
            ExistingTreeLabel.ToolTip =
                new TextBlock { Text = "Configuration from Assembly: " + AssemblyName };

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
        private static void ProcessDragDropOnItem(TreeViewItem target, TreeViewItem package, int position = -1)
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

        private static bool IsPointToSideOfElement(TreeViewItem item, Point pointOnElement)
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
        private static TreeViewItem GetItemToSideOfPoint(URDFTreeView tree, DragEventArgs e)
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
        private void ProcessDragDropOnTree(URDFTreeView tree, TreeViewItem package, DragEventArgs e)
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
                ProcessDragDropOnItem(closest, package, 0);
            }
            else
            {
                // If the closest was found, then add it to its parent at the appropriate index
                TreeViewItem parent = (TreeViewItem)closest.Parent;
                int closestIndex = parent.Items.IndexOf(closest);
                ProcessDragDropOnItem(parent, package, closestIndex + 1);
            }
        }

        private void TreeViewDrop(object sender, DragEventArgs e)
        {
            URDFTreeView tree = (URDFTreeView)sender;
            TreeViewItem package = e.Data.GetData(typeof(TreeViewItem)) as TreeViewItem;

            if (!IsValidDrop(tree, package, e))
            {
                return;
            }

            if (e.Source.GetType() == typeof(TreeViewItem))
            {
                // Dropping onto a Tree node
                ProcessDragDropOnItem((TreeViewItem)e.Source, package);
            }
            else if (e.Source.GetType() == typeof(TreeView))
            {
                // Dropping outside of a node will reorder nodes
                ProcessDragDropOnTree(tree, package, e);
            }
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