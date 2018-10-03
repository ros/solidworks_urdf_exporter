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
            TreeMerger merger = new TreeMerger(MassInertiaLoadedButton.IsChecked.Value,
                                                         VisualLoadedButton.IsChecked.Value,
                                                         JointKinematicsLoadedButton.IsChecked.Value,
                                                         OtherJointLoadedButton.IsChecked.Value);

            string whyNotMerge = merger.CanTreesBeMerged(ExistingTreeView, LoadedTreeView);
            if (!string.IsNullOrWhiteSpace(whyNotMerge))
            {
                string message = "The two configuration trees cannot be merged due to the issues " +
                    "listed below. Modify the configuration in the assembly and/or the csv " +
                    "file. Alternatively, you can remove the configuration from the assembly and load the " +
                    "CSV configuration to skip merging.\r\n\r\n" + whyNotMerge;
                MessageBox.Show(message);
                return;
            }

            if (MessageBox.Show("Do you wish to merge these configuration trees? The configuration in the assembly" +
                " will be overwritten.",
                "Confirm Merge", MessageBoxButton.YesNo) != MessageBoxResult.Yes)
            {
                return;
            }

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