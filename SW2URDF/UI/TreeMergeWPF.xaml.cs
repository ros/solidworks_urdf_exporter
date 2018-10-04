using log4net;
using SW2URDF.CSV;
using SW2URDF.URDF;
using SW2URDF.URDFMerge;
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

        private static readonly int MAX_LABEL_CHARACTER_WIDTH = 40;
        private static readonly int MAX_BUTTON_CHARACTER_WIDTH = 20;

        private readonly string CSVFileName;
        private readonly string AssemblyName;

        private readonly URDFTreeCorrespondance TreeCorrespondance;

        private readonly List<Link> LoadedCSVLinks;
        private readonly HashSet<string> LoadedCSVLinkNames;
        private Link SelectedCSVLink;
        public ObservableCollection<KeyValuePair<string, object>> SelectedLinkProperties { get; set; }

        public TreeMergeWPF(List<string> coordinateSystems, List<string> referenceAxes, string csvFileName, string assemblyName)
        {
            DataContext = this;
            Resources["SelectedLinkProperties"] = SelectedLinkProperties;
            Dispatcher.UnhandledException += App_DispatcherUnhandledException;

            CSVFileName = csvFileName;
            AssemblyName = assemblyName;

            InitializeComponent();
            ConfigureLabels();

            LoadedCSVLinks = new List<Link>();
            LoadedCSVLinkNames = new HashSet<string>();
            TreeCorrespondance = new URDFTreeCorrespondance();
            SelectedLinkProperties = new ObservableCollection<KeyValuePair<string, object>>();
        }

        private void App_DispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            logger.Error("Exception encountered in TreeMerge form", e.Exception);
            MessageBox.Show("There was a problem with the TreeMerge form: \n\"" +
                e.Exception.Message + "\"\nEmail your maintainer with the log file found at " +
                Logger.GetFileName());
            e.Handled = true;
        }

        public void SetMergeTree(LinkNode existingLink, List<Link> loadedLinks)
        {
            ExistingTreeView.SetTree(existingLink);
            LoadedCSVLinks.Clear();
            LoadedCSVLinks.AddRange(loadedLinks);

            PropertiesListView.DataContext = SelectedLinkProperties;
            UpdateForm();

            ExistingTreeView.SelectedItemChanged += OnTreeItemClick;
        }

        private void UpdateForm()
        {
            TreeCorrespondance.BuildCorrespondance(ExistingTreeView, LoadedCSVLinks, out List<Link> matched, out List<Link> unmatched);
            UpdateList(MatchingLoadedLinks, matched);
            UpdateList(UnmatchedLoadedLinks, unmatched);

            LoadedCSVLinkNames.Clear();
            LoadedCSVLinkNames.UnionWith(LoadedCSVLinks.Select(link => link.Name));
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
            TreeMerger merger = new TreeMerger(MassInertiaLoadedButton.IsChecked.Value,
                                                         VisualLoadedButton.IsChecked.Value,
                                                         JointKinematicsLoadedButton.IsChecked.Value,
                                                         OtherJointLoadedButton.IsChecked.Value);

            string whyNotMerge = "";// merger.CanTreesBeMerged(ExistingTreeView, LoadedTreeView);
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

            //URDFTreeView result = merger.Merge(ExistingTreeView, LoadedTreeView);

            //if (result != null)
            //{
            //    TreeMergedEventArgs mergedArgs = new TreeMergedEventArgs(result, true, merger);
            //    TreeMerged(this, mergedArgs);
            //}
            //else
            //{
            //    TreeMerged(this, new TreeMergedEventArgs());
            //}

            Close();
        }

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

        private void OnListBoxItemClick(object sender, RoutedEventArgs e)
        {
            ListBoxItem item = (sender as ListBoxItem);
            if (item != null)
            {
                SelectedLinkName.IsReadOnly = false;
                SelectedCSVLink = (Link)item.Tag;
                FillSelectedLinkBoxes(SelectedCSVLink);
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
                SelectedLinkName.IsReadOnly = true;
                FillSelectedLinkBoxes(link);
            }
        }

        private bool IsValidLinkName(string name)
        {
            if (!string.IsNullOrWhiteSpace(name))
            {
                return false;
            }

            if (LoadedCSVLinkNames.Contains(name))
            {
                return false;
            }

            return true;
        }

        private void OnUpdateButtonClick(object sender, RoutedEventArgs e)
        {
            string updatedName = SelectedLinkName.Text;
            if (!string.IsNullOrWhiteSpace(updatedName))
            {
                SelectedCSVLink.Name = updatedName;
                UpdateForm();
            }
        }

        private void OnResetButtonClick(object sender, RoutedEventArgs e)
        {
            FillSelectedLinkBoxes(SelectedCSVLink);
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
            MatchedListLabel.Content = BuildTextBlock("Matching Links from CSV: ", longCSVFilename);

            MassInertiaExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            VisualExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            JointKinematicsExistingButton.Content = new TextBlock { Text = shortAssemblyName };
            OtherJointExistingButton.Content = new TextBlock { Text = shortAssemblyName };

            MassInertiaLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            VisualLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            JointKinematicsLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
            OtherJointLoadedButton.Content = new TextBlock { Text = shortCSVFilename };
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