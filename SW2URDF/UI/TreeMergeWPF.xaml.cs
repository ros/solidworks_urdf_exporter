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

        private readonly Link existingBaseLink;
        private readonly List<Link> LoadedCSVLinks;
        private readonly HashSet<string> LoadedCSVLinkNames;
        private Link SelectedCSVLink;
        public ObservableCollection<KeyValuePair<string, object>> SelectedLinkProperties { get; }

        public TreeMergeWPF(Link existingLink, List<Link> loadedLinks, string csvFileName, string assemblyName)
        {
            DataContext = this;
            Resources["SelectedLinkProperties"] = SelectedLinkProperties;
            Dispatcher.UnhandledException += App_DispatcherUnhandledException;

            CSVFileName = csvFileName;
            AssemblyName = assemblyName;

            InitializeComponent();
            ConfigureLabels();

            existingBaseLink = existingLink;
            LoadedCSVLinks = new List<Link>(loadedLinks);
            LoadedCSVLinkNames = new HashSet<string>();
            TreeCorrespondance = new URDFTreeCorrespondance();
            SelectedLinkProperties = new ObservableCollection<KeyValuePair<string, object>>();

            PropertiesListView.DataContext = SelectedLinkProperties;

            ExistingTreeView.SelectedItemChanged += OnTreeItemClick;
            MergeAndUpdate();
        }

        private void App_DispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            logger.Error("Exception encountered in TreeMerge form", e.Exception);
            MessageBox.Show("There was a problem with the TreeMerge form: \n\"" +
                e.Exception.Message + "\"\nEmail your maintainer with the log file found at " +
                Logger.GetFileName());
            e.Handled = true;
        }

        /// <summary>
        /// This method performs a merge between the Loaded links and the existing configuration
        /// based the names of the loaded links. It also updates the form to match the most
        /// updated merge
        /// </summary>
        /// <returns></returns>
        private TreeMerger MergeAndUpdate()
        {
            // Setup merge to start with a fresh link,
            ExistingTreeView.SetTree(existingBaseLink.Clone());
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

            TreeMergedEventArgs mergedArgs = new TreeMergedEventArgs(ExistingTreeView, true, merger);
            TreeMerged(this, mergedArgs);

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
            HashSet<string> existingNames = new HashSet<string>(LoadedCSVLinkNames);
            existingNames.Remove(SelectedCSVLink.Name);

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
            SelectedCSVLink.Name = updatedName;
            MergeAndUpdate();
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