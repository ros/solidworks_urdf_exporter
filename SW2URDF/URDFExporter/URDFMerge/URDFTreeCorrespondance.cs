using SW2URDF.UI;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Windows.Controls;

namespace SW2URDF.URDFMerge
{
    public class URDFTreeCorrespondance
    {
        private readonly Dictionary<TreeViewItem, Link> ItemToLink;

        public URDFTreeCorrespondance()
        {
            ItemToLink = new Dictionary<TreeViewItem, Link>();
        }

        public void BuildCorrespondance(URDFTreeView existingTree, List<Link> loadedLinks, out List<Link> matchedLinks, out List<Link> unmatchedLinks)
        {
            List<TreeViewItem> existingItemsList = existingTree.Flatten();

            ItemToLink.Clear();

            Dictionary<string, TreeViewItem> existingNameToItemsLookup = new Dictionary<string, TreeViewItem>();
            foreach (TreeViewItem item in existingItemsList)
            {
                existingNameToItemsLookup[item.Name] = item;
            }

            Dictionary<string, Link> loadedNameToLinkLookup = new Dictionary<string, Link>();
            foreach (Link link in loadedLinks)
            {
                loadedNameToLinkLookup[link.Name] = link;
            }

            matchedLinks = new List<Link>();
            unmatchedLinks = new List<Link>();
            foreach (KeyValuePair<string, Link> entry in loadedNameToLinkLookup)
            {
                Link loadedLink = entry.Value;
                if (existingNameToItemsLookup.TryGetValue(entry.Key, out TreeViewItem item))
                {
                    matchedLinks.Add(loadedLink);
                    ItemToLink[item] = loadedLink;
                }
                else
                {
                    unmatchedLinks.Add(loadedLink);
                }
            }
        }

        public Link GetCorrespondingLink(TreeViewItem item)
        {
            ItemToLink.TryGetValue(item, out Link link);
            return link;
        }
    }
}