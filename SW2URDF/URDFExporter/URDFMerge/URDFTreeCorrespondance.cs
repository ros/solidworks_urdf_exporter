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

        public void BuildCorrespondance(URDFTreeView left, List<Link> loadedLinks, out List<Link> matched, out List<Link> unmatched)
        {
            List<TreeViewItem> leftList = left.Flatten();

            ItemToLink.Clear();

            Dictionary<string, TreeViewItem> existingItemsLookup = new Dictionary<string, TreeViewItem>();
            foreach (TreeViewItem item in leftList)
            {
                existingItemsLookup[item.Name] = item;
            }

            Dictionary<string, Link> loadedLinksLookup = new Dictionary<string, Link>();
            foreach (Link link in loadedLinks)
            {
                loadedLinksLookup[link.Name] = link;
            }

            matched = new List<Link>();
            foreach (KeyValuePair<string, TreeViewItem> entry in existingItemsLookup)
            {
                if (loadedLinksLookup.TryGetValue(entry.Key, out Link link))
                {
                    matched.Add(link);
                    ItemToLink[entry.Value] = link;
                }
            }

            unmatched = new List<Link>();
            foreach (KeyValuePair<string, Link> entry in loadedLinksLookup)
            {
                if (!existingItemsLookup.ContainsKey(entry.Key))
                {
                    unmatched.Add(entry.Value);
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