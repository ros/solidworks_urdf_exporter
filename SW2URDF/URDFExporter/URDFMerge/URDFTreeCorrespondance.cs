using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls;

namespace SW2URDF.URDFMerge
{
    internal class URDFTreeCorrespondance
    {
        private readonly Dictionary<TreeViewItem, TreeViewItem> LeftToRight;
        private readonly Dictionary<TreeViewItem, TreeViewItem> RightToLeft;

        public URDFTreeCorrespondance()
        {
            LeftToRight = new Dictionary<TreeViewItem, TreeViewItem>();
            RightToLeft = new Dictionary<TreeViewItem, TreeViewItem>();
        }

        public static List<TreeViewItem> FlattenTreeView(TreeView tree)
        {
            return FlattenTreeBranch(tree.Items);
        }

        private static List<TreeViewItem> FlattenTreeBranch(ItemCollection items)
        {
            List<TreeViewItem> list = new List<TreeViewItem>();
            foreach (TreeViewItem item in items)
            {
                list.Add(item);
                list.AddRange(FlattenTreeBranch(item.Items));
            }
            return list;
        }

        public void BuildCorrespondance(TreeView left, TreeView right)
        {
            List<TreeViewItem> leftList = FlattenTreeBranch(left.Items);
            List<TreeViewItem> rightList = FlattenTreeBranch(right.Items);

            LeftToRight.Clear();
            RightToLeft.Clear();

            foreach (Tuple<TreeViewItem, TreeViewItem> pair in Enumerable.Zip(leftList, rightList, Tuple.Create))
            {
                LeftToRight[pair.Item1] = pair.Item2;
                RightToLeft[pair.Item2] = pair.Item1;
            }
        }

        public TreeViewItem GetCorrespondingTreeViewItem(TreeViewItem item)
        {
            if (LeftToRight.ContainsKey(item))
            {
                return LeftToRight[item];
            }
            else if (RightToLeft.ContainsKey(item))
            {
                return RightToLeft[item];
            }
            else
            {
                return null;
            }
        }
    }
}