using SW2URDF.UI;
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

        public void BuildCorrespondance(URDFTreeView left, URDFTreeView right)
        {
            List<TreeViewItem> leftList = left.Flatten();
            List<TreeViewItem> rightList = right.Flatten();

            LeftToRight.Clear();
            RightToLeft.Clear();

            foreach (Tuple<TreeViewItem, TreeViewItem> pair in Enumerable.Zip(leftList, rightList, Tuple.Create))
            {
                LeftToRight[pair.Item1] = pair.Item2;
                RightToLeft[pair.Item2] = pair.Item1;
            }
        }

        /// <summary>
        /// Flattens the TreeView tree into a list of Items through a preorder traversal,
        /// starting from the top and proceeding downard.
        /// </summary>
        /// <param name="tree"></param>
        /// <returns></returns>
        public static List<TreeViewItem> FlattenTreeView(TreeView tree)
        {
            return FlattenTreeBranch(tree.Items);
        }

        /// <summary>
        /// This flattens the tree through a preorder traversal from the top downward
        /// </summary>
        /// <param name="tree"></param>
        /// <returns></returns>
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