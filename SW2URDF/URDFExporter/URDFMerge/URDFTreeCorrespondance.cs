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