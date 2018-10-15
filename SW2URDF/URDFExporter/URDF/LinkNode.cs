using log4net;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //A LinkNode is derived from a TreeView TreeNode. I've added many new fields to it so
    // that information can be passed around from the TreeView itself.
    public class LinkNode : TreeNode
    {
        private static readonly ILog logger = Logger.GetLogger();

        public Link Link
        { get; set; }

        public bool IsBaseNode
        { get; set; }

        public bool IsIncomplete
        { get; set; }

        public bool NeedsSaving
        { get; set; }

        public string WhyIncomplete
        { get; set; }

        public LinkNode()
        {
            Link = new Link();
        }

        public LinkNode(Link link)
        {
            logger.Info("Building node " + link.Name);

            IsBaseNode = link.Parent == null;
            IsIncomplete = true;
            Link = link;

            Name = Link.Name;
            Text = Link.Name;

            foreach (Link child in link.Children)
            {
                Nodes.Add(new LinkNode(child));
            }
        }

        public Link UpdateLinkTree(Link parent)
        {
            Link.Children.Clear();
            Link.Parent = parent;
            foreach (LinkNode child in Nodes)
            {
                Link.Children.Add(child.UpdateLinkTree(Link));
            }
            return Link;
        }

        public override object Clone()
        {
            LinkNode cloned = (LinkNode)base.Clone();
            cloned.Link = Link.Clone();
            return cloned;
        }

        public Link GetLink()
        {
            Link.Children.Clear();
            foreach (LinkNode child in Nodes)
            {
                Link.Children.Add(child.GetLink());
            }
            return Link;
        }
    }
}