using System.Windows.Forms;

namespace SW2URDF.URDFExport
{
    public enum TreeKeyAction { None, AcceptEdit, CancelEdit }

    /// <summary>What a key press in the link tree does. Free of tree nodes so it can be unit tested.</summary>
    public static class TreeKeyActions
    {
        public static TreeKeyAction Decide(bool hasNode, bool isEditing, Keys key)
        {
            if (!hasNode || !isEditing) return TreeKeyAction.None;
            if (key == Keys.Enter) return TreeKeyAction.AcceptEdit;
            if (key == Keys.Escape) return TreeKeyAction.CancelEdit;
            return TreeKeyAction.None;
        }

        /// <summary>The root link has no parent and cannot be removed.</summary>
        public static bool CanRemove(bool hasNode, bool hasParent)
        {
            return hasNode && hasParent;
        }
    }
}