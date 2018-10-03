using SolidWorks.Interop.sldworks;
using SW2URDF.UI;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls;

namespace SW2URDF.URDFMerge
{
    public class TreeMerger
    {
        public bool UseCSVInertial;
        public bool UseCSVVisualCollision;
        public bool UseCSVJointKinematics;
        public bool UseCSVJointOther;

        /// <summary>
        /// Helper class to Merge two URDFTreeViews
        /// </summary>
        /// <param name="useCSVInertial">Use loaded values for MoI and CoM properties of a link </param>
        /// <param name="useCSVVisualCollision">Use loaded values for meshes and material properties</param>
        /// <param name="useCSVJointKinematics">Use loaded values for joint coordinate system, joint
        /// axis and joint type</param>
        /// <param name="useCSVJointOther">Use loaded values for Joint Limits, Calibration, Dynamics,
        /// and Safety Controller</param>
        public TreeMerger(bool useCSVInertial, bool useCSVVisualCollision,
            bool useCSVJointKinematics, bool useCSVJointOther)
        {
            UseCSVInertial = useCSVInertial;
            UseCSVVisualCollision = useCSVVisualCollision;
            UseCSVJointKinematics = useCSVJointKinematics;
            UseCSVJointOther = useCSVJointOther;
        }

        public URDFTreeView Merge(TreeView cadTree, URDFTreeCorrespondance correspondance)
        {
            URDFTreeView merged = new URDFTreeView();

            foreach (TreeViewItem item in MergeItems(cadTree.Items, correspondance))
            {
                merged.Items.Add(item);
            }

            return merged;
        }

        public List<TreeViewItem> MergeItems(ItemCollection cadCollection, URDFTreeCorrespondance correspondance)
        {
            List<TreeViewItem> merged = new List<TreeViewItem>();
            List<TreeViewItem> cadItems = cadCollection.Cast<TreeViewItem>().ToList();

            foreach (TreeViewItem item in cadCollection)
            {
                TreeViewItem mergedItem = MergeItem(item, correspondance);
                merged.Add(mergedItem);
            }

            return merged;
        }

        private TreeViewItem MergeItem(TreeViewItem cadItem, URDFTreeCorrespondance correspondance)
        {
            TreeViewItem merged = new TreeViewItem();
            Link cadLink = (Link)cadItem.Tag;
            Link csvLink = correspondance.GetCorrespondingLink(cadItem);

            Link mergedLink = MergeLink(cadLink, csvLink);

            merged.Tag = mergedLink;

            List<TreeViewItem> children = MergeItems(cadItem.Items, correspondance);
            foreach (TreeViewItem child in children)
            {
                merged.Items.Add(child);
            }
            return merged;
        }

        private Link MergeLink(Link cadLink, Link csvLink)
        {
            if (csvLink == null)
            {
                return cadLink;
            }

            Link mergedLink = cadLink.Clone();
            // SolidWorks components won't be loaded from the file. Use the components in the model
            mergedLink.SWMainComponent = cadLink.SWMainComponent;
            mergedLink.SWcomponents = new List<Component2>(cadLink.SWcomponents);

            if (UseCSVInertial)
            {
                mergedLink.Inertial.SetElement(csvLink.Inertial);
            }

            if (UseCSVVisualCollision)
            {
                mergedLink.Visual.SetElement(csvLink.Visual);
                mergedLink.Collision.SetElement(csvLink.Collision);
            }

            if (UseCSVJointKinematics)
            {
                mergedLink.Joint.Origin.SetElement(csvLink.Joint.Origin);
                mergedLink.Joint.CoordinateSystemName = csvLink.Joint.CoordinateSystemName;

                mergedLink.Joint.Axis.SetElement(csvLink.Joint.Axis);
                mergedLink.Joint.AxisName = csvLink.Joint.AxisName;

                mergedLink.Joint.Type = csvLink.Joint.Type;
            }

            if (UseCSVJointOther)
            {
                mergedLink.Joint.Limit.SetElement(csvLink.Joint.Limit);
                mergedLink.Joint.Calibration.SetElement(csvLink.Joint.Calibration);
                mergedLink.Joint.Dynamics.SetElement(csvLink.Joint.Dynamics);
                mergedLink.Joint.Safety.SetElement(csvLink.Joint.Safety);
            }

            return mergedLink;
        }
    }
}