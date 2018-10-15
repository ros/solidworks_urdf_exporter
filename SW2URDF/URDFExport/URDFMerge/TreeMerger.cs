using SolidWorks.Interop.sldworks;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Windows.Controls;

namespace SW2URDF.URDFExport.URDFMerge
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

        public Link Merge(TreeView cadTree, URDFTreeCorrespondance correspondance)
        {
            if (cadTree.Items.Count != 1)
            {
                return null;
            }

            Link mergedRoot = MergeItem((TreeViewItem)cadTree.Items[0], correspondance);
            return mergedRoot;
        }

        public List<Link> MergeItems(ItemCollection cadCollection, URDFTreeCorrespondance correspondance)
        {
            List<Link> merged = new List<Link>();

            foreach (TreeViewItem item in cadCollection)
            {
                Link mergedItem = MergeItem(item, correspondance);
                merged.Add(mergedItem);
            }

            return merged;
        }

        private Link MergeItem(TreeViewItem cadItem, URDFTreeCorrespondance correspondance)
        {
            TreeViewItem merged = new TreeViewItem();
            Link cadLink = (Link)cadItem.Tag;
            Link csvLink = correspondance.GetCorrespondingLink(cadItem);

            Link mergedLink = MergeLink(cadLink, csvLink);

            List<Link> children = MergeItems(cadItem.Items, correspondance);
            mergedLink.Children.Clear();
            mergedLink.Children.AddRange(children);
            return mergedLink;
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
            mergedLink.SWComponents = new List<Component2>(cadLink.SWComponents);

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