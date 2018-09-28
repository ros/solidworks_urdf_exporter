using SW2URDF.URDF;
using System;
using System.Linq;

namespace SW2URDF
{
    internal class TreeMergeHelper
    {
        public static bool CanMergeLinks(Link link1, Link link2)
        {
            if (!AreLinksEquivelant(link1, link2))
            {
                return false;
            }

            if (link1.Children.Count != link2.Children.Count)
            {
                return false;
            }

            link1.Children.Sort((l1, l2) => string.Compare(l1.Name, l2.Name));
            link2.Children.Sort((l1, l2) => string.Compare(l1.Name, l2.Name));

            foreach (var pair in Enumerable.Zip(link1.Children, link2.Children, Tuple.Create))
            {
                if (!CanMergeLinks(pair.Item1, pair.Item2))
                {
                    return false;
                }
            }

            return true;
        }

        public static bool AreLinksEquivelant(Link link1, Link link2)
        {
            if (link1.Name != link2.Name)
            {
                return false;
            }
            if (link1.Joint.CoordinateSystemName != link2.Joint.CoordinateSystemName)
            {
                return false;
            }

            if (link1.Joint.AxisName != link2.Joint.AxisName)
            {
                return false;
            }

            if (link1.Joint.Type != link2.Joint.Type)
            {
                return false;
            }
            return true;
        }

        public static string ValidateMerge(Link current, Link external,
            bool keepInertial, bool keepVisual, bool keepJointKinematics, bool keepOtherJointValues)
        {
            if (current.Children.Count != external.Children.Count)
            {
                return "Links " + current.Name + " and " + external.Name + " do not have the same number of children";
            }

            foreach (var pair in Enumerable.Zip(current.Children, external.Children, Tuple.Create))
            {
                string result = ValidateMerge(pair.Item1, pair.Item2, keepInertial, keepVisual, keepJointKinematics, keepOtherJointValues);
                if (!string.IsNullOrWhiteSpace(result))
                {
                    return result;
                }
            }

            return null;
        }

        public static Link MergeInfoFromExternalLink(Link current, Link external,
            bool keepInertial, bool keepVisual, bool keepJointKinematics, bool keepOtherJointValues)
        {
            Link merged = new Link();
            merged.Name = current.Name;

            Link linkToMerge = (keepVisual) ? current : external;
            merged.Visual.SetElement(linkToMerge.Visual);
            merged.Collision.SetElement(linkToMerge.Collision);

            linkToMerge = (keepInertial) ? current : external;
            merged.Inertial.SetElement(linkToMerge.Inertial);

            linkToMerge = (keepJointKinematics) ? current : external;
            merged.Joint.SetJointKinematics(linkToMerge.Joint);

            linkToMerge = (keepOtherJointValues) ? current : external;
            merged.Joint.SetJointNonKinematics(linkToMerge.Joint);

            merged.SetSWComponents(current);

            foreach (var pair in Enumerable.Zip(current.Children, external.Children, Tuple.Create))
            {
                Link child = MergeInfoFromExternalLink(pair.Item1, pair.Item2,
                    keepInertial, keepVisual, keepJointKinematics, keepOtherJointValues);
                child.Parent = merged;
                merged.Children.Add(child);
            }

            return merged;
        }
    }
}