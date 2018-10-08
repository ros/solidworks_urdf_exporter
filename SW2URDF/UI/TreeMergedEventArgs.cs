using SW2URDF.URDFExport.URDFMerge;
using System;

namespace SW2URDF.UI
{
    public class TreeMergedEventArgs : EventArgs
    {
        public readonly URDFTreeView MergedTree;
        public readonly bool Success;
        public readonly bool UsedCSVInertial;
        public readonly bool UsedCSVVisualCollision;
        public readonly bool UsedCSVJointKinematics;
        public readonly bool UsedCSVJointOther;
        public readonly string CSVFilename;

        public TreeMergedEventArgs()
        {
            Success = false;
        }

        public TreeMergedEventArgs(URDFTreeView mergedTree, bool success, TreeMerger merger, string csvFilename)
        {
            MergedTree = mergedTree;
            Success = success;
            UsedCSVInertial = merger.UseCSVInertial;
            UsedCSVVisualCollision = merger.UseCSVVisualCollision;
            UsedCSVJointKinematics = merger.UseCSVJointKinematics;
            UsedCSVJointOther = merger.UseCSVJointOther;
            CSVFilename = csvFilename;
        }
    }
}