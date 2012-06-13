using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections;
using System.Reflection;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;
using SolidWorksTools;
using SolidWorksTools.File;
using System.Collections.Generic;
using System.Diagnostics;

namespace SW2URDF
{
    class SW2URDFExporter
    {
         #region Local variables
        ISldWorks iSwApp = null;
        ICommandManager iCmdMgr = null;

        public robot mRobot
        {get; set;}
        public link mLink
        { get; set; }
        public string mPackageName
        { get; set; }
        public string mSavePath
        { get; set;}
        private bool mBinary;
        private int mSTLUnits;
        private int mSTLQuality;
        private bool mshowInfo;
        private bool mSTLPreview;
        public List<link> mLinks
        { get; set; }
        ModelDoc2 ActiveSWModel;
        object[] varComp;
        #endregion

        public SW2URDFExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = default(ModelDoc2);
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;
            mSavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            mPackageName = ActiveSWModel.FeatureManager.FeatureStatistics.PartName;
            mRobot = new robot();
            createURDFTree();          
        }

        public void createURDFTree()
        {
            int modelType = ActiveSWModel.GetType();
            if ( modelType == (int)swDocumentTypes_e.swDocPART)
            {
                getLinkFromActiveModel();
            }
            else if ( modelType == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                getLinksFromActiveAssy();
            }
        }

        public List<link> getLinksFromAssy(ModelDoc2 swModel)
        {
            List<link> links = new List<link>();

            AssemblyDoc swAssy = (AssemblyDoc)swModel;
            varComp = (object[])swAssy.GetComponents(true);

            for (int i = 0; i < varComp.Length; i++)
            {
                IComponent swComp = default(IComponent);
                swComp = (IComponent)varComp[i];
                links.AddRange(getLinksFromComp(swComp));
            }

            return links;
        }

        public List<link> getLinksFromActiveAssy()
        {
            return getLinksFromAssy(ActiveSWModel);
        }

        public List<link> getLinksFromComp(object comp)
        {
            IComponent c = (IComponent)comp;
            List<link> links = new List<link>();
            object[] children = c.GetChildren();
            int childrenCount = c.IGetChildrenCount();
            link parent = new link();
            parent.name = c.Name2;
            for (int i = 0; i < childrenCount; i++)
            {
                parent.Children.AddRange(getLinksFromComp(children[i]));
            }

            links.Add(parent);
            return links;
        }

        #region Part Exporting methods
        public link getLinkFromPart(ModelDoc2 swModel)
        {
            link Link = new link();
            Link.name = swModel.FeatureManager.FeatureStatistics.PartName;
            
            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            Link.Inertial.Mass.Value = swMass.Mass;
            
            Link.Inertial.Inertia.Moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]
            
            double[] centerOfMass = swMass.CenterOfMass;
            Link.Inertial.Origin.XYZ = centerOfMass;
            Link.Inertial.Origin.RPY = new double[3] {0, 0, 0};
            Link.Visual.Origin.XYZ = centerOfMass;
            Link.Visual.Origin.RPY = new double[3] {0, 0, 0};
            Link.Collision.Origin.XYZ = centerOfMass;
            Link.Collision.Origin.RPY = new double[3] {0, 0, 0};

            return Link;
        }

        public link getLinkFromActiveModel()
        {
            return getLinkFromPart(ActiveSWModel);
        }


        public void exportLink()
        {
            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string meshFileName = package.MeshesDirectory + mLink.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + mLink.name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + mLink.name + ".URDF";

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh
            swModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mLink.Visual.Geometry.Mesh.filename = meshFileName;
            mLink.Collision.Geometry.Mesh.filename = meshFileName;

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            mRobot.addLink(mLink);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
        }
        #endregion


        #region STL Preference shuffling
        public void saveUserPreferences()
        {
            mBinary = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat);
            mSTLUnits = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits);
            mSTLQuality = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality);
            mshowInfo = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave);
            mSTLPreview = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview);
        }

        public void setSTLExportPreferences()
        {
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, true);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, 2);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, (int)swSTLQuality_e.swSTLQuality_Coarse);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, false);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, false);
        }

        public void resetUserPreferences()
        {
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, mBinary);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, mSTLUnits);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, mSTLQuality);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, mshowInfo);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, mSTLPreview);
        }
        #endregion
    }
}
