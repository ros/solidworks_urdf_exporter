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
    public class PartExporter
    {
        #region Local variables
        ISldWorks iSwApp = null;
        ICommandManager iCmdMgr = null;

        public link mLink
        {get; set;}
        public string mSavePath
        { get; set;}
        private bool mBinary;
        private int mSTLUnits;
        private int mSTLQuality;
        private bool mshowInfo;
        private bool mSTLPreview;

        ModelDoc2 swModel;
        PartDoc swPart;
        #endregion

        public PartExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            swModel = default(ModelDoc2);
            swPart = default(PartDoc);
            swModel = (ModelDoc2)iSwApp.ActiveDoc;
            swPart = (PartDoc)swModel;
            mLink = getLinkFromPart();
        }


        #region PartExporter Methods

        public link getLinkFromPart()
        {
            link part_link = new link();
            part_link.rgba = new double[4];

            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            part_link.mass = swMass.Mass;
            part_link.moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]
            double[] centerOfMass = swMass.CenterOfMass;
            part_link.origin_inertial = new double[6];
            part_link.origin_visual = new double[6];
            part_link.origin_collision = new double[6];
            for (int i = 0; i < 6; i++)
            {
                if (i < 3)
                {
                    part_link.origin_inertial[i] = centerOfMass[i];
                    part_link.origin_visual[i] = centerOfMass[i];
                    part_link.origin_collision[i] = centerOfMass[i];
                }
                else
                {
                    part_link.origin_inertial[i] = 0;
                    part_link.origin_visual[i] = 0;
                    part_link.origin_collision[i] = 0;
                }
            }

            return part_link;
        }

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

        public void exportLink()
        {
            saveUserPreferences();
            setSTLExportPreferences();
            int errors = 0;
            int warnings = 0;

            string saveDirectory = mSavePath + swModel.FeatureManager.FeatureStatistics.PartName;
            System.Windows.Forms.MessageBox.Show("Saving file to directory\n" + saveDirectory);
            if (!Directory.Exists(saveDirectory))
            {
                Directory.CreateDirectory(saveDirectory);
            }
            string saveName = saveDirectory + "\\" + swModel.FeatureManager.FeatureStatistics.PartName;
            swModel.Extension.SaveAs(saveName + ".STL", (int)swSaveAsVersion_e.swSaveAsCurrentVersion, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mLink.meshName = saveName + ".STL";
            URDFWriter uWriter = new URDFWriter(saveName + ".URDF");
            uWriter.writeURDFFromLink(mLink);

            resetUserPreferences();
        }





        #endregion
    }
}
