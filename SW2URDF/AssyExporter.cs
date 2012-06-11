using System;
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
    public class AssyExporter
    {
        #region Local variables
        ISldWorks iSwApp = null;
        ICommandManager iCmdMgr = null;

        public List<link> mLinks
        {get; set;}
        private bool mBinary;
        private int mSTLUnits;
        private int mSTLQuality;
        private bool mshowInfo;
        private bool mSTLPreview;

        ModelDoc2 swModel;
        AssemblyDoc swAssy;
        object[] varComp;
        #endregion

        public AssyExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            swModel = default(ModelDoc2);
            swAssy = default(AssemblyDoc);
            swModel = (ModelDoc2)iSwApp.ActiveDoc;
            swAssy = (AssemblyDoc)swModel;
        }

        #region AssyExporter Methods
        public List<link> getLinksFromAssy()
        {
            List<link> links = new List<link>();
            object[] children;
            swModel = default(ModelDoc2);
            swAssy = default(AssemblyDoc);
            swModel = (ModelDoc2)iSwApp.ActiveDoc;
            swAssy = (AssemblyDoc)swModel;
            varComp = (object[])swAssy.GetComponents(true);
            for (int i = 0; i < varComp.Length; i++)
            {
                IComponent swComp = default(IComponent);
                swComp = (IComponent)varComp[i];
                children = swComp.GetChildren();
                links.AddRange(getLinksFromComp(swComp));
            }

            return links;
        }

        public List<link> getLinksFromComp(object comp)
        {
            IComponent c = (IComponent)comp;
            List<link> links = new List<link>();
            object[] children = c.GetChildren();
            int childrenCount = c.IGetChildrenCount();
            link parent = new link();
            parent.name = c.Name2;
            links.Add(parent);
            for (int i = 0; i < childrenCount; i++)
            {
                links.AddRange(getLinksFromComp(children[i]));
            }
            return links;
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
        #endregion
    }
}
