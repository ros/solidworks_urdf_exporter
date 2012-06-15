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
using System.Windows.Forms;

namespace SW2URDF
{
    public class SW2URDFExporter
    {
         #region Local variables
        ISldWorks iSwApp = null;
        ICommandManager iCmdMgr = null;

        public robot mRobot
        {get; set;}
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
            mPackageName = ActiveSWModel.GetTitle();   
        }

        public void createRobotFromActiveModel()
        {
            mRobot = new robot();

            int modelType = ActiveSWModel.GetType();
            if (modelType == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                mRobot.BaseLink = getBaseLinkFromActiveModel();
            }
            else if (modelType == (int)swDocumentTypes_e.swDocPART)
            {
                mRobot.BaseLink = getLinkFromActiveModel();
            }

        }

        public link getBaseLinkFromAssy(ModelDoc2 swModel)
        {
            AssemblyDoc swAssy = (AssemblyDoc)swModel;
            varComp = (object[])swAssy.GetComponents(true);

            link baseLink = assignParentLinkFromChildren(varComp, swModel);
            
            foreach (IComponent comp in varComp)
            {
                baseLink.Children.Add(getLinkFromComp(comp));
            }

            return baseLink;
        }

        public link getBaseLinkFromActiveModel()
        {
            return getBaseLinkFromAssy(ActiveSWModel);
        }

        public link getLinkFromComp(object comp)
        {
            IComponent parentComp = (IComponent)comp;
            object[] children = parentComp.GetChildren();

            link parent = assignParentLinkFromChildren(children, parentComp.GetModelDoc());

            foreach (object child in children)
            {
                parent.Children.Add(getLinkFromComp(child));
            }

            return parent;
        }

        public link assignParentLinkFromChildren(object[] children, ModelDoc2 ParentDoc)
        {
            if (ParentDoc.GetType() == (int)swDocumentTypes_e.swDocPART)
            {
                return getLinkFromPartModel(ParentDoc);
            }
            else
            {
                IComponent ParentComp;
                foreach (IComponent child in children)
                {
                    if (!child.IsHidden(true))
                    {
                        ParentComp = child;
                    }
                }
                ModelDoc2 modeldoc = ParentDoc;

                int priorityLevel = -1;
                // Iteratively going through SolidWorks component structure to find the 'best' choice for the parent link
                while (priorityLevel < 0)
                {
                    double largestFixedVolume = 0;
                    double largestPartVolume = 0;
                    double largestAssyVolume = 0;
                    
                    ParentDoc = ParentComp.GetModelDoc();
                    int ParentType = ParentDoc.GetType();
                    int c = 1;
                    foreach (IComponent child in children)
                    {
                        ModelDoc2 ChildDoc = child.GetModelDoc();
                        int ChildType = (int)ChildDoc.GetType();
                        IMassProperty childMass = ChildDoc.Extension.CreateMassProperty();
                        double[] bb = child.GetBox(false, false);
                        double childBBVolume = boundingBoxVolume(bb);

                        //Highest priority is the largest fixed component
                        if (child.IsFixed() && childMass.Volume > largestFixedVolume)
                        {
                            priorityLevel = 2;
                            ParentComp = child;
                            largestFixedVolume = childBBVolume;
                        }
                        //Second highest priority is the largest floating part
                        else if (childMass.Volume > largestPartVolume && ChildType == (int)swDocumentTypes_e.swDocPART && priorityLevel < 2)
                        {
                            priorityLevel = 1;
                            ParentComp = child;
                            largestPartVolume = childBBVolume;
                        }
                        //Third priority is the 'best' choice from the largest assembly
                        else if (childMass.Volume > largestAssyVolume && ChildType == (int)swDocumentTypes_e.swDocASSEMBLY && priorityLevel < 1)
                        {
                            priorityLevel = 0;
                            ParentComp = child;
                            largestAssyVolume = childBBVolume;
                        }
                    }


                    ParentDoc = ParentComp.GetModelDoc();
                    ParentType = ParentDoc.GetType();
                    // If a fixed component was found that is an assembly, its children will be iterated through on the next run
                    if (priorityLevel == 2 && ParentType == (int)swDocumentTypes_e.swDocASSEMBLY)
                    {
                        priorityLevel = -1;
                        children = ParentComp.GetChildren();
                    }
                    // If no parts were found, then the largest assembly will be iterated through to find the best choice
                    else if (priorityLevel == 0)
                    {
                        priorityLevel = -1;
                        children = ParentComp.GetChildren();
                    }
                    // Otherwise, if a part was finally selected for parent status, the parentdoc is selected and it is converted into a link
                }
                if (ParentDoc.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
                {
                    throw new System.InvalidOperationException("Parent link cannot be made from assembly");
                }
                return getLinkFromPartModel(ParentDoc);
            }
        }

        public double boundingBoxVolume(double[] bb)
        {
            return ((bb[3] - bb[0]) * (bb[4] - bb[1]) * (bb[5] - bb[2]));
        }

        
        #region Part Exporting methods
        public link getLinkFromPartModel(ModelDoc2 swModel)
        {
            link Link = new link();
            Link.name = swModel.GetTitle();
            
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
            return getLinkFromPartModel(ActiveSWModel);
        }


        public void exportLink()
        {
            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string meshFileName = package.MeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.BaseLink.name + ".URDF";

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh
            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = meshFileName;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = meshFileName;

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            //mRobot.addLink(mLink);
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
