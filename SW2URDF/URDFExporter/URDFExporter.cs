/*
Copyright (c) 2015 Stephen Brawner



Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:



The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using System;
using System.IO;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System.Collections.Generic;
using System.Xml.Serialization;
using MathNet.Numerics.LinearAlgebra.Generic;
using log4net;
using log4net.Repository.Hierarchy;
using log4net.Appender;
using System.Linq;

namespace SW2URDF
{
    // This class contains a long list of methods that are used throughout the export process. 
    // Methods for building links and joints are contained in here.
    // Many of the methods are overloaded, but seek to reduce repeated code as much as possible 
    // (i.e. the overloaded methods call eachother).
    // These methods are used by the PartExportForm, the AssemblyExportForm and the PropertyManager Page
    public partial class URDFExporter
    {
        #region class variables
        private static readonly log4net.ILog logger = Logger.GetLogger();
        
        [XmlIgnore]
        public ISldWorks iSwApp = null;
        [XmlIgnore]
        private bool mBinary;
        private bool mshowInfo;
        private bool mSTLPreview;
        private bool mTranslateToPositive;
        private bool mSaveComponentsIntoOneFile;
        private int mSTLUnits;
        private int mSTLQuality;
        private double mHideTransitionSpeed;

        private UserProgressBar progressBar;

        [XmlIgnore]
        public ModelDoc2 ActiveSWModel;
        [XmlIgnore]
        public MathUtility swMath;
        [XmlIgnore]
        public Object swMathPID
        { get; set; }

        public robot mRobot
        { get; set; }
        public string mPackageName
        { get; set; }
        public string mSavePath
        { get; set; }
        public List<link> mLinks
        { get; set; }

        #endregion

        // Constructor for SW2URDF Exporter class
        public URDFExporter(ISldWorks iSldWorksApp)
        {
            constructExporter(iSldWorksApp);
            iSwApp.GetUserProgressBar(out progressBar);
            mSavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            mPackageName = ActiveSWModel.GetTitle();
            
        }

        private void constructExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;
            swMath = iSwApp.GetMathUtility();
        }

        #region Export Methods

        // Beginning method for exporting the full package
        public void exportRobot(bool exportSTL = true)
        {
            //Setting up the progress bar
            logger.Info("Beginning the export process");
            int progressBarBound = Common.getCount(mRobot.BaseLink);
            progressBar.Start(0, progressBarBound, "Creating package directories");

            //Creating package directories
            logger.Info("Creating package directories with name " + mPackageName + " and save path " + mSavePath);
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            mRobot.name = mPackageName;
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.name + ".urdf";
            string windowsPackageXMLFileName = package.WindowsPackageDirectory + "package.xml";

            //Create CMakeLists
            logger.Info("Creating CMakeLists.txt at " + package.WindowsCMakeLists);
            package.createCMakeLists();

            //Create Config joint names, not sure how this is used...
            logger.Info("Creating joint names config at " + package.WindowsConfigYAML);
            package.createConfigYAML(mRobot.getJointNames(false));

            //Creating package.xml file
            logger.Info("Creating package.xml at " + windowsPackageXMLFileName);
            PackageXMLWriter packageXMLWriter = new PackageXMLWriter(windowsPackageXMLFileName);
            PackageXML packageXML = new PackageXML(mPackageName);
            packageXML.writeElement(packageXMLWriter);

            //Creating RVIZ launch file
            Rviz rviz = new Rviz(mPackageName, mRobot.name + ".urdf");
            logger.Info("Creating RVIZ launch file in " + package.WindowsLaunchDirectory);
            rviz.writeFiles(package.WindowsLaunchDirectory);

            //Creating Gazebo launch file
            Gazebo gazebo = new Gazebo(this.mRobot.name, this.mPackageName, mRobot.name + ".urdf");
            logger.Info("Creating Gazebo launch file in " + package.WindowsLaunchDirectory);
            gazebo.writeFile(package.WindowsLaunchDirectory);


            //Customizing STL preferences to how I want them
            logger.Info("Saving existing STL preferences");
            saveUserPreferences();

            logger.Info("Modifying STL preferences");
            setSTLExportPreferences();

            //Saving part as STL mesh
            AssemblyDoc assyDoc = (AssemblyDoc)ActiveSWModel;
            List<string> hiddenComponents = Common.findHiddenComponents(assyDoc.GetComponents(false));
            logger.Info("Found " + hiddenComponents.Count + " hidden components " + String.Join(", ", hiddenComponents));
            logger.Info("Hiding all components");
            ActiveSWModel.Extension.SelectAll();
            ActiveSWModel.HideComponent2();

            string filename = exportFiles(mRobot.BaseLink, package, 0, exportSTL);

            logger.Info("Beginning individual files export");
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = filename;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = filename;

            logger.Info("Showing all components except previously hidden components");
            Common.showAllComponents(ActiveSWModel, hiddenComponents);
            //Writing URDF to file

            logger.Info("Writing URDF file to " + windowsURDFFileName);
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            mRobot.writeURDF(uWriter.writer);

            logger.Info("Copying log file");
            copyLogFile(package);

            logger.Info("Resetting STL preferences");
            resetUserPreferences();
            progressBar.End();
        }

        //Recursive method for exporting each link (and writing it to the URDF)
        public string exportFiles(link Link, URDFPackage package, int count, bool exportSTL = true)
        {
            progressBar.UpdateProgress(count);
            progressBar.UpdateTitle("Exporting mesh: " + Link.name);
            logger.Info("Exporting link: " + Link.name);
            // Iterate through each child and export its files
            logger.Info("Link " + Link.name + " has " + Link.Children.Count + " children");
            foreach (link child in Link.Children)
            {
                count += 1;
                if (!child.isFixedFrame)
                {
                    string filename = exportFiles(child, package, count, exportSTL);
                    child.Visual.Geometry.Mesh.filename = filename;
                    child.Collision.Geometry.Mesh.filename = filename;
                }
            }
            
            // Copy the texture file (if it was specified) to the textures directory
            if (!Link.isFixedFrame && Link.Visual.Material.Texture.wFilename != "")
            {
                if (System.IO.File.Exists(Link.Visual.Material.Texture.wFilename))
                {
                    Link.Visual.Material.Texture.filename = 
                        package.TexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    string textureSavePath = 
                        package.WindowsTexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    System.IO.File.Copy(Link.Visual.Material.Texture.wFilename, textureSavePath, true);
                }
            }

            // Create the mesh filenames. SolidWorks likes to use / but that will get messy in filenames so use _ instead
            string linkName = Link.name.Replace('/', '_');
            string meshFileName = package.MeshesDirectory + linkName + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + linkName + ".STL";
            // Export STL
            if (exportSTL)
            {
                saveSTL(Link, windowsMeshFileName);
            }

            return meshFileName;
        }

        private void saveSTL(link Link, string windowsMeshFileName)
        {
            int errors = 0;
            int warnings = 0;

            string coordsysName = "";
            coordsysName =
                (Link.Joint == null || Link.Joint.CoordinateSystemName == null)
                ? Link.CoordSysName : Link.Joint.CoordinateSystemName;
            logger.Info(Link.name + ": Exporting STL with coordinate frame " + coordsysName);

            Dictionary<string, string> names = GetComponentRefGeoNames(coordsysName);
            ModelDoc2 ActiveDoc = ActiveSWModel;

            string ComponentName = "";
            string ConfigurationName = "";
            string DisplayStateName = "";
            Component2 geoComponent = default(Component2);
            logger.Info(Link.name + ": Reference geometry name " + names["component"]);
            if (names["component"].Length > 0)
            {
                foreach (Component2 comp in Link.SWcomponents)
                {
                    if (comp.Name2 == names["component"])
                    {
                        geoComponent = comp;
                        ComponentName = comp.GetPathName();
                        ConfigurationName = comp.ReferencedConfiguration;
                        DisplayStateName = comp.ReferencedDisplayState;
                        bool usenamed = comp.UseNamedConfiguration;
                        ActiveDoc = (ModelDoc2)iSwApp.ActivateDoc3(ComponentName, false, 0, 0);

                        Configuration config = ActiveDoc.GetConfigurationByName(ConfigurationName);
                        ActiveDoc.ShowConfiguration2(ConfigurationName);
                        config.ApplyDisplayState(DisplayStateName);
                    }
                    break;
                }
            }

            if (ComponentName.Length == 0)
            {
                Common.showComponents(ActiveSWModel, Link.SWcomponents);
            }

            int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
            setLinkSpecificSTLPreferences(names["geo"], Link.STLQualityFine, ActiveDoc);

            logger.Info("Saving STL to " + windowsMeshFileName);
            ActiveDoc.Extension.SaveAs(windowsMeshFileName, 
                (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);
            if (ComponentName.Length > 0)
            {
                iSwApp.CloseDoc(ComponentName);
                geoComponent.ReferencedConfiguration = ConfigurationName;
            }
            else
            {
                Common.hideComponents(ActiveSWModel, Link.SWcomponents);
            }


            correctSTLMesh(windowsMeshFileName);
        }

        
        // Used only by the part exporter
        public void exportLink(bool zIsUp)
        {
            
            createBaseRefOrigin(zIsUp);
            MathTransform coordSysTransform = 
                ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
            Matrix<double> GlobalTransform = ops.getTransformation(coordSysTransform);

            localizeLink(mRobot.BaseLink, GlobalTransform);

            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string meshFileName = package.MeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.name + ".urdf";
            string windowsManifestFileName = package.WindowsPackageDirectory + "manifest.xml";

            //Creating manifest file
            PackageXMLWriter manifestWriter = new PackageXMLWriter(windowsManifestFileName);
            PackageXML Manifest = new PackageXML(mRobot.name);
            Manifest.writeElement(manifestWriter);

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();
            setLinkSpecificSTLPreferences("", mRobot.BaseLink.STLQualityFine, ActiveSWModel);
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh

            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, 
                (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = meshFileName;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = meshFileName;

            correctSTLMesh(windowsMeshFileName);

            mRobot.BaseLink.Visual.Material.Texture.filename = 
                package.TexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
            string textureSavePath = 
                package.WindowsTexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
            if (mRobot.BaseLink.Visual.Material.Texture.wFilename != "")
            {
                System.IO.File.Copy(mRobot.BaseLink.Visual.Material.Texture.wFilename, textureSavePath, true);
            }

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            //mRobot.addLink(mLink);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
        }

        //Writes an empty header to the STL to get rid of the BS that SolidWorks adds to a binary STL file
        public void correctSTLMesh(string filename)
        {
            logger.Info("Removing SW header in STL file");
            FileStream fileStream = new FileStream(filename, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
            byte[] emptyHeader = new byte[80];
            fileStream.Write(emptyHeader, 0, emptyHeader.Length);
            fileStream.Close();
        }
        #endregion

        private void copyLogFile(URDFPackage package)
        {
            string destination = package.WindowsPackageDirectory + "export.log";
            string log_filename = Logger.GetFileName();

            if (log_filename != null)
            {
                if (!File.Exists(log_filename))
                {
                    System.Windows.Forms.MessageBox.Show("The log file was expected to be located at " + log_filename +
                        ", but it was not found. Please contact your maintainer with this error message.");
                }
                else
                {
                    logger.Info("Copying " + log_filename + " to " + destination);
                    System.IO.File.Copy(log_filename, destination);
                }
            }
        }

        #region STL Preference shuffling

        //Saves the preferences that the user had setup so that I can change them and revert back to their configuration
        public void saveUserPreferences()
        {
            logger.Info("Saving users preferences");
            mBinary = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat);
            mTranslateToPositive = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive);
            mSTLUnits = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits);
            mSTLQuality = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality);
            mshowInfo = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave);
            mSTLPreview = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview);
            mHideTransitionSpeed = iSwApp.GetUserPreferenceDoubleValue((int)swUserPreferenceDoubleValue_e.swViewTransitionHideShowComponent);
            mSaveComponentsIntoOneFile = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLComponentsIntoOneFile);
        }

        //This is how the STL export preferences need to be to properly export
        public void setSTLExportPreferences()
        {
            logger.Info("Setting STL preferences");
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, true);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive, true);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, 2);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, (int)swSTLQuality_e.swSTLQuality_Coarse);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, false);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, false);
            iSwApp.SetUserPreferenceDoubleValue((int)swUserPreferenceDoubleValue_e.swViewTransitionHideShowComponent, 0);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLComponentsIntoOneFile, true);
        }

        //This resets the user preferences back to what they were.
        public void resetUserPreferences()
        {
            logger.Info("Returning STL preferences to user preferences");
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, mBinary);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive, mTranslateToPositive);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, mSTLUnits);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, mSTLQuality);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, mshowInfo);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, mSTLPreview);
            iSwApp.SetUserPreferenceDoubleValue((int)swUserPreferenceDoubleValue_e.swViewTransitionHideShowComponent, mHideTransitionSpeed);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLComponentsIntoOneFile, mSaveComponentsIntoOneFile);
        }

        //If the user selected something specific for a particular link, that is handled here.
        public void setLinkSpecificSTLPreferences(string CoordinateSystemName, bool qualityFine, ModelDoc2 doc)
        {
            doc.Extension.SetUserPreferenceString((int)swUserPreferenceStringValue_e.swFileSaveAsCoordinateSystem, 
                (int)swUserPreferenceOption_e.swDetailingNoOptionSpecified, CoordinateSystemName);
            if (qualityFine)
            {
                iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, (int)swSTLQuality_e.swSTLQuality_Fine);
            }
            else
            {
                iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, (int)swSTLQuality_e.swSTLQuality_Coarse);
            }
        }
        #endregion
    }
}