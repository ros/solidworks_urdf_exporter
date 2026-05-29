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

using MathNet.Numerics.LinearAlgebra;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.ROS;
using SW2URDF.URDF;
using SW2URDF.URDFExport.CSV;
using SW2URDF.Utilities;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Windows;
using System.Xml.Serialization;

namespace SW2URDF.URDFExport
{
    // This class contains a long list of methods that are used throughout the export process.
    // Methods for building links and joints are contained in here.
    // Many of the methods are overloaded, but seek to reduce repeated code as much as possible
    // (i.e. the overloaded methods call eachother).
    // These methods are used by the PartExportForm, the AssemblyExportForm and the PropertyManager Page
    public partial class ExportHelper
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
        public Object SWMathPID
        { get; set; }

        public Robot URDFRobot
        { get; set; }

        public string PackageName
        { get; set; }

        public string SavePath
        { get; set; }

        public readonly List<Link> Links;

        private readonly List<string> ReferenceCoordinateSystemNames;
        private readonly List<string> ReferenceAxesNames;

        private bool ComputeInertialValues;
        private bool ComputeVisualCollision;
        private bool ComputeJointKinematics;
        private bool ComputeJointLimits;

        // When true, reference coordinate systems and axes are searched at every component
        // depth, not just the top level assembly and its immediate components. Default off.
        private bool SearchNestedReferenceGeometry;

        // Case-insensitive substring that an enumerated coordinate system / axis name must
        // contain to appear in the dropdowns. Empty means no filtering. Useful to keep the
        // lists manageable when SearchNestedReferenceGeometry is on.
        private string ReferenceGeometryFilter;

        #endregion class variables

        // Constructor for SW2URDF Exporter class
        public ExportHelper(SldWorks iSldWorksApp)
        {
            ConstructExporter(iSldWorksApp);
            iSwApp.GetUserProgressBar(out progressBar);

            SavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            PackageName = ActiveSWModel.GetTitle();

            // Defaults must be set before the initial enumeration below so that out-of-the-box
            // behaviour (top level only, unfiltered) is unchanged.
            SearchNestedReferenceGeometry = false;
            ReferenceGeometryFilter = "";

            ReferenceCoordinateSystemNames = FindRefGeoNames("CoordSys");
            ReferenceAxesNames = FindRefGeoNames("RefAxis");

            ComputeInertialValues = true;
            ComputeVisualCollision = true;
            ComputeJointKinematics = true;
            ComputeJointLimits = true;
        }

        public void SetComputeInertial(bool computeInertial)
        {
            ComputeInertialValues = computeInertial;
        }

        public void SetComputeVisualCollision(bool computeVisual)
        {
            ComputeVisualCollision = computeVisual;
        }

        public void SetComputeJointKinematics(bool computeKinematics)
        {
            ComputeJointKinematics = computeKinematics;
        }

        public void SetComputeJointLimits(bool computeJointLimits)
        {
            ComputeJointLimits = computeJointLimits;
        }

        // Enable/disable searching for reference geometry in nested (deeper than top level)
        // components, then refresh the coordinate system and axis lists.
        public void SetSearchNestedReferenceGeometry(bool searchNested)
        {
            SearchNestedReferenceGeometry = searchNested;
            UpdateReferenceGeometries();
        }

        // Set the case-insensitive name filter applied to enumerated reference geometry, then
        // refresh the coordinate system and axis lists. Pass null or "" to disable filtering.
        public void SetReferenceGeometryFilter(string filter)
        {
            ReferenceGeometryFilter = filter ?? "";
            UpdateReferenceGeometries();
        }

        private void ConstructExporter(SldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;
            swMath = iSwApp.GetMathUtility();
        }

        #region Export Methods

        // Beginning method for exporting the full package
        public void ExportRobot(bool exportSTL = true, MeshExportFormat meshFormat = MeshExportFormat.STL,
            ROSVersion rosVersion = ROSVersion.ROS1)
        {
            //Setting up the progress bar
            logger.Info("Beginning the export process");
            int progressBarBound = CommonSwOperations.GetCount(URDFRobot.BaseLink);
            iSwApp.GetUserProgressBar(out progressBar);
            progressBar.Start(0, progressBarBound, "Creating package directories");

            // ROS 2 / colcon package names must be lowercase [a-z0-9_]; a default name like
            // "3_DOF_ARM.SLDASM" would be rejected. Sanitize before the directory and files
            // are created so the folder, package.xml <name> and CMake project all agree.
            if (rosVersion == ROSVersion.ROS2)
            {
                string sanitized = ROS2Files.SanitizePackageName(PackageName);
                if (sanitized != PackageName)
                {
                    logger.Info("Sanitized ROS 2 package name from '" + PackageName + "' to '" + sanitized + "'");
                    PackageName = sanitized;
                }
            }

            //Creating package directories
            logger.Info("Creating package directories with name " + PackageName + " and save path " + SavePath);
            URDFPackage package = new URDFPackage(PackageName, SavePath);
            package.CreateDirectories();
            URDFRobot.Name = PackageName;
            string windowsURDFFileName = package.WindowsRobotsDirectory + URDFRobot.Name + ".urdf";
            string windowsCSVFileName = package.WindowsRobotsDirectory + URDFRobot.Name + ".csv";
            string windowsPackageXMLFileName = package.WindowsPackageDirectory + "package.xml";
            string robotURDF = URDFRobot.Name + ".urdf";

            if (rosVersion == ROSVersion.ROS2)
            {
                // ROS 2 (ament_cmake) scaffold: package.xml format 3, ament CMakeLists and
                // Python launch files. Geometry/URDF export below is shared with ROS 1.
                logger.Info("Creating ROS 2 (ament) package scaffold");
                ROS2Files.WriteCMakeLists(package.WindowsCMakeLists, PackageName);
                package.CreateConfigYAML(URDFRobot.GetJointNames(false));
                ROS2Files.WritePackageXML(windowsPackageXMLFileName, PackageName);
                ROS2Files.WriteDisplayLaunch(package.WindowsLaunchDirectory, PackageName, robotURDF);
                ROS2Files.WriteGazeboLaunch(package.WindowsLaunchDirectory, PackageName, robotURDF, URDFRobot.Name);
            }
            else
            {
                //Create CMakeLists
                logger.Info("Creating CMakeLists.txt at " + package.WindowsCMakeLists);
                package.CreateCMakeLists();

                //Create Config joint names, not sure how this is used...
                logger.Info("Creating joint names config at " + package.WindowsConfigYAML);
                package.CreateConfigYAML(URDFRobot.GetJointNames(false));

                //Creating package.xml file
                logger.Info("Creating package.xml at " + windowsPackageXMLFileName);
                PackageXMLWriter packageXMLWriter = new PackageXMLWriter(windowsPackageXMLFileName);
                PackageXML packageXML = new PackageXML(PackageName);
                packageXML.WriteElement(packageXMLWriter);

                //Creating RVIZ launch file
                Rviz rviz = new Rviz(PackageName, robotURDF);
                logger.Info("Creating RVIZ launch file in " + package.WindowsLaunchDirectory);
                rviz.WriteFiles(package.WindowsLaunchDirectory);

                //Creating Gazebo launch file
                Gazebo gazebo = new Gazebo(URDFRobot.Name, PackageName, robotURDF);
                logger.Info("Creating Gazebo launch file in " + package.WindowsLaunchDirectory);

                gazebo.WriteFile(package.WindowsLaunchDirectory);
            }

            //Customizing STL preferences to how I want them
            logger.Info("Saving existing STL preferences");
            SaveUserPreferences();

            logger.Info("Modifying STL preferences");
            SetSTLExportPreferences();

            //Saving part as STL mesh
            AssemblyDoc assyDoc = (AssemblyDoc)ActiveSWModel;
            List<string> hiddenComponents = CommonSwOperations.FindHiddenComponents(assyDoc.GetComponents(false));
            logger.Info("Found " + hiddenComponents.Count + " hidden components " + String.Join(", ", hiddenComponents));
            logger.Info("Hiding all components");
            ActiveSWModel.Extension.SelectAll();
            ActiveSWModel.HideComponent2();

            bool success = false;
            try
            {
                logger.Info("Beginning individual files export");
                ExportFiles(URDFRobot.BaseLink, package, 0, exportSTL, meshFormat);
                success = true;
            }
            catch (Exception e)
            {
                logger.Error("An exception was thrown attempting to export the URDF", e);
            }
            finally
            {
                logger.Info("Showing all components except previously hidden components");
                CommonSwOperations.ShowAllComponents(ActiveSWModel, hiddenComponents);

                logger.Info("Resetting STL preferences");
                ResetUserPreferences();
            }

            if (!success)
            {
                MessageBox.Show("Exporting the URDF failed unexpectedly. Email your maintainer " +
                    "with the log file found at " + Logger.GetFileName());
                return;
            }

            logger.Info("Writing URDF file to " + windowsURDFFileName);
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            URDFRobot.WriteURDF(uWriter.writer);

            ImportExport.WriteRobotToCSV(URDFRobot, windowsCSVFileName);

            logger.Info("Copying log file");
            CopyLogFile(package);

            logger.Info("Resetting STL preferences");
            ResetUserPreferences();
            progressBar.End();
        }

        public List<string> GetJointNames()
        {
            List<string> jointNames = new List<string>();

            Queue<Link> queue = new Queue<Link>();
            queue.Enqueue(URDFRobot.BaseLink);
            while (queue.Count > 0)
            {
                Link current = queue.Dequeue();
                if (current.Parent != null)
                {
                    jointNames.Add(current.Joint.Name);
                }

                foreach (Link child in current.Children)
                {
                    queue.Enqueue(child);
                }
            }

            return jointNames;
        }

        //Recursive method for exporting each link (and writing it to the URDF)
        private void ExportFiles(Link link, URDFPackage package, int count, bool exportSTL = true, MeshExportFormat meshFormat = MeshExportFormat.STL)
        {
            progressBar.UpdateProgress(count);
            progressBar.UpdateTitle("Exporting mesh: " + link.Name);
            logger.Info("Exporting link: " + link.Name);
            // Iterate through each child and export its files
            logger.Info("Link " + link.Name + " has " + link.Children.Count + " children");
            foreach (Link child in link.Children)
            {
                count += 1;
                if (!child.isFixedFrame)
                {
                    ExportFiles(child, package, count, exportSTL, meshFormat);
                }
            }

            // Copy the texture file (if it was specified) to the textures directory
            if (!link.isFixedFrame && !String.IsNullOrWhiteSpace(link.Visual.Material.Texture.wFilename))
            {
                if (File.Exists(link.Visual.Material.Texture.wFilename))
                {
                    link.Visual.Material.Texture.Filename =

                        package.TexturesDirectory + Path.GetFileName(link.Visual.Material.Texture.wFilename);
                    string textureSavePath =
                        package.WindowsTexturesDirectory + Path.GetFileName(link.Visual.Material.Texture.wFilename);
                    File.Copy(link.Visual.Material.Texture.wFilename, textureSavePath, true);
                }
            }

            // Create the mesh filenames. SolidWorks likes to use / but that will get messy in filenames so use _ instead
            string linkName = link.Name.Replace('/', '_');
            string meshFilename = package.MeshesDirectory + linkName;
            string windowsMeshFileName = package.WindowsMeshesDirectory + linkName;
            switch(meshFormat)
            {
                case MeshExportFormat.STL:
                    meshFilename += ".STL";
                    windowsMeshFileName += ".STL";
                    break;

                case MeshExportFormat.THREEDXML:
                    meshFilename += ".3dxml";
                    windowsMeshFileName += ".3dxml";
                    break;

                default:
                    meshFilename += ".STL";
                    windowsMeshFileName += ".STL";
                    break;
            }
            // Export STL
            if (exportSTL)
            {
                switch (meshFormat)
                {
                    case MeshExportFormat.STL:
                        SaveSTL(link, windowsMeshFileName);
                        break;

                    case MeshExportFormat.THREEDXML:
                        Save3dxml(link, windowsMeshFileName);
                        break;

                    default:
                        SaveSTL(link, windowsMeshFileName);
                        break;
                }
            }
            link.Visual.Geometry.Mesh.Filename = meshFilename;
            link.Collision.Geometry.Mesh.Filename = meshFilename;
        }

        private void Save3dxml(Link link, string windowsMeshFilename)
        {
            int errors = 0;
            int warnings = 0;

            string coordsysName = link.Joint.CoordinateSystemName;

            logger.Info(link.Name + ": Exporting 3dxml with coordinate frame " + coordsysName);

            Dictionary<string, string> names = GetComponentRefGeoNames(coordsysName);
            ModelDoc2 ActiveDoc = ActiveSWModel;

            logger.Info(link.Name + ": Reference geometry name " + names["component"]);

            CommonSwOperations.ShowComponents(ActiveSWModel, link.SWComponents);

            int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent |
                (int)swSaveAsOptions_e.swSaveAsOptions_Copy;
            SetLinkSpecificSTLPreferences(names["geo"], link.STLQualityFine, ActiveDoc);

            logger.Info("Saving 3dxml to " + windowsMeshFilename);

            // Align the link's visual/collision/inertial origins with its coordinate system
            // so the exported mesh lines up with the URDF link frame.
            LocalizeLinkToCoordinateSystem(link, coordsysName);

            ActiveDoc.Extension.SaveAs(windowsMeshFilename,
                (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);

            if (errors + warnings != 0)
            {
                logger.Warn("Exporting 3dxml for link " + link.Name + " failed with error " + errors +
                    " or warnings " + warnings);
            }
            CommonSwOperations.HideComponents(ActiveSWModel, link.SWComponents);
        }

        private bool SaveSTL(Link link, string windowsMeshFilename)
        {
            int errors = 0;
            int warnings = 0;

            string coordsysName = link.Joint.CoordinateSystemName;

            logger.Info(link.Name + ": Exporting STL with coordinate frame " + coordsysName);

            Dictionary<string, string> names = GetComponentRefGeoNames(coordsysName);
            ModelDoc2 ActiveDoc = ActiveSWModel;

            logger.Info(link.Name + ": Reference geometry name " + names["component"]);

            CommonSwOperations.ShowComponents(ActiveSWModel, link.SWComponents);

            int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent |
                (int)swSaveAsOptions_e.swSaveAsOptions_Copy;
            SetLinkSpecificSTLPreferences(names["geo"], link.STLQualityFine, ActiveDoc);

            // Align the link's visual/collision/inertial origins with its coordinate system
            // so the exported mesh lines up with the URDF link frame. Without this, STL meshes
            // for links whose coordinate system is not at the assembly origin are misplaced
            // (issues #87 / #116). Mirrors the 3dxml export path.
            LocalizeLinkToCoordinateSystem(link, coordsysName);

            logger.Info("Saving STL to " + windowsMeshFilename);
            ActiveDoc.Extension.SaveAs(windowsMeshFilename,
                (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);
            if (errors + warnings != 0)
            {
                logger.Warn("Exporting STL for link " + link.Name + " failed with error " + errors + 
                    " or warnings " + warnings);
            }
            CommonSwOperations.HideComponents(ActiveSWModel, link.SWComponents);

            bool success = CorrectSTLMesh(windowsMeshFilename);
            if (!success)
            {
                logger.Warn("There was an issue exporting the STL for " + link.Name + ". It " +
                    "may not be readable by CAD programs that aren't SolidWorks");
            }
            return success;
        }

        // Repositions the link's visual/collision/inertial origins so the exported mesh lines
        // up with the URDF link origin. The mesh is exported in the top-level assembly's global
        // frame, and the joint that positions this link is also computed in that global frame
        // (see GetCoordinateSystemTransform), so the mesh must be localized with the SAME global
        // transform. GetCoordinateSystemTransform composes the owning component's Transform2,
        // which SOLIDWORKS reports relative to the root assembly regardless of nesting depth
        // (the per-level sub-assembly transforms must NOT be chained manually). That makes this
        // depth-independent: it is correct for a coordinate system nested arbitrarily deep, not
        // just at the top level or in an immediate component. Shared by the STL and 3dxml paths.
        private void LocalizeLinkToCoordinateSystem(Link link, string coordsysName)
        {
            if (string.IsNullOrWhiteSpace(coordsysName))
            {
                logger.Warn("No coordinate system for link " + link.Name + "; mesh origin not localized");
                return;
            }

            MathTransform globalTransform = GetCoordinateSystemTransform(coordsysName);
            if (globalTransform != null)
            {
                logger.Info("Localizing link " + link.Name + " mesh to global coordinate system " + coordsysName);
                LocalizeLink(link, MathOps.GetTransformation(globalTransform));
            }
            else
            {
                logger.Warn("Could not resolve a global transform for coordinate system '" +
                    coordsysName + "'; mesh origin not localized for link " + link.Name);
            }
        }

        public void ExportLink(bool zIsUp)
        {
            CreateBaseRefOrigin(zIsUp);
            MathTransform coordSysTransform =
                ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
            Matrix<double> GlobalTransform = MathOps.GetTransformation(coordSysTransform);

            LocalizeLink(URDFRobot.BaseLink, GlobalTransform);

            //Creating package directories
            URDFPackage package = new URDFPackage(PackageName, SavePath);
            package.CreateDirectories();
            string meshFileName = package.MeshesDirectory + URDFRobot.BaseLink.Name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + URDFRobot.BaseLink.Name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + URDFRobot.Name + ".urdf";
            string windowsManifestFileName = package.WindowsPackageDirectory + "manifest.xml";

            //Creating manifest file
            PackageXMLWriter manifestWriter = new PackageXMLWriter(windowsManifestFileName);
            PackageXML Manifest = new PackageXML(URDFRobot.Name);
            Manifest.WriteElement(manifestWriter);

            //Customizing STL preferences to how I want them
            SaveUserPreferences();
            SetSTLExportPreferences();
            SetLinkSpecificSTLPreferences("", URDFRobot.BaseLink.STLQualityFine, ActiveSWModel);
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh

            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion,
                (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            URDFRobot.BaseLink.Visual.Geometry.Mesh.Filename = meshFileName;
            URDFRobot.BaseLink.Collision.Geometry.Mesh.Filename = meshFileName;

            URDFRobot.BaseLink.Visual.Material.Texture.Filename =
                package.TexturesDirectory + Path.GetFileName(URDFRobot.BaseLink.Visual.Material.Texture.wFilename);
            string textureSavePath =
                package.WindowsTexturesDirectory + Path.GetFileName(URDFRobot.BaseLink.Visual.Material.Texture.wFilename);
            if (!String.IsNullOrWhiteSpace(URDFRobot.BaseLink.Visual.Material.Texture.wFilename))
            {
                File.Copy(URDFRobot.BaseLink.Visual.Material.Texture.wFilename, textureSavePath, true);
            }

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            //mRobot.addLink(mLink);
            URDFRobot.WriteURDF(uWriter.writer);

            ResetUserPreferences();
        }

        //Writes an empty header to the STL to get rid of the BS that SolidWorks adds to a binary STL file
        public static bool CorrectSTLMesh(string filename)
        {
            logger.Info("Removing SW header in STL file");
            try
            {
                using (FileStream fileStream = new FileStream(filename, FileMode.Open, FileAccess.Write, FileShare.None))
                {
                    byte[] emptyHeader = new byte[80];
                    fileStream.Write(emptyHeader, 0, emptyHeader.Length);
                }
            }
            catch (Exception e)
            {
                logger.Warn("Correcting the STL " + filename + " failed. This STL may not be " +
                    "readable by ROS or other CAD programs", e);
                return false;
            }
            return true;
        }

        #endregion Export Methods

        private static void CopyLogFile(URDFPackage package)
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
                    File.Copy(log_filename, destination, true);
                }
            }
        }

        #region STL Preference shuffling

        //Saves the preferences that the user had setup so that I can change them and revert back to their configuration
        private void SaveUserPreferences()
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
        private void SetSTLExportPreferences()
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
        private void ResetUserPreferences()
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
        private void SetLinkSpecificSTLPreferences(string CoordinateSystemName, bool qualityFine, ModelDoc2 doc)
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

        #endregion STL Preference shuffling
    }
}
