using System;
using System.Linq;
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
using System.Xml;
using System.Xml.Serialization;
using MathNet.Numerics.LinearAlgebra.Generic;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using System.Numerics;


namespace SW2URDF
{
    // This class contains a long list of methods that are used throughout the export process. Methods for building links and joints are contained in here.
    // Many of the methods are overloaded, but seek to reduce repeated code as much as possible (i.e. the overloaded methods call eachother).
    // These methods are used by the PartExportForm, the AssemblyExportForm and the PropertyManager Page
    public class SW2URDFExporter
    {
        #region class variables
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
        private string referenceSketchName;
        private UserProgressBar progressBar;

        [XmlIgnore]
        public ModelDoc2 ActiveSWModel;
        [XmlIgnore]
        public MathUtility swMath;
        [XmlIgnore]
        public AttributeDef saveConfigurationAttributeDef
        { get; set; }
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
        public SW2URDFExporter(ISldWorks iSldWorksApp)
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

            saveConfigurationAttributeDef = iSwApp.DefineAttribute("URDF Export Configuration");
            int Options = 0;

            saveConfigurationAttributeDef.AddParameter("data", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("name", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("date", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("exporterVersion", (int)swParamType_e.swParamTypeDouble, 1.0, Options);
            saveConfigurationAttributeDef.Register();

        }

        // Converts a LinkNode to a SerialNode for Serialization and saving purposes
        public SerialNode convertLinkNodeToSerialNode(LinkNode node)
        {
            SerialNode sNode = new SerialNode();

            if (node.Link == null)
            {
                sNode.linkName = node.linkName;
                sNode.jointName = node.jointName;
                sNode.axisName = node.axisName;
                sNode.coordsysName = node.coordsysName;

                // The SWComponents associated with a node are converted to persisted IDs so that they can be referenced when SolidWorks is opened later
                // or on a different machine.
                sNode.componentPIDs = saveSWComponents(node.Components);

                sNode.jointType = node.jointType;
                sNode.isBaseNode = node.isBaseNode;
                sNode.isIncomplete = node.isIncomplete;
            }
            else
            {
                sNode.linkName = (string)node.Link.name;

                if (node.Link.Joint != null)
                {
                    sNode.jointName = (string)node.Link.Joint.name;

                    if (node.Link.Joint.Axis.X == 0 && node.Link.Joint.Axis.Y == 0 && node.Link.Joint.Axis.Z == 0)
                    {
                        sNode.axisName = "None";
                    }
                    else
                    {
                        sNode.axisName = node.Link.Joint.AxisName;
                    }
                        sNode.coordsysName = node.Link.Joint.CoordinateSystemName;
                    sNode.jointType = (string)node.Link.Joint.type;
                }
                sNode.componentPIDs = saveSWComponents(node.Link.SWcomponents);
                sNode.isBaseNode = node.isBaseNode;
                sNode.isIncomplete = node.isIncomplete;
            }
            //Proceed recursively through the nodes
            foreach (LinkNode child in node.Nodes)
            {
                sNode.Nodes.Add(convertLinkNodeToSerialNode(child));
            }

            return sNode;
        }

        // Converts a SerialNode to LinkNode used when loading a configuration
        public LinkNode convertSerialNodeToLinkNode(SerialNode node)
        {
            LinkNode lNode = new LinkNode();
            lNode.linkName = node.linkName;
            lNode.jointName = node.jointName;
            lNode.axisName = node.axisName;
            lNode.coordsysName = node.coordsysName;
            lNode.ComponentPIDs = node.componentPIDs;
            lNode.jointType = node.jointType;
            lNode.isBaseNode = node.isBaseNode;
            lNode.isIncomplete = node.isIncomplete;

            //Converts the Component PIDs to actual Component references
            lNode.Components = loadSWComponents(lNode.ComponentPIDs);
            lNode.Name = lNode.linkName;
            lNode.Text = lNode.linkName;

            foreach (SerialNode child in node.Nodes)
            {
                lNode.Nodes.Add(convertSerialNodeToLinkNode(child));
            }

            return lNode;
        }

        #region SW to Robot and link methods

        //Used right now only by the Part Exporter, but this starts the building of the robot
        public void createRobotFromActiveModel()
        {
            mRobot = new robot();
            mRobot.name = ActiveSWModel.GetTitle();

            Configuration swConfig = ActiveSWModel.ConfigurationManager.ActiveConfiguration;
            foreach (string state in swConfig.GetDisplayStates())
            {
                if (state.Equals("URDF Export"))
                {
                    swConfig.ApplyDisplayState("URDF Export");
                }
            }

            //Each Robot contains a single base link, build this link
            mRobot.BaseLink = createBaseLinkFromActiveModel();
        }

        // This method now only works for the part exporter
        public link createBaseLinkFromActiveModel()
        {
            if (ActiveSWModel.GetType() == (int)swDocumentTypes_e.swDocPART) // If the model is a part
            {
                return createLinkFromPartModel(ActiveSWModel);
            }
            return null;
        }

        // This creates a Link from a Part ModelDoc. It basically just extracts the material properties and saves them to the appropriate fields.
        public link createLinkFromPartModel(ModelDoc2 swModel)
        {
            link Link = new link();
            Link.name = swModel.GetTitle();

            Link.isFixedFrame = false;
            Link.Visual = new visual();
            Link.Inertial = new inertial();
            Link.Collision = new collision();

            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            Link.Inertial.Mass.value = swMass.Mass;
            double[] moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]
            Link.Inertial.Inertia.setMomentMatrix(moment);

            double[] centerOfMass = swMass.CenterOfMass;
            Link.Inertial.Origin.xyz = centerOfMass;
            Link.Inertial.Origin.rpy = new double[3] { 0, 0, 0 };

            // Will this ever not be zeros?
            Link.Visual.Origin.xyz = new double[3] { 0, 0, 0 };
            Link.Visual.Origin.rpy = new double[3] { 0, 0, 0 };
            Link.Collision.Origin.xyz = new double[3] { 0, 0, 0 };
            Link.Collision.Origin.rpy = new double[3] { 0, 0, 0 };

            // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
            double[] values = swModel.MaterialPropertyValues;
            Link.Visual.Material.Color.Red = values[0];
            Link.Visual.Material.Color.Green = values[1];
            Link.Visual.Material.Color.Blue = values[2];
            Link.Visual.Material.Color.Alpha = 1.0 - values[7];
            Link.Visual.Material.name = "material_" + Link.name;

            return Link;
        }

        // Converts the PIDs to actual references to the components and proceeds recursively through the child links
        public void loadSWComponents(link Link)
        {
            Link.SWMainComponent = loadSWComponent(Link.SWMainComponentPID);
            Link.SWcomponents = loadSWComponents(Link.SWComponentPIDs);
            foreach (link Child in Link.Children)
            {
                loadSWComponents(Child);
            }
        }

        // Converts the PIDs to actual references to the components
        public List<Component2> loadSWComponents(List<byte[]> PIDs)
        {
            List<Component2> components = new List<Component2>();
            foreach (byte[] PID in PIDs)
            {
                components.Add(loadSWComponent(PID));
            }
            return components;
        }

        // Converts a single PID to a Component2 object
        public Component2 loadSWComponent(byte[] PID)
        {
            int Errors = 0;
            if (PID != null)
            {
                return (Component2)ActiveSWModel.Extension.GetObjectByPersistReference3(PID, out Errors);
            }
            return null;
        }

        #endregion

        #region Joint methods

        // Creates a Reference Coordinate System in the SolidWorks Model to symbolize the joint location
        public void createRefOrigin(joint Joint)
        {
            createRefOrigin(Joint.Origin, Joint.CoordinateSystemName);
        }

        // Creates a Reference Coordinate System in the SolidWorks Model to symbolize the joint location
        public void createRefOrigin(origin Origin, string CoordinateSystemName)
        {
            // Adds the sketch segments and point to the 3D sketch. The sketchEnties are the actual items created (and their locations)
            object[] sketchEntities = addSketchGeometry(Origin);

            SketchPoint OriginPoint = (SketchPoint)sketchEntities[0];
            SketchSegment xaxis = (SketchSegment)sketchEntities[1];
            SketchSegment yaxis = (SketchSegment)sketchEntities[2];

            double origin_X = (double)sketchEntities[3]; //OriginPoint X
            double origin_Y = (double)sketchEntities[4];
            double origin_Z = (double)sketchEntities[5];

            double xAxis_X = (double)sketchEntities[6];
            double xAxis_Y = (double)sketchEntities[7];
            double xAxis_Z = (double)sketchEntities[8];

            double yAxis_X = (double)sketchEntities[9];
            double yAxis_Y = (double)sketchEntities[10];
            double yAxis_Z = (double)sketchEntities[11];

            IFeature coordinates = default(IFeature);
            ActiveSWModel.ClearSelection2(true);
            SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
            SelectData data = selectionManager.CreateSelectData();

            // First select the origin
            bool SelectedOrigin = false; bool SelectedXAxis = false; bool SelectedYAxis = false;
            if (OriginPoint != null)
            {
                data.Mark = 1;
                SelectedOrigin = OriginPoint.Select4(true, data);
            }
            if (!SelectedOrigin)
            {
                SelectedOrigin = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", origin_X, origin_Y, origin_Z, true, 1, null, 0);
            }

            // Second, select the xaxis
            if (xaxis != null)
            {
                data.Mark = 2;
                SelectedXAxis = xaxis.Select4(true, data);
            }
            if (!SelectedXAxis)
            {
                SelectedXAxis = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", xAxis_X, xAxis_Y, xAxis_Z, true, 2, null, 0);
            }

            // Third, select the yaxis
            if (yaxis != null)
            {
                data.Mark = 4;
                SelectedYAxis = yaxis.Select4(true, data);
            }
            if (!SelectedYAxis)
            {
                SelectedYAxis = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", yAxis_X, yAxis_Y, yAxis_Z, true, 4, null, 0);
            }

            //From the selected items, insert a coordinate system.
            coordinates = ActiveSWModel.FeatureManager.InsertCoordinateSystem(false, false, false);
            if (coordinates != null)
            {
                coordinates.Name = CoordinateSystemName;
            }
        }

        //Creates the Origin_global coordinate system
        public void createBaseRefOrigin(bool zIsUp)
        {
            if (!ActiveSWModel.Extension.SelectByID2("Origin_global", "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                joint Joint = new joint();
                Joint.Origin = new origin();
                if (zIsUp)
                {
                    Joint.Origin.rpy = new double[] { -Math.PI / 2, 0, 0 };
                }
                else
                {
                    Joint.Origin.rpy = new double[] { 0, 0, 0 };
                }
                Joint.Origin.xyz = new double[] { 0, 0, 0 };
                Joint.CoordinateSystemName = "Origin_global";
                if (referenceSketchName == null)
                {
                    referenceSketchName = setup3DSketch();
                }
                createRefOrigin(Joint);
            }
        }

        // Creates a Reference Axis to be used to calculate the joint axis
        public void createRefAxis(joint Joint)
        {
            //Adds sketch segment
            SketchSegment rotaxis = addSketchGeometry(Joint.Axis, Joint.Origin);
            if (rotaxis != null)
            {
                //Use special method to create the axis
                Feature featAxis = insertAxis(rotaxis);
                if (featAxis != null)
                {
                    featAxis.Name = Joint.AxisName;
                }
            }
        }

        // Takes a links joint and calculates the local transform from the global transforms of the parent and child. It also converts the
        // axis to local values
        public void localizeJoint(joint Joint, string parentCoordsysName)
        {
            MathTransform parentTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(parentCoordsysName);
            Matrix<double> ParentJointGlobalTransform = ops.getTransformation(parentTransform);
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(Joint.CoordinateSystemName);
            //Transform from global origin to child joint
            Matrix<double> ChildJointGlobalTransform = ops.getTransformation(coordsysTransform);
            Matrix<double> ChildJointOrigin = ParentJointGlobalTransform.Inverse() * ChildJointGlobalTransform;

            //Localize the axis to the Link's coordinate system.
            localizeAxis(Joint.Axis.xyz, Joint.CoordinateSystemName);

            // Get the array values and threshold them so small values are set to 0.
            Joint.Origin.xyz = ops.getXYZ(ChildJointOrigin);
            ops.threshold(Joint.Origin.xyz, 0.00001);
            Joint.Origin.rpy = ops.getRPY(ChildJointOrigin);
            ops.threshold(Joint.Origin.xyz, 0.00001);
        }

        //This is only used by the Part Exporter, but it localizes the link to the Origin_global coordinate system
        public void localizeLink(link Link, Matrix<double> GlobalTransform)
        {
            Matrix<double> GlobalTransformInverse = GlobalTransform.Inverse();
            Matrix<double> linkCoMTransform = ops.getTranslation(Link.Inertial.Origin.xyz);
            Matrix<double> localLinkCoMTransform = GlobalTransformInverse * linkCoMTransform;

            Matrix<double> linkVisualTransform = ops.getTransformation(Link.Visual.Origin.xyz, Link.Visual.Origin.rpy);
            Matrix<double> localVisualTransform = GlobalTransformInverse * linkVisualTransform;

            Matrix<double> linkCollisionTransform = ops.getTransformation(Link.Collision.Origin.xyz, Link.Collision.Origin.rpy);
            Matrix<double> localCollisionTransform = GlobalTransformInverse * linkCollisionTransform;

            // The linear array in Link.Inertial.Inertia.Moment is in row major order, but this matrix constructor uses column major order
            // It's a rotation matrix, so this shouldn't matter. If it does, just transpose linkGlobalMomentInertia
            // These three matrices are 3x3 as opposed to the 4x4 transformation matrices above. You're welcome for the confusion.
            Matrix<double> linkGlobalMomentInertia = new DenseMatrix(3, 3, Link.Inertial.Inertia.Moment);
            Matrix<double> GlobalRotMat = GlobalTransform.SubMatrix(0, 3, 0, 3);
            Matrix<double> linkLocalMomentInertia = GlobalRotMat.Inverse() * linkGlobalMomentInertia;

            Link.Inertial.Origin.xyz = ops.getXYZ(localLinkCoMTransform);
            Link.Inertial.Origin.rpy = new double[] { 0, 0, 0 };

            // Wait are you saying that even though the matrix was trasposed from column major order, you are writing it in row-major order here.
            // Yes, yes I am.
            double[] moment = linkLocalMomentInertia.ToRowWiseArray();
            Link.Inertial.Inertia.setMomentMatrix(moment);


            Link.Collision.Origin.xyz = ops.getXYZ(localCollisionTransform);
            Link.Collision.Origin.rpy = ops.getRPY(localCollisionTransform);

            Link.Visual.Origin.rpy = ops.getXYZ(localVisualTransform);
            Link.Visual.Origin.xyz = ops.getRPY(localVisualTransform);
        }

        // Funny method I created that inserts a RefAxis and then finds the reference to it.
        public Feature insertAxis(SketchSegment axis)
        {
            //First select the axis
            SelectData data = ActiveSWModel.SelectionManager.CreateSelectData();
            axis.Select4(false, data);

            //Get the features before the axis is created
            object[] featuresBefore, featuresAfter;
            featuresBefore = ActiveSWModel.FeatureManager.GetFeatures(true);
            int countBefore = ActiveSWModel.FeatureManager.GetFeatureCount(true);

            //Create the axis
            ActiveSWModel.InsertAxis2(true);

            //Get the features after the axis is created
            featuresAfter = ActiveSWModel.FeatureManager.GetFeatures(true);
            int countAfter = ActiveSWModel.FeatureManager.GetFeatureCount(true);

            // If it was created, try to find it
            if (featuresBefore.Length < featuresAfter.Length)
            {
                //It was probably added at the end (hence .Reverse())
                foreach (Feature feat in featuresAfter.Reverse())
                {
                    //If the feature in featuresAfter is not in features before, its gotta be the axis we inserted
                    if (!featuresBefore.Contains(feat))
                    {
                        return feat;
                    }
                }
            }
            return null;
        }

        // Inserts a sketch into the main assembly and name it
        public string setup3DSketch()
        {
            bool sketchExists = ActiveSWModel.Extension.SelectByID2("URDF Reference", "SKETCH", 0, 0, 0, false, 0, null, 0);
            ActiveSWModel.SketchManager.Insert3DSketch(true);
            ActiveSWModel.SketchManager.CreatePoint(0, 0, 0);
            IFeature sketch = (IFeature)ActiveSWModel.SketchManager.ActiveSketch;
            ActiveSWModel.SketchManager.Insert3DSketch(true);
            if (!sketchExists)
            {
                sketch.Name = "URDF Reference";
            }
            return sketch.Name;
        }

        // Adds lines and a point to create the entities for a reference coordinates
        public object[] addSketchGeometry(origin Origin)
        {
            //Find if the sketch exists first
            if (ActiveSWModel.SketchManager.ActiveSketch == null)
            {
                bool sketchExists = ActiveSWModel.Extension.SelectByID2(referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
                ActiveSWModel.SketchManager.Insert3DSketch(true);
            }
            IFeature sketch = (IFeature)ActiveSWModel.SketchManager.ActiveSketch;
            
            //Calculate the lines that need to be drawn
            Matrix<double> transform = ops.getRotation(Origin.rpy);
            Matrix<double> Axes = 0.01 * DenseMatrix.Identity(4);
            Matrix<double> tA = transform * Axes;

            // origin at X, Y, Z
            SketchPoint OriginPoint = ActiveSWModel.SketchManager.CreatePoint(Origin.X,
                                                                      Origin.Y,
                                                                      Origin.Z);

            // xAxis is a 1cm line from the origin in the direction of the xaxis of the coordinate system
            SketchSegment XAxis = ActiveSWModel.SketchManager.CreateLine(Origin.X,
                                                                         Origin.Y,
                                                                         Origin.Z,
                                                                         Origin.X + tA[0, 0],
                                                                         Origin.Y + tA[1, 0],
                                                                         Origin.Z + tA[2, 0]);
            XAxis.ConstructionGeometry = true;

            //yAxis is a 1cm line from the origin in the direction of the yaxis of the coordinate system
            SketchSegment YAxis = ActiveSWModel.SketchManager.CreateLine(Origin.X,
                                                                         Origin.Y,
                                                                         Origin.Z,
                                                                         Origin.X + tA[0, 1],
                                                                         Origin.Y + tA[1, 1],
                                                                         Origin.Z + tA[2, 1]);
            YAxis.ConstructionGeometry = true;

            //Close the sketch
            if (ActiveSWModel.SketchManager.ActiveSketch != null)
            {
                ActiveSWModel.SketchManager.Insert3DSketch(true);

            }
            // Return an array of objects representing the sketch items that were just inserted, as well as the actual locations of those objecs (aids selection).
            return new object[] { OriginPoint, XAxis, YAxis, Origin.X, Origin.Y, Origin.Z, Origin.X + tA[0, 0], Origin.Y + tA[1, 0], Origin.Z + tA[2, 0], Origin.X + tA[0, 1], Origin.Y + tA[1, 1], Origin.Z + tA[2, 1] };
        }

        //Inserts a sketch segment for use when creating a Reference Axis
        public SketchSegment addSketchGeometry(axis Axis, origin Origin)
        {
            if (ActiveSWModel.SketchManager.ActiveSketch == null)
            {
                bool sketchExists = ActiveSWModel.Extension.SelectByID2(referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
                ActiveSWModel.SketchManager.Insert3DSketch(true);
            }
            
            //Insert sketch segment 0.1m long centered on the origin.
            SketchSegment RotAxis = ActiveSWModel.SketchManager.CreateLine(Origin.X - 0.05 * Axis.X,
                                                               Origin.Y - 0.05 * Axis.Y,
                                                               Origin.Z - 0.05 * Axis.Z,
                                                               Origin.X + 0.05 * Axis.X,
                                                               Origin.Y + 0.05 * Axis.Y,
                                                               Origin.Z + 0.05 * Axis.Z);
            if (RotAxis == null)
            {
                return null;
            }
            RotAxis.ConstructionGeometry = true;
            RotAxis.Width = 2;

            //Close sketch
            if (ActiveSWModel.SketchManager.ActiveSketch != null)
            {
                ActiveSWModel.SketchManager.Insert3DSketch(true);

            }
            return RotAxis;
        }

        //Calculates the free degree of freedom (if exists), and then determines the location of the joint, the axis of rotation/translation, and the type of joint
        public void estimateGlobalJointFromComponents(AssemblyDoc assy, link parent, link child)
        {
            //Create the ref objects
            int R1Status, R2Status, L1Status, L2Status, R1DirStatus, R2DirStatus, DOFs;
            MathPoint RPoint1, RPoint2;
            MathVector RDir1, RDir2, LDir1, LDir2;

            // Surpress Limit Mates to properly find degrees of freedom. They don't work with the API call
            List<Mate2> limitMates = new List<Mate2>();
            limitMates = suppressLimitMates(child.SWMainComponent);

            if (child.SWMainComponent != null)
            {

                // The wonderful undocumented API call I found to get the degrees of freedom in a joint. 
                // https://forum.solidworks.com/thread/57414
                int remainingDOFs = child.SWMainComponent.GetRemainingDOFs(out R1Status, out RPoint1, out R1DirStatus, out RDir1,
                                                  out R2Status, out RPoint2, out R2DirStatus, out RDir2,
                                                  out L1Status, out LDir1,
                                                  out L2Status, out LDir2);
                DOFs = remainingDOFs;


                // Convert the gotten degrees of freedom to a joint type, origin and axis
                child.Joint.type = "fixed";
                child.Joint.Origin.xyz = ops.getXYZ(child.SWMainComponent.Transform2);
                child.Joint.Origin.rpy = ops.getRPY(child.SWMainComponent.Transform2);

                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    if (R1Status == 1)
                    {
                        child.Joint.type = "continuous";
                        child.Joint.Axis.xyz = RDir1.ArrayData;
                        child.Joint.Origin.xyz = RPoint1.ArrayData;
                        child.Joint.Origin.rpy = ops.getRPY(child.SWMainComponent.Transform2);
                        moveOrigin(parent, child);
                    }
                    else if (L1Status == 1)
                    {
                        child.Joint.type = "prismatic";
                        child.Joint.Axis.xyz = LDir1.ArrayData;
                        child.Joint.Origin.xyz = RPoint1.ArrayData;
                        child.Joint.Origin.rpy = ops.getRPY(child.SWMainComponent.Transform2);
                        moveOrigin(parent, child);
                    }
                }
                ops.threshold(child.Joint.Origin.xyz, 0.00001);
                ops.threshold(child.Joint.Origin.rpy, 0.00001);
                unsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    addLimits(child.Joint, limitMates);
                }
            }
        }

        public void estimateGlobalJointFromRefGeometry(link parent, link child)
        {
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(child.Joint.CoordinateSystemName);
            child.Joint.Origin.xyz = ops.getXYZ(coordsysTransform);
            child.Joint.Origin.rpy = ops.getRPY(coordsysTransform);
            estimateAxis(child.Joint);
        }

        public void moveOrigin(link parent, link nonLocalizedChild)
        {
            double X_max = Double.MinValue; double Y_max = Double.MinValue; double Z_max = Double.MinValue;
            double X_min = Double.MaxValue; double Y_min = Double.MaxValue; double Z_min = Double.MaxValue;
            double[] points;

            foreach (Component2 comp in nonLocalizedChild.SWcomponents)
            {
                points = comp.GetBox(false, false); // Returns box as [ XCorner1, YCorner1, ZCorner1, XCorner2, YCorner2, ZCorner2 ]
                X_max = ops.max(points[0], points[3], X_max);
                Y_max = ops.max(points[1], points[4], Y_max);
                Z_max = ops.max(points[2], points[5], Z_max);
                X_min = ops.min(points[0], points[3], X_min);
                Y_min = ops.min(points[1], points[4], Y_min);
                Z_min = ops.min(points[2], points[5], Z_min);
            }
            string coordsys = (parent.Joint == null) ? "Origin_global" : parent.Joint.CoordinateSystemName;

            MathTransform parentTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(coordsys);
            double[] idealOrigin = ops.closestPointOnLineToPoint(ops.getXYZ(parentTransform), nonLocalizedChild.Joint.Axis.xyz, nonLocalizedChild.Joint.Origin.xyz);

            nonLocalizedChild.Joint.Origin.xyz = ops.closestPointOnLineWithinBox(X_min, X_max, Y_min, Y_max, Z_min, Z_max, nonLocalizedChild.Joint.Axis.xyz, idealOrigin);
            
        }

        // Calculates the axis from a Reference Axis in the model
        public void estimateAxis(joint Joint)
        {
            Joint.Axis.xyz = estimateAxis(Joint.AxisName);
        }

        public double[] estimateAxis(string axisName)
        {
            double[] XYZ = new double[3];

            //Select the axis
            ActiveSWModel.ClearSelection2(true);
            bool selected = ActiveSWModel.Extension.SelectByID2(axisName, "AXIS", 0, 0, 0, false, 0, null, 0);
            if (selected)
            {
                //Get the axis feature
                Feature feat = ActiveSWModel.SelectionManager.GetSelectedObject6(1, 0);
                RefAxis axis = (RefAxis)feat.GetSpecificFeature2();

                //Calculate!
                double[] axisParams;

                axisParams = axis.GetRefAxisParams();
                XYZ[0] = axisParams[0] - axisParams[3];
                XYZ[1] = axisParams[1] - axisParams[4];
                XYZ[2] = axisParams[2] - axisParams[5];
                XYZ = ops.pnorm(XYZ, 2);
            }
            return XYZ;
        }

        //This is called whenever the pull down menu is changed and the axis needs to be recalculated in reference to the coordinate system
        public void localizeAxis(double[] Axis, string coordsys)
        {
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(coordsys);
            Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
            Matrix<double> transform = ops.getTransformation(coordsysTransform);
            vec = transform.Inverse() * vec;
            Axis[0] = vec[0]; Axis[1] = vec[1]; Axis[2] = vec[2];
            ops.threshold(Axis, 0.00001);
            
        }

        // Used to fill Combo Boxes
        public string[] findAxes()
        {
            List<string> axesNames = new List<string>();
            object[] features;
            features = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Feature feat in features)
            {
                if (feat.GetTypeName2() == "RefAxis")
                {
                    axesNames.Add(feat.Name);
                }
            }
            return axesNames.ToArray();
        }

        //Used the fill combo boxes in the AssemblyExportForm
        public string[] findOrigins()
        {
            List<string> originNames = new List<string>();
            object[] features;
            features = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Feature feat in features)
            {
                if (feat.GetTypeName2() == "CoordSys")
                {
                    originNames.Add(feat.Name);
                }
            }
            return originNames.ToArray();
        }

        //This method adds in the limits from a limit mate, to make a joint a revolute joint. It really needs to checked for correctness.
        public void addLimits(joint Joint, List<Mate2> limitMates)
        {
            // The number of limit Mates should only be one. But for completeness, I cycle through every found limit mate.
            foreach (Mate2 swMate in limitMates)
            {
                // [TODO] This assumes the limit mate limits the right degree of freedom, it really should check that assumption
                if ((Joint.type == "continuous" && swMate.Type == (int)swMateType_e.swMateANGLE) ||
                    (Joint.type == "prismatic" && swMate.Type == (int)swMateType_e.swMateDISTANCE))
                {
                    Joint.Limit.upper = swMate.MaximumVariation; // Lucky me that no conversion is necessary
                    Joint.Limit.lower = swMate.MinimumVariation;
                    if (Joint.type == "continuous")
                    {
                        Joint.type = "revolute";
                    }
                }
            }
        }

        // Suppresses limit mates to make it easier to find the free degree of freedom in a joint
        public List<Mate2> suppressLimitMates(IComponent2 component)
        {
            ModelDoc2 modelDoc = component.GetModelDoc2();
            List<Mate2> limitMates = new List<Mate2>();

                object[] objs = component.GetMates();
                foreach (object obj in objs)
                {
                    if (obj is Mate2)
                    {
                        Mate2 swMate = (Mate2)obj;
                        if (swMate.MinimumVariation != swMate.MaximumVariation)
                        {
                            limitMates.Add(swMate);
                        }
                    }
                }
                foreach (Mate2 swMate in limitMates)
                {
                    ModelDoc2 doc = component.GetModelDoc2();
                    Feature feat = (Feature)swMate;
                    feat.Select(false);
                    feat.SetSuppression2((int)swFeatureSuppressionAction_e.swSuppressFeature, (int)swInConfigurationOpts_e.swThisConfiguration, null);
                }

            return limitMates;
        }

        // Unsuppresses limit mates that were suppressed before
        public void unsuppressLimitMates(List<Mate2> limitMates)
        {
            foreach (Mate2 swMate in limitMates)
            {
                Feature feat = (Feature)swMate;
                feat.SetSuppression2((int)swFeatureSuppressionAction_e.swUnSuppressFeature, (int)swInConfigurationOpts_e.swThisConfiguration, null);
            }
        }

        //Unfixes components that were fixed to find the free degree of freedom
        public void unFixComponents(List<Component2> components)
        {
            selectComponents(components, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.UnfixComponent();
        }
        #endregion

        #region Export Methods

        //Copy and export textures here

        // Beginning method for exporting the full package
        public void exportRobot()
        {
            //Setting up the progress bar

            int progressBarBound = getCount(mRobot.BaseLink);
            progressBar.Start(0, progressBarBound, "Creating package directories");

            //Creating package directories
            Package package = new Package(mPackageName, mSavePath);
            package.createDirectories();
            mRobot.name = mPackageName;
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.name + ".URDF";
            string windowsManifestFileName = package.WindowsPackageDirectory + "manifest.xml";

            //Creating manifest file
            manifestWriter manifestWriter = new manifestWriter(windowsManifestFileName);
            manifest Manifest = new manifest(mPackageName);
            Manifest.writeElement(manifestWriter);

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();

            //Saving part as STL mesh
            AssemblyDoc assyDoc = (AssemblyDoc)ActiveSWModel;
            List<Component2> hiddenComponents = findHiddenComponents(assyDoc.GetComponents(false));
            ActiveSWModel.Extension.SelectAll();
            ActiveSWModel.HideComponent2();
            string filename = exportFiles(mRobot.BaseLink, package, 0);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = filename;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = filename;
            
            showAllComponents(hiddenComponents);
            //Writing URDF to file

            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
            progressBar.End();
        }

        //Recursive method for exporting each link (and writing it to the URDF)
        public string exportFiles(link Link, Package package, int count)
        {
            progressBar.UpdateProgress(count);
            progressBar.UpdateTitle("Exporting mesh: " + Link.name);
            // Iterate through each child and export its files
            foreach (link child in Link.Children)
            {
                count += 1;
                if (!child.isFixedFrame)
                {
                    string filename = exportFiles(child, package, count);
                    child.Visual.Geometry.Mesh.filename = filename;
                    child.Collision.Geometry.Mesh.filename = filename;
                }
            }

            // Copy the texture file (if it was specified) to the textures directory
            if (!Link.isFixedFrame && Link.Visual.Material.Texture.wFilename != "")
            {
                if (System.IO.File.Exists(Link.Visual.Material.Texture.wFilename))
                {
                    Link.Visual.Material.Texture.filename = package.TexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    string textureSavePath = package.WindowsTexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    System.IO.File.Copy(Link.Visual.Material.Texture.wFilename, textureSavePath, true);
                }
            }

            // Create the mesh filenames. SolidWorks likes to use / but that will get messy in filenames so use _ instead
            string linkName = Link.name.Replace('/', '_');
            string meshFileName = package.MeshesDirectory + linkName + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + linkName + ".STL";

            int errors = 0;
            int warnings = 0;

            // Export STL
            showComponents(Link.SWcomponents);

            int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
            if (Link.Joint == null || Link.Joint.CoordinateSystemName == null)
            {
                setLinkSpecificSTLPreferences("Origin_global", Link.STLQualityFine);
            }
            else
            {
                setLinkSpecificSTLPreferences(Link.Joint.CoordinateSystemName, Link.STLQualityFine);
            }
            string coordsysname = ActiveSWModel.Extension.GetUserPreferenceString((int)swUserPreferenceStringValue_e.swFileSaveAsCoordinateSystem, (int)swUserPreferenceOption_e.swDetailingNoOptionSpecified);
            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);
            hideComponents(Link.SWcomponents);

            correctSTLMesh(windowsMeshFileName);

            return meshFileName;
        }

        // Used only by the part exporter
        public void exportLink(bool zIsUp)
        {

            createBaseRefOrigin(zIsUp);
            MathTransform coordSysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
            Matrix<double> GlobalTransform = ops.getTransformation(coordSysTransform);

            localizeLink(mRobot.BaseLink, GlobalTransform);

            //Creating package directories
            Package package = new Package(mPackageName, mSavePath);
            package.createDirectories();
            string meshFileName = package.MeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.name + ".URDF";
            string windowsManifestFileName = package.WindowsPackageDirectory + "manifest.xml";

            //Creating manifest file
            manifestWriter manifestWriter = new manifestWriter(windowsManifestFileName);
            manifest Manifest = new manifest(mRobot.name);
            Manifest.writeElement(manifestWriter);

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();
            setLinkSpecificSTLPreferences("", mRobot.BaseLink.STLQualityFine);
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh

            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = meshFileName;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = meshFileName;

            correctSTLMesh(windowsMeshFileName);

            mRobot.BaseLink.Visual.Material.Texture.filename = package.TexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
            string textureSavePath = package.WindowsTexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
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

        public int getCount(link Link)
        {
            int count = 1;
            foreach (link child in Link.Children)
            {
                count += getCount(child);
            }
            return count;
        }

        public int getCount(LinkNode node)
        {
            int count = 1;
            foreach (LinkNode child in node.Nodes)
            {
                count += getCount(child);
            }
            return count;
        }

        public int getCount(TreeNodeCollection nodes)
        {
            int count = 0;
            foreach (LinkNode node in nodes)
            {
                count += getCount(node);
            }
            return count;
        }

        //Selects the components of a link. Helps highlight when the associated node is selected from the tree
        public void selectComponents(link Link, bool clearSelection, int mark = -1)
        {
            if (clearSelection)
            {
                ActiveSWModel.ClearSelection2(true);
            }
            SelectionMgr manager = ActiveSWModel.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            if (Link.SWComponent != null)
            {
                Link.SWComponent.Select4(true, data, false);
            }
            else
            {
                selectComponents(Link.SWcomponents, false);
            }
            foreach (link child in Link.Children)
            {
                selectComponents(child, false, mark);
            }
        }

        //Selects components from a list.
        public void selectComponents(List<Component2> components, bool clearSelection = true, int mark = -1)
        {
            if (clearSelection)
            {
                ActiveSWModel.ClearSelection2(true);
            }
            SelectionMgr manager = ActiveSWModel.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            foreach (Component2 component in components)
            {
                component.Select4(true, data, false);
            }
        }

        //Finds the selected components and returns them, used when pulling the items from the selection box because it would be too hard
        // for SolidWorks to allow you to access the selectionbox components directly.
        public void getSelectedComponents(List<Component2> Components, int Mark=-1)
        {
            SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
            Components.Clear();
            for (int i = 0; i < selectionManager.GetSelectedObjectCount2(Mark); i++)
            {
                object obj = selectionManager.GetSelectedObject6(i + 1, Mark);
                Component2 comp = (Component2)obj;
                if (comp != null)
                {
                    Components.Add(comp);
                }
            }
        }


        //finds all the hidden components, which will be added to a new display state. Also used when exporting STLs, so that hidden components
        //remain hidden
        public List<Component2> findHiddenComponents(object[] varComp)
        {
            List<Component2> hiddenComp = new List<Component2>();
            foreach (object obj in varComp)
            {
                Component2 comp = (Component2)obj;
                if (comp.IsHidden(false))
                {
                    hiddenComp.Add(comp);
                }
            }
            return hiddenComp;
        }

        //Except for an exclusionary list, this shows all the components
        public void showAllComponents(List<Component2> hiddenComponents)
        {
            AssemblyDoc assyDoc = (AssemblyDoc)ActiveSWModel;
            List<Component2> componentsToShow = new List<Component2>();
            object[] varComps = assyDoc.GetComponents(false);
            foreach (Component2 comp in varComps)
            {
                if (!hiddenComponents.Contains(comp))
                {
                    componentsToShow.Add(comp);
                }
            }
            showComponents(componentsToShow);
        }

        //Shows the components in the list. Useful  for exporting STLs
        public void showComponents(List<Component2> components)
        {
            selectComponents(components, true);
            ActiveSWModel.ShowComponent2();
        }

        //Shows the components in the link
        public void showComponents(link Link)
        {
            selectComponents(Link, true);
            ActiveSWModel.ShowComponent2();

        }

        //Hides the components from a link
        public void hideComponents(link Link)
        {
            selectComponents(Link, true);
            ActiveSWModel.HideComponent2();
        }

        //Hides the components from a list
        public void hideComponents(List<Component2> components)
        {
            selectComponents(components, true);
            ActiveSWModel.HideComponent2();
        }

        //Writes an empty header to the STL to get rid of the BS that SolidWorks adds to a binary STL file
        public void correctSTLMesh(string filename)
        {
            FileStream fileStream = new FileStream(filename, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
            byte[] emptyHeader = new byte[80];
            fileStream.Write(emptyHeader, 0, emptyHeader.Length);
            fileStream.Close();
        }
        #endregion

        #region STL Preference shuffling

        //Saves the preferences that the user had setup so that I can change them and revert back to their configuration
        public void saveUserPreferences()
        {
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
        public void setLinkSpecificSTLPreferences(string CoordinateSystemName, bool qualityFine)
        {
            ActiveSWModel.Extension.SetUserPreferenceString((int)swUserPreferenceStringValue_e.swFileSaveAsCoordinateSystem, (int)swUserPreferenceOption_e.swDetailingNoOptionSpecified, CoordinateSystemName);
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

        #region Testing new export method

        //Converts the SW component references to PIDs
        public void saveSWComponents(link Link)
        {
            ActiveSWModel.ClearSelection2(true);
            byte[] PID = saveSWComponent(Link.SWMainComponent);
            if (PID != null)
            {
                Link.SWMainComponentPID = PID;
            }
            Link.SWComponentPIDs = saveSWComponents(Link.SWcomponents);

            foreach (link Child in Link.Children)
            {
                saveSWComponents(Child);
            }
        }

        //Converts SW component references to PIDs
        public List<byte[]> saveSWComponents(List<Component2> components)
        {
            List<byte[]> PIDs = new List<byte[]>();
            foreach (Component2 component in components)
            {
                byte[] PID = saveSWComponent(component);
                if (PID != null)
                {
                    PIDs.Add(PID);
                }
            }
            return PIDs;
        }

        public byte[] saveSWComponent(Component2 component)
        {
            if (component != null)
            {
                return ActiveSWModel.Extension.GetPersistReference3(component);
            }
            return null;
        }

        public void retrieveSWComponentPIDs(LinkNode node)
        {
            if (node.Components != null)
            {
                node.ComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in node.Components)
                {
                    byte[] PID = ActiveSWModel.Extension.GetPersistReference3(comp);
                    node.ComponentPIDs.Add(PID);
                }
            }
            foreach (LinkNode child in node.Nodes)
            {
                retrieveSWComponentPIDs(child);
            }
        }

        public void retrieveSWComponentPIDs(TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                retrieveSWComponentPIDs(node);
            }
        }
        public void createBaseLinkFromComponents(LinkNode node)
        {
            // Build the link from the partdoc
            link Link = createLinkFromComponents(null, node.Components, node);
            createBaseRefOrigin(true);

            mRobot.BaseLink = Link;
        }
        public link createLink(LinkNode node, int count)
        {
            progressBar.UpdateTitle("Building link: " + node.Name);
            progressBar.UpdateProgress(count);
            link Link;
            if (node.isBaseNode)
            {
                createBaseLinkFromComponents(node);
                Link = mRobot.BaseLink;
            }
            else
            {
                LinkNode parentNode = (LinkNode)node.Parent;
                Link = createLinkFromComponents(parentNode.Link, node.Components, node);
            }
            node.Link = Link;
            foreach (LinkNode child in node.Nodes)
            {
                link childLink = createLink(child, count + 1);
                Link.Children.Add(childLink);
            }
            return Link;
        }

        public void createRobotFromTreeView(TreeView tree)
        {
            mRobot = new robot();
            

            progressBar.Start(0, getCount(tree.Nodes), "Building links");
            int count = 0;
            foreach (LinkNode node in tree.Nodes)
            {
                progressBar.UpdateProgress(count);
                progressBar.UpdateTitle("Building link: " + node.Name);
                count++;
                if (node.Level == 0)
                {
                    link BaseLink = createLink(node, 1);
                    mRobot.BaseLink = BaseLink;
                    node.Link = BaseLink;
                }
            }
            progressBar.End();
        }

        public link createLinkFromComponents(link parent, List<Component2> components, LinkNode node )
        {
            link child = new link();
            child.name = node.linkName;

            if (components.Count > 0)
            {
                child.isFixedFrame = false;
                child.Visual = new visual();
                child.Inertial = new inertial();
                child.Collision = new collision();
                child.SWMainComponent = components[0];
                child.SWcomponents.AddRange(components);
            }
            //Get link properties from SolidWorks part

            if (parent != null)
            {
                createJoint(parent, child, node);
            }

            string childCoordSysName = "";
            if (child.Joint == null)
            {
                childCoordSysName = "Origin_global";
            }
            else
            {
                childCoordSysName = child.Joint.CoordinateSystemName;
            }
            MathTransform jointTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(childCoordSysName);
            

            if (!child.isFixedFrame)
            {
                //selectComponents(components, true);
                IMassProperty swMass = ActiveSWModel.Extension.CreateMassProperty();
                swMass.SetCoordinateSystem(jointTransform);

                Body2[] bodies = getBodies(components);
                bool addedBodies = swMass.AddBodies(bodies);
                child.Inertial.Mass.value = swMass.Mass;
                double[] moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]
                child.Inertial.Inertia.setMomentMatrix(moment);

                double[] centerOfMass = swMass.CenterOfMass;
                child.Inertial.Origin.xyz = centerOfMass;
                child.Inertial.Origin.rpy = new double[3] { 0, 0, 0 };

                // Will this ever not be zeros?
                child.Visual.Origin.xyz = new double[3] { 0, 0, 0 };
                child.Visual.Origin.rpy = new double[3] { 0, 0, 0 };
                child.Collision.Origin.xyz = new double[3] { 0, 0, 0 };
                child.Collision.Origin.rpy = new double[3] { 0, 0, 0 };

                // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
                ModelDoc2 mainCompdoc = components[0].GetModelDoc2();
                double[] values = mainCompdoc.MaterialPropertyValues;
                child.Visual.Material.Color.Red = values[0];
                child.Visual.Material.Color.Green = values[1];
                child.Visual.Material.Color.Blue = values[2];
                child.Visual.Material.Color.Alpha = 1.0 - values[7];
                //child.Visual.Material.name = "material_" + child.name;

                //The part model doesn't actually know where the origin is, but the component does and this is important when exporting from assembly
                child.Visual.Origin.xyz = new double[] { 0, 0, 0 };
                child.Visual.Origin.rpy = new double[] { 0, 0, 0 };
                child.Collision.Origin.xyz = new double[] { 0, 0, 0 };
                child.Collision.Origin.rpy = new double[] { 0, 0, 0 };
            }

            //ActiveSWModel.ClearSelection2(true);
            return child;
        }

        public void createJoint(link parent, link child, LinkNode node)
        {
            checkRefGeometryExists(node);

            string jointName = node.jointName;
            string coordSysName = node.coordsysName;
            string axisName = node.axisName;
            string jointType = node.jointType;
            
            List<Component2> componentsToFix = fixComponents(parent);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            
            child.Joint = new joint();
            child.Joint.name = jointName;
            child.Joint.Parent.name = parent.name;
            child.Joint.Child.name = child.name;

            if (child.isFixedFrame)
            {
                axisName = "";
                jointType = "fixed";
                child.Joint.type = jointType;
            }
            else if (coordSysName == "Automatically Generate" || axisName == "Automatically Generate" || jointType == "Automatically Detect")
            {
                // We have to estimate the joint if the user specifies automatic for either the reference coordinate system, the reference axis or the joint type.
                estimateGlobalJointFromComponents(assy, parent, child);
            }

            if (coordSysName == "Automatically Generate")
            {
                child.Joint.CoordinateSystemName = "Origin_" + child.Joint.name;
                ActiveSWModel.ClearSelection2(true);
                int i = 2;
                while (ActiveSWModel.Extension.SelectByID2(child.Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
                {
                    ActiveSWModel.ClearSelection2(true);
                    child.Joint.CoordinateSystemName = "Origin_" + child.Joint.name + i.ToString();
                    i++;
                }
                createRefOrigin(child.Joint);
            }
            else
            {
                child.Joint.CoordinateSystemName = coordSysName;
            }
            if (axisName == "Automatically Generate")
            {
                child.Joint.AxisName = "Axis_" + child.Joint.name;
                ActiveSWModel.ClearSelection2(true);
                int i = 2;
                while (ActiveSWModel.Extension.SelectByID2(child.Joint.AxisName, "AXIS", 0, 0, 0, false, 0, null, 0))
                {
                    ActiveSWModel.ClearSelection2(true);
                    child.Joint.AxisName = "Axis_" + child.Joint.name + i.ToString();
                    i++;
                }
                createRefAxis(child.Joint);
            }
            else
            {
                child.Joint.AxisName = axisName;
            }
            if (jointType != "Automatically Detect")
            {
                child.Joint.type = jointType;
            }

            estimateGlobalJointFromRefGeometry(parent, child);

            coordSysName = (parent.Joint == null) ? "Origin_global" : parent.Joint.CoordinateSystemName;
            unFixComponents(componentsToFix);
            localizeJoint(child.Joint, coordSysName);
        }

        public Body2[] getBodies(List<Component2> components)
        {
            List<Body2> bodies = new List<Body2>();
            object BodiesInfo;
            object[] objs;
            foreach (Component2 comp in components)
            {
                objs = comp.GetBodies3((int)swBodyType_e.swAllBodies, out BodiesInfo);
                bodies.AddRange(Array.ConvertAll(objs, obj => (Body2)obj));
            }

            return bodies.ToArray();
        }

        public void checkRefGeometryExists(joint Joint)
        {
            if (!checkRefCoordsysExists(Joint.CoordinateSystemName))
            {
                Joint.CoordinateSystemName = "Automatically Generate";
            }
            if (!checkRefAxisExists(Joint.AxisName))
            {
                Joint.AxisName = "Automatically Generate";
            }
        }

        public void checkRefGeometryExists(LinkNode node)
        {
            if (!checkRefCoordsysExists(node.coordsysName))
            {
                node.coordsysName = "Automatically Generate";
            }
            if (!checkRefAxisExists(node.axisName))
            {
                node.axisName = "Automatically Generate";
            }
        }

        public bool checkRefCoordsysExists(string OriginName)
        {
            string[] Origins = findOrigins();
            return Origins.Contains(OriginName);
        }

        public bool checkRefAxisExists(string AxisName)
        {
            string[] Axes = findAxes();
            return Axes.Contains(AxisName);
        }

        private List<Component2> fixComponents(link parent)
        {
            List<Component2> componentsToUnfix = new List<Component2>();
            foreach (Component2 comp in parent.SWcomponents)
            {
                bool isFixed = comp.IsFixed();
                if (!comp.IsFixed())
                {
                    componentsToUnfix.Add(comp);
                }
            }
            selectComponents(parent.SWcomponents, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.FixComponent();
            return componentsToUnfix;
        }

        public void saveConfigTree(LinkNode BaseNode, bool warnUser)
        {
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            string oldData = "";
            Parameter param;
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        param = att.GetParameter("data");
                        oldData = param.GetStringValue();
                    }
                }

            }


            //moveComponentsToFolder((LinkNode)tree.Nodes[0]);
            retrieveSWComponentPIDs(BaseNode);
            SerialNode sNode = convertLinkNodeToSerialNode(BaseNode);
            StringWriter stringWriter;
            XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
            stringWriter = new StringWriter();
            serializer.Serialize(stringWriter, sNode);
            stringWriter.Flush();
            stringWriter.Close();

            string newData = stringWriter.ToString();
            if (oldData != newData)
            {
                if (!warnUser || (warnUser && MessageBox.Show("The configuration has changed, would you like to save?", "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes))
                {
                    int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = createSWSaveAttribute("URDF Export Configuration");
                    param = saveExporterAttribute.GetParameter("data");
                    param.SetStringValue2(stringWriter.ToString(), ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("name");
                    param.SetStringValue2("config1", ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("date");
                    param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
                }
            }
        }

        private SolidWorks.Interop.sldworks.Attribute createSWSaveAttribute(string name)
        {
            int Options = 0;
            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            ModelDoc2 modeldoc = iSwApp.ActiveDoc;
            Object[] objects = modeldoc.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == name)
                    {
                        return att;
                    }
                }

            }
            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute = saveConfigurationAttributeDef.CreateInstance5(ActiveSWModel, null, "URDF Export Configuration", Options, ConfigurationOptions);
            return saveExporterAttribute;
        }

        public LinkNode loadConfigTree()
        {
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            string data = "";
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        Parameter param = att.GetParameter("data");
                        data = param.GetStringValue();
                    }
                }

            }
            LinkNode lNode = null;
            if (!data.Equals(""))
            {
                XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
                XmlTextReader textReader = new XmlTextReader(new StringReader(data));
                SerialNode node = (SerialNode)serializer.Deserialize(textReader);
                lNode = convertSerialNodeToLinkNode(node);
                textReader.Close();
            }
            return lNode;
        }

        public void moveComponentsToFolder(LinkNode node)
        {
            bool needToCreateFolder = true;
            Object[] objects = ActiveSWModel.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                if (feat.Name == "URDF Export Items")
                {
                    needToCreateFolder = false;
                }
            }
            ActiveSWModel.ClearSelection2(true);
            ActiveSWModel.Extension.SelectByID2("Origin_global", "COORDSYS", 0, 0, 0, true, 0, null, 0);
            if (needToCreateFolder)
            {
                Feature folderFeature = ActiveSWModel.FeatureManager.InsertFeatureTreeFolder2((int)swFeatureTreeFolderType_e.swFeatureTreeFolder_Containing);
                folderFeature.Name =  "URDF Export Items";
            }
            ActiveSWModel.Extension.SelectByID2("URDF Reference", "SKETCH", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder( "URDF Export Items", "", false);
            ActiveSWModel.Extension.SelectByID2("URDF Export Configuration", "ATTRIBUTE", 0, 0, 0, true, 0, null, 0);
            ActiveSWModel.FeatureManager.MoveToFolder( "URDF Export Items", "", false);
            selectFeatures(node);
            ActiveSWModel.FeatureManager.MoveToFolder( "URDF Export Items", "", false);

        }

        public void selectFeatures(LinkNode node)
        {
            ActiveSWModel.Extension.SelectByID2(node.coordsysName, "COORDSYS", 0, 0, 0, true, -1, null, 0);
            if (node.axisName != "None")
            {
                ActiveSWModel.Extension.SelectByID2(node.axisName, "AXIS", 0, 0, 0, true, -1, null, 0);
            }
            foreach (LinkNode child in node.Nodes)
            {
                selectFeatures(child);
            }
        }

        public void checkIfLinkNamesAreUnique(LinkNode node, string linkName, List<string> conflict)
        {
            if (node.linkName == linkName)
            {
                conflict.Add(node.linkName);
            }

            foreach (LinkNode child in node.Nodes)
            {
                checkIfLinkNamesAreUnique(child, linkName, conflict);
            }
        }

        public void checkIfJointNamesAreUnique(LinkNode node, string jointName, List<string> conflict)
        {
            if (node.jointName == jointName)
            {
                conflict.Add(node.linkName);
            }
            foreach (LinkNode child in node.Nodes)
            {
                checkIfLinkNamesAreUnique(child, jointName, conflict);
            }

        }

        public bool checkIfNamesAreUnique(LinkNode node)
        {
            List<List<string>> linkConflicts = new List<List<string>>();
            List<List<string>> jointConflicts = new List<List<string>>();
            checkIfLinkNamesAreUnique(node, node, linkConflicts);
            checkIfJointNamesAreUnique(node, node, jointConflicts);

            string message = "\r\nPlease fix these errors before proceeding.";
            string specificErrors = "";
            bool displayInitialMessage = true;
            bool linkNamesInConflict = false;
            foreach (List<string> conflict in linkConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors += "The following links have LINK names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                
                }

            }
            displayInitialMessage = true;
            foreach (List<string> conflict in jointConflicts)
            {
                if (conflict.Count > 1)
                {
                    linkNamesInConflict = true;
                    if (displayInitialMessage)
                    {
                        specificErrors += "The following links have JOINT names that conflict:\r\n\r\n";
                        displayInitialMessage = false;
                    }
                    bool isFirst = true;
                    foreach (string linkName in conflict)
                    {
                        specificErrors += (isFirst) ? "     " + linkName : ", " + linkName;
                        isFirst = false;
                    }
                    specificErrors += "\r\n";
                }
            }
            if (linkNamesInConflict)
            {
                MessageBox.Show(specificErrors + message);
                return false;
            }
            return true;
        }

        public void checkIfLinkNamesAreUnique(LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();

            //Finds the conflicts of the currentNode with all the other nodes
            checkIfLinkNamesAreUnique(basenode, currentNode.linkName, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }
            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (LinkNode child in currentNode.Nodes)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts list of lists.
                checkIfLinkNamesAreUnique(basenode, child, conflicts);
            }
        }

        public void checkIfJointNamesAreUnique(LinkNode basenode, LinkNode currentNode, List<List<string>> conflicts)
        {
            List<string> conflict = new List<string>();
            
            //Finds the conflicts of the currentNode with all the other nodes
            checkIfJointNamesAreUnique(basenode, currentNode.jointName, conflict);
            bool alreadyExists = false;
            foreach (List<string> existingConflict in conflicts)
            {
                if (conflict.Count > 0 && existingConflict.Contains(conflict[0]))
                {
                    alreadyExists = true;
                }
            }

            if (!alreadyExists)
            {
                conflicts.Add(conflict);
            }
            foreach (LinkNode child in currentNode.Nodes)
            {
                //Proceeds recursively through the children nodes and adds to the conflicts list of lists.
                checkIfJointNamesAreUnique(basenode, child, conflicts);
            }
        }
        #endregion


    }
}