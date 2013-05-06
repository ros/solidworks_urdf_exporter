

using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace SW2URDF
{
    public partial class URDFExporter
    {
        private string referenceSketchName;

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

        // The one used by the Assembly Exporter
        public void createRobotFromTreeView(LinkNode baseNode)
        {
            mRobot = new robot();

            progressBar.Start(0, Common.getCount(baseNode.Nodes) + 1, "Building links");
            int count = 0;

            progressBar.UpdateProgress(count);
            progressBar.UpdateTitle("Building link: " + baseNode.Name);
            count++;

            link BaseLink = createLink(baseNode, 1);
            mRobot.BaseLink = BaseLink;
            baseNode.Link = BaseLink;

            progressBar.End();
        }

        public void createBaseLinkFromComponents(LinkNode node)
        {
            // Build the link from the partdoc
            link Link = createLinkFromComponents(null, node.Components, node);
            if (node.coordsysName == "Automatically Generate")
            {
                createBaseRefOrigin(true);
                node.coordsysName = "Origin_global";
                Link.CoordSysName = node.coordsysName;
            }
            else
            {
                Link.CoordSysName = node.coordsysName;
            }
            mRobot.BaseLink = Link;
        }

        //Method which builds an entire link and iterates through.
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

        //Method which builds a single link
        public link createLinkFromComponents(link parent, List<Component2> components, LinkNode node)
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
                childCoordSysName = node.coordsysName;
            }
            else
            {
                childCoordSysName = child.Joint.CoordinateSystemName;
            }

            // Get the SolidWorks MathTransform that corresponds to the child coordinate system
            MathTransform jointTransform = getCoordinateSystemTransform(childCoordSysName);

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

        public Body2[] getBodies(List<Component2> components)
        {
            List<Body2> bodies = new List<Body2>();
            object BodiesInfo;
            object[] objs;
            foreach (Component2 comp in components)
            {
                ModelDoc2 modeldoc = comp.GetModelDoc2();
                if (modeldoc != null && modeldoc.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
                {
                    object[] assyObjs = comp.GetChildren();
                    List<Component2> assyComponents = Array.ConvertAll(assyObjs, assyObj => (Component2)assyObj).ToList();
                    bodies.AddRange(getBodies(assyComponents));
                }
                else
                {
                    objs = comp.GetBodies3((int)swBodyType_e.swAllBodies, out BodiesInfo);
                    if (objs != null)
                    {
                        bodies.AddRange(Array.ConvertAll(objs, obj => (Body2)obj));
                    }
                }
            }

            return bodies.ToArray();
        }

        #endregion

        #region Joint methods

        //Base method for constructing a joint from a parent link and child link.
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
                if (child.Joint.type != "fixed")
                {
                    createRefAxis(child.Joint);
                }
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

            coordSysName = (parent.Joint == null) ? parent.CoordSysName : parent.Joint.CoordinateSystemName;
            unFixComponents(componentsToFix);
            localizeJoint(child.Joint, coordSysName);
        }

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
            MathTransform parentTransform = getCoordinateSystemTransform(parentCoordsysName);
            double[] parentRPY = ops.getRPY(parentTransform);


            Matrix<double> ParentJointGlobalTransform = ops.getTransformation(parentTransform);
            MathTransform coordsysTransform = getCoordinateSystemTransform(Joint.CoordinateSystemName);
            double[] coordsysRPY = ops.getRPY(coordsysTransform);

            //Transform from global origin to child joint
            Matrix<double> ChildJointGlobalTransform = ops.getTransformation(coordsysTransform);
            Matrix<double> ChildJointOrigin = ParentJointGlobalTransform.Inverse() * ChildJointGlobalTransform;
            double[] globalRPY = ops.getRPY(ChildJointOrigin);


            //Localize the axis to the Link's coordinate system.
            localizeAxis(Joint.Axis.xyz, Joint.CoordinateSystemName);

            // Get the array values and threshold them so small values are set to 0.
            Joint.Origin.xyz = ops.getXYZ(ChildJointOrigin);
            ops.threshold(Joint.Origin.xyz, 0.00001);
            Joint.Origin.rpy = ops.getRPY(ChildJointOrigin);
            ops.threshold(Joint.Origin.xyz, 0.00001);
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

        //This now needs to be able to get the component, and it's associated coordinate system name.
        //Then it needs to transform to the top level assembly (sounds like fun).
        public void estimateGlobalJointFromRefGeometry(link parent, link child)
        {
            MathTransform GlobalCoordsysTransform = getCoordinateSystemTransform(child.Joint.CoordinateSystemName);
            child.Joint.Origin.xyz = ops.getXYZ(GlobalCoordsysTransform);
            child.Joint.Origin.rpy = ops.getRPY(GlobalCoordsysTransform);
            if (child.Joint.type != "fixed")
            {
                estimateAxis(child.Joint);
            }
        }

        // Method to get the SolidWorks MathTransform from a coordinate system. This method can account for
        // coordinate systems that are embedded in subcomponents, and apply the correct transformation to return
        // it to a global transform. It assumes that the coordinate system name is formatted like:
        // "Coordinate System 1 <assy/subassy/comp>" where the full Component2.Name2 is between the <>
        public MathTransform getCoordinateSystemTransform(string CoordinateSystemName)
        {
            ModelDoc2 ComponentModel = ActiveSWModel;
            MathTransform ComponentTransform = default(MathTransform);
            if (CoordinateSystemName.Contains("<") && CoordinateSystemName.Contains(">"))
            {
                string componentStr = "";
                string CoordinateSystemNameUnTrimmed = "";
                int index_first = CoordinateSystemName.IndexOf('<');
                int index_last = CoordinateSystemName.IndexOf('>', index_first);
                if (index_last > index_first)
                {
                    componentStr = CoordinateSystemName.Substring(index_first + 1, index_last - index_first - 1);
                    CoordinateSystemNameUnTrimmed = CoordinateSystemName.Substring(0, index_first);
                    CoordinateSystemName = CoordinateSystemNameUnTrimmed.Trim();
                }
                AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
                object[] components = assy.GetComponents(false);
                foreach (Component2 comp in components)
                {
                    if (comp.Name2 == componentStr)
                    {
                        ComponentModel = comp.GetModelDoc2();
                        ComponentTransform = comp.Transform2;
                    }
                }
            }
            MathTransform LocalCoordsysTransform = ComponentModel.Extension.GetCoordinateSystemTransformByName(CoordinateSystemName);
            MathTransform GlobalCoordsysTransform = (ComponentTransform == null) ? LocalCoordsysTransform : LocalCoordsysTransform.Multiply(ComponentTransform);
            return GlobalCoordsysTransform;
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
            string coordsys = (parent.Joint == null) ? parent.CoordSysName : parent.Joint.CoordinateSystemName;
            MathTransform parentTransform = getCoordinateSystemTransform(coordsys);
            double[] idealOrigin = ops.closestPointOnLineToPoint(ops.getXYZ(parentTransform), nonLocalizedChild.Joint.Axis.xyz, nonLocalizedChild.Joint.Origin.xyz);

            nonLocalizedChild.Joint.Origin.xyz = ops.closestPointOnLineWithinBox(X_min, X_max, Y_min, Y_max, Z_min, Z_max, nonLocalizedChild.Joint.Axis.xyz, idealOrigin);
        }

        // Calculates the axis from a Reference Axis in the model
        public void estimateAxis(joint Joint)
        {
            Joint.Axis.xyz = estimateAxis(Joint.AxisName);
        }

        //This doesn't seem to get the right values for the estimatedAxis. Check the actual values
        public double[] estimateAxis(string axisName)
        {

            //Select the axis
            ActiveSWModel.ClearSelection2(true);

            return getRefAxis(axisName);
        }

        public double[] getRefAxis(string axisStr)
        {
            ModelDoc2 ComponentModel = ActiveSWModel;
            string axisName = axisStr;
            RefAxis axis = default(RefAxis);
            MathTransform ComponentTransform = default(MathTransform);


            if (axisStr.Contains("<") && axisStr.Contains(">"))
            {
                string componentStr = "";
                string CoordinateSystemNameUnTrimmed = "";
                int index_first = axisStr.IndexOf('<');
                int index_last = axisStr.IndexOf('>', index_first);
                if (index_last > index_first)
                {
                    componentStr = axisStr.Substring(index_first + 1, index_last - index_first - 1);
                    CoordinateSystemNameUnTrimmed = axisStr.Substring(0, index_first);
                    axisName = CoordinateSystemNameUnTrimmed.Trim();
                }
                AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
                object[] components = assy.GetComponents(false);
                foreach (Component2 comp in components)
                {
                    if (comp.Name2 == componentStr)
                    {
                        ComponentModel = comp.GetModelDoc2();
                        ComponentTransform = comp.Transform2;
                    }
                }
            }
            bool selected = ComponentModel.Extension.SelectByID2(axisName, "AXIS", 0, 0, 0, false, 0, null, 0);
            if (selected)
            {
                Feature feat = ComponentModel.SelectionManager.GetSelectedObject6(1, 0);
                axis = (RefAxis)feat.GetSpecificFeature2();
            }
            //Calculate!
            double[] axisParams;
            double[] XYZ = new double[3];
            axisParams = axis.GetRefAxisParams();
            XYZ[0] = axisParams[0] - axisParams[3];
            XYZ[1] = axisParams[1] - axisParams[4];
            XYZ[2] = axisParams[2] - axisParams[5];
            XYZ = ops.pnorm(XYZ, 2);
            globalAxis(XYZ, ComponentTransform);
            return XYZ;
        }

        //This is called whenever the pull down menu is changed and the axis needs to be recalculated in reference to the coordinate system
        public void localizeAxis(double[] Axis, string coordsys)
        {
            MathTransform coordsysTransform = getCoordinateSystemTransform(coordsys);
            localizeAxis(Axis, coordsysTransform);
        }

        // This is called by the above method and the getRefAxis method
        public void localizeAxis(double[] Axis, MathTransform coordsysTransform)
        {
            if (coordsysTransform != null)
            {
                Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
                Matrix<double> transform = ops.getTransformation(coordsysTransform);
                vec = transform.Inverse() * vec;
                Axis[0] = vec[0]; Axis[1] = vec[1]; Axis[2] = vec[2];
            }
            ops.threshold(Axis, 0.00001);
        }

        public void globalAxis(double[] Axis, MathTransform coordsysTransform)
        {
            if (coordsysTransform != null)
            {
                Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
                Matrix<double> transform = ops.getTransformation(coordsysTransform);
                vec = transform * vec;
                Axis[0] = vec[0]; Axis[1] = vec[1]; Axis[2] = vec[2];
            }
            ops.threshold(Axis, 0.00001);
        }

        // Creates a list of all the features of this type.
        public Dictionary<string, List<Feature>> getFeaturesOfType(string featureName, bool topLevelOnly)
        {
            Dictionary<string, List<Feature>> features = new Dictionary<string, List<Feature>>();
            getFeaturesOfType(null, featureName, topLevelOnly, features);
            return features;
        }

        public void getFeaturesOfType(Component2 component, string featureName, bool topLevelOnly, Dictionary<string, List<Feature>> features)
        {
            ModelDoc2 modeldoc;
            string ComponentName = "";
            if (component == null)
            {
                modeldoc = ActiveSWModel;
            }
            else
            {
                modeldoc = component.GetModelDoc2();
                ComponentName = component.Name2;
            }
            features[ComponentName] = new List<Feature>();

            object[] featureObjects;
            featureObjects = modeldoc.FeatureManager.GetFeatures(false);

            foreach (Feature feat in featureObjects)
            {
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == featureName)
                {
                    features[ComponentName].Add(feat);
                }
            }

            if (!topLevelOnly && modeldoc.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                AssemblyDoc assyDoc = (AssemblyDoc)modeldoc;

                //Get all components in an assembly
                object[] components = assyDoc.GetComponents(false);
                foreach (Component2 comp in components)
                {
                    ModelDoc2 doc = comp.GetModelDoc2();
                    if (doc != null)
                    {
                        //We already have all the components in an assembly, we don't want to recur as we go through them. (topLevelOnly = true)
                        getFeaturesOfType(comp, featureName, true, features);
                    }
                }
            }
        }

        public Dictionary<string, string> GetComponentRefGeoNames(string StringToParse)
        {
            string RefGeoName = "";
            string ComponentName = "";
            if (StringToParse.Contains("<") && StringToParse.Contains(">"))
            {
                string RefGeoNameUnTrimmed = "";
                int index_first = StringToParse.IndexOf('<');
                int index_last = StringToParse.IndexOf('>', index_first);
                if (index_last > index_first)
                {
                    ComponentName = StringToParse.Substring(index_first + 1, index_last - index_first - 1);
                    RefGeoNameUnTrimmed = StringToParse.Substring(0, index_first);
                    RefGeoName = RefGeoNameUnTrimmed.Trim();
                }
            }

            Dictionary<string, string> dict = new Dictionary<string, string>();
            dict["geo"] = RefGeoName;
            dict["component"] = ComponentName;
            return dict;
        }

        public List<string> FindRefGeoNames(string FeatureName)
        {
            Dictionary<string, List<Feature>> features = getFeaturesOfType(FeatureName, false);
            List<string> featureNames = new List<string>();
            foreach (string key in features.Keys)
            {
                foreach (Feature feat in features[key])
                {
                    Entity ent = (Entity)feat;
                    Component2 comp = (Component2)ent.GetComponent();
                    if (key != "")
                    {
                        featureNames.Add(feat.Name + " <" + key + ">");
                    }
                    else
                    {
                        featureNames.Add(feat.Name);
                    }
                }
            }
            return featureNames;
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
                    Joint.Limit = new limit();
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
            Common.selectComponents(ActiveSWModel, components, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.UnfixComponent();
        }

        //Verifies that the reference geometry still exists. This can happen if the reference geometry was deleted but the configuration was kept
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
            List<string> Origins = FindRefGeoNames("CoordSys");
            return Origins.Contains(OriginName);
        }

        public bool checkRefAxisExists(string AxisName)
        {
            List<string> Axes = FindRefGeoNames("RefAxis");
            return Axes.Contains(AxisName);
        }

        //Used to fix components to estimate the degree of freedom.
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
            Common.selectComponents(ActiveSWModel, parent.SWcomponents, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.FixComponent();
            return componentsToUnfix;
        }
        #endregion
    }
}
