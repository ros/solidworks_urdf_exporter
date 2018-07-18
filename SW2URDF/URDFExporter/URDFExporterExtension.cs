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
        public void CreateRobotFromActiveModel()
        {
            URDFRobot = new Robot();
            URDFRobot.Name = ActiveSWModel.GetTitle();

            Configuration swConfig = ActiveSWModel.ConfigurationManager.ActiveConfiguration;
            foreach (string state in swConfig.GetDisplayStates())
            {
                if (state.Equals("URDF Export"))
                {
                    swConfig.ApplyDisplayState("URDF Export");
                }
            }

            //Each Robot contains a single base link, build this link
            URDFRobot.BaseLink = CreateBaseLinkFromActiveModel();
        }

        // This method now only works for the part exporter
        public Link CreateBaseLinkFromActiveModel()
        {
            // If the model is a part
            if (ActiveSWModel.GetType() == (int)swDocumentTypes_e.swDocPART)
            {
                return CreateLinkFromPartModel(ActiveSWModel);
            }
            return null;
        }

        // This creates a Link from a Part ModelDoc. It basically just extracts the material
        // properties and saves them to the appropriate fields.
        public Link CreateLinkFromPartModel(ModelDoc2 swModel)
        {
            Link Link = new Link(null);
            Link.Name = swModel.GetTitle();

            Link.isFixedFrame = false;

            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            Link.Inertial.Mass.Value = swMass.Mass;

            // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]
            double[] moment = swMass.GetMomentOfInertia(
                (int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass);
            Link.Inertial.Inertia.SetMomentMatrix(moment);

            double[] centerOfMass = swMass.CenterOfMass;
            Link.Inertial.Origin.SetXYZ(centerOfMass);
            Link.Inertial.Origin.SetRPY(new double[3] { 0, 0, 0 });

            // Will this ever not be zeros?
            Link.Visual.Origin.SetXYZ(new double[3] { 0, 0, 0 });
            Link.Visual.Origin.SetRPY(new double[3] { 0, 0, 0 });
            Link.Collision.Origin.SetXYZ(new double[3] { 0, 0, 0 });
            Link.Collision.Origin.SetRPY(new double[3] { 0, 0, 0 });

            // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
            double[] values = swModel.MaterialPropertyValues;
            Link.Visual.Material.Color.Red = values[0];
            Link.Visual.Material.Color.Green = values[1];
            Link.Visual.Material.Color.Blue = values[2];
            Link.Visual.Material.Color.Alpha = 1.0 - values[7];
            Link.Visual.Material.Name = "material_" + Link.Name;

            return Link;
        }

        //This is only used by the Part Exporter, but it localizes the link to the Origin_global
        // coordinate system
        public void LocalizeLink(Link Link, Matrix<double> GlobalTransform)
        {
            Matrix<double> GlobalTransformInverse = GlobalTransform.Inverse();
            Matrix<double> linkCoMTransform = MathOps.GetTranslation(Link.Inertial.Origin.GetXYZ());
            Matrix<double> localLinkCoMTransform = GlobalTransformInverse * linkCoMTransform;

            Matrix<double> linkVisualTransform =
                MathOps.GetTransformation(Link.Visual.Origin.GetXYZ(), Link.Visual.Origin.GetRPY());
            Matrix<double> localVisualTransform = GlobalTransformInverse * linkVisualTransform;

            Matrix<double> linkCollisionTransform =
                MathOps.GetTransformation(Link.Collision.Origin.GetXYZ(), Link.Collision.Origin.GetRPY());
            Matrix<double> localCollisionTransform =
                GlobalTransformInverse * linkCollisionTransform;

            // The linear array in Link.Inertial.Inertia.Moment is in row major order, but this
            // matrix constructor uses column major order. It's a rotation matrix, so this
            // shouldn't matter. If it does, just transpose linkGlobalMomentInertia. These three
            // matrices are 3x3 as opposed to the 4x4 transformation matrices above.
            // You're welcome for the confusion.
            Matrix<double> linkGlobalMomentInertia =
                new DenseMatrix(3, 3, Link.Inertial.Inertia.GetMoment());
            Matrix<double> GlobalRotMat =
                GlobalTransform.SubMatrix(0, 3, 0, 3);
            Matrix<double> linkLocalMomentInertia =
                GlobalRotMat * linkGlobalMomentInertia * GlobalRotMat.Transpose();

            Link.Inertial.Origin.SetXYZ(MathOps.GetXYZ(localLinkCoMTransform));
            Link.Inertial.Origin.SetRPY(new double[] { 0, 0, 0 });

            // Wait are you saying that even though the matrix was trasposed from column major
            // order, you are writing it in row-major order here. Yes, yes I am.
            double[] moment = linkLocalMomentInertia.ToRowWiseArray();
            Link.Inertial.Inertia.SetMomentMatrix(moment);

            Link.Collision.Origin.SetXYZ(MathOps.GetXYZ(localCollisionTransform));
            Link.Collision.Origin.SetRPY(MathOps.GetRPY(localCollisionTransform));

            Link.Visual.Origin.SetXYZ(MathOps.GetXYZ(localVisualTransform));
            Link.Visual.Origin.SetRPY(MathOps.GetRPY(localVisualTransform));
        }

        // The one used by the Assembly Exporter
        public void CreateRobotFromTreeView(LinkNode baseNode)
        {
            URDFRobot = new Robot();

            progressBar.Start(0, Common.GetCount(baseNode.Nodes) + 1, "Building links");
            int count = 0;

            progressBar.UpdateProgress(count);
            progressBar.UpdateTitle("Building link: " + baseNode.Name);
            count++;

            Link BaseLink = CreateLink(baseNode, 1);
            URDFRobot.BaseLink = BaseLink;
            baseNode.Link = BaseLink;

            progressBar.End();
        }

        public void CreateBaseLinkFromComponents(LinkNode node)
        {
            // Build the link from the partdoc
            Link Link = CreateLinkFromComponents(null, node.Components, node);
            if (node.CoordsysName == "Automatically Generate")
            {
                CreateBaseRefOrigin(true);
                node.CoordsysName = "Origin_global";
                Link.CoordSysName = node.CoordsysName;
            }
            else
            {
                Link.CoordSysName = node.CoordsysName;
            }
            URDFRobot.BaseLink = Link;
        }

        //Method which builds an entire link and iterates through.
        public Link CreateLink(LinkNode node, int count)
        {
            progressBar.UpdateTitle("Building link: " + node.Name);
            progressBar.UpdateProgress(count);
            Link Link;
            if (node.IsBaseNode)
            {
                CreateBaseLinkFromComponents(node);
                Link = URDFRobot.BaseLink;
            }
            else
            {
                LinkNode parentNode = (LinkNode)node.Parent;
                Link = CreateLinkFromComponents(parentNode.Link, node.Components, node);
            }
            node.Link = Link;
            foreach (LinkNode child in node.Nodes)
            {
                Link childLink = CreateLink(child, count + 1);
                Link.Children.Add(childLink);
            }
            return Link;
        }

        //Method which builds a single link
        public Link CreateLinkFromComponents(Link parent, List<Component2> components, LinkNode node)
        {
            Link child = new Link(parent);
            child.Name = node.LinkName;

            if (components.Count > 0)
            {
                child.isFixedFrame = false;
                child.SWMainComponent = components[0];
                child.SWcomponents.AddRange(components);
            }
            //Get link properties from SolidWorks part

            if (parent != null)
            {
                logger.Info("Creating joint " + child.Name);
                CreateJoint(parent, child, node);
            }

            string childCoordSysName = "";
            if (child.Joint.CoordinateSystemName == null)
            {
                childCoordSysName = node.CoordsysName;
            }
            else
            {
                childCoordSysName = child.Joint.CoordinateSystemName;
            }

            // Get the SolidWorks MathTransform that corresponds to the child coordinate system
            MathTransform jointTransform = GetCoordinateSystemTransform(childCoordSysName);

            if (!child.isFixedFrame)
            {
                List<Body2> bodies = GetBodies(components);
                MassProperty swMass = ActiveSWModel.Extension.CreateMassProperty();
                swMass.SetCoordinateSystem(jointTransform);

                bool bRet = swMass.AddBodies(bodies.ToArray());

                double[] moment = (double[])swMass.GetMomentOfInertia(
                    (int)swMomentsOfInertiaReferenceFrame_e.swMomentsOfInertiaReferenceFrame_CenterOfMass);
                child.Inertial.Mass.Value = swMass.Mass;
                child.Inertial.Inertia.SetMomentMatrix(moment);

                double[] centerOfMass = swMass.CenterOfMass;
                child.Inertial.Origin.SetXYZ(centerOfMass);
                child.Inertial.Origin.SetRPY(new double[3] { 0, 0, 0 });

                // Will this ever not be zeros?
                child.Visual.Origin.SetXYZ(new double[3] { 0, 0, 0 });
                child.Visual.Origin.SetRPY(new double[3] { 0, 0, 0 });
                child.Collision.Origin.SetXYZ(new double[3] { 0, 0, 0 });
                child.Collision.Origin.SetRPY(new double[3] { 0, 0, 0 });

                // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
                ModelDoc2 mainCompdoc = components[0].GetModelDoc2();
                double[] values = mainCompdoc.MaterialPropertyValues;
                child.Visual.Material.Color.Red = values[0];
                child.Visual.Material.Color.Green = values[1];
                child.Visual.Material.Color.Blue = values[2];
                child.Visual.Material.Color.Alpha = 1.0 - values[7];
                //child.Visual.Material.name = "material_" + child.name;

                //The part model doesn't actually know where the origin is, but the component
                // does and this is important when exporting from assembly
                child.Visual.Origin.SetXYZ(new double[] { 0, 0, 0 });
                child.Visual.Origin.SetRPY(new double[] { 0, 0, 0 });
                child.Collision.Origin.SetXYZ(new double[] { 0, 0, 0 });
                child.Collision.Origin.SetRPY(new double[] { 0, 0, 0 });
            }

            return child;
        }

        public List<Body2> GetBodies(List<Component2> components)
        {
            object bodyInfo = null;
            List<Body2> bodies = new List<Body2>();
            foreach (Component2 comp in components)
            {
                // Retreiving the Body2 bodies of the component. Also need to recur through the assembly tree
                object[] componentBodies =
                    (object[])comp.GetBodies3((int)swBodyType_e.swSolidBody, out bodyInfo);
                if (componentBodies != null)
                {
                    foreach (Body2 obj in componentBodies)
                    {
                        bodies.Add(obj);
                    }
                }
                object[] children = comp.GetChildren();
                if (children != null)
                {
                    List<Component2> childComponents = new List<Component2>();
                    foreach (Component2 child in children)
                    {
                        childComponents.Add(child);
                    }
                    bodies.AddRange(GetBodies(childComponents));
                }
            }
            return bodies;
        }

        #endregion SW to Robot and link methods

        #region Joint methods

        //Base method for constructing a joint from a parent link and child link.
        public void CreateJoint(Link parent, Link child, LinkNode node)
        {
            CheckRefGeometryExists(node);

            string jointName = node.JointName;
            string coordSysName = node.CoordsysName;
            string axisName = node.AxisName;
            string jointType = node.JointType;

            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            List<Component2> fixedComponents = FixComponents(parent);
            child.Joint.Name = jointName;
            child.Joint.Parent.Name = parent.Name;
            child.Joint.Child.Name = child.Name;
            Boolean unfix = false;
            if (child.isFixedFrame)
            {
                axisName = "";
                jointType = "fixed";
                child.Joint.Type = jointType;
            }
            else if (coordSysName == "Automatically Generate" ||
                axisName == "Automatically Generate" || jointType == "Automatically Detect")
            {
                // We have to estimate the joint if the user specifies automatic for either the
                // reference coordinate system, the reference axis or the joint type.
                unfix = EstimateGlobalJointFromComponents(assy, parent, child);
            }

            if (coordSysName == "Automatically Generate")
            {
                child.Joint.CoordinateSystemName = "Origin_" + child.Joint.Name;
                ActiveSWModel.ClearSelection2(true);
                int i = 2;
                while (ActiveSWModel.Extension.SelectByID2(
                    child.Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
                {
                    ActiveSWModel.ClearSelection2(true);
                    child.Joint.CoordinateSystemName =
                        "Origin_" + child.Joint.Name + i.ToString();
                    i++;
                }
                CreateRefOrigin(child.Joint);
            }
            else
            {
                child.Joint.CoordinateSystemName = coordSysName;
            }
            if (axisName == "Automatically Generate")
            {
                child.Joint.AxisName = "Axis_" + child.Joint.Name;
                ActiveSWModel.ClearSelection2(true);
                int i = 2;
                while (ActiveSWModel.Extension.SelectByID2(
                    child.Joint.AxisName, "AXIS", 0, 0, 0, false, 0, null, 0))
                {
                    ActiveSWModel.ClearSelection2(true);
                    child.Joint.AxisName = "Axis_" + child.Joint.Name + i.ToString();
                    i++;
                }
                if (child.Joint.Type != "fixed")
                {
                    CreateRefAxis(child.Joint);
                }
            }
            else
            {
                child.Joint.AxisName = axisName;
            }
            if (jointType != "Automatically Detect")
            {
                child.Joint.Type = jointType;
            }

            EstimateGlobalJointFromRefGeometry(parent, child);

            coordSysName = (parent.Joint == null) ?
                parent.CoordSysName : parent.Joint.CoordinateSystemName;
            if (unfix)
            {
                UnFixComponents(fixedComponents);
            }
            LocalizeJoint(child.Joint, coordSysName);
        }

        // Creates a Reference Coordinate System in the SolidWorks Model to symbolize the joint location
        public void CreateRefOrigin(Joint Joint)
        {
            CreateRefOrigin(Joint.Origin, Joint.CoordinateSystemName);
        }

        // Creates a Reference Coordinate System in the SolidWorks Model to symbolize the joint location
        public void CreateRefOrigin(Origin Origin, string CoordinateSystemName)
        {
            // Adds the sketch segments and point to the 3D sketch. The sketchEnties are the actual
            // items created (and their locations)
            object[] sketchEntities = AddSketchGeometry(Origin);

            SketchPoint OriginPoint = (SketchPoint)sketchEntities[0];
            SketchSegment xaxis = (SketchSegment)sketchEntities[1];
            SketchSegment yaxis = (SketchSegment)sketchEntities[2];

            double originX = (double)sketchEntities[3]; //OriginPoint X
            double originY = (double)sketchEntities[4];
            double originZ = (double)sketchEntities[5];

            double xAxisX = (double)sketchEntities[6];
            double xAxisY = (double)sketchEntities[7];
            double xAxisZ = (double)sketchEntities[8];

            double yAxisX = (double)sketchEntities[9];
            double yAxisY = (double)sketchEntities[10];
            double yAxisZ = (double)sketchEntities[11];

            IFeature coordinates = default(IFeature);
            ActiveSWModel.ClearSelection2(true);
            SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
            SelectData data = selectionManager.CreateSelectData();

            // First select the origin
            bool SelectedOrigin = false;
            bool SelectedXAxis = false;
            bool SelectedYAxis = false;
            if (OriginPoint != null)
            {
                data.Mark = 1;
                SelectedOrigin = OriginPoint.Select4(true, data);
            }
            if (!SelectedOrigin)
            {
                SelectedOrigin =
                    ActiveSWModel.Extension.SelectByID2(
                        "", "EXTSKETCHPOINT", originX, originY, originZ, true, 1, null, 0);
            }

            // Second, select the xaxis
            if (xaxis != null)
            {
                data.Mark = 2;
                SelectedXAxis = xaxis.Select4(true, data);
            }
            if (!SelectedXAxis)
            {
                SelectedXAxis =
                    ActiveSWModel.Extension.SelectByID2
                    ("", "EXTSKETCHPOINT", xAxisX, xAxisY, xAxisZ, true, 2, null, 0);
            }

            // Third, select the yaxis
            if (yaxis != null)
            {
                data.Mark = 4;
                SelectedYAxis = yaxis.Select4(true, data);
            }
            if (!SelectedYAxis)
            {
                SelectedYAxis =
                    ActiveSWModel.Extension.SelectByID2(
                        "", "EXTSKETCHPOINT", yAxisX, yAxisY, yAxisZ, true, 4, null, 0);
            }

            //From the selected items, insert a coordinate system.
            coordinates =
                ActiveSWModel.FeatureManager.InsertCoordinateSystem(false, false, false);
            if (coordinates != null)
            {
                coordinates.Name = CoordinateSystemName;
            }
        }

        //Creates the Origin_global coordinate system
        public void CreateBaseRefOrigin(bool zIsUp)
        {
            if (!ActiveSWModel.Extension.SelectByID2(
                    "Origin_global", "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                Joint Joint = new Joint();
                if (zIsUp)
                {
                    Joint.Origin.SetRPY(new double[] { -Math.PI / 2, 0, 0 });
                }
                else
                {
                    Joint.Origin.SetRPY(new double[] { 0, 0, 0 });
                }
                Joint.Origin.SetXYZ(new double[] { 0, 0, 0 });
                Joint.CoordinateSystemName = "Origin_global";
                if (referenceSketchName == null)
                {
                    referenceSketchName = Setup3DSketch();
                }
                CreateRefOrigin(Joint);
            }
        }

        // Creates a Reference Axis to be used to calculate the joint axis
        public void CreateRefAxis(Joint Joint)
        {
            //Adds sketch segment
            SketchSegment rotaxis = AddSketchGeometry(Joint.Axis, Joint.Origin);
            if (rotaxis != null)
            {
                //Use special method to create the axis
                Feature featAxis = InsertAxis(rotaxis);
                if (featAxis != null)
                {
                    featAxis.Name = Joint.AxisName;
                }
            }
        }

        // Takes a links joint and calculates the local transform from the global transforms of
        // the parent and child. It also converts the axis to local values
        public void LocalizeJoint(Joint Joint, string parentCoordsysName)
        {
            MathTransform parentTransform = GetCoordinateSystemTransform(parentCoordsysName);
            double[] parentRPY = MathOps.GetRPY(parentTransform);

            Matrix<double> ParentJointGlobalTransform =
                MathOps.GetTransformation(parentTransform);
            MathTransform coordsysTransform =
                GetCoordinateSystemTransform(Joint.CoordinateSystemName);
            double[] coordsysRPY = MathOps.GetRPY(coordsysTransform);

            //Transform from global origin to child joint
            Matrix<double> ChildJointGlobalTransform =
                MathOps.GetTransformation(coordsysTransform);
            Matrix<double> ChildJointOrigin =
                ParentJointGlobalTransform.Inverse() * ChildJointGlobalTransform;
            double[] globalRPY = MathOps.GetRPY(ChildJointOrigin);

            //Localize the axis to the Link's coordinate system.
            Joint.Axis.SetXYZ(LocalizeAxis(Joint.Axis.GetXYZ(), Joint.CoordinateSystemName));

            // Get the array values and threshold them so small values are set to 0.
            Joint.Origin.SetXYZ(MathOps.GetXYZ(ChildJointOrigin));
            Joint.Origin.SetXYZ(MathOps.Threshold(Joint.Origin.GetXYZ(), 0.00001));
            Joint.Origin.SetRPY(MathOps.GetRPY(ChildJointOrigin));
            Joint.Origin.SetRPY(MathOps.Threshold(Joint.Origin.GetRPY(), 0.00001));
        }

        // Funny method I created that inserts a RefAxis and then finds the reference to it.
        public Feature InsertAxis(SketchSegment axis)
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
                    //If the feature in featuresAfter is not in features before, its gotta be the
                    // axis we inserted
                    if (!featuresBefore.Contains(feat))
                    {
                        return feat;
                    }
                }
            }
            return null;
        }

        // Inserts a sketch into the main assembly and name it
        public string Setup3DSketch()
        {
            bool sketchExists =
                ActiveSWModel.Extension.SelectByID2(
                    "URDF Reference", "SKETCH", 0, 0, 0, false, 0, null, 0);
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
        public object[] AddSketchGeometry(Origin Origin)
        {
            //Find if the sketch exists first
            if (ActiveSWModel.SketchManager.ActiveSketch == null)
            {
                bool sketchExists =
                    ActiveSWModel.Extension.SelectByID2(
                        referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
                ActiveSWModel.SketchManager.Insert3DSketch(true);
            }
            IFeature sketch = (IFeature)ActiveSWModel.SketchManager.ActiveSketch;

            //Calculate the lines that need to be drawn
            Matrix<double> transform = MathOps.GetRotation(Origin.GetRPY());
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
            // Return an array of objects representing the sketch items that were just inserted,
            // as well as the actual locations of those objecs (aids selection).
            return new object[] { OriginPoint, XAxis, YAxis,
                Origin.X, Origin.Y, Origin.Z,
                Origin.X + tA[0, 0], Origin.Y + tA[1, 0], Origin.Z + tA[2, 0],
                Origin.X + tA[0, 1], Origin.Y + tA[1, 1], Origin.Z + tA[2, 1] };
        }

        //Inserts a sketch segment for use when creating a Reference Axis
        public SketchSegment AddSketchGeometry(Axis Axis, Origin Origin)
        {
            if (ActiveSWModel.SketchManager.ActiveSketch == null)
            {
                bool sketchExists =
                    ActiveSWModel.Extension.SelectByID2(
                        referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
                ActiveSWModel.SketchManager.Insert3DSketch(true);
            }

            //Insert sketch segment 0.1m long centered on the origin.
            SketchSegment RotAxis = ActiveSWModel.SketchManager.CreateLine(
                Origin.X - 0.05 * Axis.X,
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

        //Calculates the free degree of freedom (if exists), and then determines the location of the joint,
        // the axis of rotation/translation, and the type of joint
        public Boolean EstimateGlobalJointFromComponents(AssemblyDoc assy, Link parent, Link child)
        {
            //Create the ref objects
            int DOFs;

            // Surpress Limit Mates to properly find degrees of freedom. They don't work with the API call
            List<Mate2> limitMates = new List<Mate2>();
            limitMates = SuppressLimitMates(child.SWMainComponent);
            Boolean success = false;
            if (child.SWMainComponent != null)
            {
                // The wonderful undocumented API call I found to get the degrees of freedom in a joint.
                // https://forum.solidworks.com/thread/57414
                int remainingDOFs =
                    child.SWMainComponent.GetRemainingDOFs(
                        out int R1Status, out MathPoint RPoint1, out int R1DirStatus, out MathVector RDir1,
                        out int R2Status, out MathPoint RPoint2, out int R2DirStatus, out MathVector RDir2,
                        out int L1Status, out MathVector LDir1,
                        out int L2Status, out MathVector LDir2);
                if (RPoint1 != null)
                {
                    logger.Info("R1: " + R1Status + ", " + RPoint1 + ", " + R1DirStatus + ", " + RDir1.get_IArrayData());
                }
                else
                {
                    logger.Info("R1: " + R1Status + ", " + R1DirStatus);
                }

                if (RPoint2 != null)
                {
                    logger.Info("R2: " + R2Status + ", " + RPoint2 + ", " + R2DirStatus + ", " + RDir2.get_IArrayData());
                }
                else
                {
                    logger.Info("R2: " + R2Status + ", " + R2DirStatus);
                }
                if (LDir1 != null)
                {
                    logger.Info("L1: " + L1Status + ", " + LDir1.get_IArrayData());
                }
                else
                {
                    logger.Info("L1: " + L1Status);
                }
                if (LDir2 != null)
                {
                    logger.Info("L2: " + ", " + LDir2.get_IArrayData());
                }
                else
                {
                    logger.Info("L2: " + L2Status);
                }

                DOFs = remainingDOFs;

                // Convert the gotten degrees of freedom to a joint type, origin and axis
                child.Joint.Type = "fixed";
                child.Joint.Origin.SetXYZ(MathOps.GetXYZ(child.SWMainComponent.Transform2));
                child.Joint.Origin.SetRPY(MathOps.GetRPY(child.SWMainComponent.Transform2));

                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    success = true;
                    if (R1Status == 1)
                    {
                        child.Joint.Type = "continuous";
                        child.Joint.Axis.SetXYZ(RDir1.ArrayData);
                        child.Joint.Origin.SetXYZ(RPoint1.ArrayData);
                        child.Joint.Origin.SetRPY(MathOps.GetRPY(child.SWMainComponent.Transform2));
                        MoveOrigin(parent, child);
                    }
                    else if (L1Status == 1)
                    {
                        child.Joint.Type = "prismatic";
                        child.Joint.Axis.SetXYZ(LDir1.ArrayData);
                        child.Joint.Origin.SetXYZ(MathOps.GetXYZ(child.SWMainComponent.Transform2));
                        child.Joint.Origin.SetRPY(MathOps.GetRPY(child.SWMainComponent.Transform2));
                        MoveOrigin(parent, child);
                    }
                }
                child.Joint.Origin.SetXYZ(MathOps.Threshold(child.Joint.Origin.GetXYZ(), 0.00001));
                child.Joint.Origin.SetRPY(MathOps.Threshold(child.Joint.Origin.GetRPY(), 0.00001));
                UnsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    AddLimits(child.Joint, limitMates, parent.SWMainComponent, child.SWMainComponent);
                }
            }
            return success;
        }

        //This now needs to be able to get the component, and it's associated coordinate system name.
        //Then it needs to transform to the top level assembly (sounds like fun).
        public void EstimateGlobalJointFromRefGeometry(Link parent, Link child)
        {
            MathTransform GlobalCoordsysTransform =
                GetCoordinateSystemTransform(child.Joint.CoordinateSystemName);
            child.Joint.Origin.SetXYZ(MathOps.GetXYZ(GlobalCoordsysTransform));
            child.Joint.Origin.SetRPY(MathOps.GetRPY(GlobalCoordsysTransform));
            if (child.Joint.Type != "fixed")
            {
                EstimateAxis(child.Joint);
            }
        }

        // Method to get the SolidWorks MathTransform from a coordinate system. This method can account for
        // coordinate systems that are embedded in subcomponents, and apply the correct transformation to return
        // it to a global transform. It assumes that the coordinate system name is formatted like:
        // "Coordinate System 1 <assy/subassy/comp>" where the full Component2.Name2 is between the <>
        public MathTransform GetCoordinateSystemTransform(string CoordinateSystemName)
        {
            ModelDoc2 ComponentModel = ActiveSWModel;
            MathTransform ComponentTransform = default(MathTransform);
            if (CoordinateSystemName.Contains("<") && CoordinateSystemName.Contains(">"))
            {
                string componentStr = "";
                string CoordinateSystemNameUnTrimmed = "";
                int indexFirst = CoordinateSystemName.IndexOf('<');
                int indexLast = CoordinateSystemName.IndexOf('>', indexFirst);
                if (indexLast > indexFirst)
                {
                    componentStr =
                        CoordinateSystemName.Substring(indexFirst + 1, indexLast - indexFirst - 1);
                    CoordinateSystemNameUnTrimmed = CoordinateSystemName.Substring(0, indexFirst);
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
            MathTransform LocalCoordsysTransform =
                ComponentModel.Extension.GetCoordinateSystemTransformByName(CoordinateSystemName);
            MathTransform GlobalCoordsysTransform = (ComponentTransform == null) ?
                LocalCoordsysTransform : LocalCoordsysTransform.Multiply(ComponentTransform);
            return GlobalCoordsysTransform;
        }

        public void MoveOrigin(Link parent, Link nonLocalizedChild)
        {
            double xMax = Double.MinValue;
            double yMax = Double.MinValue;
            double zMax = Double.MinValue;
            double xMin = Double.MaxValue;
            double yMin = Double.MaxValue;
            double zMin = Double.MaxValue;
            double[] points;

            foreach (Component2 comp in nonLocalizedChild.SWcomponents)
            {
                // Returns box as [ XCorner1, YCorner1, ZCorner1, XCorner2, YCorner2, ZCorner2 ]
                points = comp.GetBox(false, false);
                xMax = MathOps.Max(points[0], points[3], xMax);
                yMax = MathOps.Max(points[1], points[4], yMax);
                zMax = MathOps.Max(points[2], points[5], zMax);
                xMin = MathOps.Min(points[0], points[3], xMin);
                yMin = MathOps.Min(points[1], points[4], yMin);
                zMin = MathOps.Min(points[2], points[5], zMin);
            }
            string coordsys = (parent.Joint == null) ?
                parent.CoordSysName : parent.Joint.CoordinateSystemName;
            MathTransform parentTransform = GetCoordinateSystemTransform(coordsys);

            double[] xyzParent = MathOps.GetXYZ(parentTransform);
            double[] xyzJointAxis = nonLocalizedChild.Joint.Axis.GetXYZ();
            double[] xyzOrigin = nonLocalizedChild.Joint.Origin.GetXYZ();
            double[] idealOrigin =
                MathOps.ClosestPointOnLineToPoint(xyzParent, xyzJointAxis, xyzOrigin);

            nonLocalizedChild.Joint.Origin.SetXYZ(
                MathOps.ClosestPointOnLineWithinBox(xMin, xMax, yMin, yMax, zMin, zMax,
                    nonLocalizedChild.Joint.Axis.GetXYZ(), idealOrigin));
        }

        // Calculates the axis from a Reference Axis in the model
        public void EstimateAxis(Joint Joint)
        {
            Joint.Axis.SetXYZ(EstimateAxis(Joint.AxisName));
        }

        //This doesn't seem to get the right values for the estimatedAxis. Check the actual values
        public double[] EstimateAxis(string axisName)
        {
            //Select the axis
            ActiveSWModel.ClearSelection2(true);

            return GetRefAxis(axisName);
        }

        public double[] GetRefAxis(string axisStr)
        {
            ModelDoc2 ComponentModel = ActiveSWModel;
            string axisName = axisStr;
            RefAxis axis = default(RefAxis);
            MathTransform ComponentTransform = default(MathTransform);

            if (axisStr.Contains("<") && axisStr.Contains(">"))
            {
                string componentStr = "";
                string CoordinateSystemNameUnTrimmed = "";
                int indexFirst = axisStr.IndexOf('<');
                int indexLast = axisStr.IndexOf('>', indexFirst);
                if (indexLast > indexFirst)
                {
                    componentStr = axisStr.Substring(indexFirst + 1, indexLast - indexFirst - 1);
                    CoordinateSystemNameUnTrimmed = axisStr.Substring(0, indexFirst);
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
            //Calculate!
            double[] axisParams;
            double[] xyz = new double[3];

            bool selected =
                ComponentModel.Extension.SelectByID2(axisName, "AXIS", 0, 0, 0, false, 0, null, 0);
            if (selected)
            {
                Feature feat = ComponentModel.SelectionManager.GetSelectedObject6(1, 0);
                axis = (RefAxis)feat.GetSpecificFeature2();

                // GetRefAxisParams returns {startX, startY, startZ, endX, endY, endZ}
                axisParams = axis.GetRefAxisParams();
                xyz[0] = axisParams[0] - axisParams[3];
                xyz[1] = axisParams[1] - axisParams[4];
                xyz[2] = axisParams[2] - axisParams[5];

                // Normalize and cleanup
                xyz = MathOps.PNorm(xyz, 2);
                if (MathOps.Sum(xyz) < 0.0)
                {
                    xyz = MathOps.Flip(xyz);
                }

                // Transform to proper coordinates
                GlobalAxis(xyz, ComponentTransform);
            }

            return xyz;
        }

        //This is called whenever the pull down menu is changed and the axis needs to be
        // recalculated in reference to the coordinate system
        public double[] LocalizeAxis(double[] Axis, string coordsys)
        {
            MathTransform coordsysTransform = GetCoordinateSystemTransform(coordsys);
            return LocalizeAxis(Axis, coordsysTransform);
        }

        // This is called by the above method and the getRefAxis method
        public double[] LocalizeAxis(double[] Axis, MathTransform coordsysTransform)
        {
            if (coordsysTransform != null)
            {
                Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
                Matrix<double> transform = MathOps.GetTransformation(coordsysTransform);
                vec = transform.Inverse() * vec;
                Axis[0] = vec[0]; Axis[1] = vec[1]; Axis[2] = vec[2];
            }
            return MathOps.Threshold(Axis, 0.00001);
        }

        public double[] GlobalAxis(double[] Axis, MathTransform coordsysTransform)
        {
            if (coordsysTransform != null)
            {
                Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
                Matrix<double> transform = MathOps.GetTransformation(coordsysTransform);
                vec = transform * vec;
                Axis[0] = vec[0]; Axis[1] = vec[1]; Axis[2] = vec[2];
            }
            return MathOps.Threshold(Axis, 0.00001);
        }

        // Creates a list of all the features of this type.
        public Dictionary<string, List<Feature>> GetFeaturesOfType(string featureName, bool topLevelOnly)
        {
            Dictionary<string, List<Feature>> features = new Dictionary<string, List<Feature>>();
            GetFeaturesOfType(null, featureName, topLevelOnly, features);
            return features;
        }

        public void GetFeaturesOfType(Component2 component, string featureName,
            bool topLevelOnly, Dictionary<string, List<Feature>> features)
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

                // If there are no components in an assembly, this object will be null.
                if (components != null)
                {
                    foreach (Component2 comp in components)
                    {
                        ModelDoc2 doc = comp.GetModelDoc2();
                        if (doc != null)
                        {
                            //We already have all the components in an assembly, we don't want
                            // to recur as we go through them. (topLevelOnly = true)
                            GetFeaturesOfType(comp, featureName, true, features);
                        }
                    }
                }
            }
        }

        public Dictionary<string, string> GetComponentRefGeoNames(string StringToParse)
        {
            string RefGeoName = StringToParse;
            string ComponentName = "";
            if (StringToParse.Contains("<") && StringToParse.Contains(">"))
            {
                string RefGeoNameUnTrimmed = "";
                int indexFirst = StringToParse.IndexOf('<');
                int indexLast = StringToParse.IndexOf('>', indexFirst);
                if (indexLast > indexFirst)
                {
                    ComponentName = StringToParse.Substring(indexFirst + 1, indexLast - indexFirst - 1);
                    RefGeoNameUnTrimmed = StringToParse.Substring(0, indexFirst);
                    RefGeoName = RefGeoNameUnTrimmed.Trim();
                }
            }

            Dictionary<string, string> dict = new Dictionary<string, string>
            {
                ["geo"] = RefGeoName,
                ["component"] = ComponentName
            };
            return dict;
        }

        public List<string> FindRefGeoNames(string FeatureName)
        {
            Dictionary<string, List<Feature>> features = GetFeaturesOfType(FeatureName, false);
            List<string> featureNames = new List<string>();
            foreach (string key in features.Keys)
            {
                foreach (Feature feat in features[key])
                {
                    Entity ent = (Entity)feat;
                    Component2 comp = (Component2)ent.GetComponent();
                    if (String.IsNullOrWhiteSpace(key))
                    {
                        featureNames.Add(feat.Name);
                    }
                    else
                    {
                        featureNames.Add(feat.Name + " <" + key + ">");
                    }
                }
            }
            return featureNames;
        }

        //This method adds in the limits from a limit mate, to make a joint a revolute joint.
        // It really needs to checked for correctness.
        public void AddLimits(Joint Joint, List<Mate2> limitMates,
            Component2 parentComponent, Component2 childComponent)
        {
            logger.Info("Parent SW Component: " + parentComponent.Name2);
            logger.Info("Child SW Component: " + childComponent.Name2);
            // The number of limit Mates should only be one. But for completeness, I cycle through
            // every found limit mate.
            foreach (Mate2 swMate in limitMates)
            {
                logger.Info("Determining limit mate eligibility ");
                List<Component2> entities = new List<Component2>();
                for (int i = 0; i < swMate.GetMateEntityCount(); i++)
                {
                    MateEntity2 entity = swMate.MateEntity(i);
                    entities.Add(entity.ReferenceComponent);
                    logger.Info("Adding component entity: " + entity.ReferenceComponent.Name2);

                    Component2 parent = entity.ReferenceComponent.GetParent();
                    while (parent != null)
                    {
                        logger.Info("Adding component entity: " + parent.Name2);
                        entities.Add(parent);
                        parent = parent.GetParent();
                    }
                }

                if (entities.Contains(parentComponent) && entities.Contains(childComponent))
                {
                    // [TODO] This assumes the limit mate limits the right degree of freedom,
                    // it really should check that assumption
                    if ((Joint.Type == "continuous" && swMate.Type ==
                            (int)swMateType_e.swMateANGLE) ||
                        (Joint.Type == "prismatic" && swMate.Type ==
                            (int)swMateType_e.swMateDISTANCE))
                    {
                        // Unclear if flipped is the right thing we want to be checking here.
                        // From a sample size of 1, in SolidWorks it appears that an aligned and
                        // anti-aligned mates are NOT flipped...
                        if (!swMate.Flipped)
                        {
                            // Reverse mate directions, for some reason
                            Joint.Limit.Upper = -swMate.MinimumVariation;
                            Joint.Limit.Lower = -swMate.MaximumVariation;
                        }
                        else
                        {
                            // Lucky me that no conversion is necessary
                            Joint.Limit.Upper = swMate.MaximumVariation;
                            Joint.Limit.Lower = swMate.MinimumVariation;
                        }
                        if (Joint.Type == "continuous")
                        {
                            Joint.Type = "revolute";
                        }
                    }
                }
            }
        }

        // Suppresses limit mates to make it easier to find the free degree of freedom in a joint
        public List<Mate2> SuppressLimitMates(IComponent2 component)
        {
            ModelDoc2 modelDoc = component.GetModelDoc2();
            List<Mate2> limitMates = new List<Mate2>();

            object[] objs = component.GetMates();

            //limit mates aren't always present
            if (objs != null)
            {
                foreach (object obj in objs)
                {
                    if (obj is Mate2 swMate)
                    {
                        if (swMate.MinimumVariation != swMate.MaximumVariation)
                        {
                            limitMates.Add(swMate);
                        }
                    }
                }
            }

            foreach (Mate2 swMate in limitMates)
            {
                ModelDoc2 doc = component.GetModelDoc2();
                Feature feat = (Feature)swMate;
                feat.Select(false);
                feat.SetSuppression2((int)swFeatureSuppressionAction_e.swSuppressFeature,
                    (int)swInConfigurationOpts_e.swThisConfiguration, null);
            }

            return limitMates;
        }

        // Unsuppresses limit mates that were suppressed before
        public void UnsuppressLimitMates(List<Mate2> limitMates)
        {
            foreach (Mate2 swMate in limitMates)
            {
                Feature feat = (Feature)swMate;
                feat.SetSuppression2((int)swFeatureSuppressionAction_e.swUnSuppressFeature,
                    (int)swInConfigurationOpts_e.swThisConfiguration, null);
            }
        }

        //Unfixes components that were fixed to find the free degree of freedom
        public void UnFixComponents(List<Component2> components)
        {
            foreach (Component2 comp in components)
            {
                logger.Info("Unfixing component " + comp.GetID());
            }

            Common.SelectComponents(ActiveSWModel, components, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.UnfixComponent();
        }

        //Verifies that the reference geometry still exists. This can happen if the reference
        // geometry was deleted but the configuration was kept
        public void CheckRefGeometryExists(Joint Joint)
        {
            if (!CheckRefCoordsysExists(Joint.CoordinateSystemName))
            {
                Joint.CoordinateSystemName = "Automatically Generate";
            }
            if (!CheckRefAxisExists(Joint.AxisName))
            {
                Joint.AxisName = "Automatically Generate";
            }
        }

        public void CheckRefGeometryExists(LinkNode node)
        {
            if (!CheckRefCoordsysExists(node.CoordsysName))
            {
                node.CoordsysName = "Automatically Generate";
            }
            if (!CheckRefAxisExists(node.AxisName))
            {
                node.AxisName = "Automatically Generate";
            }
        }

        public bool CheckRefCoordsysExists(string OriginName)
        {
            List<string> Origins = FindRefGeoNames("CoordSys");
            return Origins.Contains(OriginName);
        }

        public bool CheckRefAxisExists(string AxisName)
        {
            List<string> Axes = FindRefGeoNames("RefAxis");
            return Axes.Contains(AxisName);
        }

        private List<Component2> GetParentAncestorComponents(Link node)
        {
            List<Component2> components = new List<Component2>(node.SWcomponents);
            if (node.Parent != null)
            {
                components.AddRange(GetParentAncestorComponents(node.Parent));
            }
            return components;
        }

        //Used to fix components to estimate the degree of freedom.
        private List<Component2> FixComponents(Link parent)
        {
            logger.Info("Fixing components for " + parent.Name);
            List<Component2> componentsToFix = GetParentAncestorComponents(parent);
            List<Component2> componentsToUnfix = new List<Component2>();
            foreach (Component2 comp in componentsToFix)
            {
                logger.Info("Fixing " + comp.GetID());
                bool isFixed = comp.IsFixed();
                if (!comp.IsFixed())
                {
                    componentsToUnfix.Add(comp);
                }
                else
                {
                    logger.Info("Component " + comp.GetID() + " is already fixed");
                }
            }
            Common.SelectComponents(ActiveSWModel, componentsToFix, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.FixComponent();
            return componentsToUnfix;
        }

        #endregion Joint methods
    }
}