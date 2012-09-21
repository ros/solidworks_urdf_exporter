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
    [Serializable]
    public class SW2URDFExporter
    {
        #region class variables
        [XmlIgnore] public ISldWorks iSwApp = null;
        [XmlIgnore] ops OPS;
        private bool mBinary;
        private bool mshowInfo;
        private bool mSTLPreview;
        private bool mTranslateToPositive;
        private bool mSaveComponentsIntoOneFile;
        private int mSTLUnits;
        private int mSTLQuality;
        private double mHideTransitionSpeed;
        private string referenceSketchName;


        [XmlIgnore] public ModelDoc2 ActiveSWModel;
        [XmlIgnore] public MathUtility swMath;
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
        private SW2URDFExporter()
        {
            OPS = new ops();
        }
        public SW2URDFExporter(ISldWorks iSldWorksApp)
        {
            constructExporter(iSldWorksApp);

            mSavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            mPackageName = ActiveSWModel.GetTitle();
        }

        private void constructExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;

            swMath = iSwApp.GetMathUtility();
            OPS = new ops();

            saveConfigurationAttributeDef = iSwApp.DefineAttribute("URDF Export Configuration");
            int Options = 0;

            saveConfigurationAttributeDef.AddParameter("data", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("name", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("date", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter("exporterVersion", (int)swParamType_e.swParamTypeDouble, 1.0, Options);
            saveConfigurationAttributeDef.Register();
        }

        #region SW to Robot and link methods
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

        public link createBaseLinkFromActiveModel()
        {
            if (ActiveSWModel.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY) // If the model is an Assembly
            {
                return createBaseLinkFromAssy(ActiveSWModel);
            }
            else if (ActiveSWModel.GetType() == (int)swDocumentTypes_e.swDocPART) // If the model is a part
            {
                return createLinkFromPartModel(ActiveSWModel);
            }
            return null;
        }

        public link createLinkFromPartModel(ModelDoc2 swModel)
        {
            link Link = new link();
            Link.name = swModel.GetTitle();

            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            Link.Inertial.Mass.Value = swMass.Mass;

            Link.Inertial.Inertia.Moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]

            double[] centerOfMass = swMass.CenterOfMass;
            Link.Inertial.Origin.XYZ = centerOfMass;
            Link.Inertial.Origin.RPY = new double[3] { 0, 0, 0 };

            // Will this ever not be zeros?
            Link.Visual.Origin.XYZ = new double[3] { 0, 0, 0 };
            Link.Visual.Origin.RPY = new double[3] { 0, 0, 0 };
            Link.Collision.Origin.XYZ = new double[3] { 0, 0, 0 };
            Link.Collision.Origin.RPY = new double[3] { 0, 0, 0 };

            // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
            double[] values = swModel.MaterialPropertyValues;
            Link.Visual.Material.Color.Red = values[0];
            Link.Visual.Material.Color.Green = values[1];
            Link.Visual.Material.Color.Blue = values[2];
            Link.Visual.Material.Color.Alpha = 1.0 - values[7];
            Link.Visual.Material.name = "material_" + Link.name;

            return Link;
        }

        public link createBaseLinkFromAssy(ModelDoc2 swModel)
        {
            AssemblyDoc swAssy = (AssemblyDoc)swModel;

            swAssy.ResolveAllLightWeightComponents(true);
            // Getting the components (assemblies and parts, contained in this assembly)
            object[] varComp = swAssy.GetComponents(true);

            //For building tree through assembly hierachy (more robust)
            link baseLink = new link();

            // Iterate through each component and create a 'sparse branch'
            foreach (IComponent2 comp in varComp)
            {
                link sparseLink = createSparseBranchFromComponents(comp, 0);
                if (sparseLink != null)
                {
                    baseLink.Children.Add(sparseLink);
                }
            }

            //From this sparse link, promote the child links to the parent positions
            return createDenseTree(baseLink, 0);
        }

        public link createLinkFromPartComp(object comp, int level)
        {
            IComponent2 partComp = (IComponent2)comp;
            ModelDoc2 partDoc = partComp.GetModelDoc();
            link Link;

            if (partDoc == null)
            {
                throw new System.InvalidOperationException("Component " + partComp.Name2 + " is null");
            }

            // Build the link from the partdoc
            Link = createLinkFromPartModel(partDoc);

            //The part model doesn't actually know where the origin is, but the component does and this is important when exporting from assembly
            Link.Visual.Origin.XYZ = new double[] { 0, 0, 0 };
            Link.Visual.Origin.RPY = new double[] { 0, 0, 0 };
            Link.Collision.Origin.XYZ = new double[] { 0, 0, 0 };
            Link.Collision.Origin.RPY = new double[] { 0, 0, 0 };

            Link.SWComponent = partComp;
            Link.SWComponentLevel = level;
            Link.uniqueName = partComp.Name2;
            return Link;
        }

        // This method creates a 'sparse branch' where for temporary reasons, an assembly as assigned to the parent links
        // Then each part component is a leaf of the branch, but are not set as parent links yet.
        public link createSparseBranchFromComponents(IComponent2 comp, int level)
        {
            link Link = new link();

            //If the component is hidden or suppressed, it is not included in the tree.
            if (comp.IsHidden(true))
            {
                return null;
            }

            //If the component is a part, create the link from the part. Otherwise recur through this method
            ModelDoc2 modelDoc = comp.GetModelDoc2();
            if (modelDoc.GetType() == (int)swDocumentTypes_e.swDocPART)
            {
                Link = createLinkFromPartComp(comp, level);
            }
            else if (modelDoc.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                object[] children = comp.GetChildren();
                foreach (IComponent2 child in children)
                {
                    link childLink = createSparseBranchFromComponents(child, level + 1);
                    if (childLink != null)
                    {
                        Link.Children.Add(childLink);
                    }
                }
            }
            Link.SWComponent = comp;
            Link.SWComponentLevel = level;
            // If for some reason all of the children were null and this one is an assembly, then it doesn't make sense to add it to the tree.
            // Return null instead.
            if (Link.Children.Count == 0 && modelDoc.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                Link = null;
            }
            return Link;
        }

        // This method converts a dense tree from a sparse one by assigning leaves to the parent links
        public link createDenseTree(link sparseTree, int level)
        {
            // For this empty link, find the child link that is the best fit for parenting
            link parentLink = findParent(sparseTree, level + 1);

            if (parentLink == null)
            {
                return sparseTree;
            }
            if (parentLink.Children.Count == 0)
            {
                return parentLink;
            }

            List<link> linksToRemove = new List<link>();
            List<link> linksToAdd = new List<link>();
            // Iterate through children to continue finding the best parents
            foreach (link child in parentLink.Children)
            {
                // Only bother if the component is not hidden and not supressed
                if (!child.SWComponent.IsHidden(true))
                {
                    linksToAdd.Add(createDenseTree(child, level + 1));
                    linksToRemove.Add(child);
                }
            }
            // Remove unorganized links
            foreach (link Link in linksToRemove)
            {
                parentLink.Children.Remove(Link);
            }
            // Add organized links
            foreach (link Link in linksToAdd)
            {
                parentLink.Children.Add(Link);
            }
            return parentLink;
        }

        // From a links children, this method finds the 'best' choice as a parent
        public link findParent(link top, int level)
        {
            if (top.Children.Count > 0)
            {
                link AssignedParentLink = new link();
                int priorityLevel = -1;
                double largestFixedMass = -1;
                double largestPartMass = -1;
                double largestAssyMass = 0;

                // Iterate through children to find the 'best' component for parent status. It may be several assemblies down.
                foreach (link child in top.Children)
                {
                    if (!child.SWComponent.IsHidden(true))
                    {
                        ModelDoc2 ChildDoc = child.SWComponent.GetModelDoc();
                        if (ChildDoc == null)
                        {
                            throw new System.InvalidOperationException("Component " + child.SWComponent.Name2 + " is null");
                        }
                        int ChildType = (int)ChildDoc.GetType();

                        IMassProperty childMass = ChildDoc.Extension.CreateMassProperty();

                        double mass = childMass.Mass;

                        //Highest priority is the largest fixed component
                        if (child.SWComponent.IsFixed() && mass > largestFixedMass)
                        {
                            priorityLevel = 2;
                            AssignedParentLink = child;
                            largestFixedMass = mass;
                        }
                        //Second highest priority is the largest floating part
                        else if (mass > largestPartMass && ChildType == (int)swDocumentTypes_e.swDocPART && priorityLevel < 2)
                        {
                            priorityLevel = 1;
                            AssignedParentLink = child;
                            largestPartMass = mass;
                        }
                        //Third priority is the 'best' choice from the largest assembly
                        else if (mass > largestAssyMass && ChildType == (int)swDocumentTypes_e.swDocASSEMBLY && priorityLevel < 1)
                        {
                            priorityLevel = 0;
                            AssignedParentLink = child;
                            largestAssyMass = mass;
                        }
                    }
                }

                ModelDoc2 AssignedParentDoc = AssignedParentLink.SWComponent.GetModelDoc();
                int AssignedParentType = AssignedParentDoc.GetType();
                top.Children.Remove(AssignedParentLink);
                // If a fixed component was chosen and it is an assembly, iterate through assembly
                if (priorityLevel == 2 && AssignedParentType == (int)swDocumentTypes_e.swDocASSEMBLY)
                {
                    AssignedParentLink = findParent(AssignedParentLink, level + 1);

                }
                // If no parts were found, iterate through the chosen assembly
                else if (priorityLevel == 0)
                {
                    AssignedParentLink = findParent(AssignedParentLink, level + 1);
                }
                AssignedParentLink.Children.AddRange(top.Children);
                return AssignedParentLink;
            }
            return top;
        }

        #endregion

        #region Joint methods
        //Iterates through each link to create the joints between the parent and child

        public void createJoints(link Parent, bool zIsUp)
        {
            Matrix<double> ParentJointGlobalTransform;
            if (Parent.Joint != null)
            {
                //If the parent joint exists, it becomes the reference joint. Grab the MathTransform of that coordsys to use for localizing
                MathTransform coordSysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(Parent.Joint.CoordinateSystemName);
                ParentJointGlobalTransform = OPS.getTransformation(coordSysTransform);
            }
            else
            {
                //If the parent is the base_link then set the reference for the child's joint to the global origin
                createBaseRefOrigin(zIsUp);
                MathTransform coordSysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
                ParentJointGlobalTransform = OPS.getTransformation(coordSysTransform);
            }
            foreach (link Child in Parent.Children)
            {
                AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
                Child.Joint = new joint();
                Child.Joint = createJointName(Parent, Child);

                // Fix parent component to eliminate its degrees of freedom from this joint
                IComponent2 compToFix = findCompToFix(Parent, Child);
                bool isFixed = compToFix.IsFixed();
                compToFix.Select(false);
                assy.FixComponent();


                // First creates a globabl transform, that is from the joint to the origin of the assembly
                Child.Joint = createGlobalJoint(Parent, Child);
                // Localize the joint by creating transforms between the child joint and the parent's global transform
                localizeJoint(Child, ParentJointGlobalTransform);
                // Iterate through this links children
                createJoints(Child, zIsUp);

                unFixComponents(assy, compToFix, isFixed);
            }
        }


        public joint createJointName(link Parent, link Child)
        {
            Child.Joint.name = Parent.uniqueName + "_to_" + Child.uniqueName;
            Child.Joint.CoordinateSystemName = "Origin_" + Child.Joint.name;
            Child.Joint.AxisName = "Axis_" + Child.Joint.name;
            Child.Joint.Parent.name = Parent.uniqueName;
            Child.Joint.Child.name = Child.uniqueName;
            return Child.Joint;
        }
        public void createJointName2(link Parent, link Child)
        {
            string jointName = Parent.uniqueName + "_to_" + Child.uniqueName;
            createJointName2(Parent, Child, jointName);
        }
        public void createJointName2(link Parent, link Child, string jointName)
        {
            Child.Joint.name = jointName;
            Child.Joint.CoordinateSystemName = "Origin_" + Child.Joint.name;
            Child.Joint.AxisName = "Axis_" + Child.Joint.name;
            Child.Joint.Parent.name = Parent.uniqueName;
            Child.Joint.Child.name = Child.uniqueName;
        }


        public void createRefGeometry(joint Joint)
        {
            if (!ActiveSWModel.Extension.SelectByID2(Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                createRefOrigin(Joint);
            }
            if (!ActiveSWModel.Extension.SelectByID2(Joint.AxisName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                createRefAxis(Joint);
            }
        }
        public void createRefOrigin(joint Joint)
        {
            createRefOrigin(Joint.Origin, Joint.CoordinateSystemName);
        }

        public void createRefOrigin(origin Origin, string CoordinateSystemName)
        {
            object[] sketchEntities = addSketchGeometry(Origin);
            SketchPoint OriginPoint = (SketchPoint)sketchEntities[0];
            SketchSegment xaxis = (SketchSegment)sketchEntities[1];
            SketchSegment yaxis = (SketchSegment)sketchEntities[2];

            double X = (double)sketchEntities[3];
            double Y = (double)sketchEntities[4];
            double Z = (double)sketchEntities[5];

            double xX = (double)sketchEntities[6];
            double xY = (double)sketchEntities[7];
            double xZ = (double)sketchEntities[8];

            double yX = (double)sketchEntities[9];
            double yY = (double)sketchEntities[10];
            double yZ = (double)sketchEntities[11];
            IFeature coordinates = default(IFeature);
            ActiveSWModel.ClearSelection2(true);
            SelectionMgr selectionManager = ActiveSWModel.SelectionManager;
            SelectData data = selectionManager.CreateSelectData();

            if (xaxis != null && yaxis != null)
            {
                bool SelectedOrigin = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", X, Y, Z, true, 1, null, 0);
                bool SelectedXAxis = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", xX, xY, xZ, true, 2, null, 0);
                bool SelectedYAxis = ActiveSWModel.Extension.SelectByID2("", "EXTSKETCHPOINT", yX, yY, yZ, true, 4, null, 0);

                coordinates = ActiveSWModel.FeatureManager.InsertCoordinateSystem(false, false, false);
                coordinates.Name = CoordinateSystemName;
            }
        }
        public void createBaseRefOrigin(bool zIsUp)
        {
            if (!ActiveSWModel.Extension.SelectByID2("Origin_global", "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                joint Joint = new joint();
                Joint.Origin = new origin();
                if (zIsUp)
                {
                    Joint.Origin.RPY = new double[] { -Math.PI / 2, 0, 0 };
                }
                else
                {
                    Joint.Origin.RPY = new double[] { 0, 0, 0 };
                }
                Joint.Origin.XYZ = new double[] { 0, 0, 0 };
                Joint.CoordinateSystemName = "Origin_global";
                if (referenceSketchName == null)
                {
                    referenceSketchName = setup3DSketch();
                }
                createRefOrigin(Joint);
            }
        }
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

        public joint createGlobalJoint(link parent, link child)
        {
            joint Joint = estimateGlobalJointFromComponents((AssemblyDoc)ActiveSWModel, parent, child);
            if (!ActiveSWModel.Extension.SelectByID2(child.Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, false, 0, null, 0) &&
                !ActiveSWModel.Extension.SelectByID2(child.Joint.AxisName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                createRefGeometry(Joint);
            }
            //child.Joint = estimateJointFromRefGeometry(ActiveSWModel, child);
            return child.Joint;
        }

        public void localizeJoint(link Link, Matrix<double> ParentJointGlobalTransform)
        {
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(Link.Joint.CoordinateSystemName);
            //Transform from global origin to child joint
            Matrix<double> ChildJointGlobalTransform = OPS.getTransformation(coordsysTransform);
            Matrix<double> ChildJointLocalTransform = ParentJointGlobalTransform.Inverse() * ChildJointGlobalTransform;

            Vector<double> Axis = new DenseVector(new double[] { Link.Joint.Axis.X, Link.Joint.Axis.Y, Link.Joint.Axis.Z, 0 });
            Axis = ChildJointGlobalTransform.Inverse() * Axis;
            Axis = Axis.Normalize(2);

            Matrix<double> linkCoMTransform = OPS.getTranslation(Link.Inertial.Origin.XYZ);
            Matrix<double> localLinkCoMTransform = ChildJointGlobalTransform.Inverse() * linkCoMTransform;

            // The linear array in Link.Inertial.Inertia.Moment is in row major order, but this matrix constructor uses column major order
            // It's a rotation matrix, so this shouldn't matter. If it does, just transpose linkGlobalMomentInertia
            // These three matrices are 3x3 as opposed to the 4x4 transformation matrices above. You're welcome for the confusion.
            Matrix<double> linkGlobalMomentInertia = new DenseMatrix(3, 3, Link.Inertial.Inertia.Moment);
            Matrix<double> ChildJointRotMat = ChildJointGlobalTransform.SubMatrix(0, 3, 0, 3);
            Matrix<double> linkLocalMomentInertia = ChildJointRotMat.Inverse() * linkGlobalMomentInertia;

            //Save the data from the transforms
            Link.Joint.Axis.XYZ = new double[] { Axis[0], Axis[1], Axis[2] };

            Link.Joint.Origin.XYZ = OPS.getXYZ(ChildJointLocalTransform);
            Link.Joint.Origin.RPY = OPS.getRPY(ChildJointLocalTransform);

            //Inertial is the transform from the joint origin to the center of mass
            Link.Inertial.Origin.XYZ = OPS.getXYZ(localLinkCoMTransform);
            Link.Inertial.Origin.RPY = new double[] { 0, 0, 0 };

            Link.Inertial.Inertia.Moment = linkLocalMomentInertia.ToRowWiseArray();
        }

        public void localizeJoint2(link Link, Matrix<double> ParentJointGlobalTransform)
        {
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(Link.Joint.CoordinateSystemName);
            //Transform from global origin to child joint
            Matrix<double> ChildJointGlobalTransform = OPS.getTransformation(coordsysTransform);
            Matrix<double> ChildJointOrigin = ParentJointGlobalTransform.Inverse() * ChildJointGlobalTransform;

            Vector<double> Axis = new DenseVector(new double[] { Link.Joint.Axis.X, Link.Joint.Axis.Y, Link.Joint.Axis.Z, 0 });
            Axis = ChildJointGlobalTransform.Inverse() * Axis;
            Axis = Axis.Normalize(2);

            //Save the data from the transforms
            Link.Joint.Axis.XYZ = new double[] { Axis[0], Axis[1], Axis[2] };

            Link.Joint.Origin.XYZ = OPS.getXYZ(ChildJointOrigin);
            Link.Joint.Origin.RPY = OPS.getRPY(ChildJointOrigin);
        }

        public void localizeLink(link Link, Matrix<double> GlobalTransform)
        {
            Matrix<double> GlobalTransformInverse = GlobalTransform.Inverse();
            Matrix<double> linkCoMTransform = OPS.getTranslation(Link.Inertial.Origin.XYZ);
            Matrix<double> localLinkCoMTransform = GlobalTransformInverse * linkCoMTransform;

            Matrix<double> linkVisualTransform = OPS.getTransformation(Link.Visual.Origin.XYZ, Link.Visual.Origin.RPY);
            Matrix<double> localVisualTransform = GlobalTransformInverse * linkVisualTransform;

            Matrix<double> linkCollisionTransform = OPS.getTransformation(Link.Collision.Origin.XYZ, Link.Collision.Origin.RPY);
            Matrix<double> localCollisionTransform = GlobalTransformInverse * linkCollisionTransform;

            // The linear array in Link.Inertial.Inertia.Moment is in row major order, but this matrix constructor uses column major order
            // It's a rotation matrix, so this shouldn't matter. If it does, just transpose linkGlobalMomentInertia
            // These three matrices are 3x3 as opposed to the 4x4 transformation matrices above. You're welcome for the confusion.
            Matrix<double> linkGlobalMomentInertia = new DenseMatrix(3, 3, Link.Inertial.Inertia.Moment);
            Matrix<double> GlobalRotMat = GlobalTransform.SubMatrix(0, 3, 0, 3);
            Matrix<double> linkLocalMomentInertia = GlobalRotMat.Inverse() * linkGlobalMomentInertia;

            Link.Inertial.Origin.XYZ = OPS.getXYZ(localLinkCoMTransform);
            Link.Inertial.Origin.RPY = new double[] { 0, 0, 0 };

            // Wait are you saying that even though the matrix was trasposed from column major order, you are writing it in row-major order here.
            // Yes, yes I am.
            Link.Inertial.Inertia.Moment = linkLocalMomentInertia.ToRowWiseArray();


            Link.Collision.Origin.XYZ = OPS.getXYZ(localCollisionTransform);
            Link.Collision.Origin.RPY = OPS.getRPY(localCollisionTransform);

            Link.Visual.Origin.XYZ = OPS.getXYZ(localVisualTransform);
            Link.Visual.Origin.RPY = OPS.getRPY(localVisualTransform);
        }

        public Feature insertAxis(SketchSegment axis)
        {
            SelectData data = ActiveSWModel.SelectionManager.CreateSelectData();
            axis.Select4(false, data);

            object[] featuresBefore, featuresAfter;
            featuresBefore = ActiveSWModel.FeatureManager.GetFeatures(true);
            int countBefore = ActiveSWModel.FeatureManager.GetFeatureCount(true);
            ActiveSWModel.InsertAxis2(true);
            featuresAfter = ActiveSWModel.FeatureManager.GetFeatures(true);
            int countAfter = ActiveSWModel.FeatureManager.GetFeatureCount(true);

            if (featuresBefore.Length < featuresAfter.Length)
            {
                foreach (Feature feat in featuresAfter.Reverse())
                {
                    if (!featuresBefore.Contains(feat))
                    {
                        return feat;
                    }
                }

            }

            return null;
        }

        // Inserts a sketch into the main assembly
        public string setup3DSketch()
        {
            bool sketchExists = ActiveSWModel.Extension.SelectByID2("URDF_reference", "SKETCH", 0, 0, 0, false, 0, null, 0);
            ActiveSWModel.SketchManager.Insert3DSketch(true);
            ActiveSWModel.SketchManager.CreatePoint(0, 0, 0);
            IFeature sketch = (IFeature)ActiveSWModel.SketchManager.ActiveSketch;
            ActiveSWModel.SketchManager.Insert3DSketch(true);
            if (!sketchExists)
            {
                sketch.Name = "URDF_reference";
            }
            return sketch.Name;
        }

        // Adds lines and a point to create the entities for a reference coordinates
        public object[] addSketchGeometry(origin Origin)
        {
            if (ActiveSWModel.SketchManager.ActiveSketch == null)
            {
                bool sketchExists = ActiveSWModel.Extension.SelectByID2(referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
                ActiveSWModel.SketchManager.Insert3DSketch(true);
            }
            IFeature sketch = (IFeature)ActiveSWModel.SketchManager.ActiveSketch;
            Matrix<double> transform = OPS.getRotation(Origin.RPY);
            Matrix<double> Axes = 0.01 * DenseMatrix.Identity(4);
            Matrix<double> tA = transform * Axes;

            SketchPoint OriginPoint = ActiveSWModel.SketchManager.CreatePoint(Origin.X,
                                                                      Origin.Y,
                                                                      Origin.Z);

            SketchSegment XAxis = ActiveSWModel.SketchManager.CreateLine(Origin.X,
                                                                         Origin.Y,
                                                                         Origin.Z,
                                                                         Origin.X + tA[0, 0],
                                                                         Origin.Y + tA[1, 0],
                                                                         Origin.Z + tA[2, 0]);
            XAxis.ConstructionGeometry = true;
            SketchSegment YAxis = ActiveSWModel.SketchManager.CreateLine(Origin.X,
                                                                         Origin.Y,
                                                                         Origin.Z,
                                                                         Origin.X + tA[0, 1],
                                                                         Origin.Y + tA[1, 1],
                                                                         Origin.Z + tA[2, 1]);
            YAxis.ConstructionGeometry = true;

            if (ActiveSWModel.SketchManager.ActiveSketch != null)
            {
                ActiveSWModel.SketchManager.Insert3DSketch(true);

            }
            sketch.Name = "URDF_reference";
            return new object[] { OriginPoint, XAxis, YAxis, Origin.X, Origin.Y, Origin.Z, Origin.X + tA[0, 0], Origin.Y + tA[1, 0], Origin.Z + tA[2, 0], Origin.X + tA[0, 1], Origin.Y + tA[1, 1], Origin.Z + tA[2, 1] };
        }

        public SketchSegment addSketchGeometry(axis Axis, origin Origin)
        {
            bool sketchExists = ActiveSWModel.Extension.SelectByID2(referenceSketchName, "SKETCH", 0, 0, 0, false, 0, null, 0);
            ActiveSWModel.SketchManager.Insert3DSketch(true);
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

            ActiveSWModel.SketchManager.Insert3DSketch(true);
            return RotAxis;
        }

        public void configureDisplayState(ListBox list)
        {

            Configuration config = ActiveSWModel.ConfigurationManager.ActiveConfiguration;
            bool createNew = true;
            foreach (string state in config.GetDisplayStates())
            {
                if (state.Equals("URDF Export"))
                {
                    config.ApplyDisplayState("URDF Export");
                    createNew = false;
                }
            }
            if (createNew)
            {
                config.CreateDisplayState("URDF Export");
            }

            hideComponents(list);
        }

        // This estimates the origin and the axes given two components in an assembly. The geometries are all in reference to
        // the parent assembly
        public joint estimateGlobalJointFromComponents(AssemblyDoc assy, link parent, link child)
        {
            joint Joint = child.Joint;

            int R1Status, R2Status, L1Status, L2Status;
            int R1DirStatus, R2DirStatus;
            int DOFs;
            MathPoint RPoint1, RPoint2;
            MathVector RDir1, RDir2;
            MathVector LDir1, LDir2;
            // Surpress Limit Mates to properly find degrees of freedom
            List<Mate2> limitMates = new List<Mate2>();
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
                Joint.type = "fixed";
                Joint.Origin.XYZ = OPS.getXYZ(child.SWMainComponent.Transform2);
                Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);
                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    if (R1Status == 1)
                    {
                        Joint.type = "continuous";
                        Joint.Axis.XYZ = RDir1.ArrayData;
                        Joint.Origin.XYZ = RPoint1.ArrayData;
                        Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);

                    }
                    else if (L1Status == 1)
                    {
                        Joint.type = "prismatic";
                        Joint.Axis.XYZ = LDir1.ArrayData;
                        Joint.Origin.XYZ = OPS.getXYZ(child.SWMainComponent.Transform2);
                        Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);
                    }
                }
                unsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    Joint = addLimits(Joint, limitMates);
                }
                return Joint;
            }
            else
            {

                limitMates = suppressLimitMates(child.SWComponent);
                // The wonderful undocumented API call I found to get the degrees of freedom in a joint. 
                // https://forum.solidworks.com/thread/57414
                int remainingDOFs = child.SWComponent.GetRemainingDOFs(out R1Status, out RPoint1, out R1DirStatus, out RDir1,
                                                  out R2Status, out RPoint2, out R2DirStatus, out RDir2,
                                                  out L1Status, out LDir1,
                                                  out L2Status, out LDir2);
                DOFs = remainingDOFs;


                // Convert the gotten degrees of freedom to a joint type, origin and axis
                Joint.type = "fixed";
                Joint.Origin.XYZ = OPS.getXYZ(child.SWComponent.Transform2);
                Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);
                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    if (R1Status == 1)
                    {
                        Joint.type = "continuous";
                        Joint.Axis.XYZ = RDir1.ArrayData;
                        Joint.Origin.XYZ = RPoint1.ArrayData;
                        Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);

                    }
                    else if (L1Status == 1)
                    {
                        Joint.type = "prismatic";
                        Joint.Axis.XYZ = LDir1.ArrayData;
                        Joint.Origin.XYZ = OPS.getXYZ(child.SWComponent.Transform2);
                        Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);
                    }
                }
                unsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    Joint = addLimits(Joint, limitMates);
                }
                return Joint;
            }

        }

        public void estimateGlobalJointFromComponents2(AssemblyDoc assy, link parent, link child)
        {
            int R1Status, R2Status, L1Status, L2Status;
            int R1DirStatus, R2DirStatus;
            int DOFs;
            MathPoint RPoint1, RPoint2;
            MathVector RDir1, RDir2;
            MathVector LDir1, LDir2;
            // Surpress Limit Mates to properly find degrees of freedom
            List<Mate2> limitMates = new List<Mate2>();
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
                child.Joint.Origin.XYZ = OPS.getXYZ(child.SWMainComponent.Transform2);
                child.Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);
                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    if (R1Status == 1)
                    {
                        child.Joint.type = "continuous";
                        child.Joint.Axis.XYZ = RDir1.ArrayData;
                        child.Joint.Origin.XYZ = RPoint1.ArrayData;
                        child.Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);

                    }
                    else if (L1Status == 1)
                    {
                        child.Joint.type = "prismatic";
                        child.Joint.Axis.XYZ = LDir1.ArrayData;
                        child.Joint.Origin.XYZ = OPS.getXYZ(child.SWMainComponent.Transform2);
                        child.Joint.Origin.RPY = OPS.getRPY(child.SWMainComponent.Transform2);
                    }
                }
                unsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    addLimits2(child.Joint, limitMates);
                }
            }
            else
            {

                limitMates = suppressLimitMates(child.SWComponent);
                // The wonderful undocumented API call I found to get the degrees of freedom in a joint. 
                // https://forum.solidworks.com/thread/57414
                int remainingDOFs = child.SWComponent.GetRemainingDOFs(out R1Status, out RPoint1, out R1DirStatus, out RDir1,
                                                  out R2Status, out RPoint2, out R2DirStatus, out RDir2,
                                                  out L1Status, out LDir1,
                                                  out L2Status, out LDir2);
                DOFs = remainingDOFs;


                // Convert the gotten degrees of freedom to a joint type, origin and axis
                child.Joint.type = "fixed";
                child.Joint.Origin.XYZ = OPS.getXYZ(child.SWComponent.Transform2);
                child.Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);
                if (DOFs == 0 && (R1Status + L1Status > 0))
                {
                    if (R1Status == 1)
                    {
                        child.Joint.type = "continuous";
                        child.Joint.Axis.XYZ = RDir1.ArrayData;
                        child.Joint.Origin.XYZ = RPoint1.ArrayData;
                        child.Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);

                    }
                    else if (L1Status == 1)
                    {
                        child.Joint.type = "prismatic";
                        child.Joint.Axis.XYZ = LDir1.ArrayData;
                        child.Joint.Origin.XYZ = OPS.getXYZ(child.SWComponent.Transform2);
                        child.Joint.Origin.RPY = OPS.getRPY(child.SWComponent.Transform2);
                    }
                }
                unsuppressLimitMates(limitMates);
                if (limitMates.Count > 0)
                {
                    addLimits(child.Joint, limitMates);
                }
            }

        }
        public joint estimateJointFromRefGeometry(ModelDoc2 model, link Link)
        {
            joint Joint = Link.Joint;

            MathTransform coordsysTransform = model.Extension.GetCoordinateSystemTransformByName(Joint.CoordinateSystemName);
            Joint.Origin.XYZ = OPS.getXYZ(coordsysTransform);
            Joint.Origin.RPY = OPS.getRPY(coordsysTransform);

            Joint.Axis.XYZ = estimateAxis(Joint.AxisName);

            return Joint;
        }

        public double[] estimateAxis(string axisName)
        {
            double[] XYZ = new double[3]; ;

            ActiveSWModel.ClearSelection2(true);
            bool selected = ActiveSWModel.Extension.SelectByID2(axisName, "AXIS", 0, 0, 0, false, 0, null, 0);
            if (selected)
            {
                Feature feat = ActiveSWModel.SelectionManager.GetSelectedObject6(1, 0);
                RefAxis axis = (RefAxis)feat.GetSpecificFeature2();
                double[] axisParams;
                axisParams = axis.GetRefAxisParams();
                XYZ[0] = axisParams[0] - axisParams[3];
                XYZ[1] = axisParams[1] - axisParams[4];
                XYZ[2] = axisParams[2] - axisParams[5];
                XYZ = OPS.pnorm(XYZ, 2);
            }

            return XYZ;
        }


        public double[] localizeAxis(double[] Axis, double[] XYZ, double[] RPY)
        {
            Matrix<double> transformation = OPS.getTransformation(XYZ, RPY);
            Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
            vec = transformation.Inverse() * vec;
            return vec.ToArray();
        }

        public double[] localizeAxis(double[] Axis, string coordsys)
        {
            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(coordsys);
            Vector<double> vec = new DenseVector(new double[] { Axis[0], Axis[1], Axis[2], 0 });
            Matrix<double> transform = OPS.getTransformation(coordsysTransform);
            vec = transform.Inverse() * vec;
            return vec.ToArray();
        }

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
        public joint addLimits(joint Joint, List<Mate2> limitMates)
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
            return Joint;
        }
        public void addLimits2(joint Joint, List<Mate2> limitMates)
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

        public List<Mate2> suppressLimitMates(IComponent2 component)
        {
            ModelDoc2 modelDoc = component.GetModelDoc2();
            List<Mate2> limitMates = new List<Mate2>();
            if (modelDoc.GetType() == (int)swDocumentTypes_e.swDocPART)
            {
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
            }
            return limitMates;
        }

        public void unsuppressLimitMates(List<Mate2> limitMates)
        {
            foreach (Mate2 swMate in limitMates)
            {
                Feature feat = (Feature)swMate;
                feat.SetSuppression2((int)swFeatureSuppressionAction_e.swUnSuppressFeature, (int)swInConfigurationOpts_e.swThisConfiguration, null);
            }
        }

        public IComponent2 findCommonSWAncestor(link Link1, link Link2)
        {
            int levelChange = Math.Abs(Link1.SWComponentLevel - Link2.SWComponentLevel + 1);
            IComponent2 parent1 = Link1.SWComponent;
            IComponent2 parent2 = Link2.SWComponent;
            for (int i = 0; i < Link1.SWComponentLevel - levelChange; i++)
            {
                parent1 = parent1.GetParent();
                if (parent1 == null)
                {
                    return null;
                }
            }
            for (int i = 0; i < Link2.SWComponentLevel - levelChange; i++)
            {
                parent2 = parent2.GetParent();
                if (parent2 == null)
                {
                    return null;
                }
            }
            while (parent1 != parent2)
            {
                parent1 = parent1.GetParent();
                parent2 = parent2.GetParent();
                if (parent1 == null || parent2 == null)
                {
                    return null;
                }
            }
            return parent1;

        }
        public IComponent2 findCompToFix(link parent, link child)
        {
            IComponent2 compToFix = parent.SWComponent;
            for (int i = 0; i < parent.SWComponentLevel; i++)
            {
                compToFix = compToFix.GetParent();
            }
            return compToFix;
        }
        public void unFixComponents(AssemblyDoc assy, IComponent2 comp, bool isFixed)
        {
            // Unfix components (if they weren't fixed beforehand)
            comp.Select(false);
            if (!isFixed)
            {
                assy.UnfixComponent();
            }
        }
        public void unFixComponents(List<IComponent2> components)
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
            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.BaseLink.name + ".URDF";
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
            List<IComponent2> hiddenComponents = findHiddenComponents(assyDoc.GetComponents(false));
            ActiveSWModel.Extension.SelectAll();
            ActiveSWModel.HideComponent2();
            string filename = exportFiles(mRobot.BaseLink, package);
            showComponents(mRobot.BaseLink);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = filename;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = filename;
            showAllComponents(hiddenComponents);
            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
        }

        public string exportFiles(link Link, URDFPackage package)
        {
            // Iterate through each child and export its files
            foreach (link child in Link.Children)
            {
                string filename = exportFiles(child, package);
                child.Visual.Geometry.Mesh.filename = filename;
                child.Collision.Geometry.Mesh.filename = filename;
            }

            // Copy the texture file (if it was specified) to the textures directory
            if (Link.Visual.Material.Texture.wFilename != "")
            {
                if (!System.IO.File.Exists(Link.Visual.Material.Texture.wFilename))
                {
                    Link.Visual.Material.Texture.filename = package.TexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    string textureSavePath = package.WindowsTexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    System.IO.File.Copy(Link.Visual.Material.Texture.wFilename, textureSavePath, true);
                }
            }

            // Create the mesh filenames. SolidWorks likes to use / but that will get messy in filenames so use _ instead
            string linkName = Link.uniqueName.Replace('/', '_');
            string meshFileName = package.MeshesDirectory + linkName + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + linkName + ".STL";

            int errors = 0;
            int warnings = 0;

            // Export STL
            showComponents(Link);

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
            hideComponents(Link);

            correctSTLMesh(windowsMeshFileName);

            return meshFileName;
        }

        public void exportLink(bool zIsUp)
        {

            createBaseRefOrigin(zIsUp);
            MathTransform coordSysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
            Matrix<double> GlobalTransform = OPS.getTransformation(coordSysTransform);

            localizeLink(mRobot.BaseLink, GlobalTransform);

            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
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
        public void selectComponents(List<IComponent2> components, bool clearSelection, int mark = -1)
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


        public List<IComponent2> findHiddenComponents(object[] varComp)
        {
            List<IComponent2> hiddenComp = new List<IComponent2>();
            foreach (object obj in varComp)
            {
                IComponent2 comp = (IComponent2)obj;
                if (comp.IsHidden(false))
                {
                    hiddenComp.Add(comp);
                }
            }
            return hiddenComp;
        }
        public void showAllComponents(List<IComponent2> hiddenComponents)
        {
            AssemblyDoc assyDoc = (AssemblyDoc)ActiveSWModel;
            ActiveSWModel.Extension.SelectAll();
            foreach (IComponent2 comp in hiddenComponents)
            {
                comp.DeSelect();
            }
            ActiveSWModel.ShowComponent2();
        }
        public void showComponents(List<IComponent2> components)
        {
            selectComponents(components, true);
            ActiveSWModel.ShowComponent2();
        }
        public void showComponents(link Link)
        {
            selectComponents(Link, true);
            ActiveSWModel.ShowComponent2();

        }
        public void hideComponents(link Link)
        {
            selectComponents(Link, true);
            ActiveSWModel.HideComponent2();
        }
        public void hideComponents(ListBox list)
        {
            ActiveSWModel.ClearSelection2(true);

            foreach (LinkItem item in list.Items)
            {
                selectComponents(item.Link, true);
            }
            ActiveSWModel.HideComponent2();
        }
        public void hideComponents(List<IComponent2> components)
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
        public void loadExporter(ISldWorks iSldWorksApp)
        {
            constructExporter(iSldWorksApp);
            loadSWComponents(mRobot.BaseLink);
        }

        public void loadSWComponents(link Link)
        {
            int Errors = 0;
            if (Link.SWMainComponentPID != null)
            {
                Link.SWMainComponent = (Component2)ActiveSWModel.Extension.GetObjectByPersistReference3(Link.SWMainComponentPID, out Errors);
            }
            if (Link.SWComponentPIDs != null)
            {
                Link.SWcomponents = new List<IComponent2>();
                foreach (Object PID in Link.SWComponentPIDs)
                {
                    IComponent2 comp = (IComponent2)ActiveSWModel.Extension.GetObjectByPersistReference3(PID, out Errors);
                    Link.SWcomponents.Add(comp);
                }
            }
            foreach (link Child in Link.Children)
            {
                loadSWComponents(Child);
            }
        }

        public void saveExporter()
        {
            saveSWComponents(mRobot.BaseLink);
        }

        public void saveSWComponents(link Link)
        {
            ActiveSWModel.ClearSelection2(true);
            SelectionMgr manager = ActiveSWModel.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = 1;
            if (Link.SWMainComponent != null)
            {
                Link.SWMainComponentPID = ActiveSWModel.Extension.GetPersistReference3(Link.SWMainComponent);
            }
            if (Link.SWcomponents != null)
            {
                Link.SWComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in Link.SWcomponents)
                {
                    byte[] PID = ActiveSWModel.Extension.GetPersistReference3(comp);
                    Link.SWComponentPIDs.Add(PID);
                }
            }
            foreach (link Child in Link.Children)
            {
                saveSWComponents(Child);
            }
        }
        public void createBaseLinkFromComponents(List<Component2> components, string linkName)
        {
            // Build the link from the partdoc
            link Link = createLinkFromComponents(null, components, linkName, "");
            createBaseRefOrigin(true);

            mRobot.BaseLink = Link;
        }

        public link createLinkFromComponents(link parent, List<Component2> components, string linkName, string jointName)
        {
            link child = new link();
            child.name = linkName;
            child.uniqueName = linkName;

            child.SWMainComponent = components[0];
            child.SWcomponents.AddRange(components);
            //Get link properties from SolidWorks part

            if (parent != null)
            {
                createJoint(parent, child, jointName);
            }

            ActiveSWModel.ClearSelection2(true);
            foreach (Component2 comp in components)
            {
                comp.Select(true);
            }
            IMassProperty swMass = ActiveSWModel.Extension.CreateMassProperty();

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
            swMass.SetCoordinateSystem(jointTransform);
            child.Inertial.Mass.Value = swMass.Mass;
            
            child.Inertial.Inertia.Moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]

            double[] centerOfMass = swMass.CenterOfMass;
            child.Inertial.Origin.XYZ = centerOfMass;
            child.Inertial.Origin.RPY = new double[3] { 0, 0, 0 };

            createRefOrigin(child.Inertial.Origin, "CoM_" + child.name);

            // Will this ever not be zeros?
            child.Visual.Origin.XYZ = new double[3] { 0, 0, 0 };
            child.Visual.Origin.RPY = new double[3] { 0, 0, 0 };
            child.Collision.Origin.XYZ = new double[3] { 0, 0, 0 };
            child.Collision.Origin.RPY = new double[3] { 0, 0, 0 };

            // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
            double[] values = ActiveSWModel.MaterialPropertyValues;
            child.Visual.Material.Color.Red = values[0];
            child.Visual.Material.Color.Green = values[1];
            child.Visual.Material.Color.Blue = values[2];
            child.Visual.Material.Color.Alpha = 1.0 - values[7];
            child.Visual.Material.name = "material_" + child.name;

            //The part model doesn't actually know where the origin is, but the component does and this is important when exporting from assembly
            child.Visual.Origin.XYZ = new double[] { 0, 0, 0 };
            child.Visual.Origin.RPY = new double[] { 0, 0, 0 };
            child.Collision.Origin.XYZ = new double[] { 0, 0, 0 };
            child.Collision.Origin.RPY = new double[] { 0, 0, 0 };

            return child;
        }

        public void createJoint(link parent, link child, string jointName)
        {
            List<IComponent2> componentsToFix = fixComponents(parent);

            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            child.Joint = new joint();
            createJointName2(parent, child, jointName);
            estimateGlobalJointFromComponents2(assy, parent, child);
            if (!ActiveSWModel.Extension.SelectByID2(child.Joint.CoordinateSystemName, "COORDSYS", 0, 0, 0, false, 0, null, 0) &&
                !ActiveSWModel.Extension.SelectByID2(child.Joint.AxisName, "COORDSYS", 0, 0, 0, false, 0, null, 0))
            {
                createRefGeometry(child.Joint);
            }

            MathTransform coordsysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName(child.Joint.CoordinateSystemName);
            child.Joint.Origin.XYZ = OPS.getXYZ(coordsysTransform);
            child.Joint.Origin.RPY = OPS.getRPY(coordsysTransform);
            child.Joint.Axis.XYZ = estimateAxis(child.Joint.AxisName);

            Matrix<double> ParentJointGlobalTransform;
            string coordSysName = (parent.Joint == null) ? "Origin_global" : parent.Joint.CoordinateSystemName;

            //If the parent joint exists, it becomes the reference joint, otherwise use the Origin_global. Grab the MathTransform of that coordsys to use for localizing
            MathTransform coordSysTransform = ActiveSWModel.Extension.GetCoordinateSystemTransformByName("Origin_global");
            ParentJointGlobalTransform = OPS.getTransformation(coordSysTransform);

            unFixComponents(componentsToFix);
            localizeJoint2(child, ParentJointGlobalTransform);
        }

        private List<IComponent2> fixComponents(link parent)
        {
            List<IComponent2> componentsToUnfix = new List<IComponent2>();
            foreach (Component2 comp in parent.SWcomponents)
            {
                if (!comp.IsFixed())
                {
                    componentsToUnfix.Add(comp);
                }
            }
            selectComponents(parent, true);
            AssemblyDoc assy = (AssemblyDoc)ActiveSWModel;
            assy.FixComponent();
            return componentsToUnfix;
        }


        #endregion


    }
}