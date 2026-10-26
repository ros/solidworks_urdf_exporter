using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using System.Collections.Generic;
using System.Linq;
using PrivateObject = Microsoft.VisualStudio.TestTools.UnitTesting.PrivateObject;
using SW2URDF.Utilities;
using Xunit;

namespace SW2URDF.Test
{
    public class TestComponentMassProperties
    {
        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void ThreeDxmlExportPreservesInertialAndCorrectsMeshPlacement(bool subassembly)
        {
            var helper = CreateHelper();
            var app = new Moq.Mock<ISldWorks>();
            var document = new Moq.Mock<ModelDoc2>();
            var extension = new Moq.Mock<ModelDocExtension>();
            var componentDocument = new Moq.Mock<ModelDoc2>();
            var componentExtension = new Moq.Mock<ModelDocExtension>();
            var component = new Moq.Mock<Component2>();
            var transform = new Moq.Mock<MathTransform>();
            var selection = new Moq.Mock<SelectionMgr>();
            selection.Setup(s => s.CreateSelectData()).Returns(new Moq.Mock<SelectData>().Object);
            document.SetupGet(d => d.SelectionManager).Returns(selection.Object);
            // A translated frame rotated 90 degrees about Z exposes both double translation
            // and unwanted rotation/sign changes of the off-diagonal inertia entries.
            transform.SetupGet(t => t.ArrayData).Returns(new double[] {
                0, 1, 0, -1, 0, 0, 0, 0, 1, 0.1, -0.2, 0.3, 1, 0, 0, 0 });
            document.SetupGet(d => d.Extension).Returns(extension.Object);
            componentDocument.SetupGet(d => d.Extension).Returns(componentExtension.Object);
            component.Setup(c => c.GetModelDoc2()).Returns(componentDocument.Object);
            var coordinateExtension = subassembly ? componentExtension : extension;
            coordinateExtension.Setup(e => e.GetCoordinateSystemTransformByName("frame")).Returns(transform.Object);
            helper.iSwApp = app.Object;
            helper.ActiveSWModel = document.Object;
            var link = new Link { Name = "link", SWMainComponent = component.Object };
            link.Joint.CoordinateSystemName = subassembly ? "frame <component-1>" : "frame";
            link.Inertial.Mass.Value = 2.5;
            link.Inertial.Origin.SetXYZ(new double[] { 0.01, 0.02, -0.03 });
            link.Inertial.Origin.SetRPY(new double[] { 0.1, 0.2, 0.3 });
            link.Inertial.Inertia.SetMomentMatrix(new double[] { 4, 0.1, 0.2, 0.1, 5, 0.3, 0.2, 0.3, 6 });
            double[] moment = link.Inertial.Inertia.GetMoment();
            link.Visual.Origin.SetXYZ(new double[] { 0.4, 0.5, 0.6 });
            link.Collision.Origin.SetXYZ(new double[] { 0.2, 0.3, 0.4 });

            new PrivateObject(helper).Invoke("Save3dxml", link, "unused.3dxml");

            Assert.Equal(2.5, link.Inertial.Mass.Value);
            Assert.Equal(new double[] { 0.01, 0.02, -0.03 }, link.Inertial.Origin.GetXYZ());
            Assert.Equal(new double[] { 0.1, 0.2, 0.3 }, link.Inertial.Origin.GetRPY());
            Assert.Equal(moment, link.Inertial.Inertia.GetMoment());
            double[] visual = { 0.7, -0.3, 0.3 };
            double[] collision = { 0.5, -0.1, 0.1 };
            for (int i = 0; i < 3; i++)
            {
                Assert.Equal(visual[i], link.Visual.Origin.GetXYZ()[i], 12);
                Assert.Equal(collision[i], link.Collision.Origin.GetXYZ()[i], 12);
            }
            Assert.Equal(-System.Math.PI / 2, link.Visual.Origin.Yaw, 12);
            Assert.Equal(-System.Math.PI / 2, link.Collision.Origin.Yaw, 12);
            coordinateExtension.Verify(e => e.GetCoordinateSystemTransformByName("frame"), Moq.Times.Once());
        }

        [Fact]
        public void PartLocalizationStillTransformsCenterOfMass()
        {
            var link = new Link();
            link.Inertial.Origin.SetXYZ(new double[] { 1, 2, 3 });
            var transform = MathOps.GetTranslation(new double[] { 0.1, 0.2, 0.3 });
            var method = typeof(ExportHelper).GetMethod("LocalizeLink",
                System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);
            method.Invoke(null, new object[] { link, transform });
            double[] expected = { 0.9, 1.8, 2.7 };
            for (int i = 0; i < 3; i++)
                Assert.Equal(expected[i], link.Inertial.Origin.GetXYZ()[i], 12);
        }

        private static ExportHelper CreateHelper()
        {
            return (ExportHelper)System.Runtime.Serialization.FormatterServices
                .GetUninitializedObject(typeof(ExportHelper));
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void EmptyLinkHasZeroMassWithoutQueryingSolidWorks(bool nullComponents)
        {
            var helper = CreateHelper();
            var link = new Link();
            if (nullComponents) link.SWComponents = null;
            link.Inertial.Mass.Value = 12;
            link.Inertial.Origin.X = 3;
            link.Inertial.Origin.Roll = 1;
            link.Inertial.Inertia.SetMomentMatrix(Enumerable.Repeat(7.0, 9).ToArray());
            new PrivateObject(helper).Invoke("ComputeInertialProperties", link);
            Assert.Equal(0, link.Inertial.Mass.Value);
            Assert.Equal(new double[9], link.Inertial.Inertia.GetMoment());
            Assert.Equal(new double[3], link.Inertial.Origin.GetXYZ());
            Assert.Equal(new double[3], link.Inertial.Origin.GetRPY());
        }

        [Fact]
        public void OldSolidWorksDoesNotCallUnavailableApi()
        {
            var helper = CreateHelper();
            var app = new Moq.Mock<ISldWorks>();
            app.Setup(sw => sw.RevisionNumber()).Returns("27.5.0");
            helper.iSwApp = app.Object;
            // ActiveSWModel is deliberately null: accessing the new API would fail.
            Assert.Null(new PrivateObject(helper).Invoke("GetComponentsMassProperty",
                new List<Component2>(), null));
        }
    }

    [Collection("Requires SW Test Collection")]
    public class TestExportHelper : SW2URDFTest
    {
        public TestExportHelper(SWTestFixture fixture) : base(fixture)
        {
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4, MeshExportFormat.STL)]
        [InlineData("4_WHEELER", 5, MeshExportFormat.STL)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4, MeshExportFormat.STL)]
        [InlineData("3_DOF_ARM", 4, MeshExportFormat.THREEDXML)]
        [InlineData("4_WHEELER", 5, MeshExportFormat.THREEDXML)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4, MeshExportFormat.THREEDXML)]
        public void TestExportRobot(string modelName, int expNumLinks, MeshExportFormat meshExportFormat)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true, meshExportFormat);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobotNoSTL(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(false);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobotSkipInertial(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(false);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobotSkipVisual(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(false);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobotSkipKinematics(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(false);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobotSkipLimits(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(false);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, CommonSwOperations.GetCount(helper.URDFRobot.BaseLink));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 3)]
        [InlineData("4_WHEELER", 4)]
        [InlineData("ORIGINAL_3_DOF_ARM", 3)]
        public void TestGetJointNames(string modelName, int expNumJoints)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            List<string> jointNames = helper.GetJointNames();
            Assert.NotNull(jointNames);
            Assert.Equal(jointNames.Count, expNumJoints);
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        /*
         * TODO(SIMINT-164) Part document tests not working (OpenSWPartDocument)
        [Theory]
        [InlineData("TOY_BLOCK")]
        public void TestExportLink(string modelName)
        {
            ModelDoc2 doc = OpenSWPartDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.ExportLink(true);
            Assert.True(true, "Part export failed");
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("TOY_BLOCK")]
        public void TestCreateRobotFromActiveModel(string modelName)
        {
            ModelDoc2 doc = OpenSWPartDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.CreateRobotFromActiveModel();
            Assert.NotNull(helper.URDFRobot);
            Assert.True(SwApp.CloseAllDocuments(true));
        }
        */

        [Theory]
        [InlineData("3_DOF_ARM")]
        public void TestCreateRobotFromTreeView(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);

            helper.CreateRobotFromTreeView(baseNode);
            Assert.NotNull(helper.URDFRobot);
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", new double[] { 0, 0, 1 }, "global_origin", new double[] { 0, 0, 1 })]
        public void TestLocalizeAxis(string modelName, double[] axis, string coordSys, double[] expected)
        {
            OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(expected, helper.LocalizeAxis(axis, coordSys));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", new string[] {
            "Origin_global",
            "Origin_prox_joint",
            "Origin_dist_joint",
            "Origin_effector_joint" })]
        public void TestGetRefCoordinateSystems(string modelName, string[] expected)
        {
            OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(new List<string>(expected), helper.GetRefCoordinateSystems());
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", new string[] {
            "Axis_prox_joint",
            "Axis_dist_joint",
            "Axis_effector_joint" })]
        public void TestGetRefAxes(string modelName, string[] expected)
        {
            OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(new List<string>(expected), helper.GetRefAxes());
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        /// <summary>
        /// A link's exported mass and moment of inertia must match what SolidWorks itself reports
        /// for that link's components, which is what a user compares the export against.
        /// Computing these from the components' bodies instead silently drops mass properties the
        /// user has overridden, and misplaces bodies that belong to a subassembly.
        ///
        /// Both quantities asserted here are independent of the coordinate system: mass trivially
        /// so, and the trace of the inertia tensor about the center of mass because it is
        /// invariant under rotation. That keeps the test from having to resolve the link's
        /// coordinate system, which for a subassembly lives in a subcomponent.
        /// </summary>
        [Theory]
        [InlineData("3_DOF_ARM")]
        [InlineData("4_WHEELER")]
        [InlineData("ORIGINAL_3_DOF_ARM")]
        public void TestInertialMatchesSolidWorks(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
            Assert.False(error);

            // The saved configuration stores components as persistent IDs. The export UI resolves
            // them before building the robot, so do the same here.
            List<string> problemLinks = new List<string>();
            CommonSwOperations.LoadSWComponents(doc, baseNode, problemLinks);
            Assert.Empty(problemLinks);

            Assert.True(helper.CreateRobotFromTreeView(baseNode));

            double totalMass = doc.Extension.CreateMassProperty().Mass;
            int linksChecked = 0;

            foreach (Link link in Flatten(helper.URDFRobot.BaseLink))
            {
                if (link.SWComponents.Count == 0)
                {
                    continue;
                }

                IMassProperty2 expected = (IMassProperty2)doc.Extension.CreateMassProperty2();
                Assert.NotNull(expected);
                expected.UseSystemUnits = true;
                expected.IncludeHiddenBodiesOrComponents = true;
                expected.SelectedItems = link.SWComponents.ToArray();
                Assert.True(expected.Recalculate());

                Assert.Equal(expected.Mass, link.Inertial.Mass.Value, 9);

                double[] expectedMoment = (double[])expected.GetMomentOfInertia(
                    (int)swMomentsOfInertiaReferenceFrame_e.swMomentsOfInertiaReferenceFrame_CenterOfMass);
                double expectedTrace = expectedMoment[0] + expectedMoment[4] + expectedMoment[8];
                double actualTrace = link.Inertial.Inertia.Ixx +
                    link.Inertial.Inertia.Iyy + link.Inertial.Inertia.Izz;
                Assert.Equal(expectedTrace, actualTrace, 12);

                Assert.True(link.Inertial.Mass.Value > 0.0, "link " + link.Name + " has no mass");

                // An ignored or empty selection degrades into the whole document's properties.
                // No single link of these robots is the entire assembly.
                Assert.True(link.Inertial.Mass.Value < totalMass,
                    "link " + link.Name + " has the mass of the whole assembly");

                linksChecked++;
            }

            Assert.True(linksChecked > 0, "no links were checked for " + modelName);
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM")]
        [InlineData("ORIGINAL_3_DOF_ARM")]
        public void TestInertialHonorsMassOverride(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            try
            {
                var assembly = (AssemblyDoc)doc;
                assembly.ResolveAllLightWeightComponents(true);
                Component2 component = ((object[])assembly.GetComponents(false))
                    .Cast<Component2>().OrderBy(comp => comp.Name2).First(comp =>
                        ((ModelDoc2)comp.GetModelDoc2()).GetType() == (int)swDocumentTypes_e.swDocPART);
                var part = (ModelDoc2)component.GetModelDoc2();
                if (part.ConfigurationManager.ActiveConfiguration.Name != component.ReferencedConfiguration)
                {
                    Assert.True(part.ShowConfiguration2(component.ReferencedConfiguration),
                        "Could not activate component configuration " + component.ReferencedConfiguration);
                }
                part.ClearSelection2(true);
                var partMass = (IMassProperty2)part.Extension.CreateMassProperty2();
                partMass.UseSystemUnits = true;
                var identity = (MathTransform)((MathUtility)SwApp.GetMathUtility()).CreateTransform(
                    new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 });
                Assert.True(partMass.SetCoordinateSystem(identity));
                Assert.True(partMass.Recalculate());
                double originalMass = partMass.Mass;
                double[] originalMoment = (double[])partMass.GetMomentOfInertia(
                    (int)swMomentsOfInertiaReferenceFrame_e.swMomentsOfInertiaReferenceFrame_CenterOfMass);
                double overriddenMass = originalMass + 2.5;
                var options = (IMassPropertyOverrideOptions)partMass.GetOverrideOptions();
                options.OverrideMass = true;
                options.OverrideCenterOfMass = false;
                options.OverrideMomentsOfInertia = false;
                Assert.True(options.SetOverrideMassValue(overriddenMass));
                Assert.True(partMass.SetOverrideOptions(options,
                    (int)swInConfigurationOpts_e.swThisConfiguration, null));
                part.ForceRebuild3(false);
                doc.ForceRebuild3(false);
                doc.ClearSelection2(true);

                var helper = new ExportHelper(SwApp);
                var access = new PrivateObject(helper);
                var node = ConfigurationSerialization.LoadBaseNodeFromModel(doc, out bool error);
                Assert.False(error);
                var link = new Link { Name = "overridden_part" };
                link.SWComponents.Add(component);
                link.Joint.CoordinateSystemName = node.Link.Joint.CoordinateSystemName;
                var frame = (MathTransform)access.Invoke("GetCoordinateSystemTransform",
                    link.Joint.CoordinateSystemName);
                Assert.NotNull(frame);

                access.Invoke("ComputeInertialProperties", link);
                Assert.Equal(overriddenMass, link.Inertial.Mass.Value, 9);

                // Independently transform the part's own CoM and inertia into the link frame.
                part.ClearSelection2(true);
                var expected = (IMassProperty2)part.Extension.CreateMassProperty2();
                expected.UseSystemUnits = true;
                Assert.True(expected.SetCoordinateSystem(identity));
                Assert.True(expected.Recalculate());
                double[] overriddenMoment = (double[])expected.GetMomentOfInertia(
                    (int)swMomentsOfInertiaReferenceFrame_e.swMomentsOfInertiaReferenceFrame_CenterOfMass);
                double massRatio = overriddenMass / originalMass;
                for (int i = 0; i < 9; i++)
                {
                    Assert.Equal(originalMoment[i] * massRatio, overriddenMoment[i], 9);
                }
                System.Console.WriteLine("{0}: mass ratio={1:R}, Ixx ratio={2:R}",
                    modelName, massRatio, overriddenMoment[0] / originalMoment[0]);
                var relative = MathOps.GetTransformation(frame).Inverse() *
                    MathOps.GetTransformation(component.Transform2);
                double[] center = (double[])expected.CenterOfMass;
                var point = MathNet.Numerics.LinearAlgebra.Double.DenseVector.OfArray(
                    new double[] { center[0], center[1], center[2], 1 });
                var transformedCenter = relative * point;
                double[] actualCenter = link.Inertial.Origin.GetXYZ();
                for (int i = 0; i < 3; i++)
                {
                    Assert.Equal(transformedCenter[i], actualCenter[i], 9);
                }
                var moment = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3, 3,
                    (double[])expected.GetMomentOfInertia(
                        (int)swMomentsOfInertiaReferenceFrame_e.swMomentsOfInertiaReferenceFrame_CenterOfMass)).Transpose();
                var rotation = relative.SubMatrix(0, 3, 0, 3);
                double[] expectedMoment = (rotation * moment * rotation.Transpose()).ToRowMajorArray();
                double[] actualMoment = link.Inertial.Inertia.GetMoment();
                for (int i = 0; i < 9; i++)
                {
                    Assert.Equal(expectedMoment[i], actualMoment[i], 9);
                }

                // Demonstrate that this fixture actually catches the original body-based bug.
                var oldLink = new Link { Name = "old_calculation" };
                oldLink.SWComponents.Add(component);
                access.Invoke("ComputeInertialPropertiesFromBodies", oldLink, frame);
                Assert.True(System.Math.Abs(oldLink.Inertial.Mass.Value - overriddenMass) > 1.0);
                System.Console.WriteLine("{0}: assigned={1:R} kg, exported={2:R} kg, old={3:R} kg",
                    modelName, overriddenMass, link.Inertial.Mass.Value, oldLink.Inertial.Mass.Value);

                // Combining an overridden part with another part must add their masses.
                Component2 other = ((object[])assembly.GetComponents(false)).Cast<Component2>()
                    .First(comp => comp.GetPathName() != component.GetPathName() &&
                        ((ModelDoc2)comp.GetModelDoc2()).GetType() == (int)swDocumentTypes_e.swDocPART);
                var otherLink = new Link { Name = "other_part" };
                otherLink.Joint.CoordinateSystemName = link.Joint.CoordinateSystemName;
                otherLink.SWComponents.Add(other);
                access.Invoke("ComputeInertialProperties", otherLink);
                Assert.True(otherLink.Inertial.Mass.Value > 0);
                link.SWComponents.Add(other);
                access.Invoke("ComputeInertialProperties", link);
                Assert.Equal(overriddenMass + otherLink.Inertial.Mass.Value, link.Inertial.Mass.Value, 9);
            }
            finally
            {
                // Never save the temporary override to the example files.
                SwApp.CloseAllDocuments(true);
            }
        }

        private static IEnumerable<Link> Flatten(Link link)
        {
            yield return link;
            foreach (Link child in link.Children)
            {
                foreach (Link descendant in Flatten(child))
                {
                    yield return descendant;
                }
            }
        }
    }
}
