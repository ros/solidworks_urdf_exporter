using SolidWorks.Interop.sldworks;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace SW2URDF.Test
{
    [Collection("Requires SW Test Collection")]
    public class TestExportHelper : SW2URDFTest
    {
        public TestExportHelper(SWTestFixture fixture) : base(fixture)
        {

        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        [InlineData("ORIGINAL_3_DOF_ARM", 4)]
        public void TestExportRobot(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            helper.SetComputeInertial(true);
            helper.SetComputeJointKinematics(true);
            helper.SetComputeJointLimits(true);
            helper.SetComputeVisualCollision(true);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            Assert.NotNull(helper.URDFRobot);
            Assert.Equal(expNumLinks, Common.GetCount(helper.URDFRobot.BaseLink));
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);
            helper.CreateRobotFromTreeView(baseNode);
            helper.ExportRobot(true);
            List<string> jointNames = helper.GetJointNames();
            Assert.NotNull(jointNames);
            Assert.Equal(jointNames.Count, expNumJoints);
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        /*
         * TODO(brawnen) Part document tests not working (OpenSWPartDocument)
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
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Assert.False(error);

            helper.CreateRobotFromTreeView(baseNode);
            Assert.NotNull(helper.URDFRobot);
            Assert.True(SwApp.CloseAllDocuments(true));
        }


        [Theory]
        [InlineData("3_DOF_ARM", new double[] { 0, 0, 1 }, "global_origin", new double[] { 0, 0, 1 })]
        public void TestLocalizeAxis(string modelName, double[] axis, string coordSys, double[] expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(expected, helper.LocalizeAxis(axis, coordSys));
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", new string[] { "Origin_global", "Origin_prox_joint", "Origin_dist_joint", "Origin_effector_joint" })]
        public void TestGetRefCoordinateSystems(string modelName, string[] expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(new List<string>(expected), helper.GetRefCoordinateSystems());
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        [Theory]
        [InlineData("3_DOF_ARM", new string[] { "Axis_prox_joint", "Axis_dist_joint", "Axis_effector_joint" })]
        public void TestGetRefAxes(string modelName, string[] expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportHelper helper = new ExportHelper(SwApp);
            Assert.Equal(new List<string>(expected), helper.GetRefAxes());
            Assert.True(SwApp.CloseAllDocuments(true));
        }
    }
}