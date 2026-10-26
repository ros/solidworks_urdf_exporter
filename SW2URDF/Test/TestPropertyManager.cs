using Microsoft.VisualStudio.TestTools.UnitTesting;
using SW2URDF.SW;
using SW2URDF.URDFExport;
using Xunit;

namespace SW2URDF.Test
{
    // These checks do not create a SolidWorks application or property-manager UI.
    public class TestReferenceFrames
    {
        [Theory]
        [InlineData(false, "ReferenceOrigin", false)]
        [InlineData(false, "Automatically Generate", true)]
        [InlineData(false, "", true)]
        [InlineData(true, "Automatically Generate", false)]
        public void ValidateEmptyLink(bool isBase, string origin, bool incomplete)
        {
            var manager = (ExportPropertyManager)System.Runtime.Serialization.FormatterServices
                .GetUninitializedObject(typeof(ExportPropertyManager));
            var node = new URDF.LinkNode { IsBaseNode = isBase };
            node.Link.Name = "reference";
            node.Link.Joint.Name = "reference_joint";
            node.Link.Joint.CoordinateSystemName = origin;
            node.Link.Joint.AxisName = "Automatically Generate";
            node.Link.Joint.Type = "Automatically Detect";
            node.Nodes.Add(new URDF.LinkNode());

            manager.CheckNodeComplete(node);

            Xunit.Assert.Equal(incomplete, node.IsIncomplete);
        }

        [Fact]
        public void EmptyLinkSkipsPhysicalPropertiesAndUsesFixedJoint()
        {
            var exporter = (ExportHelper)System.Runtime.Serialization.FormatterServices
                .GetUninitializedObject(typeof(ExportHelper));
            var access = new PrivateObject(exporter);
            access.SetField("ComputeInertialValues", true);
            access.SetField("ComputeVisualCollision", true);
            var node = new URDF.LinkNode();
            node.Link.Name = "reference";
            var parent = new URDF.Link { Name = "parent" };

            // No SolidWorks model is attached: attempting mass/geometry access would fail.
            var result = (URDF.Link)access.Invoke("CreateLinkFromComponents", parent, node);

            Xunit.Assert.True(result.isFixedFrame);
            Xunit.Assert.Equal("fixed", result.Joint.Type);
            Xunit.Assert.Equal("parent", result.Joint.Parent.Name);
            Xunit.Assert.Equal("reference", result.Joint.Child.Name);
        }

        [Fact]
        public void ReferenceFrameOmitsPhysicalDataAndPreservesDescendants()
        {
            var root = new URDF.Link { Name = "root", isFixedFrame = true };
            var frame = new URDF.Link(root) { Name = "reference", isFixedFrame = true };
            root.Children.Add(frame);
            frame.Joint.Name = "reference_joint";
            frame.Joint.Type = "fixed";
            frame.Joint.Parent.Name = root.Name;
            frame.Joint.Child.Name = frame.Name;
            frame.Joint.Origin.X = 0.25;
            frame.Inertial.Mass.Value = 10; // Stale physical data must not leak into URDF.
            var child = new URDF.Link(frame) { Name = "physical" };
            child.Inertial.Mass.Value = 2;
            frame.Children.Add(child);

            var buffer = new System.Text.StringBuilder();
            using (var writer = System.Xml.XmlWriter.Create(buffer))
            {
                writer.WriteStartElement("robot");
                root.WriteURDF(writer);
                writer.WriteEndElement();
            }
            var document = new System.Xml.XmlDocument { XmlResolver = null };
            var settings = new System.Xml.XmlReaderSettings { DtdProcessing = System.Xml.DtdProcessing.Prohibit };
            using (var reader = System.Xml.XmlReader.Create(new System.IO.StringReader(buffer.ToString()), settings))
            {
                document.Load(reader);
            }
            Xunit.Assert.Equal(3, document.SelectNodes("/robot/link").Count);
            Xunit.Assert.Equal(0, document.SelectNodes("/robot/link[@name='reference']/*").Count);
            Xunit.Assert.Equal(0, document.SelectNodes("/robot/link[@name='root']/*").Count);
            Xunit.Assert.NotNull(document.SelectSingleNode("/robot/link[@name='physical']/inertial"));
            Xunit.Assert.NotNull(document.SelectSingleNode("/robot/joint[@name='reference_joint'][@type='fixed']/origin"));
            Xunit.Assert.Equal("root", document.SelectSingleNode("/robot/joint/parent/@link").Value);
            Xunit.Assert.Equal("reference", document.SelectSingleNode("/robot/joint/child/@link").Value);
        }
    }

    /// <summary>
    ///  TODO (SIMINT-164), code in UI components needs to be tested, but
    ///  pm.show() crashes SolidWorks. 
    /// </summary>
    [Collection("Requires SW Test Collection")]
    public class TestPropertyManager : SW2URDFTest
    {
        public TestPropertyManager(SWTestFixture fixture) : base(fixture)
        {
        }


        // TODO(SIMINT-164) pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpens(string modelName)
        {
            OpenSWDocument(modelName);
            SwAddin addin = new SwAddin();
            addin.ConnectToSW(SwApp, 0);
            addin.SetupAssemblyExporter();
            SwApp.CloseAllDocuments(true);
        }

        // TODO(SIMINT-164) pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpenCloseOk(string modelName)
        {
            OpenSWDocument(modelName);

            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();
            pm.Close(true);
            SwApp.CloseAllDocuments(true);
            Xunit.Assert.True(true, "Property manager failed to open/close with okay");
        }

        // TODO(SIMINT-164) pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpenCloseNotOk(string modelName)
        {
            OpenSWDocument(modelName);

            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();
            pm.Close(false);
            SwApp.CloseAllDocuments(true);
            Xunit.Assert.True(true, "Property manager failed to open/close with cancel");
        }

        // TODO(SIMINT-164) pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPreviewExport(string modelName)
        {
            OpenSWDocument(modelName);
            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();

            PrivateObject obj = new PrivateObject(pm);
            obj.Invoke("ExportButtonPress");
            Xunit.Assert.NotNull(obj.GetProperty("Exporter.URDFRobot"));
        }
    }
}
