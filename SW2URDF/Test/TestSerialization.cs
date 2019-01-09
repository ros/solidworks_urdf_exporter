using Microsoft.VisualStudio.TestTools.UnitTesting;
using SolidWorks.Interop.sldworks;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using Xunit;

namespace SW2URDF.Test
{
    [Collection("Requires SW Test Collection")]
    public class TestSerialization : SW2URDFTest
    {
        public TestSerialization(SWTestFixture fixture) : base(fixture)
        {

        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        public void TestLoadConfigFromStringXML(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            PrivateType serialization = new PrivateType(typeof(Serialization));
            object swAttObj = serialization.InvokeStatic("FindSWSaveAttribute", new object[] { doc, "URDF Export Configuration" });
            Xunit.Assert.NotNull(swAttObj);

            Attribute swAtt = (Attribute)swAttObj;
            Parameter param = swAtt.GetParameter("data");

            Xunit.Assert.NotNull(param);
            string data = param.GetStringValue();

            Xunit.Assert.NotNull(data);
            Xunit.Assert.NotEmpty(data);

            LinkNode baseNode = (LinkNode)serialization.InvokeStatic("LoadConfigFromStringXML", new object[] { data });
            Link link = baseNode.GetLink();
            Xunit.Assert.Equal(expNumLinks, Common.GetCount(link));
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        [InlineData("4_WHEELER", 5)]
        public void TestLoadBaseNodeFromModel(string modelName, int expNumLinks)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);
            Xunit.Assert.False(error);
            Xunit.Assert.NotNull(baseNode);
            Xunit.Assert.Equal(expNumLinks, Common.GetCount(baseNode.GetLink()));
        }

        [Theory]
        [InlineData("3_DOF_ARM")]
        [InlineData("4_WHEELER")]
        public void TestSerializeToString(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool error);

            PrivateType serialization = new PrivateType(typeof(Serialization));
            string newData = (string)serialization.InvokeStatic("SerializeToString", new object[] { baseNode });
            Xunit.Assert.NotNull(newData);
            Xunit.Assert.NotEmpty(newData);

        }
    }
}