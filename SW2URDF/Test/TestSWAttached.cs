using Xunit;

namespace SW2URDF.Test
{
    [Collection("Requires SW Test Collection")]
    public class TestSWAttached : SW2URDFTest
    {
        public TestSWAttached(SWTestFixture fixture) : base(fixture)
        {
        }

        [Fact]
        public void Test_SWOpens()
        {
            Assert.NotNull(SwApp);
        }

        [Theory]
        [InlineData("3_DOF_ARM")]
        [InlineData("4_WHEELER")]
        [InlineData("ORIGINAL_3_DOF_ARM")]
        public void Test_ModelDoc_Opens(string modelName)
        {
            OpenSWDocument(modelName);
            Assert.True(SwApp.CloseAllDocuments(true));
        }
    }
}
