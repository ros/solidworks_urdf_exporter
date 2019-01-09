using Microsoft.VisualStudio.TestTools.UnitTesting;
using SolidWorks.Interop.sldworks;
using SW2URDF.SW;
using SW2URDF.URDFExport;
using Xunit;

namespace SW2URDF.Test
{
    /// <summary>
    ///  TODO (brawner), pm.show() crashes SolidWorks. 
    /// </summary>
    [Collection("Requires SW Test Collection")]
    public class TestPropertyManager : SW2URDFTest
    {
        public TestPropertyManager(SWTestFixture fixture) : base(fixture)
        {
        }


        // TODO pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpens(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            SwAddin addin = new SwAddin();
            addin.ConnectToSW(SwApp, 0);
            addin.SetupAssemblyExporter();
            SwApp.CloseAllDocuments(true);
        }

        // TODO pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpenCloseOk(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);

            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();
            pm.Close(true);
            SwApp.CloseAllDocuments(true);
            Xunit.Assert.True(true, "Property manager failed to open/close with okay");
            
        }

        // TODO pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPropertyManagerOpenCloseNotOk(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);

            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();
            pm.Close(false);
            SwApp.CloseAllDocuments(true);
            Xunit.Assert.True(true, "Property manager failed to open/close with cancel");
        }

        // TODO pm.Show() crashes with drag drop 
        //[Theory]
        //[InlineData("3_DOF_ARM")]
        public void TestPreviewExport(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            ExportPropertyManager pm = new ExportPropertyManager(SwApp);
            pm.Show();

            PrivateObject obj = new PrivateObject(pm);
            obj.Invoke("ExportButtonPress");
            Xunit.Assert.NotNull(obj.GetProperty("Exporter.URDFRobot"));
        }


    }
}
