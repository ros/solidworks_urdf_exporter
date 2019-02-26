using SolidWorks.Interop.sldworks;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using SW2URDF.URDFExport.CSV;
using System;
using System.Collections.Generic;
using System.IO;
using Xunit;

namespace SW2URDF.Test
{
    [Collection("Requires SW Test Collection")]
    public class TestCSVImportExport : SW2URDFTest
    {
        public TestCSVImportExport(SWTestFixture fixture) : base(fixture)
        {
        }

        [Theory]
        [InlineData("3_DOF_ARM", 4)]
        public void TestLoadURDFRobotFromCSV(string modelName, int expNumLinks)
        {
            string csvFilename = GetCSVPath(modelName);
            using (StreamReader reader = new StreamReader(csvFilename))
            {
                List<Link> links = ImportExport.LoadURDFRobotFromCSV(reader.BaseStream);
                Assert.Equal(expNumLinks, links.Count);
                foreach (Link link in links)
                {
                    Assert.NotNull(link);
                }
            }
        }

        [Theory]
        [InlineData("3_DOF_ARM", 5)]
        public void TestWriteRobotToCSV(string modelName, int expNumLines)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(SwApp, doc, out bool abortProcess);
            Assert.False(abortProcess);
            Link baseLink = baseNode.GetLink();
            Robot robot = new Robot();
            robot.SetBaseLink(baseLink);

            string tempFile = Path.GetTempFileName();
            
            try
            {
                ImportExport.WriteRobotToCSV(robot, tempFile);
                Assert.True(File.Exists(tempFile));

                string[] text = File.ReadAllLines(tempFile);
                Assert.Equal(expNumLines, text.Length);
            }
            catch (Exception e)
            {
                Assert.True(false, "Failed to write robot to CSV " + e.Message);
            }
            finally
            {
                if (File.Exists(tempFile))
                {
                    File.Delete(tempFile);
                    Assert.True(SwApp.CloseAllDocuments(true));
                }
            }

        }
    }
}
