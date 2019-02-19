using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using System.IO;
using System.Reflection;
using Xunit;

namespace SW2URDF.Test
{
    /// <summary>
    /// Base class for each Test class. This file contains many helper functions as well
    /// as access to the TestFixture which contains the SwApp reference.
    /// </summary>
    public abstract class SW2URDFTest : IClassFixture<SWTestFixture>
    {
        public const string MODEL_NAME_3_DOF_ARM = "3_DOF_ARM";
        public const string MODEL_NAME_4_WHEELER = "4_WHEELER";
        public const string MODEL_NAME_ORIGINAL_3_DOF_ARM = "ORIGINAL_3_DOF_ARM";
        protected readonly SWTestFixture TestFixture;
        protected readonly SldWorks SwApp;
        public SW2URDFTest(SWTestFixture fixture)
        {
            SWTestFixture.Initialize();
            TestFixture = fixture;
            SwApp = SWTestFixture.SwApp;
        }

        public void Dispose()
        {
            Assert.True(SwApp.CloseAllDocuments(true));
        }

        public string GetDebugDirectory()
        {
            string baseDirectory = System.AppDomain.CurrentDomain.BaseDirectory;
            return baseDirectory;
        }

        public string GetX64Directory()
        {
            string debugDirectory = GetDebugDirectory();
            return Path.GetDirectoryName(debugDirectory);
        }

        public string GetBinDirectory()
        {
            string x64Directory = GetX64Directory();
            return Path.GetDirectoryName(x64Directory);
        }

        public string GetProjectDirectory()
        {
            string binDirectory = GetBinDirectory();
            return Path.GetDirectoryName(binDirectory);
        }

        public string GetSolutionDirectory()
        {
            string projectDirectory = GetProjectDirectory();
            return Path.GetDirectoryName(projectDirectory);
        }

        public string GetExamplesDirectory()
        {
            string solutionDirectory = GetSolutionDirectory();
            return Path.Combine(solutionDirectory, "examples");
        }

        public string GetModelDirectory(string modelName)
        {
            string examplesDirectory = GetExamplesDirectory();
            return Path.Combine(examplesDirectory, modelName);
        }

        public string GetPackageDirectory(string modelName)
        {
            string modelDirectory = GetModelDirectory(modelName);
            string package_name = modelName + "_description";
            return Path.Combine(modelDirectory, package_name);
        }

        public string GetURDFDirectory(string modelName)
        {
            string packageDirectory = GetPackageDirectory(modelName);
            return Path.Combine(packageDirectory, "urdf");
        }

        public string GetMeshesDirectory(string modelName)
        {
            string packageDirectory = GetPackageDirectory(modelName);
            return Path.Combine(packageDirectory, "meshes");
        }

        public string GetCSVPath(string modelName)
        {
            string urdfDirectory = GetURDFDirectory(modelName);
            string fileName = modelName + "_description.csv";
            return Path.Combine(urdfDirectory, fileName);
        }

        public ModelDoc2 OpenSWDocument(string modelName)
        {
            Assert.True(SwApp.CloseAllDocuments(true));

            string modelDirectory = GetModelDirectory(modelName);
            string filename = Path.Combine(modelDirectory, modelName + ".SLDASM");
            Assert.True(File.Exists(filename));
            int Errors = 0;
            int Warnings = 0;
            int filetype = (int)swDocumentTypes_e.swDocASSEMBLY;
            string configuration = "";

            ModelDoc2 doc = SwApp.OpenDoc6(filename, filetype, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, 
                                           configuration, ref Errors, ref Warnings);
            Assert.Equal(0, Errors);
            Assert.Equal(0, Warnings);
            return doc;
        }

        public ModelDoc2 OpenSWPartDocument(string modelName)
        {
            Assert.True(SwApp.CloseAllDocuments(true));

            string modelDirectory = GetModelDirectory(modelName);
            string filename = Path.Combine(modelDirectory, modelName + ".SLDPRT");
            Assert.True(File.Exists(filename));
            int Errors = 0;
            int Warnings = 0;
            int filetype = (int)swDocumentTypes_e.swDocASSEMBLY;
            string configuration = "";

            ModelDoc2 doc = SwApp.OpenDoc6(filename, filetype, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, 
                                           configuration, ref Errors, ref Warnings);
            Assert.Equal(0, Errors);
            Assert.Equal(0, Warnings);
            return doc;
        }

        public string CreateRandomTempDirectory()
        {
            string name = Path.GetRandomFileName();
            string tempDirectory = Path.Combine(Path.GetTempPath(), name);
            Assert.True(Directory.CreateDirectory(tempDirectory).Exists);
            return tempDirectory;
        }
    }
}
