using Moq;
using SW2URDF.UI;
using SW2URDF.URDFExport;
using System;
using System.IO;
using System.Windows;
using Xunit;

namespace SW2URDF.Test
{
    public class TestURDFPackage : SW2URDFTest
    {
        public TestURDFPackage(SWTestFixture fixture) : base(fixture)
        {
        }

        [Fact]
        public void TestCreateDirectories()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            URDFPackage pkg = new URDFPackage(name, tempDirectory);

            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();

            Assert.True(Directory.Exists(pkg.WindowsPackageDirectory));
            Assert.True(Directory.Exists(pkg.WindowsMeshesDirectory));
            Assert.True(Directory.Exists(pkg.WindowsRobotsDirectory));
            Assert.True(Directory.Exists(pkg.WindowsTexturesDirectory));
            Assert.True(Directory.Exists(pkg.WindowsLaunchDirectory));
            Assert.True(Directory.Exists(pkg.WindowsConfigDirectory));

            Directory.Delete(tempDirectory, true);
        }

        [Fact]
        public void TestCreateCMakeLists()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            URDFPackage pkg = new URDFPackage(name, tempDirectory);
            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();
            pkg.CreateCMakeLists();

            Assert.True(File.Exists(pkg.WindowsCMakeLists));

            Console.WriteLine("Deleting directory " + tempDirectory);
            Directory.Delete(tempDirectory, true);
        }

        [Fact]
        public void TestCreateConfigYAML()
        {
            string tempDirectory = CreateRandomTempDirectory();
            string name = Path.GetRandomFileName();
            Directory.CreateDirectory(tempDirectory);
            URDFPackage pkg = new URDFPackage(name, tempDirectory);
            Mock<IMessageBox> messageBoxMock = new Mock<IMessageBox>();
            messageBoxMock.Setup(m => m.Show(It.IsAny<string>()))
                .Returns(MessageBoxResult.OK); //can be whatever depends on test case
            URDFPackage.MessageBox = messageBoxMock.Object;
            pkg.CreateDirectories();
            pkg.CreateConfigYAML(new string[] {"a", "b", "c"});

            Assert.True(File.Exists(pkg.WindowsConfigYAML));

            Console.WriteLine("Deleting directory " + tempDirectory);
            Directory.Delete(tempDirectory, true);
        }
    }
}
