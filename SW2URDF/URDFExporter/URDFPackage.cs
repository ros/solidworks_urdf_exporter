using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace SW2URDF
{
    public class URDFPackage
    {
        private string packageName;
        public string PackageName
        {
            get
            {
                return packageName;
            }
        }

        private string packageDirectory;
        public string PackageDirectory
        {
            get
            {
                return packageDirectory;
            }
        }

        private string windowsPackageDirectory;
        public string WindowsPackageDirectory
        {
            get
            {
                return windowsPackageDirectory;
            }
        }

        private string meshesDirectory;
        public string MeshesDirectory
        {
            get
            {
                return meshesDirectory;
            }
        }

        private string windowsMeshesDirectory;
        public string WindowsMeshesDirectory
        {
            get
            {
                return windowsMeshesDirectory;
            }
        }

        private string robotsDirectory;
        public string RobotsDirectory
        {
            get
            {
                return robotsDirectory;
            }
        }

        private string windowsRobotsDirectory;
        public string WindowsRobotsDirectory
        {
            get
            {
                return windowsRobotsDirectory;
            }
        }

        private string texturesDirectory;
        public string TexturesDirectory
        {
            get
            {
                return texturesDirectory;
            }
        }

        private string windowsTexturesDirectory;
        public string WindowsTexturesDirectory
        {
            get
            {
                return windowsTexturesDirectory;
            }
        }

        private string launchDirectory;
        public string LaunchDirectory
        {
            get
            {
                return launchDirectory;
            }
        }

        private string windowsLaunchDirectory;
        public string WindowsLaunchDirectory
        {
            get
            {
                return windowsLaunchDirectory;
            }
        }


        public URDFPackage(string name, string dir)
        {
            packageName = name;
            packageDirectory = @"package://" + name + @"/";
            meshesDirectory = packageDirectory + @"meshes/";
            robotsDirectory = packageDirectory + @"robots/";
            texturesDirectory = packageDirectory + @"textures/";
            launchDirectory = packageDirectory + @"launch/";

            char last = dir[dir.Length - 1];
            dir = (last == '\\') ? dir : dir + @"\";
            windowsPackageDirectory = dir + name + @"\";
            windowsMeshesDirectory = windowsPackageDirectory + @"meshes\";
            windowsRobotsDirectory = windowsPackageDirectory + @"robots\";
            windowsTexturesDirectory = windowsPackageDirectory + @"textures\";
            windowsLaunchDirectory = windowsPackageDirectory + @"launch\";
        }

        public void createDirectories()
        {
            System.Windows.Forms.MessageBox.Show("Creating URDF Package \"" + packageName + "\" at:\n" + windowsPackageDirectory);
            if (!Directory.Exists(windowsPackageDirectory))
            {
                Directory.CreateDirectory(windowsPackageDirectory);
            }
            if (!Directory.Exists(windowsMeshesDirectory))
            {
                Directory.CreateDirectory(windowsMeshesDirectory);
            }
            if (!Directory.Exists(windowsRobotsDirectory))
            {
                Directory.CreateDirectory(windowsRobotsDirectory);
            }
            if (!Directory.Exists(windowsTexturesDirectory))
            {
                Directory.CreateDirectory(windowsTexturesDirectory);
            }
            if (!Directory.Exists(windowsLaunchDirectory))
            {
                Directory.CreateDirectory(windowsLaunchDirectory);
            }
        }
    }
}
