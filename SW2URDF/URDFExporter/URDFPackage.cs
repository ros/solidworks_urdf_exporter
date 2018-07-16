/*
Copyright (c) 2015 Stephen Brawner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using System;
using System.IO;

namespace SW2URDF
{
    public class URDFPackage
    {
        public string PackageName { get; }

        public string PackageDirectory { get; }
        public string MeshesDirectory { get; }
        public string TexturesDirectory { get; }
        public string RobotsDirectory { get; }
        public string ConfigDirectory { get; }
        public string LaunchDirectory { get; }

        public string WindowsPackageDirectory { get; }
        public string WindowsMeshesDirectory { get; }
        public string WindowsTexturesDirectory { get; }
        public string WindowsRobotsDirectory { get; }
        public string WindowsLaunchDirectory { get; }
        public string WindowsConfigDirectory { get; }
        public string WindowsCMakeLists { get; }
        public string WindowsConfigYAML { get; }

        public URDFPackage(string name, string dir)
        {
            PackageName = name;
            PackageDirectory = @"package://" + name + @"/";
            MeshesDirectory = PackageDirectory + @"meshes/";
            RobotsDirectory = PackageDirectory + @"urdf/";
            TexturesDirectory = PackageDirectory + @"textures/";
            LaunchDirectory = PackageDirectory + @"launch/";
            ConfigDirectory = PackageDirectory + @"config/";

            char last = dir[dir.Length - 1];
            dir = (last == '\\') ? dir : dir + @"\";
            WindowsPackageDirectory = dir + name + @"\";
            WindowsMeshesDirectory = WindowsPackageDirectory + @"meshes\";
            WindowsRobotsDirectory = WindowsPackageDirectory + @"urdf\";
            WindowsTexturesDirectory = WindowsPackageDirectory + @"textures\";
            WindowsLaunchDirectory = WindowsPackageDirectory + @"launch\";
            WindowsConfigDirectory = WindowsPackageDirectory + @"config\";
            WindowsCMakeLists = WindowsPackageDirectory + @"CMakeLists.txt";
            WindowsConfigYAML = WindowsConfigDirectory + @"joint_names_" + name + ".yaml";
        }

        public void CreateDirectories()
        {
            System.Windows.Forms.MessageBox.Show("Creating URDF Package \"" +
                PackageName + "\" at:\n" + WindowsPackageDirectory);
            if (!Directory.Exists(WindowsPackageDirectory))
            {
                Directory.CreateDirectory(WindowsPackageDirectory);
            }
            if (!Directory.Exists(WindowsMeshesDirectory))
            {
                Directory.CreateDirectory(WindowsMeshesDirectory);
            }
            if (!Directory.Exists(WindowsRobotsDirectory))
            {
                Directory.CreateDirectory(WindowsRobotsDirectory);
            }
            if (!Directory.Exists(WindowsTexturesDirectory))
            {
                Directory.CreateDirectory(WindowsTexturesDirectory);
            }
            if (!Directory.Exists(WindowsLaunchDirectory))
            {
                Directory.CreateDirectory(WindowsLaunchDirectory);
            }
            if (!Directory.Exists(WindowsConfigDirectory))
            {
                Directory.CreateDirectory(WindowsConfigDirectory);
            }
        }

        public void CreateCMakeLists()
        {
            using (StreamWriter file = new StreamWriter(WindowsCMakeLists))
            {
                file.WriteLine("cmake_minimum_required(VERSION 2.8.3)\r\n");
                file.WriteLine("project(" + PackageName + ")\r\n");
                file.WriteLine("find_package(catkin REQUIRED)\r\n");
                file.WriteLine("catkin_package()\r\n");
                file.WriteLine("find_package(roslaunch)\r\n");
                file.WriteLine("foreach(dir config launch meshes urdf)");
                file.WriteLine("\tinstall(DIRECTORY ${dir}/");
                file.WriteLine("\t\tDESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})");
                file.WriteLine("endforeach(dir)");
            }
        }

        public void CreateConfigYAML(String[] jointNames)
        {
            using (StreamWriter file = new StreamWriter(WindowsConfigYAML))
            {
                file.Write("controller_joint_names: " + "[");

                foreach (String name in jointNames)
                {
                    file.Write("'" + name + "', ");
                }

                file.WriteLine("]");
            }
        }
    }
}