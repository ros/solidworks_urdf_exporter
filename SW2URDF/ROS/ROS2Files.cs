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

using log4net;
using SW2URDF.Utilities;
using System.IO;

namespace SW2URDF.ROS
{
    // Selects which ROS ecosystem the exported package scaffold targets.
    public enum ROSVersion
    {
        ROS1,
        ROS2,
    }

    // Writes the ROS 2 (ament_cmake) flavour of the package scaffold: package.xml
    // (format 3), CMakeLists.txt and Python launch files. ROS 1 output is left
    // untouched in URDFPackage / PackageXMLWriter / ROSFiles.
    public static class ROS2Files
    {
        private static readonly ILog logger = Logger.GetLogger();

        // ament_cmake package manifest (package.xml, format 3).
        public static void WritePackageXML(string savePath, string packageName)
        {
            logger.Info("Creating ROS 2 package.xml at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("<?xml version=\"1.0\"?>");
                file.WriteLine("<?xml-model href=\"http://download.ros.org/schema/package_format3.xsd\" " +
                    "schematypens=\"http://www.w3.org/2001/XMLSchema\"?>");
                file.WriteLine("<package format=\"3\">");
                file.WriteLine("  <name>" + packageName + "</name>");
                file.WriteLine("  <version>1.0.0</version>");
                file.WriteLine("  <description>URDF Description package for " + packageName + "</description>");
                file.WriteLine("  <maintainer email=\"todo@todo.todo\">TODO</maintainer>");
                file.WriteLine("  <license>BSD</license>");
                file.WriteLine();
                file.WriteLine("  <buildtool_depend>ament_cmake</buildtool_depend>");
                file.WriteLine();
                file.WriteLine("  <exec_depend>robot_state_publisher</exec_depend>");
                file.WriteLine("  <exec_depend>joint_state_publisher_gui</exec_depend>");
                file.WriteLine("  <exec_depend>rviz2</exec_depend>");
                file.WriteLine("  <exec_depend>xacro</exec_depend>");
                file.WriteLine("  <exec_depend>launch</exec_depend>");
                file.WriteLine("  <exec_depend>launch_ros</exec_depend>");
                file.WriteLine();
                file.WriteLine("  <export>");
                file.WriteLine("    <build_type>ament_cmake</build_type>");
                file.WriteLine("  </export>");
                file.WriteLine("</package>");
            }
        }

        // ament_cmake CMakeLists.txt that installs the resource directories.
        public static void WriteCMakeLists(string savePath, string packageName)
        {
            logger.Info("Creating ROS 2 CMakeLists.txt at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("cmake_minimum_required(VERSION 3.8)");
                file.WriteLine("project(" + packageName + ")");
                file.WriteLine();
                file.WriteLine("find_package(ament_cmake REQUIRED)");
                file.WriteLine();
                // Install only the resource directories the exporter actually produced.
                // install(DIRECTORY ...) errors on a directory that does not exist (e.g. a
                // model with no textures or meshes), which would fail colcon build.
                file.WriteLine("foreach(dir config launch meshes textures urdf)");
                file.WriteLine("  if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/${dir}\")");
                file.WriteLine("    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})");
                file.WriteLine("  endif()");
                file.WriteLine("endforeach()");
                file.WriteLine();
                file.WriteLine("ament_package()");
            }
        }

        // Python launch file: robot_state_publisher + joint_state_publisher_gui + rviz2.
        public static void WriteDisplayLaunch(string launchDir, string packageName, string robotURDF)
        {
            string savePath = launchDir + "display.launch.py";
            logger.Info("Creating ROS 2 display launch at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("import os");
                file.WriteLine();
                file.WriteLine("from ament_index_python.packages import get_package_share_directory");
                file.WriteLine("from launch import LaunchDescription");
                file.WriteLine("from launch.actions import DeclareLaunchArgument");
                file.WriteLine("from launch.substitutions import Command, LaunchConfiguration");
                file.WriteLine("from launch_ros.actions import Node");
                file.WriteLine("from launch_ros.parameter_descriptions import ParameterValue");
                file.WriteLine();
                file.WriteLine();
                file.WriteLine("def generate_launch_description():");
                file.WriteLine("    pkg_share = get_package_share_directory('" + packageName + "')");
                file.WriteLine("    default_model = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')");
                file.WriteLine();
                file.WriteLine("    robot_description = ParameterValue(");
                file.WriteLine("        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)");
                file.WriteLine();
                file.WriteLine("    return LaunchDescription([");
                file.WriteLine("        DeclareLaunchArgument(name='model', default_value=default_model),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='robot_state_publisher',");
                file.WriteLine("            executable='robot_state_publisher',");
                file.WriteLine("            parameters=[{'robot_description': robot_description}],");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='joint_state_publisher_gui',");
                file.WriteLine("            executable='joint_state_publisher_gui',");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='rviz2',");
                file.WriteLine("            executable='rviz2',");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("    ])");
            }
        }

        // Python launch file: bring up the robot in Gazebo (gazebo_ros) and spawn it.
        public static void WriteGazeboLaunch(string launchDir, string packageName, string robotURDF, string modelName)
        {
            string savePath = launchDir + "gazebo.launch.py";
            logger.Info("Creating ROS 2 gazebo launch at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("import os");
                file.WriteLine();
                file.WriteLine("from ament_index_python.packages import get_package_share_directory");
                file.WriteLine("from launch import LaunchDescription");
                file.WriteLine("from launch.actions import IncludeLaunchDescription");
                file.WriteLine("from launch.launch_description_sources import PythonLaunchDescriptionSource");
                file.WriteLine("from launch_ros.actions import Node");
                file.WriteLine();
                file.WriteLine();
                file.WriteLine("def generate_launch_description():");
                file.WriteLine("    pkg_share = get_package_share_directory('" + packageName + "')");
                file.WriteLine("    urdf = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')");
                file.WriteLine("    gazebo_ros_share = get_package_share_directory('gazebo_ros')");
                file.WriteLine();
                file.WriteLine("    return LaunchDescription([");
                file.WriteLine("        IncludeLaunchDescription(");
                file.WriteLine("            PythonLaunchDescriptionSource(");
                file.WriteLine("                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py'))),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='robot_state_publisher',");
                file.WriteLine("            executable='robot_state_publisher',");
                file.WriteLine("            arguments=[urdf],");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='gazebo_ros',");
                file.WriteLine("            executable='spawn_entity.py',");
                file.WriteLine("            arguments=['-entity', '" + modelName + "', '-file', urdf],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("    ])");
            }
        }
    }
}
