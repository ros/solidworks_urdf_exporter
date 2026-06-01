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
                file.WriteLine("  <exec_depend>gazebo_ros</exec_depend>");
                file.WriteLine("  <exec_depend>ros_gz_sim</exec_depend>");
                file.WriteLine("  <exec_depend>ros_gz_bridge</exec_depend>");
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
                file.WriteLine("foreach(dir config launch meshes rviz textures urdf)");
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
                file.WriteLine("    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')");
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
                file.WriteLine("            arguments=['-d', rviz_config],");
                file.WriteLine("        ),");
                file.WriteLine("    ])");
            }
        }

        // Python launch file: spawn the robot in Gazebo Classic via gazebo_ros.
        public static void WriteGazeboLaunch(string launchDir, string packageName, string robotURDF, string modelName)
        {
            string savePath = launchDir + "gazebo.launch.py";
            logger.Info("Creating ROS 2 gazebo launch at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("import glob");
                file.WriteLine("import os");
                file.WriteLine();
                file.WriteLine("from ament_index_python.packages import get_package_share_directory");
                file.WriteLine("from launch import LaunchDescription");
                file.WriteLine("from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable");
                file.WriteLine("from launch.launch_description_sources import PythonLaunchDescriptionSource");
                file.WriteLine("from launch.substitutions import Command");
                file.WriteLine("from launch_ros.actions import Node");
                file.WriteLine("from launch_ros.parameter_descriptions import ParameterValue");
                file.WriteLine();
                file.WriteLine();
                file.WriteLine("def generate_launch_description():");
                file.WriteLine("    pkg_share = get_package_share_directory('" + packageName + "')");
                file.WriteLine("    urdf = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')");
                file.WriteLine("    gazebo_ros_share = get_package_share_directory('gazebo_ros')");
                file.WriteLine();
                file.WriteLine("    # gzclient (GUI) does not load the gazebo_ros plugins, so it cannot resolve");
                file.WriteLine("    # the URDF's package:// meshes itself; point GAZEBO_RESOURCE_PATH/MODEL_PATH at");
                file.WriteLine("    # the directory containing the package share. Keep the built-in");
                file.WriteLine("    # /usr/share/gazebo-* dirs too, or gazebo loses ground_plane/sun and stalls");
                file.WriteLine("    # on the online model DB.");
                file.WriteLine("    share_parent = os.path.dirname(pkg_share)");
                file.WriteLine("    gazebo_dirs = sorted(glob.glob('/usr/share/gazebo-*'))");
                file.WriteLine("    resource_dirs = [share_parent] + gazebo_dirs");
                file.WriteLine("    model_dirs = [share_parent] + [d + '/models' for d in gazebo_dirs]");
                file.WriteLine();
                file.WriteLine("    robot_description = ParameterValue(");
                file.WriteLine("        Command(['xacro ', urdf]), value_type=str)");
                file.WriteLine();
                file.WriteLine("    return LaunchDescription([");
                file.WriteLine("        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join(resource_dirs)),");
                file.WriteLine("        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(model_dirs)),");
                file.WriteLine("        # Pin Gazebo Classic transport to loopback (host-network containers with");
                file.WriteLine("        # many interfaces can pick one with no multicast route, leaving the GUI empty).");
                file.WriteLine("        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),");
                file.WriteLine("        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345'),");
                file.WriteLine("        IncludeLaunchDescription(");
                file.WriteLine("            PythonLaunchDescriptionSource(");
                file.WriteLine("                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py'))),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='robot_state_publisher',");
                file.WriteLine("            executable='robot_state_publisher',");
                file.WriteLine("            parameters=[{'robot_description': robot_description}],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("        # Spawn from the /robot_description topic, NOT -file: spawn_entity reads");
                file.WriteLine("        # -file as a string and chokes on the URDF's encoding=\"utf-8\" declaration.");
                file.WriteLine("        Node(");
                file.WriteLine("            package='gazebo_ros',");
                file.WriteLine("            executable='spawn_entity.py',");
                file.WriteLine("            arguments=['-entity', '" + modelName + "', '-topic', 'robot_description'],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("    ])");
            }
        }

        // Python launch file: spawn the robot in modern Gazebo (Gazebo Sim / Fortress)
        // via ros_gz_sim. Classic Gazebo 11 reached EOL in 2025-01.
        public static void WriteGzSimLaunch(string launchDir, string packageName, string robotURDF, string modelName)
        {
            string savePath = launchDir + "gz_sim.launch.py";
            logger.Info("Creating ROS 2 gz_sim launch at " + savePath);
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("import os");
                file.WriteLine();
                file.WriteLine("from ament_index_python.packages import get_package_share_directory");
                file.WriteLine("from launch import LaunchDescription");
                file.WriteLine("from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription");
                file.WriteLine("from launch.launch_description_sources import PythonLaunchDescriptionSource");
                file.WriteLine("from launch.substitutions import Command");
                file.WriteLine("from launch_ros.actions import Node");
                file.WriteLine("from launch_ros.parameter_descriptions import ParameterValue");
                file.WriteLine();
                file.WriteLine();
                file.WriteLine("def generate_launch_description():");
                file.WriteLine("    pkg_share = get_package_share_directory('" + packageName + "')");
                file.WriteLine("    urdf = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')");
                file.WriteLine("    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')");
                file.WriteLine();
                file.WriteLine("    # URDF->SDF rewrites package:// mesh URIs to model://; point gz's resource");
                file.WriteLine("    # path at the directory containing the package share so meshes are found.");
                file.WriteLine("    resource_path = os.path.dirname(pkg_share)");
                file.WriteLine();
                file.WriteLine("    robot_description = ParameterValue(");
                file.WriteLine("        Command(['xacro ', urdf]), value_type=str)");
                file.WriteLine();
                file.WriteLine("    return LaunchDescription([");
                file.WriteLine("        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),");
                file.WriteLine("        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),");
                file.WriteLine("        IncludeLaunchDescription(");
                file.WriteLine("            PythonLaunchDescriptionSource(");
                file.WriteLine("                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),");
                file.WriteLine("            launch_arguments={'gz_args': '-r empty.sdf'}.items(),");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='robot_state_publisher',");
                file.WriteLine("            executable='robot_state_publisher',");
                file.WriteLine("            parameters=[{'robot_description': robot_description,");
                file.WriteLine("                         'use_sim_time': True}],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='ros_gz_sim',");
                file.WriteLine("            executable='create',");
                file.WriteLine("            arguments=['-topic', 'robot_description', '-name', '" + modelName + "'],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("        Node(");
                file.WriteLine("            package='ros_gz_bridge',");
                file.WriteLine("            executable='parameter_bridge',");
                file.WriteLine("            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],");
                file.WriteLine("            output='screen',");
                file.WriteLine("        ),");
                file.WriteLine("    ])");
            }
        }

        // Writes the RViz2 display config referenced by display.launch.py (-d), so RViz
        // opens already showing the model (Grid + RobotModel via /robot_description + TF,
        // Fixed Frame = the model's base link). Creates the rviz/ directory if needed.
        public static void WriteRvizConfig(string savePath, string fixedFrame)
        {
            logger.Info("Creating ROS 2 RViz config at " + savePath);
            string dir = Path.GetDirectoryName(savePath);
            if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir))
            {
                Directory.CreateDirectory(dir);
            }
            using (StreamWriter file = new StreamWriter(savePath))
            {
                file.WriteLine("Panels:");
                file.WriteLine("  - Class: rviz_common/Displays");
                file.WriteLine("    Name: Displays");
                file.WriteLine("    Property Tree Widget:");
                file.WriteLine("      Expanded:");
                file.WriteLine("        - /Global Options1");
                file.WriteLine("        - /RobotModel1");
                file.WriteLine("      Splitter Ratio: 0.5");
                file.WriteLine("  - Class: rviz_common/Views");
                file.WriteLine("    Name: Views");
                file.WriteLine("Visualization Manager:");
                file.WriteLine("  Class: \"\"");
                file.WriteLine("  Displays:");
                file.WriteLine("    - Class: rviz_default_plugins/Grid");
                file.WriteLine("      Name: Grid");
                file.WriteLine("      Enabled: true");
                file.WriteLine("      Cell Size: 0.1");
                file.WriteLine("      Plane Cell Count: 20");
                file.WriteLine("    - Class: rviz_default_plugins/RobotModel");
                file.WriteLine("      Name: RobotModel");
                file.WriteLine("      Enabled: true");
                file.WriteLine("      Description Source: Topic");
                file.WriteLine("      Description Topic:");
                file.WriteLine("        Value: /robot_description");
                file.WriteLine("        Depth: 5");
                file.WriteLine("        Durability Policy: Volatile");
                file.WriteLine("        History Policy: Keep Last");
                file.WriteLine("        Reliability Policy: Reliable");
                file.WriteLine("      Visual Enabled: true");
                file.WriteLine("      Collision Enabled: false");
                file.WriteLine("    - Class: rviz_default_plugins/TF");
                file.WriteLine("      Name: TF");
                file.WriteLine("      Enabled: true");
                file.WriteLine("      Show Names: true");
                file.WriteLine("      Show Axes: true");
                file.WriteLine("      Show Arrows: false");
                file.WriteLine("      Marker Scale: 0.3");
                file.WriteLine("  Global Options:");
                file.WriteLine("    Background Color: 48; 48; 48");
                file.WriteLine("    Fixed Frame: " + fixedFrame);
                file.WriteLine("    Frame Rate: 30");
                file.WriteLine("  Tools:");
                file.WriteLine("    - Class: rviz_default_plugins/MoveCamera");
                file.WriteLine("    - Class: rviz_default_plugins/Select");
                file.WriteLine("    - Class: rviz_default_plugins/FocusCamera");
                file.WriteLine("    - Class: rviz_default_plugins/Measure");
                file.WriteLine("  Views:");
                file.WriteLine("    Current:");
                file.WriteLine("      Class: rviz_default_plugins/Orbit");
                file.WriteLine("      Name: Current View");
                file.WriteLine("      Distance: 1.0");
                file.WriteLine("      Focal Point:");
                file.WriteLine("        X: 0");
                file.WriteLine("        Y: 0");
                file.WriteLine("        Z: 0.1");
                file.WriteLine("      Pitch: 0.4");
                file.WriteLine("      Yaw: 0.785");
                file.WriteLine("      Target Frame: " + fixedFrame);
                file.WriteLine("Window Geometry:");
                file.WriteLine("  Height: 800");
                file.WriteLine("  Width: 1200");
                file.WriteLine("  Displays:");
                file.WriteLine("    collapsed: false");
            }
        }
    }
}
