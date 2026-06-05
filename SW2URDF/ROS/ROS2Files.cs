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
using System.Collections.Generic;
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
            new Ros2LaunchFile()
                .Import(
                    "import os",
                    "",
                    "from ament_index_python.packages import get_package_share_directory",
                    "from launch import LaunchDescription",
                    "from launch.actions import DeclareLaunchArgument",
                    "from launch.substitutions import Command, LaunchConfiguration",
                    "from launch_ros.actions import Node",
                    "from launch_ros.parameter_descriptions import ParameterValue")
                .Setup(
                    "pkg_share = get_package_share_directory('" + packageName + "')",
                    "default_model = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')",
                    "rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')",
                    "",
                    "robot_description = ParameterValue(",
                    "    Command(['xacro ', LaunchConfiguration('model')]), value_type=str)")
                .Add(new Ros2DeclareArg("model", "default_model"))
                .Add(new Ros2Node("robot_state_publisher", "robot_state_publisher",
                    parameters: "[{'robot_description': robot_description}]"))
                .Add(new Ros2Node("joint_state_publisher_gui", "joint_state_publisher_gui"))
                .Add(new Ros2Node("rviz2", "rviz2", arguments: "['-d', rviz_config]", output: "screen"))
                .Write(savePath);
        }

        // Python launch file: spawn the robot in Gazebo Classic via gazebo_ros.
        public static void WriteGazeboLaunch(string launchDir, string packageName, string robotURDF, string modelName)
        {
            string savePath = launchDir + "gazebo.launch.py";
            logger.Info("Creating ROS 2 gazebo launch at " + savePath);
            new Ros2LaunchFile()
                .Import(
                    "import glob",
                    "import os",
                    "",
                    "from ament_index_python.packages import get_package_share_directory",
                    "from launch import LaunchDescription",
                    "from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable",
                    "from launch.launch_description_sources import PythonLaunchDescriptionSource",
                    "from launch.substitutions import Command",
                    "from launch_ros.actions import Node",
                    "from launch_ros.parameter_descriptions import ParameterValue")
                .Setup(
                    "pkg_share = get_package_share_directory('" + packageName + "')",
                    "urdf = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')",
                    "gazebo_ros_share = get_package_share_directory('gazebo_ros')",
                    "",
                    "# gzclient (GUI) does not load the gazebo_ros plugins, so it cannot resolve",
                    "# the URDF's package:// meshes itself; point GAZEBO_RESOURCE_PATH/MODEL_PATH at",
                    "# the directory containing the package share. Keep the built-in",
                    "# /usr/share/gazebo-* dirs too, or gazebo loses ground_plane/sun and stalls",
                    "# on the online model DB.",
                    "share_parent = os.path.dirname(pkg_share)",
                    "gazebo_dirs = sorted(glob.glob('/usr/share/gazebo-*'))",
                    "resource_dirs = [share_parent] + gazebo_dirs",
                    "model_dirs = [share_parent] + [d + '/models' for d in gazebo_dirs]",
                    "",
                    "robot_description = ParameterValue(",
                    "    Command(['xacro ', urdf]), value_type=str)")
                .Add(new Ros2SetEnv("SetEnvironmentVariable", "GAZEBO_RESOURCE_PATH", "os.pathsep.join(resource_dirs)"))
                .Add(new Ros2SetEnv("SetEnvironmentVariable", "GAZEBO_MODEL_PATH", "os.pathsep.join(model_dirs)"))
                .Add(new Ros2Comment(
                    "Pin Gazebo Classic transport to loopback (host-network containers with",
                    "many interfaces can pick one with no multicast route, leaving the GUI empty)."))
                .Add(new Ros2SetEnv("SetEnvironmentVariable", "GAZEBO_IP", "'127.0.0.1'"))
                .Add(new Ros2SetEnv("SetEnvironmentVariable", "GAZEBO_MASTER_URI", "'http://127.0.0.1:11345'"))
                .Add(new Ros2Include("os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')"))
                .Add(new Ros2Node("robot_state_publisher", "robot_state_publisher",
                    parameters: "[{'robot_description': robot_description}]", output: "screen"))
                .Add(new Ros2Comment(
                    "Spawn from the /robot_description topic, NOT -file: spawn_entity reads",
                    "-file as a string and chokes on the URDF's encoding=\"utf-8\" declaration."))
                .Add(new Ros2Node("gazebo_ros", "spawn_entity.py",
                    arguments: "['-entity', '" + modelName + "', '-topic', 'robot_description']", output: "screen"))
                .Write(savePath);
        }

        // Python launch file: spawn the robot in modern Gazebo (Gazebo Sim / Fortress)
        // via ros_gz_sim. Classic Gazebo 11 reached EOL in 2025-01.
        public static void WriteGzSimLaunch(string launchDir, string packageName, string robotURDF, string modelName)
        {
            string savePath = launchDir + "gz_sim.launch.py";
            logger.Info("Creating ROS 2 gz_sim launch at " + savePath);
            new Ros2LaunchFile()
                .Import(
                    "import os",
                    "",
                    "from ament_index_python.packages import get_package_share_directory",
                    "from launch import LaunchDescription",
                    "from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription",
                    "from launch.launch_description_sources import PythonLaunchDescriptionSource",
                    "from launch.substitutions import Command",
                    "from launch_ros.actions import Node",
                    "from launch_ros.parameter_descriptions import ParameterValue")
                .Setup(
                    "pkg_share = get_package_share_directory('" + packageName + "')",
                    "urdf = os.path.join(pkg_share, 'urdf', '" + robotURDF + "')",
                    "ros_gz_sim_share = get_package_share_directory('ros_gz_sim')",
                    "",
                    "# URDF->SDF rewrites package:// mesh URIs to model://; point gz's resource",
                    "# path at the directory containing the package share so meshes are found.",
                    "resource_path = os.path.dirname(pkg_share)",
                    "",
                    "robot_description = ParameterValue(",
                    "    Command(['xacro ', urdf]), value_type=str)")
                .Add(new Ros2SetEnv("AppendEnvironmentVariable", "IGN_GAZEBO_RESOURCE_PATH", "resource_path"))
                .Add(new Ros2SetEnv("AppendEnvironmentVariable", "GZ_SIM_RESOURCE_PATH", "resource_path"))
                .Add(new Ros2Include("os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')",
                    "{'gz_args': '-r empty.sdf'}.items()"))
                .Add(new Ros2Node("robot_state_publisher", "robot_state_publisher",
                    parameters: "[{'robot_description': robot_description, 'use_sim_time': True}]", output: "screen"))
                .Add(new Ros2Node("ros_gz_sim", "create",
                    arguments: "['-topic', 'robot_description', '-name', '" + modelName + "']", output: "screen"))
                .Add(new Ros2Node("ros_gz_bridge", "parameter_bridge",
                    arguments: "['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']", output: "screen"))
                .Write(savePath);
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

    // Indentation-tracking writer for the Python launch files, the ROS 2 analogue of
    // the XmlWriter the ROS 1 LaunchElement classes write through.
    internal sealed class Ros2LaunchWriter
    {
        private readonly StreamWriter file;
        private int indent;

        public Ros2LaunchWriter(StreamWriter file)
        {
            this.file = file;
        }

        public void Line(string text = "")
        {
            file.WriteLine(text.Length == 0 ? "" : new string(' ', indent * 4) + text);
        }

        public void Indent()
        {
            indent++;
        }

        public void Outdent()
        {
            indent--;
        }
    }

    // One entry in a generate_launch_description() list, mirroring ROS 1's LaunchElement.
    public abstract class Ros2LaunchElement
    {
        internal abstract void WriteFile(Ros2LaunchWriter writer);
    }

    // A '# ...' comment line (or block) inside the LaunchDescription list.
    public class Ros2Comment : Ros2LaunchElement
    {
        private readonly string[] lines;

        public Ros2Comment(params string[] lines)
        {
            this.lines = lines;
        }

        internal override void WriteFile(Ros2LaunchWriter writer)
        {
            foreach (string line in lines)
            {
                writer.Line("# " + line);
            }
        }
    }

    // DeclareLaunchArgument(name=..., default_value=...). The default value is raw
    // Python (a variable like default_model, not a quoted literal).
    public class Ros2DeclareArg : Ros2LaunchElement
    {
        private readonly string argName;
        private readonly string argDefault;

        public Ros2DeclareArg(string name, string def)
        {
            argName = name;
            argDefault = def;
        }

        internal override void WriteFile(Ros2LaunchWriter writer)
        {
            writer.Line("DeclareLaunchArgument(name='" + argName + "', default_value=" + argDefault + "),");
        }
    }

    // Set/AppendEnvironmentVariable(name, value). value is raw Python.
    public class Ros2SetEnv : Ros2LaunchElement
    {
        private readonly string action;
        private readonly string varName;
        private readonly string varValue;

        public Ros2SetEnv(string action, string name, string value)
        {
            this.action = action;
            varName = name;
            varValue = value;
        }

        internal override void WriteFile(Ros2LaunchWriter writer)
        {
            writer.Line(action + "('" + varName + "', " + varValue + "),");
        }
    }

    // IncludeLaunchDescription(PythonLaunchDescriptionSource(source), launch_arguments=...).
    // source and launchArguments are raw Python; launchArguments is optional.
    public class Ros2Include : Ros2LaunchElement
    {
        private readonly string source;
        private readonly string launchArguments;

        public Ros2Include(string source, string launchArguments = null)
        {
            this.source = source;
            this.launchArguments = launchArguments;
        }

        internal override void WriteFile(Ros2LaunchWriter writer)
        {
            writer.Line("IncludeLaunchDescription(");
            writer.Indent();
            writer.Line("PythonLaunchDescriptionSource(" + source + "),");
            if (launchArguments != null)
            {
                writer.Line("launch_arguments=" + launchArguments + ",");
            }
            writer.Outdent();
            writer.Line("),");
        }
    }

    // launch_ros Node(...). parameters and arguments are raw Python lists; each
    // optional field is omitted when null, matching ROS 1's LaunchNode.
    public class Ros2Node : Ros2LaunchElement
    {
        private readonly string package;
        private readonly string executable;
        private readonly string output;
        private readonly string parameters;
        private readonly string arguments;

        public Ros2Node(string package, string executable,
            string parameters = null, string arguments = null, string output = null)
        {
            this.package = package;
            this.executable = executable;
            this.parameters = parameters;
            this.arguments = arguments;
            this.output = output;
        }

        internal override void WriteFile(Ros2LaunchWriter writer)
        {
            writer.Line("Node(");
            writer.Indent();
            writer.Line("package='" + package + "',");
            writer.Line("executable='" + executable + "',");
            if (output != null)
            {
                writer.Line("output='" + output + "',");
            }
            if (parameters != null)
            {
                writer.Line("parameters=" + parameters + ",");
            }
            if (arguments != null)
            {
                writer.Line("arguments=" + arguments + ",");
            }
            writer.Outdent();
            writer.Line("),");
        }
    }

    // Assembles a .launch.py: import lines, a setup block inside
    // generate_launch_description(), and the returned LaunchDescription([...]) elements.
    public class Ros2LaunchFile
    {
        private readonly List<string> imports = new List<string>();
        private readonly List<string> setup = new List<string>();
        private readonly List<Ros2LaunchElement> elements = new List<Ros2LaunchElement>();

        public Ros2LaunchFile Import(params string[] lines)
        {
            imports.AddRange(lines);
            return this;
        }

        public Ros2LaunchFile Setup(params string[] lines)
        {
            setup.AddRange(lines);
            return this;
        }

        public Ros2LaunchFile Add(Ros2LaunchElement element)
        {
            elements.Add(element);
            return this;
        }

        public void Write(string savePath)
        {
            using (StreamWriter file = new StreamWriter(savePath))
            {
                Ros2LaunchWriter writer = new Ros2LaunchWriter(file);
                foreach (string line in imports)
                {
                    writer.Line(line);
                }
                writer.Line();
                writer.Line();
                writer.Line("def generate_launch_description():");
                writer.Indent();
                foreach (string line in setup)
                {
                    writer.Line(line);
                }
                if (setup.Count > 0)
                {
                    writer.Line();
                }
                writer.Line("return LaunchDescription([");
                writer.Indent();
                foreach (Ros2LaunchElement element in elements)
                {
                    element.WriteFile(writer);
                }
                writer.Outdent();
                writer.Line("])");
                writer.Outdent();
            }
        }
    }
}
