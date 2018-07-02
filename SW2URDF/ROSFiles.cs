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
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;

namespace SW2URDF
{
    class ROSFiles
    {
    }



    public abstract class launchElement
    {
        public abstract void writeFile(XmlWriter writer);
    }

    public class launchComment : launchElement
    {
        string commentString;
        public launchComment(string comment)
        {
            this.commentString = comment;
        }

        public override void writeFile(XmlWriter writer)
        {
            writer.WriteComment(this.commentString);
        }
    }

    public class launchArg : launchElement
    {
        string argName;
        string argDefault;

        public launchArg(string name, string def = "")
        {
            this.argName = name;
            this.argDefault = def;
        }

        public override void writeFile(XmlWriter writer)
        {
            writer.WriteStartElement("arg");
            writer.WriteAttributeString("name", this.argName);
            if (this.argDefault.Length > 0)
            {
                writer.WriteAttributeString("default", this.argDefault);
            }
            writer.WriteEndElement();
        }
    }

    public class launchParam : launchElement
    {
        string paramName;
        string paramValue;
        string paramTextFile;

        public launchParam(string name, string value = "", string textfile = "")
        {
            this.paramName = name;
            this.paramValue = value;
            this.paramTextFile = textfile;
        }

        public override void writeFile(XmlWriter writer)
        {
            writer.WriteStartElement("param");
            writer.WriteAttributeString("name", this.paramName);

            if (this.paramValue.Length > 0)
            {
                writer.WriteAttributeString("value", this.paramValue);
            }

            if (this.paramTextFile.Length > 0)
            {
                writer.WriteAttributeString("textfile", this.paramTextFile);
            }
            writer.WriteEndElement();
        }
    }

    public class launchNode : launchElement
    {
        string nodeName;
        string nodePkg;
        string nodeType;
        string nodeArgs;
        string nodeOutput;
        bool nodeRespawn;
        public launchNode(string name, string pkg, string type, string args="", string output="", bool respawn=false)
        {
            this.nodeName = name;
            this.nodePkg = pkg;
            this.nodeType = type;
            this.nodeArgs = args;
            this.nodeOutput = output;
            this.nodeRespawn = respawn;
        }

        public override void writeFile(XmlWriter writer)
        {
            writer.WriteStartElement("node");
            writer.WriteAttributeString("name", this.nodeName);
            writer.WriteAttributeString("pkg", this.nodePkg);
            writer.WriteAttributeString("type", this.nodeType);

            if (this.nodeArgs.Length != 0)
            {
                writer.WriteAttributeString("args", this.nodeArgs);
            }

            if (this.nodeOutput.Length != 0)
            {
                writer.WriteAttributeString("output", this.nodeOutput);
            }

            if (this.nodeRespawn)
            {
                writer.WriteAttributeString("respawn", "True");
            }

            writer.WriteEndElement();
        }
    }

    public class launchInclude : launchElement
    {
        string includeFile;
        public launchInclude(string file)
        {
            this.includeFile = file;
        }

        public override void writeFile(XmlWriter writer)
        {
            writer.WriteStartElement("include");
            writer.WriteAttributeString("file", this.includeFile);
            writer.WriteEndElement();
        }
    }


    public class Gazebo
    {
        string package;
        string robotURDF;
        string model;
        List<launchElement> elements;
        
        public Gazebo(string modelName, string packageName, string URDFName)
        {
            this.elements = new List<launchElement>();
            this.model = modelName;
            this.package = packageName;
            this.robotURDF = URDFName;

            this.elements.Add(new launchInclude("$(find gazebo_ros)/launch/empty_world.launch"));
            this.elements.Add(new launchNode("tf_footprint_base", "tf", "static_transform_publisher", "0 0 0 0 0 0 base_link base_footprint 40"));
            this.elements.Add(new launchNode("spawn_model", "gazebo_ros", "spawn_model", "-file $(find " + this.package + ")/urdf/" + this.robotURDF + " -urdf -model " + this.model, "screen"));
            this.elements.Add(new launchNode("fake_joint_calibration", "rostopic", "rostopic", "pub /calibrated std_msgs/Bool true"));
        }

        public void writeFile(string dir)
        {
            XmlWriter writer;
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = true;
            string displayLaunch = dir + @"gazebo.launch";
            writer = XmlWriter.Create(displayLaunch, settings);

            writer.WriteStartDocument();
            writer.WriteStartElement("launch");

            foreach (launchElement element in this.elements)
            {
                element.writeFile(writer);
            }

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
    }

    public class Rviz
    {
        string package;
        string robotURDF;
        List<launchElement> elements;
        public Rviz(string packageName, string URDFName)
        {
            package = packageName;
            robotURDF = URDFName;

            this.elements = new List<launchElement>();

            this.elements.Add(new launchArg("model"));
            this.elements.Add(new launchArg("gui", "False"));

            this.elements.Add(new launchParam("robot_description", "", "$(find " + package + ")/urdf/" + robotURDF));
            this.elements.Add(new launchParam("use_gui", "$(arg gui)"));

            this.elements.Add(new launchNode("joint_state_publisher", "joint_state_publisher", "joint_state_publisher"));
            this.elements.Add(new launchNode("robot_state_publisher", "robot_state_publisher", "state_publisher"));
            this.elements.Add(new launchNode("rviz", "rviz", "rviz", "-d $(find " + this.package + ")/urdf.rviz"));
        }

        public void writeFiles(string dir)
        {
            XmlWriter writer;
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = true;
            string displayLaunch = dir + @"display.launch";
            writer = XmlWriter.Create(displayLaunch, settings);

            writer.WriteStartDocument();
            writer.WriteStartElement("launch");

            foreach (launchElement element in this.elements)
            {
                element.writeFile(writer);
            }

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }


    }


}
