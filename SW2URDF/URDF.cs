using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections;
using System.Reflection;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;
using SolidWorksTools;
using SolidWorksTools.File;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;
using System.Text;

namespace SW2URDF
{
    //Initiates the XMLWriter and its necessary settings
    public class URDFWriter
    {
        public XmlWriter writer;
        public URDFWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = true;
            writer = XmlWriter.Create(savePath, settings);

        }
    }

    //Not sure why I have a class that everything else inherits from that is empty. But maybe we'll want to add things to it
    public class URDFElement
    {
        public URDFElement() { }
        public void writeURDF(XmlWriter writer)
        {
        }
    }

    //The base URDF element, a robot
    public class robot : URDFElement
    {
        public link BaseLink
        { get; set; }
        public string name
        { get; set; }

        public robot()
        {
            BaseLink = new link();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartDocument();
            writer.WriteStartElement("robot");
            writer.WriteAttributeString("name", name);

            BaseLink.writeURDF(writer);

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
    }

    //The link class, it contains many other elements not found in the URDF.
    public class link : URDFElement
    {
        public List<link> Children;
        public string name;
        public string uniqueName;
        public inertial Inertial;
        public visual Visual;
        public collision Collision;
        public joint Joint;
        public bool STLQualityFine;
        public bool isIncomplete;

        // The SW part component object
        public Component2 SWComponent;
        public Component2 SWMainComponent;
        public List<Component2> SWcomponents;
        public List<byte[]> SWComponentPIDs;
        public byte[] SWMainComponentPID;

        public link()
        {
            Children = new List<link>();
            Inertial = new inertial();
            Visual = new visual();
            Collision = new collision();
            SWcomponents = new List<Component2>();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            writer.WriteAttributeString("name", uniqueName);

            Inertial.writeURDF(writer);
            Visual.writeURDF(writer);
            Collision.writeURDF(writer);

            writer.WriteEndElement();
            if (Joint != null && Joint.name != "")
            {
                Joint.writeURDF(writer);
            }

            foreach (link child in Children)
            {
                child.writeURDF(writer);
            }
        }
    }

    //The serial node class, it is used only for saving the configuration.
    public class SerialNode
    {
        public string linkName;
        public string jointName;
        public string axisName;
        public string coordsysName;
        public List<byte[]> componentPIDs;
        public string jointType;
        public bool isBaseNode;
        public bool isIncomplete;
        public List<SerialNode> Nodes;

        public SerialNode()
        {
            Nodes = new List<SerialNode>();
        }
    }


    //The inertial element of a link
    public class inertial : URDFElement
    {
        public origin Origin;
        public mass Mass;
        public inertia Inertia;

        public inertial()
        {
            Origin = new origin();
            Mass = new mass();
            Inertia = new inertia();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("inertial");

            Origin.writeURDF(writer);
            Mass.writeURDF(writer);
            Inertia.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //The Origin element, used in several other elements
    public class origin : URDFElement
    {
        private double[] xyz;
        public double[] XYZ
        {
            get
            {
                return xyz;
            }
            set
            {
                xyz = value;
            }
        }
        public double X
        {
            get
            {
                return xyz[0];
            }
            set
            {
                xyz[0] = value;
            }
        }
        public double Y
        {
            get
            {
                return xyz[1];
            }
            set
            {
                xyz[1] = value;
            }
        }
        public double Z
        {
            get
            {
                return xyz[2];
            }
            set
            {
                xyz[2] = value;
            }
        }

        private double[] rpy;
        public double[] RPY
        {
            get
            {
                return rpy;
            }
            set
            {
                rpy = value;
            }
        }
        public double Roll
        {
            get
            {
                return rpy[0];
            }
            set
            {
                rpy[0] = value;
            }
        }
        public double Pitch
        {
            get
            {
                return rpy[1];
            }
            set
            {
                rpy[1] = value;
            }
        }
        public double Yaw
        {
            get
            {
                return rpy[2];
            }
            set
            {
                rpy[2] = value;
            }
        }
        public origin()
        {
            xyz = new double[3] { 0, 0, 0 };
            rpy = new double[3] { 0, 0, 0 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("origin");
            writer.WriteAttributeString("xyz", X.ToString() + " " + Y.ToString() + " " + Z.ToString());
            writer.WriteAttributeString("rpy", Roll.ToString() + " " + Pitch.ToString() + " " + Yaw.ToString());
            writer.WriteEndElement();
        }
    }

    //mass element, belongs to the inertial element
    public class mass : URDFElement
    {
        public double Value;
        public mass()
        {
            Value = 0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("mass");
            writer.WriteAttributeString("value", Value.ToString());
            writer.WriteEndElement();
        }
    }

    //Inertia element, which means moment of inertia. In the inertial element
    public class inertia : URDFElement
    {
        private double[] moment;
        public double[] Moment
        {
            get
            {
                return moment;
            }
            set
            {
                moment = value;
            }
        }
        public double Ixx
        {
            get
            {
                return moment[0];
            }
            set
            {
                moment[0] = value;
            }
        }
        public double Ixy
        {
            get
            {
                return moment[1];
            }
            set
            {
                moment[1] = value;
            }
        }
        public double Ixz
        {
            get
            {
                return moment[2];
            }
            set
            {
                moment[2] = value;
            }
        }
        public double Iyx
        {
            get
            {
                return moment[3];
            }
            set
            {
                moment[3] = value;
            }
        }
        public double Iyy
        {
            get
            {
                return moment[4];
            }
            set
            {
                moment[4] = value;
            }
        }
        public double Iyz
        {
            get
            {
                return moment[5];
            }
            set
            {
                moment[5] = value;
            }
        }
        public double Izx
        {
            get
            {
                return moment[6];
            }
            set
            {
                moment[6] = value;
            }
        }
        public double Izy
        {
            get
            {
                return moment[7];
            }
            set
            {
                moment[7] = value;
            }
        }
        public double Izz
        {
            get
            {
                return moment[8];
            }
            set
            {
                moment[8] = value;
            }
        }
        public inertia()
        {
            moment = new double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("inertia");
            writer.WriteAttributeString("ixx", Ixx.ToString());
            writer.WriteAttributeString("ixy", Ixy.ToString());
            writer.WriteAttributeString("ixz", Ixz.ToString());
            writer.WriteAttributeString("iyy", Iyy.ToString());
            writer.WriteAttributeString("iyz", Iyz.ToString());
            writer.WriteAttributeString("izz", Izz.ToString());
            writer.WriteEndElement();
        }
    }

    //The visual element of a link
    public class visual : URDFElement
    {
        public origin Origin;
        public geometry Geometry;
        public material Material;
        public visual()
        {
            Origin = new origin();
            Geometry = new geometry();
            Material = new material();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("visual");

            Origin.writeURDF(writer);
            Geometry.writeURDF(writer);
            Material.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //The geometry element the visual element
    public class geometry : URDFElement
    {
        public mesh Mesh;
        public geometry()
        {
            Mesh = new mesh();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("geometry");

            Mesh.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //The mesh element of the geometry element. This contains only a filename location of the mesh.
    public class mesh : URDFElement
    {
        public string filename;
        public mesh()
        {
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("mesh");
            writer.WriteAttributeString("filename", filename);
            writer.WriteEndElement(); //mesh
        }
    }

    //The material element of the visual element. 
    public class material : URDFElement
    {
        public color Color;
        public texture Texture;
        public string name;
        public material()
        {
            Color = new color();
            Texture = new texture();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("material");
            writer.WriteAttributeString("name", name);

            Color.writeURDF(writer);
            Texture.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //The color element of the material element. Contains a single RGBA.
    public class color : URDFElement
    {
        private double[] rgba;
        public double[] RGBA
        {
            get
            {
                return rgba;
            }
            set
            {
                rgba = value;
            }
        }
        public double Red
        {
            get
            {
                return rgba[0];
            }
            set
            {
                rgba[0] = value;
            }
        }
        public double Green
        {
            get
            {
                return rgba[1];
            }
            set
            {
                rgba[1] = value;
            }
        }
        public double Blue
        {
            get
            {
                return rgba[2];
            }
            set
            {
                rgba[2] = value;
            }
        }
        public double Alpha
        {
            get
            {
                return rgba[3];
            }
            set
            {
                rgba[3] = value;
            }
        }
        public color()
        {
            rgba = new double[4] { 1, 1, 1, 1 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("color");
            writer.WriteAttributeString("rgba", Red.ToString() + " " + Green.ToString() + " " + Blue.ToString() + " " + Alpha.ToString());
            writer.WriteEndElement();
        }
    }

    //The texture element of the material element. 
    public class texture : URDFElement
    {
        public string filename;
        public string wFilename;
        public texture()
        {
            wFilename = "";
            filename = "";
        }
        new public void writeURDF(XmlWriter writer)
        {
            if (wFilename != "")
            {
                writer.WriteStartElement("texture");
                writer.WriteAttributeString("filename", filename);
                writer.WriteEndElement();
            }
        }
    }

    //The collision element of a link.
    public class collision : URDFElement
    {
        public origin Origin
        { get; set; }
        public geometry Geometry
        { get; set; }
        public collision()
        {
            Origin = new origin();
            Geometry = new geometry();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("collision");

            Origin.writeURDF(writer);
            Geometry.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //The joint class. There is one for every link but the base link
    public class joint : URDFElement
    {
        public string name;
        public string type;
        public origin Origin;
        public parent_link Parent;
        public child_link Child;
        public axis Axis;
        public limit Limit;
        public calibration Calibration;
        public dynamics Dynamics;
        public safety_controller Safety;
        public string CoordinateSystemName;
        public string AxisName;

        public joint()
        {
            Origin = new origin();
            Parent = new parent_link();
            Child = new child_link();
            Axis = new axis();
            Limit = new limit();
            Calibration = new calibration();
            Dynamics = new dynamics();
            Safety = new safety_controller();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("joint");
            writer.WriteAttributeString("name", "joint_" + name);
            writer.WriteAttributeString("type", type);

            Origin.writeURDF(writer);
            Parent.writeURDF(writer);
            Child.writeURDF(writer);
            Axis.writeURDF(writer);
            Limit.writeURDF(writer);
            Calibration.writeURDF(writer);
            Dynamics.writeURDF(writer);
            Safety.writeURDF(writer);

            writer.WriteEndElement();
        }
    }

    //parent_link element of a joint. 
    public class parent_link : URDFElement
    {
        public string name;
        public parent_link()
        {
            name = "";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("parent");
            writer.WriteAttributeString("link", name);
            writer.WriteEndElement();

        }
    }

    //The child link element
    public class child_link : URDFElement
    {
        public string name
        { get; set; }
        public child_link()
        {
            name = "";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("child");
            writer.WriteAttributeString("link", name);
            writer.WriteEndElement();
        }
    }

    //The axis element of a joint.
    public class axis : URDFElement
    {
        private double[] xyz;
        public double[] XYZ
        {
            get
            {
                return xyz;
            }
            set
            {
                xyz = value;
            }
        }
        public double X
        {
            get
            {
                return xyz[0];
            }
            set
            {
                xyz[0] = value;
            }
        }
        public double Y
        {
            get
            {
                return xyz[1];
            }
            set
            {
                xyz[1] = value;
            }
        }
        public double Z
        {
            get
            {
                return xyz[2];
            }
            set
            {
                xyz[2] = value;
            }
        }
        public axis()
        {
            xyz = new double[] { 0, 0, 0 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("axis");
            writer.WriteAttributeString("xyz", X.ToString() + " " + Y.ToString() + " " + Z.ToString());
            writer.WriteEndElement();
        }
    }

    //The limit element of a joint.
    public class limit : URDFElement
    {
        public double lower;
        public double upper;
        public double effort;
        public double velocity;
        public limit()
        {
            lower = 0; upper = 0; effort = 0; velocity = 0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("limit");
            writer.WriteAttributeString("lower", lower.ToString());
            writer.WriteAttributeString("upper", upper.ToString());
            writer.WriteAttributeString("effort", effort.ToString());
            writer.WriteAttributeString("velocity", velocity.ToString());
            writer.WriteEndElement();
        }
    }

    //The calibration element of a joint.
    public class calibration : URDFElement
    {
        public double rising;
        public double falling;

        public calibration()
        {
            rising = 0; falling = 0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("calibration");
            writer.WriteAttributeString("rising", rising.ToString());
            writer.WriteAttributeString("falling", falling.ToString());
            writer.WriteEndElement();
        }
    }

    //The dynamics element of a joint.
    public class dynamics : URDFElement
    {
        public double damping;
        public double friction;

        public dynamics()
        {
            damping = 0; friction = 0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("dynamics");
            writer.WriteAttributeString("damping", damping.ToString());
            writer.WriteAttributeString("friction", friction.ToString());
            writer.WriteEndElement();
        }
    }

    //The safety_controller element of a joint.
    public class safety_controller : URDFElement
    {
        public double soft_lower;
        public double soft_upper;
        public double k_position;
        public double k_velocity;
        public safety_controller()
        {
            soft_lower = 0; soft_upper = 0; k_position = 0; k_velocity = 0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("safety_controller");
            writer.WriteAttributeString("soft_lower_limit", soft_lower.ToString());
            writer.WriteAttributeString("soft_upper_limit", soft_upper.ToString());
            writer.WriteAttributeString("k_position", k_position.ToString());
            writer.WriteAttributeString("k_velocity", k_velocity.ToString());
            writer.WriteEndElement();
        }
    }
    
    //A class that just writes the bare minimum of the manifest file necessary for ROS packages.
    public class manifestWriter
    {
        public XmlWriter writer;
        public manifestWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = false;
            writer = XmlWriter.Create(savePath, settings);
        }
    }

    //The base class for manifest elements. Again, I guess I like having empty base classes
    public class manifestElement
    {
        public manifestElement() { }
        public void writeElement() { }
    }

    //Top level class for the manifest file.
    public class manifest : manifestElement
    {
        public description Description;
        public depend[] Depends;
        public author Author;
        public license License;

        public manifest(string name)
        {
            Description = new description();
            Description.brief = name;
            Description.longDescription = name;

            Depends = new depend[1];
            depend dep = new depend();
            dep.package = "gazebo";
            Depends[0] = dep;

            Author = new author();
            Author.name = "me";

            License = new license();
            License.lic = "BSD";
        }
        public void writeElement(manifestWriter mWriter)
        {
            XmlWriter writer = mWriter.writer;
            writer.WriteStartDocument();
            writer.WriteStartElement("package");

            Description.writeElement(writer);
            foreach (depend dep in Depends)
            {
                dep.writeElement(writer);
            }

            Author.writeElement(writer);
            License.writeElement(writer);

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
    }

    //description element of the manifest file
    public class description : manifestElement
    {
        public string brief;
        public string longDescription;
        public description()
        {
            brief = "";
            longDescription = "";
        }
        public void writeElement(XmlWriter writer)
        {
            writer.WriteStartElement("description");
            writer.WriteAttributeString("brief", brief);
            writer.WriteString(longDescription);
            writer.WriteEndElement();
        }
    }

    //The depend element of the manifest file
    public class depend : manifestElement
    {
        public string package;
        public depend()
        {
            package = "";
        }
        public void writeElement(XmlWriter writer)
        {
            writer.WriteStartElement("depend");
            writer.WriteAttributeString("package", package);
            writer.WriteEndElement();
        }
    }

    //The author element of the manifest file
    public class author : manifestElement
    {
        public string name;
        public author()
        {
            name = "";
        }
        public void writeElement(XmlWriter writer)
        {
            writer.WriteStartElement("author");
            writer.WriteString(name);
            writer.WriteEndElement();
        }
    }

    //The license element of the manifest file
    public class license : manifestElement
    {
        public string lic;
        public license()
        {
            lic = "";
        }
        public void writeElement(XmlWriter writer)
        {
            writer.WriteStartElement("license");
            writer.WriteString(lic);
            writer.WriteEndElement();
        }
    }


    #region Windows Forms Derived classes

    //A LinkNode is derived from a TreeView TreeNode. I've added many new fields to it so that information can be passed around
    //from the TreeView itself.
    public class LinkNode : TreeNode
    {
        public link Link
        { get; set; }
        public string linkName
        { get; set; }
        public string jointName
        { get; set; }
        public string axisName
        { get; set; }
        public string coordsysName
        { get; set; }
        public List<Component2> Components
        { get; set; }
        public List<byte[]> ComponentPIDs
        { get; set; }
        public string jointType
        { get; set; }
        public bool isBaseNode
        { get; set; }
        public bool isIncomplete
        { get; set; }
        public bool needsSaving
        { get; set; }

    }
    #endregion
}
