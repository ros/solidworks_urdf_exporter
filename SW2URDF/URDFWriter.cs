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
using System.Text;

namespace SW2URDF
{
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
    public class URDFElement
    {
        public URDFElement() { }
        public void writeURDF(XmlWriter writer)
        {

        }

    }
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
    public class link : URDFElement
    {
        public List<link> Children
        { get; set; }
        public string name
        { get; set; }
        public string uniqueName
        { get; set; }
        public inertial Inertial
        { get; set; }
        public visual Visual
        { get; set; }
        public collision Collision
        { get; set; }
        public gazebo Gazebo
        { get; set; }
        public joint Joint
        { get; set; }
        public bool STLQualityFine
        { get; set; }

        // The SW part component object
        public IComponent2 SWComponent
        { get; set; }
        public IComponent2 SWMainComponent
        { get; set; }
        public List<IComponent2> SWcomponents
        { get; set; }

        // The distance from the SW root assembly
        public int SWComponentLevel
        { get; set; }
        public link()
        {
            Children = new List<link>();
            Inertial = new inertial();
            Visual = new visual();
            Collision = new collision();
            SWcomponents = new List<IComponent2>();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            writer.WriteAttributeString("name", uniqueName);

            Inertial.writeURDF( writer);
            Visual.writeURDF( writer);
            Collision.writeURDF( writer);

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

    public class inertial : URDFElement
    {
        public origin Origin
        { get; set; }
        public mass Mass
        { get; set; }
        public inertia Inertia
        { get; set; }

        public inertial()
        {
            Origin = new origin();
            Mass = new mass();
            Inertia = new inertia();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("inertial");

            Origin.writeURDF( writer);
            Mass.writeURDF( writer);
            Inertia.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

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

    public class mass : URDFElement
    {
        public double Value
        { get; set; }
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

    public class visual : URDFElement
    {
        public origin Origin
        { get; set; }
        public geometry Geometry
        { get; set; }
        public material Material
        { get; set; }
        public visual()
        {
            Origin = new origin();
            Geometry = new geometry();
            Material = new material();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("visual");

            Origin.writeURDF( writer);
            Geometry.writeURDF( writer);
            Material.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class geometry : URDFElement
    {
        public mesh Mesh
        { get; set; }
        public geometry()
        {
            Mesh = new mesh();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("geometry");

            Mesh.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class mesh : URDFElement
    {
        public string filename
        { get; set; }
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

    public class resolution : URDFElement
    {
        public resolution()
        {
        }
        new public void writeURDF(XmlWriter writer) { }
    }

    public class material : URDFElement
    {
        public color Color
        { get; set; }
        public texture Texture
        { get; set; }
        public string name
        { get; set; }
        public material()
        {
            Color = new color();
            Texture = new texture();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("material");
            writer.WriteAttributeString("name", name);

            Color.writeURDF( writer);
            Texture.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

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

    public class texture : URDFElement
    {
        public string filename
        { get; set; }
        public string wFilename
        { get; set; }
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

            Origin.writeURDF( writer);
            Geometry.writeURDF( writer);

            writer.WriteEndElement();
        }
    }




    public class joint : URDFElement
    {
        public string name
        { get; set; }
        public string type
        { get; set; }
        public origin Origin
        { get; set; }
        public parent_link Parent
        { get; set; }
        public child_link Child
        { get; set; }
        public axis Axis
        { get; set;}
        public limit Limit
        { get; set; }
        public calibration Calibration
        { get; set; }
        public dynamics Dynamics
        { get; set; }
        public safety_controller Safety
        { get; set; }
        public string CoordinateSystemName
        { get; set; }
        public string AxisName
        { get; set; }

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

    public class parent_link : URDFElement
    {
        public string name
        { get; set; }
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

    public class limit : URDFElement
    {
        public double lower
        { get; set; }
        public double upper
        { get; set; }
        public double effort
        { get; set; }
        public double velocity
        { get; set; }
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

    public class calibration : URDFElement
    {
        public double rising
        { get; set; }
        public double falling
        { get; set; }

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

    public class dynamics : URDFElement
    {
        public double damping
        { get; set; }
        public double friction
        { get; set; }

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

    public class safety_controller : URDFElement
    {
        public double soft_lower
        { get; set; }
        public double soft_upper
        { get; set; }
        public double k_position
        { get; set; }
        public double k_velocity
        { get; set; }
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


    public class gazebo : URDFElement
    {
        public string reference
        { get; set; }
        public gazebo()
        {
        }
        new public void writeURDF(XmlWriter writer)
        {
        }
    }

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

    public class manifestElement
    {

        public manifestElement() {}
        public void writeElement(){}
    }
    public class manifest : manifestElement
    {
        public description Description
        { get; set;}
        public depend[] Depends
        { get; set; }
        public author Author
        { get; set; }
        public license License
        { get; set; }

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
    public class description : manifestElement
    {
        public string brief
        { get; set; }
        public string longDescription
        { get; set; }
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
    public class depend : manifestElement
    {
        public string package
        { get; set; }
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
    public class author : manifestElement
    {
        public string name
        { get; set; }
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
    public class license : manifestElement
    {
        public string lic
        { get; set; }
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
}
