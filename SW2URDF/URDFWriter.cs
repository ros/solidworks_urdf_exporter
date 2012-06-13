using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;

namespace SW2URDF
{
    public class URDFWriter
    {
        public XmlWriter writer;
        public URDFWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
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
        private List<link> mLinks;
        private List<joint> mJoints;
        public string name
        { get; set; }

        public robot()
        {
            mLinks = new List<link>();
            mJoints = new List<joint>();
        }
        new public void writeURDF(XmlWriter writer)
        {            
            writer.WriteStartDocument();    
            writer.WriteStartElement("robot");
            writer.WriteAttributeString("name", name);

            foreach (link Link in mLinks)
            {
                Link.writeURDF(writer);
            }

            foreach (joint Joint in mJoints)
            {
                Joint.writeURDF(writer);
            }

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
        public void addLink(link Link)
        {
            mLinks.Add(Link);
        }
        public void addJoint(joint Joint)
        {
            mJoints.Add(Joint);
        }
    }
    public class link : URDFElement
    {
        public string name
        { get; set; }
        public inertial Inertial
        { get; set; }
        public visual Visual
        { get; set; }
        public collision Collision
        { get; set; }
        public gazebo Gazebo
        { get; set; }
        public link()
        {
            Inertial = new inertial();
            Visual = new visual();
            Collision = new collision();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            writer.WriteAttributeString("name", "link_" + name);

            Inertial.writeURDF( writer);
            Visual.writeURDF( writer);
            Collision.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class inertial : URDFElement
    {
        public origin_inertial Origin
        { get; set; }
        public mass Mass
        { get; set; }
        public inertia Inertia
        { get; set; }

        public inertial()
        {
            Origin = new origin_inertial();
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

    public class origin_inertial : URDFElement
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
        public origin_inertial()
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
        public origin_visual Origin
        { get; set; }
        public geometry_visual Geometry
        { get; set; }
        public material Material
        { get; set; }
        public visual()
        {
            Origin = new origin_visual();
            Geometry = new geometry_visual();
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

    public class origin_visual : URDFElement
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
        public origin_visual()
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

    public class geometry_visual : URDFElement
    {
        public mesh_visual Mesh
        { get; set; }
        public geometry_visual()
        {
            Mesh = new mesh_visual();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("geometry");

            Mesh.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class mesh_visual : URDFElement
    {
        public string filename
        { get; set; }
        public mesh_visual()
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
        public texture()
        {
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("texture");
            writer.WriteAttributeString("filename", filename);
            writer.WriteEndElement();
        }
    }

    public class collision : URDFElement
    {
        public origin_collision Origin
        { get; set; }
        public geometry_collision Geometry
        { get; set; }
        public collision()
        {
            Origin = new origin_collision();
            Geometry = new geometry_collision();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("collision");

            Origin.writeURDF( writer);
            Geometry.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class origin_collision : URDFElement
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
        public origin_collision()
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

    public class geometry_collision : URDFElement
    {
        public mesh_collision Mesh
        { get; set; }
        public geometry_collision()
        {
            Mesh = new mesh_collision();
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("geometry");

            Mesh.writeURDF( writer);

            writer.WriteEndElement();
        }
    }

    public class mesh_collision : URDFElement
    {
        public string filename
        { get; set; }
        public mesh_collision()
        {
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("mesh");
            writer.WriteAttributeString("filename", filename);
            writer.WriteEndElement(); //mesh
        }
    }



    public class joint : URDFElement
    {
        public joint()
        {
        }
        new public void writeURDF(XmlWriter writer)
        {

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
}
