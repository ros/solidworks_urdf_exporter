using System;
using System.Globalization;

using SolidWorks.Interop.sldworks;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Xml;
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

        protected bool isRequired;
    }

    public class Attribute
    {
        string USStringFormat = "en-US";
        public bool isRequired;
        public string type;
        public object value;

        public Attribute() 
        {
            type = "";
            isRequired = false;
        }
        public void writeURDF(XmlWriter writer)
        {
            string value_string = "";
            if (value.GetType() == typeof(double[]))
            {
                double[] value_array = (double[])value;
                foreach (double d in value_array)
                {
                    value_string += d.ToString(CultureInfo.CreateSpecificCulture(USStringFormat)) + " ";
                }
                value_string = value_string.Trim();
            }
            else if (value.GetType() == typeof(double))
            {
                value_string = ((Double)value).ToString(CultureInfo.CreateSpecificCulture(USStringFormat));
            }
            else if (value.GetType() == typeof(string))
            {
                value_string = (string)value;
            }

            else if (value != null) 
            {
                throw new Exception("Unhandled object type in write attribute");
                return;
            }
            if (isRequired && value == null)
            {
                throw new Exception("Required attribute has null value");
            }
            if (type == "")
            {
                throw new Exception("No type specified");
                return;
            }
            if (value != null)
            {
                writer.WriteAttributeString(type, value_string);
            }
        }

    }

    //The base URDF element, a robot
    public class robot : URDFElement
    {
        public link BaseLink;
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }


        public robot()
        {
            BaseLink = new link();
            isRequired = true;
            Name = new Attribute();
            Name.isRequired = true;
            Name.type = "name";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartDocument();
            writer.WriteStartElement("robot");
            Name.writeURDF(writer);

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
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }

        public inertial Inertial;
        public visual Visual;
        public collision Collision;
        public joint Joint;
        public bool STLQualityFine;
        public bool isIncomplete;
        public bool isFixedFrame;
        public string CoordSysName;

        // The SW part component object
        public Component2 SWComponent;
        public Component2 SWMainComponent;
        public List<Component2> SWcomponents;
        public List<byte[]> SWComponentPIDs;
        public byte[] SWMainComponentPID;

        public link()
        {
            Children = new List<link>();
            SWcomponents = new List<Component2>();
            Name = new Attribute();
            Name.isRequired = true;
            Name.type = "name";
            isRequired = true;
            isFixedFrame = true;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            Name.writeURDF(writer);

            if (Inertial != null)
            {
                Inertial.writeURDF(writer);
            }
            if (Visual != null)
            {
                Visual.writeURDF(writer);
            }
            if (Collision != null)
            {
                Collision.writeURDF(writer);
            }

            writer.WriteEndElement();
            if (Joint != null && Joint.name != null)
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

        //This is only used by the serialization module.
        public SerialNode()
        {
            Nodes = new List<SerialNode>();
        }

        public SerialNode(LinkNode node)
        {
            Nodes = new List<SerialNode>();
            if (node.Link == null)
            {
                linkName = node.linkName;
                jointName = node.jointName;
                axisName = node.axisName;
                coordsysName = node.coordsysName;
                componentPIDs = node.ComponentPIDs;
                jointType = node.jointType;
                isBaseNode = node.isBaseNode;
                isIncomplete = node.isIncomplete;
            }
            else
            {
                linkName = (string)node.Link.name;
                
                componentPIDs = node.ComponentPIDs;
                if (node.Link.Joint != null)
                {
                    jointName = (string)node.Link.Joint.name;

                    if (node.Link.Joint.Axis.X == 0 && node.Link.Joint.Axis.Y == 0 && node.Link.Joint.Axis.Z == 0)
                    {
                        axisName = "None";
                    }
                    else
                    {
                        axisName = node.Link.Joint.AxisName;
                    }
                    coordsysName = node.Link.Joint.CoordinateSystemName;
                    jointType = (string)node.Link.Joint.type;
                }
                else
                {
                    coordsysName = node.coordsysName;
                }

                isBaseNode = node.isBaseNode;
                isIncomplete = node.isIncomplete;
            }
            //Proceed recursively through the nodes
            foreach (LinkNode child in node.Nodes)
            {
                Nodes.Add(new SerialNode(child));
            }
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
        private Attribute XYZ;
        private Attribute RPY;
        
        public double[] xyz
        {
            get
            {
                return (double[])XYZ.value;
            }
            set
            {
                XYZ.value = value;
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

        public double[] rpy
        {
            get
            {
                return (double[])RPY.value;
            }
            set
            {
                RPY.value = value;
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

        public bool isCustomized;
        public origin()
        {
            isCustomized = false;
            XYZ = new Attribute();
            RPY = new Attribute();
            XYZ.type = "xyz";
            RPY.type = "rpy";            
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("origin");
            XYZ.writeURDF(writer);
            RPY.writeURDF(writer);
            writer.WriteEndElement();
        }
        public void fillBoxes(TextBox box_x, TextBox box_y, TextBox box_z, TextBox box_roll, TextBox box_pitch, TextBox box_yaw, string format)
        {
            if (XYZ.value != null)
            {
                box_x.Text = X.ToString(format);
                box_y.Text = Y.ToString(format);
                box_z.Text = Z.ToString(format);
            }
            else
            {
                box_x.Text = ""; box_y.Text = ""; box_z.Text = "";
            }

            if (RPY.value != null)
            {
                box_roll.Text = Roll.ToString(format);
                box_pitch.Text = Pitch.ToString(format);
                box_yaw.Text = Yaw.ToString(format);
            }
            else
            {
                box_roll.Text = ""; box_pitch.Text = ""; box_yaw.Text = "";
            }
        }

        public void update(TextBox box_x, TextBox box_y, TextBox box_z, TextBox box_roll, TextBox box_pitch, TextBox box_yaw)
        {
            double value;
            if (box_x.Text == "" && box_y.Text == "" && box_z.Text == "")
            {
                xyz = null;
            }
            else
            {
                X = (Double.TryParse(box_x.Text, out value)) ? value : 0;
                Y = (Double.TryParse(box_y.Text, out value)) ? value : 0;
                Z = (Double.TryParse(box_z.Text, out value)) ? value : 0;
            }

            if (box_roll.Text == "" && box_pitch.Text == "" && box_yaw.Text == "")
            {
                rpy = null;
            }
            else
            {
                Roll = (Double.TryParse(box_roll.Text, out value)) ? value : 0;
                Pitch = (Double.TryParse(box_pitch.Text, out value)) ? value : 0;
                Yaw = (Double.TryParse(box_yaw.Text, out value)) ? value : 0;

            }
        }
    }

    //mass element, belongs to the inertial element
    public class mass : URDFElement
    {
        private Attribute Value;
        public double value
        {
            get
            {
                return (double)Value.value;
            }
            set
            {
                Value.value = value;
            }
        }
        public mass()
        {
            Value = new Attribute();
            Value.type = "value";
            Value.isRequired = true;
            value = 0.0;
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("mass");
            Value.writeURDF(writer);
            writer.WriteEndElement();
        }
        public void fillBoxes(TextBox box, string format)
        {
            if (Value != null)
            {
                box.Text = value.ToString(format);
            }
            else
            {
                box.Text = "0";
            }
        }
        public void update(TextBox box)
        {
            double tmp;
            value = (Double.TryParse(box.Text, out tmp)) ? tmp : 0;
        }
    }

    //Inertia element, which means moment of inertia. In the inertial element
    public class inertia : URDFElement
    {
        private Attribute Ixx;
        public double ixx
        {
            get
            {
                return (double)Ixx.value;
            }
            set
            {
                Ixx.value = value;
            }
        }
        private Attribute Ixy;
        public double ixy
        {
            get
            {
                return (double)Ixy.value;
            }
            set
            {
                Ixy.value = value;
            }
        }
        private Attribute Ixz;
        public double ixz
        {
            get
            {
                return (double)Ixz.value;
            }
            set
            {
                Ixz.value = value;
            }
        }
        private Attribute Iyy;
        public double iyy
        {
            get
            {
                return (double)Iyy.value;
            }
            set
            {
                Iyy.value = value;
            }
        }
        private Attribute Iyz;
        public double iyz
        {
            get
            {
                return (double)Iyz.value;
            }
            set
            {
                Iyz.value = value;
            }
        }
        private Attribute Izz;
        public double izz
        {
            get
            {
                return (double)Izz.value;
            }
            set
            {
                Izz.value = value;
            }
        }

        private double[] moment;

        public double[] Moment
        {
            get
            {
                return moment;
            }
        }

        /*
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
        */
        public inertia()
        {
            moment = new double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            Ixx = new Attribute();
            Ixy = new Attribute();
            Ixz = new Attribute();
            Iyy = new Attribute();
            Iyz = new Attribute();
            Izz = new Attribute();
                
            Ixx.isRequired = true;
            Ixx.type = "ixx";
            ixx = 0.0;
            Ixy.isRequired = true;
            Ixy.type = "ixy";
            ixy = 0.0;
            Ixz.isRequired = true;
            Ixz.type = "ixz";
            ixz = 0.0;
            Iyy.isRequired = true;
            Iyy.type = "iyy";
            iyy = 0.0;
            Iyz.isRequired = true;
            Iyz.type = "iyz";
            iyz = 0.0;
            Izz.isRequired = true;
            Izz.type = "izz";
            izz = 0.0;
        }
        public void setMomentMatrix(double[] array)
        {
            moment = array;
            ixx = Moment[0];
            ixy = Moment[1];
            ixz = Moment[2];
            iyy = Moment[4];
            iyz = Moment[5];
            izz = Moment[8];
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("inertia");
            Ixx.writeURDF(writer);
            Ixy.writeURDF(writer);
            Ixz.writeURDF(writer);
            Iyy.writeURDF(writer);
            Iyz.writeURDF(writer);
            Izz.writeURDF(writer);
            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_ixx, TextBox box_ixy, TextBox box_ixz, TextBox box_iyy, TextBox box_iyz, TextBox box_izz, string format)
        {
            box_ixx.Text = ixx.ToString(format);
            box_ixy.Text = ixy.ToString(format);
            box_ixz.Text = ixz.ToString(format);
            box_iyy.Text = iyy.ToString(format);
            box_iyz.Text = iyz.ToString(format);
            box_izz.Text = izz.ToString(format);
        }

        public void update(TextBox box_ixx, TextBox box_ixy, TextBox box_ixz, TextBox box_iyy, TextBox box_iyz, TextBox box_izz)
        {
            double value;
            ixx = (Double.TryParse(box_ixx.Text, out value)) ? value : 0;
            ixy = (Double.TryParse(box_ixy.Text, out value)) ? value : 0;
            ixz = (Double.TryParse(box_ixz.Text, out value)) ? value : 0;
            iyy = (Double.TryParse(box_iyy.Text, out value)) ? value : 0;
            iyz = (Double.TryParse(box_iyz.Text, out value)) ? value : 0;
            izz = (Double.TryParse(box_izz.Text, out value)) ? value : 0;
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
            isRequired = true;
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
        private Attribute Filename;
        public string filename
        {
            get
            {
                return (string)Filename.value;
            }
            set
            {
                Filename.value = value;
            }
        }
        public mesh()
        {
            Filename = new Attribute();
            Filename.isRequired = true;
            Filename.type = "filename";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("mesh");
            Filename.writeURDF(writer);
            writer.WriteEndElement(); //mesh
        }
    }

    //The material element of the visual element. 
    public class material : URDFElement
    {
        public color Color;
        public texture Texture;
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }

        public material()
        {
            Color = new color();
            Texture = new texture();
            Name = new Attribute();
            Name.value = "";
            Name.isRequired = true;
            Name.type = "name";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("material");
            Name.writeURDF(writer);

            Color.writeURDF(writer);
            Texture.writeURDF(writer);

            writer.WriteEndElement();
        }

        public void fillBoxes(ComboBox box, string format)
        {
            box.Text = name;
        }
    }

    //The color element of the material element. Contains a single RGBA.
    public class color : URDFElement
    {
        private Attribute RGBA;
        public double[] rgba
        {
            get
            {
                return (double[])RGBA.value;
            }
            set
            {
                RGBA.value = value;
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
            RGBA = new Attribute();
            RGBA.isRequired = true;
            RGBA.type = "rgba";
            rgba = new double[4] { 1, 1, 1, 1 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("color");
            RGBA.writeURDF(writer);
            writer.WriteEndElement();
        }

        public void fillBoxes(DomainUpDown box_red, DomainUpDown box_green, DomainUpDown box_blue, DomainUpDown box_alpha, string format)
        {
            double[] rgba = (double[])RGBA.value;
            box_red.Text = Red.ToString(format);
            box_green.Text = Green.ToString(format);
            box_blue.Text = Blue.ToString(format);
            box_alpha.Text = Alpha.ToString(format);
        }

        public void update(DomainUpDown box_red, DomainUpDown box_green, DomainUpDown box_blue, DomainUpDown box_alpha)
        {
            double value;
            Red = (Double.TryParse(box_red.Text, out value)) ? value : 0;
            Green = (Double.TryParse(box_green.Text, out value)) ? value : 0;
            Blue = (Double.TryParse(box_blue.Text, out value)) ? value : 0;
            Alpha = (Double.TryParse(box_alpha.Text, out value)) ? value : 0;
        }
    }

    //The texture element of the material element. 
    public class texture : URDFElement
    {
        private Attribute Filename;
        public string filename
        {
            get
            {
                return (string)Filename.value;
            }
            set
            {
                Filename.value = value;
            }
        }
        public string wFilename;
        public texture()
        {
            wFilename = "";
            isRequired = false;
            Filename = new Attribute();
            Filename.isRequired = true;
            filename = "";
            Filename.type = "filename";
        }
        new public void writeURDF(XmlWriter writer)
        {
            if (wFilename != "")
            {
                writer.WriteStartElement("texture");
                Filename.writeURDF(writer);
                writer.WriteEndElement();
            }
        }
    }

    //The collision element of a link.
    public class collision : URDFElement
    {
        public origin Origin;
        public geometry Geometry;

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
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }
        private Attribute Type;
        public string type
        {
            get
            {
                return (string)Type.value;
            }
            set
            {
                Type.value = value;
            }
        }
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
            Name = new Attribute();
            Name.isRequired = true;
            Name.type = "name";
            name = "";
            Type = new Attribute();
            Type.isRequired = true;
            Type.type = "type";
            
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("joint");
            Name.writeURDF(writer);
            Type.writeURDF(writer);
            //writer.WriteAttributeString("name", "joint_" + name);
            //writer.WriteAttributeString("type", type);

            Origin.writeURDF(writer);
            Parent.writeURDF(writer);
            Child.writeURDF(writer);
            Axis.writeURDF(writer);
            if (Limit != null)
            {
                Limit.writeURDF(writer);
            }
            if (Calibration != null)
            {
                Calibration.writeURDF(writer);
            }
            if (Dynamics != null)
            {
                Dynamics.writeURDF(writer);
            }
            if (Safety != null)
            {
                Safety.writeURDF(writer);
            }
            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_name, ComboBox box_type)
        {
            box_name.Text = name;
            box_type.Text = type;
        }

        public void update(TextBox box_name, ComboBox box_type)
        {
            name = box_name.Text;
            type = box_type.Text;
        }
    }

    //parent_link element of a joint. 
    public class parent_link : URDFElement
    {
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }
        public parent_link()
        {
            isRequired = true;
            Name = new Attribute();
            Name.isRequired = true;
            Name.type = "link";
            name = "";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("parent");
            Name.writeURDF(writer);
            writer.WriteEndElement();

        }
        public void fillBoxes(Label box)
        {
            box.Text = name;
        }
        public void update(Label box)
        {
            name = box.Text;
        }
    }

    //The child link element
    public class child_link : URDFElement
    {
        private Attribute Name;
        public string name
        {
            get
            {
                return (string)Name.value;
            }
            set
            {
                Name.value = value;
            }
        }
        public child_link()
        {
            isRequired = true;
            Name = new Attribute();
            Name.type = "link";
            Name.isRequired = true;
            name = "";
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("child");
            Name.writeURDF(writer);
            writer.WriteEndElement();
        }
        public void fillBoxes(Label box)
        {
            box.Text = name;
        }
        public void update(Label box)
        {
            name = box.Text;
        }


    }

    //The axis element of a joint.
    public class axis : URDFElement
    {
        private Attribute XYZ;
        public double[] xyz
        {
            get
            {
                return (double[])XYZ.value;
            }
            set
            {
                XYZ.value = value;
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
            XYZ = new Attribute();
            XYZ.isRequired = true;
            XYZ.type = "xyz";
            xyz = new double[] { 0, 0, 0 };
        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("axis");
            XYZ.writeURDF(writer);
            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_x, TextBox box_y, TextBox box_z, string format)
        {
            box_x.Text = X.ToString(format);
            box_y.Text = Y.ToString(format);
            box_z.Text = Z.ToString(format);
        }

        public void update(TextBox box_x, TextBox box_y, TextBox box_z)
        {
            double value;
            X = (Double.TryParse(box_x.Text, out value)) ? value : 0;
            Y = (Double.TryParse(box_y.Text, out value)) ? value : 0;
            Z = (Double.TryParse(box_z.Text, out value)) ? value : 0;
        }
    }

    //The limit element of a joint.
    public class limit : URDFElement
    {
        private Attribute Lower;
        private Attribute Upper;
        private Attribute Effort;
        private Attribute Velocity;

        public double lower
        {
            get
            {
                return (double)Lower.value;
            }
            set
            {
                if (Lower == null)
                {
                    Lower = new Attribute();
                    Lower.type = "lower";
                }
                Lower.value = value;
            }
        }
        public double upper
        {
            get
            {
                return (double)Upper.value;
            }
            set
            {
                if (Upper == null)
                {
                    Upper = new Attribute();
                    Upper.type = "upper";
                }
                Upper.value = value;
            }
        }
        public double effort
        {
            get
            {
                return (double)Effort.value;
            }
            set
            {
                Effort.value = value;
            }
        }
        public double velocity
        {
            get
            {
                return (double)Velocity.value;
            }
            set
            {
                Velocity.value = value;
            }
        }
        public limit()
        {
            Effort = new Attribute();
            Velocity = new Attribute();
            Effort.isRequired = true;
            Velocity.isRequired = true;
            Effort.type = "effort";
            Velocity.type = "velocity";
            Effort.value = 0.0;
            Velocity.value = 0.0;

        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("limit");
            if (Lower != null)
            {
                Lower.writeURDF(writer);
            }
            if (Upper != null)
            {
                Upper.writeURDF(writer);
            }
            if (Effort != null)
            {
                Effort.writeURDF(writer);
            }
            if (Velocity != null)
            {
                Velocity.writeURDF(writer);
            }
            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_lower, TextBox box_upper, TextBox box_effort, TextBox box_velocity, string format)
        {
            if (Lower != null)
            {
                box_lower.Text = lower.ToString(format);
            }

            if (Upper != null)
            {
                box_upper.Text = upper.ToString(format);
            }

            box_effort.Text = effort.ToString(format);
            box_velocity.Text = velocity.ToString(format);
        }

        public void setValues(TextBox box_lower, TextBox box_upper, TextBox box_effort, TextBox box_velocity)
        {
            double value;
            if (box_lower.Text == "")
            {
                Lower = null;
            }
            else
            {
                lower = (Double.TryParse(box_lower.Text, out value)) ? value : 0;
            }
            if (box_upper.Text == "")
            {
                Upper = null;
            }
            else
            {
                upper = (Double.TryParse(box_upper.Text, out value)) ? value : 0;
            }

            effort = (Double.TryParse(box_effort.Text, out value)) ? value : 0;
            velocity = (Double.TryParse(box_velocity.Text, out value)) ? value : 0;

        }
    }

    //The calibration element of a joint.
    public class calibration : URDFElement
    {
        private Attribute Rising;
        public double rising
        {
            get
            {
                return (double)Rising.value;
            }
            set
            {
                if (Rising == null)
                {
                    Rising = new Attribute();
                    Rising.type = "rising";
                }
                Rising.value = value;
            }
        }
        private Attribute Falling;
        public double falling
        {
            get
            {
                return (double)Falling.value;
            }
            set
            {
                if (Falling == null)
                {
                    Falling = new Attribute();
                    Falling.type = "falling";
                }
                    
                Falling.value = value;
            }
        }

        public calibration()
        {

        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("calibration");
            if (Rising != null)
            {
                Rising.writeURDF(writer);
            }
            if (Falling != null)
            {
                Falling.writeURDF(writer);
            }
            writer.WriteEndElement();
        }
        public void fillBoxes(TextBox box_rising, TextBox box_falling, string format)
        {
            if (Rising != null)
            {
                box_rising.Text = rising.ToString(format);
            }

            if (Falling != null)
            {
                box_falling.Text = falling.ToString(format);
            }
        }

        public void setValues(TextBox box_rising, TextBox box_falling)
        {
            double value;
            if (box_rising.Text == "")
            {
                Rising = null;
            }
            else
            {
                rising = (Double.TryParse(box_rising.Text, out value)) ? value : 0;
            }
            if (box_falling.Text == "")
            {
                Falling = null;
            }
            else
            {
                falling = (Double.TryParse(box_falling.Text, out value)) ? value : 0;
            }
        }
    }

    //The dynamics element of a joint.
    public class dynamics : URDFElement
    {
        private Attribute Damping;
        public double damping
        {
            get
            {
                return (double)Damping.value;
            }
            set
            {
                if (Damping == null)
                {
                    Damping = new Attribute();
                    Damping.type = "damping";
                }
                Damping.value = value;
            }
        }
        private Attribute Friction;
        public double friction
        {
            get
            {
                return (double)Friction.value;
            }
            set
            {
                if (Friction == null)
                {
                    Friction = new Attribute();
                    Friction.type = "friction";
                }
                Friction.value = value;
            }
        }

        public dynamics()
        {

        }
        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("dynamics");
            if (Damping != null)
            {
                Damping.writeURDF(writer);
            }
            if (Friction != null)
            {
                Friction.writeURDF(writer);
            }
            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_damping, TextBox box_friction, string format)
        {
            if (Damping != null)
            {
                box_damping.Text = damping.ToString(format);
            }
            if (Friction != null)
            {
                box_friction.Text = friction.ToString(format);
            }
        }

        public void setValues(TextBox box_damping, TextBox box_friction)
        {
            double value;
            if (box_damping.Text == "")
            {
                Damping = null;
            }
            else
            {
                damping = (Double.TryParse(box_damping.Text, out value)) ? value : 0;
            }
            if (box_friction.Text == "")
            {
                Friction = null;
            }
            else
            {
                friction = (Double.TryParse(box_friction.Text, out value)) ? value : 0;
            }
        }
    }

    //The safety_controller element of a joint.
    public class safety_controller : URDFElement
    {
        private Attribute Soft_lower;
        public double soft_lower
        {
            get
            {
                return (double)Soft_lower.value;
            }
            set
            {
                if (Soft_lower == null)
                {
                    Soft_lower = new Attribute();
                    Soft_lower.type = "soft_lower";
                }
                Soft_lower.value = value;
            }
        }
        private Attribute Soft_upper;
        public double soft_upper
        {
            get
            {
                return (double)Soft_upper.value;
            }
            set
            {
                if (Soft_upper == null)
                {
                    Soft_upper = new Attribute();
                    Soft_upper.type = "soft_upper";
                }
                Soft_upper.value = value;
            }
        }
        private Attribute K_position;
        public double k_position
        {
            get
            {
                return (double)K_position.value;
            }
            set
            {
                if (K_position == null)
                {
                    K_position = new Attribute();
                    K_position.type = "k_position";
                }
                K_position.value = value;
            }
        }
        private Attribute K_velocity;
        public double k_velocity
        {
            get
            {
                return (double)K_velocity.value;
            }
            set
            {
                K_velocity.value = value;
            }
        }
        public safety_controller()
        {
            K_velocity = new Attribute();
            K_velocity.type = "k_velocity";
            K_velocity.isRequired = true;
        }

        new public void writeURDF(XmlWriter writer)
        {
            writer.WriteStartElement("safety_controller");
            if (Soft_upper != null)
            {
                Soft_upper.writeURDF(writer);
            }
            if (Soft_lower != null)
            {
                Soft_lower.writeURDF(writer);
            }
            if (K_position != null)
            {
                K_position.writeURDF(writer);
            }
            K_velocity.writeURDF(writer);

            writer.WriteEndElement();
        }

        public void fillBoxes(TextBox box_lower, TextBox box_upper, TextBox box_position, TextBox box_velocity, string format)
        {
            if (Soft_lower != null)
            {
                box_lower.Text = soft_lower.ToString(format);
            }

            if (Soft_upper != null)
            {
                box_upper.Text = soft_upper.ToString(format);
            }

            if (K_position != null)
            {
                box_position.Text = k_position.ToString(format);
            }

            box_velocity.Text = k_velocity.ToString(format);
            
        }

        public void setValues(TextBox box_lower, TextBox box_upper, TextBox box_position, TextBox box_velocity)
        {
            double value;
            if (box_lower.Text == "")
            {
                Soft_lower = null;
            }
            else
            {
                soft_lower = (Double.TryParse(box_lower.Text, out value)) ? value : 0;
            }

            if (box_upper.Text == "")
            {
                Soft_upper = null;
            }
            else
            {
                soft_upper = (Double.TryParse(box_upper.Text, out value)) ? value : 0;
            }

            if (box_position.Text == "")
            {
                K_position = null;
            }
            else
            {
                k_position = (Double.TryParse(box_position.Text, out value)) ? value : 0;   
            }
            k_velocity = (Double.TryParse(box_velocity.Text, out value)) ? value : 0;
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
        public string whyIncomplete
        { get; set; }

        public LinkNode()
        {
        }

        public LinkNode(SerialNode node)
        {
            linkName = node.linkName;
            jointName = node.jointName;
            axisName = node.axisName;
            coordsysName = node.coordsysName;
            ComponentPIDs = node.componentPIDs;
            jointType = node.jointType;
            isBaseNode = node.isBaseNode;
            isIncomplete = node.isIncomplete;

            Name = linkName;
            Text = linkName;

            foreach (SerialNode child in node.Nodes)
            {
                Nodes.Add(new LinkNode(child));
            }
        }

    }
    #endregion
}
