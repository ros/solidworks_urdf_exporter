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
using System.Globalization;
using System.Runtime.Serialization;
using System.Text;
using System.Windows.Forms;
using System.Xml;
using log4net;
using SolidWorks.Interop.sldworks;

namespace SW2URDF
{
    //Initiates the XMLWriter and its necessary settings
    public class URDFWriter
    {
        public XmlWriter writer;

        public URDFWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings
            {
                Encoding = new UTF8Encoding(false),
                OmitXmlDeclaration = true,
                Indent = true,
                NewLineOnAttributes = true,
            };
            writer = XmlWriter.Create(savePath, settings);
        }
    }

    //Not sure why I have a class that everything else inherits from that is empty.
    // But maybe we'll want to add things to it
    [Serializable]
    public class URDFElement
    {
        protected static readonly ILog logger = Logger.GetLogger();
        protected List<URDFElement> ChildElements;
        protected List<Attribute> Attributes;
        protected string ElementName;

        public URDFElement(string elementName)
        {
            ElementName = elementName;
            ChildElements = new List<URDFElement>();
            Attributes = new List<Attribute>();
        }

        public virtual void WriteURDF(XmlWriter writer)
        {
            writer.WriteStartElement(ElementName);
            foreach (Attribute attribute in Attributes)
            {
                if (attribute.Value != null)
                {
                    attribute.WriteURDF(writer);
                }
            }

            foreach (URDFElement child in ChildElements)
            {
                if (child.IsElementSet())
                {
                    child.WriteURDF(writer);
                }
            }

            writer.WriteEndElement();
        }

        public void Unset()
        {
            foreach (Attribute attribute in Attributes)
            {
                attribute.Value = null;
            }
            foreach (URDFElement child in ChildElements)
            {
                child.Unset();
            }
        }

        public virtual bool IsElementSet()
        {
            return true;
        }

        protected bool isRequired;
    }

    [Serializable]
    public class Attribute
    {
        private readonly string USStringFormat = "en-US";
        public readonly bool IsRequired;
        public readonly string AttributeType;
        public object Value;

        public Attribute(string type, bool isRequired, object initialValue)
        {
            AttributeType = type;
            IsRequired = isRequired;
            Value = initialValue;
        }

        public void WriteURDF(XmlWriter writer)
        {
            string valueString = "";
            if (Value.GetType() == typeof(double[]))
            {
                double[] valueArray = (double[])Value;
                foreach (double d in valueArray)
                {
                    valueString +=
                        d.ToString(CultureInfo.CreateSpecificCulture(USStringFormat)) + " ";
                }
                valueString = valueString.Trim();
            }
            else if (Value.GetType() == typeof(double))
            {
                valueString =
                    ((Double)Value).ToString(CultureInfo.CreateSpecificCulture(USStringFormat));
            }
            else if (Value.GetType() == typeof(string))
            {
                valueString = (string)Value;
            }
            else if (Value != null)
            {
                throw new Exception("Unhandled object type in write attribute");
            }
            if (IsRequired && Value == null)
            {
                throw new Exception("Required attribute " + AttributeType + " has null value");
            }
            if (String.IsNullOrWhiteSpace(AttributeType))
            {
                throw new Exception("No type specified");
            }
            if (Value != null)
            {
                writer.WriteAttributeString(AttributeType, valueString);
            }
        }
    }

    //The base URDF element, a robot
    [Serializable]
    public class Robot : URDFElement
    {
        public Link BaseLink { get; private set; }
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        public Robot() : base("robot")
        {
            BaseLink = new Link(null);
            isRequired = true;
            NameAttribute = new Attribute("name", true, "");

            ChildElements.Add(BaseLink);
            Attributes.Add(NameAttribute);
        }

        public override void WriteURDF(XmlWriter writer)
        {
            writer.WriteStartDocument();
            base.WriteURDF(writer);
            writer.WriteEndDocument();
            writer.Close();
        }

        public void SetBaseLink(Link link)
        {
            BaseLink = link;
            ChildElements.Clear();
            ChildElements.Add(link);
        }

        internal string[] GetJointNames(bool includeFixed)
        {
            return BaseLink.GetJointNames(includeFixed);
        }
    }

    //The link class, it contains many other elements not found in the URDF.
    [Serializable]
    public class Link : URDFElement
    {
        public readonly Link Parent;
        public readonly List<Link> Children;
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        public readonly Inertial Inertial;
        public readonly Visual Visual;
        public readonly Collision Collision;
        public readonly Joint Joint;
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

        public Link(Link parent) : base("link")
        {
            Parent = parent;
            Children = new List<Link>();
            SWcomponents = new List<Component2>();
            NameAttribute = new Attribute("name", true, "");

            Inertial = new Inertial();
            Visual = new Visual();
            Collision = new Collision();
            Joint = new Joint();

            isRequired = true;
            isFixedFrame = true;

            Attributes.Add(NameAttribute);
            ChildElements.Add(Inertial);
            ChildElements.Add(Visual);
            ChildElements.Add(Collision);
        }

        public override void WriteURDF(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            NameAttribute.WriteURDF(writer);

            if (Inertial != null)
            {
                Inertial.WriteURDF(writer);
            }
            if (Visual != null)
            {
                Visual.WriteURDF(writer);
            }
            if (Collision != null)
            {
                Collision.WriteURDF(writer);
            }

            writer.WriteEndElement();
            if (Joint.IsElementSet())
            {
                Joint.WriteURDF(writer);
            }

            foreach (Link child in Children)
            {
                child.WriteURDF(writer);
            }
        }

        public String[] GetJointNames(bool includeFixed)
        {
            List<String> names = new List<string>();

            if (Joint != null && (includeFixed || Joint.Type != "fixed"))
            {
                names.Add(Joint.Name);
            }
            foreach (Link child in Children)
            {
                names.AddRange(child.GetJointNames(includeFixed));
            }

            return names.ToArray();
        }
    }

    //The serial node class, it is used only for saving the configuration.
    [Serializable]
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
                linkName = node.LinkName;
                jointName = node.JointName;
                axisName = node.AxisName;
                coordsysName = node.CoordsysName;
                componentPIDs = node.ComponentPIDs;
                jointType = node.JointType;
                isBaseNode = node.IsBaseNode;
                isIncomplete = node.IsIncomplete;
            }
            else
            {
                linkName = node.Link.Name;

                componentPIDs = node.ComponentPIDs;
                if (node.Link.Joint != null)
                {
                    jointName = node.Link.Joint.Name;

                    if (node.Link.Joint.Axis.X == 0 &&
                        node.Link.Joint.Axis.Y == 0 &&
                        node.Link.Joint.Axis.Z == 0)
                    {
                        axisName = "None";
                    }
                    else
                    {
                        axisName = node.Link.Joint.AxisName;
                    }
                    coordsysName = node.Link.Joint.CoordinateSystemName;
                    jointType = node.Link.Joint.Type;
                }
                else
                {
                    coordsysName = node.CoordsysName;
                }

                isBaseNode = node.IsBaseNode;
                isIncomplete = node.IsIncomplete;
            }
            //Proceed recursively through the nodes
            foreach (LinkNode child in node.Nodes)
            {
                Nodes.Add(new SerialNode(child));
            }
        }
    }

    //The inertial element of a link
    [Serializable]
    public class Inertial : URDFElement
    {
        public readonly Origin Origin;
        public readonly Mass Mass;
        public readonly Inertia Inertia;

        public Inertial() : base("inertial")
        {
            Origin = new Origin();
            Mass = new Mass();
            Inertia = new Inertia();

            ChildElements.Add(Origin);
            ChildElements.Add(Mass);
            ChildElements.Add(Inertia);
        }
    }

    //The Origin element, used in several other elements
    [Serializable]
    public class Origin : URDFElement
    {
        private readonly Attribute XYZAttribute;
        private readonly Attribute RPYAttribute;

        private double[] XYZ
        {
            get
            {
                return (double[])XYZAttribute.Value;
            }
            set
            {
                XYZAttribute.Value = value;
            }
        }

        public double[] GetXYZ()
        {
            return (double[])XYZ.Clone();
        }

        public void SetXYZ(double[] xyz)
        {
            XYZ = xyz;
        }

        public double X
        {
            get
            {
                return XYZ[0];
            }
            set
            {
                XYZ[0] = value;
            }
        }

        public double Y
        {
            get
            {
                return XYZ[1];
            }
            set
            {
                XYZ[1] = value;
            }
        }

        public double Z
        {
            get
            {
                return XYZ[2];
            }
            set
            {
                XYZ[2] = value;
            }
        }

        private double[] RPY
        {
            get
            {
                return (double[])RPYAttribute.Value;
            }
            set
            {
                RPYAttribute.Value = value;
            }
        }

        public double[] GetRPY()
        {
            return (double[])RPY.Clone();
        }

        public void SetRPY(double[] rpy)
        {
            RPY = rpy;
        }

        public double Roll
        {
            get
            {
                return RPY[0];
            }
            set
            {
                RPY[0] = value;
            }
        }

        public double Pitch
        {
            get
            {
                return RPY[1];
            }
            set
            {
                RPY[1] = value;
            }
        }

        public double Yaw
        {
            get
            {
                return RPY[2];
            }
            set
            {
                RPY[2] = value;
            }
        }

        public bool isCustomized;

        public Origin() : base("origin")
        {
            isCustomized = false;
            XYZAttribute = new Attribute("xyz", true, new double[] { 0, 0, 0 });
            RPYAttribute = new Attribute("rpy", true, new double[] { 0, 0, 0 });

            Attributes.Add(XYZAttribute);
            Attributes.Add(RPYAttribute);
        }

        public void FillBoxes(TextBox boxX, TextBox boxY, TextBox boxZ, TextBox boxRoll,
            TextBox boxPitch, TextBox boxYaw, string format)
        {
            if (XYZAttribute.Value != null)
            {
                boxX.Text = X.ToString(format);
                boxY.Text = Y.ToString(format);
                boxZ.Text = Z.ToString(format);
            }
            else
            {
                boxX.Text = ""; boxY.Text = ""; boxZ.Text = "";
            }

            if (RPYAttribute.Value != null)
            {
                boxRoll.Text = Roll.ToString(format);
                boxPitch.Text = Pitch.ToString(format);
                boxYaw.Text = Yaw.ToString(format);
            }
            else
            {
                boxRoll.Text = ""; boxPitch.Text = ""; boxYaw.Text = "";
            }
        }

        public void Update(TextBox boxX, TextBox boxY, TextBox boxZ,
            TextBox boxRoll, TextBox boxPitch, TextBox boxYaw
            )
        {
            double value;
            if (String.IsNullOrWhiteSpace(boxX.Text) &&
                String.IsNullOrWhiteSpace(boxY.Text) &&
                String.IsNullOrWhiteSpace(boxZ.Text))
            {
                XYZ = null;
            }
            else
            {
                X = (Double.TryParse(boxX.Text, out value)) ? value : 0;
                Y = (Double.TryParse(boxY.Text, out value)) ? value : 0;
                Z = (Double.TryParse(boxZ.Text, out value)) ? value : 0;
            }

            if (String.IsNullOrWhiteSpace(boxRoll.Text) &&
                String.IsNullOrWhiteSpace(boxPitch.Text) &&
                String.IsNullOrWhiteSpace(boxYaw.Text))
            {
                RPY = null;
            }
            else
            {
                Roll = (Double.TryParse(boxRoll.Text, out value)) ? value : 0;
                Pitch = (Double.TryParse(boxPitch.Text, out value)) ? value : 0;
                Yaw = (Double.TryParse(boxYaw.Text, out value)) ? value : 0;
            }
        }
    }

    //mass element, belongs to the inertial element
    [Serializable]
    public class Mass : URDFElement
    {
        private readonly Attribute ValueAttribute;

        public double Value
        {
            get
            {
                return (double)ValueAttribute.Value;
            }
            set
            {
                ValueAttribute.Value = value;
            }
        }

        public Mass() : base("mass")
        {
            ValueAttribute = new Attribute("value", true, 0.0);

            Attributes.Add(ValueAttribute);
        }

        public void FillBoxes(TextBox box, string format)
        {
            if (ValueAttribute != null)
            {
                box.Text = Value.ToString(format);
            }
            else
            {
                box.Text = "0";
            }
        }

        public void Update(TextBox box)
        {
            Value = (Double.TryParse(box.Text, out double tmp)) ? tmp : 0;
        }
    }

    //Inertia element, which means moment of inertia. In the inertial element
    [Serializable]
    public class Inertia : URDFElement
    {
        private readonly Attribute IxxAttribute;

        public double Ixx
        {
            get
            {
                return (double)IxxAttribute.Value;
            }
            set
            {
                IxxAttribute.Value = value;
            }
        }

        private readonly Attribute IxyAttribute;

        public double Ixy
        {
            get
            {
                return (double)IxyAttribute.Value;
            }
            set
            {
                IxyAttribute.Value = value;
            }
        }

        private readonly Attribute IxzAttribute;

        public double Ixz
        {
            get
            {
                return (double)IxzAttribute.Value;
            }
            set
            {
                IxzAttribute.Value = value;
            }
        }

        private readonly Attribute IyyAttribute;

        public double Iyy
        {
            get
            {
                return (double)IyyAttribute.Value;
            }
            set
            {
                IyyAttribute.Value = value;
            }
        }

        private readonly Attribute IyzAttribute;

        public double Iyz
        {
            get
            {
                return (double)IyzAttribute.Value;
            }
            set
            {
                IyzAttribute.Value = value;
            }
        }

        private readonly Attribute IzzAttribute;

        public double Izz
        {
            get
            {
                return (double)IzzAttribute.Value;
            }
            set
            {
                IzzAttribute.Value = value;
            }
        }

        private double[] Moment { get; set; }

        public Inertia() : base("robot")
        {
            Moment = new double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            IxxAttribute = new Attribute("ixx", true, 0.0);
            IxyAttribute = new Attribute("ixy", true, 0.0);
            IxzAttribute = new Attribute("ixz", true, 0.0);
            IyyAttribute = new Attribute("iyy", true, 0.0);
            IyzAttribute = new Attribute("iyz", true, 0.0);
            IzzAttribute = new Attribute("izz", true, 0.0);

            Attributes.Add(IxxAttribute);
            Attributes.Add(IxyAttribute);
            Attributes.Add(IxzAttribute);
            Attributes.Add(IyyAttribute);
            Attributes.Add(IyzAttribute);
            Attributes.Add(IzzAttribute);
        }

        public void SetMomentMatrix(double[] array)
        {
            Moment = (double[])array.Clone();
            Ixx = Moment[0];
            Ixy = -Moment[1];
            Ixz = -Moment[2];
            Iyy = Moment[4];
            Iyz = -Moment[5];
            Izz = Moment[8];
        }

        public void FillBoxes(TextBox boxIxx, TextBox boxIxy, TextBox boxIxz,
            TextBox boxIyy, TextBox boxIyz, TextBox boxIzz, string format)
        {
            boxIxx.Text = Ixx.ToString(format);
            boxIxy.Text = Ixy.ToString(format);
            boxIxz.Text = Ixz.ToString(format);
            boxIyy.Text = Iyy.ToString(format);
            boxIyz.Text = Iyz.ToString(format);
            boxIzz.Text = Izz.ToString(format);
        }

        public void Update(TextBox boxIxx, TextBox boxIxy, TextBox boxIxz,
            TextBox boxIyy, TextBox boxIyz, TextBox boxIzz)
        {
            double value = 0;
            Ixx = (Double.TryParse(boxIxx.Text, out value)) ? value : 0;
            Ixy = (Double.TryParse(boxIxy.Text, out value)) ? value : 0;
            Ixz = (Double.TryParse(boxIxz.Text, out value)) ? value : 0;
            Iyy = (Double.TryParse(boxIyy.Text, out value)) ? value : 0;
            Iyz = (Double.TryParse(boxIyz.Text, out value)) ? value : 0;
            Izz = (Double.TryParse(boxIzz.Text, out value)) ? value : 0;
        }

        internal double[] GetMoment()
        {
            return (double[])Moment.Clone();
        }
    }

    //The visual element of a link
    [Serializable]
    public class Visual : URDFElement
    {
        public readonly Origin Origin;
        public readonly Geometry Geometry;
        public readonly Material Material;

        public Visual() : base("visual")
        {
            Origin = new Origin();
            Geometry = new Geometry();
            Material = new Material();

            ChildElements.Add(Origin);
            ChildElements.Add(Geometry);
            ChildElements.Add(Material);
        }
    }

    //The geometry element the visual element
    [Serializable]
    public class Geometry : URDFElement
    {
        public readonly Mesh Mesh;

        public Geometry() : base("geometry")
        {
            Mesh = new Mesh();
            isRequired = true;
            ChildElements.Add(Mesh);
        }
    }

    //The mesh element of the geometry element. This contains only a filename location of the mesh.
    [Serializable]
    public class Mesh : URDFElement
    {
        private readonly Attribute FilenameAttribute;

        public string Filename
        {
            get
            {
                return (string)FilenameAttribute.Value;
            }
            set
            {
                FilenameAttribute.Value = value;
            }
        }

        public Mesh() : base("mesh")
        {
            FilenameAttribute = new Attribute("filename", true, null);

            Attributes.Add(FilenameAttribute);
        }
    }

    //The material element of the visual element.
    [Serializable]
    public class Material : URDFElement
    {
        public readonly Color Color;
        public readonly Texture Texture;
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        public Material() : base("material")
        {
            Color = new Color();
            Texture = new Texture();
            NameAttribute = new Attribute("name", true, null);

            Attributes.Add(NameAttribute);
            ChildElements.Add(Color);
            ChildElements.Add(Texture);
        }

        public void FillBoxes(ComboBox box, string format)
        {
            box.Text = Name;
        }
    }

    //The color element of the material element. Contains a single RGBA.
    [Serializable]
    public class Color : URDFElement
    {
        private readonly Attribute RGBAAttribute;

        private double[] RGBA
        {
            get
            {
                return (double[])RGBAAttribute.Value;
            }
            set
            {
                RGBAAttribute.Value = value;
            }
        }

        public double Red
        {
            get
            {
                return RGBA[0];
            }
            set
            {
                RGBA[0] = value;
            }
        }

        public double Green
        {
            get
            {
                return RGBA[1];
            }
            set
            {
                RGBA[1] = value;
            }
        }

        public double Blue
        {
            get
            {
                return RGBA[2];
            }
            set
            {
                RGBA[2] = value;
            }
        }

        public double Alpha
        {
            get
            {
                return RGBA[3];
            }
            set
            {
                RGBA[3] = value;
            }
        }

        public Color() : base("color")
        {
            RGBAAttribute = new Attribute("rgba", true, new double[] { 1, 1, 1, 1 });

            Attributes.Add(RGBAAttribute);
        }

        public void FillBoxes(DomainUpDown boxRed, DomainUpDown boxGreen,
            DomainUpDown boxBlue, DomainUpDown boxAlpha, string format)
        {
            double[] rgba = (double[])RGBAAttribute.Value;
            boxRed.Text = Red.ToString(format);
            boxGreen.Text = Green.ToString(format);
            boxBlue.Text = Blue.ToString(format);
            boxAlpha.Text = Alpha.ToString(format);
        }

        public void Update(DomainUpDown boxRed, DomainUpDown boxGreen,
            DomainUpDown boxBlue, DomainUpDown boxAlpha)
        {
            double value;
            Red = (Double.TryParse(boxRed.Text, out value)) ? value : 0;
            Green = (Double.TryParse(boxGreen.Text, out value)) ? value : 0;
            Blue = (Double.TryParse(boxBlue.Text, out value)) ? value : 0;
            Alpha = (Double.TryParse(boxAlpha.Text, out value)) ? value : 0;
        }
    }

    //The texture element of the material element.
    [Serializable]
    public class Texture : URDFElement
    {
        private readonly Attribute FilenameAttribute;

        public string Filename
        {
            get
            {
                return (string)FilenameAttribute.Value;
            }
            set
            {
                FilenameAttribute.Value = value;
            }
        }

        public string wFilename;

        public Texture() : base("texture")
        {
            wFilename = "";
            isRequired = false;
            FilenameAttribute = new Attribute("filename", true, null);

            Attributes.Add(FilenameAttribute);
        }
    }

    //The collision element of a link.
    [Serializable]
    public class Collision : URDFElement
    {
        public readonly Origin Origin;
        public readonly Geometry Geometry;

        public Collision() : base("collision")
        {
            Origin = new Origin();
            Geometry = new Geometry();

            ChildElements.Add(Origin);
            ChildElements.Add(Geometry);
        }
    }

    //The joint class. There is one for every link but the base link
    [Serializable]
    public class Joint : URDFElement
    {
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        private readonly Attribute TypeAttribute;

        public string Type
        {
            get
            {
                return (string)TypeAttribute.Value;
            }
            set
            {
                TypeAttribute.Value = value;
            }
        }

        public readonly Origin Origin;
        public readonly ParentLink Parent;
        public readonly ChildLink Child;
        public readonly Axis Axis;
        public readonly Limit Limit;
        public readonly Calibration Calibration;
        public readonly Dynamics Dynamics;
        public readonly SafetyController Safety;
        public string CoordinateSystemName;
        public string AxisName;

        public Joint() : base("joint")
        {
            Origin = new Origin();
            Parent = new ParentLink();
            Child = new ChildLink();
            Axis = new Axis();

            Limit = new Limit();
            Calibration = new Calibration();
            Dynamics = new Dynamics();
            Safety = new SafetyController();

            NameAttribute = new Attribute("name", true, "");
            TypeAttribute = new Attribute("type", true, "");

            Attributes.Add(NameAttribute);
            Attributes.Add(TypeAttribute);

            ChildElements.Add(Origin);
            ChildElements.Add(Parent);
            ChildElements.Add(Child);
            ChildElements.Add(Axis);

            ChildElements.Add(Limit);
            ChildElements.Add(Calibration);
            ChildElements.Add(Dynamics);
            ChildElements.Add(Safety);
        }

        public void FillBoxes(TextBox boxName, ComboBox boxType)
        {
            boxName.Text = Name;
            boxType.Text = Type;
        }

        public void Update(TextBox boxName, ComboBox boxType)
        {
            Name = boxName.Text;
            Type = boxType.Text;
        }

        public override bool IsElementSet()
        {
            return !string.IsNullOrWhiteSpace(Name) && !string.IsNullOrWhiteSpace(Type);
        }
    }

    //parent_link element of a joint.

    [Serializable]
    public class ParentLink : URDFElement
    {
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        public ParentLink() : base("parent")
        {
            isRequired = true;
            NameAttribute = new Attribute("link", true, "");

            Attributes.Add(NameAttribute);
        }

        public void FillBoxes(Label box)
        {
            box.Text = Name;
        }

        public void Update(Label box)
        {
            Name = box.Text;
        }
    }

    //The child link element
    [Serializable]
    public class ChildLink : URDFElement
    {
        private readonly Attribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        public ChildLink() : base("child")
        {
            isRequired = true;
            NameAttribute = new Attribute("link", true, "");

            Attributes.Add(NameAttribute);
        }

        public void FillBoxes(Label box)
        {
            box.Text = Name;
        }

        public void Update(Label box)
        {
            Name = box.Text;
        }
    }

    //The axis element of a joint.
    [Serializable]
    public class Axis : URDFElement
    {
        private readonly Attribute XYZAttribute;

        private double[] XYZ
        {
            get
            {
                return (double[])XYZAttribute.Value;
            }
            set
            {
                XYZAttribute.Value = value;
            }
        }

        public double[] GetXYZ()
        {
            return (double[])XYZ.Clone();
        }

        public void SetXYZ(double[] xyz)
        {
            XYZ = (double[])xyz.Clone();
        }

        public double X
        {
            get
            {
                return XYZ[0];
            }
            set
            {
                XYZ[0] = value;
            }
        }

        public double Y
        {
            get
            {
                return XYZ[1];
            }
            set
            {
                XYZ[1] = value;
            }
        }

        public double Z
        {
            get
            {
                return XYZ[2];
            }
            set
            {
                XYZ[2] = value;
            }
        }

        public Axis() : base("axis")
        {
            XYZAttribute = new Attribute("xyz", true, new double[] { 0, 0, 0 });

            Attributes.Add(XYZAttribute);
        }

        public void FillBoxes(TextBox boxX, TextBox boxY, TextBox boxZ, string format)
        {
            boxX.Text = X.ToString(format);
            boxY.Text = Y.ToString(format);
            boxZ.Text = Z.ToString(format);
        }

        public void Update(TextBox boxX, TextBox boxY, TextBox boxZ)
        {
            double value;
            X = (Double.TryParse(boxX.Text, out value)) ? value : 0;
            Y = (Double.TryParse(boxY.Text, out value)) ? value : 0;
            Z = (Double.TryParse(boxZ.Text, out value)) ? value : 0;
        }
    }

    //The limit element of a joint.
    [Serializable]
    public class Limit : URDFElement
    {
        private readonly Attribute LowerAttribute;
        private readonly Attribute UpperAttribute;
        private readonly Attribute EffortAttribute;
        private readonly Attribute VelocityAttribute;

        public double Lower
        {
            get
            {
                return (double)LowerAttribute.Value;
            }
            set
            {
                LowerAttribute.Value = value;
            }
        }

        public double Upper
        {
            get
            {
                return (double)UpperAttribute.Value;
            }
            set
            {
                UpperAttribute.Value = value;
            }
        }

        public double Effort
        {
            get
            {
                return (double)EffortAttribute.Value;
            }
            set
            {
                EffortAttribute.Value = value;
            }
        }

        public double Velocity
        {
            get
            {
                return (double)VelocityAttribute.Value;
            }
            set
            {
                VelocityAttribute.Value = value;
            }
        }

        public Limit() : base("limit")
        {
            EffortAttribute = new Attribute("effort", true, 0.0);
            VelocityAttribute = new Attribute("velocity", true, 0.0);
            LowerAttribute = new Attribute("lower", false, null);
            UpperAttribute = new Attribute("upper", false, null);

            Attributes.Add(LowerAttribute);
            Attributes.Add(UpperAttribute);
            Attributes.Add(EffortAttribute);
            Attributes.Add(VelocityAttribute);
        }

        public void FillBoxes(TextBox boxLower, TextBox boxUpper,
            TextBox boxEffort, TextBox boxVelocity, string format)
        {
            if (LowerAttribute.Value != null)
            {
                boxLower.Text = Lower.ToString(format);
            }

            if (UpperAttribute.Value != null)
            {
                boxUpper.Text = Upper.ToString(format);
            }

            if (EffortAttribute.Value != null)
            {
                boxEffort.Text = Effort.ToString(format);
            }

            if (VelocityAttribute.Value != null)
            {
                boxVelocity.Text = Velocity.ToString(format);
            }
        }

        public void SetValues(TextBox boxLower, TextBox boxUpper,
            TextBox boxEffort, TextBox boxVelocity)
        {
            double value;
            if (String.IsNullOrWhiteSpace(boxLower.Text))
            {
                LowerAttribute.Value = null;
            }
            else
            {
                Lower = (Double.TryParse(boxLower.Text, out value)) ? value : 0;
            }
            if (String.IsNullOrWhiteSpace(boxUpper.Text))
            {
                UpperAttribute.Value = null;
            }
            else
            {
                Upper = (Double.TryParse(boxUpper.Text, out value)) ? value : 0;
            }

            Effort = (Double.TryParse(boxEffort.Text, out value)) ? value : 0;
            Velocity = (Double.TryParse(boxVelocity.Text, out value)) ? value : 0;
        }

        public bool IsValid()
        {
            return ((EffortAttribute.Value != null) && (VelocityAttribute.Value != null));
        }
    }

    //The calibration element of a joint.
    [Serializable]
    public class Calibration : URDFElement
    {
        private readonly Attribute RisingAttribute;

        public double Rising
        {
            get
            {
                return (double)RisingAttribute.Value;
            }
            set
            {
                RisingAttribute.Value = value;
            }
        }

        private readonly Attribute FallingAttribute;

        public double Falling
        {
            get
            {
                return (double)FallingAttribute.Value;
            }
            set
            {
                FallingAttribute.Value = value;
            }
        }

        public Calibration() : base("calibration")
        {
            RisingAttribute = new Attribute("rising", false, null);
            FallingAttribute = new Attribute("falling", false, null);
            Attributes.Add(RisingAttribute);
            Attributes.Add(FallingAttribute);
        }

        public void FillBoxes(TextBox boxRising, TextBox boxFalling, string format)
        {
            if (RisingAttribute.Value != null)
            {
                boxRising.Text = Rising.ToString(format);
            }

            if (FallingAttribute.Value != null)
            {
                boxFalling.Text = Falling.ToString(format);
            }
        }

        public void SetValues(TextBox boxRising, TextBox boxFalling)
        {
            double value;
            if (String.IsNullOrWhiteSpace(boxRising.Text))
            {
                RisingAttribute.Value = null;
            }
            else
            {
                Rising = (Double.TryParse(boxRising.Text, out value)) ? value : 0;
            }
            if (String.IsNullOrWhiteSpace(boxFalling.Text))
            {
                FallingAttribute.Value = null;
            }
            else
            {
                Falling = (Double.TryParse(boxFalling.Text, out value)) ? value : 0;
            }
        }
    }

    //The dynamics element of a joint.
    [Serializable]
    public class Dynamics : URDFElement
    {
        private readonly Attribute DampingAttribute;

        public double Damping
        {
            get
            {
                return (double)DampingAttribute.Value;
            }
            set
            {
                DampingAttribute.Value = value;
            }
        }

        private readonly Attribute FrictionAttribute;

        public double Friction
        {
            get
            {
                return (double)FrictionAttribute.Value;
            }
            set
            {
                FrictionAttribute.Value = value;
            }
        }

        public Dynamics() : base("dynamics")
        {
            DampingAttribute = new Attribute("damping", false, null);
            FrictionAttribute = new Attribute("friction", false, null);

            Attributes.Add(DampingAttribute);
            Attributes.Add(FrictionAttribute);
        }

        public void FillBoxes(TextBox boxDamping, TextBox boxFriction, string format)
        {
            if (DampingAttribute.Value != null)
            {
                boxDamping.Text = Damping.ToString(format);
            }
            if (FrictionAttribute.Value != null)
            {
                boxFriction.Text = Friction.ToString(format);
            }
        }

        public void SetValues(TextBox boxDamping, TextBox boxFriction)
        {
            double value;
            if (String.IsNullOrWhiteSpace(boxDamping.Text))
            {
                DampingAttribute.Value = null;
            }
            else
            {
                Damping = (Double.TryParse(boxDamping.Text, out value)) ? value : 0;
            }
            if (String.IsNullOrWhiteSpace(boxFriction.Text))
            {
                FrictionAttribute.Value = null;
            }
            else
            {
                Friction = (Double.TryParse(boxFriction.Text, out value)) ? value : 0;
            }
        }
    }

    //The safety_controller element of a joint.
    [Serializable]
    public class SafetyController : URDFElement
    {
        private readonly Attribute SoftLowerAttribute;

        public double SoftLower
        {
            get
            {
                return (double)SoftLowerAttribute.Value;
            }
            set
            {
                SoftLowerAttribute.Value = value;
            }
        }

        private readonly Attribute SoftUpperAttribute;

        public double SoftUpper
        {
            get
            {
                return (double)SoftUpperAttribute.Value;
            }
            set
            {
                SoftUpperAttribute.Value = value;
            }
        }

        private readonly Attribute KPositionAttribute;

        public double KPosition
        {
            get
            {
                return (double)KPositionAttribute.Value;
            }
            set
            {
                KPositionAttribute.Value = value;
            }
        }

        private readonly Attribute KVelocityAttribute;

        public double KVelocity
        {
            get
            {
                return (double)KVelocityAttribute.Value;
            }
            set
            {
                KVelocityAttribute.Value = value;
            }
        }

        public SafetyController() : base("safety_controller")
        {
            SoftUpperAttribute = new Attribute("soft_upper", false, null);
            SoftLowerAttribute = new Attribute("soft_lower", false, null);
            KPositionAttribute = new Attribute("k_position", false, null);
            KVelocityAttribute = new Attribute("k_velocity", true, 0.0);

            Attributes.Add(SoftUpperAttribute);
            Attributes.Add(SoftLowerAttribute);
            Attributes.Add(KPositionAttribute);
            Attributes.Add(KVelocityAttribute);
        }

        public void FillBoxes(TextBox boxLower, TextBox boxUpper,
            TextBox boxPosition, TextBox boxVelocity, string format)
        {
            if (SoftLowerAttribute.Value != null)
            {
                boxLower.Text = SoftLower.ToString(format);
            }

            if (SoftUpperAttribute.Value != null)
            {
                boxUpper.Text = SoftUpper.ToString(format);
            }

            if (KPositionAttribute.Value != null)
            {
                boxPosition.Text = KPosition.ToString(format);
            }

            boxVelocity.Text = KVelocity.ToString(format);
        }

        public void SetValues(TextBox boxLower, TextBox boxUpper,
            TextBox boxPosition, TextBox boxVelocity)
        {
            double value;
            if (String.IsNullOrWhiteSpace(boxLower.Text))
            {
                SoftLowerAttribute.Value = null;
            }
            else
            {
                SoftLower = (Double.TryParse(boxLower.Text, out value)) ? value : 0;
            }

            if (String.IsNullOrWhiteSpace(boxUpper.Text))
            {
                SoftUpperAttribute.Value = null;
            }
            else
            {
                SoftUpper = (Double.TryParse(boxUpper.Text, out value)) ? value : 0;
            }

            if (String.IsNullOrWhiteSpace(boxPosition.Text))
            {
                KPositionAttribute.Value = null;
            }
            else
            {
                KPosition = (Double.TryParse(boxPosition.Text, out value)) ? value : 0;
            }

            KVelocity = (Double.TryParse(boxVelocity.Text, out value)) ? value : 0;
        }
    }

    //A class that just writes the bare minimum of the manifest file necessary for ROS packages.
    [Serializable]
    public class PackageXMLWriter
    {
        public XmlWriter writer;
        private static readonly ILog logger = Logger.GetLogger();

        public PackageXMLWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = false;
            logger.Info("Creating package.xml at " + savePath);
            writer = XmlWriter.Create(savePath, settings);
        }
    }

    //The base class for packageXML elements. Again, I guess I like having empty base classes
    [Serializable]
    public class PackageElement
    {
        public PackageElement()
        {
        }

        public void WriteElement()
        {
        }
    }

    //Top level class for the package XML file.
    [Serializable]
    public class PackageXML : PackageElement
    {
        public Description description;
        public Dependencies dependencies;
        public Author author;
        public License license;

        public PackageXML(string name)
        {
            description = new Description(name);

            dependencies = new Dependencies(
                new String[] { "catkin" },
                new String[] {
                    "roslaunch", "robot_state_publisher", "rviz", "joint_state_publisher", "gazebo" });

            author = new Author("TODO");

            license = new License("BSD");
        }

        public void WriteElement(PackageXMLWriter mWriter)
        {
            XmlWriter writer = mWriter.writer;
            writer.WriteStartDocument();
            writer.WriteStartElement("package");

            description.WriteElement(writer);

            author.WriteElement(writer);
            license.WriteElement(writer);
            dependencies.WriteElement(writer);

            writer.WriteStartElement("export");

            writer.WriteStartElement("architecture_independent");
            writer.WriteEndElement();

            writer.WriteEndElement();

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
    }

    //description element of the manifest file
    [Serializable]
    public class Description : PackageElement
    {
        private readonly string name;
        private readonly string brief;
        private readonly string longDescription;

        public Description(string name)
        {
            this.name = name;
            brief = "URDF Description package for " + name;
            longDescription = "This package contains configuration data, 3D models and launch files\r\n" +
                                    "for " + name + " robot";
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("name");
            writer.WriteString(name);
            writer.WriteEndElement();

            writer.WriteStartElement("version");
            writer.WriteString("1.0.0");
            writer.WriteEndElement();

            writer.WriteStartElement("description");

            writer.WriteStartElement("p");
            writer.WriteString(brief);
            writer.WriteEndElement();

            writer.WriteStartElement("p");
            writer.WriteString(longDescription);
            writer.WriteEndElement();

            writer.WriteEndElement();
        }
    }

    //The depend element of the manifest file
    [Serializable]
    public class Dependencies : PackageElement
    {
        private readonly string[] buildTool;
        private readonly string[] buildExec;

        public Dependencies(String[] buildTool, String[] buildExec)
        {
            this.buildTool = buildTool;
            this.buildExec = buildExec;
        }

        public void WriteElement(XmlWriter writer)
        {
            foreach (String depend in buildTool)
            {
                writer.WriteStartElement("buildtool_depend");
                writer.WriteString(depend);
                writer.WriteEndElement();
            }

            foreach (String depend in buildExec)
            {
                writer.WriteStartElement("depend");
                writer.WriteString(depend);
                writer.WriteEndElement();
            }
        }
    }

    //The author element of the manifest file
    [Serializable]
    public class Author : PackageElement
    {
        private readonly string name;

        public Author(string name)
        {
            this.name = name;
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("author");
            writer.WriteString(name);
            writer.WriteEndElement();

            writer.WriteStartElement("maintainer");
            writer.WriteAttributeString("email", name + "@email.com");
            writer.WriteEndElement();
        }
    }

    //The license element of the manifest file
    [Serializable]
    public class License : PackageElement
    {
        private readonly string lic;

        public License(string lic)
        {
            this.lic = lic;
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("license");
            writer.WriteString(lic);
            writer.WriteEndElement();
        }
    }

    #region Windows Forms Derived classes

    //A LinkNode is derived from a TreeView TreeNode. I've added many new fields to it so
    // that information can be passed around from the TreeView itself.
    [Serializable]
    public class LinkNode : TreeNode
    {
        private static readonly ILog logger = Logger.GetLogger();

        public Link Link
        { get; set; }

        public string LinkName
        { get; set; }

        public string JointName
        { get; set; }

        public string AxisName
        { get; set; }

        public string CoordsysName
        { get; set; }

        public List<Component2> Components;
        public List<byte[]> ComponentPIDs;

        public string JointType
        { get; set; }

        public bool IsBaseNode
        { get; set; }

        public bool IsIncomplete
        { get; set; }

        public bool NeedsSaving
        { get; set; }

        public string WhyIncomplete
        { get; set; }

        public LinkNode()
        {
        }

        protected LinkNode(SerializationInfo info, StreamingContext context) : base(info, context)
        {
        }

        public LinkNode(SerialNode node)
        {
            logger.Info("Deserializing node " + node.linkName);
            LinkName = node.linkName;
            JointName = node.jointName;
            AxisName = node.axisName;
            CoordsysName = node.coordsysName;
            ComponentPIDs = node.componentPIDs;
            JointType = node.jointType;
            IsBaseNode = node.isBaseNode;
            IsIncomplete = node.isIncomplete;

            Name = LinkName;
            Text = LinkName;

            foreach (SerialNode child in node.Nodes)
            {
                Nodes.Add(new LinkNode(child));
            }
        }
    }

    #endregion Windows Forms Derived classes
}