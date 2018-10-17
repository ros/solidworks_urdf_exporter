using SolidWorks.Interop.sldworks;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using System.Runtime.Serialization;
using System.Xml;

namespace SW2URDF.URDF
{
    //The link class, it contains many other elements not found in the URDF.
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Link : URDFElement//, ISerializable
    {
        [DataMember]
        public Link Parent;

        [DataMember]
        public List<Link> Children;

        [DataMember]
        private readonly URDFAttribute NameAttribute;

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

        [DataMember]
        public Inertial Inertial;

        [DataMember]
        public Visual Visual;

        [DataMember]
        public Collision Collision;

        [DataMember]
        public Joint Joint;

        [DataMember]
        public bool STLQualityFine;

        [DataMember]
        public bool isIncomplete;

        [DataMember]
        public bool isFixedFrame;

        public Component2 SWMainComponent;

        public List<Component2> SWComponents;

        [DataMember]
        public List<byte[]> SWComponentPIDs;

        [DataMember]
        public byte[] SWMainComponentPID;

        public Link() : base("link", true)
        {
            Parent = null;
            Children = new List<Link>();
            SWComponents = new List<Component2>();
            SWComponentPIDs = new List<byte[]>();
            NameAttribute = new URDFAttribute("name", true, "");

            Inertial = new Inertial();
            Visual = new Visual();
            Collision = new Collision();
            Joint = new Joint();

            isFixedFrame = false;

            Attributes.Add(NameAttribute);
            ChildElements.Add(Inertial);
            ChildElements.Add(Visual);
            ChildElements.Add(Collision);
            ChildElements.Add(Joint);
        }

        public Link Clone()
        {
            Link cloned = new Link();
            cloned.SetElement(this);
            foreach (Link child in Children)
            {
                Link clonedChild = child.Clone();
                clonedChild.Parent = this;
                cloned.Children.Add(clonedChild);
            }
            return cloned;
        }

        public Link(Link parent) : base("link", true)
        {
            Parent = parent;
            Children = new List<Link>();
            SWComponents = new List<Component2>();
            SWComponentPIDs = new List<byte[]>();
            NameAttribute = new URDFAttribute("name", true, "");

            Inertial = new Inertial();
            Visual = new Visual();
            Collision = new Collision();
            Joint = new Joint();

            isFixedFrame = false;

            Attributes.Add(NameAttribute);
            ChildElements.Add(Inertial);
            ChildElements.Add(Visual);
            ChildElements.Add(Collision);
            ChildElements.Add(Joint);
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
            if (Joint.ElementContainsData())
            {
                Joint.WriteURDF(writer);
            }

            foreach (Link child in Children)
            {
                child.WriteURDF(writer);
            }
        }

        public override void AppendToCSVDictionary(List<string> context, OrderedDictionary dictionary)
        {
            IEnumerable<string> componentNames = SWComponents.Select(component => component.Name2);
            string componentNamesStr = string.Join(";", componentNames);
            string componentsContext = "Link.SWComponents";
            dictionary.Add(componentsContext, componentNamesStr);

            base.AppendToCSVDictionary(context, dictionary);
        }

        public override void SetElement(URDFElement externalElement)
        {
            base.SetElement(externalElement);
            SetSWComponents((Link)externalElement);
        }

        public override void SetElementFromData(List<string> context, StringDictionary dictionary)
        {
            string componentsContext = "Link.SWComponents";
            string componentsValue = dictionary[componentsContext];
            string[] componentNames = componentsValue.Split(';');

            base.SetElementFromData(context, dictionary);
        }

        public void SetSWComponents(Link externalLink)
        {
            SWComponents = new List<Component2>(externalLink.SWComponents);
            SWComponentPIDs = new List<byte[]>(externalLink.SWComponentPIDs);
            SWMainComponent = externalLink.SWMainComponent;
            SWMainComponentPID = externalLink.SWMainComponentPID;

            isFixedFrame = externalLink.isFixedFrame;
        }

        public string[] GetJointNames(bool includeFixed)
        {
            List<string> names = new List<string>();

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

        public override bool AreRequiredFieldsSatisfied()
        {
            if (!base.AreRequiredFieldsSatisfied())
            {
                return false;
            }

            foreach (Link child in Children)
            {
                if (!child.AreRequiredFieldsSatisfied())
                {
                    return false;
                }
            }

            return true;
        }

        [OnDeserialized]
        private void OnDeserialized(StreamingContext context)
        {
            SWComponents = new List<Component2>();
        }
    }
}