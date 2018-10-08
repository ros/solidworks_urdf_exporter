using System.Runtime.Serialization;
using System.Xml;

namespace SW2URDF.URDF
{
    //The base URDF element, a robot
    [DataContract(IsReference = true)]
    public class Robot : URDFElement
    {
        [DataMember]
        public Link BaseLink { get; private set; }

        [DataMember]
        private URDFAttribute NameAttribute;

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

        public Robot() : base("robot", true)
        {
            BaseLink = new Link(null);
            NameAttribute = new URDFAttribute("name", true, "");

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
}