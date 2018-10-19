using System.Runtime.Serialization;
using System.Xml;

namespace SW2URDF.URDF
{
    //The base URDF element, a robot
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
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
            string buildVersion = Versioning.Version.GetBuildVersion();
            string commitVersion = Versioning.Version.GetCommitVersion();

            writer.WriteComment(" This URDF was automatically created by SolidWorks to URDF Exporter! " +
                "zOriginally created by Stephen Brawner (brawner@gmail.com) \r\n" +
                string.Format("     Commit Version: {0}  Build Version: {1}\r\n", commitVersion, buildVersion) +
                "     For more information, please see http://wiki.ros.org/sw_urdf_exporter ");

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