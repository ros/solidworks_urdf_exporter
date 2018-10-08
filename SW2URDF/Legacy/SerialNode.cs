using log4net;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Runtime.Serialization;
using System.Xml.Serialization;

namespace SW2URDF.Legacy
{
    //The serial node class, it is used only for saving the configuration.
    [DataContract(IsReference = true)]
    public class SerialNode
    {
        private readonly ILog logger = Logger.GetLogger();

        [DataMember]
        public string linkName;

        [DataMember]
        public string jointName;

        [DataMember]
        public string axisName;

        [DataMember]
        public string coordsysName;

        [DataMember]
        public List<byte[]> componentPIDs;

        [DataMember]
        public string jointType;

        [DataMember]
        public bool isBaseNode;

        [DataMember]
        public bool isIncomplete;

        [DataMember]
        public List<SerialNode> Nodes;

        [XmlIgnore]
        [DataMember]
        public Link URDFLink;

        //This is only used by the serialization module.
        public SerialNode()
        {
            Nodes = new List<SerialNode>();
        }

        public LinkNode BuildLinkNodeFromSerialNode()
        {
            logger.Info("Deserializing node " + linkName);
            LinkNode node = new LinkNode();
            node.Link.Name = linkName;
            node.Link.Joint.Name = jointName;
            node.Link.Joint.AxisName = axisName;
            node.Link.Joint.CoordinateSystemName = coordsysName;
            node.Link.SWComponentPIDs = componentPIDs;
            node.Link.Joint.Type = jointType;
            node.IsBaseNode = isBaseNode;
            node.IsIncomplete = isIncomplete;

            node.Name = node.Link.Name;
            node.Text = node.Link.Name;

            foreach (SerialNode child in Nodes)
            {
                node.Nodes.Add(child.BuildLinkNodeFromSerialNode());
            }
            return node;
        }
    }
}