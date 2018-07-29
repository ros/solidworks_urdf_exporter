using System.Collections.Generic;
using System.Runtime.Serialization;
using System.Xml.Serialization;

namespace SW2URDF.Legacy
{
    //The serial node class, it is used only for saving the configuration.
    [DataContract(IsReference = true)]
    public class SerialNode
    {
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
        public Link Link;

        //This is only used by the serialization module.
        public SerialNode()
        {
            Nodes = new List<SerialNode>();
        }
    }
}