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
                Link = node.Link;
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
}