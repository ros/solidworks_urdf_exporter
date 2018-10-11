using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The geometry element of the visual and collision elements
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Geometry : URDFElement
    {
        [DataMember]
        public readonly Mesh Mesh;

        public Geometry() : base("geometry", true)
        {
            Mesh = new Mesh();
            ChildElements.Add(Mesh);
        }
    }
}