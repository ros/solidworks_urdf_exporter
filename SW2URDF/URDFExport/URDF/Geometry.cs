using System.Runtime.Serialization;

namespace SW2URDF.URDFExport.URDF
{
    //The geometry element of the visual and collision elements
    [DataContract(IsReference = true)]
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