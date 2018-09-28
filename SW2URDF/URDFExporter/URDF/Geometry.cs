using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The geometry element the visual element
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