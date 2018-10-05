using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The collision element of a link.
    [DataContract]
    public class Collision : URDFElement
    {
        [DataMember]
        public readonly Origin Origin;

        [DataMember]
        public readonly Geometry Geometry;

        public Collision() : base("collision", false)
        {
            Origin = new Origin("Colission", false);
            Geometry = new Geometry();

            ChildElements.Add(Origin);
            ChildElements.Add(Geometry);
        }
    }
}