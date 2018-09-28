using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The inertial element of a link
    [DataContract(IsReference = true)]
    public class Inertial : URDFElement
    {
        [DataMember]
        public readonly Origin Origin;

        [DataMember]
        public readonly Mass Mass;

        [DataMember]
        public readonly Inertia Inertia;

        public Inertial() : base("inertial", false)
        {
            Origin = new Origin("Inertia", false);
            Mass = new Mass();
            Inertia = new Inertia();

            ChildElements.Add(Origin);
            ChildElements.Add(Mass);
            ChildElements.Add(Inertia);
        }
    }
}