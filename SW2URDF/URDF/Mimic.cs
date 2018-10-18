using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    [DataContract(Name = "Mimic", Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Mimic : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute JointNameAttribute;

        public string JointName
        {
            get => (string)JointNameAttribute.Value;
            set
            {
                if (value.GetType() == typeof(double))
                {
                    JointNameAttribute.Value = value;
                }
            }
        }

        [DataMember]
        private readonly URDFAttribute MultiplierAttribute;

        public double Multiplier
        {
            get => (double)MultiplierAttribute.Value;
            set
            {
                if (value.GetType() == typeof(double))
                {
                    MultiplierAttribute.Value = value;
                }
            }
        }

        [DataMember]
        private readonly URDFAttribute OffsetAttribute;

        public double Offset
        {
            get => (double)OffsetAttribute.Value;
            set
            {
                if (value.GetType() == typeof(double))
                {
                    OffsetAttribute.Value = value;
                }
            }
        }

        public Mimic() : base("mimic", false)
        {
            JointNameAttribute = new URDFAttribute("joint", true, null);
            MultiplierAttribute = new URDFAttribute("multiplier", false, null);
            OffsetAttribute = new URDFAttribute("offset", false, null);
        }
    }
}