using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //The safety_controller element of a joint.
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class SafetyController : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute SoftLowerAttribute;

        [DataMember]
        private readonly URDFAttribute SoftUpperAttribute;

        [DataMember]
        private readonly URDFAttribute KPositionAttribute;

        [DataMember]
        private readonly URDFAttribute KVelocityAttribute;

        public double SoftLower
        {
            get
            {
                return (double)SoftLowerAttribute.Value;
            }
            set
            {
                SoftLowerAttribute.Value = value;
            }
        }

        public double SoftUpper
        {
            get
            {
                return (double)SoftUpperAttribute.Value;
            }
            set
            {
                SoftUpperAttribute.Value = value;
            }
        }

        public double KPosition
        {
            get
            {
                return (double)KPositionAttribute.Value;
            }
            set
            {
                KPositionAttribute.Value = value;
            }
        }

        public double KVelocity
        {
            get
            {
                return (double)KVelocityAttribute.Value;
            }
            set
            {
                KVelocityAttribute.Value = value;
            }
        }

        public SafetyController() : base("safety_controller", false)
        {
            SoftUpperAttribute = new URDFAttribute("soft_upper", false, null);
            SoftLowerAttribute = new URDFAttribute("soft_lower", false, null);
            KPositionAttribute = new URDFAttribute("k_position", false, null);
            KVelocityAttribute = new URDFAttribute("k_velocity", true, 0.0);

            Attributes.Add(SoftUpperAttribute);
            Attributes.Add(SoftLowerAttribute);
            Attributes.Add(KPositionAttribute);
            Attributes.Add(KVelocityAttribute);
        }

        public void FillBoxes(TextBox boxLower, TextBox boxUpper,
            TextBox boxPosition, TextBox boxVelocity, string format)
        {
            boxLower.Text = SoftLowerAttribute.GetTextFromDoubleValue();
            boxUpper.Text = SoftUpperAttribute.GetTextFromDoubleValue();
            boxPosition.Text = KPositionAttribute.GetTextFromDoubleValue();
            boxVelocity.Text = KVelocityAttribute.GetTextFromDoubleValue();
        }

        public void SetValues(TextBox boxLower, TextBox boxUpper,
            TextBox boxPosition, TextBox boxVelocity)
        {
            SoftLowerAttribute.SetDoubleValueFromString(boxLower.Text);
            SoftUpperAttribute.SetDoubleValueFromString(boxUpper.Text);
            KPositionAttribute.SetDoubleValueFromString(boxPosition.Text);
            KVelocityAttribute.SetDoubleValueFromString(boxVelocity.Text);
        }
    }
}