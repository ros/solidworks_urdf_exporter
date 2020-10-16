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
            get => (double)SoftLowerAttribute.Value;
            set => SoftLowerAttribute.Value = value;
        }

        public double SoftUpper
        {
            get => (double)SoftUpperAttribute.Value;
            set => SoftUpperAttribute.Value = value;
        }

        public double KPosition
        {
            get => (double)KPositionAttribute.Value;
            set => KPositionAttribute.Value = value;
        }

        public double KVelocity
        {
            get => (double)KVelocityAttribute.Value;
            set => KVelocityAttribute.Value = value;
        }

        public SafetyController() : base("safety_controller", false)
        {
            SoftUpperAttribute = new URDFAttribute("soft_upper", false, null);
            SoftLowerAttribute = new URDFAttribute("soft_lower", false, null);
            KPositionAttribute = new URDFAttribute("k_position", false, null);
            KVelocityAttribute = new URDFAttribute("k_velocity", true, null);

            Attributes.Add(SoftUpperAttribute);
            Attributes.Add(SoftLowerAttribute);
            Attributes.Add(KPositionAttribute);
            Attributes.Add(KVelocityAttribute);
        }

        public void FillBoxes(TextBox boxLower, TextBox boxUpper,
            TextBox boxPosition, TextBox boxVelocity, string format)
        {
            boxLower.Text = SoftLowerAttribute.GetTextFromDoubleValue(format);
            boxUpper.Text = SoftUpperAttribute.GetTextFromDoubleValue(format);
            boxPosition.Text = KPositionAttribute.GetTextFromDoubleValue(format);
            boxVelocity.Text = KVelocityAttribute.GetTextFromDoubleValue(format);
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