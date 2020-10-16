using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //The limit element of a joint.
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Limit : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute LowerAttribute;

        [DataMember]
        private readonly URDFAttribute UpperAttribute;

        [DataMember]
        private readonly URDFAttribute EffortAttribute;

        [DataMember]
        private readonly URDFAttribute VelocityAttribute;

        public double Lower
        {
            get => (double)LowerAttribute.Value;
            set => LowerAttribute.Value = value;
        }

        public double Upper
        {
            get => (double)UpperAttribute.Value;
            set => UpperAttribute.Value = value;
        }

        public double Effort
        {
            get => (double)EffortAttribute.Value;
            set => EffortAttribute.Value = value;
        }

        public double Velocity
        {
            get => (double)VelocityAttribute.Value;
            set => VelocityAttribute.Value = value;
        }

        public Limit() : base("limit", false)
        {
            EffortAttribute = new URDFAttribute("effort", true, null);
            VelocityAttribute = new URDFAttribute("velocity", true, null);
            LowerAttribute = new URDFAttribute("lower", false, null);
            UpperAttribute = new URDFAttribute("upper", false, null);

            Attributes.Add(LowerAttribute);
            Attributes.Add(UpperAttribute);
            Attributes.Add(EffortAttribute);
            Attributes.Add(VelocityAttribute);
        }

        public void FillBoxes(TextBox boxLower, TextBox boxUpper,
            TextBox boxEffort, TextBox boxVelocity, string format)
        {
            boxLower.Text = LowerAttribute.GetTextFromDoubleValue(format);
            boxUpper.Text = UpperAttribute.GetTextFromDoubleValue(format);
            boxEffort.Text = EffortAttribute.GetTextFromDoubleValue(format);
            boxVelocity.Text = VelocityAttribute.GetTextFromDoubleValue(format);
        }

        public void SetValues(TextBox boxLower, TextBox boxUpper,
            TextBox boxEffort, TextBox boxVelocity)
        {
            if (string.IsNullOrWhiteSpace(boxLower.Text) &&
                string.IsNullOrWhiteSpace(boxUpper.Text) &&
                string.IsNullOrWhiteSpace(boxEffort.Text) &&
                string.IsNullOrWhiteSpace(boxVelocity.Text) &&
                !IsRequired())
            {
                // If all text boxes are empty and this element isn't required, then leave blank
                return;
            }
            LowerAttribute.SetDoubleValueFromString(boxLower.Text);
            UpperAttribute.SetDoubleValueFromString(boxUpper.Text);
            EffortAttribute.SetDoubleValueFromString(boxEffort.Text);
            VelocityAttribute.SetDoubleValueFromString(boxVelocity.Text);
        }

        public override void SetRequired(bool required)
        {
            base.SetRequired(required);
            UpperAttribute.SetRequired(required);
            LowerAttribute.SetRequired(required);
        }

        public override bool AreRequiredFieldsSatisfied()
        {
            // If a limit is required, then these fields should be as well.
            UpperAttribute.SetRequired(IsRequired());
            LowerAttribute.SetRequired(IsRequired());
            return base.AreRequiredFieldsSatisfied();
        }
    }
}