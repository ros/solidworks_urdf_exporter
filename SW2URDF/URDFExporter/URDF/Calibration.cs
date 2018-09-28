using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //The calibration element of a joint.
    [DataContract(IsReference = true)]
    public class Calibration : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute RisingAttribute;

        public double Rising
        {
            get
            {
                return (double)RisingAttribute.Value;
            }
            set
            {
                RisingAttribute.Value = value;
            }
        }

        [DataMember]
        private readonly URDFAttribute FallingAttribute;

        public double Falling
        {
            get
            {
                return (double)FallingAttribute.Value;
            }
            set
            {
                FallingAttribute.Value = value;
            }
        }

        public Calibration() : base("calibration", false)
        {
            RisingAttribute = new URDFAttribute("rising", false, null);
            FallingAttribute = new URDFAttribute("falling", false, null);
            Attributes.Add(RisingAttribute);
            Attributes.Add(FallingAttribute);
        }

        public void FillBoxes(TextBox boxRising, TextBox boxFalling, string format)
        {
            boxRising.Text = RisingAttribute.GetTextFromDoubleValue(format);
            boxFalling.Text = FallingAttribute.GetTextFromDoubleValue(format);
        }

        public void SetValues(TextBox boxRising, TextBox boxFalling)
        {
            RisingAttribute.SetDoubleValueFromString(boxRising.Text);
            FallingAttribute.SetDoubleValueFromString(boxFalling.Text);
        }
    }
}