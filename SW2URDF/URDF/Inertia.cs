using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //Inertia element, which means moment of inertia. In the inertial element
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Inertia : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute IxxAttribute;

        [DataMember]
        private readonly URDFAttribute IxyAttribute;

        [DataMember]
        private readonly URDFAttribute IxzAttribute;

        [DataMember]
        private readonly URDFAttribute IyyAttribute;

        [DataMember]
        private readonly URDFAttribute IyzAttribute;

        [DataMember]
        private readonly URDFAttribute IzzAttribute;

        public double Ixx
        {
            get => (double)IxxAttribute.Value;
            set => IxxAttribute.Value = value;
        }

        public double Ixy
        {
            get => (double)IxyAttribute.Value;
            set => IxyAttribute.Value = value;
        }

        public double Ixz
        {
            get => (double)IxzAttribute.Value;
            set => IxzAttribute.Value = value;
        }

        public double Iyy
        {
            get => (double)IyyAttribute.Value;
            set => IyyAttribute.Value = value;
        }

        public double Iyz
        {
            get => (double)IyzAttribute.Value;
            set => IyzAttribute.Value = value;
        }

        public double Izz
        {
            get => (double)IzzAttribute.Value;
            set => IzzAttribute.Value = value;
        }

        public Inertia() : base("inertia", false)
        {
            IxxAttribute = new URDFAttribute("ixx", true, 0.0);
            IxyAttribute = new URDFAttribute("ixy", true, 0.0);
            IxzAttribute = new URDFAttribute("ixz", true, 0.0);
            IyyAttribute = new URDFAttribute("iyy", true, 0.0);
            IyzAttribute = new URDFAttribute("iyz", true, 0.0);
            IzzAttribute = new URDFAttribute("izz", true, 0.0);

            Attributes.Add(IxxAttribute);
            Attributes.Add(IxyAttribute);
            Attributes.Add(IxzAttribute);
            Attributes.Add(IyyAttribute);
            Attributes.Add(IyzAttribute);
            Attributes.Add(IzzAttribute);
        }

        public void SetMomentMatrix(double[] array)
        {
            Ixx = array[0];
            Ixy = -array[1];
            Ixz = -array[2];
            Iyy = array[4];
            Iyz = -array[5];
            Izz = array[8];
        }

        public void FillBoxes(TextBox boxIxx, TextBox boxIxy, TextBox boxIxz,
            TextBox boxIyy, TextBox boxIyz, TextBox boxIzz, string format)
        {
            boxIxx.Text = IxxAttribute.GetTextFromDoubleValue(format);
            boxIxy.Text = IxyAttribute.GetTextFromDoubleValue(format);
            boxIxz.Text = IxzAttribute.GetTextFromDoubleValue(format);
            boxIyy.Text = IyyAttribute.GetTextFromDoubleValue(format);
            boxIyz.Text = IyzAttribute.GetTextFromDoubleValue(format);
            boxIzz.Text = IzzAttribute.GetTextFromDoubleValue(format);
        }

        public void Update(TextBox boxIxx, TextBox boxIxy, TextBox boxIxz,
            TextBox boxIyy, TextBox boxIyz, TextBox boxIzz)
        {
            IxxAttribute.SetDoubleValueFromString(boxIxx.Text);
            IxyAttribute.SetDoubleValueFromString(boxIxy.Text);
            IxzAttribute.SetDoubleValueFromString(boxIxz.Text);
            IyyAttribute.SetDoubleValueFromString(boxIyy.Text);
            IyzAttribute.SetDoubleValueFromString(boxIyz.Text);
            IzzAttribute.SetDoubleValueFromString(boxIzz.Text);
        }

        internal double[] GetMoment()
        {
            return new double[] { Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz };
        }
    }
}