using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    [DataContract(IsReference = true)]
    public class Axis : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute XYZAttribute;

        private double[] XYZ
        {
            get
            {
                return (double[])XYZAttribute.Value;
            }
            set
            {
                XYZAttribute.Value = value;
            }
        }

        public double[] GetXYZ()
        {
            return (double[])XYZ.Clone();
        }

        public void SetXYZ(double[] xyz)
        {
            XYZ = (double[])xyz.Clone();
        }

        public double X
        {
            get
            {
                return XYZ[0];
            }
            set
            {
                XYZ[0] = value;
            }
        }

        public double Y
        {
            get
            {
                return XYZ[1];
            }
            set
            {
                XYZ[1] = value;
            }
        }

        public double Z
        {
            get
            {
                return XYZ[2];
            }
            set
            {
                XYZ[2] = value;
            }
        }

        public Axis() : base("axis", false)
        {
            XYZAttribute = new URDFAttribute("xyz", true, new double[] { 0, 0, 0 });

            Attributes.Add(XYZAttribute);
        }

        public void FillBoxes(TextBox boxX, TextBox boxY, TextBox boxZ, string format)
        {
            string[] xyzText = XYZAttribute.GetTextArrayFromDoubleArray(format);
            if (xyzText != null)
            {
                boxX.Text = xyzText[0];
                boxY.Text = xyzText[1];
                boxZ.Text = xyzText[2];
            }
        }

        public void Update(TextBox boxX, TextBox boxY, TextBox boxZ)
        {
            XYZAttribute.SetDoubleArrayFromStringArray(new string[] { boxX.Text, boxY.Text, boxZ.Text });
        }
    }
}
