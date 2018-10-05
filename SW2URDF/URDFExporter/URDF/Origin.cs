using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //The Origin element, used in several other elements
    [DataContract]
    public class Origin : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute XYZAttribute;

        [DataMember]
        private readonly URDFAttribute RPYAttribute;

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
            XYZ = xyz;
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

        private double[] RPY
        {
            get
            {
                return (double[])RPYAttribute.Value;
            }
            set
            {
                RPYAttribute.Value = value;
            }
        }

        public double[] GetRPY()
        {
            return (double[])RPY.Clone();
        }

        public void SetRPY(double[] rpy)
        {
            RPY = rpy;
        }

        public double Roll
        {
            get
            {
                return RPY[0];
            }
            set
            {
                RPY[0] = value;
            }
        }

        public double Pitch
        {
            get
            {
                return RPY[1];
            }
            set
            {
                RPY[1] = value;
            }
        }

        public double Yaw
        {
            get
            {
                return RPY[2];
            }
            set
            {
                RPY[2] = value;
            }
        }

        [DataMember]
        public bool isCustomized;

        public Origin(string csvContext, bool isRequired) : base("origin", isRequired)
        {
            isCustomized = false;

            XYZAttribute = new URDFAttribute("xyz", true, new double[] { 0, 0, 0 });
            RPYAttribute = new URDFAttribute("rpy", true, new double[] { 0, 0, 0 });

            Attributes.Add(XYZAttribute);
            Attributes.Add(RPYAttribute);
        }

        public void FillBoxes(TextBox boxX, TextBox boxY, TextBox boxZ, TextBox boxRoll,
            TextBox boxPitch, TextBox boxYaw, string format)
        {
            string[] xyzText = XYZAttribute.GetTextArrayFromDoubleArray(format);
            if (xyzText != null)
            {
                boxX.Text = xyzText[0];
                boxY.Text = xyzText[1];
                boxZ.Text = xyzText[2];
            }

            string[] rpyText = RPYAttribute.GetTextArrayFromDoubleArray(format);
            if (rpyText != null)
            {
                boxRoll.Text = rpyText[0];
                boxY.Text = rpyText[1];
                boxZ.Text = rpyText[2];
            }
        }

        public void Update(TextBox boxX, TextBox boxY, TextBox boxZ,
            TextBox boxRoll, TextBox boxPitch, TextBox boxYaw
            )
        {
            XYZAttribute.SetDoubleArrayFromStringArray(new string[] { boxX.Text, boxY.Text, boxZ.Text });
            RPYAttribute.SetDoubleArrayFromStringArray(new string[] { boxRoll.Text, boxPitch.Text, boxYaw.Text });
        }
    }
}