using System.Collections.Generic;
using System.Collections.Specialized;
using System.Runtime.Serialization;
using System.Windows.Forms;

namespace SW2URDF.URDF
{
    //The joint class. There is one for every link but the base link
    [DataContract(IsReference = true)]
    public class Joint : URDFElement
    {
        public static readonly List<string> AVAILABLE_TYPES = new List<string>
        {
            "revolute", "continuous", "prismatic", "fixed", "floating", "planar"
        };

        [DataMember]
        private readonly URDFAttribute NameAttribute;

        public string Name
        {
            get
            {
                return (string)NameAttribute.Value;
            }
            set
            {
                NameAttribute.Value = value;
            }
        }

        [DataMember]
        private readonly URDFAttribute TypeAttribute;

        public string Type
        {
            get
            {
                return (string)TypeAttribute.Value;
            }
            set
            {
                TypeAttribute.Value = value;
            }
        }

        [DataMember]
        public readonly Origin Origin;

        [DataMember]
        public readonly ParentLink Parent;

        [DataMember]
        public readonly ChildLink Child;

        [DataMember]
        public readonly Axis Axis;

        [DataMember]
        public readonly Limit Limit;

        [DataMember]
        public readonly Calibration Calibration;

        [DataMember]
        public readonly Dynamics Dynamics;

        [DataMember]
        public readonly SafetyController Safety;

        [DataMember]
        public string CoordinateSystemName;

        [DataMember]
        public string AxisName;

        public Joint() : base("joint", false)
        {
            Origin = new Origin("Joint", false);
            Parent = new ParentLink();
            Child = new ChildLink();
            Axis = new Axis();

            Limit = new Limit();
            Calibration = new Calibration();
            Dynamics = new Dynamics();
            Safety = new SafetyController();

            NameAttribute = new URDFAttribute("name", true, "");
            TypeAttribute = new URDFAttribute("type", true, "");

            Attributes.Add(NameAttribute);
            Attributes.Add(TypeAttribute);

            ChildElements.Add(Origin);
            ChildElements.Add(Parent);
            ChildElements.Add(Child);
            ChildElements.Add(Axis);

            ChildElements.Add(Limit);
            ChildElements.Add(Calibration);
            ChildElements.Add(Dynamics);
            ChildElements.Add(Safety);
        }

        public void FillBoxes(TextBox boxName, ComboBox boxType)
        {
            boxName.Text = Name;
            boxType.Text = Type;
        }

        public void Update(TextBox boxName, ComboBox boxType)
        {
            Name = boxName.Text;
            Type = boxType.Text;
        }

        public override bool ElementContainsData()
        {
            return !string.IsNullOrWhiteSpace(Name) && !string.IsNullOrWhiteSpace(Type);
        }

        public override bool AreRequiredFieldsSatisfied()
        {
            Limit.SetRequired((Type == "prismatic" || Type == "revolute"));
            return base.AreRequiredFieldsSatisfied();
        }

        public override void AppendToCSVDictionary(List<string> context, OrderedDictionary dictionary)
        {
            string contextString = string.Join(".", context);

            string coordSysContext = contextString + ".CoordSysName";
            dictionary.Add(coordSysContext, CoordinateSystemName);

            string axisContext = contextString + ".AxisName";
            dictionary.Add(axisContext, AxisName);

            base.AppendToCSVDictionary(context, dictionary);
        }

        public override void SetElement(URDFElement externalElement)
        {
            base.SetElement(externalElement);

            // The base method already performs the type check, so we don't have to for this cast
            Joint joint = (Joint)externalElement;

            // These strings aren't kept as URDFAttribute objects and so they are tracked separately
            CoordinateSystemName = joint.CoordinateSystemName;
            AxisName = joint.AxisName;
        }

        public override void SetElementFromData(List<string> context, StringDictionary dictionary)
        {
            string contextString = string.Join(".", context);

            string coordSysContext = contextString + ".CoordSysName";
            CoordinateSystemName = dictionary[coordSysContext];

            string axisContext = contextString + ".AxisName";
            AxisName = dictionary[axisContext];

            base.SetElementFromData(context, dictionary);
        }

        public void SetJointKinematics(Joint joint)
        {
            CoordinateSystemName = joint.CoordinateSystemName;
            AxisName = joint.AxisName;
            Type = joint.Type;
            Axis.SetElement(joint.Axis);
            Origin.SetElement(joint.Origin);
        }

        public void SetJointNonKinematics(Joint joint)
        {
            Limit.SetElement(joint.Limit);
            Calibration.SetElement(joint.Calibration);
            Dynamics.SetElement(joint.Dynamics);
            Safety.SetElement(joint.Safety);
        }
    }
}