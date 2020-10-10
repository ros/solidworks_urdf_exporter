using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The texture element of the material element.
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Texture : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute FilenameAttribute;

        public string Filename
        {
            get => (string)FilenameAttribute.Value;
            set => FilenameAttribute.Value = value;
        }

        [DataMember]
        public string wFilename;

        public Texture() : base("texture", false)
        {
            wFilename = "";
            FilenameAttribute = new URDFAttribute("filename", true, null);

            Attributes.Add(FilenameAttribute);
        }
    }
}