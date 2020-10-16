using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The mesh element of the geometry element. This contains only a filename location of the mesh.
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Mesh : URDFElement
    {
        [DataMember]
        private readonly URDFAttribute FilenameAttribute;

        public string Filename
        {
            get => (string)FilenameAttribute.Value;
            set => FilenameAttribute.Value = value;
        }

        public Mesh() : base("mesh", false)
        {
            FilenameAttribute = new URDFAttribute("filename", true, null);

            Attributes.Add(FilenameAttribute);
        }
    }
}