using log4net;
using System.Text;
using System.Xml;

namespace SW2URDF.URDF
{
    //A class that just writes the bare minimum of the manifest file necessary for ROS packages.
    public class PackageXMLWriter
    {
        public XmlWriter writer;
        private static readonly ILog logger = Logger.GetLogger();

        public PackageXMLWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Encoding = new UTF8Encoding(false);
            settings.OmitXmlDeclaration = true;
            settings.Indent = true;
            settings.NewLineOnAttributes = false;
            logger.Info("Creating package.xml at " + savePath);
            writer = XmlWriter.Create(savePath, settings);
        }
    }

    //The base class for packageXML elements. Again, I guess I like having empty base classes
    public class PackageElement
    {
        public PackageElement()
        {
        }

        public void WriteElement()
        {
        }
    }

    //Top level class for the package XML file.
    public class PackageXML : PackageElement
    {
        public Description description;
        public Dependencies dependencies;
        public Author author;
        public License license;

        public PackageXML(string name)
        {
            description = new Description(name);

            dependencies = new Dependencies(
                new string[] { "catkin" },
                new string[] {
                    "roslaunch", "robot_state_publisher", "rviz", "joint_state_publisher", "gazebo" });

            author = new Author("TODO");

            license = new License("BSD");
        }

        public void WriteElement(PackageXMLWriter mWriter)
        {
            XmlWriter writer = mWriter.writer;
            writer.WriteStartDocument();
            writer.WriteStartElement("package");

            description.WriteElement(writer);

            author.WriteElement(writer);
            license.WriteElement(writer);
            dependencies.WriteElement(writer);

            writer.WriteStartElement("export");

            writer.WriteStartElement("architecture_independent");
            writer.WriteEndElement();

            writer.WriteEndElement();

            writer.WriteEndElement();
            writer.WriteEndDocument();
            writer.Close();
        }
    }

    //description element of the manifest file
    public class Description : PackageElement
    {
        private readonly string name;
        private readonly string brief;
        private readonly string longDescription;

        public Description(string name)
        {
            this.name = name;
            brief = "URDF Description package for " + name;
            longDescription = "This package contains configuration data, 3D models and launch files\r\n" +
                                    "for " + name + " robot";
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("name");
            writer.WriteString(name);
            writer.WriteEndElement();

            writer.WriteStartElement("version");
            writer.WriteString("1.0.0");
            writer.WriteEndElement();

            writer.WriteStartElement("description");

            writer.WriteStartElement("p");
            writer.WriteString(brief);
            writer.WriteEndElement();

            writer.WriteStartElement("p");
            writer.WriteString(longDescription);
            writer.WriteEndElement();

            writer.WriteEndElement();
        }
    }

    //The depend element of the manifest file
    public class Dependencies : PackageElement
    {
        private readonly string[] buildTool;
        private readonly string[] buildExec;

        public Dependencies(string[] buildTool, string[] buildExec)
        {
            this.buildTool = buildTool;
            this.buildExec = buildExec;
        }

        public void WriteElement(XmlWriter writer)
        {
            foreach (string depend in buildTool)
            {
                writer.WriteStartElement("buildtool_depend");
                writer.WriteString(depend);
                writer.WriteEndElement();
            }

            foreach (string depend in buildExec)
            {
                writer.WriteStartElement("depend");
                writer.WriteString(depend);
                writer.WriteEndElement();
            }
        }
    }

    //The author element of the manifest file
    public class Author : PackageElement
    {
        private readonly string name;

        public Author(string name)
        {
            this.name = name;
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("author");
            writer.WriteString(name);
            writer.WriteEndElement();

            writer.WriteStartElement("maintainer");
            writer.WriteAttributeString("email", name + "@email.com");
            writer.WriteEndElement();
        }
    }

    //The license element of the manifest file
    public class License : PackageElement
    {
        private readonly string lic;

        public License(string lic)
        {
            this.lic = lic;
        }

        public void WriteElement(XmlWriter writer)
        {
            writer.WriteStartElement("license");
            writer.WriteString(lic);
            writer.WriteEndElement();
        }
    }
}