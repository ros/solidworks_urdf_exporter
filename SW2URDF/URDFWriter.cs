using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;

namespace SW2URDF
{
    class URDFWriter
    {
        public XmlWriter writer;
        public URDFWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Indent = true;
            settings.NewLineOnAttributes = true;
            writer = XmlWriter.Create(savePath, settings);
                       
        }

        public robot[] children;
        public void writeURDF()
        {
            writer.WriteStartDocument();
            for (int i = 0; i < children.Length; i++)
            {
                children[i].writeURDF();
            }
            writer.WriteEndDocument();
            writer.Close();
        }

        //<origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
        //<color rgba="1 1 1 1"/>
        public void writeURDFFromLink(link Link)
        {
            writer.WriteStartDocument();
            writeOneLink(Link);
            writer.WriteEndDocument();
            writer.Close();
        }
        public void writeOneLink(link Link)
        {
           



            



            



            writer.WriteEndElement(); //inertial

            writer.WriteStartElement("visual");
            writer.WriteStartElement("origin");
            writer.WriteAttributeString("xyz", Link.origin_visual[0].ToString() + " " + Link.origin_visual[1].ToString() + " " + Link.origin_visual[2].ToString());
            writer.WriteAttributeString("rpy", Link.origin_visual[3].ToString() + " " + Link.origin_visual[4].ToString() + " " + Link.origin_visual[5].ToString());
            writer.WriteEndElement(); //origin

            writer.WriteStartElement("geometry");
            writer.WriteStartElement("mesh");
            writer.WriteAttributeString("filename", Link.meshName);
            writer.WriteEndElement(); //mesh
            writer.WriteEndElement(); //geometry

            writer.WriteStartElement("material");
            if (Link.material == "Custom Color")
            {
                writer.WriteStartElement("color");
                writer.WriteAttributeString("rgba", Link.rgba[0].ToString() + " " + Link.rgba[1].ToString() + " " + Link.rgba[2].ToString() + " " + Link.rgba[3].ToString());
                writer.WriteEndElement();
            }
            else if (Link.material == "Texture")
            {
                writer.WriteStartElement("texture");
                writer.WriteAttributeString("filename", Link.texture);
                writer.WriteEndElement();
            }
            else
            {
                writer.WriteAttributeString("name", Link.material);
            }
            writer.WriteEndElement();

            writer.WriteEndElement(); //visual

            writer.WriteStartElement("collision");

            writer.WriteEndElement(); //collision
            writer.WriteStartElement("origin");
            writer.WriteAttributeString("xyz", Link.origin_collision[0].ToString() + " " + Link.origin_collision[1].ToString() + " " + Link.origin_collision[2].ToString());
            writer.WriteAttributeString("rpy", Link.origin_collision[3].ToString() + " " + Link.origin_collision[4].ToString() + " " + Link.origin_collision[5].ToString());
            writer.WriteEndElement(); //origin

            writer.WriteStartElement("geometry");
            writer.WriteStartElement("mesh");
            writer.WriteAttributeString("filename", Link.meshName);
            writer.WriteEndElement(); //mesh
            writer.WriteEndElement(); //geometry

            writer.WriteEndElement(); //link
            writer.WriteEndElement(); //robot
        }
    }
}
