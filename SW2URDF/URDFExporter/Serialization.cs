using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Text;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.Legacy;

namespace SW2URDF
{
    public static class Serialization
    {
        private static readonly log4net.ILog logger = Logger.GetLogger();
        public static readonly double SERIALIZATION_VERSION = 1.3;
        public static readonly double MIN_DATA_CONTRACT_VERSION = 1.3;

        public static void SavePropertiesLinkNodeToLink(LinkNode node)
        {
            if (node.Link == null)
            {
                node.Link = new Link();
                return;
            }

            node.Link.Name = node.Name;

            foreach (LinkNode child in node.Nodes)
            {
                SavePropertiesLinkNodeToLink(child);
            }
        }

        public static string SerializeToString(LinkNode node)
        {
            // TODO need to make sure this contains all the info from the LinkNode
            SavePropertiesLinkNodeToLink(node);
            Link link = node.UpdateLinkTree(null);
            string data = "";
            using (MemoryStream stream = new MemoryStream())
            {
                DataContractSerializer ser =
                    new DataContractSerializer(typeof(Link));
                ser.WriteObject(stream, link);
                stream.Flush();
                data = Encoding.ASCII.GetString(stream.GetBuffer(), 0, (int)stream.Position);
            }
            return data;
        }

        public static LinkNode DeserializeFromString(string data)
        {
            LinkNode baseNode = null;
            if (!string.IsNullOrWhiteSpace(data))
            {
                using (MemoryStream stream = new MemoryStream(Encoding.ASCII.GetBytes(data)))
                {
                    DataContractSerializer ser =
                        new DataContractSerializer(typeof(Link));
                    Link link = (Link)ser.ReadObject(stream);
                    baseNode = new LinkNode(link);
                }
            }
            return baseNode;
        }

        public static LinkNode LoadBaseNodeFromModel(ModelDoc2 model)
        {
            string data = GetConfigTreeData(model, out double configVersion);

            LinkNode basenode = null;
            if (configVersion >= MIN_DATA_CONTRACT_VERSION)
            {
                basenode = DeserializeFromString(data);
            }
            else
            {
                basenode = LoadConfigFromStringXML(data);
            }

            return basenode;
        }

        public static LinkNode LoadConfigFromStringXML(string data)
        {
            LinkNode baseNode = null;
            if (!string.IsNullOrWhiteSpace(data))
            {
                XmlSerializer serializer = new XmlSerializer(typeof(SerialNode));
                XmlTextReader textReader = new XmlTextReader(new StringReader(data));
                SerialNode sNode = (SerialNode)serializer.Deserialize(textReader);
                textReader.Close();

                baseNode = sNode.BuildLinkNodeFromSerialNode();
            }
            return baseNode;
        }

        public static string GetConfigTreeData(ModelDoc2 model, out double version)
        {
            object[] objects = model.FeatureManager.GetFeatures(true);
            string data = "";
            version = 0.0;
            foreach (object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        Parameter param = att.GetParameter("data");
                        data = param.GetStringValue();
                        logger.Info("URDF Configuration found\n" + data);

                        param = att.GetParameter("exporterVersion");
                        version = param.GetDoubleValue();
                    }
                }
            }
            return data;
        }

        public static SolidWorks.Interop.sldworks.Attribute
            CreateSWSaveAttribute(SldWorks swApp, ModelDoc2 model, string name)
        {
            int Options = 0;
            AttributeDef saveConfigurationAttributeDef;
            saveConfigurationAttributeDef = swApp.DefineAttribute("URDF Export Configuration");

            saveConfigurationAttributeDef.AddParameter(
                "data", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "name", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "date", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "exporterVersion", (int)swParamType_e.swParamTypeDouble, SERIALIZATION_VERSION, Options);
            saveConfigurationAttributeDef.Register();

            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            ModelDoc2 modeldoc = swApp.ActiveDoc;
            Object[] objects = modeldoc.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == name)
                    {
                        return att;
                    }
                }
            }
            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                saveConfigurationAttributeDef.CreateInstance5(
                    model, null, "URDF Export Configuration", Options, ConfigurationOptions);
            return saveExporterAttribute;
        }

        public static void SaveConfigTreeXML(SldWorks swApp, ModelDoc2 model, LinkNode BaseNode, bool warnUser)
        {
            string oldData = GetConfigTreeData(model, out double version);

            string newData = SerializeToString(BaseNode);
            if (oldData != newData)
            {
                if (!warnUser ||
                    (warnUser &&
                    MessageBox.Show("The configuration has changed, would you like to save?",
                    "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes))
                {
                    int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
                    SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                        CreateSWSaveAttribute(swApp, model, "URDF Export Configuration");
                    Parameter param = saveExporterAttribute.GetParameter("data");
                    param.SetStringValue2(newData, ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("name");
                    param.SetStringValue2("config1", ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("date");
                    param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
                    param = saveExporterAttribute.GetParameter("exporterVersion");
                    param.SetDoubleValue2(SERIALIZATION_VERSION, ConfigurationOptions, "");
                }
            }
        }
    }
}