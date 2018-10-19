using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.Legacy;
using SW2URDF.URDF;
using SW2URDF.Utilities;
using System;
using System.IO;
using System.Runtime.Serialization;
using System.Text;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;

namespace SW2URDF.URDFExport
{
    /// <summary>
    /// Class to serialize URDF trees to string so they can be saved to an SW Attribute in the
    /// top-level assembly document.
    ///
    /// Any changes to the serialization scheme need to support backwards compatibility in some way.
    /// At least in regards to reading the old configuration. I'm also choosing to save any old xml
    /// formats to a second attribute in case they need to be reloaded.
    /// </summary>
    public static class Serialization
    {
        private static readonly log4net.ILog logger = Logger.GetLogger();

        /// <summary>
        /// Current Serialization version
        /// </summary>
        private const double SERIALIZATION_VERSION = 1.3;

        /// <summary>
        /// Previous versions of serialization were set at 1 in the SW Document. This is
        /// used to denote the version at which the serialization scheme was modified.
        /// </summary>
        private const double MIN_DATA_CONTRACT_VERSION = 1.3;

        /// <summary>
        /// The name given to the URDF configuration in the ModelDoc Feature tree. This is displayed to the
        /// user
        /// </summary>
        public const string URDF_CONFIGURATION_SW_ATTRIBUTE_NAME = "URDF Export Configuration (v1.3)";

        public const string V1_URDF_CONFIGURATION_ATTRIBUTE_NAME = "URDF Export Configuration";

        #region Public Methods

        /// <summary>
        /// Loads the URDF tree from the SW Model Document
        /// </summary>
        /// <param name="model">ModelDoc containing the URDF configuration</param>
        /// <returns>TreeView LinkNode loaded from configuration</returns>
        public static LinkNode LoadBaseNodeFromModel(SldWorks swApp, ModelDoc2 model, out bool error)
        {
            string data = GetConfigTreeData(model, out double configVersion);

            LinkNode basenode = null;
            if (configVersion > SERIALIZATION_VERSION)
            {
                MessageBox.Show("The configuration saved in this model is newer than what this " +
                    "exporter supports " + string.Format("({0} > {1})", configVersion, SERIALIZATION_VERSION) +
                    ". Please update your exporter version");
                error = true;
                return null;
            }

            if (configVersion >= MIN_DATA_CONTRACT_VERSION)
            {
                basenode = DeserializeFromString(data);
            }
            else
            {
                basenode = LoadConfigFromStringXML(data);
            }

            error = false;
            return basenode;
        }

        /// <summary>
        /// Public method to serialize a Treeview LinkNode URDF data to a string and saves it to a SW ModelDoc
        /// </summary>
        /// <param name="swApp">SldWorks application</param>
        /// <param name="model">ModelDoc to which you are saving this data</param>
        /// <param name="BaseNode">TreeView LinkNode which contains the data you are saving</param>
        /// <param name="warnUser">Warn the user with a YesNo MessageBox, otherwise this will be done silently
        ///  and overwrite any existing data</param>
        public static void SaveConfigTreeXML(SldWorks swApp, ModelDoc2 model, LinkNode BaseNode, bool warnUser)
        {
            string oldData = GetConfigTreeData(model, out double version);
            if (version < SERIALIZATION_VERSION)
            {
                MessageBox.Show("You have a URDF configuration with an outdated save format. It will automatically be " +
                    "upgraded to the latest version and saved to the configuration named \"" +
                    URDF_CONFIGURATION_SW_ATTRIBUTE_NAME + "\". " +
                    "Old configurations can be deleted at your convenience.");
                warnUser = false;
            }

            string newData = SerializeToString(BaseNode);
            if (BaseNode != null && string.IsNullOrEmpty(newData))
            {
                MessageBox.Show("Serializing this link failed. Please email your maintainer with your SW assembly.");
                return;
            }
            if (oldData != newData)
            {
                if (!warnUser ||
                    (warnUser &&
                    MessageBox.Show("The configuration has changed, would you like to save?",
                    "Save Export Configuration", MessageBoxButtons.YesNo) == DialogResult.Yes))
                {
                    SaveDataToModelDoc(swApp, model, newData);
                }
            }
        }

        #endregion Public Methods

        #region Private Methods

        /// <summary>
        /// If someone updates the name of a LinkNode in the Treeview, it needs to be pushed down
        /// to the URDF Link itself.
        /// </summary>
        /// <param name="node">TreeView LinkNode to save properties of to its URDF Link</param>
        private static void SavePropertiesLinkNodeToLink(LinkNode node)
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

        /// <summary>
        /// Data Contract serialization. All members of an object need to be annotated with a
        /// [DataMember] attribute.
        /// </summary>
        /// <param name="node">TreeView LinkNode to serialize</param>
        /// <returns>A string serialized utilizing DataContract serialization XML scheme</returns>
        private static string SerializeToString(LinkNode node)
        {
            SavePropertiesLinkNodeToLink(node);
            Link link = node.UpdateLinkTree(null);
            string data = "";
            using (MemoryStream stream = new MemoryStream())
            {
                DataContractSerializer ser =
                    new DataContractSerializer(typeof(Link));

                try
                {
                    ser.WriteObject(stream, link);
                    stream.Flush();
                    data = Encoding.ASCII.GetString(stream.GetBuffer(), 0, (int)stream.Position);
                }
                catch (SerializationException e)
                {
                    logger.Error("Serialization failed with exception, returning empty string", e);
                }
            }
            return data;
        }

        /// <summary>
        /// Read a URDF Link from a serialized string
        /// </summary>
        /// <param name="data">Data string to read into a TreeView LinkNode</param>
        /// <returns>Deserialized LinkNode</returns>
        private static LinkNode DeserializeFromString(string data)
        {
            LinkNode baseNode = null;
            if (!string.IsNullOrWhiteSpace(data))
            {
                using (MemoryStream stream = new MemoryStream(Encoding.ASCII.GetBytes(data)))
                {
                    DataContractSerializer ser =
                        new DataContractSerializer(typeof(Link));

                    try
                    {
                        XmlDictionaryReader reader = XmlDictionaryReader.CreateDictionaryReader(XmlReader.Create(stream));
                        SerializationResolver resolver = new SerializationResolver();
                        Link link = (Link)ser.ReadObject(reader, false, resolver);
                        baseNode = new LinkNode(link);
                    }
                    catch (SerializationException e)
                    {
                        logger.Error("Deserialization failed with exception, returning empty LinkNode", e);
                        logger.Error(data);
                    }
                }
            }
            return baseNode;
        }

        /// <summary>
        /// Load from the deprecated XML serialized scheme
        /// </summary>
        /// <param name="data">Data string to deserialize using XMLSerializer</param>
        /// <returns>TreeView LinkNode</returns>
        private static LinkNode LoadConfigFromStringXML(string data)
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

        /// <summary>
        /// For the future, if serialization is upgraded again, this might look for several versions
        /// </summary>
        /// <param name="model">ModelDoc to look through</param>
        /// <returns>SolidWorks Attribute with older serialization schemes if found, otherwise null.</returns>
        private static SolidWorks.Interop.sldworks.Attribute CheckForOldAttributes(ModelDoc2 model)
        {
            SolidWorks.Interop.sldworks.Attribute swAtt =
                FindSWSaveAttribute(model, V1_URDF_CONFIGURATION_ATTRIBUTE_NAME);

            return swAtt;
        }

        /// <summary>
        /// Find the SW attribute that contains the URDF configuration serialized string
        /// </summary>
        /// <param name="model">ModelDoc model to load URDF configuration from</param>
        /// <param name="version">Output parameter of the serialization version</param>
        /// <returns>Serialized data string</returns>
        private static string GetConfigTreeData(ModelDoc2 model, out double version)
        {
            object[] objects = model.FeatureManager.GetFeatures(true);
            string data = "";
            version = 0.0;

            // Check for most recent serialization version
            SolidWorks.Interop.sldworks.Attribute swAtt =
                FindSWSaveAttribute(model, URDF_CONFIGURATION_SW_ATTRIBUTE_NAME);

            // If not found, check for an older version
            if (swAtt == null)
            {
                swAtt = CheckForOldAttributes(model);
            }

            if (swAtt != null)
            {
                Parameter param = swAtt.GetParameter("data");
                data = param.GetStringValue();
                logger.Info("URDF Configuration found\n" + data);

                param = swAtt.GetParameter("exporterVersion");
                version = param.GetDoubleValue();
            }

            return data;
        }

        /// <summary>
        ///  Iterates through features in ModelDoc to find a feature of the correct name
        /// </summary>
        /// <param name="model">ModelDoc of features to iterate through</param>
        /// <param name="featName">The name of the feature to get</param>
        /// <returns>The SolidWorks Feature if found, null otherwise</returns>
        private static Feature GetFeatureAttributeByName(ModelDoc2 model, string featName)
        {
            Object[] objects = model.FeatureManager.GetFeatures(true);
            foreach (Object obj in objects)
            {
                Feature feature = (Feature)obj;
                string t = feature.GetTypeName2();
                if (feature.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att =
                        (SolidWorks.Interop.sldworks.Attribute)feature.GetSpecificFeature2();
                    if (att.GetName() == featName)
                    {
                        return feature;
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Finds existing SWSave Attribute in a ModelDoc
        /// </summary>
        /// <param name="model">ModelDoc model to search through</param>
        /// <param name="name">Name of attribute to find</param>
        /// <returns>SolidWorks Attribute if found, otherwise null</returns>
        private static SolidWorks.Interop.sldworks.Attribute
            FindSWSaveAttribute(ModelDoc2 model, string name)
        {
            Feature feature = GetFeatureAttributeByName(model, name);

            if (feature == null)
            {
                return null;
            }
            return (SolidWorks.Interop.sldworks.Attribute)feature.GetSpecificFeature2();
        }

        /// <summary>
        /// Builds a SW Attribute for saving our serialized data
        /// </summary>
        /// <param name="swApp">SolidWorks Application to build Feature Definition</param>
        /// <param name="model">ModelDoc in which this attribute will be saved</param>
        /// <param name="name">Name of the attribute to create</param>
        /// <returns>Constructed SolidWorks Attribute</returns>
        private static SolidWorks.Interop.sldworks.Attribute
            CreateSWSaveAttribute(SldWorks swApp, ModelDoc2 model, string name)
        {
            SolidWorks.Interop.sldworks.Attribute existingAttribute =
                FindSWSaveAttribute(model, name);
            if (existingAttribute != null)
            {
                return existingAttribute;
            }

            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;

            int Options = 0;
            AttributeDef saveConfigurationAttributeDef;
            saveConfigurationAttributeDef = swApp.DefineAttribute(URDF_CONFIGURATION_SW_ATTRIBUTE_NAME);

            saveConfigurationAttributeDef.AddParameter(
                "data", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "name", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "date", (int)swParamType_e.swParamTypeString, 0, Options);
            saveConfigurationAttributeDef.AddParameter(
                "exporterVersion", (int)swParamType_e.swParamTypeDouble, SERIALIZATION_VERSION, Options);
            saveConfigurationAttributeDef.Register();

            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                saveConfigurationAttributeDef.CreateInstance5(
                    model, null, URDF_CONFIGURATION_SW_ATTRIBUTE_NAME, Options, ConfigurationOptions);
            return saveExporterAttribute;
        }

        /// <summary>
        /// Saves a string of data to the SWModelDoc
        /// </summary>
        /// <param name="swApp">SolidWorks Application</param>
        /// <param name="model">ModelDoc model to save data string to</param>
        /// <param name="data">string to save</param>
        /// <param name="attributeName">Name of attribute to save to</param>
        private static void SaveDataToModelDoc(SldWorks swApp, ModelDoc2 model,
            string data, string archivedData = null)
        {
            int ConfigurationOptions = (int)swInConfigurationOpts_e.swAllConfiguration;
            SolidWorks.Interop.sldworks.Attribute saveExporterAttribute =
                CreateSWSaveAttribute(swApp, model, URDF_CONFIGURATION_SW_ATTRIBUTE_NAME);

            Parameter param = saveExporterAttribute.GetParameter("data");
            param.SetStringValue2(data, ConfigurationOptions, "");
            param = saveExporterAttribute.GetParameter("name");
            param.SetStringValue2("config1", ConfigurationOptions, "");
            param = saveExporterAttribute.GetParameter("date");
            param.SetStringValue2(DateTime.Now.ToString(), ConfigurationOptions, "");
            param = saveExporterAttribute.GetParameter("exporterVersion");
            param.SetDoubleValue2(SERIALIZATION_VERSION, ConfigurationOptions, "");
        }

        #endregion Private Methods
    }

    public class SerializationResolver : DataContractResolver
    {
        public override bool TryResolveType(Type dataContractType, Type declaredType, DataContractResolver knownTypeResolver, out XmlDictionaryString typeName, out XmlDictionaryString typeNamespace)
        {
            return knownTypeResolver.TryResolveType(dataContractType, declaredType, null, out typeName, out typeNamespace);
        }

        public override Type ResolveName(string typeName, string typeNamespace, Type declaredType, DataContractResolver knownTypeResolver)
        {
            try
            {
                return knownTypeResolver.ResolveName(typeName, typeNamespace, declaredType, null);
            }
            catch
            {
                return null;
            }
        }
    }
}