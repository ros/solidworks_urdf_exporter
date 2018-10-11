using log4net;
using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Xml;

namespace SW2URDF.URDF
{
    // Base class of each URDFElement. The goal is to minimize the amount of code in the derived classes;s
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    [KnownType("GetKnownTypes")]
    public class URDFElement// : ISerializable
    {
        protected static readonly ILog logger = Logger.GetLogger();

        [DataMember]
        protected readonly List<URDFElement> ChildElements;

        [DataMember]
        protected readonly List<URDFAttribute> Attributes;

        [DataMember]
        protected readonly string ElementName;

        [DataMember]
        protected bool required;

        public URDFElement(string elementName, bool required)
        {
            ElementName = elementName;
            this.required = required;
            ChildElements = new List<URDFElement>();
            Attributes = new List<URDFAttribute>();
        }

        public bool IsRequired()
        {
            return required;
        }

        public virtual void SetRequired(bool required)
        {
            this.required = required;
        }

        public virtual void WriteURDF(XmlWriter writer)
        {
            if (!AreRequiredFieldsSatisfied())
            {
                throw new Exception("The required fields of the element " + ElementName + " have not been satisfied");
            }

            if (!ElementContainsData())
            {
                return;
            }

            writer.WriteStartElement(ElementName);
            foreach (URDFAttribute attribute in Attributes)
            {
                if (attribute.Value != null)
                {
                    attribute.WriteURDF(writer);
                }
            }

            foreach (URDFElement child in ChildElements)
            {
                if (child.ElementContainsData())
                {
                    child.WriteURDF(writer);
                }
            }

            writer.WriteEndElement();
        }

        public virtual void AppendToCSVDictionary(List<string> context, OrderedDictionary dictionary)
        {
            string typeName = GetType().Name;
            List<string> updatedContext = new List<string>(context) { typeName };

            foreach (URDFAttribute att in Attributes)
            {
                if (att.Value != null)
                {
                    att.AppendToCSVDictionary(updatedContext, dictionary);
                }
            }

            foreach (URDFElement child in ChildElements)
            {
                child.AppendToCSVDictionary(updatedContext, dictionary);
            }
        }

        public virtual void SetElementFromData(List<string> context, StringDictionary dictionary)
        {
            string typeName = GetType().Name;
            List<string> updatedContext = new List<string>(context) { typeName };

            foreach (URDFAttribute att in Attributes)
            {
                att.SetValueFromData(updatedContext, dictionary);
            }

            foreach (URDFElement child in ChildElements)
            {
                child.SetElementFromData(updatedContext, dictionary);
            }
        }

        public void Unset()
        {
            foreach (URDFAttribute attribute in Attributes)
            {
                attribute.Value = null;
            }
            foreach (URDFElement child in ChildElements)
            {
                child.Unset();
            }
        }

        public virtual bool AreRequiredFieldsSatisfied()
        {
            foreach (URDFAttribute attribute in Attributes)
            {
                if (attribute.GetIsRequired() && attribute.Value == null)
                {
                    return false;
                }
            }
            foreach (URDFElement child in ChildElements)
            {
                if (!child.AreRequiredFieldsSatisfied() &&
                    (child.IsRequired() || child.ElementContainsData()))
                {
                    return false;
                }
            }
            return true;
        }

        public virtual bool ElementContainsData()
        {
            foreach (URDFAttribute attribute in Attributes)
            {
                if (attribute.Value != null)
                {
                    return true;
                }
            }

            foreach (URDFElement child in ChildElements)
            {
                if (child.ElementContainsData())
                {
                    return true;
                }
            }
            return false;
        }

        public static Type[] GetKnownTypes()
        {
            return new List<Type>(
                Assembly.GetExecutingAssembly().GetTypes().Where(_ => _.IsSubclassOf(typeof(URDFElement))))
            {
                typeof(double[])
            }.ToArray();
        }

        public virtual void SetElement(URDFElement externalElement)
        {
            if (externalElement.GetType() != GetType())
            {
                throw new Exception("URDFElements need to be the same type to set the internal values");
            }

            foreach (Tuple<URDFAttribute, URDFAttribute> pair in
                Enumerable.Zip(Attributes, externalElement.Attributes, Tuple.Create))
            {
                if (pair.Item2.Value != null && pair.Item2.Value.GetType() == typeof(double[]))
                {
                    double[] valueArray = (double[])pair.Item2.Value;
                    pair.Item1.Value = valueArray.Clone();
                }
                else
                {
                    pair.Item1.Value = pair.Item2.Value;
                }
            }

            foreach (Tuple<URDFElement, URDFElement> pair in
                Enumerable.Zip(ChildElements, externalElement.ChildElements, Tuple.Create))
            {
                pair.Item1.SetElement(pair.Item2);
            }
        }
    }
}