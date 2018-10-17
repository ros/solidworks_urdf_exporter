using System;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.Globalization;
using System.Runtime.Serialization;
using System.Xml;

namespace SW2URDF.URDF
{
    [DataContract(IsReference = true, Name = "Attribute", Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class URDFAttribute
    {
        public static readonly NumberFormatInfo URDFNumberFormat =
            CultureInfo.CreateSpecificCulture("en-US").NumberFormat;

        public static readonly NumberStyles URDFNumberStyle = NumberStyles.Any;

        [DataMember]
        private bool IsRequired;

        [DataMember]
        private readonly string AttributeType;

        [DataMember]
        public object Value;

        public URDFAttribute(string type, bool isRequired, object initialValue)
        {
            AttributeType = type;
            IsRequired = isRequired;
            Value = initialValue;
        }

        public void WriteURDF(XmlWriter writer)
        {
            string valueString = "";
            if (Value.GetType() == typeof(double[]))
            {
                double[] valueArray = (double[])Value;
                foreach (double d in valueArray)
                {
                    valueString +=
                        d.ToString(URDFNumberFormat) + " ";
                }
                valueString = valueString.Trim();
            }
            else if (Value.GetType() == typeof(double))
            {
                valueString =
                    ((Double)Value).ToString(URDFNumberFormat);
            }
            else if (Value.GetType() == typeof(string))
            {
                valueString = (string)Value;
            }
            else if (Value != null)
            {
                throw new Exception("Unhandled object type in write attribute");
            }
            if (IsRequired && Value == null)
            {
                throw new Exception("Required attribute " + AttributeType + " has null value");
            }
            if (String.IsNullOrWhiteSpace(AttributeType))
            {
                throw new Exception("No type specified");
            }
            if (Value != null)
            {
                writer.WriteAttributeString(AttributeType, valueString);
            }
        }

        public void AppendToCSVDictionary(List<string> context, OrderedDictionary dictionary)
        {
            if (Value.GetType() == typeof(double[]))
            {
                double[] values = (double[])Value;
                for (int i = 0; i < values.Length; i++)
                {
                    string contextString = string.Join(".", context) + "." + AttributeType + "." + AttributeType[i];
                    dictionary.Add(contextString, values[i]);
                }
            }
            else
            {
                string contextString = string.Join(".", context) + "." + AttributeType;
                dictionary.Add(contextString, Value);
            }
        }

        public void SetValueFromData(List<string> context, StringDictionary dictionary)
        {
            string contextString = string.Join(".", context) + "." + AttributeType;
            if (dictionary.ContainsKey(contextString))
            {
                Value = GetValueFromString(dictionary[contextString]);
            }
        }

        public static object GetValueFromString(string valueStr)
        {
            if (valueStr == null)
            {
                return null;
            }
            double resultDouble;
            if (valueStr.Contains(";"))
            {
                string[] valueFields = valueStr.Split(';');
                double[] arry = new double[valueFields.Length];
                for (int i = 0; i < valueFields.Length; i++)
                {
                    if (!Double.TryParse(valueFields[i], out resultDouble))
                    {
                        throw new Exception("Parsing failed for CSV field " + valueStr);
                    }
                    arry[i] = resultDouble;
                }
                return arry;
            }
            else
            {
                if (Double.TryParse(valueStr, out resultDouble))
                {
                    return resultDouble;
                }
            }
            return valueStr;
        }

        public bool GetIsRequired()
        {
            return IsRequired;
        }

        public void SetRequired(bool required)
        {
            IsRequired = required;
        }

        public bool IsSet()
        {
            return Value != null;
        }

        public string GetTextFromDoubleValue(string format = "G")
        {
            string result = "";
            if (Value != null && Value.GetType() == typeof(double))
            {
                double dValue = (double)Value;
                result = dValue.ToString(format, URDFNumberFormat);
            }
            return result;
        }

        public string[] GetTextArrayFromDoubleArray(string format = "G")
        {
            string[] result = null;
            if (Value != null)
            {
                double[] dArray = (double[])Value;
                result = new string[dArray.Length];
                for (int i = 0; i < dArray.Length; i++)
                {
                    result[i] = dArray[i].ToString(format, URDFNumberFormat);
                }
            }
            return result;
        }

        public void SetDoubleValueFromString(string text)
        {
            if (Double.TryParse(text, URDFNumberStyle, URDFNumberFormat, out double result))
            {
                Value = result;
            }
            else if (GetIsRequired())
            {
                Value = 0.0;
            }
        }

        public void SetDoubleArrayFromStringArray(string[] textArray)
        {
            double[] dArray = new double[textArray.Length];
            for (int i = 0; i < textArray.Length; i++)
            {
                if (Double.TryParse(textArray[i], URDFNumberStyle, URDFNumberFormat, out double result))
                {
                    dArray[i] = result;
                }
                else if (GetIsRequired())
                {
                    dArray[i] = 0.0;
                }
            }
            Value = dArray;
        }
    }
}