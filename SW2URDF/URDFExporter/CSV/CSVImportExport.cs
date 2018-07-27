using System.Collections.Generic;
using System.Collections.Specialized;
using System.IO;
using System.Linq;
using System.Text;
using log4net;

namespace SW2URDF.CSV
{
    public static class ImportExport
    {
        private static readonly ILog logger = Logger.GetLogger();

        public static void WriteHeaderToCSV(StreamWriter stream)
        {
            StringBuilder builder = new StringBuilder();
            foreach (KeyValuePair<string, string> contextToColumn in ContextToColumns.Dictionary)
            {
                string context = contextToColumn.Key;
                string column = contextToColumn.Value;

                builder = builder.Append(column).Append(",");
            }
            stream.WriteLine(builder.ToString() + "\n");
        }

        public static void WriteValuesToCSV(StreamWriter stream, OrderedDictionary dictionary)
        {
            StringBuilder builder = new StringBuilder();
            foreach (KeyValuePair<string, string> contextToColumn in ContextToColumns.Dictionary)
            {
                string context = contextToColumn.Key;
                string column = contextToColumn.Value;
                if (dictionary.Contains(context))
                {
                    object value = dictionary[context];
                    builder = builder.Append(value).Append(",");
                }
                else
                {
                    builder = builder.Append("").Append(",");
                }
            }

            // invalid cast
            IEnumerable<string> keys1 = (IEnumerable<string>)ContextToColumns.Dictionary.Keys
            IEnumerable<string> keys2 = (IEnumerable<string>)dictionary.Keys;

            StringBuilder missingColumns = new StringBuilder();
            foreach (string missing in keys1.Except(keys2))
            {
                missingColumns.Append(missing).Append(",");
            }
            if (missingColumns.Length > 0)
            {
                logger.Info("The following columns were not written to the CSV: " + missingColumns.ToString());
            }

            stream.WriteLine(builder.ToString() + "\n");
        }

        public static void WriteLinkToCSV(StreamWriter stream, Link link)
        {
            OrderedDictionary dictionary = new OrderedDictionary();
            link.AppendToCSVDictionary(new List<string>(), dictionary);
            WriteValuesToCSV(stream, dictionary);

            foreach (Link child in link.Children)
            {
                WriteLinkToCSV(stream, child);
            }
        }

        public static void WriteRobotToCSV(Robot robot, string filename)
        {
            logger.Info("Writing CSV file " + filename);
            using (StreamWriter stream = new StreamWriter(filename))
            {
                WriteHeaderToCSV(stream);
                WriteLinkToCSV(stream, robot.BaseLink);
            }
        }
    }
}