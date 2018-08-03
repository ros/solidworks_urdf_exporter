using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.IO;
using System.Linq;
using System.Text;
using log4net;

namespace SW2URDF.CSV
{
    /// <summary>
    /// Class to perform exporting and eventually exporting of data to a CSV file
    /// </summary>
    public static class ImportExport
    {
        private static readonly ILog logger = Logger.GetLogger();

        #region Public Methods

        /// <summary>
        /// Method to write a full URDF robot to a CSV
        /// </summary>
        /// <param name="robot">URDF robot tree</param>
        /// <param name="filename">Fully qualified string name to write to</param>
        public static void WriteRobotToCSV(Robot robot, string filename)
        {
            logger.Info("Writing CSV file " + filename);
            using (StreamWriter stream = new StreamWriter(filename))
            {
                WriteHeaderToCSV(stream);
                WriteLinkToCSV(stream, robot.BaseLink);
            }
        }

        #endregion Public Methods

        #region Private Methods

        /// <summary>
        /// Iterates through the column names and writes them to a file stream
        /// </summary>
        /// <param name="stream">Stream to write to</param>
        private static void WriteHeaderToCSV(StreamWriter stream)
        {
            StringBuilder builder = new StringBuilder();
            foreach (DictionaryEntry entry in ContextToColumns.Dictionary)
            {
                string context = (string)entry.Key;
                string column = (string)entry.Value;

                builder = builder.Append(column).Append(",");
            }
            stream.WriteLine(builder.ToString() + "\n");
        }

        /// <summary>
        /// Appends a line to an open CSV document of a URDF's Link properties
        /// </summary>
        /// <param name="stream">Stream representing opened CSV file</param>
        /// <param name="dictionary">Dictionary of values</param>
        private static void WriteValuesToCSV(StreamWriter stream, OrderedDictionary dictionary)
        {
            StringBuilder builder = new StringBuilder();
            foreach (DictionaryEntry entry in ContextToColumns.Dictionary)
            {
                string context = (string)entry.Key;
                string column = (string)entry.Value;
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

            HashSet<string> keys1 = new HashSet<string>(ContextToColumns.Dictionary.Keys.Cast<string>());
            HashSet<string> keys2 = new HashSet<string>(dictionary.Keys.Cast<string>());

            StringBuilder missingColumns = new StringBuilder();
            foreach (string missing in keys2.Except(keys1))
            {
                missingColumns.Append(missing).Append(",");
            }
            if (missingColumns.Length > 0)
            {
                logger.Error("The following columns were not written to the CSV: " + missingColumns.ToString());
            }

            stream.WriteLine(builder.ToString() + "\n");
        }

        /// <summary>
        /// Converts a URDF Link to a dictionary of values and writes them to a CSV
        /// </summary>
        /// <param name="stream">StreamWriter of opened CSV document</param>
        /// <param name="link">URDF link to append to the file</param>
        private static void WriteLinkToCSV(StreamWriter stream, Link link)
        {
            OrderedDictionary dictionary = new OrderedDictionary();
            link.AppendToCSVDictionary(new List<string>(), dictionary);
            WriteValuesToCSV(stream, dictionary);

            foreach (Link child in link.Children)
            {
                WriteLinkToCSV(stream, child);
            }
        }

        #endregion Private Methods
    }
}