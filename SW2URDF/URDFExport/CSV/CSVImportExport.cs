using CsvHelper;
using log4net;
using Microsoft.VisualBasic.FileIO;
using SW2URDF.URDF;
using SW2URDF.Utilities;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Specialized;
using System.IO;
using System.Linq;
using System.Text;

namespace SW2URDF.URDFExport.CSV
{
    /// <summary>
    /// Class to perform exporting to CSV and eventually importing of data from a CSV file
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
                CsvWriter writer = new CsvWriter(stream);
                WriteHeaderToCSV(writer);
                WriteLinkToCSV(writer, robot.BaseLink);
            }
        }

        /// <summary>
        /// Loads a list of URDF Links from a CSV file
        /// </summary>
        /// <param name="stream"></param>
        /// <returns></returns>
        public static List<Link> LoadURDFRobotFromCSV(Stream stream)
        {
            List<StringDictionary> loadedFields = new List<StringDictionary>();
            using (TextFieldParser csvParser = new TextFieldParser(stream))
            {
                csvParser.SetDelimiters(new string[] { "," });

                string[] headers = csvParser.ReadFields();
                while (!csvParser.EndOfData)
                {
                    string[] fields = csvParser.ReadFields();
                    StringDictionary dictionary = new StringDictionary();
                    int minArrayLength = Math.Min(fields.Length, headers.Length);
                    logger.Warn("The number of columns in the row do not match the number of columns in the header");
                    for (int i = 0; i < minArrayLength; i++)
                    {
                        if (!string.IsNullOrWhiteSpace(fields[i]))
                        {
                            dictionary[headers[i]] = fields[i];
                        }
                    }
                    loadedFields.Add(dictionary);
                }

                return loadedFields.Select(fields => BuildLinkFromData(fields)).ToList();
            }
        }

        #endregion Public Methods

        #region Private Methods

        /// <summary>
        /// Iterates through the column names and writes them to a file stream
        /// </summary>
        /// <param name="stream">Stream to write to</param>
        private static void WriteHeaderToCSV(CsvWriter writer)
        {
            StringBuilder builder = new StringBuilder();
            foreach (DictionaryEntry entry in ContextToColumns.Dictionary)
            {
                string columnName = (string)entry.Value;
                writer.WriteField(columnName);
            }
            writer.NextRecord();
        }

        /// <summary>
        /// Appends a line to an open CSV document of a URDF's Link properties
        /// </summary>
        /// <param name="stream">Stream representing opened CSV file</param>
        /// <param name="dictionary">Dictionary of values</param>
        private static void WriteValuesToCSV(CsvWriter writer, OrderedDictionary dictionary)
        {
            foreach (DictionaryEntry entry in ContextToColumns.Dictionary)
            {
                string context = (string)entry.Key;
                string column = (string)entry.Value;
                if (dictionary.Contains(context))
                {
                    object value = dictionary[context];
                    writer.WriteField(value);
                }
                else
                {
                    writer.WriteField(null);
                }
            }

            writer.NextRecord();
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
        }

        /// <summary>
        /// Converts a URDF Link to a dictionary of values and writes them to a CSV
        /// </summary>
        /// <param name="stream">StreamWriter of opened CSV document</param>
        /// <param name="link">URDF link to append to the file</param>
        private static void WriteLinkToCSV(CsvWriter writer, Link link)
        {
            OrderedDictionary dictionary = new OrderedDictionary();
            link.AppendToCSVDictionary(new List<string>(), dictionary);
            WriteValuesToCSV(writer, dictionary);

            foreach (Link child in link.Children)
            {
                WriteLinkToCSV(writer, child);
            }
        }

        private static Link BuildLinkFromData(StringDictionary dictionary)
        {
            StringDictionary contextDictionary = new StringDictionary();
            foreach (DictionaryEntry entry in ContextToColumns.Dictionary)
            {
                string context = (string)entry.Key;
                string columnName = (string)entry.Value;

                if (dictionary.ContainsKey(columnName))
                {
                    contextDictionary[context] = dictionary[columnName];
                }
            }

            Link link = new Link();
            link.SetElementFromData(new List<string>(), contextDictionary);
            return link;
        }

        #endregion Private Methods
    }
}