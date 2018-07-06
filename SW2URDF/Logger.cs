using log4net;
using log4net.Repository.Hierarchy;
using log4net.Core;
using log4net.Appender;
using log4net.Layout;
using System;
using System.Linq;
using log4net.Layout.Pattern;
using System.IO;

namespace SW2URDF
{
    public class FileNamePatternConverter : PatternLayoutConverter
    {
        override protected void Convert(TextWriter writer, LoggingEvent loggingEvent)
        {
            writer.Write(Path.GetFileName(loggingEvent.LocationInformation.FileName));
        }
    }

    public class Logger
    {
        private static bool Initialized = false;
        public static void Setup()
        {
            if (Initialized)
            {
                return;
            }

            Hierarchy hierarchy = (Hierarchy)LogManager.GetRepository();

            PatternLayout patternLayout = new PatternLayout();
            patternLayout.ConversionPattern = "%date %-5level %filename: %line - %message%newline"; // Slower
            //patternLayout.ConversionPattern = "%date %-5level %logger %file: $line - %message%newline"; // Faster, apparently
            patternLayout.AddConverter("filename", typeof(FileNamePatternConverter));
            patternLayout.ActivateOptions();

            RollingFileAppender roller = new RollingFileAppender();
            roller.AppendToFile = false;
            roller.File = @"C:\\sw2urdf_logs\\sw2urdf.log";
            roller.Layout = patternLayout;
            roller.MaxSizeRollBackups = 5;
            roller.MaximumFileSize = "10MB";
            roller.RollingStyle = RollingFileAppender.RollingMode.Size;
            roller.StaticLogFileName = true;
            roller.ActivateOptions();
            hierarchy.Root.AddAppender(roller);

            MemoryAppender memory = new MemoryAppender();
            memory.ActivateOptions();
            hierarchy.Root.AddAppender(memory);

            hierarchy.Root.Level = Level.Info;
            hierarchy.Configured = true;
            Logger.Initialized = true;
            var logger = log4net.LogManager.GetLogger(
                System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
            logger.Info("\n" + String.Concat(Enumerable.Repeat("-", 80)));
            logger.Info("Logging commencing for SW2URDF exporter");
        }

        public static log4net.ILog GetLogger()
        {
            Logger.Setup();
            return log4net.LogManager.GetLogger(
                System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        }

        public static string GetFileName()
        {
            var rootAppender = LogManager.GetRepository().GetAppenders().OfType<RollingFileAppender>()
                                         .FirstOrDefault();
            if (rootAppender != null)
            {
                return rootAppender.File;
            }
            else
            {
                return null;
            }
        }
    }
}
