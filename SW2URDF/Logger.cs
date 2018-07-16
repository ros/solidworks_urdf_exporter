using System;
using System.IO;
using System.Linq;

using log4net;
using log4net.Appender;
using log4net.Core;
using log4net.Layout;
using log4net.Layout.Pattern;
using log4net.Repository.Hierarchy;

namespace SW2URDF
{
    public class FileNamePatternConverter : PatternLayoutConverter
    {
        protected override void Convert(TextWriter writer, LoggingEvent loggingEvent)
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

            // This ConversionPattern is slow because any location-based parameter in log4net is
            // slow. If it becomes an issue this might have to be wrapped into a compile time macro
            PatternLayout patternLayout = new PatternLayout()
            {
                ConversionPattern = "%date %-5level %filename: %line - %message%newline"
            };

            patternLayout.AddConverter("filename", typeof(FileNamePatternConverter));
            patternLayout.ActivateOptions();

            string homeDir = Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            RollingFileAppender roller = new RollingFileAppender
            {
                AppendToFile = false,
                File = Path.Combine(homeDir, "sw2urdf_logs", "sw2urdf.log"),
                Layout = patternLayout,
                MaxSizeRollBackups = 5,
                MaximumFileSize = "10MB",
                RollingStyle = RollingFileAppender.RollingMode.Size,
                StaticLogFileName = true
            };

            roller.ActivateOptions();
            hierarchy.Root.AddAppender(roller);

            MemoryAppender memory = new MemoryAppender();
            memory.ActivateOptions();
            hierarchy.Root.AddAppender(memory);

            hierarchy.Root.Level = Level.Info;
            hierarchy.Configured = true;
            Initialized = true;
            var logger = LogManager.GetLogger(
                System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
            logger.Info("\n" + String.Concat(Enumerable.Repeat("-", 80)));
            logger.Info("Logging commencing for SW2URDF exporter");
        }

        public static ILog GetLogger()
        {
            Setup();
            return LogManager.GetLogger(
                System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        }

        public static string GetFileName()
        {
            var rootAppender =
                LogManager.GetRepository().GetAppenders().OfType<RollingFileAppender>()
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