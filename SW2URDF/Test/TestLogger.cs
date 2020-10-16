using log4net;
using SW2URDF.Utilities;
using System.IO;
using Xunit;

namespace SW2URDF.Test
{
    public class TestLogger : SW2URDFTest
    {
        public TestLogger(SWTestFixture fixture) : base(fixture)
        {
        }

        public static void TestGetLogger()
        {
            Assert.NotNull(Logger.GetLogger());
        }

        public static void TestGetLoggerTwice()
        {
            Assert.Equal(Logger.GetLogger(), Logger.GetLogger());
        }

        public static void TestLoggerFileExists()
        {
            ILog logger = Logger.GetLogger();
            string filename = Logger.GetFileName();
            string message = "Hello there";
            logger.Info(message);
            LogManager.Flush(1000);
            Assert.True(File.Exists(filename));

            string[] text = File.ReadAllLines(filename);
            Assert.Contains(message, text[text.Length - 1]);
        }
    }
}
