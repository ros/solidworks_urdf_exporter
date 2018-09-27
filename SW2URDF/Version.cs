using System.IO;
using System.Reflection;

namespace SW2URDF
{
    internal class Version
    {
        public static string GetCommitVersion()
        {
            string gitVersion = "";
            using (Stream stream = Assembly.GetExecutingAssembly()
                    .GetManifestResourceStream("SW2URDF.git_version.txt"))
            {
                using (StreamReader reader = new StreamReader(stream))
                {
                    gitVersion = reader.ReadToEnd();
                }
            }
            return gitVersion;
        }

        public static string GetBuildVersion()
        {
            // Getting AssemblyVersion which is auto incremented for each build. See the AssemblyInfo.cs
            return typeof(Logger).Assembly.GetName().Version.ToString();
        }
    }
}