using System;
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
            using (StreamReader reader = new StreamReader(stream))
            {
                gitVersion = reader.ReadToEnd();
            }
            return gitVersion;
        }

        public static string GetBuildVersion()
        {
            // Getting AssemblyVersion which is auto incremented for each build. See the AssemblyInfo.cs
            string buildVersion = "";
            object[] attributes = typeof(Logger).Assembly.GetCustomAttributes(typeof(AssemblyVersionAttribute), false);
            if (attributes.Length == 0)
            {
                throw new Exception("The AssemblyVersion is not included in this assembly");
            }
            else
            {
                buildVersion = (attributes[0] as AssemblyVersionAttribute).Version;
            }
            return buildVersion;
        }
    }
}