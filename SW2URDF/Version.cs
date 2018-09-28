using System.Diagnostics;

namespace SW2URDF
{
    internal class Version
    {
        public static string GetCommitVersion()
        {
            // Getting commit version which is attached to the latest git commit
            return FileVersionInfo.GetVersionInfo(typeof(Logger).Assembly.Location).ProductVersion;
        }

        public static string GetBuildVersion()
        {
            // Getting AssemblyVersion which is auto incremented for each build. See the AssemblyInfo.cs
            return typeof(Logger).Assembly.GetName().Version.ToString();
        }
    }
}