
$CommitVersion = git describe --tags --long --dirty --always
$FileContent = 'using System.Reflection;

[assembly: AssemblyInformationalVersion("{0}")]' -f $CommitVersion
$FileContent | Out-File SW2URDF\VersionInfo.cs
