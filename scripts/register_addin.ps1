<#
.SYNOPSIS
    Register (or unregister) the built SW2URDF add-in with SolidWorks via RegAsm.

.DESCRIPTION
    SolidWorks loads SW2URDF as a COM add-in, which must be registered under
    HKEY_CLASSES_ROOT - this needs administrator rights. Normally the project's
    post-build step does this, but only when Visual Studio itself runs elevated.

    This script lets you build normally (no elevation needed) and then register
    the resulting DLL once. It self-elevates, so just run it from a normal shell:

        pwsh -File scripts\register_addin.ps1
        pwsh -File scripts\register_addin.ps1 -Unregister
        pwsh -File scripts\register_addin.ps1 -Configuration Release

    After registering, (re)start SolidWorks and the add-in appears in the task pane.

.PARAMETER Configuration
    Build configuration whose output to register. Default: Debug.

.PARAMETER Platform
    Output platform subfolder. "" for AnyCPU (bin\Debug), or "x64" (bin\x64\Debug).
    Default: "" (AnyCPU).

.PARAMETER Dll
    Explicit path to SW2URDF.dll. Overrides Configuration/Platform if given.

.PARAMETER Unregister
    Unregister instead of register.
#>
[CmdletBinding()]
param(
    [string]$Configuration = "Debug",
    [string]$Platform = "",
    [string]$Dll = "",
    [switch]$Unregister
)

$ErrorActionPreference = "Stop"

# Resolve the DLL path (repo root is the parent of this script's folder).
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $Dll) {
    $platSub = if ($Platform) { "$Platform\" } else { "" }
    $Dll = Join-Path $repoRoot "SW2URDF\bin\$platSub$Configuration\SW2URDF.dll"
}
if (-not (Test-Path $Dll)) {
    Write-Error "SW2URDF.dll not found at: $Dll`nBuild the SW2URDF project first (Configuration=$Configuration, Platform='$Platform')."
}
$Dll = (Resolve-Path $Dll).Path

# Re-launch elevated if we are not already administrator.
$isAdmin = ([Security.Principal.WindowsPrincipal] `
    [Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "Elevating to administrator..."
    $argList = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$PSCommandPath`"",
                 "-Dll", "`"$Dll`"")
    if ($Unregister) { $argList += "-Unregister" }
    Start-Process -FilePath (Get-Process -Id $PID).Path -Verb RunAs -ArgumentList $argList
    return
}

$regasm = Join-Path $env:WINDIR "Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe"
if (-not (Test-Path $regasm)) {
    Write-Error "RegAsm.exe (x64, v4.0.30319) not found at: $regasm"
}

if ($Unregister) {
    Write-Host "Unregistering $Dll ..."
    & $regasm /unregister $Dll
} else {
    Write-Host "Registering $Dll ..."
    & $regasm /codebase $Dll
}

if ($LASTEXITCODE -ne 0) {
    Write-Error "RegAsm failed with exit code $LASTEXITCODE."
}
Write-Host "Done. (Re)start SolidWorks to pick up the change." -ForegroundColor Green
