# One-click build script (PowerShell)
# Usage: .\build.ps1            (small exe, needs .NET 10 Desktop Runtime on target machine)
#        .\build.ps1 -SelfContained   (bundle runtime, runs on any x64 Windows, ~110MB)
# Output: publish\MotorControlApp.exe (single file; config.json stays next to it, editable)
param(
    [switch]$SelfContained
)

$ErrorActionPreference = 'Stop'
$projectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectFile = Join-Path $projectDir 'MotorControlApp.csproj'
$publishDir = Join-Path $projectDir 'MotorControl'
$nativeDir = Join-Path $projectDir 'NativeDlls'

Write-Host '==> Cleaning old output...' -ForegroundColor Cyan
if (Test-Path $publishDir) {
    Remove-Item $publishDir -Recurse -Force
}

Write-Host '==> Publishing Release single-file exe (x64)...' -ForegroundColor Cyan

if ($SelfContained) {
    # Bundle the runtime: target machine needs nothing installed
    dotnet publish $projectFile `
        -c Release `
        -r win-x64 `
        --self-contained true `
        -p:PublishSingleFile=true `
        -p:IncludeNativeLibrariesForSelfExtract=true `
        -o $publishDir
}
else {
    # Framework-dependent: small exe, target machine needs .NET 10 Desktop Runtime
    dotnet publish $projectFile `
        -c Release `
        -r win-x64 `
        --no-self-contained `
        -p:PublishSingleFile=true `
        -o $publishDir
}

if ($LASTEXITCODE -ne 0) {
    Write-Host '==> BUILD FAILED!' -ForegroundColor Red
    exit 1
}

Write-Host ''
Write-Host '==> BUILD SUCCEEDED!' -ForegroundColor Green
Write-Host ("    exe   : " + (Join-Path $publishDir 'MotorControlApp.exe'))
Write-Host ("    config: " + (Join-Path $publishDir 'config.json'))
Write-Host '    note  : keep MotorControlApp.exe and config.json in the same folder.'
if (Test-Path $nativeDir) {
    $nativeCount = (Get-ChildItem $nativeDir -Filter *.dll -Recurse | Measure-Object).Count
    Write-Host ("    native: " + $nativeCount + " native dll(s) copied from NativeDlls.")
}
else {
    Write-Host '    native: NativeDlls folder not found. For real EtherCAT hardware, put EtherCAT_DLL_x64.dll'
    Write-Host '            into a new "NativeDlls" project folder (auto-copied on build), or next to the exe.'
}
