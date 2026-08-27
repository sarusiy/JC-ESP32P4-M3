param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('build', 'flash', 'monitor')]
    [string]$Action,

    [string]$ProjectPath = "$PSScriptRoot\..",
    [string]$Port = "COM3",
    [ValidateSet('esp32', 'esp32p4', 'esp32c6', 'esp32s3', 'esp32s2', 'esp32c3')]
    [string]$Target
)

$ErrorActionPreference = 'Stop'

function Find-IdfRoot {
    $candidateRoots = @(
        (Join-Path $env:USERPROFILE ".espressif\frameworks"),
        (Join-Path $env:USERPROFILE ".espressif"),
        "C:\Espressif",
        "C:\projects\Espressif-IDE-4.4.0-win32.win32.x86_64"
    )

    foreach ($root in $candidateRoots) {
        if (-not (Test-Path $root)) {
            continue
        }

        $candidates = Get-ChildItem -Path $root -Directory -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -like "esp-idf*" -or $_.Name -eq "framework-espidf" } |
            Sort-Object Name -Descending

        foreach ($candidate in $candidates) {
            $exportBat = Join-Path $candidate.FullName "export.bat"
            if (Test-Path $exportBat) {
                return $candidate.FullName
            }
        }
    }

    return $null
}

$idfRoot = Find-IdfRoot
if (-not $idfRoot) {
    Write-Error "ESP-IDF not found under $env:USERPROFILE\.espressif\frameworks. Open Espressif IDE and install ESP-IDF tools first."
}

$projectFull = (Resolve-Path $ProjectPath).Path
$exportBat = Join-Path $idfRoot "export.bat"

function Get-ConfiguredTarget {
    $sdkconfig = Join-Path $projectFull "sdkconfig"
    if (-not (Test-Path $sdkconfig)) {
        return $null
    }

    $line = Select-String -Path $sdkconfig -Pattern '^CONFIG_IDF_TARGET="(.+)"$' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($line -and $line.Matches.Count -gt 0) {
        return $line.Matches[0].Groups[1].Value
    }

    return $null
}

function Get-TargetPrefix {
    param([string]$RequestedTarget)

    return ""
}

function Get-TargetArgs {
    param([string]$RequestedTarget)

    if (-not $RequestedTarget) {
        return ""
    }

    return '-DIDF_TARGET={0} ' -f $RequestedTarget
}

switch ($Action) {
    'build' {
        $targetPrefix = Get-TargetPrefix $Target
        $targetArgs = Get-TargetArgs $Target
        $cmd = 'call "{0}" && {1}idf.py -C "{2}" {3}build' -f $exportBat, $targetPrefix, $projectFull, $targetArgs
    }
    'flash' {
        $targetPrefix = Get-TargetPrefix $Target
        $targetArgs = Get-TargetArgs $Target
        $cmd = 'call "{0}" && {1}idf.py -C "{2}" {3}-p {4} flash' -f $exportBat, $targetPrefix, $projectFull, $targetArgs, $Port
    }
    'monitor' {
        $targetPrefix = Get-TargetPrefix $Target
        $targetArgs = Get-TargetArgs $Target
        $cmd = 'call "{0}" && {1}idf.py -C "{2}" {3}-p {4} monitor' -f $exportBat, $targetPrefix, $projectFull, $targetArgs, $Port
    }
}

Write-Host "Using ESP-IDF root: $idfRoot"
Write-Host "Project: $projectFull"
Write-Host "Action: $Action"
if ($Action -ne 'build') {
    Write-Host "Port: $Port"
}
if ($Target) {
    Write-Host "Target: $Target"
}

cmd /c $cmd
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
