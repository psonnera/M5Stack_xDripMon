<#
.SYNOPSIS
    Compiles M5Stack_xDripMon and exports the flashable binaries to
    <repo>\Binaries\ESP32_16MB\.

.DESCRIPTION
    One M5Unified firmware covers the whole 16 MB ESP32 lineup (Basic 16MB/v2.7,
    Fire, all Core2), so there is a single build group:

      ESP32_16MB - FQBN m5stack:esp32:m5stack_fire with default options. The
                   defaults are already right for this project: 16 MB flash,
                   default_16MB partition scheme (2x 6400K OTA app slots +
                   3.6 MB spiffs), flash mode dio @ 80 MHz - matching
                   Binaries\firmware.json and the shipped partition table.

    Requires the m5stack:esp32 board package (tested with 2.1.1) and the
    libraries M5Unified, NimBLE-Arduino 2.x and Adafruit NeoPixel.

    The on-screen version (#define XDRIPMON_VERSION, semver) is hand-maintained
    in the sketch and never touched here. The OTA build number lives in
    Binaries\ESP32_16MB\update.inf (YYYYMMDDnn): test builds leave it alone;
    -Release bumps it after a successful build.

.PARAMETER Release
    Bump the OTA build number in update.inf (same-day sequence, or a new day at
    01) after a successful build. Omit for test iterations.

.PARAMETER ArduinoCli
    Path to arduino-cli.exe. If omitted, the script looks at the ARDUINO_CLI
    environment variable, then PATH, then the known Arduino IDE 2.x install
    locations (per-user and all-users).

.EXAMPLE
    .\build.ps1              # test build, update.inf untouched
    .\build.ps1 -Release     # release build, update.inf bumped
#>

[CmdletBinding()]
param(
    [switch]$Release,

    [string]$ArduinoCli,

    # Wipe the build folder before compiling (guaranteed-fresh build).
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

# --- Paths --------------------------------------------------------------------
# The sketch is the single .ino in the repo root (this script lives in <repo>\Scripts\).
$RepoRoot = Split-Path $PSScriptRoot -Parent
$inoFiles = @(Get-ChildItem -Path $RepoRoot -Filter *.ino -File)
if ($inoFiles.Count -ne 1) {
    throw "Expected exactly one .ino sketch in $RepoRoot, found $($inoFiles.Count)."
}
$Sketch     = $inoFiles[0].FullName
$SketchName = $inoFiles[0].BaseName

# --- Locate arduino-cli -------------------------------------------------------
# Order: -ArduinoCli param > ARDUINO_CLI env var > PATH > known install locations
# (Arduino IDE 2.x bundles arduino-cli; both its per-user and all-users layouts
# are probed, plus a conventional standalone location).
$probed = @()
if (-not $ArduinoCli) { $ArduinoCli = $env:ARDUINO_CLI }
if (-not $ArduinoCli) {
    $pathCmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($pathCmd) { $ArduinoCli = $pathCmd.Source }
}
if (-not $ArduinoCli) {
    $probed = @(
        (Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
        (Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\node_modules\arduino-ide-extension\build\arduino-cli.exe'),
        (Join-Path $env:ProgramFiles  'Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
        (Join-Path $env:ProgramFiles  'Arduino IDE\resources\app\node_modules\arduino-ide-extension\build\arduino-cli.exe'),
        (Join-Path $env:USERPROFILE   'tools\arduino-cli\arduino-cli.exe')
    )
    $ArduinoCli = $probed | Where-Object { Test-Path $_ } | Select-Object -First 1
}
if (-not $ArduinoCli -or -not (Test-Path $ArduinoCli)) {
    $msg = "arduino-cli not found."
    if ($probed) { $msg += " Locations tried (after param/env/PATH):`n  " + ($probed -join "`n  ") }
    elseif ($ArduinoCli) { $msg = "arduino-cli not found at '$ArduinoCli'." }
    $msg += "`nPass -ArduinoCli <path>, set `$env:ARDUINO_CLI, or add arduino-cli to PATH."
    throw $msg
}

# --- Environment so the CLI finds the board cores + libraries -----------------
# Respect the caller's env; otherwise use the standard per-user defaults
# (GetFolderPath handles OneDrive-redirected Documents).
if (-not $env:ARDUINO_DIRECTORIES_DATA) {
    $env:ARDUINO_DIRECTORIES_DATA = Join-Path $env:LOCALAPPDATA 'Arduino15'
}
if (-not $env:ARDUINO_DIRECTORIES_USER) {
    $env:ARDUINO_DIRECTORIES_USER = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Arduino'
}

# --- Target -------------------------------------------------------------------
$Fqbn   = 'm5stack:esp32:m5stack_fire'
$Folder = 'ESP32_16MB'

# --- Board-package sanity check -----------------------------------------------
# Fail early with an actionable message instead of a cryptic compile error when
# the FQBN's vendor core is not installed.
$vendor    = ($Fqbn -split ':')[0]
$vendorDir = Join-Path $env:ARDUINO_DIRECTORIES_DATA "packages\$vendor"
if (-not (Test-Path $vendorDir)) {
    throw ("Board package '$vendor' not found in $env:ARDUINO_DIRECTORIES_DATA\packages.`n" +
           "Install it via Arduino IDE Boards Manager or 'arduino-cli core install ${vendor}:esp32'`n" +
           "(if your cores live elsewhere, set `$env:ARDUINO_DIRECTORIES_DATA).")
}

# --- Display version (informational only, hand-maintained in the sketch) ------
$versionMatch = Select-String -Path $Sketch -Pattern '#define\s+XDRIPMON_VERSION\s+"([^"]+)"' | Select-Object -First 1
$DisplayVersion = if ($versionMatch) { $versionMatch.Matches[0].Groups[1].Value } else { '?' }

# --- OTA build number (update.inf, YYYYMMDDnn) --------------------------------
$outDir  = Join-Path (Join-Path $RepoRoot 'Binaries') $Folder
$infPath = Join-Path $outDir 'update.inf'
$NewBuild = $null
if ($Release) {
    # OTA compares this token lexicographically, so it must stay a fixed-width,
    # zero-padded, monotonically increasing YYYYMMDDnn string.
    if (-not (Test-Path $infPath)) {
        throw "Cannot -Release: $infPath not found to derive the next build number."
    }
    $curBuild = (Get-Content $infPath -Raw).Trim()
    if ($curBuild -notmatch '^(\d{8})(\d{2})$') {
        throw "update.inf content '$curBuild' is not in YYYYMMDDnn format; cannot auto-bump."
    }
    $today = Get-Date -Format 'yyyyMMdd'
    if ($Matches[1] -eq $today) {
        $seq = [int]$Matches[2] + 1
        if ($seq -gt 99) {
            throw "Already at sequence 99 for $today; cannot auto-bump further today."
        }
    } else {
        $seq = 1
    }
    $NewBuild = '{0}{1:D2}' -f $today, $seq
    Write-Host ("Release build: update.inf {0} -> {1}" -f $curBuild, $NewBuild) -ForegroundColor Yellow
}

# --- Build --------------------------------------------------------------------
$buildPath = Join-Path $env:LOCALAPPDATA "arduino\builds\$SketchName\$Folder"

Write-Host ''
Write-Host ("=== Building $SketchName v$DisplayVersion ($Folder) ===") -ForegroundColor Green
Write-Host ("    FQBN : {0}" -f $Fqbn)
Write-Host ("    Out  : {0}" -f $outDir)

New-Item -ItemType Directory -Force -Path $outDir | Out-Null

if ($Clean -and (Test-Path $buildPath)) {
    Remove-Item -Recurse -Force $buildPath
}
New-Item -ItemType Directory -Force -Path $buildPath | Out-Null

& $ArduinoCli compile `
    --fqbn $Fqbn `
    --build-path $buildPath `
    --output-dir $outDir `
    $Sketch

if ($LASTEXITCODE -ne 0) {
    throw "Build FAILED (arduino-cli exit $LASTEXITCODE)."
}

# Keep only the flashable bins; drop the heavy .elf/.map debug artifacts.
Get-ChildItem $outDir -Include *.elf, *.map -File -Recurse | Remove-Item -Force

if ($NewBuild) {
    Set-Content -Path $infPath -Value $NewBuild -NoNewline -Encoding ascii
    Write-Host ("update.inf written: {0}" -f $NewBuild)
    Write-Host 'Reminder: Binaries\firmware.json version and whatsnew.txt are hand-maintained.' -ForegroundColor Yellow
}

Write-Host ''
Write-Host 'Done.' -ForegroundColor Cyan
Write-Host "Binaries are in <repo>\Binaries\$Folder\ (app + bootloader + partitions). See Scripts\README.md to flash."
