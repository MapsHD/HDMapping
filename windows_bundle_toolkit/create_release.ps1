<#
.SYNOPSIS
    Creates a portable Windows release bundle for HD Mapping.

.DESCRIPTION
    Collects all built executables and DLLs, adds the PROJ database,
    downloads a sample GTX geoid, and packages everything into a ZIP file.

.PARAMETER BuildDir
    Path to the CMake build directory. Defaults to ../build relative to this script.

.PARAMETER OutputDir
    Path where the release bundle directory will be created.
    Defaults to <BuildDir>/HDMapping_release.

.PARAMETER GeoidUrl
    URL of the sample GTX geoid to download.
    Defaults to egm96_15.gtx from the PROJ CDN.

.EXAMPLE
    .\create_release.ps1 -BuildDir C:\src\HDMapping\build
#>

param(
    [string]$BuildDir  = (Join-Path $PSScriptRoot "..\build"),
    [string]$OutputDir = "",
    [string]$GeoidUrl  = "https://cdn.proj.org/egm96_15.gtx"
)

$ErrorActionPreference = "Stop"

# ── Resolve paths ──────────────────────────────────────────────────────────────
$BuildDir = (Resolve-Path $BuildDir).Path

if ($OutputDir -eq "") {
    $OutputDir = Join-Path $BuildDir "HDMapping_release"
}

# On Windows with MSVC the runtime output ends up in bin\Release
$BinDir = Join-Path $BuildDir "bin\Release"
if (-not (Test-Path $BinDir)) {
    # Fallback: flat bin/ layout (single-config generators)
    $BinDir = Join-Path $BuildDir "bin"
}

$ProjDbPath = Join-Path $BuildDir "3rd_binary\PROJ\install_proj\share\proj\proj.db"
$ScriptDir  = $PSScriptRoot
$BatSrc     = Join-Path $ScriptDir "HDMapping.bat"

Write-Host "=================================================" -ForegroundColor Green
Write-Host " HD Mapping - Creating Windows Release Bundle"    -ForegroundColor Green
Write-Host "=================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  Build dir  : $BuildDir"
Write-Host "  Bin dir    : $BinDir"
Write-Host "  Output dir : $OutputDir"
Write-Host ""

# ── Validate ───────────────────────────────────────────────────────────────────
if (-not (Test-Path $BinDir)) {
    Write-Error "Bin directory not found: $BinDir`nPlease build the project first."
    exit 1
}

# ── Create output directory structure ─────────────────────────────────────────
if (Test-Path $OutputDir) {
    Write-Host "Removing existing output directory..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $OutputDir
}

New-Item -ItemType Directory -Path $OutputDir             | Out-Null
New-Item -ItemType Directory -Path "$OutputDir\share\proj" | Out-Null

# ── Step 1: Copy executables and DLLs ─────────────────────────────────────────
Write-Host "Step 1: Copying executables and DLLs..." -ForegroundColor Yellow
$exeCount = 0
$dllCount = 0

Get-ChildItem -Path $BinDir -Filter "*.exe" | ForEach-Object {
    Copy-Item $_.FullName $OutputDir
    Write-Host "  + $($_.Name)"
    $exeCount++
}

Get-ChildItem -Path $BinDir -Filter "*.dll" | ForEach-Object {
    Copy-Item $_.FullName $OutputDir
    $dllCount++
}

Write-Host "  Executables : $exeCount"
Write-Host "  DLLs        : $dllCount"

if ($exeCount -eq 0) {
    Write-Warning "No executables found in $BinDir"
}

# ── Step 2: Copy PROJ database ─────────────────────────────────────────────────
Write-Host "Step 2: Copying PROJ database..." -ForegroundColor Yellow
if (Test-Path $ProjDbPath) {
    Copy-Item $ProjDbPath "$OutputDir\share\proj\"
    Write-Host "  + proj.db"
} else {
    Write-Warning "proj.db not found at $ProjDbPath — skipping."
}

# ── Step 3: Download sample GTX geoid ─────────────────────────────────────────
Write-Host "Step 3: Downloading sample geoid (egm96_15.gtx)..." -ForegroundColor Yellow
$geoidDest = "$OutputDir\share\proj\egm96_15.gtx"
try {
    Invoke-WebRequest -Uri $GeoidUrl -OutFile $geoidDest -UseBasicParsing
    $sizeMb = [math]::Round((Get-Item $geoidDest).Length / 1MB, 1)
    Write-Host "  + egm96_15.gtx  ($sizeMb MB)"
} catch {
    Write-Warning "Could not download geoid from $GeoidUrl`n$_"
    Write-Warning "Geoid will not be included in the release bundle."
    Write-Warning "Note: Without a geoid file, applications that perform height"
    Write-Warning "corrections (e.g. GNSS-based workflows) will use the WGS84"
    Write-Warning "ellipsoid instead of a gravimetric model."
}

# ── Step 4: Copy BAT launcher ─────────────────────────────────────────────────
Write-Host "Step 4: Copying launcher script..." -ForegroundColor Yellow
if (Test-Path $BatSrc) {
    Copy-Item $BatSrc $OutputDir
    Write-Host "  + HDMapping.bat"
} else {
    Write-Warning "Launcher script not found: $BatSrc"
}

# ── Step 5: Create ZIP archive ────────────────────────────────────────────────
Write-Host "Step 5: Creating ZIP archive..." -ForegroundColor Yellow
$ZipPath = Join-Path $BuildDir "HDMapping_windows.zip"
if (Test-Path $ZipPath) {
    Remove-Item -Force $ZipPath
}

Compress-Archive -Path "$OutputDir\*" -DestinationPath $ZipPath -CompressionLevel Optimal
$zipSizeMb = [math]::Round((Get-Item $ZipPath).Length / 1MB, 1)
Write-Host "  ZIP created: $ZipPath  ($zipSizeMb MB)"

# ── Summary ───────────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "=================================================" -ForegroundColor Green
Write-Host " Release bundle created successfully!"            -ForegroundColor Green
Write-Host "=================================================" -ForegroundColor Green
Write-Host ""
Write-Host "  Bundle : $OutputDir"
Write-Host "  ZIP    : $ZipPath"
Write-Host ""
Write-Host "  Structure:"
Write-Host "    HDMapping_release\"
Write-Host "      *.exe  *.dll          ($exeCount executables, $dllCount DLLs)"
Write-Host "      share\proj\proj.db    (PROJ cartographic database)"
Write-Host "      share\proj\*.gtx      (geoid grid files)"
Write-Host "      HDMapping.bat         (tool launcher)"
Write-Host ""
