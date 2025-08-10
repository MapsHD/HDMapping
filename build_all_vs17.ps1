# Automatizare builduri lidar_odometry_step_1.exe pentru Visual Studio 17 2022
# Creează toate variantele de optimizare și copiază binarele cu denumiri clare

$ErrorActionPreference = 'Stop'

# Director pentru binare finale
$binDir = "binaries"
if (-not (Test-Path $binDir)) { New-Item -ItemType Directory -Path $binDir | Out-Null }

$builds = @(
    @{ Name = "generic"; Opt = "GENERIC" },
    @{ Name = "amd";     Opt = "AMD"     },
    @{ Name = "intel";   Opt = "INTEL"   }
)

foreach ($b in $builds) {
    $buildDir = "build_release_" + $b.Name
    if (Test-Path $buildDir) { Remove-Item -Recurse -Force $buildDir }
    cmake -S . -B $buildDir -G "Visual Studio 17 2022" -A x64 -DHD_CPU_OPTIMIZATION=$($b.Opt) -DCMAKE_BUILD_TYPE=Release
    cmake --build $buildDir --config Release --target lidar_odometry_step_1
    $src = Join-Path $buildDir 'bin\Release\lidar_odometry_step_1.exe'
    $dst = Join-Path $binDir ("lidar_odometry_step_1_release_{0}_pr160_vs17.exe" -f $b.Name)
    Copy-Item $src $dst -Force
}

Write-Host "Toate binarele au fost generate și copiate în $binDir" -ForegroundColor Green
