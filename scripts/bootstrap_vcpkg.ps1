# Bootstrap vcpkg into vendor/vcpkg (run once before first cmake configure)
param(
    [string]$VcpkgPath = "$PSScriptRoot\..\vendor\vcpkg"
)

$ErrorActionPreference = "Stop"
$VcpkgPath = [System.IO.Path]::GetFullPath($VcpkgPath)

if (Test-Path "$VcpkgPath\vcpkg.exe") {
    Write-Host "vcpkg already bootstrapped at $VcpkgPath"
} elseif (Test-Path $VcpkgPath) {
    Write-Host "Bootstrapping existing clone at $VcpkgPath ..."
    & "$VcpkgPath\bootstrap-vcpkg.bat" -disableMetrics
} else {
    Write-Host "Cloning vcpkg into $VcpkgPath ..."
    git clone https://github.com/microsoft/vcpkg.git $VcpkgPath
    Write-Host "Bootstrapping..."
    & "$VcpkgPath\bootstrap-vcpkg.bat" -disableMetrics
}

$env:VCPKG_ROOT = $VcpkgPath
Write-Host ""
Write-Host "Done. To configure and build:"
Write-Host "  `$env:VCPKG_ROOT = '$VcpkgPath'"
Write-Host "  cmake --preset windows-msvc-release"
Write-Host "  cmake --build build\windows-msvc-release --config Release"
Write-Host ""
Write-Host "To also build the Open3D viewer (heavy, ~30 min first time):"
Write-Host "  cmake --preset windows-msvc-viewer"
