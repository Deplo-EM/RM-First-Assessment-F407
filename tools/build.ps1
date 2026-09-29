# One-click firmware build.
# Usage:  .\tools\build.ps1
# Full reconfigure (after changing CubeMX settings, adding sources, or a broken build dir):
#         .\tools\build.ps1 -Reconfigure
#
# It does four things for you:
#   1) adds the ninja folder to PATH for this window only
#   2) configures with "cmake --preset Debug" when needed
#   3) builds with "cmake --build --preset Debug"
#   4) tells you where the .elf is
#
# ASCII only on purpose: avoids Chinese mojibake in the Windows console.

param([switch]$Reconfigure)

$root     = Split-Path -Parent $PSScriptRoot
$ninjaDir = 'D:\ForDownload\stm32-env\ninja'
$buildDir = Join-Path $root 'build\Debug'

# 1) fix PATH for this window only
if ((Test-Path $ninjaDir) -and (($env:Path -split ';') -notcontains $ninjaDir)) {
  $env:Path = $env:Path + ';' + $ninjaDir
}

$code = 0
try {
  Push-Location $root

  # 2) configure
  if ($Reconfigure -and (Test-Path $buildDir)) {
    Write-Host '[setup] removing old build\Debug' -ForegroundColor DarkGray
    Remove-Item $buildDir -Recurse -Force
  }
  if (-not (Test-Path (Join-Path $buildDir 'CMakeCache.txt'))) {
    Write-Host '[1/2] configure: cmake --preset Debug' -ForegroundColor Cyan
    cmake --preset Debug
    if ($LASTEXITCODE -ne 0) {
      Write-Host ''
      Write-Host '[CONFIGURE FAILED] send me the full output above' -ForegroundColor Red
      $code = $LASTEXITCODE
      return
    }
  } else {
    Write-Host '[1/2] already configured, skipped (use -Reconfigure to redo)' -ForegroundColor DarkGray
  }

  # 3) build
  Write-Host '[2/2] build: cmake --build --preset Debug' -ForegroundColor Cyan
  cmake --build --preset Debug
  $code = $LASTEXITCODE
} finally {
  Pop-Location
}

Write-Host ''
if ($code -eq 0) {
  $elf = Join-Path $buildDir 'C_project.elf'
  Write-Host 'BUILD OK' -ForegroundColor Green
  if (Test-Path $elf) {
    Write-Host ('artifact: ' + $elf + '  (' + [Math]::Round((Get-Item $elf).Length / 1KB, 1) + ' KB)')
  }
} else {
  Write-Host 'BUILD FAILED: send me the full output above' -ForegroundColor Red
}
exit $code
