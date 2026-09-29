# B stage judge: build the control core (App\) on the PC and run the tests.
# Usage:  .\tools\check_core.ps1
#
# ASCII only on purpose: avoids Chinese mojibake in the Windows console.
$root  = Split-Path -Parent $PSScriptRoot
$app   = Join-Path $root 'App'
$tests = Join-Path $root 'tests'
$build = Join-Path $root 'build_pc'

Write-Host ""
Write-Host "=================================================="
Write-Host "  B stage - control core (PC side judge)"
Write-Host "=================================================="

if (-not (Test-Path $app)) {
  Write-Host "App folder not found: $app" -ForegroundColor Red
  exit 1
}
New-Item -ItemType Directory -Force -Path $build | Out-Null

[string[]]$flags = @('-std=c++20', '-Wall', '-Wextra', '-g', "-I$app", "-I$tests")
[string[]]$src   = @(Get-ChildItem $app -Filter '*.cpp' | ForEach-Object { $_.FullName })
[string[]]$test  = @((Join-Path $tests 'test_core.cpp'))
$exe = Join-Path $build 'test_core.exe'

if ($src.Count -eq 0) {
  Write-Host "No .cpp file inside App\" -ForegroundColor Red
  exit 1
}

$buildLog = & g++ ($flags + $src + $test + @('-o', $exe)) 2>&1
if ($LASTEXITCODE -ne 0) {
  Write-Host "[COMPILE FAILED] fix the FIRST error first - later ones are often knock-on:" -ForegroundColor Red
  $errLines = @($buildLog | Select-String -Pattern 'error:' | ForEach-Object { $_.Line.Trim() })
  if ($errLines.Count -gt 0) {
    Write-Host ("  {0} error(s), root causes first:" -f $errLines.Count) -ForegroundColor Red
    $errLines | Select-Object -First 12 | ForEach-Object { Write-Host "   $_" }
    if ($errLines.Count -gt 12) {
      Write-Host ("   ... and {0} more" -f ($errLines.Count - 12))
    }
  } else {
    Write-Host "  (no 'error:' line found - showing the raw output):" -ForegroundColor Red
    $buildLog | Select-Object -First 20 | ForEach-Object { Write-Host "  $_" }
  }
  Write-Host ""
  Write-Host "VERDICT: not passed (compile error)" -ForegroundColor Red
  exit 1
}
Write-Host "[compile] OK" -ForegroundColor Green
Write-Host ""

$out  = & $exe 2>&1
$code = $LASTEXITCODE
$out | ForEach-Object { Write-Host "  $_" }

$fails = @($out | Where-Object { "$_" -match '^\[FAIL\]' })

Write-Host ""
if ($code -eq 0) {
  Write-Host "VERDICT: * ALL TESTS PASSED (B1 + B2 + B3 + B4 + B5)" -ForegroundColor Green
  Write-Host ""
  Write-Host "Now answer the B5 questions in the level answer sheet (docs\ folder):"
  Write-Host "  1) why is feeding the watchdog inside the 1 ms interrupt a bad idea ?"
  Write-Host "  2) what are the three health conditions and what does each one catch ?"
  Write-Host "  3) why must the timeout test use unsigned subtraction (now - last) ?"
} else {
  Write-Host "VERDICT: NOT PASSED" -ForegroundColor Yellow
  $fails | ForEach-Object { Write-Host "  $_" }
  Write-Host ""
  Write-Host "Hint: edit only the TODOs in App\WatchdogPolicy.cpp; do not change WatchdogPolicy.h"
}
exit $code
