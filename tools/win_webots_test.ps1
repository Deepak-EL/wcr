# Headless Webots test on Windows: copies a world with test controllerArgs, runs it minimized in
# fast mode and prints the controller's result file (works even if Webots is already open, which
# makes the launcher hand the world to the running instance).
#   tools\win_webots_test.ps1 [-World winter_dam] [-Mode climb|drive|sensors] [-Distance 3.0] [-Speed 0.12]
param([string]$World = 'winter_dam', [string]$Mode = 'climb', [double]$Distance = 3.0,
      [double]$Speed = 0.12, [int]$TimeoutSec = 600)
$root = Split-Path -Parent $PSScriptRoot
$webots = Join-Path $env:LOCALAPPDATA 'Programs\Webots\msys64\mingw64\bin\webots.exe'
$src = Join-Path $root "webots\worlds\$World.wbt"
$test = Join-Path $root "webots\worlds\_test_$World.wbt"
$results = Join-Path $root 'results'
New-Item -ItemType Directory -Force $results | Out-Null
if ($Mode -eq 'drive') { $ctl = '"drive" "quit"'; $out = 'webots_drive_test.txt' }
elseif ($Mode -eq 'sensors') { $ctl = '"sensors" "quit"'; $out = 'webots_sensors.txt' }
else { $ctl = "`"climb`" `"$Distance`" `"$Speed`" `"quit`" `"snapshot`""; $out = 'webots_climb.txt' }
$outFile = Join-Path $results $out
Remove-Item $outFile -ErrorAction SilentlyContinue
(Get-Content $src -Raw) -replace 'controllerArgs \[ "keyboard" \]', "controllerArgs [ $ctl ]" |
  Set-Content -NoNewline -Encoding ascii $test
$log = Join-Path $results 'webots_test.log'
Start-Process -FilePath $webots -ArgumentList '--mode=fast', '--minimize', '--batch', '--stdout', '--stderr', "`"$test`"" `
  -RedirectStandardOutput $log -RedirectStandardError "$log.err" -WindowStyle Minimized | Out-Null
$t0 = Get-Date
while (-not (Test-Path $outFile) -and ((Get-Date) - $t0).TotalSeconds -lt $TimeoutSec) { Start-Sleep 2 }
Start-Sleep 2
if (Test-Path $outFile) { Get-Content $outFile } else { "no result after $TimeoutSec s" }
"elapsed $([int]((Get-Date) - $t0).TotalSeconds) s wall"
Get-Content $log, "$log.err" -ErrorAction SilentlyContinue | Select-String -Pattern 'Error|ERROR|Traceback|WARNING' | Select-Object -First 15
Remove-Item $test -ErrorAction SilentlyContinue
