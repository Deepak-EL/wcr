# Save views of a preview world (one with a "snapshot" supervisor and controllerArgs [ "VIEWS" ]).
#   tools\win_webots_snap.ps1 -World goldeneye_dam_preview -Prefix ge -Views "0 -400 100 0 0 -100", "300 -300 50 0 -100 -100"
# Each view is "eye_x eye_y eye_z target_x target_y target_z"; writes results\<Prefix>_<i>.png.
param([string]$World = 'goldeneye_dam_preview', [string]$Prefix = 'snap', [string[]]$Views, [int]$TimeoutSec = 300)
$root = Split-Path -Parent $PSScriptRoot
$webots = Join-Path $env:LOCALAPPDATA 'Programs\Webots\msys64\mingw64\bin\webots.exe'
$results = Join-Path $root 'results'
$done = Join-Path $results "$Prefix.done"
Remove-Item $done -ErrorAction SilentlyContinue
$args_ = (@("`"$Prefix`"") + ($Views | ForEach-Object { $_.Split(' ', [StringSplitOptions]::RemoveEmptyEntries) | ForEach-Object { "`"$_`"" } })) -join ' '
$tmp = Join-Path $root "webots\worlds\_snap_$World.wbt"
(Get-Content (Join-Path $root "webots\worlds\$World.wbt") -Raw).Replace('controllerArgs [ "VIEWS" ]', "controllerArgs [ $args_ ]") |
  Set-Content -NoNewline -Encoding ascii $tmp
Start-Process -FilePath $webots -ArgumentList '--mode=realtime', '--minimize', '--batch', "`"$tmp`"" -WindowStyle Minimized | Out-Null
$t0 = Get-Date
while (-not (Test-Path $done) -and ((Get-Date) - $t0).TotalSeconds -lt $TimeoutSec) { Start-Sleep 2 }
Remove-Item $tmp -ErrorAction SilentlyContinue
if (Test-Path $done) { Get-ChildItem (Join-Path $results "${Prefix}_*.png") | ForEach-Object { $_.FullName } } else { "no snapshots after $TimeoutSec s" }
