# Snapshot a Webots world from a fixed camera (no follow): runs 0.5 m of the climb test with the
# Viewpoint replaced, saves results\webots_<World>_view.png.
#   tools\win_webots_view.ps1 -World winter_dam -Eye "-40 -70 45" -Target "-10 10 5"
param([string]$World = 'winter_dam', [string]$Eye = '-40 -70 45', [string]$Target = '-10 10 5', [int]$TimeoutSec = 300)
$root = Split-Path -Parent $PSScriptRoot
$py = "C:\Users\DEEPAK E L\.wcrvenv\Scripts\python.exe"
$orient = & $py -I -c @"
import math, sys
sys.path.insert(0, r'$root\tools'); from build_webots import rot
e = [float(v) for v in '$Eye'.split()]; t = [float(v) for v in '$Target'.split()]
d = [b - a for a, b in zip(e, t)]
print(rot(0.0, math.atan2(-d[2], math.hypot(d[0], d[1])), math.atan2(d[1], d[0])))
"@
$src = Join-Path $root "webots\worlds\$World.wbt"
$tmp = Join-Path $root "webots\worlds\${World}_view.wbt"
$txt = Get-Content $src -Raw
$txt = [regex]::Replace($txt, '(?s)Viewpoint \{.*?\n\}', "Viewpoint {`n  orientation $orient`n  position $Eye`n}")
Set-Content -NoNewline -Encoding ascii $tmp $txt
& (Join-Path $PSScriptRoot 'win_webots_test.ps1') -World "${World}_view" -Mode climb -Distance 0.5 -TimeoutSec $TimeoutSec | Select-Object -First 1
Copy-Item (Join-Path $root 'results\webots_climb.png') (Join-Path $root "results\webots_${World}_view.png")
Remove-Item $tmp
"saved results\webots_${World}_view.png"
