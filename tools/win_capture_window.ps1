# Capture the on-screen image of a window (by title) N times, to see presentation flicker.
#   win_capture_window.ps1 -Title 'Gazebo' -Count 8 -OutDir <dir>
param([string]$Title = 'Gazebo', [int]$Count = 8, [int]$DelayMs = 150, [string]$OutDir = '.')
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System; using System.Text; using System.Runtime.InteropServices;
public class WinC { public delegate bool CB(IntPtr h, IntPtr l);
[StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
[DllImport("user32.dll")] public static extern bool EnumWindows(CB cb, IntPtr l);
[DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
[DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
public static IntPtr Find(string t) { IntPtr found = IntPtr.Zero; EnumWindows((h,l)=>{ if(IsWindowVisible(h)){ var s=new StringBuilder(512); GetWindowText(h,s,512); if(s.ToString().Contains(t)){ found=h; return false; } } return true; }, IntPtr.Zero); return found; } }
'@
[WinC]::SetProcessDPIAware() | Out-Null
$h = [WinC]::Find($Title)
if ($h -eq [IntPtr]::Zero) { "window '$Title' not found"; exit 1 }
[WinC]::SetForegroundWindow($h) | Out-Null; Start-Sleep -Milliseconds 400
$r = New-Object WinC+RECT; [WinC]::GetWindowRect($h, [ref]$r) | Out-Null
$w = $r.R - $r.L; $hh = $r.B - $r.T
New-Item -ItemType Directory -Force $OutDir | Out-Null
for ($i = 0; $i -lt $Count; $i++) {
  $bmp = New-Object System.Drawing.Bitmap $w, $hh
  $g = [System.Drawing.Graphics]::FromImage($bmp)
  $g.CopyFromScreen($r.L, $r.T, 0, 0, $bmp.Size)
  $bmp.Save((Join-Path $OutDir ("cap_{0:D2}.png" -f $i)), [System.Drawing.Imaging.ImageFormat]::Png)
  $g.Dispose(); $bmp.Dispose()
  Start-Sleep -Milliseconds $DelayMs
}
"captured $Count frames of ${w}x${hh} at ($($r.L),$($r.T))"
