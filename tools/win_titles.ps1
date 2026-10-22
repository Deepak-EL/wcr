# List visible Windows window titles matching a pattern (used to spot WSLg's [WARN:COPY MODE]).
param([string]$Pattern = 'Gazebo|COPY|RViz')
Add-Type @'
using System; using System.Text; using System.Runtime.InteropServices; using System.Collections.Generic;
public class WinT { public delegate bool CB(IntPtr h, IntPtr l);
[DllImport("user32.dll")] public static extern bool EnumWindows(CB cb, IntPtr l);
[DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
public static List<string> All() { var r = new List<string>(); EnumWindows((h,l)=>{ if(IsWindowVisible(h)){ var s=new StringBuilder(512); GetWindowText(h,s,512); if(s.Length>0) r.Add(s.ToString()); } return true; }, IntPtr.Zero); return r; } }
'@
[WinT]::All() | Where-Object { $_ -match $Pattern }
