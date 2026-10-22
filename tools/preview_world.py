#!/usr/bin/env python3
"""Robot-free copy of a world for clean renders: webots/worlds/<world>.wbt ->
webots/worlds/_preview_<world>.wbt with the Wcr robot replaced by the 'snapshot' supervisor
(render with tools/win_webots_snap.ps1 -World _preview_<world>).
  python tools/preview_world.py <world> [--keep-robot]   (--keep-robot: robot stays, unpowered)"""
import re
import sys
from pathlib import Path

WB = Path(__file__).resolve().parents[1] / 'webots' / 'worlds'
world = sys.argv[1]
text = (WB / f'{world}.wbt').read_text()
snap = 'Robot { name "snapshot" controller "snapshot" supervisor TRUE controllerArgs [ "VIEWS" ] }\n'
if '--keep-robot' in sys.argv:
    text = text.replace('controllerArgs [ "keyboard" ]', 'controller "<none>"') + snap
else:
    text = re.sub(r'\nWcr \{.*?\n\}\n', '\n' + snap, text, flags=re.S)
text = re.sub(r'\n  follow "wcr"\n  followType "[^"]*"', '', text)
(WB / f'_preview_{world}.wbt').write_text(text)
print(WB / f'_preview_{world}.wbt')
