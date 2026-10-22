#!/usr/bin/env python3
"""Print 'eye|target' (world frame) for an overview of a world's inspection area, or of one item:
  python tools/inspection_views.py <world> [item_id] [distance]
Used with tools/win_webots_view.ps1 to render the Phase 3 layout."""
import sys

import numpy as np

import inspection

world = sys.argv[1]
fr = inspection.face_frame(world)
items = {it['id']: it for it in inspection.layout(world, inspection.FIXED_SEED)}
if len(sys.argv) > 2 and sys.argv[2] in items:
    it = items[sys.argv[2]]
    dist = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    target = inspection.to_world(fr, it['s'], it['t'], 0.1)
    eye = target + fr['n'] * dist - fr['up'] * dist * 0.35 + fr['across'] * dist * 0.25
else:
    A = inspection.AREAS[world]
    s_mid = min(sum(A['s']) / 2, fr['spawn'][0] + 5)
    width = A['t'][1] - A['t'][0]
    target = inspection.to_world(fr, s_mid, sum(A['t']) / 2)
    dist = float(sys.argv[2]) if len(sys.argv) > 2 else min(width * 0.9, 14)
    eye = target + fr['n'] * dist - fr['up'] * dist * 0.25
print(' '.join(f'{v:.2f}' for v in eye) + '|' + ' '.join(f'{v:.2f}' for v in target))
