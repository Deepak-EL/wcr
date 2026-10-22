#!/usr/bin/env python3
"""Nav2 occupancy map of a dam face (Phase 4) from an inspection layout YAML.

The map frame lies on the face: x = s (up the face), y = t (across, robot's left when facing up),
origin = the face base point of the layout. Free = the inspection area plus a 0.3 m margin;
everything outside it is occupied (crest, water, the face edges). Obstacles that are on the
known map (mapped: true: bushes, the drain pipe, ledges) are occupied too; the fallen rocks
(mapped: false) are left out, so the robot has to find them with the lidar.

  python tools/wall_map.py <layout.yaml> <out_prefix>   -> <out_prefix>.yaml + .pgm (map_server)
needs: numpy, pyyaml
"""
import sys
from pathlib import Path

import numpy as np
import yaml

RES = 0.05            # m per cell
MARGIN = 0.6          # map border around the free area [m]
FREE_MARGIN = 0.3     # free space beyond the inspection area [m]


def write_map(layout_path, out_prefix):
    doc = yaml.safe_load(Path(layout_path).read_text())
    (s0, s1), (t0, t1) = doc['area']['s'], doc['area']['t']
    x0, y0 = s0 - FREE_MARGIN - MARGIN, t0 - FREE_MARGIN - MARGIN
    W = int(round((t1 - t0 + 2 * (FREE_MARGIN + MARGIN)) / RES))
    H = int(round((s1 - s0 + 2 * (FREE_MARGIN + MARGIN)) / RES))
    # cell centres; array index [i, j] = (x = s, y = t)
    xs = x0 + (np.arange(H) + 0.5) * RES
    ys = y0 + (np.arange(W) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    occ = ~((X > s0 - FREE_MARGIN) & (X < s1 + FREE_MARGIN) & (Y > t0 - FREE_MARGIN) & (Y < t1 + FREE_MARGIN))
    for it in doc['items']:
        if not it.get('mapped', False):
            continue
        ls, lt = it['size']
        if it['type'] == 'bush':                                 # canopy leans up the face
            occ |= np.hypot(X - it['s'] - 0.2 * ls, Y - it['t']) < 0.5 * ls
        elif it['type'] == 'pipe':
            occ |= (np.abs(X - it['s']) < ls / 2) & (np.abs(Y - it['t']) < 0.1)
        else:                                                    # ledge (s extent = its height)
            occ |= (np.abs(X - it['s']) < ls / 2) & (np.abs(Y - it['t']) < lt / 2)
    # map_server image: columns = map x, rows from the top = decreasing map y
    img = np.where(occ, 0, 254).astype(np.uint8)             # [x index, y index]
    img = img.T[::-1, :]
    out_prefix = Path(out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm = out_prefix.with_suffix('.pgm')
    pgm.write_bytes(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]) + img.tobytes())
    out_prefix.with_suffix('.yaml').write_text(
        f'# Nav2 wall map of {doc["world"]} (seed {doc["seed"]}), tools/wall_map.py: map frame on the face,\n'
        f'# x up the face, y across; free = inspection area + {FREE_MARGIN} m, mapped obstacles occupied\n'
        f'image: {pgm.name}\nmode: trinary\nresolution: {RES}\norigin: [{x0:.3f}, {y0:.3f}, 0.0]\n'
        f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return out_prefix.with_suffix('.yaml')


if __name__ == '__main__':
    print(write_map(sys.argv[1], sys.argv[2]))
