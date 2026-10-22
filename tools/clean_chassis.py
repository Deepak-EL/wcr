#!/usr/bin/env python3
"""Robot visual meshes from the CAD STLs: side motors removed, light but faithful, smooth shaded.

The chassis STL is a merged CAD export (~2955 disconnected bodies). Bodies are classified
by position in the CAD frame (mm, z up, x forward):
  * prop motors + mounts : body centre |y| > 150 mm   (2 back-to-back pairs per side)
  * arm tubes            : the two long tubes, y 79..166 mm, that carried them

Simplification is done PER BODY with an error bound (simplifying the merged mesh in one
go collapses thin plates into spikes). A body is reduced only if every check point stays
within --tol mm of the original surface in both directions; otherwise a gentler reduction
is tried, and if none passes the body is kept as is. Bodies smaller than --min-size mm are
dropped (not visible at robot scale). Output is OBJ with crease-angle normals: curved parts
shade smoothly, sharp edges stay crisp.

Outputs (mm; URDF scales by 0.001):
  meshes/chassis_body.obj          motors + arm tubes removed  (default in URDF)
  meshes/chassis_body_arms.obj     motors removed, arm tubes kept
  meshes/active_wheel.obj          steering pod (activeWheel.stl)
  meshes/wheel.obj                 wheel (wheel.stl)

usage: python tools/clean_chassis.py [--keep 0.1] [--tol 0.5] [--min-size 0.8] [--crease 30]
needs: pip install trimesh numpy scipy networkx rtree fast-simplification
"""
import argparse
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

MESHES = Path(__file__).resolve().parents[1] / 'wcr_description' / 'meshes'
SMALL = 200          # bodies with fewer triangles are kept as they are


def is_motor(body):
    return abs(body.bounds.mean(0)[1]) > 150.0


def is_arm_tube(body):
    lo, hi = body.bounds
    return (abs(body.bounds.mean(0)[0]) < 1.0 and 75 < min(abs(lo[1]), abs(hi[1])) < 85
            and 160 < max(abs(lo[1]), abs(hi[1])) < 170 and np.allclose(body.extents[0], 25, atol=1))


def max_deviation(a, b, n=300):
    """Symmetric max distance [mm] between surfaces a and b, from n samples each way."""
    pa = a.sample(n)
    pb = b.sample(n)
    return max(b.nearest.on_surface(pa)[1].max(), a.nearest.on_surface(pb)[1].max())


def simplify_body(body, keep, tol):
    n = len(body.faces)
    if n <= SMALL:
        return body
    for k in sorted({keep, keep * 2, keep * 3, keep * 4, 0.6, 0.75}):
        if k < keep or k >= 0.9:
            continue
        target = max(SMALL, int(n * k))
        v, f = fast_simplification.simplify(body.vertices, body.faces, 1.0 - target / n)
        if len(f) == 0:
            continue
        cand = trimesh.Trimesh(v, f, process=True)
        if cand.area > 0 and max_deviation(body, cand) <= tol:
            return cand
    return body


def simplify(bodies, args, name):
    kept = [b for b in bodies if b.extents.max() >= args.min_size]
    out = [simplify_body(b, args.keep, args.tol) for b in kept]
    m = trimesh.util.concatenate(out)
    n0 = sum(len(b.faces) for b in bodies)
    print(f'{name}: {n0} -> {len(m.faces)} tris ({len(bodies) - len(kept)} tiny bodies dropped, '
          f'{sum(o is not b for o, b in zip(out, kept))}/{len(kept)} bodies simplified)')
    return m


def export_smooth(mesh, path, crease_deg):
    """OBJ with normals averaged only across edges sharper than crease_deg."""
    smooth = trimesh.graph.smooth_shade(mesh, angle=np.radians(crease_deg))
    smooth.export(path, include_normals=True, include_texture=False)
    print(f'  -> {path.name}: {len(smooth.faces)} tris, {path.stat().st_size / 1e6:.1f} MB')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keep', type=float, default=0.1, help='first-try fraction of triangles kept per body')
    ap.add_argument('--tol', type=float, default=0.5, help='max surface deviation [mm]')
    ap.add_argument('--min-size', type=float, default=0.8, help='drop bodies smaller than this [mm]')
    ap.add_argument('--crease', type=float, default=30.0, help='edges sharper than this stay hard [deg]')
    args = ap.parse_args()

    src = trimesh.load(MESHES / 'robotChassisNoProp.stl', force='mesh')
    bodies = src.split(only_watertight=False)
    motors = [b for b in bodies if is_motor(b)]
    tubes = [b for b in bodies if not is_motor(b) and is_arm_tube(b)]
    keep = [b for b in bodies if not is_motor(b) and not is_arm_tube(b)]
    print(f'{len(bodies)} bodies: {len(motors)} motor/mount, {len(tubes)} arm tube, {len(keep)} kept')
    assert len(tubes) == 2, 'arm tube rule no longer matches the CAD export'

    body = simplify(keep, args, 'chassis')
    export_smooth(body, MESHES / 'chassis_body.obj', args.crease)
    export_smooth(trimesh.util.concatenate([body] + tubes), MESHES / 'chassis_body_arms.obj', args.crease)
    for src_name, out_name in (('activeWheel.stl', 'active_wheel.obj'), ('wheel.stl', 'wheel.obj')):
        parts = trimesh.load(MESHES / src_name, force='mesh').split(only_watertight=False)
        export_smooth(simplify(parts, args, src_name), MESHES / out_name, args.crease)


if __name__ == '__main__':
    main()
