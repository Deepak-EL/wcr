#!/usr/bin/env python3
"""Phase 3 inspection features on the dam faces: defects (cracks, wet patches, weep holes) and
obstacles (bushes, a drain pipe, a ledge, fallen-rock debris).

Items are placed in face coordinates: s = metres up the face from its base, t = metres across
(t > 0 to the robot's left when it faces up the face), on the face plane of each world (or snapped
onto the visible surface for the GoldenEye gorge). A layout is reproducible from a seed; the
spawn corridor stays clear of obstacles, and one crack and one wet patch are placed on the climb
line so the wall camera sees them during a climb test.

  layout   wcr_description/worlds/inspection/<world>.yaml   (list of items: type, s, t, size,
           world centre, corners of the visible mark; obstacles have 'mapped' = on the known map)
  assets   wcr_description/worlds/meshes/inspection/*       (generated textures + meshes)
  Webots   a 'DEF INSPECTION Group' inserted in the world by tools/build_webots.py

usage:
  python tools/inspection.py --assets                     regenerate textures + meshes (PIL)
  python tools/inspection.py --world W --seed N [--wbt in.wbt --out out.wbt] [--layout out.yaml]
      random layout for world W; with --wbt, replaces the INSPECTION group of in.wbt -> out.wbt
      (numpy + pyyaml only: used by the ROS launch in WSL for layout:=random)
"""
import argparse
import math
import re
import zlib
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / 'wcr_description' / 'worlds'
ASSETS = WORLDS / 'meshes' / 'inspection'
LAYOUTS = WORLDS / 'inspection'
REL = '../../wcr_description/worlds/meshes/inspection'          # from webots/worlds
FIXED_SEED = 7

# inspection area (s, t ranges) and item counts per world
AREAS = {
    'winter_dam': dict(s=(0.6, 12.0), t=(-6.0, 6.0), crack=6, wet=3, weep=3, bush=2, pipe=1, ledge=1, debris=0,
                       foliage='winter'),
    'gravity_dam': dict(s=(0.4, 6.8), t=(-3.5, 3.5), crack=4, wet=2, weep=2, bush=1, pipe=1, ledge=0, debris=0,
                        foliage='green'),
    'goldeneye_dam': dict(s=(0.3, 7.5), t=(-10.5, 10.5), crack=5, wet=2, weep=2, bush=1, pipe=1, ledge=1, debris=0,
                          foliage='green'),
    'goldeneye_gorge': dict(s=(0.3, 30.0), t=(-5.5, 5.5), crack=6, wet=3, weep=3, bush=2, pipe=1, ledge=1, debris=0,
                            foliage='green'),
}
SIZES = {   # (along s, along t) of the mark / footprint [m]
    'crack': (0.8, 0.8), 'wet_patch': (1.4, 1.0), 'weep_hole': (0.45, 0.18),
    'bush': (0.7, 0.7), 'pipe': (3.0, 0.3), 'ledge': (0.25, 2.5), 'debris': (0.6, 0.6),
}
OBSTACLES = ('bush', 'pipe', 'ledge', 'debris')
# Phase 4 demo coverage area relative to the spawn (s, t offsets, metres)
DEMO_AREA = dict(s=(0.4, 2.6), t=(-1.0, 1.0))
DECAL_OFFSET = {'crack': 0.006, 'wet_patch': 0.005, 'weep_hole': 0.0055}   # above the grain skin (3 mm)


# ------------------------------------------------------------------ face frames
def face_frame(world):
    """base point, unit vectors up / across / n (outward), spawn (s, t) and optional snap
    triangles (the visible surface, for faces that are not planar)."""
    if world == 'winter_dam':
        g = yaml.safe_load((WORLDS / 'dam_geometry.yaml').read_text())
        n, up, base = (np.array(g['slope'][k], float) for k in ('normal', 'up', 'base'))
    elif world == 'gravity_dam':
        g = yaml.safe_load((WORLDS / 'gravity_dam_geometry.yaml').read_text())
        n, up = np.array(g['face']['normal'], float), np.array(g['face']['up'], float)
        base = np.array(g['face']['toe'], float)
        base[0] = 0.5 * sum(g['face']['x'])
    else:
        name = 'goldeneye_dam_geometry.yaml' if world == 'goldeneye_dam' else 'goldeneye_gorge_geometry.yaml'
        g = yaml.safe_load((WORLDS / name).read_text())
        n, up, base = (np.array(g['face'][k], float) for k in ('normal', 'up', 'base'))
    n /= np.linalg.norm(n)
    up /= np.linalg.norm(up)
    across = np.cross(n, up)
    sp = np.array([g['spawn'][k] for k in 'xyz'])
    snap = None
    if g.get('collision_mesh'):
        snap = load_triangles(WORLDS / 'meshes' / 'goldeneye_dam' / g['collision_mesh'])
    return dict(base=base, up=up, across=across, n=n, spawn=((sp - base) @ up, (sp - base) @ across), snap=snap)


def load_triangles(path):
    V, F = [], []
    for line in open(path):
        if line.startswith('v '):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            F.append([int(x.split('/')[0]) - 1 for x in line.split()[1:4]])
    return np.array(V)[np.array(F)]


def to_world(fr, s, t, h=0.0):
    p = fr['base'] + fr['up'] * s + fr['across'] * t + fr['n'] * h
    if fr['snap'] is not None:
        p = snap_points(fr, np.atleast_2d(p))[0] + fr['n'] * h
    return p


def snap_points(fr, P):
    """Move points along -n onto the visible surface (Moller-Trumbore against the triangles)."""
    T, d = fr['snap'], -fr['n']
    O = P + fr['n'] * 2.0
    e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
    pv = np.cross(d, e2)
    det = (e1 * pv).sum(1)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    out = P.copy()
    for i, o in enumerate(O):
        tv = o - T[:, 0]
        u = (tv * pv).sum(1) * inv
        qv = np.cross(tv, e1)
        v = (qv @ d) * inv
        dist = (e2 * qv).sum(1) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (dist > 0)
        if hit.any():
            out[i] = o + d * dist[hit].min()
    return out


# ------------------------------------------------------------------ layout
def layout(world, seed):
    rng = np.random.default_rng(seed)
    A, fr = AREAS[world], face_frame(world)
    s_sp, t_sp = fr['spawn']
    items = []

    def free(s, t, size, gap):
        for it in items:
            ds = abs(s - it['s']) - (size[0] + it['size'][0]) / 2
            dt = abs(t - it['t']) - (size[1] + it['size'][1]) / 2
            if max(ds, dt) < gap:
                return False
        return True

    def in_corridor(s, t, size):
        return (abs(t - t_sp) - size[1] / 2 < 0.55) and (s_sp - 0.8 < s + size[0] / 2) and (s - size[0] / 2 < s_sp + 4.5)

    def place(kind, k, fixed=None, size_fixed=None):
        size = np.array(size_fixed if size_fixed is not None else SIZES[kind], float)
        if size_fixed is None:
            if kind == 'debris':                                # >= full size: top above the lidar plane
                size = size * rng.uniform(1.0, 1.3)
            elif kind in ('crack', 'wet_patch', 'bush'):
                size = size * rng.uniform(0.75, 1.25)
        for _ in range(4000):
            if fixed is not None:
                s, t = fixed
            else:
                s = rng.uniform(A['s'][0] + size[0] / 2, A['s'][1] - size[0] / 2)
                t = rng.uniform(A['t'][0] + size[1] / 2, A['t'][1] - size[1] / 2)
            if fixed is None and kind in OBSTACLES and in_corridor(s, t, size):
                continue
            if free(s, t, size, 0.6 if kind in OBSTACLES else 0.25) or fixed is not None:
                it = dict(id=f'{kind}_{k}', type=kind, s=float(s), t=float(t), size=[float(x) for x in size],
                          variant=int(rng.integers(0, 4)), yaw=float(rng.uniform(-0.6, 0.6)) if kind == 'crack' else 0.0)
                if kind in OBSTACLES:
                    it['mapped'] = kind != 'debris'                # debris: not on the known map
                items.append(it)
                return it
        return None

    # on the climb line, inside the wall camera's footprint (0.16 m left of the centre line)
    place('crack', 0, fixed=(s_sp + 1.3, t_sp - 0.07))
    # in the Phase 4 demo coverage area (DEMO_AREA above the spawn): a wet patch, and a bush in the
    # top corner, clear of the climb line, so the lanes there simply end below it (no squeezing past)
    place('wet_patch', 0, fixed=(s_sp + 2.05, t_sp - 0.5))
    place('bush', 0, fixed=(s_sp + 2.35, t_sp + 0.75), size_fixed=(0.6, 0.6))
    for kind, key in (('pipe', 'pipe'), ('ledge', 'ledge'), ('bush', 'bush'), ('debris', 'debris'),
                      ('crack', 'crack'), ('wet_patch', 'wet'), ('weep_hole', 'weep')):
        have = sum(1 for it in items if it['type'] == kind)
        for k in range(have, A[key]):
            place(kind, k)
    marks = yaml.safe_load((ASSETS / 'marks.yaml').read_text())
    for it in items:
        c = to_world(fr, it['s'], it['t'])
        it['centre'] = [round(float(v), 4) for v in c]
        cy, sy = math.cos(it['yaw']), math.sin(it['yaw'])           # same rotation as the decal
        tex = decal_texture(it)
        if tex:                                                     # the defect's own pixels on the wall
            ls, lt = it['size']
            pts = []
            for u, v in marks[tex]:
                ds, dt = (v - 0.5) * ls, (0.5 - u) * lt
                pts.append(np.array([it['s'] + cy * ds - sy * dt, it['t'] + sy * ds + cy * dt]))
            P = fr['base'] + np.array([fr['up'] * s + fr['across'] * t for s, t in pts])
            if fr['snap'] is not None:
                P = snap_points(fr, P)
            it['mark'] = [[round(float(x), 4) for x in p] for p in P]
        corners = []
        for a, b in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
            ds, dt = a * it['size'][0] / 2, b * it['size'][1] / 2
            corners.append(to_world(fr, it['s'] + cy * ds - sy * dt, it['t'] + sy * ds + cy * dt))
        it['corners'] = [[round(float(v), 4) for v in p] for p in corners]
    return items


def save_layout(world, items, path, seed):
    fr = face_frame(world)
    s_sp, t_sp = fr['spawn']
    A = AREAS[world]
    doc = dict(world=world, seed=seed,
               face=dict(base=fr['base'].round(5).tolist(), up=fr['up'].round(5).tolist(),
                         across=fr['across'].round(5).tolist(), normal=fr['n'].round(5).tolist()),
               spawn=[round(float(s_sp), 4), round(float(t_sp), 4)],
               area=dict(s=list(A['s']), t=list(A['t'])),
               demo_area=dict(s=[round(float(s_sp + v), 3) for v in DEMO_AREA['s']],
                              t=[round(float(t_sp + v), 3) for v in DEMO_AREA['t']]),
               items=items)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f'# Generated by tools/inspection.py (world frame, metres; s up / t across the face)\n'
                    + yaml.safe_dump(doc, sort_keys=False))


# ------------------------------------------------------------------ Webots nodes
def rot_axis_angle(R):
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if angle < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    if abs(angle - math.pi) < 1e-6:
        w, v = np.linalg.eigh(R)
        ax = v[:, np.argmax(w)]
        return (*ax, math.pi)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(angle))
    return (*ax, angle)


def pose(p, R):
    a = rot_axis_angle(R)
    return (f'translation {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} '
            f'rotation {a[0]:.6f} {a[1]:.6f} {a[2]:.6f} {a[3]:.6f}')


def decal(fr, it, tex, offset, grid=6):
    """Textured grid on the face (snapped onto the surface when needed), texture up = up the face."""
    ls, lt = it['size']
    c, sn = math.cos(it['yaw']), math.sin(it['yaw'])
    pts, uv = [], []
    for i in range(grid + 1):
        for j in range(grid + 1):
            u, v = j / grid, i / grid
            ds, dt = (v - 0.5) * ls, (0.5 - u) * lt                 # u -> -t (left to right seen from outside)
            ds, dt = c * ds - sn * dt, sn * ds + c * dt
            pts.append(fr['base'] + fr['up'] * (it['s'] + ds) + fr['across'] * (it['t'] + dt))
            uv.append((u, v))
    P = np.array(pts)
    if fr['snap'] is not None:
        P = snap_points(fr, P)
    P = P + fr['n'] * offset
    idx = []
    for i in range(grid):
        for j in range(grid):
            a = i * (grid + 1) + j
            idx += [a, a + 1, a + grid + 2, a + grid + 1, -1]
    return f'''    Shape {{  # {it['id']}
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "{REL}/{tex}" ] filtering 5 repeatS FALSE repeatT FALSE }}
        roughness 0.95
        metalness 0
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{ point [ {", ".join(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in P)} ] }}
        texCoord TextureCoordinate {{ point [ {", ".join(f"{a:.3f} {b:.3f}" for a, b in uv)} ] }}
        coordIndex [ {" ".join(map(str, idx))} ]
        texCoordIndex [ {" ".join(map(str, idx))} ]
      }}
      castShadows FALSE
    }}
'''


def face_R(fr):
    return np.c_[fr['up'], fr['across'], fr['n']]                 # x up the face, y across, z out


def obstacle(fr, it, foliage):
    s, t = it['s'], it['t']
    R = face_R(fr)
    p0 = to_world(fr, s, t)
    name = it['id']
    if it['type'] == 'pipe':
        L, r, off = it['size'][0], 0.08, 0.17                     # Ø16 cm drain pipe 17 cm out
        pipe_R = np.c_[fr['across'], fr['n'], fr['up']]           # cylinder axis (z) along the face
        kids = [f'''Pose {{ {pose(p0 + fr['n'] * off, pipe_R)} children [ DEF PIPE_{name} Shape {{
          appearance DEF GALV PBRAppearance {{ baseColor 0.62 0.64 0.66 metalness 0.75 roughness 0.38 }}
          geometry Cylinder {{ height {L:.3f} radius {r} subdivision 48 }} }} ] }}''']
        for k in range(int(L // 1.0) + 1):
            sb = s - L / 2 + 0.15 + k * (L - 0.3) / max(1, int(L // 1.0))
            pb = to_world(fr, sb, t)
            kids.append(f'''Pose {{ {pose(pb + fr['n'] * (off / 2), R)} children [ Shape {{
          appearance USE GALV geometry Box {{ size 0.05 0.04 {off:.3f} }} }} ] }}''')
        bound = f'Pose {{ {pose(p0 + fr["n"] * off, pipe_R)} children [ Cylinder {{ height {L:.3f} radius {r} }} ] }}'
    elif it['type'] == 'ledge':
        d, h, w = 0.45, it['size'][0], it['size'][1]
        pc = p0 + fr['n'] * (d / 2)
        kids = [f'''Pose {{ {pose(pc, R)} children [ Shape {{
          appearance PBRAppearance {{ baseColorMap ImageTexture {{ url [ "{REL}/concrete.jpg" ] filtering 5 }}
            roughness 0.95 metalness 0 }}
          geometry {box_ifs((h, w, d))} }} ] }}''']
        bound = f'Pose {{ {pose(pc, R)} children [ Box {{ size {h:.3f} {w:.3f} {d:.3f} }} ] }}'
    elif it['type'] == 'bush':
        g = fr['n'] + 0.6 * fr['up']
        g /= np.linalg.norm(g)
        x = fr['up'] - (fr['up'] @ g) * g
        x /= np.linalg.norm(x)
        Rb = np.c_[x, np.cross(g, x), g]
        sc = it['size'][0] / SIZES['bush'][0]
        kids = [f'''Pose {{ {pose(p0 - g * 0.02, Rb)} children [ Transform {{ scale {sc:.3f} {sc:.3f} {sc:.3f} children [ Shape {{
          appearance PBRAppearance {{ baseColorMap ImageTexture {{ url [ "{REL}/leaf_atlas_{foliage}.png" ] }}
            roughness 0.8 metalness 0 }}
          geometry Mesh {{ url [ "{REL}/bush_{it["variant"] % 2}.obj" ] }} }} ] }} ] }}''']
        bound = f'Pose {{ {pose(p0 + g * 0.28 * sc, Rb)} children [ Sphere {{ radius {0.27 * sc:.3f} }} ] }}'
    else:                                                          # debris: a few fallen rocks
        rng = np.random.default_rng(zlib.crc32(name.encode()))
        kids, bounds = [], []
        for k, (ds, dt, sz) in enumerate(((0, 0, 1.0), (-0.18, 0.2, 0.6), (0.12, -0.22, 0.5))):
            sc = sz * it['size'][0] / SIZES['debris'][0]
            pk = to_world(fr, s + ds, t + dt) + fr['n'] * 0.14 * sc         # main rock top ~0.30 m out
            yaw = rng.uniform(0, 2 * math.pi)
            Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            kids.append(f'''Pose {{ {pose(pk, R @ Rz)} children [ Transform {{ scale {sc:.3f} {sc:.3f} {sc:.3f} children [ Shape {{
          appearance PBRAppearance {{ baseColor 0.6 0.58 0.55 baseColorMap ImageTexture {{ url [ "{REL}/rock.jpg" ] }}
            roughness 1 metalness 0 }}
          geometry Mesh {{ url [ "{REL}/rock_{(it["variant"] + k) % 3}.obj" ] }} }} ] }} ] }}''')
            bounds.append(f'Pose {{ {pose(pk, R)} children [ Box {{ size {0.36 * sc:.3f} {0.3 * sc:.3f} {0.28 * sc:.3f} }} ] }}')
        bound = ' '.join(bounds)
    return f'''    Solid {{  # {name}{'' if it.get('mapped', True) else ' (not on the map)'}
      name "{name}"
      children [
        {chr(10).join('        ' + k if i else k for i, k in enumerate(kids))}
      ]
      boundingObject Group {{ children [ {bound} ] }}
    }}
'''


def box_ifs(size, tile=0.5):
    """Box as an IndexedFaceSet with texture coordinates in units of `tile` metres on every face
    (a Webots Box spreads one texture over its faces in a fixed layout)."""
    hx, hy, hz = (s / 2 for s in size)
    faces = []                                                     # (corner list, u axis, v axis)
    for ax in range(3):
        for sgn in (-1, 1):
            u, v = [(1, 2), (2, 0), (0, 1)][ax]
            if sgn < 0:
                u, v = v, u
            c = np.zeros(3)
            c[ax] = sgn
            pts = []
            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                p = c.copy()
                p[u], p[v] = a, b
                pts.append(p * (hx, hy, hz))
            faces.append((pts, u, v))
    P, T, idx = [], [], []
    for pts, u, v in faces:
        i0 = len(P)
        for p in pts:
            P.append(p)
            T.append((p[u] / tile, p[v] / tile))
        idx += [i0, i0 + 1, i0 + 2, i0 + 3, -1]
    return (f'IndexedFaceSet {{ coord Coordinate {{ point [ {", ".join(f"{a:.4f} {b:.4f} {c:.4f}" for a, b, c in P)} ] }} '
            f'texCoord TextureCoordinate {{ point [ {", ".join(f"{a:.3f} {b:.3f}" for a, b in T)} ] }} '
            f'coordIndex [ {" ".join(map(str, idx))} ] texCoordIndex [ {" ".join(map(str, idx))} ] }}')


def weep_stub(fr, it):
    """Short dark drain stub at the top of the weep-hole stain (cylinder axis = face normal)."""
    p = to_world(fr, it['s'] + it['size'][0] * 0.37, it['t'])
    return f'''    Pose {{ {pose(p + fr['n'] * 0.01, face_R(fr))} children [ Shape {{  # {it['id']} stub
      appearance PBRAppearance {{ baseColor 0.12 0.12 0.12 roughness 0.6 metalness 0.3 }}
      geometry Cylinder {{ height 0.04 radius 0.03 subdivision 32 top FALSE }} }} ] }}
'''


def decal_texture(it):
    return {'crack': f'crack_{it["variant"]}.png', 'wet_patch': f'wet_{it["variant"] % 2}.png',
            'weep_hole': 'weep_stain.png'}.get(it['type'])


def wbt_group(world, items):
    fr = face_frame(world)
    foliage = AREAS[world]['foliage']
    body = ''
    for it in items:
        k = it['type']
        if decal_texture(it):
            body += decal(fr, it, decal_texture(it), DECAL_OFFSET[k])
            if k == 'weep_hole':
                body += weep_stub(fr, it)
        else:
            body += obstacle(fr, it, foliage)
    return f'''DEF INSPECTION Group {{  # tools/inspection.py: {world}, {len(items)} items
  children [
{body}  ]
}}
'''


def replace_group(wbt_text, group):
    pat = re.compile(r'DEF INSPECTION Group \{.*?\n\}\n', re.S)
    if not pat.search(wbt_text):
        raise ValueError('no DEF INSPECTION Group in the world')
    return pat.sub(lambda m: group, wbt_text, count=1)


# ------------------------------------------------------------------ assets (PIL)
def make_assets():
    from PIL import Image, ImageDraw, ImageFilter
    ASSETS.mkdir(parents=True, exist_ok=True)

    def fbm(n, rng, octaves=5, base=4):
        out = np.zeros((n, n))
        for o in range(octaves):
            k = base * 2 ** o
            g = rng.standard_normal((k + 1, k + 1))
            out += np.asarray(Image.fromarray(g.astype(np.float32)).resize((n, n), Image.BICUBIC)) / 2 ** o
        return (out - out.mean()) / out.std()

    # cracks: main crack + branches + hairlines, weathered halo; variant 3 with white leaching
    for v in range(4):
        rng = np.random.default_rng(100 + v)
        n, big = 1024, 2048
        m = Image.new('L', (big, big), 0)
        d = ImageDraw.Draw(m)

        def walk(p, h, length, w0):
            pts, h0 = [np.array(p, float)], h
            for _ in range(int(length / 8)):
                h = 0.8 * (h + rng.normal(0, 0.35)) + 0.2 * h0
                pts.append(pts[-1] + 8 * np.array([math.cos(h), math.sin(h)]))
            w = w0 * np.sin(np.linspace(0.12, math.pi - 0.12, len(pts))) ** 0.6 + 1.6
            for k in range(len(pts) - 1):                    # stamp every ~1 px: continuous line
                for f in np.linspace(0, 1, 9)[:-1]:
                    x, y = pts[k] + (pts[k + 1] - pts[k]) * f
                    r = w[k] + (w[k + 1] - w[k]) * f
                    d.ellipse([x - r / 2, y - r / 2, x + r / 2, y + r / 2], fill=255)
            return pts, w
        h = rng.uniform(-0.5, 0.5)
        pts, w = walk((big * 0.08, big * rng.uniform(0.35, 0.65)), h, big * 0.85, rng.uniform(12, 18))
        for _ in range(rng.integers(3, 6)):
            i = rng.integers(len(pts) // 6, len(pts) * 5 // 6)
            walk(pts[i], h + rng.choice([-1, 1]) * rng.uniform(0.5, 1.1), big * rng.uniform(0.12, 0.32), w[i] * 0.55)
        for _ in range(rng.integers(2, 4)):
            i = rng.integers(0, len(pts))
            walk(pts[i], rng.uniform(-math.pi, math.pi), big * rng.uniform(0.05, 0.12), 3)
        mask = np.asarray(m.resize((n, n), Image.LANCZOS), float) / 255
        halo = np.asarray(m.resize((n, n), Image.LANCZOS).filter(ImageFilter.GaussianBlur(7)), float) / 255
        rgba = np.zeros((n, n, 4))
        if v == 3:                                           # leaching: white crust running down
            leach = np.zeros((n, n))
            for k in range(0, 60, 3):
                leach[k:] = np.maximum(leach[k:], mask[:n - k] * (1 - k / 60) ** 2)
            leach = np.asarray(Image.fromarray((leach * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(3)), float) / 255
            rgba[..., :3] = [222, 220, 210]
            rgba[..., 3] = np.clip(leach * 0.4, 0, 1) * 255
        a_halo = np.clip(halo * 0.35, 0, 1)
        a_core = np.clip(mask * 1.1, 0, 1)
        for col, a in (((70, 64, 58), a_halo), ((22, 19, 17), a_core)):
            A = a[..., None]
            prev_a = rgba[..., 3:] / 255
            out_a = A + prev_a * (1 - A)
            rgba[..., :3] = (np.array(col) * A + rgba[..., :3] * prev_a * (1 - A)) / np.maximum(out_a, 1e-6)
            rgba[..., 3:] = out_a * 255
        Image.fromarray(rgba.clip(0, 255).astype(np.uint8), 'RGBA').save(ASSETS / f'crack_{v}.png')

    # wet patches (seepage): soft irregular damp zone in the upper third, darker and slightly
    # green-brown at its core, many wavy drip runs below it, a few faint efflorescence specks
    for v in range(2):
        rng = np.random.default_rng(200 + v)
        n = 512
        yy, xx = np.mgrid[0:n, 0:n] / n
        r = np.hypot((xx - 0.5) / 0.30, (yy - 0.30) / 0.20)
        val = 1 - r + 0.35 * fbm(n, rng, 5, 3)
        damp = np.clip((val + 0.1) / 0.6, 0, 1) ** 1.5                   # soft edge, no outline
        core = np.clip((val - 0.35) / 0.4, 0, 1)
        runs = np.zeros((n, n))
        for _ in range(40):
            x0 = 0.5 + rng.normal(0, 0.12)
            top = 0.25 + rng.uniform(0, 0.15)
            length = rng.uniform(0.2, 0.68)
            width = rng.uniform(0.002, 0.009)
            prof = np.clip(1 - (yy - top) / length, 0, 1) ** 0.7 * (yy > top)
            wig = x0 + 0.006 * np.sin(yy * rng.uniform(15, 45) + rng.uniform(0, 6)) + 0.01 * (yy - top) * rng.normal()
            runs = np.maximum(runs, prof * np.exp(-((xx - wig) / width) ** 2) * rng.uniform(0.4, 1.0))
        a = np.clip(damp * 0.55 + core * 0.18 + runs * 0.6, 0, 0.78)
        col = np.array([46, 48, 48]) * (1 - core[..., None]) + np.array([40, 44, 34]) * core[..., None]
        specks = (rng.random((n, n)) < 0.002) * (damp > 0.2) * (damp < 0.5)
        rgba = np.zeros((n, n, 4))
        rgba[..., :3] = np.where(specks[..., None], [215, 212, 205], col)
        rgba[..., 3] = np.where(specks, 0.35, a) * 255
        img = Image.fromarray(rgba.clip(0, 255).astype(np.uint8), 'RGBA').filter(ImageFilter.GaussianBlur(0.7))
        img.save(ASSETS / f'wet_{v}.png')

    # weep hole: dark hole at the top, rust streak below
    rng = np.random.default_rng(300)
    w, h = 256, 512                                       # powers of two (Webots rescales others)
    yy, xx = np.mgrid[0:h, 0:w].astype(float)
    hole = np.clip(1 - np.hypot(xx - w / 2, yy - 52) / 40, 0, 1) ** 0.3
    spread = 26 + (yy - 52) * 0.1
    wig = w / 2 + 6 * np.sin(yy / 23) + 3 * np.sin(yy / 7)
    streak = np.exp(-((xx - wig) / spread) ** 2) * np.clip(1 - (yy - 52) / (h - 60), 0, 1) ** 0.6 * (yy > 40)
    rgba = np.zeros((h, w, 4))
    rgba[..., :3] = np.array([78, 52, 32]) * (1 - hole[..., None]) + np.array([10, 9, 8]) * hole[..., None]
    rgba[..., 3] = np.clip(hole + streak * 0.8, 0, 1) * 255
    Image.fromarray(rgba.astype(np.uint8), 'RGBA').save(ASSETS / 'weep_stain.png')

    # rock and concrete textures
    rng = np.random.default_rng(400)
    t = fbm(512, rng, 6, 4)
    t2 = fbm(512, rng, 4, 2)
    spk = (rng.random((512, 512)) < 0.03) * rng.uniform(-70, 50, (512, 512))
    rock = np.clip(np.array([122, 112, 98])[None, None] + t[..., None] * 34 + t2[..., None] * np.array([14, 8, 2])
                   + spk[..., None], 0, 255)
    Image.fromarray(rock.astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6)).save(ASSETS / 'rock.jpg', quality=92)
    t = fbm(512, rng, 6, 6)
    pores = (rng.random((512, 512)) < 0.004) * -45
    conc = np.clip(np.array([152, 150, 144])[None, None] + t[..., None] * 9 + pores[..., None], 0, 255)
    Image.fromarray(conc.astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.5)).save(ASSETS / 'concrete.jpg', quality=92)

    # leaf colour atlases (8 x 8 cells) and bush / rock meshes
    pal = {'green': [(52, 86, 38), (70, 105, 45), (88, 122, 52), (45, 72, 34), (102, 132, 60), (62, 96, 40), (78, 98, 44)],
           'winter': [(58, 72, 40), (70, 84, 46), (48, 62, 36), (84, 92, 52), (104, 92, 60), (96, 80, 50), (62, 76, 42)]}
    for name, cols in pal.items():
        img = np.zeros((256, 256, 3))
        for i in range(8):
            for j in range(8):
                c = np.array(cols[rng.integers(len(cols))]) * rng.uniform(0.85, 1.12)
                img[i * 32:(i + 1) * 32, j * 32:(j + 1) * 32] = c
        Image.fromarray(img.clip(0, 255).astype(np.uint8)).save(ASSETS / f'leaf_atlas_{name}.png')
    for v in range(2):
        write_obj(ASSETS / f'bush_{v}.obj', *bush_mesh(np.random.default_rng(500 + v)))
    for v in range(3):
        write_obj(ASSETS / f'rock_{v}.obj', *rock_mesh(np.random.default_rng(600 + v)))
    # mark points: where each decal texture actually shows the defect (texture u, v in 0..1,
    # v up), so the detector can require the crack line / damp area itself to be in view
    marks = {}
    rng = np.random.default_rng(700)
    for name, thr, k in ([(f'crack_{v}.png', 200, 160) for v in range(4)] + [(f'wet_{v}.png', 90, 160) for v in range(2)]
                         + [('weep_stain.png', 110, 60)]):
        a = np.asarray(Image.open(ASSETS / name))[..., 3]
        ys, xs = np.nonzero(a >= thr)
        pick = rng.choice(len(xs), min(k, len(xs)), replace=False)
        marks[name] = [[round(float(xs[i]) / a.shape[1], 4), round(1 - float(ys[i]) / a.shape[0], 4)] for i in pick]
    (ASSETS / 'marks.yaml').write_text('# tools/inspection.py: defect pixels of each decal texture (u, v; v up)\n'
                                       + yaml.safe_dump(marks, default_flow_style=None))
    print('assets written to', ASSETS)


def write_obj(path, V, N, UV, F):
    lines = [f'v {a:.5f} {b:.5f} {c:.5f}' for a, b, c in V] + [f'vn {a:.4f} {b:.4f} {c:.4f}' for a, b, c in N]
    lines += [f'vt {a:.4f} {b:.4f}' for a, b in UV]
    lines += ['f ' + ' '.join(f'{i}/{i}/{i}' for i in tri) for tri in (np.array(F) + 1)]
    path.write_text('\n'.join(lines) + '\n')


def bush_mesh(rng, leaves=2200):
    """Woody stems + ~2200 small solid leaves in an outward-biased dome (local z = growth axis).
    Leaf normals point away from the bush centre so it shades like a soft volume; UVs pick a
    random cell of the leaf colour atlas."""
    V, N, UV, F = [], [], [], []
    centre = np.array([0, 0, 0.28])
    for _ in range(leaves):
        d = rng.normal(size=3)
        d[2] = abs(d[2]) * 1.3 + 0.1
        d /= np.linalg.norm(d)
        p = centre + d * np.array([0.33, 0.33, 0.26]) * rng.uniform(0.55, 1.0) ** 0.5
        p[2] = max(p[2], 0.03)
        nrm = (p - centre) / np.linalg.norm(p - centre)
        a = rng.normal(size=3)
        a -= (a @ nrm) * nrm
        a /= np.linalg.norm(a)
        b = np.cross(nrm, a)
        L, W = rng.uniform(0.025, 0.045), rng.uniform(0.012, 0.02)
        tilt = nrm * rng.uniform(-0.3, 0.3)
        quad = [p - a * L / 2, p - b * W / 2 + tilt * W, p + a * L / 2, p + b * W / 2 - tilt * W, p + a * L * 0.15]
        i0 = len(V)
        cu, cv = (rng.integers(8) + 0.5) / 8, (rng.integers(8) + 0.5) / 8
        for q in quad:
            V.append(q)
            N.append(nrm)
            UV.append((cu, cv))
        F += [(i0, i0 + 1, i0 + 4), (i0 + 4, i0 + 1, i0 + 2), (i0, i0 + 4, i0 + 3), (i0 + 4, i0 + 2, i0 + 3)]
        F += [(i0 + 4, i0 + 1, i0), (i0 + 2, i0 + 1, i0 + 4), (i0 + 3, i0 + 4, i0), (i0 + 3, i0 + 2, i0 + 4)]
    for _ in range(5):                                       # stems (brown cell of the atlas edge)
        tip = centre + rng.normal(size=3) * [0.12, 0.12, 0.05]
        path = [np.zeros(3) + rng.normal(size=3) * [0.02, 0.02, 0]]
        for k in range(1, 9):
            path.append(path[0] + (tip - path[0]) * k / 8 + np.array([0, 0, 0.03 * math.sin(k / 8 * math.pi)]))
        for k in range(len(path) - 1):
            r0, r1 = 0.012 * (1 - k / 9), 0.012 * (1 - (k + 1) / 9)
            ax = path[k + 1] - path[k]
            ax /= np.linalg.norm(ax)
            u = np.cross(ax, [1, 0, 0]) if abs(ax[0]) < 0.9 else np.cross(ax, [0, 1, 0])
            u /= np.linalg.norm(u)
            w = np.cross(ax, u)
            i0 = len(V)
            for j in range(10):
                ang = 2 * math.pi * j / 10
                dvec = u * math.cos(ang) + w * math.sin(ang)
                for pp, rr in ((path[k], r0), (path[k + 1], r1)):
                    V.append(pp + dvec * rr)
                    N.append(dvec)
                    UV.append((0.06, 0.06))
            for j in range(10):
                a0, b0 = i0 + 2 * j, i0 + 2 * ((j + 1) % 10)
                F += [(a0, b0, a0 + 1), (a0 + 1, b0, b0 + 1)]
    return np.array(V), np.array(N), np.array(UV), F


def rock_mesh(rng, level=4):
    """Broken-stone rock: a subdivided icosphere pushed onto a random convex polyhedron (14 cut
    planes -> large flat facets and crisp edges) plus fine noise; flat-shaded triangles, planar
    UVs (in metres x 2)."""
    t = (1 + 5 ** 0.5) / 2
    V = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    F = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
         (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
         (8, 6, 7), (9, 8, 1)]
    V = [np.array(v, float) / np.linalg.norm(v) for v in V]
    for _ in range(level):
        cache, F2 = {}, []

        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k not in cache:
                m = V[a] + V[b]
                V.append(m / np.linalg.norm(m))
                cache[k] = len(V) - 1
            return cache[k]
        for a, b, c in F:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            F2 += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        F = F2
    D = np.array(V)
    planes = rng.normal(size=(14, 3))
    planes /= np.linalg.norm(planes, axis=1)[:, None]
    dist = rng.uniform(0.62, 0.95, 14)
    cosang = D @ planes.T                                          # radius to each cut plane
    r = np.min(np.where(cosang > 1e-3, dist / np.maximum(cosang, 1e-3), 9.0), axis=1)
    r = np.minimum(r, 1.15)
    noise = np.zeros(len(D))
    for _ in range(10):
        k = rng.normal(size=3) * rng.uniform(4, 12)
        noise += np.sin(D @ k + rng.uniform(0, 6.28)) / np.linalg.norm(k)
    V = D * (r * (1 + 0.04 * noise / np.abs(noise).max()))[:, None] * np.array([0.2, 0.16, 0.14])
    # flat shading: own vertices per triangle
    T = V[np.array(F)]
    fn = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    fn /= np.linalg.norm(fn, axis=1)[:, None]
    Vf = T.reshape(-1, 3)
    Nf = np.repeat(fn, 3, axis=0)
    ax = np.argmax(np.abs(Nf), axis=1)                             # planar (box) UV projection
    UVf = np.where(ax[:, None] == 0, Vf[:, [1, 2]], np.where(ax[:, None] == 1, Vf[:, [0, 2]], Vf[:, [0, 1]])) * 2
    Ff = [(3 * i, 3 * i + 1, 3 * i + 2) for i in range(len(F))]
    return Vf, Nf, UVf, Ff


# ------------------------------------------------------------------ main
def fixed_group(world):
    """Fixed layout (FIXED_SEED): writes the layout yaml and returns the Webots group."""
    items = layout(world, FIXED_SEED)
    save_layout(world, items, LAYOUTS / f'{world}.yaml', FIXED_SEED)
    import wall_map
    wall_map.write_map(LAYOUTS / f'{world}.yaml', WORLDS / 'maps' / world)
    return wbt_group(world, items)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--assets', action='store_true')
    ap.add_argument('--world')
    ap.add_argument('--seed', type=int, default=FIXED_SEED)
    ap.add_argument('--wbt')
    ap.add_argument('--out')
    ap.add_argument('--layout')
    ap.add_argument('--map', help='also write the Nav2 wall map (<path>.yaml + .pgm, tools/wall_map.py)')
    a = ap.parse_args()
    if a.assets:
        make_assets()
    if a.world:
        items = layout(a.world, a.seed)
        out = Path(a.layout) if a.layout else LAYOUTS / f'{a.world}.yaml'
        save_layout(a.world, items, out, a.seed)
        if a.wbt:
            Path(a.out).write_text(replace_group(Path(a.wbt).read_text(), wbt_group(a.world, items)))
        if a.map:
            import wall_map
            wall_map.write_map(out, Path(a.map))
        print(f'{a.world}: {len(items)} items (seed {a.seed})')


if __name__ == '__main__':
    main()
