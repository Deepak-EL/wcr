#!/usr/bin/env python3
"""Build the Gazebo dam world from the Sketchfab scan `dam_during_winter.glb`.

Model: "Dam during winter" by orphanrtg, CC-BY-4.0
https://sketchfab.com/3d-models/dam-during-winter-15f640fda8f14a4997850ab62d87a99b

Steps
  1. glTF Y-up -> Z-up, yaw so the downstream face normal is exactly -y, scale x SCALE.
  2. Fit planes to the downstream slope (~53.6 deg), the upper wall under the crest
     (~84 deg, slight overhang) and the basin floor, inside the plain working section
     (the curved ogee spillway is excluded).
  3. Flatten the scan vertices onto those planes inside the working section, so the
     textured visual and the box collisions coincide (scan is ~7 cm rms ragged).
  4. Write OBJ + MTL with JPEG textures (8K with --textures), the SDF world, and dam_geometry.yaml
     (planes, spawn pose) that the launch files and nodes read.

World frame: x along the dam, face toe line at y = 0 in the middle of the working
section, floor at z = 0, dam body toward +y, face outward normal toward -y.

usage: python tools/build_dam_world.py --glb path/to/dam_during_winter.glb [--scale 25]
                                       [--textures assets_src/dam_winter_8k/textures]
needs: pip install trimesh numpy pillow pyyaml
"""
import argparse
import io
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / 'wcr_description' / 'worlds'
OUT_MESH = WORLDS / 'meshes' / 'dam'

# Working section and surface bands, in aligned scan units (before scaling).
# Profile: straight 53.6 deg slope to z~0.54, a short curve into a vertical wall at
# y~0.535 (z 0.61-0.70), then a cornice that juts back out under the crest (z~0.74).
WORK_X = (-0.10, 2.05)        # plain face, right of the spillway
SLOPE_FIT_Z = (0.105, 0.535)  # straight part of the slope used for the fit
WALL_FIT_Z = (0.610, 0.695)   # vertical part used for the fit
WALL_Y = 0.535                # rough wall position for the fit window
WALL_TOP_Z = 0.700            # cornice starts here; the crest box covers above it
FLOOR_DY = 1.0                # floor flattened this far downstream of the toe
# scan debris: a sparse vertical sheet hanging off the crest in front of the wall
CURTAIN = dict(y=(0.485, 0.512), z=(0.575, 0.712))
TAPER = 0.012                 # blend band at the edges of flattened regions
TOL = 0.02                    # only vertices this close to a plane are moved
TEX_SIZE = 8192      # cap; GLB textures are 4K, the original download 8K (2K looked soft)
DETAIL_OFFSET = 0.003  # concrete grain overlay above the flattened surfaces [m]
DETAIL_TILE = 1.0      # grain texture repeat [m]
DETAIL_PX = 2048       # grain texture size (0.5 mm per pixel)
DETAIL_ALPHA = 0.3     # strongest grain opacity


def load_scene(glb):
    """Return list of (mesh, PIL texture) in aligned Z-up scan units."""
    scene = trimesh.load(glb)
    out = []
    for node in scene.graph.nodes_geometry:
        mat, name = scene.graph[node]
        g = scene.geometry[name].copy()
        g.apply_transform(mat)
        tex = g.visual.material.baseColorTexture if hasattr(g.visual.material, 'baseColorTexture') else g.visual.material.image
        v = g.vertices.copy()
        g.vertices = np.c_[v[:, 0], -v[:, 2], v[:, 1]]          # Y-up -> Z-up
        out.append((g, tex))
    return out


def original_textures(parts, tex_dir):
    """Swap in the 8K originals from the Sketchfab "original format" download.

    The zip's DAM_u?_v?_diffuse-sharpen.jpeg tiles are stored upside down relative to the GLB
    textures; each part gets the (flipped) tile that best matches its GLB texture.
    """
    Image.MAX_IMAGE_PIXELS = None
    tiles = {}
    for f in sorted(Path(tex_dir).glob('*.jp*g')):
        img = ImageOps.flip(Image.open(f).convert('RGB'))
        tiles[f.name] = (img, np.asarray(img.resize((128, 128), Image.BOX), float))
    out = []
    for g, tex in parts:
        small = np.asarray(tex.convert('RGB').resize((128, 128), Image.BOX), float)
        err, name = min((np.abs(small - t[1]).mean(), n) for n, t in tiles.items())
        print(f'  {name}: mean diff {err:.1f}')
        if err > 5:
            raise SystemExit(f'no original texture matches a GLB part (best {name}, {err:.1f})')
        out.append((g, tiles[name][0]))
    return out


def detail_texture(path, n=DETAIL_PX, seed=3):
    """Seamless concrete grain (band-limited periodic noise + pores) as RGBA: light or dark pixels
    whose alpha is the grain strength, so it lightens/darkens the scan colour underneath."""
    rng = np.random.default_rng(seed)
    mm = DETAIL_TILE * 1000 / n                               # mm per pixel
    f = np.fft.fftfreq(n)
    fr = np.hypot(*np.meshgrid(f, f))

    def band(lo_mm, hi_mm):                                   # features lo..hi mm across
        m = (fr >= mm / (2 * hi_mm)) & (fr <= mm / (2 * lo_mm))
        x = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((n, n))) * m))
        return x / x.std()

    g = 0.5 * band(0.5, 2) + 0.35 * band(3, 15) + 0.3 * band(20, 120)
    sigma = 0.7 / mm                                          # pore radius ~0.7 mm, in pixels
    pores = np.real(np.fft.ifft2(np.fft.fft2(rng.random((n, n)) < 0.0012)
                                 * np.exp(-2 * (math.pi * sigma * fr) ** 2)))
    g = np.clip((g - 6 * pores / pores.max()) / 3, -1, 1)
    rgba = np.empty((n, n, 4), np.uint8)
    rgba[..., :3] = np.where(g[..., None] > 0, [235, 232, 225], [25, 25, 28])
    rgba[..., 3] = np.abs(g) * DETAIL_ALPHA * 255
    Image.fromarray(rgba, 'RGBA').save(path)


def fit_plane(P, outward):
    c = P.mean(0)
    for _ in range(4):
        n = np.linalg.svd(P - c)[2][2]
        n = n if n @ outward > 0 else -n
        r = (P - c) @ n
        keep = np.abs(r) < max(3 * r.std(), 1e-4)
        P = P[keep]
        c = P.mean(0)
    n = np.linalg.svd(P - c)[2][2]
    n = n if n @ outward > 0 else -n
    r = (P - c) @ n
    return n, c, float(r.std())


def fix_uv_outliers(g, face_mask, k=12, tol=0.02):
    """Scan triangles whose texture coordinates point to a different part of the atlas than their
    neighbours (a photogrammetry mapping error, shows as an odd pale patch on the face). Only the
    triangles in face_mask (the flattened climbing surfaces) are checked; they are re-mapped with
    an affine world->UV fit of their neighbours. Elsewhere tiny photo patches are genuine."""
    from scipy.spatial import cKDTree
    c, uv = g.triangles_center, g.visual.uv
    uvc = uv[g.faces].mean(1)
    idx = np.where(face_mask)[0]
    if len(idx) <= k:
        return 0
    _, nb = cKDTree(c[idx]).query(c[idx], k=k + 1)
    nb = idx[nb[:, 1:]]
    # stray = fewer than 30 % of the neighbours use a nearby atlas spot. Seams between photo
    # patches split the neighbours roughly in half (kept); small groups of strays are caught.
    close = (np.linalg.norm(uvc[idx][:, None, :] - uvc[nb], axis=2) < tol).mean(axis=1)
    stray = close < 0.3
    bad = idx[stray]
    bad_set = set(bad.tolist())
    uv = uv.copy()
    for f, neigh in zip(bad, nb[stray]):
        good = [n for n in neigh if n not in bad_set]
        if len(good) < 3:
            continue
        vs = g.faces[good].ravel()
        X = np.c_[g.vertices[vs], np.ones(len(vs))]
        A, *_ = np.linalg.lstsq(X, uv[vs], rcond=None)
        fv = g.faces[f]
        uv[fv] = np.c_[g.vertices[fv], np.ones(3)] @ A
    g.visual.uv = uv
    return len(bad)


def plane_roll(n):
    """Roll about world x that maps local z onto normal n (n has no x component) and local y up the plane."""
    return math.atan2(-n[1], n[2])


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glb', required=True)
    ap.add_argument('--scale', type=float, default=25.0, help='scan units -> metres')
    ap.add_argument('--mu', type=float, default=0.7, help='concrete friction (dry 0.7, wet 0.45)')
    ap.add_argument('--spawn-up', type=float, default=1.5, help='spawn distance up the slope from the toe [m]')
    ap.add_argument('--fillet', type=float, default=2.5,
                    help='radius [m] of the curved slope-to-wall transition (scan: ~2.7 m)')
    ap.add_argument('--textures', help='folder with the 8K original textures (Sketchfab "original '
                    'format" zip, textures/); default: the 4K textures inside the GLB')
    args = ap.parse_args()
    S = args.scale

    parts = load_scene(args.glb)
    if args.textures:
        parts = original_textures(parts, args.textures)
    allm = trimesh.util.concatenate([g for g, _ in parts])
    P, _ = trimesh.sample.sample_surface_even(allm, 400000, seed=1)

    # rough slope guess from the profile: z = 0.09 + (y - 0.17) * tan(55 deg)
    def slope_sel(P):
        return ((P[:, 0] > WORK_X[0]) & (P[:, 0] < WORK_X[1]) & (P[:, 2] > SLOPE_FIT_Z[0]) & (P[:, 2] < SLOPE_FIT_Z[1])
                & (np.abs(P[:, 2] - (0.09 + (P[:, 1] - 0.17) * 1.457)) < 0.04))
    n_s, _, _ = fit_plane(P[slope_sel(P)], np.array([0, -1.0, 0]))
    yaw = -math.atan2(n_s[0], -n_s[1])
    R = rotz(yaw)
    for g, _ in parts:
        g.vertices = g.vertices @ R.T
    P = P @ R.T

    n_s, c_s, rms_s = fit_plane(P[slope_sel(P)], np.array([0, -1.0, 0]))
    floor_sel = ((P[:, 0] > WORK_X[0]) & (P[:, 0] < WORK_X[1]) & (P[:, 1] > -1.0) & (P[:, 1] < 0.12)
                 & (np.abs(P[:, 2] - 0.09) < 0.03))
    z_floor = float(np.median(P[floor_sel, 2]))
    # toe: slope plane meets floor, in the middle of the working section
    xm = 0.5 * (WORK_X[0] + WORK_X[1])
    # n.(p - c) = 0 with p = (xm, y, z_floor)
    y_toe = c_s[1] - (n_s[0] * (xm - c_s[0]) + n_s[2] * (z_floor - c_s[2])) / n_s[1]
    wall_sel = ((P[:, 0] > WORK_X[0]) & (P[:, 0] < WORK_X[1]) & (P[:, 2] > WALL_FIT_Z[0]) & (P[:, 2] < WALL_FIT_Z[1])
                & (np.abs(P[:, 1] - WALL_Y) < 0.012))
    n_w, c_w, rms_w = fit_plane(P[wall_sel], np.array([0, -1.0, 0]))
    # treat the face as an extrusion along x (the wall's 0.2 deg yaw is within scan noise)
    for n in (n_s, n_w):
        n[0] = 0.0
        n /= np.linalg.norm(n)
    # fillet between slope and wall: centre C is R from both planes on the outer side
    Rf = args.fillet / S
    A2 = np.array([[n_s[1], n_s[2]], [n_w[1], n_w[2]]])
    C = np.linalg.solve(A2, [Rf + n_s[1:] @ c_s[1:], Rf + n_w[1:] @ c_w[1:]])
    T1, T2 = C - Rf * n_s[1:], C - Rf * n_w[1:]
    a1, a2 = math.atan2(-n_s[2], -n_s[1]), math.atan2(-n_w[2], -n_w[1])
    crest_sel = (P[:, 0] > WORK_X[0]) & (P[:, 0] < WORK_X[1]) & (P[:, 1] > 0.53) & (P[:, 1] < 0.67) & (P[:, 2] > 0.725)
    z_crest = float(np.median(P[crest_sel, 2]))
    ice_sel = (P[:, 1] > 0.8) & (P[:, 1] < 1.9) & (np.abs(P[:, 2] - 0.65) < 0.03)
    z_ice = float(np.median(P[ice_sel, 2]))
    print(f'yaw {math.degrees(yaw):.2f} deg, slope rms {rms_s * S * 100:.1f} cm, wall rms {rms_w * S * 100:.1f} cm')

    # ---- flatten vertices onto planes inside the working section --------------------
    def band(x, lo, hi):
        return np.clip(np.minimum(x - lo, hi - x) / TAPER, 0, 1)

    moved = dropped = 0
    flat_faces = {}
    for g, _ in parts:
        c, n = g.triangles_center, g.face_normals
        curtain = ((c[:, 0] > WORK_X[0]) & (c[:, 0] < WORK_X[1]) & (c[:, 1] > CURTAIN['y'][0]) & (c[:, 1] < CURTAIN['y'][1])
                   & (c[:, 2] > CURTAIN['z'][0]) & (c[:, 2] < CURTAIN['z'][1]) & (np.abs(n[:, 1]) > 0.9))
        if curtain.any():
            g.update_faces(~curtain)
            dropped += int(curtain.sum())
        v = g.vertices.copy()
        wx = band(v[:, 0], *WORK_X)
        d_s = (v - c_s) @ n_s
        d_w = (v - c_w) @ n_w
        d_f = v[:, 2] - z_floor
        # fillet: polar coordinates about C in the y-z section; the arc spans angles [a1, a2]
        dyz = v[:, 1:] - C
        rr = np.linalg.norm(dyz, axis=1)
        ang = np.arctan2(dyz[:, 1], dyz[:, 0])
        d_a = Rf - rr
        w_s = wx * band(v[:, 2], z_floor - 0.02, 0.650) * (np.abs(d_s) < TOL) * (ang < a1)
        w_w = wx * band(v[:, 2], 0.500, WALL_TOP_Z + TAPER / 2) * (np.abs(d_w) < TOL) * (ang > a2)
        w_a = wx * ((ang >= a1) & (ang <= a2) & (np.abs(d_a) < TOL))
        w_f = wx * band(v[:, 1], y_toe - FLOOR_DY, y_toe + 0.02) * (np.abs(d_f) < TOL)
        # each vertex goes to the surface it is closest to
        best = np.argmin(np.c_[np.where(w_s > 0, np.abs(d_s), 9), np.where(w_w > 0, np.abs(d_w), 9),
                               np.where(w_f > 0, np.abs(d_f), 9), np.where(w_a > 0, np.abs(d_a), 9)], axis=1)
        sel = np.c_[w_s, w_w, w_f, w_a][np.arange(len(v)), best]
        delta = np.zeros_like(v)
        for k, (d, n) in enumerate(((d_s, n_s), (d_w, n_w), (d_f, np.array([0, 0, 1.0])))):
            m = best == k
            delta[m] = -(sel[m] * d[m])[:, None] * n
        m = best == 3
        delta[m, 1:] = (sel[m] * (Rf / np.maximum(rr[m], 1e-9) - 1))[:, None] * dyz[m]
        moved += int((sel > 0).sum())
        g.vertices = v + delta
        flat_faces[id(g)] = (sel > 0.999)[g.faces].all(axis=1)    # fully flattened triangles
    print(f'flattened {moved} vertices, removed {dropped} curtain triangles')
    print(f're-mapped {sum(fix_uv_outliers(g, flat_faces[id(g)]) for g, _ in parts)} '
          f'triangles with stray texture coordinates (on the flattened climbing surfaces)')

    # ---- to world frame (metres) --------------------------------------------------
    origin = np.array([xm, y_toe, z_floor])
    def to_w(p):
        return (np.asarray(p) - origin) * S

    OUT_MESH.mkdir(parents=True, exist_ok=True)
    obj, mtl = io.StringIO(), io.StringIO()
    obj.write('# Dam during winter by orphanrtg (CC-BY-4.0), processed by tools/build_dam_world.py\n')
    obj.write('mtllib dam.mtl\n')
    off, n_off = 1, 1      # next v/vt index, next vn index (vn has two blocks per part)
    for i, (g, tex) in enumerate(parts):
        img = tex.convert('RGB')
        img.thumbnail((TEX_SIZE, TEX_SIZE), Image.LANCZOS)
        img.save(OUT_MESH / f'dam_tex_{i}.jpg', quality=88)
        mtl.write(f'newmtl dam_{i}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd dam_tex_{i}.jpg\n\n')
        V = to_w(g.vertices)
        N = g.vertex_normals
        # The scan is UDIM-tiled: each part's UVs sit in its own tile (u 1..2, v -1..0, ...), which
        # Sketchfab maps but Webots/Gazebo did not (the brown field rendered as snow). Shift into 0..1.
        UV = g.visual.uv - np.floor(np.median(g.visual.uv, axis=0))
        verts = (''.join(f'v {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in V)
                 + ''.join(f'vt {u:.5f} {v:.5f}\n' for u, v in UV)
                 # The scan is a thin open shell shown double-sided on Sketchfab; renderers that cull
                 # back faces (Webots, Gazebo) show holes wherever its back side faces the camera.
                 # Write every triangle for both sides, the back copy flipped with negated normals.
                 + ''.join(f'vn {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in np.concatenate([N, -N])))

        def faces(off, n_off):
            F = g.faces + off
            nb = n_off + len(V)                     # first vn index of the negated block
            return (''.join(f'f {a}/{a}/{a - off + n_off} {b}/{b}/{b - off + n_off} {c}/{c}/{c - off + n_off}\n'
                            for a, b, c in F)
                    + ''.join(f'f {c}/{c}/{c - off + nb} {b}/{b}/{b - off + nb} {a}/{a}/{a - off + nb}\n'
                              for a, b, c in F))
        head = f'o dam_{i}\nusemtl dam_{i}\n'
        obj.write(head + verts + faces(off, n_off))
        # one file per part too: Webots ignores Mesh.name for OBJ objects, so a Shape given the
        # combined file draws all parts, and the 4 textures overlapped (last one won everywhere)
        (OUT_MESH / f'dam_{i}.obj').write_text(f'mtllib dam.mtl\n{head}{verts}{faces(1, 1)}')
        off += len(V)
        n_off += 2 * len(V)
    (OUT_MESH / 'dam.obj').write_text(obj.getvalue())
    (OUT_MESH / 'dam.mtl').write_text(mtl.getvalue())
    (OUT_MESH / 'ATTRIBUTION.txt').write_text(
        '"Dam during winter" by orphanrtg\nhttps://sketchfab.com/3d-models/dam-during-winter-15f640fda8f14a4997850ab62d87a99b\n'
        'License: CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/)\n'
        'Changes: rescaled, re-oriented, face/floor flattened onto fitted planes, made two-sided.\n')

    # ---- geometry in metres -------------------------------------------------------
    width = (WORK_X[1] - WORK_X[0]) * S
    beta = math.acos(n_s[2])                    # slope angle from horizontal
    t_up = np.array([0.0, n_s[2], -n_s[1]])     # up-slope tangent (x component ~0 after yaw)
    t_up /= np.linalg.norm(t_up)
    # fillet tangent points / centre in world (y, z)
    yz = lambda q: to_w([xm, q[0], q[1]])[1:]
    t1, t2, cw_ = yz(T1), yz(T2), yz(C)
    slope_len = float(np.hypot(*t1))                 # toe (origin) -> start of the fillet
    beta_w = math.acos(n_w[2])
    z_crest_w, z_ice_w = (z_crest - z_floor) * S, (z_ice - z_floor) * S
    z_wall_top = (WALL_TOP_Z - z_floor) * S
    wall_len = (z_wall_top - t2[1]) / math.sin(beta_w)
    t_w = np.array([0.0, n_w[2], -n_w[1]]); t_w /= np.linalg.norm(t_w)
    print(f'slope {math.degrees(beta):.2f} deg, {slope_len:.2f} m to the fillet (R {args.fillet} m, '
          f'z {t1[1]:.2f}-{t2[1]:.2f} m); wall {math.degrees(beta_w):.2f} deg, {wall_len:.2f} m; '
          f'crest z {z_crest_w:.2f} m; working width {width:.1f} m')

    # ---- concrete grain overlay on the flattened climbing surfaces --------------------
    # The scan has ~8 mm texels (8K textures), blurry at robot scale. A thin translucent skin
    # DETAIL_OFFSET above the flattened faces carries a tiling grain texture (u = x, v = distance
    # along the floor/slope/fillet/wall profile, both in metres).
    D = trimesh.util.concatenate([trimesh.Trimesh(to_w(g.vertices), g.faces[flat_faces[id(g)]], process=False)
                                  for g, _ in parts])
    D.merge_vertices()
    D.remove_unreferenced_vertices()
    out = D.face_normals @ np.array([0.0, -1.0, 1.0]) >= 0     # outward: floor +z, slope/wall -y
    D = trimesh.Trimesh(D.vertices, np.where(out[:, None], D.faces, D.faces[:, ::-1]), process=False)
    Vd = D.vertices + D.vertex_normals * DETAIL_OFFSET
    y, z = Vd[:, 1], Vd[:, 2]
    c1, c2 = t1 - cw_, t2 - cw_
    arc = lambda d: np.arccos(np.clip(d @ c1 / (np.linalg.norm(d, axis=-1) * np.linalg.norm(c1)), -1, 1))
    s = np.select([(y <= 0) & (z < 0.05), z <= t1[1], z >= t2[1]],
                  [y, Vd[:, 1:] @ t_up[1:], slope_len + args.fillet * arc(c2) + (Vd[:, 1:] - t2) @ t_w[1:]],
                  slope_len + args.fillet * arc(Vd[:, 1:] - cw_))
    UVd = np.c_[Vd[:, 0], s] / DETAIL_TILE
    det = io.StringIO()
    det.write('# concrete grain overlay for dam.obj, generated by tools/build_dam_world.py\n')
    det.write(''.join(f'v {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in Vd))
    det.write(''.join(f'vt {u:.5f} {v:.5f}\n' for u, v in UVd))
    det.write(''.join(f'vn {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in D.vertex_normals))
    det.write(''.join(f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n' for a, b, c in D.faces + 1))
    (OUT_MESH / 'dam_detail.obj').write_text(det.getvalue())
    detail_texture(OUT_MESH / 'dam_detail.png')
    print(f'detail overlay: {len(D.faces)} triangles, {DETAIL_TILE} m tile')

    # robot pose on the slope: body z = n_s, body x = up-slope; rpy for R = Rz(yaw) Ry(pitch)
    spawn_s = args.spawn_up
    wheel_r, base_h = 0.0254, 0.0           # base_link sits at wheel contact plane
    p_spawn = t_up * spawn_s + np.array(n_s) * (base_h + 0.003)
    spawn = dict(x=float(p_spawn[0]), y=float(p_spawn[1]), z=float(p_spawn[2]),
                 roll=0.0, pitch=float(-beta), yaw=float(math.pi / 2))
    # cradle: ledge just below the rear wheels (rear axle 0.1125 m behind base_link)
    cradle_s = spawn_s - 0.1125 - wheel_r - 0.012
    p_cr = t_up * cradle_s

    def plain(o):
        if isinstance(o, dict):
            return {k: plain(v) for k, v in o.items()}
        if isinstance(o, (list, tuple, np.ndarray)):
            return [plain(v) for v in o]
        return round(float(o), 5) if isinstance(o, (float, np.floating)) else o

    geom = dict(
        scale=S, mu=args.mu,
        slope=dict(angle_deg=math.degrees(beta), normal=n_s, up=t_up, base=[0.0, 0.0, 0.0],
                   length_m=slope_len, rms_cm=rms_s * S * 100),
        fillet=dict(radius_m=args.fillet, centre=[0.0, *cw_], start=[0.0, *t1], end=[0.0, *t2]),
        wall=dict(angle_deg=math.degrees(beta_w), normal=n_w, up=t_w, base=[0.0, *t2],
                  length_m=wall_len, rms_cm=rms_w * S * 100),
        wall_top_z=z_wall_top, crest_z=z_crest_w, ice_z=z_ice_w, working_x=[-width / 2, width / 2],
        spawn=spawn, cradle=p_cr)
    (WORLDS / 'dam_geometry.yaml').write_text(
        '# Generated by tools/build_dam_world.py (world frame, metres)\n' + yaml.safe_dump(plain(geom), sort_keys=False))

    def box_on_plane(name, n, base, along, width, thick, mu):
        """Static box whose outer face lies on plane (n, base), spanning `along` metres up the plane.
        Box axes: x along the dam, y up the plane, z = n.  Pose is Rz(yaw) Rx(roll)."""
        n = np.asarray(n)
        yaw, roll = math.atan2(n[0], -n[1]), math.atan2(math.hypot(n[0], n[1]), n[2])
        up = np.array([-math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.cos(roll), math.sin(roll)])
        centre = np.asarray(base) + up * along / 2 - n * thick / 2
        return f'''
        <collision name="{name}">
          <pose>{centre[0]:.4f} {centre[1]:.4f} {centre[2]:.4f} {roll:.6f} 0 {yaw:.6f}</pose>
          <geometry><box><size>{width:.3f} {along:.3f} {thick:.3f}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>'''

    mu = args.mu
    slope_box = box_on_plane('slope', n_s, [0, 0, 0], slope_len, width, 1.0, mu)
    wall_box = box_on_plane('upper_wall', n_w, [0, *t2], wall_len, width, 1.0, mu)
    # fillet as short chords (each ~6 cm, < 2.5 deg apart), thin boxes behind each chord
    Rw = args.fillet
    n_seg = max(4, int(math.ceil(Rw * (a2 - a1) / 0.06)))
    for i in range(n_seg):
        p0 = cw_ + Rw * np.array([math.cos(a1 + (a2 - a1) * i / n_seg), math.sin(a1 + (a2 - a1) * i / n_seg)])
        p1 = cw_ + Rw * np.array([math.cos(a1 + (a2 - a1) * (i + 1) / n_seg), math.sin(a1 + (a2 - a1) * (i + 1) / n_seg)])
        tang = (p1 - p0) / np.linalg.norm(p1 - p0)
        nrm = np.array([0.0, -tang[1], tang[0]])          # outward (toward the fillet centre)
        slope_box += box_on_plane(f'fillet_{i:02d}', nrm, [0, *p0], float(np.linalg.norm(p1 - p0)) + 0.002,
                                  width, 0.3, mu)
    # crest + cornice: one box from the cornice lip (y 0.495) back over the crest, under the walkway
    crest_y = ((0.495 + 0.685) / 2 - y_toe) * S
    crest_t = z_crest_w - z_wall_top
    sdf = WORLD_TEMPLATE.format(
        mu=mu, slope=slope_box, wall=wall_box, crest_y=crest_y, crest_z=z_crest_w - crest_t / 2,
        crest_dy=0.19 * S, crest_t=crest_t, width=width, cr=p_cr, cr_roll=plane_roll(n_s), cam_y=-14.0, cam_z=4.0)
    (WORLDS / 'dam_world.sdf').write_text(sdf)
    print('wrote', WORLDS / 'dam_world.sdf', 'and', OUT_MESH / 'dam.obj')
    print('spawn', {k: round(v, 4) for k, v in spawn.items()})


WORLD_TEMPLATE = '''<?xml version="1.0" ?>
<!-- Generated by tools/build_dam_world.py - do not edit by hand.
     Dam scan: "Dam during winter" by orphanrtg, CC-BY-4.0. -->
<sdf version="1.8">
  <world name="dam_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.7 0.72 0.76 1</ambient>
      <background>0.72 0.78 0.86 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 -30 40 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.2 0.6 -0.75</direction>
    </light>

    <model name="dam">
      <static>true</static>
      <link name="body">
        <visual name="scan">
          <geometry><mesh><uri>package://wcr_description/worlds/meshes/dam/dam.obj</uri></mesh></geometry>
        </visual>
        <!-- Collision: planes fitted to the scan; the scan is flattened onto them. -->{slope}{wall}
        <collision name="floor">
          <pose>0 -20 -0.5 0 0 0</pose>
          <geometry><box><size>200 40 1</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <collision name="crest_and_cornice">
          <pose>0 {crest_y:.3f} {crest_z:.3f} 0 0 0</pose>
          <geometry><box><size>{width:.3f} {crest_dy:.3f} {crest_t:.3f}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>

    <!-- Deployment cradle: a ledge on the face the robot rests on until adhesion is on. -->
    <model name="cradle">
      <static>true</static>
      <pose>{cr[0]:.4f} {cr[1]:.4f} {cr[2]:.4f} {cr_roll:.6f} 0 0</pose>
      <link name="ledge">
        <collision name="c">
          <pose>0 -0.01 0.03 0 0 0</pose>
          <geometry><box><size>0.45 0.02 0.06</size></box></geometry>
        </collision>
        <visual name="v">
          <pose>0 -0.01 0.03 0 0 0</pose>
          <geometry><box><size>0.45 0.02 0.06</size></box></geometry>
          <material><ambient>0.9 0.6 0.05 1</ambient><diffuse>0.9 0.6 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Fixed overview camera on the spawn area (1 Hz; for screenshots and docs) -->
    <model name="overview_camera">
      <static>true</static>
      <pose>2.6 -2.2 2.6 0 0.27 2.27</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <update_rate>1</update_rate>
          <always_on>true</always_on>
          <topic>/overview/image</topic>
          <camera><horizontal_fov>1.0</horizontal_fov><image><width>1280</width><height>720</height></image>
            <clip><near>0.05</near><far>300</far></clip></camera>
        </sensor>
      </link>
    </model>

    <gui fullscreen="0">
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre</engine>
        <scene>scene</scene>
        <ambient_light>0.7 0.72 0.76</ambient_light>
        <background_color>0.72 0.78 0.86</background_color>
        <camera_pose>0 {cam_y} {cam_z} 0 0.12 1.5708</camera_pose>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <ignition-gui><title>World control</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">72</property>
          <property type="double" key="width">121</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors>
        </ignition-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property><property type="double" key="height">110</property>
          <property type="double" key="width">290</property><property type="double" key="z">1</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors>
        </ignition-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree"/>
    </gui>
  </world>
</sdf>
'''

if __name__ == '__main__':
    main()
