#!/usr/bin/env python3
"""Export the GoldenEye 007 dam level (Sketchfab, baked lighting) for Webots.

Model: "GoldenEye 007 - Dam Level Baked" by Vince_Crusty, CC-BY-4.0
https://sketchfab.com/3d-models/goldeneye-007-dam-level-baked-94a3dfb06fc2480fb19f1db3efa83169

The materials are KHR_materials_unlit with the lighting baked into the textures, so in Webots
they are drawn as emissive (baseColor black, emissiveColorMap = texture): no scene light changes
them, as on Sketchfab. Skinned meshes (animated birds) are skipped. glTF Y-up -> Webots Z-up.
One OBJ per mesh (Webots ignores Mesh.name, so multi-object files draw every object).

The robot climbs the upstream (reservoir) face above the water, which stays as in the model.

Writes wcr_description/worlds/meshes/goldeneye_dam/{obj_<n>.obj, tex_<k>.jpg|png, parts.yaml},
       wcr_description/worlds/goldeneye_dam_geometry.yaml (face plane, waterline, spawn) and
       webots/worlds/goldeneye_dam_preview.wbt (no robot); build_webots.py writes goldeneye_dam.wbt
usage: python tools/build_goldeneye_world.py --glb assets_src/goldeneye_dam/goldeneye_dam.glb
needs: pip install trimesh numpy pillow pyyaml
"""
import argparse
import json
import struct
from pathlib import Path

import numpy as np
import trimesh
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'wcr_description' / 'worlds' / 'meshes' / 'goldeneye_dam'
ATTRIBUTION = ('"GoldenEye 007 - Dam Level Baked" by Vince_Crusty\n'
               'https://sketchfab.com/3d-models/goldeneye-007-dam-level-baked-94a3dfb06fc2480fb19f1db3efa83169\n'
               'License: CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/)\n'
               'Changes: rotated to Z-up, animated (skinned) birds removed, made two-sided, split per mesh.\n')


WORLDS = ROOT / 'wcr_description' / 'worlds'
SPAWN_S = 0.8                                    # spawn distance up the face from its base [m]
WATER = 'Material.012'                           # reservoir surface (Object_16)
# Climbing sites, each a planar strip of the dam:
#  reservoir: the plain 72 deg upstream face between the central tower (x~0) and the next arch
#             segment (x<-42), from the waterline (z~-7.5) up to the crest walkway (z 0);
#             a floating pontoon at the waterline.
#  gorge:     the dry downstream face, ~vertical, from z -104.5 up to the crest (105 m); a
#             hanging work platform at the foot. Its visible surface is a patchwork of two
#             interpenetrating meshes (Object_11 wall, Object_12 central panel) whose facets
#             change with height, so instead of a fitted plane the robot touches the visible
#             surface itself (collision mesh) and climbs at x -28..-16, where the panel is
#             visible from bottom to top; the grain skin follows that surface.
SITES = {
    'reservoir': dict(yaml='goldeneye_dam_geometry.yaml', meshes=('obj_Object_7', 'obj_Object_11'),
                      ref=(-0.22, -0.92, 0.32), strip_x=(-40.0, -17.0), z=(-9.0, 0.5), base='water',
                      side='upstream (reservoir) face above the water', platform='pontoon'),
    'gorge': dict(yaml='goldeneye_gorge_geometry.yaml', meshes=('obj_Object_11', 'obj_Object_12'),
                  ref=(0.24, 0.97, 0.0), strip_x=(-28.0, -16.0), z=(-104.5, 0.0), base=-104.5, surface=True,
                  side='downstream (gorge) face, dry', platform='hanging work platform'),
}


def first_hits(meshes, origins, d):
    """Nearest hit of rays (origins, common direction d) on any of the meshes:
    points (NaN where missed) and unit normals oriented against d."""
    best_t = np.full(len(origins), np.inf)
    pts = np.full((len(origins), 3), np.nan)
    nrm = np.full((len(origins), 3), np.nan)
    D = np.tile(d, (len(origins), 1))
    for m in meshes:
        loc, ray, tri = m.ray.intersects_location(origins, D, multiple_hits=False)
        if len(ray) == 0:
            continue
        t = ((loc - origins[ray]) * D[ray]).sum(1)
        better = t < best_t[ray]
        r = ray[better]
        best_t[r], pts[r], nrm[r] = t[better], loc[better], m.face_normals[tri[better]]
    nrm *= -np.sign((nrm @ d))[:, None]
    return pts, nrm


def surface_meshes(site, meshes, ref, z_base):
    """Gorge site: collision mesh = the meshes' triangles facing the climbing side near the
    strip (single-sided, wound outwards); grain skin = a 0.5 m grid ray-cast onto the visible
    surface, 3 mm out, texture coordinates in metres (x, z)."""
    S = SITES[site]
    x0, x1 = S['strip_x']
    V, F, off = [], [], 0
    for m in meshes:
        T = m.faces[:len(m.faces) // 2]                                  # front copies
        n = m.face_normals[:len(T)]
        c = m.triangles_center[:len(T)]
        flip = n @ ref < 0
        T = np.where(flip[:, None], T[:, ::-1], T)
        keep = ((np.abs(n @ ref) > 0.5) & (c[:, 0] > x0 - 15) & (c[:, 0] < x1 + 15)
                & (c[:, 2] > z_base - 5) & (c[:, 2] < 1.0))
        V.append(m.vertices)
        F.append(T[keep] + off)
        off += len(m.vertices)
    V, F = np.concatenate(V), np.concatenate(F)
    (OUT / f'{site}_collision.obj').write_text(
        '\n'.join([f'v {a:.4f} {b:.4f} {c:.4f}' for a, b, c in V] + [f'f {a} {b} {c}' for a, b, c in F + 1]) + '\n')
    xs = np.arange(x0, x1 + 1e-6, 0.5)
    zs = np.arange(z_base + 0.05, S['z'][1] - 0.15, 0.5)
    X, Z = np.meshgrid(xs, zs)
    orig = np.c_[X.ravel(), np.full(X.size, 100.0) * np.sign(ref[1]), Z.ravel()]
    p, nn = first_hits(meshes, orig, np.array([0, -np.sign(ref[1]), 0.0]))
    G = (p + nn * 0.003).reshape(len(zs), len(xs), 3)
    ok = np.isfinite(G[..., 0])
    lines = [f'# grain skin for the {site} climbing strip, generated by tools/build_goldeneye_world.py']
    idx = -np.ones(ok.shape, int)
    k = 0
    for (i, j), good in np.ndenumerate(ok):
        if good:
            k += 1
            idx[i, j] = k
            lines.append(f'v {G[i, j, 0]:.4f} {G[i, j, 1]:.4f} {G[i, j, 2]:.4f}')
            lines.append(f'vt {X[i, j]:.3f} {Z[i, j]:.3f}')
    nf = 0
    for i in range(len(zs) - 1):
        for j in range(len(xs) - 1):
            q = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)]
            if all(ok[a] for a in q) and np.ptp([G[a][1] for a in q]) < 0.4:   # skip mesh seams
                a, b, c, d = (idx[x] for x in q)
                tri = [(a, b, c), (a, c, d)]
                # wind outwards (towards the climbing side)
                if np.cross(G[q[1]] - G[q[0]], G[q[2]] - G[q[0]]) @ ref < 0:
                    tri = [t[::-1] for t in tri]
                lines += [f'f {u}/{u} {v}/{v} {w}/{w}' for u, v, w in tri]
                nf += 2
    (OUT / f'{site}_grain.obj').write_text('\n'.join(lines) + '\n')
    print(f'{site}: collision mesh {len(F)} triangles, grain skin {nf} triangles')
    return f'{site}_collision.obj', f'{site}_grain.obj'


def face_geometry(site):
    """Fit a site's climbing face, find its base (waterline or given z), place spawn, ledge and
    platform; writes wcr_description/worlds/<site yaml> (world frame, metres)."""
    S = SITES[site]
    STRIP_X = S['strip_x']
    ref = np.array(S['ref']) / np.linalg.norm(S['ref'])
    xc = 0.5 * (STRIP_X[0] + STRIP_X[1])
    extra = {}
    if S.get('surface'):
        # local plane at the foot of the strip; flatness = deviation of the visible surface along
        # the climbing line from that plane
        ms = [trimesh.load(OUT / f'{name}.obj', process=False, force='mesh') for name in S['meshes']]
        d = np.array([0, -np.sign(ref[1]), 0.0])
        p, nn = first_hits(ms, np.array([[xc, 100.0 * np.sign(ref[1]), S['base'] + 0.4]]), d)
        c0, n = p[0], nn[0]
        col, _ = first_hits(ms, np.c_[np.full(100, xc), np.full(100, 100.0 * np.sign(ref[1])),
                                      np.linspace(S['base'], S['z'][1] - 0.2, 100)], d)
        rms = float(np.sqrt(np.nanmean(((col - c0) @ n) ** 2)))
        z_base, z_top = float(S['base']), float(S['z'][1])
        extra = dict(zip(('collision_mesh', 'grain_mesh'), surface_meshes(site, ms, ref, z_base)))
        return finish(site, S, ref, n, c0, xc, z_base, z_top, rms, extra)
    P = []
    for name in S['meshes']:
        m = trimesh.load(OUT / f'{name}.obj', process=False, force='mesh')
        T = m.vertices[m.faces[:len(m.faces) // 2]]                    # front copies
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1), 1e-12)[:, None]
        n *= np.sign(n @ ref)[:, None]
        c = T.mean(1)
        sel = ((n @ ref > np.cos(np.radians(3))) & (c[:, 0] > STRIP_X[0] - 5) & (c[:, 0] < STRIP_X[1] + 5)
               & (c[:, 2] > S['z'][0]) & (c[:, 2] < S['z'][1]))
        P.append(T[sel].reshape(-1, 3))
    P = np.concatenate(P)
    c0 = P.mean(0)
    n = np.linalg.svd(P - c0)[2][2]
    n *= np.sign(n @ ref)
    rms = float(np.sqrt((((P - c0) @ n) ** 2).mean()))
    if S['base'] == 'water':
        water = trimesh.load(OUT / 'obj_Object_16.obj', process=False, force='mesh')
        hit, _, _ = water.ray.intersects_location(np.array([[xc, c0[1] - 6.0, 20.0]]), np.array([[0, 0, -1.0]]))
        z_base = float(hit[:, 2].max())
    else:
        z_base = float(S['base'])
    z_top = float(P[:, 2].max())
    return finish(site, S, ref, n, c0, xc, z_base, z_top, rms, extra)


def finish(site, S, ref, n, c0, xc, z_base, z_top, rms, extra):
    """Spawn, ledge and platform from the face plane (n through c0); writes the site yaml."""
    STRIP_X = S['strip_x']
    up = np.array([0, 0, 1.0]) - n[2] * n
    up /= np.linalg.norm(up)
    along = np.cross(up, n)                                         # horizontal, along the dam
    # base: the face at the waterline / foot in the middle of the strip
    y_b = c0[1] - (n[0] * (xc - c0[0]) + n[2] * (z_base - c0[2])) / n[1]
    base = np.array([xc, y_b, z_base])
    length = (z_top - z_base) / up[2]
    beta = float(np.degrees(np.arccos(n[2])))
    # robot: x up the face, z = normal  ->  R = [up, n x up, n]
    R = np.c_[up, np.cross(n, up), n]
    spawn_p = base + up * SPAWN_S + n * 0.003
    spawn = dict(x=spawn_p[0], y=spawn_p[1], z=spawn_p[2], roll=float(np.arctan2(R[2, 1], R[2, 2])),
                 pitch=float(-np.arcsin(R[2, 0])), yaw=float(np.arctan2(R[1, 0], R[0, 0])))
    cradle_s = SPAWN_S - 0.1125 - 0.0254 - 0.012      # ledge just below the rear wheels
    geom = dict(
        model='GoldenEye 007 - Dam Level Baked (Vince_Crusty, CC-BY-4.0)', mu=0.7,
        side=S['side'],
        face=dict(angle_deg=beta, normal=n, up=up, along=along, base=base, length_m=length, rms_cm=rms * 100,
                  strip_x=list(STRIP_X)),
        base_z=z_base, crest_z=z_top, spawn=spawn,
        cradle=base + up * cradle_s,
        # deployment platform against the face below the robot: centre 0.6 m out
        platform=S['platform'], platform_centre=base + np.r_[n[:2] / np.linalg.norm(n[:2]), 0] * 0.6,
        **extra)

    def plain(o):
        if isinstance(o, dict):
            return {k: plain(v) for k, v in o.items()}
        if isinstance(o, (list, tuple, np.ndarray)):
            return [plain(v) for v in o]
        return round(float(o), 5) if isinstance(o, (float, np.floating)) else o

    (WORLDS / S['yaml']).write_text(
        '# Generated by tools/build_goldeneye_world.py (world frame, metres)\n' + yaml.safe_dump(plain(geom), sort_keys=False))
    print(f'{site}: face {beta:.1f} deg, {length:.2f} m from the base (z {z_base:.2f}) to the top (z {z_top:.2f}), '
          f'flatness rms {rms * 100:.1f} cm, strip x {STRIP_X}')


def gltf_json(path):
    with open(path, 'rb') as f:
        f.read(12)
        n, _ = struct.unpack('<II', f.read(8))
        return json.loads(f.read(n))


def shapes(parts, rel='../../wcr_description/worlds/meshes/goldeneye_dam'):
    """Unlit look: black base colour + the baked texture as emissive map. Transparent materials
    (water) also get it as base colour map, whose alpha Webots uses for transparency."""
    out = ''
    for name, p in parts.items():
        tex = f'ImageTexture {{ url [ "{rel}/{p["texture"]}" ] filtering 5 }}'
        base = f'baseColorMap {tex}\n        ' if p['transparent'] else ''
        out += f'''    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0 0
        {base}emissiveColorMap {tex}
        emissiveIntensity 1
        roughness 1
        metalness 0
      }}
      geometry Mesh {{ url [ "{rel}/{name}.obj" ] }}
      castShadows FALSE
    }}
'''
    return out


def preview_world(parts):
    """World without the robot, for tools/win_webots_snap.ps1 (snapshot supervisor)."""
    return f'''#VRML_SIM R2025a utf8
# Generated by tools/build_goldeneye_world.py - preview of the GoldenEye dam level (no robot)
WorldInfo {{ basicTimeStep 16 }}
DEF VIEW Viewpoint {{ position 0 -400 100 fieldOfView 1.4 near 0.5 far 3000 }}
Background {{ skyColor [ 0.13 0.13 0.14 ] }}
Robot {{ name "snapshot" controller "snapshot" supervisor TRUE controllerArgs [ "VIEWS" ] }}
Pose {{ children [
{shapes(parts)}] }}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glb', required=True)
    args = ap.parse_args()
    j = gltf_json(args.glb)
    skinned = {m['name'] for m in j['meshes'] if any('JOINTS_0' in p['attributes'] for p in m['primitives'])}
    blend = {m['name'] for m in j.get('materials', []) if m.get('alphaMode') == 'BLEND'}

    OUT.mkdir(parents=True, exist_ok=True)
    for old in [*OUT.glob('obj_*.obj'), *OUT.glob('tex_*')]:
        old.unlink()
    scene = trimesh.load(args.glb)
    textures, parts = {}, {}
    for node in scene.graph.nodes_geometry:
        mat4, name = scene.graph[node]
        if name in skinned:
            continue
        g = scene.geometry[name].copy()
        g.apply_transform(mat4)
        v = g.vertices
        V = np.c_[v[:, 0], -v[:, 2], v[:, 1]]                       # Y-up -> Z-up
        N = g.vertex_normals
        N = np.c_[N[:, 0], -N[:, 2], N[:, 1]]
        mat = g.visual.material
        img = mat.baseColorTexture
        key = mat.name                     # trimesh copies a shared material per mesh
        if key not in textures:
            if mat.name == WATER:
                # 36 % see-through on Sketchfab, but below it there is only the dam face and empty
                # sky (no lake bed), which made it look white: flatten onto deep water instead
                deep = Image.new('RGBA', img.size, (18, 32, 40, 255))
                img = Image.alpha_composite(deep, img.convert('RGBA'))
            alpha = img.mode in ('RGBA', 'LA') and np.asarray(img.convert('RGBA'))[..., 3].min() < 250
            fname = f'tex_{len(textures)}.' + ('png' if alpha else 'jpg')
            (img.convert('RGBA') if alpha else img.convert('RGB')).save(OUT / fname, **({} if alpha else {'quality': 90}))
            textures[key] = fname
        uv = g.visual.uv
        lines = [f'o {name}']
        lines += [f'v {a:.4f} {b:.4f} {c:.4f}' for a, b, c in V]
        lines += [f'vt {a:.5f} {b:.5f}' for a, b in uv]
        lines += [f'vn {a:.4f} {b:.4f} {c:.4f}' for a, b, c in np.concatenate([N, -N])]
        F, n = g.faces + 1, len(V)
        lines += [f'f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}' for a, b, c in F]
        lines += [f'f {c}/{c}/{c + n} {b}/{b}/{b + n} {a}/{a}/{a + n}' for a, b, c in F]   # back side
        (OUT / f'obj_{name}.obj').write_text('\n'.join(lines) + '\n')
        parts[f'obj_{name}'] = dict(texture=textures[key], material=mat.name, faces=len(g.faces),
                                    transparent=mat.name in blend and mat.name != WATER,
                                    bbox=[np.round(V.min(0), 2).tolist(), np.round(V.max(0), 2).tolist()],
                                    uv_range=[np.round(uv.min(0), 3).tolist(), np.round(uv.max(0), 3).tolist()])
    (OUT / 'parts.yaml').write_text('# generated by tools/build_goldeneye_world.py (Z-up, metres)\n'
                                    + yaml.safe_dump(parts, sort_keys=False))
    (OUT / 'ATTRIBUTION.txt').write_text(ATTRIBUTION)
    (ROOT / 'webots' / 'worlds' / 'goldeneye_dam_preview.wbt').write_text(preview_world(parts))
    for site in SITES:
        face_geometry(site)
    print(f'wrote {len(parts)} meshes, {len(textures)} textures to {OUT} (skipped skinned: {sorted(skinned)})')


if __name__ == '__main__':
    main()
