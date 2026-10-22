#!/usr/bin/env python3
"""Build a second Gazebo world from the Sketchfab model "Gravity Dam" (gravity_dam.glb).

Model: "Gravity Dam" by USACE Sacramento District, CC-BY-4.0
https://sketchfab.com/3d-models/gravity-dam-d761a8b4ee184839912d41aefe6f096d

Unlike the winter scan (tools/build_dam_world.py) this is a clean CAD-style model in real
metres (29.3 m long, 13.2 m high), so nothing is fitted or flattened: the geometry is only
re-oriented and written as OBJ + textures.

World frame (same convention as dam_world.sdf): x along the dam, the climbing face is the
vertical UPSTREAM face lying in the plane y = 0 with its outward normal toward -y, the dam
body toward +y, ground at z = 0. The robot spawns on that face on a small cradle.

Outputs: wcr_description/worlds/gravity_dam_world.sdf, gravity_dam_geometry.yaml and
         wcr_description/worlds/meshes/gravity_dam/ (OBJ, MTL, textures, ATTRIBUTION.txt)

usage: python tools/build_gravity_dam_world.py --glb assets/gravity_dam.glb [--spawn-up 1.0]
needs: pip install trimesh numpy pillow pyyaml
"""
import argparse
import io
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
WORLDS = ROOT / 'wcr_description' / 'worlds'
OUT = WORLDS / 'meshes' / 'gravity_dam'
CREASE = math.radians(30)


def corner_normals(V, F):
    """Per-corner normals averaged over faces that share the corner position and are within CREASE."""
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    area = np.linalg.norm(fn, axis=1, keepdims=True)
    fn = fn / np.maximum(area, 1e-12)
    key = np.round(V / 1e-4).astype(np.int64)
    _, pos_id = np.unique(key, axis=0, return_inverse=True)
    pos_id = pos_id.reshape(-1)
    by_pos = {}
    for f, tri in enumerate(F):
        for v in tri:
            by_pos.setdefault(pos_id[v], []).append(f)
    cos_c = math.cos(CREASE)
    N = np.zeros((len(F), 3, 3))
    for f, tri in enumerate(F):
        for k, v in enumerate(tri):
            nb = by_pos[pos_id[v]]
            w = [g for g in nb if fn[g] @ fn[f] >= cos_c]
            n = (fn[w] * area[w]).sum(0)
            N[f, k] = n / max(np.linalg.norm(n), 1e-12)
    return N


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--glb', required=True)
    ap.add_argument('--mu', type=float, default=0.7, help='concrete friction (dry 0.7, wet 0.45)')
    ap.add_argument('--spawn-up', type=float, default=1.0, help='spawn height of base_link above the ground [m]')
    args = ap.parse_args()

    scene = trimesh.load(args.glb, process=False)
    # glTF Y-up -> Z-up, then turn 180 deg about z so the upstream face looks toward -y
    T = trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]) @ \
        trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
    parts = []
    for node in scene.graph.nodes_geometry:
        M, name = scene.graph[node]
        g = scene.geometry[name].copy()
        g.apply_transform(T @ M)
        parts.append(g)

    allm = trimesh.util.concatenate(parts)
    c, n = allm.triangles_center, allm.face_normals
    face = (n[:, 1] < -0.95) & (c[:, 1] < allm.bounds[0][1] + 0.5)
    y_face = float(np.average(c[face, 1], weights=allm.area_faces[face]))
    x_lo, x_hi = c[face, 0].min(), c[face, 0].max()
    top = float(c[face, 2].max())
    shift = np.array([-(allm.bounds[0][0] + allm.bounds[1][0]) / 2, -y_face, -allm.bounds[0][2]])
    for g in parts:
        g.apply_translation(shift)
    # The road lies 1-50 mm above the crest and z-fights (shimmers) at distance; lift it 2 cm.
    # (Gates sit 5-50 mm off the body, which the 0.1 m GUI near clip resolves.)
    for g in parts:
        if getattr(g.visual.material, 'name', '') == 'dam-road':
            g.apply_translation([0.0, 0.0, 0.02])
    bounds = trimesh.util.concatenate(parts).bounds
    crest_z = float(bounds[1][2])
    print(f'face: {face.sum()} tris, {allm.area_faces[face].sum():.0f} m2, x {x_lo:.1f}..{x_hi:.1f}, '
          f'top {top:.2f} m; dam {bounds[1] - bounds[0]} m')

    # ---- OBJ + MTL + textures ---------------------------------------------------
    OUT.mkdir(parents=True, exist_ok=True)
    obj, mtl = io.StringIO(), io.StringIO()
    obj.write('# "Gravity Dam" by USACE Sacramento District (CC-BY-4.0); tools/build_gravity_dam_world.py\n'
              'mtllib gravity_dam.mtl\n')
    # vt indices are written equal to v indices, so every part needs UVs
    assert all(getattr(g.visual, 'uv', None) is not None for g in parts), 'part without UVs'
    off, n_off = 1, 1
    seen = {}
    part_materials = []
    for i, g in enumerate(parts):
        mat = g.visual.material
        mname = (getattr(mat, 'name', None) or f'mat{i}').replace(' ', '_')
        if mname not in seen:
            seen[mname] = True
            img = getattr(mat, 'baseColorTexture', None)
            col = np.asarray(getattr(mat, 'baseColorFactor', None) if getattr(mat, 'baseColorFactor', None)
                             is not None else [255, 255, 255, 255], float)[:3]
            col = col / 255.0 if col.max() > 1 else col
            mtl.write(f'newmtl {mname}\nKa {col[0]:.3f} {col[1]:.3f} {col[2]:.3f}\n'
                      f'Kd {col[0]:.3f} {col[1]:.3f} {col[2]:.3f}\nKs 0.05 0.05 0.05\n')
            if img is not None:
                tex = f'{mname}.jpg'
                img.convert('RGB').save(OUT / tex, quality=92)
                mtl.write(f'map_Kd {tex}\n')
            mtl.write('\n')
            # full PBR set for renderers that use it (Webots): glTF metallicRoughness = G rough, B metal
            mr = getattr(mat, 'metallicRoughnessTexture', None)
            if mr is not None:
                _, g_ch, b_ch = mr.convert('RGB').split()
                g_ch.save(OUT / f'{mname}_roughness.png')
                b_ch.save(OUT / f'{mname}_metalness.png')
            nt = getattr(mat, 'normalTexture', None)
            if nt is not None:
                nt.convert('RGB').save(OUT / f'{mname}_normal.png')
        part_materials.append((f'part_{i}', mname))
        V, F = np.asarray(g.vertices), np.asarray(g.faces)
        N = corner_normals(V, F)
        obj.write(f'o part_{i}\nusemtl {mname}\n')
        obj.write(''.join(f'v {a:.4f} {b:.4f} {c_:.4f}\n' for a, b, c_ in V))
        obj.write(''.join(f'vt {u:.5f} {v:.5f}\n' for u, v in g.visual.uv))
        # The model's triangles face mixed directions (Sketchfab renders two-sided; Gazebo culls
        # back faces, so parts of the dam vanish). Write every triangle for both sides, each side
        # with its own normals; only the copy facing the camera is drawn, so no z-fighting.
        Nb = -N[:, ::-1]
        obj.write(''.join(f'vn {a:.4f} {b:.4f} {c_:.4f}\n' for a, b, c_ in np.concatenate([N, Nb]).reshape(-1, 3)))
        for f, tri in enumerate(F):
            ns = n_off + 3 * f + np.arange(3)
            obj.write('f ' + ' '.join(f'{v}/{v}/{k}' for v, k in zip(tri + off, ns)) + '\n')
        nb_off = n_off + 3 * len(F)
        for f, tri in enumerate(F):
            ns = nb_off + 3 * f + np.arange(3)
            obj.write('f ' + ' '.join(f'{v}/{v}/{k}' for v, k in zip(tri[::-1] + off, ns)) + '\n')
        off += len(V)
        n_off += 6 * len(F)
    (OUT / 'gravity_dam.obj').write_text(obj.getvalue())
    (OUT / 'parts.yaml').write_text('# OBJ object -> material (tools/build_webots.py)\n'
                                    + yaml.safe_dump(dict(part_materials), sort_keys=False))
    (OUT / 'gravity_dam.mtl').write_text(mtl.getvalue())
    (OUT / 'ATTRIBUTION.txt').write_text(
        '"Gravity Dam" by USACE Sacramento District\n'
        'https://sketchfab.com/3d-models/gravity-dam-d761a8b4ee184839912d41aefe6f096d\n'
        'License: CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/)\n'
        'Changes: re-oriented (Z-up, upstream face toward -y), recentred, converted to OBJ, normals recomputed.\n')

    # ---- spawn on the vertical face + cradle ------------------------------------
    up = args.spawn_up
    spawn = dict(x=0.0, y=-0.003, z=up, roll=0.0, pitch=-math.pi / 2, yaw=math.pi / 2)
    cradle_top = up - 0.1125 - 0.0254 - 0.012
    geom = dict(model='Gravity Dam (USACE Sacramento District, CC-BY-4.0)', mu=args.mu,
                face=dict(angle_deg=90.0, normal=[0.0, -1.0, 0.0], up=[0.0, 0.0, 1.0], plane_y=0.0,
                          x=[round(float(x_lo + shift[0]), 3), round(float(x_hi + shift[0]), 3)],
                          top_z=round(top + float(shift[2]), 3)),
                crest_z=round(crest_z, 3), spawn=spawn, cradle_top_z=round(cradle_top, 4))
    (WORLDS / 'gravity_dam_geometry.yaml').write_text(
        '# Generated by tools/build_gravity_dam_world.py (world frame, metres)\n' + yaml.safe_dump(geom, sort_keys=False))

    back_geom = dict(geom, side='back (upstream, vertical)')
    (WORLDS / 'gravity_dam_back_geometry.yaml').write_text(
        '# Generated by tools/build_gravity_dam_world.py (world frame, metres)\n'
        + yaml.safe_dump(back_geom, sort_keys=False))

    # ---- front (downstream) face: 79 deg face + 87 deg upper face, plain strip --------
    front, front_boxes, front_cradle = front_face(trimesh.util.concatenate(parts), args)
    front_geom = dict(model=geom['model'], mu=args.mu, side='front (downstream, 79 deg)', **front)
    (WORLDS / 'gravity_dam_geometry.yaml').write_text(
        '# Generated by tools/build_gravity_dam_world.py (world frame, metres)\n'
        '# default spawn: front (downstream) face; gravity_dam_back_geometry.yaml spawns on the back\n'
        + yaml.safe_dump(front_geom, sort_keys=False))

    width = float(x_hi - x_lo)
    face_top = top + float(shift[2])
    body_depth = float(bounds[1][1])
    sdf = TEMPLATE.format(mu=args.mu, width=width, face_h=face_top, face_zc=face_top / 2,
                          crest_y=body_depth / 2, crest_dy=body_depth, crest_z=face_top - 0.3,
                          cr_z=cradle_top - 0.01, hidden=HIDDEN, front=front_boxes, front_cradle=front_cradle)
    (WORLDS / 'gravity_dam_world.sdf').write_text(sdf)
    print('wrote', WORLDS / 'gravity_dam_world.sdf', '| spawn', spawn)


FRONT_X = (6.9, 14.4)        # plain strip of the downstream face (no spillway bay / powerhouse in front)
FRONT_N = np.array([0.0, 0.981, 0.193])   # 79 deg face (rough, refined by the fit)
UPPER_N = np.array([0.0, 0.9987, 0.050])  # 87 deg face under the crest
WHEEL_R, HALF_WB = 0.0254, 0.1125


def plane_pose(n):
    """yaw, roll so a box's local z = n and local y points up the plane (n has no x component)."""
    yaw, roll = math.atan2(n[0], -n[1]), math.atan2(math.hypot(n[0], n[1]), n[2])
    up = np.array([-math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.cos(roll), math.sin(roll)])
    return yaw, roll, up


def box_on_plane(name, n, base, along, width, xc, thick, mu):
    yaw, roll, up = plane_pose(n)
    c = np.asarray(base, float) + up * along / 2 - n * thick / 2
    c[0] = xc
    return (f'\n        <collision name="{name}">\n'
            f'          <pose>{c[0]:.4f} {c[1]:.4f} {c[2]:.4f} {roll:.6f} 0 {yaw:.6f}</pose>\n'
            f'          <geometry><box><size>{width:.3f} {along:.4f} {thick:.3f}</size></box></geometry>\n'
            f'          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>\n'
            f'        </collision>')


def fit_face(m, n_guess, sel_extra):
    """Plane (n, d) from the model triangles parallel to n_guess (either winding) inside the strip."""
    nn = m.face_normals * np.sign(m.face_normals @ n_guess)[:, None]
    c = m.triangles_center
    s = (nn @ n_guess > 0.9995) & (c[:, 0] > FRONT_X[0]) & (c[:, 0] < FRONT_X[1]) & sel_extra(c)
    w = m.area_faces[s]
    n = (nn[s] * w[:, None]).sum(0); n /= np.linalg.norm(n); n[0] = 0.0; n /= np.linalg.norm(n)
    d = float(np.average(c[s] @ n, weights=w))
    return n, d, m.triangles[s].reshape(-1, 3)    # corners (the model's triangles are large)


def front_face(m, args):
    n1, d1, c1 = fit_face(m, FRONT_N, lambda c: (c[:, 2] > 2.0) & (c[:, 2] < 9.1))
    n2, d2, c2 = fit_face(m, UPPER_N, lambda c: c[:, 2] > 9.2)
    z_toe, z_top = float(c1[:, 2].min()) , float(c2[:, 2].max())
    # points in the y-z section: toe of the 79 deg face, knee (plane intersection), top
    y_at = lambda n, d, z: (d - n[2] * z) / n[1]
    toe = np.array([0.0, y_at(n1, d1, z_toe), z_toe])
    A = np.array([[n1[1], n1[2]], [n2[1], n2[2]]])
    yk, zk = np.linalg.solve(A, [d1, d2])
    knee = np.array([0.0, yk, zk])
    top = np.array([0.0, y_at(n2, d2, z_top), z_top])
    _, _, up1 = plane_pose(n1)
    _, _, up2 = plane_pose(n2)
    len1, len2 = float(np.linalg.norm(knee - toe)), float(np.linalg.norm(top - knee))
    width, xc = FRONT_X[1] - FRONT_X[0], 0.5 * (FRONT_X[0] + FRONT_X[1])
    beta = math.degrees(math.acos(n1[2]))
    print(f'front: {beta:.1f} deg face z {z_toe:.2f}-{zk:.2f} m ({len1:.2f} m), '
          f'{math.degrees(math.acos(n2[2])):.1f} deg face to z {z_top:.2f} m ({len2:.2f} m), x {FRONT_X}')
    # base slab in front of the strip: its top surface
    nn = m.face_normals * np.sign(m.face_normals[:, 2])[:, None]
    c = m.triangles_center
    s = (nn[:, 2] > 0.99) & (c[:, 0] > FRONT_X[0]) & (c[:, 0] < FRONT_X[1]) & (c[:, 1] > toe[1] + 0.05)
    z_slab = float(np.median(c[s, 2])) if s.any() else z_toe
    y_far = float(m.bounds[1][1])
    boxes = (box_on_plane('front_face', n1, toe, len1 + 0.05, width, xc, 0.6, args.mu)
             + box_on_plane('front_upper_face', n2, knee, len2, width, xc, 0.6, args.mu)
             + f'\n        <collision name="front_slab">\n'
               f'          <pose>{xc:.3f} {(toe[1] + y_far) / 2:.3f} {z_slab - 0.25:.3f} 0 0 0</pose>\n'
               f'          <geometry><box><size>{width:.3f} {y_far - toe[1]:.3f} 0.5</size></box></geometry>\n'
               f'          <surface><friction><ode><mu>{args.mu}</mu><mu2>{args.mu}</mu2></ode></friction></surface>\n'
               f'        </collision>')
    # spawn: body z = face normal, body x = up the face, 1 m up from the toe on a cradle
    s_up = 1.0
    p = toe + up1 * s_up + n1 * 0.003
    p[0] = xc
    R = np.column_stack([up1, np.cross(n1, up1), n1])
    M = np.eye(4); M[:3, :3] = R
    roll, pitch, yaw = trimesh.transformations.euler_from_matrix(M, 'sxyz')
    spawn = dict(x=float(p[0]), y=float(p[1]), z=float(p[2]), roll=float(roll), pitch=float(pitch), yaw=float(yaw))
    pc = toe + up1 * (s_up - HALF_WB - WHEEL_R - 0.012)
    cy, cr, _ = plane_pose(n1)
    cradle = (f'<model name="cradle_front"><static>true</static>'
              f'<pose>{xc:.4f} {pc[1]:.4f} {pc[2]:.4f} {cr:.6f} 0 {cy:.6f}</pose><link name="ledge">'
              f'<collision name="c"><pose>0 -0.01 0.03 0 0 0</pose><geometry><box><size>0.45 0.02 0.06</size></box></geometry></collision>'
              f'<visual name="v"><pose>0 -0.01 0.03 0 0 0</pose><geometry><box><size>0.45 0.02 0.06</size></box></geometry>'
              f'<material><ambient>0.9 0.6 0.05 1</ambient><diffuse>0.9 0.6 0.05 1</diffuse></material></visual>'
              f'</link></model>')
    r = lambda v: [round(float(a), 5) for a in v]
    geom = dict(face=dict(angle_deg=round(beta, 2), normal=r(n1), up=r(up1), toe=r(toe), length_m=round(len1, 3),
                          x=list(FRONT_X)),
                upper_face=dict(angle_deg=round(math.degrees(math.acos(n2[2])), 2), normal=r(n2), up=r(up2),
                                base=r(knee), length_m=round(len2, 3)),
                slab_z=round(z_slab, 3), spawn={k: round(v, 5) for k, v in spawn.items()})
    print('front spawn', geom['spawn'], '| slab z', round(z_slab, 2))
    return geom, boxes, cradle


HIDDEN = ('<ignition-gui><property key="state" type="string">floating</property>'
          '<property key="width" type="double">5</property><property key="height" type="double">5</property>'
          '<property key="showTitleBar" type="bool">false</property></ignition-gui>')

TEMPLATE = '''<?xml version="1.0" ?>
<!-- Generated by tools/build_gravity_dam_world.py - do not edit by hand.
     Dam model: "Gravity Dam" by USACE Sacramento District, CC-BY-4.0. -->
<sdf version="1.8">
  <world name="gravity_dam_world">
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
      <ambient>0.42 0.43 0.46 1</ambient>
      <background>0.62 0.74 0.88 1</background>
      <!-- off: ogre (v1) shadow maps give striped "shadow acne" that shimmers when the view moves -->
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <!-- sun in front of the downstream (front) face, high and from the left, so the piers,
         gates and powerhouse are lit from the side and keep their shape -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>15 40 40 0 0 0</pose>
      <diffuse>1.0 0.97 0.92 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>-0.35 -0.6 -0.72</direction>
    </light>
    <!-- weak fill from the upstream side so the back face is not black -->
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 -40 30 0 0 0</pose>
      <diffuse>0.35 0.37 0.42 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.2 0.7 -0.5</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <pose>0 0 -0.5 0 0 0</pose>
          <geometry><box><size>200 200 1</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <!-- visual top 5 cm below the dam base: coincident surfaces z-fight (shimmer) -->
        <visual name="v">
          <pose>0 0 -0.55 0 0 0</pose>
          <geometry><box><size>200 200 1</size></box></geometry>
          <material><ambient>0.42 0.40 0.36 1</ambient><diffuse>0.48 0.46 0.42 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="gravity_dam">
      <static>true</static>
      <link name="body">
        <visual name="model">
          <geometry><mesh><uri>package://wcr_description/worlds/meshes/gravity_dam/gravity_dam.obj</uri></mesh></geometry>
        </visual>
        <!-- climbing face: the vertical upstream face, plane y = 0 -->
        <collision name="upstream_face">
          <pose>0 0.5 {face_zc:.4f} 0 0 0</pose>
          <geometry><box><size>{width:.3f} 1.0 {face_h:.4f}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <!-- crest: stops the robot at the top edge -->
        <collision name="crest">
          <pose>0 {crest_y:.3f} {crest_z:.3f} 0 0 0</pose>
          <geometry><box><size>{width:.3f} {crest_dy:.3f} 0.6</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <!-- front (downstream) climbing strip beside the spillway: 79 deg face, 87 deg upper face, slab -->{front}
      </link>
    </model>

    <!-- Deployment cradles: ledges the robot rests on until adhesion is on (front = default spawn). -->
    {front_cradle}
    <model name="cradle">
      <static>true</static>
      <pose>0 -0.03 {cr_z:.4f} 0 0 0</pose>
      <link name="ledge">
        <collision name="c"><geometry><box><size>0.45 0.06 0.02</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.45 0.06 0.02</size></box></geometry>
          <material><ambient>0.9 0.6 0.05 1</ambient><diffuse>0.9 0.6 0.05 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Fixed overview camera on the spawn area (1 Hz; for screenshots and docs) -->
    <model name="overview_camera">
      <static>true</static>
      <pose>13.2 10.2 4.6 0 0.22 -2.0</pose>
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
      <!-- MinimalScene (not the deprecated GzScene3D, which ignores camera_clip): its ~1 cm default
           near clip leaves too little depth precision at 15-30 m and stacked surfaces shimmer.
           (A segfault printed when the GUI closes is a known Fortress shutdown issue.) -->
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre</engine>
        <scene>scene</scene>
        <ambient_light>0.6 0.62 0.66</ambient_light>
        <background_color>0.62 0.74 0.88</background_color>
        <camera_pose>17 18 7 0 0.22 -2.05</camera_pose>
        <camera_clip><near>0.1</near><far>300</far></camera_clip>
      </plugin>
      <!-- helper plugins MinimalScene needs (scene updates, mouse control, selection, follow) -->
      <plugin filename="GzSceneManager" name="Scene Manager">{hidden}</plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">{hidden}</plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">{hidden}</plugin>
      <plugin filename="MarkerManager" name="Marker manager">{hidden}</plugin>
      <plugin filename="SelectEntities" name="Select Entities">{hidden}</plugin>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu">{hidden}</plugin>
      <plugin filename="VisualizationCapabilities" name="Visualization Capabilities">{hidden}</plugin>
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
