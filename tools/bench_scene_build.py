#!/usr/bin/env python3
"""Pack the robot (chassis + 4 pods + 4 wheels) and the gravity dam into one float32 array
(positions + normals per vertex, metres) for tools/bench_gl.py.
usage: python tools/bench_scene_build.py   -> assets/bench_scene.npy"""
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import euler_matrix, translation_matrix as T

ROOT = Path(__file__).resolve().parents[1]
M = ROOT / 'wcr_description' / 'meshes'
W = ROOT / 'wcr_description' / 'worlds' / 'meshes' / 'gravity_dam' / 'gravity_dam.obj'


def flat(mesh):
    tris = mesh.triangles.astype(np.float32)
    n = np.repeat(mesh.face_normals.astype(np.float32)[:, None, :], 3, axis=1)
    return np.concatenate([tris, n], axis=2).reshape(-1, 6)


parts = []
body = trimesh.load(M / 'chassis_body.obj', force='mesh'); body.apply_scale(0.001); parts.append(body)
pod = trimesh.load(M / 'active_wheel.obj', force='mesh'); pod.apply_scale(0.001)
wheel = trimesh.load(M / 'wheel.obj', force='mesh'); wheel.apply_scale(0.001)
for sx in (1, -1):
    for sy in (1, -1):
        p = pod.copy(); p.apply_transform(T([sx * 0.1125, sy * 0.1125, 0.05]) @ euler_matrix(0, 0, sy * np.pi / 2)); parts.append(p)
        w = wheel.copy(); w.apply_transform(T([sx * 0.1125, sy * 0.1125, 0.015])); parts.append(w)
robot = trimesh.util.concatenate(parts)
robot.apply_translation([0, -1.0, 1.0])
dam = trimesh.load(W, force='mesh', process=False)
data = np.concatenate([flat(robot), flat(dam)])
out = ROOT / 'assets' / 'bench_scene.npy'
np.save(out, data)
print(f'robot {len(robot.faces)} tris + dam {len(dam.faces)} tris -> {out} ({out.stat().st_size / 1e6:.0f} MB)')
