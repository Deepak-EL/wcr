"""Supervisor that saves views of a world and quits (no robot needed).

controllerArgs: <out_prefix> then groups of 6 numbers "ex ey ez tx ty tz" (eye, target);
writes results/<out_prefix>_<i>.png for each view, then results/<out_prefix>.done.
"""
import math
import os
import sys

from controller import Supervisor

RESULTS = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'results')


def look_at(eye, target):
    """Webots Viewpoint orientation (axis-angle) looking from eye to target, z up, view along +x."""
    d = [b - a for a, b in zip(eye, target)]
    yaw, pitch = math.atan2(d[1], d[0]), math.atan2(-d[2], math.hypot(d[0], d[1]))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]     # Rz(yaw) Ry(pitch)
    angle = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if angle < 1e-6:
        return [0, 0, 1, 0]
    s = 2 * math.sin(angle)
    return [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, angle]


def main():
    sv = Supervisor()
    dt = int(sv.getBasicTimeStep())
    prefix, nums = sys.argv[1], [float(a) for a in sys.argv[2:]]
    vp = sv.getFromDef('VIEW') or sv.getRoot().getField('children').getMFNode(1)
    for _ in range(int(1000 / dt)):            # let textures load
        sv.step(dt)
    for i in range(len(nums) // 6):
        eye, target = nums[6 * i:6 * i + 3], nums[6 * i + 3:6 * i + 6]
        vp.getField('position').setSFVec3f(eye)
        vp.getField('orientation').setSFRotation(look_at(eye, target))
        for _ in range(int(500 / dt)):
            sv.step(dt)
        sv.exportImage(os.path.join(RESULTS, f'{prefix}_{i}.png'), 90)
        sv.step(dt)
    open(os.path.join(RESULTS, f'{prefix}.done'), 'w').close()
    sv.simulationQuit(0)


main()
