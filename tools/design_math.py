#!/usr/bin/env python3
"""Mass properties and adhesion requirements for the EDF-only WCR on the dam.

Masses are ESTIMATES (nothing has been weighed yet); replace them with scale
readings and rerun.  The chassis centre of mass and inertia come from the
watertight solids in meshes/chassis_body source (CAD), scaled to the chassis mass.

Adhesion model
--------------
Gazebo cannot simulate airflow, so the EDF is one force F pressing the robot into
the surface (thrust of the fan + suction under the skirt). Driving up a face at
angle beta from horizontal without wheel slip needs

    mu (F + m g cos beta) >= m g sin beta      ->   F >= m g (sin beta / mu - cos beta)

and the upper wheels must stay loaded (no peel-off about the lower axle):

    F * L/2 >= m g sin beta * h                (h = CoM height above the surface)

usage: python tools/design_math.py
"""
import math
from pathlib import Path

import numpy as np

G = 9.81
MESHES = Path(__file__).resolve().parents[1] / 'wcr_description' / 'meshes'

# ---- mass budget [kg] (estimates) -------------------------------------------
MASS = {
    'chassis': 1.45,       # printed frame + skirt + EDF motor/duct + ESC + ESP32 + sensors + wiring
    'steering_pod': 0.08,  # x4: steering actuator, bracket, drive gearmotor
    'wheel': 0.035,        # x4
    'edf_rotor': 0.02,     # rotor only; stator is in 'chassis'
    'tether_hang': 0.08,   # off-board battery lead hanging from the robot
}
WHEELBASE = 0.225          # m, axle to axle
WHEEL_R = 0.0254
CHASSIS_Z = 0.01036        # chassis_link above base_link (contact plane)
EDF_MAX_THRUST_N = 10.0    # 70 mm EDF, 3S, through a 2 m lead (estimate ~1 kgf)

SURFACES = [  # name, angle from horizontal [deg], mu
    ('basin floor', 0.0, 0.7),
    ('downstream slope dry', 53.55, 0.7),
    ('downstream slope wet', 53.55, 0.45),
    ('downstream slope icy', 53.55, 0.25),
    ('upper wall dry', 89.6, 0.7),
    ('upper wall wet', 89.6, 0.45),
]


def total_mass():
    m = MASS
    return m['chassis'] + 4 * (m['steering_pod'] + m['wheel']) + m['edf_rotor'] + m['tether_hang']


def chassis_props():
    """CoM [m, chassis frame] and inertia diag about CoM from CAD solids, scaled to MASS['chassis']."""
    try:
        import trimesh
    except ImportError:
        return None
    src = trimesh.load(MESHES / 'robotChassisNoProp.stl', force='mesh')
    bodies = [b for b in src.split(only_watertight=False)
              if b.is_watertight and b.volume > 0 and abs(b.bounds.mean(0)[1]) <= 150]
    vol = sum(b.volume for b in bodies)
    com = sum(b.volume * b.center_mass for b in bodies) / vol
    inertia = np.zeros((3, 3))
    for b in bodies:  # parallel axis, unit density
        d = b.center_mass - com
        inertia += b.moment_inertia + b.volume * (d @ d * np.eye(3) - np.outer(d, d))
    inertia *= MASS['chassis'] / vol
    return com * 1e-3, np.diag(inertia) * 1e-6


def main():
    m = total_mass()
    print('Mass budget (estimates):')
    for k, v in MASS.items():
        n = 4 if k in ('steering_pod', 'wheel') else 1
        print(f'  {k:13s} {n} x {v * 1000:5.0f} g')
    print(f'  total          {m * 1000:7.0f} g  (weight {m * G:.1f} N)')

    props = chassis_props()
    h = 0.08
    if props:
        com, inertia = props
        h = com[2] + CHASSIS_Z
        print(f'\nChassis CoM (chassis_link frame) [m]: {np.round(com, 4).tolist()}')
        print(f'Chassis inertia about CoM [kg m^2]: ixx={inertia[0]:.5f} iyy={inertia[1]:.5f} izz={inertia[2]:.5f}')
        print(f'CoM height above surface h = {h * 1000:.0f} mm')

    print(f'\nRequired adhesion force F (EDF thrust + skirt suction), robot {m:.2f} kg:')
    print(f'  {"surface":24s} {"beta":>6s} {"mu":>5s} {"F_slip":>8s} {"F_peel":>8s} {"F_req":>8s} {"x1.5":>8s}')
    for name, beta_d, mu in SURFACES:
        b = math.radians(beta_d)
        f_slip = max(0.0, m * G * (math.sin(b) / mu - math.cos(b)))
        f_peel = max(0.0, m * G * math.sin(b) * h / (WHEELBASE / 2))
        f = max(f_slip, f_peel)
        print(f'  {name:24s} {beta_d:6.1f} {mu:5.2f} {f_slip:7.1f}N {f_peel:7.1f}N {f:7.1f}N {1.5 * f:7.1f}N')
    print(f'\nEDF thrust alone (est.): ~{EDF_MAX_THRUST_N:.0f} N. Anything above that has to come from skirt suction:')
    skirt = 0.174 * 0.175
    for f in (20, 30, 45):
        print(f'  F = {f} N needs a skirt pressure drop of {(f - EDF_MAX_THRUST_N) / skirt:5.0f} Pa over {skirt * 1e4:.0f} cm2')


if __name__ == '__main__':
    main()
