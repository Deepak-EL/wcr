#!/usr/bin/env python3
"""Generate the Webots version of the WCR + winter dam from the Gazebo assets.

Writes
  webots/protos/Wcr.proto           robot (mirrors wcr_base.xacro + robot_params.yaml)
  webots/worlds/winter_dam.wbt      dam scan visual + collision boxes from dam_world.sdf + cradle + robot

Webots is z-up/ENU like ROS, so poses carry over; rotations are converted to axis-angle.
usage: python tools/build_webots.py            (everything above + wcr_webots/resource/wcr.urdf)
       python tools/build_webots.py --simple   (only webots/protos/WcrSimple.proto + resource/wcr_simple.urdf:
                                                the same robot drawn with the simplified CAD meshes,
                                                run tools/build_simple_cad.py first)
needs: pip install numpy pyyaml
"""
import math
import re
from pathlib import Path

import numpy as np
import yaml

import inspection                     # Phase 3 defects + obstacles (tools/inspection.py)

ROOT = Path(__file__).resolve().parents[1]
WB = ROOT / 'webots'
WORLDS = ROOT / 'wcr_description' / 'worlds'
P = yaml.safe_load((ROOT / 'wcr_launcher' / 'config' / 'robot_params.yaml').read_text())
# Phase 2 sensors, in the robot base frame (wheel contact plane = the wall; x forward/up the
# face, z out of the wall). The chassis reaches z 0.235 (top rail) and x 0.168 (front edge).
SENSORS = dict(
    # 2D lidar (RPLidar A1-like): scan plane parallel to the wall, just above the robot
    lidar=dict(xyz=(0.0, -0.04, 0.25), samples=360, min_range=0.15, max_range=12.0, hz=10, noise=0.001),
    # wall camera (OV2640-like, VGA stream, 66 deg): in front of the chassis, looking at the wall;
    # off-centre so it stays out of the front camera's view (front camera at y 0.009, z 0.077)
    wall_camera=dict(xyz=(0.18, -0.07, 0.09), pitch=math.radians(70), width=640, height=480, fov=1.15, hz=15),
    # inspection light next to the wall camera; quadratic fall-off, intensity chosen so it lights
    # the wall ~0.1 m away like the previous constant 0.8
    wall_light=dict(xyz=(0.18, -0.07, 0.10), intensity=0.008, beam=0.6, cutoff=0.8),
)
# Visual mounts tying the sensors to the chassis (no mass, no collision), base frame:
# (name, kind, centre xyz, size: box x y z | cylinder radius height)
SENSOR_MOUNTS = [
    ('lidar_mast', 'cylinder', (0.0, -0.04, 0.1625), (0.006, 0.127)),     # chassis top (z 0.099) -> puck
    ('lidar_plate', 'cylinder', (0.0, -0.04, 0.224), (0.038, 0.004)),
    ('cam_arm', 'box', (0.1745, -0.07, 0.1135), (0.031, 0.022, 0.005)),    # from the front edge (x 0.16)
    ('cam_tab', 'box', (0.188, -0.07, 0.102), (0.004, 0.022, 0.026)),      # down to the camera housing
]
MOUNT_RGB = (0.62, 0.63, 0.66)                                            # anodised aluminium
# robot meshes split below Webots' shadow limit (tools/webots_meshes.py)
LOD = '../../wcr_description/meshes/webots'
LOD_DIR = ROOT / 'wcr_description' / 'meshes' / 'webots'
# simplified robot (tools/build_simple_cad.py): same frames as the original meshes, the sensor
# mounts included as one mesh in the base frame. Used by WcrSimple.proto / wcr_simple.urdf only.
SIMPLE = '../../wcr_description/meshes/simple'
SIMPLE_DIR = ROOT / 'wcr_description' / 'meshes' / 'simple'


def rpy_to_axis_angle(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    return matrix_to_axis_angle(R)


def matrix_to_axis_angle(R):
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if angle < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    if abs(angle - math.pi) < 1e-6:          # 180 deg: axis from the diagonal
        ax = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        i = int(np.argmax(ax))
        ax[(i + 1) % 3] *= np.sign(R[i, (i + 1) % 3]) or 1
        ax[(i + 2) % 3] *= np.sign(R[i, (i + 2) % 3]) or 1
        ax /= np.linalg.norm(ax)
        return (*ax, angle)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(angle))
    return (*ax, angle)


def rot(r, p, y):
    return '{:.6f} {:.6f} {:.6f} {:.6f}'.format(*rpy_to_axis_angle(r, p, y))


# ---------------------------------------------------------------- robot PROTO
def split_obj(path):
    """Write each `o` object of an OBJ to <dir>/<object>.obj (indices renumbered, mtllib kept).
    Webots ignores Mesh.name for OBJ objects: a Shape given a multi-object file draws all of them."""
    lines = path.read_text().splitlines()
    mtllib = [l for l in lines if l.startswith('mtllib')]
    data = {'v': [], 'vt': [], 'vn': []}
    objs, cur = {}, None
    for l in lines:
        key = l.split(' ', 1)[0]
        if key in data:
            data[key].append(l)
        elif key == 'o':
            cur = objs.setdefault(l[2:].strip(), [])
        elif key in ('f', 'usemtl') and cur is not None:
            cur.append(l)
    for name, body in objs.items():
        used = {k: {} for k in data}            # old index -> new index, per kind
        faces = []
        for l in body:
            if l.startswith('usemtl'):
                faces.append(l)
                continue
            corners = []
            for c in l.split()[1:]:
                idx = c.split('/')
                for kind, s in zip(('v', 'vt', 'vn'), idx):
                    if s:
                        used[kind].setdefault(int(s), len(used[kind]) + 1)
                corners.append('/'.join(str(used[k][int(s)]) if s else '' for k, s in zip(('v', 'vt', 'vn'), idx)))
            faces.append('f ' + ' '.join(corners))
        out = mtllib + [f'o {name}']
        for kind in ('v', 'vt', 'vn'):
            out += [data[kind][i - 1] for i in used[kind]]
        (path.parent / f'{name}.obj').write_text('\n'.join(out + faces) + '\n')


def split_shapes(stem, app_def, indent, define=True):
    """One Shape per chunk file LOD_DIR/<stem>_<k>.obj; the first defines the appearance (DEF ...)
    when `define`, the others USE it."""
    use = 'USE ' + app_def.split()[1]
    n = len(list(LOD_DIR.glob(f'{stem}_*.obj')))
    return ''.join(f'{indent}Shape {{ appearance {app_def if define and k == 0 else use} '
                   f'geometry Mesh {{ url "{LOD}/{stem}_{k}.obj" }} }}\n' for k in range(n))


def wheel_unit(prefix, sx, sy, pod_yaw, wheel_rpy, first=False, simple=False):
    pod_app = 'DEF POD_APP PBRAppearance { baseColor 0.72 0.22 0.18 roughness 0.5 metalness 0.1 }'
    if simple:
        pods = (f'{" " * 12}Shape {{ appearance {pod_app if first else "USE POD_APP"} '
                f'geometry Mesh {{ url "{SIMPLE}/pod.obj" }} }}\n')
    else:
        pods = split_shapes('active_wheel', pod_app, ' ' * 12, define=first)
    wheel_mesh = f'{SIMPLE}/wheel.obj' if simple else '../../wcr_description/meshes/wheel.obj'
    tire_app = ('DEF TIRE_APP PBRAppearance { baseColor 0.08 0.08 0.09 roughness 0.9 metalness 0 }'
                if first else 'USE TIRE_APP')
    L, Wd = P['length'] / 2, P['width'] / 2
    sh, wh = P['steering_height'], P['wheel_height']
    ia, ir = P['wheel_i_axial'], P['wheel_i_radial']
    si = P['steering_i']
    return f'''
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 0 1
        anchor {sx * L} {sy * Wd} {0.01036 + sh}
        minStop -1.5708
        maxStop 1.5708
      }}
      device [
        RotationalMotor {{ name "{prefix}_steering" maxTorque {P['steering_effort']} maxVelocity {P['steering_velocity']} minPosition -1.5708 maxPosition 1.5708 }}
        PositionSensor {{ name "{prefix}_steering_sensor" }}
      ]
      endPoint Solid {{
        translation {sx * L} {sy * Wd} {0.01036 + sh}
        name "{prefix}_steering_link"
        children [
          Transform {{ rotation {rot(0, 0, pod_yaw)} scale 0.001 0.001 0.001 children [
{pods}            ] }}
          HingeJoint {{
            jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0 {wh} }}
            device [
              RotationalMotor {{ name "{prefix}_wheel" maxTorque {P['wheel_effort']} maxVelocity {P['wheel_velocity'] * 2.5:.4f} }}
              PositionSensor {{ name "{prefix}_wheel_sensor" }}
            ]
            endPoint Solid {{
              translation 0 0 {wh}
              name "{prefix}_wheel_link"
              contactMaterial "wheel"
              children [
                Transform {{ rotation {rot(*wheel_rpy)} scale 0.001 0.001 0.001 children [
                  Shape {{ appearance {tire_app} geometry Mesh {{ url "{wheel_mesh}" }} }} ] }}
              ]
              boundingObject Pose {{ rotation 1 0 0 1.5708 children [ Cylinder {{ height {P['wheel_width']} radius {P['wheel_radius']} subdivision 32 }} ] }}
              physics Physics {{ density -1 mass {P['wheel_mass']} centerOfMass [0 0 0] inertiaMatrix [{ir} {ia} {ir}, 0 0 0] }}
            }}
          }}
        ]
        physics Physics {{ density -1 mass {P['steering_mass']} centerOfMass [0 0 0] inertiaMatrix [{si} {si} {si}, 0 0 0] }}
      }}
    }}'''


def sensors(simple=False):
    """Lidar, wall camera and inspection light (SENSORS), in the robot base frame."""
    L, C, S = SENSORS['lidar'], SENSORS['wall_camera'], SENSORS['wall_light']
    d = (math.cos(C['pitch']), 0.0, -math.sin(C['pitch']))           # wall camera view direction
    mounts = ''
    if simple:                                         # mounts + bridge plate as one mesh (base frame)
        mounts = (f'      Transform {{ scale 0.001 0.001 0.001 children [ Shape {{ appearance PBRAppearance {{ '
                  f'baseColor {" ".join(map(str, MOUNT_RGB))} roughness 0.35 metalness 0.8 }} '
                  f'geometry Mesh {{ url "{SIMPLE}/mounts.obj" }} }} ] }}  # sensor mounts\n')
    for k, (name, kind, xyz, size) in enumerate([] if simple else SENSOR_MOUNTS):
        app = (f'DEF MOUNT_APP PBRAppearance {{ baseColor {" ".join(map(str, MOUNT_RGB))} roughness 0.35 metalness 0.8 }}'
               if k == 0 else 'USE MOUNT_APP')
        geom = (f'Cylinder {{ radius {size[0]} height {size[1]} subdivision 32 }}' if kind == 'cylinder'
                else f'Box {{ size {size[0]} {size[1]} {size[2]} }}')
        mounts += f'      Pose {{ translation {xyz[0]} {xyz[1]} {xyz[2]} children [ Shape {{ appearance {app} geometry {geom} }} ] }}  # {name}\n'
    return mounts + f'''      Lidar {{
        name "lidar"
        translation {L['xyz'][0]} {L['xyz'][1]} {L['xyz'][2]}
        children [ Pose {{ translation 0 0 -0.012 children [ Shape {{
          appearance PBRAppearance {{ baseColor 0.08 0.08 0.09 roughness 0.4 metalness 0.2 }}
          geometry Cylinder {{ height 0.024 radius 0.035 }} }} ] }} ]
        horizontalResolution {L['samples']}
        fieldOfView 6.2832
        numberOfLayers 1
        verticalFieldOfView 0.05
        near 0.05
        minRange {L['min_range']}
        maxRange {L['max_range']}
        type "fixed"
        noise {L['noise']}
        defaultFrequency {L['hz']}
      }}
      Camera {{
        name "wall_camera"
        translation {C['xyz'][0]} {C['xyz'][1]} {C['xyz'][2]}
        rotation 0 1 0 {C['pitch']:.4f}
        children [ Shape {{
          appearance PBRAppearance {{ baseColor 0.1 0.1 0.12 roughness 0.5 metalness 0 }}
          geometry Box {{ size 0.012 0.024 0.024 }} }} ]
        fieldOfView {C['fov']}
        width {C['width']}
        height {C['height']}
        near 0.01
        noise 0.01
      }}
      SpotLight {{
        location {S['xyz'][0]} {S['xyz'][1]} {S['xyz'][2]}
        direction {d[0]:.4f} {d[1]:.4f} {d[2]:.4f}
        intensity {S['intensity']}
        attenuation 0 0 1
        beamWidth {S['beam']}
        cutOffAngle {S['cutoff']}
        castShadows FALSE
      }}'''


def ros_urdf(simple=False):
    """URDF of the same robot for robot_state_publisher / RViz (wcr_webots/resource/wcr.urdf):
    meshes from wcr_description, steering + wheel joints (names match /joint_states of the
    wcr_driver plugin), sensor frames matching SENSORS and the Webots device poses.
    simple=True: wcr_simple.urdf, the same links and joints with the simplified meshes."""
    cz = 0.01036
    L, Wd = P['length'] / 2, P['width'] / 2
    sh, wh = P['steering_height'], P['wheel_height']
    pkg = 'package://wcr_description/meshes'
    mesh = lambda f: f'<mesh filename="{pkg}/{f}" scale="0.001 0.001 0.001"/>'
    # mesh files per link: (original, simplified)
    files = dict(chassis=['chassis_body.obj'], edf='edfProp.stl', pod='active_wheel.obj', wheel='wheel.obj')
    if simple:
        files = dict(chassis=[f'simple/{p.name}' for p in sorted(SIMPLE_DIR.glob('chassis_*.obj'))],
                     edf='simple/edf.obj', pod='simple/pod.obj', wheel='simple/wheel.obj')
    mats = {'chassis': '0.2 0.21 0.24 1', 'edf': '0.05 0.55 0.88 1', 'pod': '0.72 0.22 0.18 1',
            'tire': '0.08 0.08 0.09 1', 'sensor': '0.08 0.08 0.09 1'}

    def link(name, visuals=''):
        return f'  <link name="{name}">{visuals}</link>\n'

    def visual(geom, mat, rpy='0 0 0', xyz='0 0 0'):
        return (f'\n    <visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geom}</geometry>'
                f'<material name="{mat}"/></visual>\n  ')

    def joint(name, kind, parent, child, xyz, rpy='0 0 0', extra=''):
        return (f'  <joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
                f'<origin xyz="{" ".join(f"{v:.5f}" for v in xyz)}" rpy="{rpy}"/>{extra}</joint>\n')

    out = ['<?xml version="1.0"?>',
           '<!-- Generated by tools/build_webots.py: the Webots WCR (webots/protos/Wcr.proto) for RViz -->',
           '<robot name="wcr">']
    out += [f'  <material name="{k}"><color rgba="{v}"/></material>' for k, v in mats.items()]
    s = ''
    s += link('base_link')
    s += link('chassis_link', ''.join(visual(mesh(f), 'chassis') for f in files['chassis']))
    s += joint('chassis_joint', 'fixed', 'base_link', 'chassis_link', (0, 0, cz))
    s += link('edf_link', visual(mesh(files['edf']), 'edf'))         # EDF rotor (edf_joint in /joint_states)
    s += joint('edf_joint', 'continuous', 'chassis_link', 'edf_link', (0, 0, 0), extra='<axis xyz="0 0 1"/>')
    s += link('imu_link') + joint('imu_joint', 'fixed', 'chassis_link', 'imu_link', (P['imu_x'], P['imu_y'], P['imu_z']))
    optical = '-1.5708 0 -1.5708'                      # x forward -> z forward (ROS optical frame)
    s += link('front_camera_link') + link('front_camera_optical')
    s += joint('front_camera_joint', 'fixed', 'chassis_link', 'front_camera_link',
               (P['camera_x'], P['camera_y'], P['camera_z']))
    s += joint('front_camera_optical_joint', 'fixed', 'front_camera_link', 'front_camera_optical', (0, 0, 0), optical)
    Ls, C = SENSORS['lidar'], SENSORS['wall_camera']
    s += link('lidar_link', visual('<cylinder radius="0.035" length="0.024"/>', 'sensor', xyz='0 0 -0.012'))
    s += joint('lidar_joint', 'fixed', 'base_link', 'lidar_link', Ls['xyz'])
    s += link('wall_camera_link', visual('<box size="0.012 0.024 0.024"/>', 'sensor')) + link('wall_camera_optical')
    s += joint('wall_camera_joint', 'fixed', 'base_link', 'wall_camera_link', C['xyz'], f'0 {C["pitch"]:.5f} 0')
    s += joint('wall_camera_optical_joint', 'fixed', 'wall_camera_link', 'wall_camera_optical', (0, 0, 0), optical)
    out.insert(3, f'  <material name="mount"><color rgba="{" ".join(map(str, MOUNT_RGB))} 1"/></material>')
    mvis = visual(mesh('simple/mounts.obj'), 'mount') if simple else ''
    for name, kind, xyz, size in [] if simple else SENSOR_MOUNTS:
        g = (f'<cylinder radius="{size[0]}" length="{size[1]}"/>' if kind == 'cylinder'
             else f'<box size="{size[0]} {size[1]} {size[2]}"/>')
        mvis += visual(g, 'mount', xyz=' '.join(map(str, xyz)))
    s += link('sensor_mounts', mvis) + joint('sensor_mounts_joint', 'fixed', 'base_link', 'sensor_mounts', (0, 0, 0))
    for n, sx, sy, pod_yaw, wrpy in (('FL', 1, 1, math.pi / 2, (0, math.pi / 2, 0)), ('BL', -1, 1, math.pi / 2, (0, math.pi / 2, 0)),
                                     ('BR', -1, -1, -math.pi / 2, (0, 0, math.pi)), ('FR', 1, -1, -math.pi / 2, (0, 0, math.pi))):
        s += link(f'{n}_steering_link', visual(mesh(files['pod']), 'pod', f'0 0 {pod_yaw:.5f}'))
        s += joint(f'{n}_steering_joint', 'revolute', 'base_link', f'{n}_steering_link', (sx * L, sy * Wd, cz + sh),
                   extra='<axis xyz="0 0 1"/><limit lower="-1.5708" upper="1.5708" effort="1.9" velocity="5.24"/>')
        s += link(f'{n}_wheel_link', visual(mesh(files['wheel']), 'tire', ' '.join(f'{v:.5f}' for v in wrpy)))
        s += joint(f'{n}_wheel_joint', 'continuous', f'{n}_steering_link', f'{n}_wheel_link', (0, 0, wh),
                   extra='<axis xyz="0 1 0"/>')
    out.append(s.rstrip('\n'))
    out.append('</robot>')
    return '\n'.join(out) + '\n'


def proto(simple=False):
    """Wcr.proto, or with simple=True WcrSimple.proto: the same robot (physics, joints, devices)
    drawn with the simplified meshes of tools/build_simple_cad.py."""
    cz = 0.01036
    units = (wheel_unit('FL', 1, 1, math.pi / 2, (0, math.pi / 2, 0), first=True, simple=simple)
             + wheel_unit('BL', -1, 1, math.pi / 2, (0, math.pi / 2, 0), simple=simple)
             + wheel_unit('BR', -1, -1, -math.pi / 2, (0, 0, math.pi), simple=simple)
             + wheel_unit('FR', 1, -1, -math.pi / 2, (0, 0, math.pi), simple=simple))
    chassis_app = 'DEF CHASSIS_APP PBRAppearance { baseColor 0.2 0.21 0.24 roughness 0.6 metalness 0.2 }'
    if simple:
        n = len(list(SIMPLE_DIR.glob('chassis_*.obj')))
        chassis = ''.join(f'{" " * 12}Shape {{ appearance {chassis_app if k == 0 else "USE CHASSIS_APP"} '
                          f'geometry Mesh {{ url "{SIMPLE}/chassis_{k}.obj" }} }}\n' for k in range(n))
    else:
        chassis = split_shapes('chassis_body', chassis_app, ' ' * 12)
    edf_mesh = f'{SIMPLE}/edf.obj' if simple else '../../wcr_description/meshes/edfProp.stl'
    name = 'WcrSimple' if simple else 'Wcr'
    header = ('# Simplified visuals (tools/build_simple_cad.py); physics, joints and devices as Wcr.proto.\n'
              if simple else '')
    return f'''#VRML_SIM R2025a utf8
# Generated by tools/build_webots.py - WCR wall-climbing robot (EDF-only), mirrors wcr_base.xacro.
{header}# base frame = wheel contact plane; chassis_link is {cz} m above it.
# Adhesion (EDF thrust + skirt suction) is applied by the controller as a force along -z (supervisor).
PROTO {name} [
  field SFVec3f    translation    0 0 0
  field SFRotation rotation       0 0 1 0
  field SFString   name           "wcr"
  field SFString   controller     "wcr_climb"
  field MFString   controllerArgs []
  field SFBool     supervisor     TRUE
]
{{
  Robot {{
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    controllerArgs IS controllerArgs
    supervisor IS supervisor
    children [
      Pose {{
        translation 0 0 {cz}
        children [
          Transform {{ scale 0.001 0.001 0.001 children [
{chassis}            ] }}
          # EDF rotor: spins about its axis (z through the chassis centre). No Physics node, so the
          # motor turns it kinematically (visual only; the thrust is the controller's adhesion force)
          HingeJoint {{
            jointParameters HingeJointParameters {{ axis 0 0 1 }}
            device [
              RotationalMotor {{ name "edf_motor" maxVelocity 200 maxTorque 10 }}
              PositionSensor {{ name "edf_sensor" }}
            ]
            endPoint Solid {{
              name "edf_rotor"
              children [ Transform {{ scale 0.001 0.001 0.001 children [ Shape {{
                appearance PBRAppearance {{ baseColor 0.05 0.55 0.88 roughness 0.4 metalness 0.3 }}
                geometry Mesh {{ url "{edf_mesh}" }}
              }} ] }} ]
            }}
          }}
          InertialUnit {{ name "imu" translation {P['imu_x']} {P['imu_y']} {P['imu_z']} }}
          Gyro {{ name "gyro" translation {P['imu_x']} {P['imu_y']} {P['imu_z']} }}
          Accelerometer {{ name "accelerometer" translation {P['imu_x']} {P['imu_y']} {P['imu_z']} }}
          Camera {{
            name "front_camera"
            translation {P['camera_x']} {P['camera_y']} {P['camera_z']}
            fieldOfView {P['camera_hfov']}
            width 640
            height 480
            near 0.02
          }}
        ]
      }}{units}
{sensors(simple)}
    ]
    boundingObject Group {{ children [
      Pose {{ translation 0 0 {cz + 0.040} children [ Box {{ size 0.174 0.175 0.080 }} ] }}
      Pose {{ translation 0.0085 0 {cz + 0.095} children [ Box {{ size 0.317 0.265 0.070 }} ] }}
    ] }}
    physics Physics {{
      density -1
      mass {P['chassis_mass'] + P['edf_mass']}
      centerOfMass [ {P['chassis_com_x']} {P['chassis_com_y']} {cz + P['chassis_com_z']} ]
      inertiaMatrix [ {P['chassis_ixx']} {P['chassis_iyy']} {P['chassis_izz']}, 0 0 0 ]
    }}
  }}
}}
'''


# ---------------------------------------------------------------- winter dam world
def sdf_collisions(sdf_text, model):
    """(name, pose[6], size[3]) for every box collision of a model in an SDF world."""
    block = re.search(rf'<model name="{model}">(.*?)</model>', sdf_text, re.S).group(1)
    model_pose = re.search(r'<pose>([^<]+)</pose>', block[:block.find('<link')])   # model pose precedes the link
    mp = np.array(model_pose.group(1).split(), float) if model_pose else np.zeros(6)
    out = []
    for m in re.finditer(r'<collision name="(\w+)">\s*(?:<pose>([^<]+)</pose>)?\s*<geometry><box><size>([^<]+)</size>', block):
        pose = np.array(m.group(2).split(), float) if m.group(2) else np.zeros(6)
        out.append((m.group(1), pose, np.array(m.group(3).split(), float), mp))
    return out


def compose(mp, cp):
    """World pose of a child pose cp given in a model pose mp (both x y z r p y)."""
    def R(r, p, y):
        cr, sr, cp_, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return np.array([[cy * cp_, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp_, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp_ * sr, cp_ * cr]])
    Rm, Rc = R(*mp[3:]), R(*cp[3:])
    return mp[:3] + Rm @ cp[:3], Rm @ Rc


def box_pose(name, pos, Rm, size, appearance=None):
    ax = matrix_to_axis_angle(Rm)
    box = f'Box {{ size {size[0]:.4f} {size[1]:.4f} {size[2]:.4f} }}'
    if appearance:
        box = f'Shape {{ appearance {appearance} geometry {box} }}'
    return (f'      Pose {{ translation {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} '
            f'rotation {ax[0]:.6f} {ax[1]:.6f} {ax[2]:.6f} {ax[3]:.6f} '
            f'children [ {box} ] }}  # {name}\n')


# Winter scan: multiplier on its photo colours that cancels the bluish sky light, so the rendered
# slope matches the texture (measured: render R/B 0.86 vs photo 1.08 without it); exposure in
# winter_dam() restores the brightness.
SCAN_TINT = '1 0.91 0.76'
CRADLE_APP ='PBRAppearance { baseColor 0.9 0.6 0.05 roughness 0.5 metalness 0 }'


def boxes(sdf, model, appearance=None):
    return ''.join(box_pose(n, *compose(mp, pose), size, appearance) for n, pose, size, mp in sdf_collisions(sdf, model))


def solid(name, children, bounding, material='concrete'):
    return f'''Solid {{
  name "{name}"
  contactMaterial "{material}"
  children [
{children}  ]
  boundingObject Group {{ children [
{bounding}  ] }}
}}
'''


def viewpoint(geom, back=3.5, down=1.2, side=1.5):
    """Start the 3D view in front of the robot, looking at it (Webots view looks along +x, z up).
    'follow' then keeps this offset as the robot climbs."""
    s = geom['spawn']
    face = geom.get('face') or geom['slope']
    n, up = np.array(face['normal'], float), np.array(face['up'], float)
    p0 = np.array([s['x'], s['y'], s['z']])
    eye = p0 + n * back - up * down + np.cross(up, n) * side
    d = p0 - eye
    yaw, pitch = math.atan2(d[1], d[0]), math.atan2(-d[2], math.hypot(d[0], d[1]))
    return eye, rot(0.0, pitch, yaw)


def world(title, credit, geom, mu, solids, sun_dir, sun_intensity, background, luminosity, exposure=1.0,
          sky_sun=None, sun_color='1 1 1', backdrop=None, view=(3.5, 1.2, 1.5)):
    """luminosity: sky image brightness (ambient/reflections); sky_sun: strength of the sky's own sun
    light (TexturedBackgroundLight), defaults to luminosity; sun_intensity: extra DirectionalLight.
    backdrop: plain background colour instead of the sky image (the sky still lights the scene).
    view: (back, down, side) offset of the start/follow camera from the robot [m]."""
    sky_sun = luminosity if sky_sun is None else sky_sun
    s = geom['spawn']
    eye, orient = viewpoint(geom, *view)
    sky = f' skybox FALSE skyColor [ {backdrop} ]' if backdrop else ''
    return f'''#VRML_SIM R2025a utf8
# Generated by tools/build_webots.py - {title}
# {credit}
# Collision boxes are the ones of the matching Gazebo world in wcr_description/worlds.
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "../protos/Wcr.proto"

WorldInfo {{
  info [ "WCR wall-climbing robot: {title}" ]
  basicTimeStep 8
  contactProperties [
    ContactProperties {{ material1 "wheel" material2 "concrete" coulombFriction [ {mu} ] softCFM 1e-5 }}
    ContactProperties {{ material1 "default" material2 "concrete" coulombFriction [ 0.5 ] }}
  ]
}}
Viewpoint {{
  orientation {orient}
  position {eye[0]:.3f} {eye[1]:.3f} {eye[2]:.3f}
  exposure {exposure}
  follow "wcr"
  followType "Pan and Tilt Shot"
}}
TexturedBackground {{ texture "{background}" luminosity {luminosity}{sky} }}
TexturedBackgroundLight {{ texture "{background}" luminosity {sky_sun} }}
DirectionalLight {{ direction {sun_dir} color {sun_color} intensity {sun_intensity} castShadows TRUE }}

{solids}
Wcr {{
  translation {s['x']:.5f} {s['y']:.5f} {s['z']:.5f}
  rotation {rot(s['roll'], s['pitch'], s['yaw'])}
  controllerArgs [ "keyboard" ]
}}
'''


def winter_dam():
    """Photo scan: its photos already contain the real sunlight and shadows, and the glTF marks the
    materials KHR_materials_unlit (Sketchfab shows it unlit). Webots samples emissive maps at low
    detail, so instead the colour map is kept and the light is made soft and even (mostly sky light,
    very little direct sun): the photo colours come through almost unchanged, without wash-out."""
    # castShadows FALSE on the scan: its photos already contain the real shadows, and at 158k
    # triangles per part Webots could not cast them anyway (it still receives the robot's shadow)
    sdf = (WORLDS / 'dam_world.sdf').read_text()
    geom = yaml.safe_load((WORLDS / 'dam_geometry.yaml').read_text())
    shapes = ''.join(f'''    Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "../../wcr_description/worlds/meshes/dam/dam_tex_{i}.jpg" ] filtering 5 }}
        baseColor {SCAN_TINT}
        roughness 1
        metalness 0
      }}
      geometry Mesh {{ url [ "../../wcr_description/worlds/meshes/dam/dam_{i}.obj" ] }}
      castShadows FALSE
    }}
''' for i in range(4))
    # translucent tiling concrete grain just above the climbing surfaces (scan texels are ~8 mm)
    shapes += '''    Shape {
      appearance PBRAppearance {
        baseColorMap ImageTexture { url [ "../../wcr_description/worlds/meshes/dam/dam_detail.png" ] filtering 5 }
        roughness 1
        metalness 0
      }
      geometry Mesh { url [ "../../wcr_description/worlds/meshes/dam/dam_detail.obj" ] }
      castShadows FALSE
    }
'''
    solids = (solid('winter_dam', shapes, boxes(sdf, 'dam'))
              + solid('cradle', boxes(sdf, 'cradle', CRADLE_APP), boxes(sdf, 'cradle'))
              + inspection.fixed_group('winter_dam'))
    # dark backdrop like Sketchfab (hides the scan's holes); the hidden sky only lights the scene and
    # noon_park_empty keeps the photo colours neutral (noon_cloudy_countryside tinted them ~10% blue);
    # camera further back so the scan is seen at the distance it was captured for
    return world('winter dam scan', '"Dam during winter" by orphanrtg, CC-BY-4.0', geom, geom.get('mu', 0.7),
                 solids, '0.3 0.7 -0.65', 0.15, 'noon_park_empty', 1.0, exposure=1.45, sky_sun=0.35,
                 sun_color='1 0.9 0.75', backdrop='0.13 0.13 0.14', view=(6.0, 1.5, 3.0))


def grain_quad(base, up, across, n, W, L, offset=0.003):
    """Translucent tiling concrete grain on a planar face strip (W across x L up from base),
    texture coordinates in metres."""
    d = '../../wcr_description/worlds/meshes/dam'
    q = [base + across * sx * W / 2 + up * s + n * offset for sx, s in ((1, 0.05), (-1, 0.05), (-1, L - 0.1), (1, L - 0.1))]
    return f'''    Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "{d}/dam_detail.png" ] filtering 5 }}
        roughness 1
        metalness 0
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{ point [ {", ".join(" ".join(f"{c:.4f}" for c in p) for p in q)} ] }}
        texCoord TextureCoordinate {{ point [ 0 0, {W:.2f} 0, {W:.2f} {L - 0.15:.2f}, 0 {L - 0.15:.2f} ] }}
        coordIndex [ 0 1 2 3 -1 ]
        texCoordIndex [ 0 1 2 3 -1 ]
      }}
      castShadows FALSE
    }}
'''


def gravity_dam():
    """USACE model with its full PBR maps (colour, roughness, metalness, normal)."""
    sdf = (WORLDS / 'gravity_dam_world.sdf').read_text()
    geom = yaml.safe_load((WORLDS / 'gravity_dam_geometry.yaml').read_text())
    d = '../../wcr_description/worlds/meshes/gravity_dam'
    parts = yaml.safe_load((WORLDS / 'meshes' / 'gravity_dam' / 'parts.yaml').read_text())
    split_obj(WORLDS / 'meshes' / 'gravity_dam' / 'gravity_dam.obj')       # -> part_<i>.obj
    apps, shapes = set(), ''
    for obj_name, mat in parts.items():
        key = mat.replace('-', '_').upper()
        if key in apps:
            app = f'USE {key}'
        else:
            apps.add(key)
            app = f'''DEF {key} PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "{d}/{mat}.jpg" ] filtering 5 }}
        roughnessMap ImageTexture {{ url [ "{d}/{mat}_roughness.png" ] filtering 5 }}
        normalMap ImageTexture {{ url [ "{d}/{mat}_normal.png" ] filtering 5 }}
        roughness 1
        metalness 0
      }}'''   # model's metalness map is all zero (concrete); metalness 1 made it mirror the sky
        shapes += f'''    Shape {{
      appearance {app}
      geometry Mesh {{ url [ "{d}/{obj_name}.obj" ] }}
    }}
'''
    ground = '''    Pose { translation 0 0 -0.55 children [ Shape {
      appearance PBRAppearance { baseColor 0.42 0.40 0.35 roughness 1 metalness 0 }
      geometry Box { size 200 200 1 } } ] }
'''
    # concrete grain skin on the climbing face (the model's texture is soft at robot scale)
    fr = inspection.face_frame('gravity_dam')
    W, L = geom['face']['x'][1] - geom['face']['x'][0], geom['face']['length_m']
    shapes += grain_quad(fr['base'], fr['up'], fr['across'], fr['n'], W, L)
    solids = (solid('ground', ground, boxes(sdf, 'ground'))
              + solid('gravity_dam', shapes, boxes(sdf, 'gravity_dam'))
              + solid('cradle_front', boxes(sdf, 'cradle_front', CRADLE_APP), boxes(sdf, 'cradle_front'))
              + solid('cradle_back', boxes(sdf, 'cradle', CRADLE_APP), boxes(sdf, 'cradle'))
              + inspection.fixed_group('gravity_dam'))
    return world('USACE gravity dam (robot on the downstream face)', '"Gravity Dam" by USACE Sacramento District, CC-BY-4.0',
                 geom, geom.get('mu', 0.7), solids, '-0.35 -0.6 -0.72', 0.15, 'noon_cloudy_countryside', 0.6,
                 exposure=0.55)   # the sky light already contains a sun; light concrete saturates otherwise


def goldeneye_dam(site='reservoir'):
    """GoldenEye dam level: unlit baked-lighting meshes (tools/build_goldeneye_world.py), robot on
    a planar strip of the dam: 'reservoir' = 72 deg upstream face above the water (pontoon),
    'gorge' = vertical dry downstream face (hanging work platform). Collision: one box under the
    strip, the deployment ledge and the platform; a concrete grain skin covers the strip (the
    game texture is ~5 cm per pixel)."""
    import build_goldeneye_world as ge
    geom = yaml.safe_load((WORLDS / ge.SITES[site]['yaml']).read_text())
    parts = yaml.safe_load((WORLDS / 'meshes' / 'goldeneye_dam' / 'parts.yaml').read_text())
    f = geom['face']
    n, up, al = (np.array(f[k], float) for k in ('normal', 'up', 'along'))
    base, L = np.array(f['base'], float), f['length_m']
    xc = 0.5 * sum(f['strip_x'])
    W = f['strip_x'][1] - f['strip_x'][0]
    Rf = np.c_[al, up, n]                                          # x along the dam, y up, z out
    ge_dir = '../../wcr_description/worlds/meshes/goldeneye_dam'
    if geom.get('collision_mesh'):      # gorge: the visible surface itself
        face_box = f'      Mesh {{ url [ "{ge_dir}/{geom["collision_mesh"]}" ] }}\n'
    else:
        face_box = box_pose('face', base + up * (L / 2 - 0.25) - n * 0.5, Rf, (W, L + 0.5, 1.0))
    cr = np.array(geom['cradle'], float)
    ledge = (cr - up * 0.01 + n * 0.03, Rf, (0.45, 0.02, 0.06))
    zz = np.array([0, 0, 1.0])
    Rp = np.c_[al, np.cross(zz, al), zz]                           # level, x along the dam
    if site == 'reservoir':     # floating pontoon
        platform = (np.array(geom['platform_centre'], float) + [0, 0, 0.02], Rp, (1.6, 1.0, 0.3))
        platform_app = 'PBRAppearance { baseColor 0.85 0.45 0.08 roughness 0.6 metalness 0 }'
    else:                       # steel work platform hanging at the foot of the strip
        platform = (np.array(geom['platform_centre'], float) - [0, 0, 0.05], Rp, (1.6, 1.0, 0.1))
        platform_app = 'PBRAppearance { baseColor 0.35 0.37 0.4 roughness 0.4 metalness 0.8 }'
    # grain skin: a quad 3 mm above the strip (or the site's surface-following mesh), texture
    # coordinates in metres
    d = '../../wcr_description/worlds/meshes/dam'
    q = [base + al * sx * W / 2 + up * s + n * 0.003 for sx, s in ((-1, 0.05), (1, 0.05), (1, L - 0.1), (-1, L - 0.1))]
    if geom.get('grain_mesh'):
        grain_geom = f'Mesh {{ url [ "{ge_dir}/{geom["grain_mesh"]}" ] }}'
    else:
        grain_geom = f'''IndexedFaceSet {{
        coord Coordinate {{ point [ {", ".join(" ".join(f"{c:.4f}" for c in p) for p in q)} ] }}
        texCoord TextureCoordinate {{ point [ 0 0, {W:.2f} 0, {W:.2f} {L - 0.15:.2f}, 0 {L - 0.15:.2f} ] }}
        coordIndex [ 0 1 2 3 -1 ]
        texCoordIndex [ 0 1 2 3 -1 ]
      }}'''
    grain = f'''    Shape {{
      appearance PBRAppearance {{
        baseColorMap ImageTexture {{ url [ "{d}/dam_detail.png" ] filtering 5 }}
        roughness 1
        metalness 0
      }}
      geometry {grain_geom}
      castShadows FALSE
    }}
'''
    solids = (solid('goldeneye_dam', ge.shapes(parts) + grain, face_box)
              + solid('cradle', box_pose('ledge', *ledge, CRADLE_APP), box_pose('ledge', *ledge))
              + solid('platform', box_pose('platform', *platform, platform_app), box_pose('platform', *platform))
              + inspection.fixed_group('goldeneye_dam' if site == 'reservoir' else 'goldeneye_gorge'))
    return world(f'GoldenEye 007 dam level (robot on the {geom["side"]})',
                 '"GoldenEye 007 - Dam Level Baked" by Vince_Crusty, CC-BY-4.0', geom, geom.get('mu', 0.7),
                 solids, '-0.3 0.8 -0.5', 1.0, 'noon_cloudy_mountains', 1.0, exposure=1.0,
                 backdrop='0.6 0.7 0.85', view=(5.0, 1.0, 2.5))


def main():
    import sys
    if '--simple' in sys.argv:              # only the simplified robot: WcrSimple.proto + wcr_simple.urdf
        (WB / 'protos' / 'WcrSimple.proto').write_text(proto(simple=True))
        (ROOT / 'wcr_webots' / 'resource' / 'wcr_simple.urdf').write_text(ros_urdf(simple=True))
        print('wrote WcrSimple.proto and wcr_simple.urdf')
        return
    (WB / 'protos').mkdir(parents=True, exist_ok=True)
    (WB / 'worlds').mkdir(parents=True, exist_ok=True)
    (WB / 'protos' / 'Wcr.proto').write_text(proto())
    (ROOT / 'wcr_webots' / 'resource').mkdir(parents=True, exist_ok=True)
    (ROOT / 'wcr_webots' / 'resource' / 'wcr.urdf').write_text(ros_urdf())
    (WB / 'worlds' / 'winter_dam.wbt').write_text(winter_dam())
    (WB / 'worlds' / 'gravity_dam.wbt').write_text(gravity_dam())
    (WB / 'worlds' / 'goldeneye_dam.wbt').write_text(goldeneye_dam('reservoir'))
    (WB / 'worlds' / 'goldeneye_gorge.wbt').write_text(goldeneye_dam('gorge'))
    print('wrote Wcr.proto, winter_dam.wbt, gravity_dam.wbt, goldeneye_dam.wbt, goldeneye_gorge.wbt in', WB)


if __name__ == '__main__':
    main()
