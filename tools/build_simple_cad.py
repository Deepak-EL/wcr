"""Simplified WCR robot for Fusion 360: ~20 unique parts, modelled the plain way (chamfered blocks,
tubes, plates, simple revolves) at the real positions and sizes of the parts in the CAD export
(robotChassisNoProp / activeWheel / wheel / edfProp; bounds measured with tools/build_chassis_cad.py).
Electronics, battery, screws and all small parts are left out.

Frames (mm) as in tools/build_chassis_cad.py and resource/wcr.urdf: chassis parts in the chassis
frame (10.36 above base_link), pod parts in the steering pod frame (z = 0 at the steering joint),
wheel parts in the wheel frame (axle = y), EDF rotor in the chassis frame.
Identical parts are one shape placed several times (mirror pairs: one left and one right part).

  python tools/build_simple_cad.py   -> cad/WCR_Simple/ (wcr_simple.step, preview.png, WCR_Build.py
                                        + parts.json: the Fusion script adds the joints)
needs: cadquery trimesh shapely
"""
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np
from shapely.geometry import box as rect
from shapely.ops import unary_union

import build_chassis_cad as B

ROOT = Path(__file__).resolve().parents[1]
TUBE_Z = 67.3                       # axis height of the frame tubes (chassis frame)
POD = 112.5                         # steering axes at (+-POD, +-POD)
POD_Z = B.STEER_Z - B.CHASSIS_Z     # steering joint in the chassis frame (50.29)

# the simulation's colours (tools/build_webots.py): charcoal chassis, red steering pods, black
# wheels, anodised aluminium sensor mounts, near-black lidar and camera
COL = dict(chassis=(0.2, 0.21, 0.24), pod=(0.72, 0.22, 0.18), wheel=(0.08, 0.08, 0.09),
           mount=(0.62, 0.63, 0.66), lidar=(0.08, 0.08, 0.09), camera=(0.1, 0.1, 0.12))


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1, fillet=0.0):
    b = cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
    return b.edges('|Z').fillet(fillet) if fillet else b


def mbox(x0, x1, y0, y1, z0, z1, cv=3.0, ch=0.8):
    """Machined-look block: vertical corners chamfered (cv), top and bottom edges broken (ch)."""
    b = box(x0, x1, y0, y1, z0, z1)
    if cv:
        b = b.edges('|Z').chamfer(cv)
    if ch:
        b = b.faces('>Z or <Z').edges().chamfer(ch)
    return b


def cyl(axis, c, r, a0, a1):
    """Cylinder along 'x' | 'y' | 'z' through point c (the other two coordinates), from a0 to a1."""
    wp = {'z': cq.Workplane('XY'), 'x': cq.Workplane('YZ'), 'y': cq.Workplane('XZ')}[axis]
    i = 'xyz'.index(axis)
    u, v = [c[k] for k in range(3) if k != i]
    if axis == 'y':                                             # XZ workplane: normal -y, local x = x, local y = z
        return wp.workplane(offset=-a1).center(u, v).circle(r).extrude(a1 - a0)
    return wp.workplane(offset=a0).center(u, v).circle(r).extrude(a1 - a0)


def tube(axis, c, ro, ri, a0, a1):
    return cyl(axis, c, ro, a0, a1).cut(cyl(axis, c, ri, a0 - 1, a1 + 1))


def revolve(points):
    """Closed (r, z) profile revolved about z."""
    return cq.Workplane('XZ').polyline(points).close().revolve(360, (0, 0, 0), (0, 1, 0))


ROT180 = cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 180)
I = cq.Location()


def T(x=0, y=0, z=0):
    return cq.Location(cq.Vector(x, y, z))


# ---------------------------------------------------------------- frame (chassis frame)
def square_tube():                       # 30 mm tube, 100 long, along x (placed rotated for y)
    return tube('x', (0, 0, 0), 15, 13, -50, 50)


def cross_tube():                        # 25 mm tube, 136 long, along y
    return tube('y', (0, 0, 0), 12.5, 10.5, -68, 68)


def corner_bracket():
    """Front-left corner (+x, +y): a flat plate joining the square-frame corner block, the cross-tube
    clamp block and a solid tower that carries the steering servo; the fork post turns in a plain
    hole through plate and tower."""
    outline = unary_union([rect(46, 46, 84, 84), rect(94, 60, 131, 93), rect(88.5, 88.5, 136.5, 136.5),
                           rect(101.2, 93.5, 147.8, 131.5)]).convex_hull
    plate = cq.Workplane('XY').workplane(offset=52.3).polyline(list(outline.exterior.coords)[:-1]).close().extrude(8)
    plate = plate.faces('>Z').edges().chamfer(1)
    blk = mbox(46, 84, 46, 84, TUBE_Z - 18, TUBE_Z + 18, cv=5, ch=1)
    blk = blk.cut(cyl('y', (65, 0, TUBE_Z), 15.2, 45, 66)).cut(cyl('x', (0, 65, TUBE_Z), 15.2, 45, 66))
    sock = mbox(94, 131, 60, 93, TUBE_Z - 17, TUBE_Z + 17, cv=4, ch=1)  # cross-tube clamp block
    tower = mbox(101.2, 147.8, 93.5, 131.5, 60.3, 89.9, cv=4, ch=1)
    part = plate.union(blk).union(sock).union(tower)
    part = part.cut(cyl('y', (POD, 0, TUBE_Z), 12.7, 59, 84))            # cross tube
    part = part.cut(cyl('z', (POD, POD, 0), 11, 50, 86))                 # fork post
    part = part.cut(cyl('z', (POD, POD, 0), 3.2, 85, 91))                # servo output
    return part.cut(cyl('z', (POD, 76.5, 0), 3.2, 76, 86))               # antenna mount hole


def antenna():
    """Wi-Fi whip antenna standing on a clamp block: hex base, tilt knuckle, rubber whip (base at z = 0)."""
    base = cq.Workplane('XY').polygon(6, 11).extrude(5).faces('>Z').edges().chamfer(0.6)
    knuckle = cyl('y', (0, 0, 9), 4.5, -4, 4).union(mbox(-4.5, 4.5, -4, 4, 5, 9, cv=1, ch=0))
    whip = cyl('z', (0, 0, 0), 4.5, 13, 88).faces('>Z').edges().chamfer(2)
    sleeve = cyl('z', (0, 0, 0), 4.8, 13, 30)
    return base.union(knuckle).union(whip).union(sleeve).union(cyl('z', (0, 0, 0), 3, -8, 0))


def tube_clamp():                        # block around a 30 mm tube along x, 60 long
    blk = mbox(-30, 30, -20, 20, TUBE_Z - 20, TUBE_Z + 20, cv=4, ch=1)
    blk = blk.cut(cyl('x', (0, 0, TUBE_Z), 15.2, -31, 31))
    return blk.cut(box(-31, 31, -21, 21, TUBE_Z - 0.5, TUBE_Z + 0.5))        # split line: two halves


def steering_servo():                    # plain block on the tower, output shaft down into it
    body = mbox(103.5, 145.5, 102.5, 122.5, 89.9, 112.9, cv=1.5, ch=1)
    return body.union(cyl('z', (POD, POD, 0), 3, 85, 89.9))


# ---------------------------------------------------------------- EDF + suction (chassis frame)
def suction_bell():
    outer = [(87.5, 0), (75, 3), (60, 7.5), (48, 13), (42, 19), (40.5, 26)]
    inner = [(r - 2.5, z + 1.8) for r, z in reversed(outer)]
    bell = revolve(outer + [(38, 26)] + inner[1:] + [(85, 0)])
    for y in (-42, 42):
        bell = bell.union(mbox(-8, 8, y - 4, y + 4, 18, 80.5, cv=1.5, ch=0))
    return bell


def skirt_flap():
    """Curved sheet clipped under the square tube, for the +y side, centred on x = 0 (70 long)."""
    sheet = (cq.Workplane('YZ', origin=(-35, 0, 0)).moveTo(86, 45).threePointArc((97, 22), (125, 8))
             .lineTo(125, 10.2).threePointArc((99.2, 23.2), (88.2, 45)).close().extrude(70))
    tab = box(-35, 35, 70, 90, 43, 47)
    return sheet.union(tab)


def edf_duct():
    duct = tube('z', (0, 0, 0), 37.9, 35.5, 29.8, 81.2)
    duct = duct.union(tube('z', (0, 0, 0), 44, 36, 78.2, 81.2))
    for k in range(3):                                       # stator struts carrying the motor
        a = math.radians(90 + 120 * k)
        s = box(-1.5, 1.5, 15, 36, 75, 78).rotate((0, 0, 0), (0, 0, 1), math.degrees(a) - 90)
        duct = duct.union(s)
    return duct.union(tube('z', (0, 0, 0), 17.5, 15.4, 72, 78))


def edf_motor():
    can = cyl('z', (0, 0, 0), 15.4, 62, 103.9).faces('>Z').edges().fillet(2)
    return can.union(cyl('z', (0, 0, 0), 2.5, 28.2, 62))


def edf_rotor():                         # edfProp frame (= chassis frame)
    rotor = tube('z', (0, 0, 0), 12, 2.6, 28.2, 61.7)
    for k in range(9):
        blade = box(11, 33.9, -0.8, 0.8, 34, 56).rotate((0, 0, 45), (1, 0, 45), 30)
        rotor = rotor.union(blade.rotate((0, 0, 0), (0, 0, 1), 40 * k))
    return rotor.union(revolve([(0, 5.2), (2, 5.2), (14.7, 28.2), (0, 28.2)]))


# ---------------------------------------------------------------- steering pod (pod frame)
def steering_fork():
    """One part: round post (turns in the corner bracket, servo on top), top plate, two plain tapered
    side plates with the axle hole."""
    fork = mbox(-17.4, 19.9, -26, 26, -5, 0, cv=5, ch=0.8)
    leg = [(-22, -4), (22, -4), (10, -42), (-10, -42)]             # (y, z) outline
    for x0 in (-17.4, 14.9):
        fork = fork.union(cq.Workplane('YZ', origin=(x0, 0, 0)).polyline(leg).close().extrude(5))
    fork = fork.cut(cyl('x', (0, 0, -35.25), 4.2, -20, 22))
    return fork.union(cyl('z', (0, 0, 0), 10, 0, 30))             # post up into the tower


def drive_motor():                       # servo-sized drive motor beside the fork, shaft into the wheel
    body = mbox(-56.4, -22.4, -14.2, 14.2, -46.5, 0, cv=2, ch=1)
    return body.union(cyl('x', (0, 0, -35.25), 2.5, -22.4, -12.4))


# ---------------------------------------------------------------- wheel (wheel frame, axle = y)
def wheel():
    """One solid wheel: chamfered tyre, recessed faces, hub boss towards the drive motor (-y)."""
    prof = [(3, -14), (10, -14), (10, -9.15), (18, -9.15), (18, -10.65), (23.6, -10.65), (25.4, -8.85),
            (25.4, 8.85), (23.6, 10.65), (18, 10.65), (18, 9.15), (10, 9.15), (10, 10.65), (3, 10.65)]
    return cq.Workplane('XY').polyline(prof).close().revolve(360, (0, 0, 0), (0, 1, 0))


# ---------------------------------------------------------------- assembly
def build():
    root = cq.Assembly(name='wcr_simple')
    ch = cq.Assembly(name='chassis', loc=T(z=B.CHASSIS_Z))

    def add(asm, name, shape, color, locs):
        for k, loc in enumerate(locs):
            asm.add(shape, name=f'{name}_{k + 1}' if len(locs) > 1 else name, loc=loc, color=cq.Color(*COL[color]))

    rz90 = cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), 90)
    C = 'chassis'
    add(ch, 'square_tube', square_tube().val(), C,
        [T(0, 65, TUBE_Z), T(0, -65, TUBE_Z), T(65, 0, TUBE_Z) * rz90, T(-65, 0, TUBE_Z) * rz90])
    add(ch, 'cross_tube', cross_tube().val(), C, [T(POD, 0, TUBE_Z), T(-POD, 0, TUBE_Z)])
    left = corner_bracket()
    add(ch, 'corner_bracket_left', left.val(), C, [I, ROT180])
    add(ch, 'corner_bracket_right', left.mirror('XZ').val(), C, [I, ROT180])
    add(ch, 'tube_clamp', tube_clamp().val(), C, [T(0, 65), T(0, -65), T(65, 0) * rz90, T(-65, 0) * rz90])
    corners = [I, ROT180, T(0, -2 * POD), T(0, 2 * POD) * ROT180]  # (+,+) (-,-) (+,-) (-,+)
    add(ch, 'steering_servo', steering_servo().val(), C, corners)
    top = TUBE_Z + 17                                         # clamp block top: two front antennas, one rear
    add(ch, 'antenna', antenna().val(), C, [T(POD, 76.5, top), T(POD, -76.5, top), T(-POD, 76.5, top)])
    add(ch, 'suction_bell', suction_bell().val(), C, [I])
    add(ch, 'skirt_flap', skirt_flap().val(), C, [T(35), T(-35), ROT180 * T(35), ROT180 * T(-35)])
    add(ch, 'edf_duct', edf_duct().val(), C, [I])
    add(ch, 'edf_motor', edf_motor().val(), C, [I])
    root.add(ch)

    edf = cq.Assembly(name='edf_rotor', loc=T(z=B.CHASSIS_Z))
    add(edf, 'rotor', edf_rotor().val(), C, [I])
    root.add(edf)

    pod_parts = [('steering_fork', steering_fork().val()), ('drive_motor', drive_motor().val())]
    wheel_shape = wheel().val()
    for n, (sx, sy, pod_rpy, wheel_rpy) in B.CORNERS.items():
        x, y = sx * POD, sy * POD
        pod = cq.Assembly(name=f'{n}_steering_pod', loc=B.rpy_loc((x, y, B.STEER_Z), pod_rpy))
        for name, shape in pod_parts:
            add(pod, name, shape, 'pod', [I])
        root.add(pod)
        wh = cq.Assembly(name=f'{n}_wheel', loc=B.rpy_loc((x, y, B.STEER_Z - B.WHEEL_DROP), wheel_rpy))
        add(wh, 'wheel', wheel_shape, 'wheel', [I])
        root.add(wh)

    sens = cq.Assembly(name='sim_sensors')                    # base_link frame
    s = B.sim_sensors()
    zc = B.CHASSIS_Z
    bridge = (box(-85, 85, -50, -30, 87.3 + zc, 91.3 + zc)                    # across the side tube clamps
              .union(box(80, 90, -76, -64, 91.3 + zc, 116)).union(box(80, 190, -76, -64, 111, 116)))
    mounts = cq.Workplane().add(s['sensor_mounts']).union(bridge)
    sens.add(mounts.val(), name='sensor_mounts', color=cq.Color(*COL['mount']))
    sens.add(s['lidar'], name='lidar', color=cq.Color(*COL['lidar']))
    sens.add(s['wall_camera'], name='wall_camera', color=cq.Color(*COL['camera']))
    root.add(sens)
    return root


def preview(assy, png):
    """Coloured isometric preview (point splatting of the tessellated parts)."""
    import trimesh
    from PIL import Image
    meshes = []
    for shape, name, loc, color in assy:                       # leaves with absolute locations
        v, f = shape.moved(loc).tessellate(0.2, 0.3)
        m = trimesh.Trimesh(np.array([p.toTuple() for p in v]), np.array(f), process=False)
        meshes.append((m, np.array(color.toTuple()[:3]) * 255 if color else np.array([200, 200, 200])))
    az, el = math.radians(-50), math.radians(28)
    R = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    E = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    V = E @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ R
    P, C = [], []
    light = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
    for m, col in meshes:
        pts, fi = trimesh.sample.sample_surface(m, int(max(2000, m.area * 30)))
        sh = 0.35 + 0.65 * np.abs(m.face_normals[fi] @ V.T @ light)
        P.append(pts @ V.T)
        C.append(np.clip(np.maximum(col, 25) * sh[:, None], 0, 255))
    P, C = np.vstack(P), np.vstack(C)
    s = 3.0
    u = ((P[:, 0] - P[:, 0].min()) * s).astype(int)
    v = ((P[:, 1].max() - P[:, 1]) * s).astype(int)
    img = np.full((v.max() + 1, u.max() + 1, 3), 255.0)
    o = np.argsort(P[:, 2])
    img[v[o], u[o]] = C[o]
    Image.fromarray(img.astype(np.uint8)).save(png)


SIM_MESHES = ROOT / 'wcr_description' / 'meshes' / 'simple'
MAX_TRIS = 20000                        # per OBJ: Webots casts no shadows from meshes over 21845 triangles


def export_sim_meshes(assy):
    """The model as OBJ meshes (mm) for the Webots robot WcrSimple.proto and wcr_simple.urdf
    (tools/build_webots.py --simple), each in its link frame like the original meshes:
      chassis_<k>.obj  chassis frame (chunks below MAX_TRIS)     mounts.obj  base frame (sensor mounts)
      pod.obj          steering pod frame (FL)                    wheel.obj   wheel frame (FL)
      edf.obj          EDF rotor (chassis frame)"""
    import trimesh
    SIM_MESHES.mkdir(parents=True, exist_ok=True)
    for f in SIM_MESHES.glob('*.obj'):
        f.unlink()

    def leaves(child, only=None):
        for shape, name, loc, _ in child.__iter__(child.loc.inverse):          # in the child's own frame
            if only is None or name.split('/')[-1] in only:
                v, f = shape.moved(loc).tessellate(0.08, 0.2)
                yield trimesh.Trimesh(np.array([p.toTuple() for p in v]), np.array(f))

    def write(meshes, stem, chunked=False):
        chunks, cur = [], []
        for m in meshes:
            if chunked and cur and sum(len(c.faces) for c in cur) + len(m.faces) > MAX_TRIS:
                chunks.append(cur)
                cur = []
            cur.append(m)
        chunks.append(cur)
        for k, c in enumerate(chunks):
            m = trimesh.graph.smooth_shade(trimesh.util.concatenate(c), angle=math.radians(30))
            name = f'{stem}_{k}.obj' if chunked else f'{stem}.obj'
            m.export(SIM_MESHES / name, include_normals=True, include_texture=False)
            print(f'  {name}: {len(m.faces)} tris')

    kids = {c.name: c for c in assy.children}
    write(leaves(kids['chassis']), 'chassis', chunked=True)
    write(leaves(kids['edf_rotor']), 'edf')
    write(leaves(kids['FL_steering_pod']), 'pod')
    write(leaves(kids['FL_wheel']), 'wheel')
    write(leaves(kids['sim_sensors'], only={'sensor_mounts'}), 'mounts')


def main():
    out = ROOT / 'cad' / 'WCR_Simple'
    out.mkdir(parents=True, exist_ok=True)
    assy = build()
    step = out / 'wcr_simple.step'
    assy.export(str(step), write_pcurves=False)
    leaves = [name.split('/')[-1] for _, name, _, _ in assy]
    kinds = sorted({n.rsplit('_', 1)[0] if n.rsplit('_', 1)[-1].isdigit() else n for n in leaves})
    print(f'{step}: {step.stat().st_size / 1e6:.1f} MB, {len(leaves)} placed parts, {len(kinds)} kinds:')
    print('  ' + ', '.join(kinds))
    manifest = dict(units='mm', step=step.name, joints=B.joints(), groups=[])
    (out / 'parts.json').write_text(json.dumps(manifest, indent=1))
    for f in ('WCR_Build.py', 'WCR_Build.manifest'):
        (out / f).write_text((Path(__file__).parent / 'fusion' / f).read_text())
    preview(assy, out / 'preview.png')
    export_sim_meshes(assy)


if __name__ == '__main__':
    main()
