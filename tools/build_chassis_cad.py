"""WCR robot for Fusion 360, rebuilt solid by solid from the original CAD export.

The meshes in wcr_description/meshes are a merged CAD export: every original body (plate, bracket,
servo, board, screw, nut, ...) is a separate piece of the mesh. Each piece is rebuilt as a true
B-rep solid where its shape allows, and kept only if its volume matches the original:
  - round parts (screws, washers, standoffs, spacers, shafts, tubes, ...): the exact half profile
    is cut through the axis and revolved (true cylindrical / conical faces)
  - 2.5D parts (plates, brackets, boards, boxes): a stack of extrusions, one per flat level, with
    the tessellated circles and arcs of each outline turned back into true arcs
Identical pieces become instances of one part. These go into wcr_robot.step.

The rest (filleted 3D parts: servo brackets, fan, suction bell, nuts with chamfers, ...) is left to
Fusion's own mesh -> B-rep conversion (Prismatic, Fusion 2025.07+): each unique piece is written
once as a centred STL with the positions of all its copies, the tiny board details (< 3 mm) merged
into one mesh per component, and the Fusion script WCR_Build.py (from tools/fusion/) imports the
STEP, converts and places the pieces and adds the joints -> save the result as .f3d.

Assembly (mm, origin = base_link: z = 0 is the wall, x forward / up the wall, y left), one
component per URDF link so Fusion joints map onto the URDF:
  chassis            robotChassisNoProp.stl without the side motors and arm tubes (= chassis_body.obj)
  edf_rotor          edfProp.stl                                    edf_joint   (z axis, continuous)
  XX_steering_pod    activeWheel.stl at (+-112.5, +-112.5, 60.65)   XX_steering_joint (z axis)
  XX_wheel           wheel.stl, 35.25 mm below the pod               XX_wheel_joint (y axis)
  sim_sensors        lidar + wall camera with their mounts (added for the simulation)

  python tools/build_chassis_cad.py [--out cad] [--parts N] [--stats]
      -> cad/WCR_Build/ (wcr_robot.step, parts.json, meshes/, WCR_Build.py + .manifest)
      --parts N: only the N largest chassis pieces (tests); --stats: recognition statistics only
needs: cadquery trimesh shapely networkx scipy
"""
import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh
from shapely.geometry import Polygon, box as sbox

ROOT = Path(__file__).resolve().parents[1]
MESHES = ROOT / 'wcr_description' / 'meshes'
CHASSIS_Z = 10.36                   # chassis_link above base_link [mm]
POD_XY, STEER_Z, WHEEL_DROP = 112.5, 60.65, 35.25

# pod / wheel visual orientation per corner, as in resource/wcr.urdf (rpy, rad)
CORNERS = {'FL': (1, 1, (0, 0, math.pi / 2), (0, math.pi / 2, 0)),
           'BL': (-1, 1, (0, 0, math.pi / 2), (0, math.pi / 2, 0)),
           'FR': (1, -1, (0, 0, -math.pi / 2), (0, 0, math.pi)),
           'BR': (-1, -1, (0, 0, -math.pi / 2), (0, 0, math.pi))}


# ---------------------------------------------------------------- mesh -> B-rep
def revolved(mesh):
    """(local shape with the axis = z, axis index, centre) if the piece is a solid of revolution
    about an axis parallel to x, y or z, else None."""
    if not mesh.is_watertight or mesh.volume <= 0:
        return None
    e, c = mesh.extents, mesh.bounds.mean(0)
    for ax in sorted(range(3), key=lambda i: -abs(e[i] - np.median(e))):
        u_ax, n_ax = [i for i in range(3) if i != ax]
        if abs(e[u_ax] - e[n_ax]) > 0.02 * e[u_ax] + 0.02:
            continue
        n = np.zeros(3)
        n[n_ax] = 1.0
        s = mesh.section(plane_origin=c, plane_normal=n)
        if s is None:
            continue
        region = Polygon()
        for loop in s.discrete:                                 # even-odd: nested loops are voids
            p = Polygon(np.c_[loop[:, u_ax] - c[u_ax], loop[:, ax] - c[ax]]).buffer(0)
            region = region.symmetric_difference(p)
        half = region.intersection(sbox(0, -1e4, 1e4, 1e4)).simplify(0.002)
        polys = [g for g in getattr(half, 'geoms', [half]) if g.geom_type == 'Polygon' and g.area > 1e-4]
        if not polys:
            continue
        solid = None
        for p in polys:
            part = revolve_poly(p.exterior.coords)
            for hole in p.interiors:
                part = part.cut(revolve_poly(hole.coords))
            solid = part if solid is None else solid.fuse(part)
        if solid is not None and volume_matches(solid.Volume(), mesh.volume):
            return solid.clean(), ax, c
    return None


FRAME = {2: (0, 1, 2), 0: (1, 2, 0), 1: (2, 0, 1)}      # axis -> (u, v, w) coordinate indices (right-handed)


def extruded(mesh):
    """The piece as a stack of extrusions along x, y or z (2.5D CAD parts: plates, brackets, boards,
    boxes): one slab between each pair of flat levels, its outline taken from a section at the
    slab's middle, tessellated circles and arcs turned back into true arcs. None if no axis
    reproduces the volume."""
    if not mesh.is_watertight or mesh.volume <= 0:
        return None
    fn, centres = mesh.face_normals, mesh.triangles_center
    for ax in sorted(range(3), key=lambda i: -mesh.area_faces[np.abs(fn[:, i]) > 0.9999].sum()):
        flat = np.abs(fn[:, ax]) > 0.9999
        side = np.abs(fn[:, ax]) < 1e-3
        if mesh.area_faces[~flat & ~side].sum() > 0.10 * mesh.area:     # slanted faces: not a 2.5D part
            continue
        lo, hi = mesh.bounds[:, ax]
        levels = np.unique(np.r_[np.round(centres[flat, ax], 3), lo, hi])
        levels = levels[np.r_[True, np.diff(levels) > 0.01]]
        if len(levels) > 30:
            continue
        u, v, w = FRAME[ax]
        slabs = []
        try:
            for z0, z1 in zip(levels[:-1], levels[1:]):
                o = mesh.bounds.mean(0)
                o[ax] = (z0 + z1) / 2
                n = np.zeros(3)
                n[ax] = 1.0
                s = mesh.section(plane_origin=o, plane_normal=n)
                if s is None:
                    continue
                region = Polygon()
                for loop in s.discrete:
                    region = region.symmetric_difference(Polygon(loop[:, [u, v]]).buffer(0))
                for poly in getattr(region, 'geoms', [region]):
                    if poly.geom_type != 'Polygon' or poly.area < 1e-4:
                        continue
                    face = cq.Face.makeFromWires(ring_wire(poly.exterior.coords, ax, z0),
                                                 [ring_wire(h.coords, ax, z0) for h in poly.interiors])
                    d = [0.0, 0.0, 0.0]
                    d[ax] = float(z1 - z0)
                    slabs.append(cq.Solid.extrudeLinear(face, cq.Vector(*d)))
            if not slabs:
                continue
            # slabs touch along exact shared faces: glue fusion, then merge the split side faces
            solid = (slabs[0].fuse(*slabs[1:], glue=True) if len(slabs) > 1 else slabs[0]).clean()
        except Exception:
            continue
        if volume_matches(solid.Volume(), mesh.volume):
            return solid
    return None


def volume_matches(rebuilt, original):
    """The rebuilt part has true circles where the mesh has inscribed polygons, so it may be up to
    ~5 % larger (a 12-gon has 4.7 % less area than its circle), but hardly smaller."""
    d = (rebuilt - original) / original
    return -0.02 < d < 0.055 or abs(rebuilt - original) < 0.3


def ring_wire(coords, ax, w0):
    """Closed polygon ring (u, v) at w = w0 -> wire of lines and true arcs (runs of >= 3 vertices
    on one circle, as CAD tessellates arcs, become one arc; a ring all on one circle a circle)."""
    p = np.array(coords)[:-1]
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-6]]
    N = len(p)
    u, v, w = FRAME[ax]

    def P(q):
        x = [0.0, 0.0, 0.0]
        x[u], x[v], x[w] = float(q[0]), float(q[1]), float(w0)
        return cq.Vector(*x)

    # circumcircle of every vertex with its neighbours
    a, b, c = np.roll(p, 1, 0), p, np.roll(p, -1, 0)
    d = 2 * (a[:, 0] * (b[:, 1] - c[:, 1]) + b[:, 0] * (c[:, 1] - a[:, 1]) + c[:, 0] * (a[:, 1] - b[:, 1]))
    ok = np.abs(d) > 1e-9
    d = np.where(ok, d, 1.0)
    sa, sb, sc = (a ** 2).sum(1), (b ** 2).sum(1), (c ** 2).sum(1)
    cx = (sa * (b[:, 1] - c[:, 1]) + sb * (c[:, 1] - a[:, 1]) + sc * (a[:, 1] - b[:, 1])) / d
    cy = (sa * (c[:, 0] - b[:, 0]) + sb * (a[:, 0] - c[:, 0]) + sc * (b[:, 0] - a[:, 0])) / d
    r = np.hypot(b[:, 0] - cx, b[:, 1] - cy)
    seg = np.linalg.norm(c - b, axis=1)
    arc = ok & (r < 500) & (seg < r * 0.8)                       # vertex inside a tessellated arc

    def same(i, j):
        i, j = i % N, j % N
        return arc[i] and arc[j] and abs(cx[i] - cx[j]) < 0.02 and abs(cy[i] - cy[j]) < 0.02 and abs(r[i] - r[j]) < 0.02

    # a vertex is inside an arc if it shares its circle with a neighbour; the others are breakpoints
    # (corners, arc ends, junctions between two arcs). Edges run from breakpoint to breakpoint.
    inner = [same(k, k - 1) or same(k, k + 1) for k in range(N)]
    breaks = [k for k in range(N) if not inner[k]]
    if not breaks:
        if N >= 8 and all(same(0, k) for k in range(N)):          # full circle
            circle = cq.Edge.makeCircle(float(r[0]), P((cx[0], cy[0])), cq.Vector(*np.eye(3)[ax]))
            return cq.Wire.assembleEdges([circle])
        breaks = list(range(N))
    edges = []
    for b0, b1 in zip(breaks, breaks[1:] + [breaks[0] + N]):
        between = [k % N for k in range(b0 + 1, b1)]
        if not between:
            edges.append(cq.Edge.makeLine(P(p[b0 % N]), P(p[b1 % N])))
        elif len(between) >= 2 and all(same(between[0], k) for k in between):
            mid = between[len(between) // 2]
            edges.append(cq.Edge.makeThreePointArc(P(p[b0 % N]), P(p[mid]), P(p[b1 % N])))
        else:
            chain = [b0 % N] + between + [b1 % N]
            edges += [cq.Edge.makeLine(P(p[s]), P(p[e])) for s, e in zip(chain, chain[1:])]
    return cq.Wire.assembleEdges(edges)


def revolve_poly(coords):
    pts = [(max(float(u), 0.0), float(z)) for u, z in list(coords)[:-1]]
    wp = cq.Workplane('XZ').polyline(pts).close()
    return wp.revolve(360, (0, 0, 0), (0, 1, 0)).val()


AXIS_ROT = {2: ((0, 0, 1), 0.0), 0: ((0, 1, 0), 90.0), 1: ((1, 0, 0), -90.0)}   # local z -> axis


def axis_location(ax, c):
    loc = cq.Location(cq.Vector(*map(float, c)))
    axis, ang = AXIS_ROT[ax]
    return loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), ang) if ang else loc


# ---------------------------------------------------------------- pieces -> unique parts + instances
def convert(pieces, label):
    """pieces -> [(proto_key, shape, local Location, kind, piece)], identical pieces share a shape."""
    protos, out, stats = {}, [], defaultdict(int)
    t0 = time.time()
    for i, m in enumerate(pieces):
        c = m.bounds.mean(0)
        key = (len(m.faces), *np.round(m.extents, 2), round(float(m.area), 1))
        if key in protos:                                      # same piece, check it is a translation
            proto_m, shape, kind, ax, pc = protos[key]
            a = np.round(np.sort(proto_m.vertices - pc, axis=0), 2)
            b = np.round(np.sort(m.vertices - c, axis=0), 2)
            if a.shape == b.shape and np.allclose(a, b, atol=0.02):
                loc = (axis_location(ax, c) if kind == 'revolved' else c if kind in ('mesh', 'detail')
                       else cq.Location(cq.Vector(*map(float, c - pc))))
                out.append((key, shape, loc, kind, m))
                stats['instance'] += 1
                continue
        r = revolved(m) if m.extents.max() < 120 else None
        x = extruded(m) if r is None else None
        if r is not None:
            shape, ax, _ = r
            protos.setdefault(key, (m, shape, 'revolved', ax, c))
            out.append((key, shape, axis_location(ax, c), 'revolved', m))
            stats['revolved'] += 1
        elif x is not None:
            protos.setdefault(key, (m, x, 'extruded', None, c))
            out.append((key, x, cq.Location(), 'extruded', m))
            stats['extruded'] += 1
        else:                                                   # left to Fusion (see module doc)
            kind = 'detail' if m.extents.max() < 3 or (not m.is_watertight and m.extents.max() < 10) else 'mesh'
            protos.setdefault(key, (m, None, kind, None, c))
            out.append((key, None, c, kind, m))
            stats[kind] += 1
            stats[kind + '_tris'] += len(m.faces)
        if i % 250 == 0:
            print(f'  {label}: {i}/{len(pieces)} pieces, {time.time() - t0:.0f} s', flush=True)
    print(f'  {label}: {len(pieces)} pieces -> {dict(stats)} in {time.time() - t0:.0f} s')
    return out


# ---------------------------------------------------------------- naming
def classify(m, kind):
    e = np.sort(m.extents)
    if e[2] < 3.0:
        return 'components'                                     # SMD parts, pins, LEDs on the boards
    if kind == 'revolved' and e[2] <= 30:
        return 'fasteners'
    if e[2] <= 12 and m.is_watertight and m.volume < 300:
        return 'fasteners'
    return 'parts'


COLORS = {'parts': (0.80, 0.80, 0.78), 'fasteners': (0.55, 0.56, 0.6), 'components': (0.15, 0.15, 0.15),
          'pod': (0.30, 0.32, 0.36), 'wheel': (0.08, 0.08, 0.08), 'edf': (0.12, 0.12, 0.12),
          'sensors': (0.85, 0.55, 0.15)}


def add_group(assy, items, prefix, color_of):
    names = defaultdict(int)
    for key, shape, loc, kind, m in items:
        if shape is None:                                       # mesh / detail: left to Fusion
            continue
        cat = color_of(m, kind)
        e = np.round(np.sort(m.extents)[::-1], 1)
        base = f'{prefix}{cat[:-1] if cat.endswith("s") else cat}_{e[0]:g}x{e[1]:g}x{e[2]:g}'
        names[base] += 1
        assy.add(shape, name=f'{base}_{names[base]}', loc=loc,
                 color=cq.Color(*COLORS.get(cat, COLORS['parts'])))


def pieces_of(stl, keep=lambda b: True):
    m = trimesh.load(MESHES / stl, force='mesh')
    return [b for b in m.split(only_watertight=False) if keep(b)]


def chassis_keep(b):
    """Same selection as tools/clean_chassis.py (chassis_body.obj)."""
    lo, hi = b.bounds
    motor = abs(b.bounds.mean(0)[1]) > 150.0
    tube = (abs(b.bounds.mean(0)[0]) < 1.0 and 75 < min(abs(lo[1]), abs(hi[1])) < 85
            and 160 < max(abs(lo[1]), abs(hi[1])) < 170 and np.allclose(b.extents[0], 25, atol=1))
    return not motor and not tube


def rpy_loc(xyz, rpy):
    r, p, y = rpy
    loc = cq.Location(cq.Vector(*xyz))
    for axis, ang in (((0, 0, 1), y), ((0, 1, 0), p), ((1, 0, 0), r)):     # R = Rz * Ry * Rx
        if ang:
            loc = loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(*axis), math.degrees(ang))
    return loc


def sim_sensors():
    """Lidar, wall camera and their mounts as in tools/build_webots.py (base_link frame, mm)."""
    cyl = lambda x, y, r, z0, z1: cq.Workplane('XY').workplane(offset=z0).center(x, y).circle(r).extrude(z1 - z0)
    box = lambda c, s: cq.Workplane('XY').box(*s).translate(c)
    mounts = (cyl(0, -40, 6, 99, 226).union(cyl(0, -40, 38, 222, 226))
              .union(box((174.5, -70, 113.5), (31, 22, 5))).union(box((188, -70, 102), (4, 22, 26))))
    lidar = cyl(0, -40, 35, 238, 262)
    cam = box((0, 0, 0), (12, 24, 24)).rotate((0, 0, 0), (0, 1, 0), 70).translate((180, -70, 90))
    return {'sensor_mounts': mounts.val(), 'lidar': lidar.val(), 'wall_camera': cam.val()}


def mesh_parts(items, group, out):
    """Write the pieces left to Fusion: one centred STL per unique 'mesh' piece (+ the centres of all
    its instances) and all 'detail' pieces of the group merged into one STL in component coordinates."""
    (out / 'meshes').mkdir(parents=True, exist_ok=True)
    uniq, details = {}, []
    for key, shape, c, kind, m in items:
        if kind == 'detail':
            details.append(m)
        elif kind == 'mesh':
            if key not in uniq:
                e = np.round(np.sort(m.extents)[::-1], 1)
                name = f'{group}_{len(uniq):03d}_{e[0]:g}x{e[1]:g}x{e[2]:g}'
                m.copy().apply_translation(-m.bounds.mean(0)).export(out / 'meshes' / f'{name}.stl')
                uniq[key] = dict(name=name, file=f'meshes/{name}.stl', tris=len(m.faces), at=[])
            uniq[key]['at'].append([round(float(v), 4) for v in c])
    entry = dict(parts=list(uniq.values()), details=None)
    if details:
        trimesh.util.concatenate(details).export(out / 'meshes' / f'{group}_details.stl')
        entry['details'] = f'meshes/{group}_details.stl'
    print(f'  {group}: {len(uniq)} unique parts for Fusion ({sum(len(p["at"]) for p in uniq.values())} placed), '
          f'{len(details)} detail pieces')
    return entry


def build(args, out):
    root, fusion = cq.Assembly(name='wcr'), {}
    chassis = pieces_of('robotChassisNoProp.stl', chassis_keep)
    if args.parts:
        chassis = sorted(chassis, key=lambda b: -b.area)[:args.parts]
    ch = cq.Assembly(name='chassis', loc=cq.Location(cq.Vector(0, 0, CHASSIS_Z)))
    items = convert(chassis, 'chassis')
    for cat in ('parts', 'fasteners', 'components'):
        sub = cq.Assembly(name=cat)
        add_group(sub, [it for it in items if classify(it[4], it[3]) == cat], '', classify)
        ch.add(sub)
    root.add(ch)
    fusion['chassis'] = mesh_parts(items, 'chassis', out)

    edf = cq.Assembly(name='edf_rotor', loc=cq.Location(cq.Vector(0, 0, CHASSIS_Z)))
    items = convert(pieces_of('edfProp.stl'), 'edf')
    add_group(edf, items, '', lambda m, k: 'edf')
    root.add(edf)
    fusion['edf_rotor'] = mesh_parts(items, 'edf_rotor', out)

    pod_items = convert(pieces_of('activeWheel.stl'), 'steering pod')
    wheel_items = convert(pieces_of('wheel.stl'), 'wheel')
    fusion['steering_pod'] = mesh_parts(pod_items, 'steering_pod', out)
    fusion['wheel'] = mesh_parts(wheel_items, 'wheel', out)
    for n, (sx, sy, pod_rpy, wheel_rpy) in CORNERS.items():
        x, y = sx * POD_XY, sy * POD_XY
        pod = cq.Assembly(name=f'{n}_steering_pod', loc=rpy_loc((x, y, STEER_Z), pod_rpy))
        add_group(pod, pod_items, '', lambda m, k: 'pod')
        root.add(pod)
        wheel = cq.Assembly(name=f'{n}_wheel', loc=rpy_loc((x, y, STEER_Z - WHEEL_DROP), wheel_rpy))
        add_group(wheel, wheel_items, '', lambda m, k: 'wheel')
        root.add(wheel)

    sens = cq.Assembly(name='sim_sensors')
    for name, shape in sim_sensors().items():
        sens.add(shape, name=name, color=cq.Color(*COLORS['sensors']))
    root.add(sens)
    return root, fusion


def joints():
    """Joints for the Fusion script (base_link frame, mm), as in resource/wcr.urdf."""
    js = [dict(name='edf_joint', parent='chassis', child='edf_rotor', at=[0, 0, CHASSIS_Z], axis='z')]
    for n, (sx, sy, _, _) in CORNERS.items():
        x, y = sx * POD_XY, sy * POD_XY
        js.append(dict(name=f'{n}_steering_joint', parent='chassis', child=f'{n}_steering_pod',
                       at=[x, y, STEER_Z], axis='z', limits=[-90, 90]))
        js.append(dict(name=f'{n}_wheel_joint', parent=f'{n}_steering_pod', child=f'{n}_wheel',
                       at=[x, y, STEER_Z - WHEEL_DROP], axis='y'))
    for s in ('sensor_mounts', 'lidar', 'wall_camera'):
        js.append(dict(name=f'{s}_fixed', parent='chassis', child=s, at=[0, 0, 0], axis='rigid'))
    return js


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(ROOT / 'cad'))
    ap.add_argument('--parts', type=int, default=0)
    ap.add_argument('--stats', action='store_true')
    args = ap.parse_args()
    if args.stats:
        convert(pieces_of('robotChassisNoProp.stl', chassis_keep), 'chassis')
        for stl in ('activeWheel.stl', 'wheel.stl', 'edfProp.stl'):
            convert(pieces_of(stl), stl)
        return
    out = Path(args.out) / 'WCR_Build'
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    assy, fusion = build(args, out)
    step = out / 'wcr_robot.step'
    print('writing', step, flush=True)
    assy.export(str(step), write_pcurves=False)                  # no 2D pcurves: much smaller file
    pods = [f'{n}_steering_pod' for n in CORNERS]
    wheels = [f'{n}_wheel' for n in CORNERS]
    manifest = dict(units='mm', step='wcr_robot.step', joints=joints(),
                    groups=[dict(targets=['chassis'], **fusion['chassis']),
                            dict(targets=['edf_rotor'], **fusion['edf_rotor']),
                            dict(targets=pods, **fusion['steering_pod']),
                            dict(targets=wheels, **fusion['wheel'])])
    (out / 'parts.json').write_text(json.dumps(manifest, indent=1))
    for f in ('WCR_Build.py', 'WCR_Build.manifest', 'README.md'):    # the Fusion script itself
        (out / f).write_text((Path(__file__).parent / 'fusion' / f).read_text())
    print(f'{step}: {step.stat().st_size / 1e6:.1f} MB, {time.time() - t0:.0f} s total')


if __name__ == '__main__':
    main()
