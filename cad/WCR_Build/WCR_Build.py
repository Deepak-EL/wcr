"""WCR_Build: builds the WCR wall-climbing robot as a Fusion design.

Files next to this script (made by tools/build_chassis_cad.py):
  wcr_robot.step   the parts rebuilt as exact solids, one component per robot link
  parts.json       the remaining parts (centred STL + every position), board details, joints
  meshes/          those STLs

Steps: new design -> import the STEP -> import every remaining part once, convert it to a solid
with Fusion's Convert Mesh (Prismatic, else Faceted; needs Fusion 2025.07+, else it stays a mesh
body) and place a copy at each position -> board details as mesh bodies -> as-built joints
(steering +-90 deg, wheels and EDF continuous, sensors rigid). Progress / problems: WCR_Build.log.
Run: Utilities > Add-Ins > Scripts and Add-Ins > + (this folder) > Run. Then File > Save.
"""
import json
import math
import os
import time
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
CM = 0.1                                    # parts.json is in mm, the Fusion API in cm
LOG = os.path.join(HERE, 'WCR_Build.log')


def log(msg):
    with open(LOG, 'a', encoding='utf-8') as f:
        f.write(time.strftime('%H:%M:%S ') + msg + '\n')


def find_comp(design, name):
    comps = list(design.allComponents)
    for c in comps:
        if c.name == name:
            return c
    for c in comps:                          # Fusion may append ' (1)' etc.
        if c.name.startswith(name + ' ') or c.name.startswith(name + '_'):
            return c
    return None


def find_occ(root, comp):
    occs = root.allOccurrencesByComponent(comp)
    return occs.item(0) if occs.count else None


def import_mesh(comp, path):
    """Mesh body from an STL (mm) in comp, inside a base feature (parametric design)."""
    before = comp.meshBodies.count
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    try:
        comp.meshBodies.add(path, adsk.fusion.MeshUnits.MillimeterMeshUnit, bf)
    finally:
        bf.finishEdit()
    return comp.meshBodies.item(comp.meshBodies.count - 1) if comp.meshBodies.count > before else None


def convert(comp, mesh, stats):
    """Mesh body -> solid with Fusion's Convert Mesh: Prismatic, else Faceted, else keep the mesh."""
    try:
        feats = comp.features.meshConvertFeatures
    except AttributeError:
        stats['mesh kept (Fusion older than 2025.07)'] += 1
        return
    for method in ('PrismaticMeshConvertMethodType', 'FacetedMeshConvertMethodType'):
        try:
            try:
                inp = feats.createInput([mesh])
            except Exception:
                coll = adsk.core.ObjectCollection.create()
                coll.add(mesh)
                inp = feats.createInput(coll)
            inp.meshConvertMethodType = getattr(adsk.fusion.MeshConvertMethodTypes, method)
            feats.add(inp)
            mesh.isLightBulbOn = False
            stats[method.replace('MeshConvertMethodType', '').lower()] += 1
            return
        except Exception as e:
            log(f'  {comp.name}: {method} failed: {e}')
    stats['mesh kept (conversion failed)'] += 1


def place_parts(design, groups, ui):
    stats = {k: 0 for k in ('prismatic', 'faceted', 'mesh kept (conversion failed)',
                            'mesh kept (Fusion older than 2025.07)', 'copies')}
    total = sum(len(g['parts']) for g in groups)
    prog = ui.createProgressDialog()
    prog.isCancelButtonShown = True
    prog.show('WCR build', 'Converting parts: %v of %m', 0, max(total, 1))
    done = 0
    for g in groups:
        targets = [c for c in (find_comp(design, t) for t in g['targets']) if c]
        if not targets:
            log(f'no component for {g["targets"]}: skipped')
            continue
        for p in g['parts']:
            if prog.wasCancelled:
                return stats
            part = None
            for t in targets:
                for c in p['at']:
                    m = adsk.core.Matrix3D.create()
                    m.translation = adsk.core.Vector3D.create(c[0] * CM, c[1] * CM, c[2] * CM)
                    if part is None:
                        occ = t.occurrences.addNewComponent(m)
                        part = occ.component
                        part.name = p['name']
                        mesh = import_mesh(part, os.path.join(HERE, p['file']))
                        if mesh:
                            convert(part, mesh, stats)
                    else:
                        t.occurrences.addExistingComponent(part, m)
                        stats['copies'] += 1
            done += 1
            prog.progressValue = done
            adsk.doEvents()
        if g.get('details'):
            for t in targets:                    # tiny board parts: visual mesh bodies
                mesh = import_mesh(t, os.path.join(HERE, g['details']))
                if mesh:
                    mesh.name = 'details'
    prog.hide()
    return stats


def add_joints(design, root, joints):
    planes, made = {}, 0
    for j in joints:
        try:
            parent, child = find_comp(design, j['parent']), find_comp(design, j['child'])
            po, co = (find_occ(root, parent) if parent else None), (find_occ(root, child) if child else None)
            if not po or not co:
                log(f'joint {j["name"]}: occurrence missing')
                continue
            x, y, z = (v * CM for v in j['at'])
            if z not in planes:                  # joint origin: a sketch point on a plane at that height
                pi = root.constructionPlanes.createInput()
                pi.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(z))
                plane = root.constructionPlanes.add(pi)
                plane.isLightBulbOn = False
                sk = root.sketches.add(plane)
                sk.name = f'joint origins z {z / CM:.2f} mm'
                sk.isVisible = False
                planes[z] = sk
            sk = planes[z]
            pt = sk.sketchPoints.add(sk.modelToSketchSpace(adsk.core.Point3D.create(x, y, z)))
            geo = adsk.fusion.JointGeometry.createByPoint(pt)
            inp = root.asBuiltJoints.createInput(co, po, geo)
            if j['axis'] == 'rigid':
                inp.setAsRigidJointMotion()
            else:
                d = {'x': adsk.fusion.JointDirections.XAxisJointDirection,
                     'y': adsk.fusion.JointDirections.YAxisJointDirection,
                     'z': adsk.fusion.JointDirections.ZAxisJointDirection}[j['axis']]
                inp.setAsRevoluteJointMotion(d)
            abj = root.asBuiltJoints.add(inp)
            abj.name = j['name']
            if j.get('limits'):
                lim = abj.jointMotion.rotationLimits
                lim.isMinimumValueEnabled, lim.minimumValue = True, math.radians(j['limits'][0])
                lim.isMaximumValueEnabled, lim.maximumValue = True, math.radians(j['limits'][1])
            made += 1
        except Exception as e:
            log(f'joint {j["name"]}: {e}')
    return made


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        open(LOG, 'w').close()
        man = json.load(open(os.path.join(HERE, 'parts.json'), encoding='utf-8'))
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        try:
            design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception as e:
            log(f'display units: {e}')
        root = design.rootComponent
        root.name = 'WCR wall-climbing robot'

        log('importing ' + man['step'])
        im = app.importManager
        im.importToTarget(im.createSTEPImportOptions(os.path.join(HERE, man['step'])), root)
        adsk.doEvents()

        stats = place_parts(design, man['groups'], ui)
        log(f'parts: {stats}')
        try:
            find_occ(root, find_comp(design, 'chassis')).isGrounded = True
        except Exception as e:
            log(f'grounding the chassis: {e}')
        n = add_joints(design, root, man['joints'])
        log(f'joints: {n} of {len(man["joints"])}')
        app.activeViewport.fit()
        ui.messageBox(f'WCR robot built.\n\nParts converted: {stats["prismatic"]} prismatic, {stats["faceted"]} faceted, '
                      f'{stats["mesh kept (conversion failed)"] + stats["mesh kept (Fusion older than 2025.07)"]} left as mesh; '
                      f'{stats["copies"]} copies placed.\nJoints: {n} of {len(man["joints"])}.\n\n'
                      f'Details in WCR_Build.log. Save with File > Save.', 'WCR build')
    except Exception:
        log(traceback.format_exc())
        ui.messageBox('WCR build failed:\n' + traceback.format_exc(), 'WCR build')
