"""Orchestrator-owned fixes for V5 simulacra defects that no workstream owns (runs after actuation).

The simulacra export dropped most of the V5 SolidWorks cut tools, so some mating features are missing:

1. Index palm bone: the pockets for cover3's three bracket straps are missing. The straps (2.45 mm plates,
   world Z 4.5..6.95) overlapped the bone by 373 mm3. The pinky bone still has its pockets for cover2's
   straps (floor 0.05 below the strap, flush top), so the index bone gets the same: strap outline + 0.2 mm
   per side, floor 0.05 below the strap. The screw then clamps strap to bone (head in the strap counterbore,
   captive M2 nut in the bone).
2. cover2: its three straps over the pinky bone have no screw holes. Cut the same hole + counterbore the
   cover3 straps have: dia 2.4 through, dia 4.2 counterbore down to Z 5.227 (seat of an M2x5 low/button head).

All positions come from the V5 geometry placed with the exact Onshape occurrence transforms.
"""
import json, os
import FreeCAD as App, Part
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
EXP = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/v5_step/Robotic Hand_V5_simulacra - '
CLEAR = 0.2
FLOOR_GAP = 0.05
P_SITES = [(-100.97, -71.16), (-120.43, -65.81), (-139.12, -59.8)]   # cover2 strap screws (world XY), axis -Z
STRAP_TOP, SEAT_Z = 6.95, 5.227
STRAP_BOTTOM = 4.45          # world Z of the strap pocket floors (V5 pinky design)
# Integration clearances between modules (found by the global audit, 2026-09-26):
# (a) mod_thumb's yaw carrier sweeps within r 16.06 of the R1 axis (Z -21.3..7.03, measured on the built carrier);
#     mod_actuation's index magazine corner reached into that sweep -> keep r 16.06 + CLEAR clear of the carrier.
YAW_AXIS_XY = (-165.3445, 20.958)
CARRIER_R, CARRIER_Z = 16.06 + CLEAR, (-21.3 - CLEAR, 7.03 + CLEAR)
# (b) a corner of mod_actuation's T1/T2 housing on cover3 touched Shell2's rim (vertex contact); a R1.5 spherical
#     relief restores the shells design gap (0.25 mm).
SHELL2_CORNER, CORNER_R = (-179.6, 25.9, -60.98), 1.5
_STATE = {}

_occ = None
def _placement(name):
    global _occ
    if _occ is None:
        d = json.load(open(os.path.join(ROOT, 'data/top_assembly_definition.json')))
        ra = d['rootAssembly']
        inst = {i['id']: i for i in ra['instances']}
        for s in d['subAssemblies']:
            for i in s['instances']:
                inst[i['id']] = i
        _occ = {}
        for o in ra['occurrences']:
            t = o['transform']
            _occ[inst[o['path'][-1]]['name']] = App.Placement(App.Matrix(
                t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000, t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))
    return _occ[name]

def _placed_v5(fname, occ_name):
    s = Part.read(EXP + fname)
    s.Placement = _placement(occ_name).multiply(s.Placement)
    return s

def strap_pockets_world(cov, bone, floor=None):
    """Pocket solids (world) for every place the cover's straps overlap the bone: strap footprint (union of horizontal
    sections through the strap, widest outline wins) + CLEAR per side, floor FLOOR_GAP below the strap, open upward."""
    bb_bone = bone.BoundBox
    pockets = []
    for sol in cov.common(bone).Solids:
        if sol.Volume < 0.05:
            continue
        bb = sol.optimalBoundingBox(True, False)      # tight: Shape.BoundBox can be loose on spline/torus faces
        region = cov.common(Part.makeBox(bb.XLength + 2, bb_bone.YMax + 0.3 - (bb_bone.YMin - 0.3), bb.ZLength + 0.7,
                                         App.Vector(bb.XMin - 1, bb_bone.YMin - 0.3, bb.ZMin - 0.2)))
        faces = []
        for z in [bb.ZMin + 0.05 + k * max(bb.ZMax - bb.ZMin - 0.1, 0.01) / 5 for k in range(6)]:
            for w in region.slice(App.Vector(0, 0, 1), z):
                f = Part.Face(w)
                if f.Area > 0.5:
                    f.translate(App.Vector(0, 0, -z))
                    faces.append(f)
        if not faces:
            continue
        foot = faces[0]
        for f in faces[1:]:
            foot = foot.fuse(f)
        foot = foot.removeSplitter()
        z0 = (bb.ZMin - FLOOR_GAP) if floor is None else floor   # an explicit floor avoids near-coincident faces
        for ff in foot.Faces:                       # offset each outline face (holes dropped) and extrude it
            off = Part.Face(ff.OuterWire).makeOffset2D(CLEAR, 0, False, False, False)
            off.translate(App.Vector(0, 0, z0))
            pockets.append(off.extrude(App.Vector(0, 0, bb.ZMax + 1.0 - z0)))
    return pockets

def index_strap_pockets_world():
    cov = _placed_v5('driver_side_palm_cover3.step', 'driver_side_palm_cover3 <1>')
    bone = _placed_v5('Base Bone 1.2_V02_pointerfinger_and_thumb_attachment.step', 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>')
    return strap_pockets_world(cov, bone)

def cover2_holes_world():
    tools = []
    for x, y in P_SITES:
        tools.append(Part.makeCylinder(1.2, 6.0, App.Vector(x, y, 3.0), App.Vector(0, 0, 1)))   # through the strap only (Z 4.5..6.95)
        tools.append(Part.makeCylinder(2.1, 5, App.Vector(x, y, SEAT_Z), App.Vector(0, 0, 1)))
    return tools

def _to_local(tools, occ_name):
    inv = _placement(occ_name).inverse()
    out = []
    for t in tools:
        t = t.copy()
        t.Placement = inv.multiply(t.Placement)
        out.append(t)
    return out

def _fresh(shape):
    """Independent copy via a BREP round trip: no TShapes shared with the input, tolerance history cleaned."""
    c = Part.Shape()
    c.importBrepFromString(shape.exportBrepToString())
    return c

def _robust_cut(shape, tools):
    """Boolean cut that must end in one valid solid. Works on a fresh copy so healing can never mutate the caller's
    shape (OCC booleans share untouched faces with their arguments): plain cut, healed copy, then fuzzy cuts."""
    base = _fresh(shape)
    tried = []
    for fuzzy in (None, 1e-4, 1e-3):
        try:
            r = _largest(base.cut(tools) if fuzzy is None else base.cut(tools, fuzzy))
            if r.isValid():
                return r
            h = _fresh(r)
            h.fix(1e-6, 1e-6, 1e-4)
            h = _largest(h)
            if h.isValid():
                return h
            tried.append('fuzzy=%s invalid' % fuzzy)
        except Exception as e:
            tried.append('fuzzy=%s %s' % (fuzzy, e))
    raise RuntimeError('mod_fixes: cut did not give a valid solid: ' + '; '.join(tried))

def _largest(s):
    """Largest solid of s. No refine (removeSplitter): on these OCC results it mutates shared sub-shapes in place and
    leaves invalid solids; the extra coplanar faces it would merge are harmless for printing."""
    return max(s.Solids, key=lambda x: x.Volume)

def modify(key, shape):
    if key == 'palm_index':
        carrier_zone = Part.makeCylinder(CARRIER_R, CARRIER_Z[1] - CARRIER_Z[0],
                                         App.Vector(YAW_AXIS_XY[0], YAW_AXIS_XY[1], CARRIER_Z[0]), App.Vector(0, 0, 1))
        tools = _to_local(index_strap_pockets_world() + [carrier_zone], 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>')
        return _robust_cut(shape, tools)
    if key == 'cover3':
        relief = Part.makeSphere(CORNER_R, App.Vector(*SHELL2_CORNER))
        return _robust_cut(shape, _to_local([relief], 'driver_side_palm_cover3 <1>'))
    if key == 'palm_pinky':
        # cover2's straps (world Z 4.5..6.95) sit in pockets in the pinky bone's palm face. Actuation's worm-seat fills
        # close them again, so re-cut the STRAP LAYER only against the current bone (the overlap is confined to it).
        pl = _placement('Base Bone 1.2_V02_pinky <1>')
        bone_w = shape.copy(); bone_w.Placement = pl.multiply(bone_w.Placement)
        cov = _placed_v5('driver_side_palm_cover2.step', 'driver_side_palm_cover2 <1>')
        bb = bone_w.BoundBox
        strap_layer = cov.common(Part.makeBox(bb.XLength + 20, bb.YLength + 20, STRAP_TOP + 1.0 - STRAP_BOTTOM,
                                              App.Vector(bb.XMin - 10, bb.YMin - 10, STRAP_BOTTOM)))
        pockets = strap_pockets_world(strap_layer, bone_w, floor=STRAP_BOTTOM)   # coplanar with the V5 pocket floors
        new = _robust_cut(shape, _to_local(pockets, 'Base Bone 1.2_V02_pinky <1>')) if pockets else shape
        return new
    if key == 'cover2':
        return _robust_cut(shape, _to_local(cover2_holes_world(), 'driver_side_palm_cover2 <1>'))
    return shape
