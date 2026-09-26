"""V6 pillars (requirement 6): PLA pillars through the flexible TPU palm insert + TPU/bone clearance fixes.

Problem: the TPU palm insert (strips Palm1/Palm2_2/Palm3_2/Palm_pinky, webs Palm2 x3, wrist plate palm_bone_flex) lies in
one plane and is placed during a print pause. It splits every palm bone into a palm half (bone y < 0.5) and a back half
(y > 1.1) that were joined only beyond the strip ends (J0 block, some wrist ends), so the bones broke there. The wrist
segments are split the same way (their finger-side bar is two thin half-bars).

What this module does (all dimensions mm, clearance C = 0.2 per side, requirement 7):
  palm bones   1. pocket: cut offset(TPU footprint, C) through the TPU layer (bone y 0.5..1.1). This removes the unslotted
                  ring/pinky wrist ends and the 0.1-0.17 mm skins the webs ran through (all audit overlaps) and gives the
                  insert 0.2 mm in-plane clearance to every pocket wall (J0 block, index/pinky end caps).
               2. pillars: stadium-shaped PLA pillars through the layer at the ribs between the motor seats (3 per bone)
                  and through the wrist plate at the middle/ring wrist ends.
  wrist segs   same pocket rule (wrist frame, layer z -1.1..-0.5) + a bar pillar in wrist_middle / wrist_ring that fills
               the layer gap in the finger-side bar between the two plate pegs (index/pinky segments are already solid
               through the layer on their outer sides).
  peg holes    reamed to r 1.21 about each TPU peg's own axis (V5 holes were up to 0.08 mm eccentric).
  TPU parts    holes = pillar footprint offset by C; pegs shortened by PEG_TRIM so the sheet, not the pegs, bottoms out.
Print: back side down, pause when the pillars reach the TPU palm-side face (world z -1.023); see notes/pillars.md.
Frames: palm bones as in the README (J0 axis (89.3, y, 0), palm side -y, layer y 0.5..1.1). Wrist segments: the shared
Palm_bone1 frame (palm side +z, layer z -1.1..-0.5). Relative poses come from data/top_assembly_definition.json.
"""
import os, json, math
import FreeCAD as App, Part
V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
C = 0.2                       # clearance per side
LIG = 1.2                     # minimum TPU ligament / PLA wall
BY0, BY1 = 0.5, 1.1           # TPU layer in palm-bone local y (palm side = -y)
WZ0, WZ1 = -1.1, -0.5         # TPU layer in wrist-segment local z (palm side = +z)
EPS = 0.15                    # pillars overlap the halves by this much for a clean fuse (raster kept >=1.2 mm
                              # of bone wall around every pillar over the first 1.2 mm of depth, so 0.15 adds no
                              # material outside the bone and gives a ~3 mm^3 measured embed into each half)
PEG_TRIM = 0.25               # TPU pegs shortened by this much (tip clearance in their holes; V5 holes are
                              # up to ~0.005 shallower than the pegs are long, so 0.25 guarantees >= 0.2)

BONE_INST = {'palm_index': 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>',
             'palm_middle': 'Base Bone 1.2_V02_middlefinger <1>',
             'palm_ring': 'Base Bone 1.2_V02_ringfing <1>',
             'palm_pinky': 'Base Bone 1.2_V02_pinky <1>'}
WRIST_INST = {'wrist_index': 'Palm_bone1 <5>', 'wrist_middle': 'Palm_bone1 <2>',
              'wrist_ring': 'Palm_bone1 <4>', 'wrist_pinky': 'Palm_bone1 <1>'}
TPU_INST = {'tpu_index': ['Palm1 <1>'], 'tpu_middle': ['Palm2_2 <1>'], 'tpu_ring': ['Palm3_2 <1>'],
            'tpu_pinky': ['Palm_pinky <1>'], 'tpu_web': ['Palm2 <1>', 'Palm2 <2>', 'Palm2 <3>'],
            'tpu_wrist': ['palm_bone_flex <1>']}
TPU_SRC = {'tpu_index': 'SRC_Palm1', 'tpu_middle': 'SRC_Palm2_2', 'tpu_ring': 'SRC_Palm3_2', 'tpu_pinky': 'SRC_Palm_pinky',
           'tpu_web': 'SRC_Palm2', 'tpu_wrist': 'SRC_palm_bone_flex'}

# ---- pillar layout ----------------------------------------------------------------------------------------------
# Palm bones, bone-local x-z plane (pillar = footprint x TPU layer y 0.5..1.1, fused into both halves by EPS).
#   ('s', cx, cz, wx, lz): stadium, extent wx along the bone (x) and lz across it (z); round ends on the short axis.
#   ('p', [(x, z), ...]):  polygon (index / pinky, see below).
# Placement (work/pillars/scratch/raster_opt2.py + poly_pillars.py): largest footprint per rib such that
#   - both halves have material over it, with >= 1.2 mm 3D wall to every void of the halves (seats, windows, peg holes,
#     cover2/cover3 screw bores) -- raster margin 1.25;
#   - >= 1.2 mm wall to the ACTUATION DRAFT motor envelopes (regions/actuation_motors.json: n20.body_envelope(0.2)
#     + cap + 8 mm plug keep-out + shaft) and the HARDWARE bracket-fastener keep-outs of the index bone
#     (regions/hardware.json *BRK*: M2x5 + captive nut pockets); the TPU hole never overlaps a fastener keep-out;
#   - TPU hole (footprint + C) keeps >= 1.2 mm of TPU to every window, edge, peg (r 1.0) and to the plate/strip seam
#     at bone x ~5.93 (wrist-end pillars stop at x <= 4.5).
# index/pinky: polygons = largest allowed region per rib with minimum width 2.4 mm (raster opening r 1.2, 0.1 mm inset);
# the index ribs carry the cover3 bracket nut pockets (hardware), so rib 2 has no room and rib 1 wraps below its pocket.
# The actuation magazine design (regions/actuation_motors.json of 21:28) keeps every motor >= 6.4 mm from all pillars.
PILLARS = {
    'palm_index':  [('p', [(23.500, -4.375), (23.550, -4.325), (23.800, -4.325), (24.150, -4.175), (24.425, -3.900), (24.575, -3.650), (25.175, -3.050), (25.325, -2.700), (25.325, -2.450), (25.375, -2.400), (25.325, -2.350), (25.325, -2.100), (25.175, -1.750), (24.725, -1.300), (24.625, -1.050), (24.625, 3.100), (24.575, 3.150), (24.575, 3.400), (24.425, 3.750), (24.150, 4.025), (23.800, 4.175), (23.550, 4.175), (23.500, 4.225), (23.450, 4.175), (23.200, 4.175), (23.150, 4.125), (23.050, 4.125), (22.775, 3.950), (22.575, 3.750), (22.475, 3.550), (22.475, 3.450), (22.425, 3.400), (22.425, 3.150), (22.375, 3.100), (22.375, -3.250), (22.425, -3.300), (22.425, -3.550), (22.475, -3.600), (22.475, -3.700), (22.575, -3.900), (22.775, -4.100), (23.050, -4.275), (23.150, -4.275), (23.200, -4.325), (23.450, -4.325)]),
                    ('p', [(63.200, -6.225), (64.750, -6.225), (64.800, -6.175), (65.050, -6.175), (65.400, -6.025), (65.675, -5.750), (65.775, -5.550), (65.775, -5.450), (65.825, -5.400), (65.825, -5.150), (65.875, -5.100), (65.875, -4.850), (65.925, -4.800), (65.925, -4.600), (66.025, -4.400), (66.025, -4.300), (66.325, -3.750), (66.875, -3.200), (67.025, -2.850), (67.025, -2.600), (67.075, -2.550), (67.025, -2.500), (67.025, -2.250), (66.875, -1.900), (66.550, -1.575), (66.400, -1.525), (66.075, -1.200), (65.975, -0.950), (65.975, 4.400), (65.925, 4.450), (65.925, 4.700), (65.875, 4.750), (65.875, 5.000), (65.725, 5.350), (65.450, 5.625), (65.100, 5.775), (64.850, 5.775), (64.800, 5.825), (63.350, 5.825), (63.300, 5.775), (63.050, 5.775), (63.000, 5.725), (62.900, 5.725), (62.700, 5.625), (62.425, 5.350), (62.275, 5.000), (62.275, 4.750), (62.225, 4.700), (62.275, 4.600), (62.275, -4.350), (62.125, -4.750), (62.125, -5.000), (62.075, -5.100), (62.125, -5.150), (62.125, -5.400), (62.175, -5.450), (62.175, -5.550), (62.300, -5.775), (62.550, -6.025), (62.900, -6.175), (63.150, -6.175)])],
    'palm_middle': [('s', 25.55, 0.0, 5.8, 5.4), ('s', 45.525, 0.05, 5.75, 5.4), ('s', 66.025, 0.05, 6.95, 5.4),
                    ('s', 3.025, -0.05, 2.95, 9.6)],
    'palm_ring':   [('s', 25.575, 0.1, 5.75, 5.4), ('s', 45.55, 0.1, 5.7, 5.5), ('s', 66.075, 0.15, 6.95, 5.4),
                    ('s', 3.075, 0.0, 2.85, 9.9)],
    'palm_pinky':  [('p', [(23.500, -3.475), (23.550, -3.425), (23.800, -3.425), (23.850, -3.375), (23.950, -3.375), (24.550, -3.025), (25.050, -2.825), (25.300, -2.825), (25.350, -2.775), (25.850, -2.775), (25.900, -2.725), (26.150, -2.725), (26.200, -2.675), (26.300, -2.675), (26.500, -2.575), (26.775, -2.300), (26.925, -1.950), (26.925, -1.700), (26.975, -1.650), (26.925, -1.600), (26.925, -1.350), (26.875, -1.300), (26.875, -1.200), (26.625, -0.750), (26.575, -0.450), (26.525, -0.400), (26.525, 0.250), (26.575, 0.300), (26.625, 0.600), (26.975, 1.250), (26.975, 1.350), (27.025, 1.400), (27.025, 1.650), (27.075, 1.700), (27.025, 1.750), (27.025, 2.000), (26.875, 2.350), (26.600, 2.625), (26.400, 2.725), (26.300, 2.725), (26.250, 2.775), (26.000, 2.775), (25.950, 2.825), (25.050, 2.825), (24.550, 3.025), (23.950, 3.375), (23.850, 3.375), (23.800, 3.425), (23.550, 3.425), (23.500, 3.475), (23.450, 3.425), (23.200, 3.425), (23.150, 3.375), (23.050, 3.375), (22.775, 3.200), (22.575, 3.000), (22.475, 2.800), (22.475, 2.700), (22.425, 2.650), (22.425, 2.400), (22.375, 2.350), (22.375, -2.350), (22.425, -2.400), (22.425, -2.650), (22.575, -3.000), (22.850, -3.275), (23.200, -3.425), (23.450, -3.425)]),
                    ('p', [(43.450, -3.075), (43.500, -3.025), (43.750, -3.025), (44.050, -2.875), (44.400, -2.825), (44.450, -2.775), (45.100, -2.775), (45.150, -2.725), (45.400, -2.725), (45.750, -2.575), (46.025, -2.300), (46.175, -1.950), (46.175, -1.700), (46.225, -1.650), (46.175, -1.600), (46.175, -1.350), (46.125, -1.300), (46.125, -0.750), (46.175, -0.700), (46.225, -0.400), (46.425, 0.000), (46.950, 0.575), (47.500, 0.875), (47.600, 0.875), (47.800, 0.975), (48.125, 1.300), (48.275, 1.650), (48.275, 1.900), (48.325, 1.950), (48.325, 2.500), (48.275, 2.550), (48.275, 2.800), (48.125, 3.150), (47.850, 3.425), (47.500, 3.575), (47.250, 3.575), (47.200, 3.625), (47.150, 3.575), (46.900, 3.575), (46.550, 3.425), (46.150, 3.125), (45.450, 2.825), (44.250, 2.825), (43.750, 3.025), (43.500, 3.025), (43.450, 3.075), (43.400, 3.025), (43.150, 3.025), (42.800, 2.875), (42.525, 2.600), (42.375, 2.250), (42.375, 2.000), (42.325, 1.950), (42.325, -1.950), (42.375, -2.000), (42.375, -2.250), (42.425, -2.300), (42.425, -2.400), (42.600, -2.675), (42.800, -2.875), (43.000, -2.975), (43.100, -2.975), (43.150, -3.025), (43.400, -3.025)]),
                    ('p', [(63.300, -5.925), (63.850, -5.925), (63.900, -5.875), (64.150, -5.875), (64.200, -5.825), (64.300, -5.825), (64.500, -5.725), (64.825, -5.400), (64.975, -5.050), (64.975, -4.800), (65.275, -4.050), (65.525, -3.700), (66.325, -2.950), (66.475, -2.600), (66.475, -2.350), (66.525, -2.300), (66.525, -2.200), (66.475, -2.150), (66.475, -1.900), (66.325, -1.550), (66.325, -0.800), (66.375, -0.750), (66.425, -0.450), (66.575, -0.150), (67.250, 0.525), (67.500, 0.675), (67.600, 0.675), (67.800, 0.775), (68.075, 1.050), (68.225, 1.400), (68.225, 1.650), (68.275, 1.700), (68.225, 1.750), (68.225, 2.000), (68.075, 2.350), (67.800, 2.625), (67.450, 2.775), (66.900, 2.825), (66.200, 3.125), (65.425, 3.850), (65.125, 4.350), (65.025, 4.750), (64.975, 4.800), (64.925, 5.350), (64.775, 5.700), (64.500, 5.975), (64.150, 6.125), (63.900, 6.125), (63.850, 6.175), (63.300, 6.175), (63.250, 6.125), (63.000, 6.125), (62.650, 5.975), (62.375, 5.700), (62.225, 5.350), (62.225, 5.100), (62.175, 5.050), (62.175, 4.950), (62.225, 4.900), (62.225, 4.650), (62.275, 4.600), (62.275, -4.350), (62.225, -4.400), (62.225, -4.650), (62.175, -4.700), (62.175, -4.800), (62.225, -4.850), (62.225, -5.100), (62.275, -5.150), (62.275, -5.250), (62.375, -5.450), (62.550, -5.625), (62.850, -5.825), (62.950, -5.825), (63.000, -5.875), (63.250, -5.875)])],
}
# wrist segments (Palm_bone1 frame): bar pillar = (both halves' material at the layer) clipped to x in [xa, xb], y in [ya, yb].
# x range = between the two plate pegs of the bar, 2.4 mm from each peg centre (peg r 1.0 + TPU ligament 1.2 + C).
# y <= 7.3 keeps the hole (+C) inside the segment's finger face (y 7.5), so the plate's exposed hinge strip is untouched.
# wrist_index / wrist_pinky: both plate pegs sit 3.9 mm apart in the bar (no room between them), but these two segments
# are already solid through the layer on their outer sides (the plate spans only x 6..56), so they get no bar pillar.
WRIST_BARS = {
    'wrist_middle': (19.06, 26.84, 3.0, 7.3),
    'wrist_ring':   (35.16, 43.33, 3.0, 7.3),
}

# ---- helpers ----------------------------------------------------------------------------------------------------
_cache = {}

def _placements():
    if 'pl' in _cache:
        return _cache['pl']
    d = json.load(open(os.path.join(ROOT, 'data/top_assembly_definition.json')))
    ra = d['rootAssembly']
    inst = {i['id']: i for i in ra['instances']}
    for s in d['subAssemblies']:
        for i in s['instances']:
            inst[i['id']] = i
    pl = {}
    for o in ra['occurrences']:
        nm = inst[o['path'][-1]]['name']
        if nm in pl:
            continue                       # palm_bone_flex is duplicated at an identical transform
        t = o['transform']
        pl[nm] = App.Placement(App.Matrix(t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000,
                                          t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))
    _cache['pl'] = pl
    return pl

def _rel(src_inst, dst_inst):
    """Matrix mapping src-instance-local coordinates into dst-instance-local coordinates."""
    pl = _placements()
    return pl[dst_inst].inverse().multiply(pl[src_inst]).toMatrix()

def _moved(shape, m):
    s = shape.copy()
    s.transformShape(m)
    return s

def _tpu_source_shapes():
    """Original (V5) TPU shapes by key, from the open document that holds the V5 sources."""
    if 'tpu' in _cache:
        return _cache['tpu']
    doc = None
    for d in App.listDocuments().values():
        if d.getObject('SRC_Palm1') is not None:
            doc = d
            break
    if doc is None:
        raise RuntimeError('mod_pillars: no open document with the V5 sources (SRC_Palm1)')
    _cache['tpu'] = {k: doc.getObject(n).Shape.copy() for k, n in TPU_SRC.items()}
    return _cache['tpu']

def _plane_face(axis, val, big=400.0):
    if axis == 'y':
        pts = [V(-big, val, -big), V(big, val, -big), V(big, val, big), V(-big, val, big)]
    else:
        pts = [V(-big, -big, val), V(big, -big, val), V(big, big, val), V(-big, big, val)]
    return Part.Face(Part.makePolygon(pts + [pts[0]]))

def _section(shape, axis, val):
    """Planar section (faces) of a solid."""
    return shape.common(_plane_face(axis, val))

def _offset_faces(faces, d):
    out = []
    for f in faces.Faces:
        out.append(f.makeOffset2D(d, 0, False, False, False))
    s = out[0]
    if len(out) > 1:
        s = s.fuse(out[1:])
    return s

def stadium_face(cx, cz, wx, lz, grow=0.0, y=0.0):
    """Stadium in the plane y=const (bone frame): extent wx along x, lz along z, round ends on the short axis."""
    if wx > lz:                                   # long along x: round ends at +-x
        r = lz / 2.0 + grow
        h = wx / 2.0 - lz / 2.0
        p1, p2 = V(cx - h, y, cz - r), V(cx + h, y, cz - r)
        p3, p4 = V(cx + h, y, cz + r), V(cx - h, y, cz + r)
        e = [Part.LineSegment(p1, p2).toShape(), Part.Arc(p2, V(cx + h + r, y, cz), p3).toShape(),
             Part.LineSegment(p3, p4).toShape(), Part.Arc(p4, V(cx - h - r, y, cz), p1).toShape()]
        return Part.Face(Part.Wire(e))
    r = wx / 2.0 + grow
    h = max(lz / 2.0 - wx / 2.0, 0.0)
    if h < 1e-6:
        return Part.Face(Part.Wire(Part.makeCircle(r, V(cx, y, cz), V(0, 1, 0))))
    p1, p2 = V(cx - r, y, cz - h), V(cx - r, y, cz + h)
    p3, p4 = V(cx + r, y, cz + h), V(cx + r, y, cz - h)
    e = [Part.LineSegment(p1, p2).toShape(),
         Part.Arc(p2, V(cx, y, cz + h + r), p3).toShape(),
         Part.LineSegment(p3, p4).toShape(),
         Part.Arc(p4, V(cx, y, cz - h - r), p1).toShape()]
    return Part.Face(Part.Wire(e))

def pillar_face(spec, grow=0.0, y=0.0):
    """Footprint face of one PILLARS entry in the plane y=const (bone frame), optionally grown by `grow`."""
    if spec[0] == 's':
        return stadium_face(*spec[1:5], grow=grow, y=y)
    pts = [V(x, y, z) for (x, z) in spec[1]]
    f = Part.Face(Part.makePolygon(pts + [pts[0]]))
    if grow:
        f = f.makeOffset2D(grow, 0, False, False, False)
    return f

def bone_pillar_solids(key, grow=0.0, y0=BY0 - EPS, y1=BY1 + EPS):
    return [pillar_face(sp, grow, y0).extrude(V(0, y1 - y0, 0)) for sp in PILLARS.get(key, [])]

def _layer_footprint(s, axis, lo, hi, n=3):
    """Union of planar sections of s over [lo, hi], projected to the mid-plane (the TPU sheet is not
    perfectly prismatic: the plate end edges vary ~0.08 over the 0.6 thickness, which a single mid-plane
    section misses; the fattest profile governs the clearance)."""
    mid = 0.5 * (lo + hi)
    faces = []
    for i in range(n):
        v = lo + (hi - lo) * (i + 0.5) / n
        sec = _section(s, axis, v)
        if not sec.Faces:
            continue
        d = mid - v
        sec.translate(V(0, d, 0) if axis == 'y' else V(0, 0, d))
        faces.extend(sec.Faces)
    if not faces:
        return None
    return faces[0].fuse(faces[1:]) if len(faces) > 1 else faces[0]

def _near(a, b, m=1.0):
    return not (a.XMax < b.XMin - m or a.XMin > b.XMax + m or a.YMax < b.YMin - m or a.YMin > b.YMax + m
                or a.ZMax < b.ZMin - m or a.ZMin > b.ZMax + m)

def _bone_pocket(key, host_bb):
    """offset(TPU footprint, C) x TPU layer, in the bone's local frame: one prism per TPU instance near the bone."""
    tpu = _tpu_source_shapes()
    prisms = []
    for tk, insts in TPU_INST.items():
        for inst in insts:
            s = _moved(tpu[tk], _rel(inst, BONE_INST[key]))
            if not _near(s.BoundBox, host_bb):
                continue
            foot = _layer_footprint(s, 'y', BY0 + 0.02, BY1 - 0.02, n=3)
            if foot is None or not foot.Faces:
                continue
            off = _offset_faces(foot, C)
            off.translate(V(0, BY0 - 0.5 * (BY0 + BY1), 0))
            prisms.append(off.extrude(V(0, BY1 - BY0, 0)))
    return prisms

def _wrist_pocket():
    tpu = _tpu_source_shapes()
    ref = WRIST_INST['wrist_index']
    s = _moved(tpu['tpu_wrist'], _rel('palm_bone_flex <1>', ref))
    foot = _layer_footprint(s, 'z', WZ0 + 0.02, WZ1 - 0.02, n=3)
    off = _offset_faces(foot, C)
    off.translate(V(0, 0, WZ0 - 0.5 * (WZ0 + WZ1)))
    return off.extrude(V(0, 0, WZ1 - WZ0))

def _clean(s):
    s = s.removeSplitter()
    if not s.isValid():
        s.fix(1e-7, 1e-7, 1e-7)
    if len(s.Solids) == 1:
        s = s.Solids[0]
    return s

# ---- wrist bar pillars ----------------------------------------------------------------------------------------------
def wrist_bar_face(key, seg_shape):
    """Bar pillar footprint (face in the plane z=WZ0-EPS, wrist frame): where both halves have material at the layer,
    clipped to the bar window of WRIST_BARS."""
    xa, xb, ya, yb = WRIST_BARS[key]
    xa, xb = min(xa, xb), max(xa, xb)
    A = _section(seg_shape, 'z', WZ1 + EPS)        # palm half just above the layer
    B = _section(seg_shape, 'z', WZ0 - EPS)        # back half just below the layer
    A.translate(V(0, 0, (WZ0 - EPS) - (WZ1 + EPS)))
    AB = A.common(B)
    rect = Part.Face(Part.makePolygon([V(xa, ya, WZ0 - EPS), V(xb, ya, WZ0 - EPS), V(xb, yb, WZ0 - EPS),
                                       V(xa, yb, WZ0 - EPS), V(xa, ya, WZ0 - EPS)]))
    f = AB.common(rect)
    faces = [x for x in f.Faces if x.Area > 0.5]
    if not faces:
        raise ValueError('mod_pillars: empty wrist bar pillar for %s' % key)
    return max(faces, key=lambda x: x.Area)

def wrist_pillar_solids(key, seg_shape, grow=0.0):
    f = wrist_bar_face(key, seg_shape)
    if grow:
        f = f.makeOffset2D(grow, 0, False, False, False)
    return [f.extrude(V(0, 0, (WZ1 + EPS) - (WZ0 - EPS)))]

# V5 source object of every wrist segment (= partkeys.resolve at build time; fixed here so that the module also works
# on a built document, where the wrist links already point at V6_* objects and partkeys.resolve cannot find them)
WRIST_SRC = {'wrist_index': 'SRC_Palm_bone1', 'wrist_middle': 'SRC_Palm_bone003',
             'wrist_ring': 'SRC_Palm_bone001', 'wrist_pinky': 'SRC_Palm_bone002'}

def _wrist_seg_sources():
    if 'wseg' in _cache:
        return _cache['wseg']
    doc = None
    for d in App.listDocuments().values():
        if d.getObject('SRC_Palm1') is not None:
            doc = d
            break
    if doc is None:
        raise RuntimeError('mod_pillars: no open document with the V5 sources (SRC_Palm1)')
    _cache['wseg'] = {k: doc.getObject(n).Shape.copy() for k, n in WRIST_SRC.items()}
    return _cache['wseg']

# ---- TPU holes ---------------------------------------------------------------------------------------------------
def _hole_tools_world():
    """All pillar holes (pillar footprint + C) as solids in world coordinates, spanning the layer with margin."""
    if 'holes' in _cache:
        return _cache['holes']
    pl = _placements()
    tools = []
    for key in PILLARS:
        for s in bone_pillar_solids(key, grow=C, y0=BY0 - 0.3, y1=BY1 + 0.3):
            tools.append((key, _moved(s, pl[BONE_INST[key]].toMatrix())))
    segs = _wrist_seg_sources()
    for key in WRIST_BARS:
        f = wrist_bar_face(key, segs[key]).makeOffset2D(C, 0, False, False, False)
        f.translate(V(0, 0, (WZ0 - 0.3) - (WZ0 - EPS)))
        s = f.extrude(V(0, 0, (WZ1 - WZ0) + 0.6))
        tools.append((key, _moved(s, pl[WRIST_INST[key]].toMatrix())))
    _cache['holes'] = tools
    return tools

def _peg_tip(shape, key, c, face_lo, probe_r=1.02):
    """True free-end coordinate of a peg measured from material (the r=1 cylindrical face can stop at a
    tapered neck: the two plate-end pegs of palm_bone_flex taper ~1.2 mm below their cylinder faces).
    face_lo = the face's free-end coordinate (min y for tpu_wrist, min z otherwise)."""
    axis = V(0, 1, 0) if key == 'tpu_wrist' else V(0, 0, 1)
    base = V(c.x, face_lo, c.z) if key == 'tpu_wrist' else V(c.x, c.y, face_lo)

    def mat(t):
        """material within probe_r of the axis, in the 0.02 slab [face_lo + t, face_lo + t + 0.02] (t <= 0)."""
        d = Part.makeCylinder(probe_r, 0.02, base + axis * t, axis)
        return shape.common(d).Volume > 1e-9

    if not mat(-0.03):                  # nothing beyond the face end: plain peg, tip = face end
        return face_lo
    k = 1                               # tapered peg: coarse scan down, then bisect (offsets, negative = beyond)
    while k < 28 and mat(-0.25 * k):
        k += 1
    lo, hi = -0.25 * (k - 1), -0.25 * k  # mat(lo) True, mat(hi) False
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        if mat(mid):
            lo = mid
        else:
            hi = mid
    return face_lo + lo

def _trim_pegs(key, shape):
    """Shorten every r=1 peg by PEG_TRIM at its free end."""
    cuts = []
    for f in shape.Faces:
        srf = f.Surface
        if srf.TypeId != 'Part::GeomCylinder' or abs(srf.Radius - 1.0) > 1e-3:
            continue
        # peg tip = the end of the peg farthest from the sheet (sheet at local z 0..0.6 or y -0.6..0)
        pts = [v.Point for v in f.Vertexes]
        if not pts:
            continue
        c = srf.Center
        if key == 'tpu_wrist':
            tip = _peg_tip(shape, key, c, min(p.y for p in pts))
            cuts.append(Part.makeCylinder(1.1, PEG_TRIM + 0.1, V(c.x, tip - 0.1, c.z), V(0, 1, 0)))
        else:
            tip = _peg_tip(shape, key, c, min(p.z for p in pts))
            cuts.append(Part.makeCylinder(1.1, PEG_TRIM + 0.1, V(c.x, c.y, tip - 0.1), V(0, 0, 1)))
    if not cuts:
        return shape
    return shape.cut(cuts)

def tpu_part(key, shape):
    insts = TPU_INST[key]
    pl = _placements()
    s = shape
    for inst in insts:
        inv = pl[inst].inverse().toMatrix()
        mine = []
        for owner, tw in _hole_tools_world():
            t = _moved(tw, inv)
            if t.BoundBox.intersect(s.BoundBox) and t.common(s).Volume > 1e-6:
                mine.append(t)
        if mine:
            if key == 'tpu_web':
                raise ValueError('mod_pillars: a pillar hole reaches a web instance (%s); webs are shared by 3 instances' % inst)
            s = s.cut(mine)
    s = _trim_pegs(key, s)
    if s is shape:
        return shape                   # nothing to do (webs: no holes, no pegs)
    return _clean(s)

# ---- peg-hole reaming ----------------------------------------------------------------------------------------------
def _peg_axes(host_lbl, near):
    """Axes (point on axis) of the r=1 TPU pegs of every TPU instance, in the host's local frame."""
    tpu = _tpu_source_shapes()
    out = []
    for tk, insts in TPU_INST.items():
        for inst in insts:
            s = _moved(tpu[tk], _rel(inst, host_lbl))
            bb = s.BoundBox
            if (bb.XMax < near.XMin - 1 or bb.XMin > near.XMax + 1 or bb.YMax < near.YMin - 1
                    or bb.YMin > near.YMax + 1 or bb.ZMax < near.ZMin - 1 or bb.ZMin > near.ZMax + 1):
                continue
            for f in s.Faces:
                srf = f.Surface
                if srf.TypeId == 'Part::GeomCylinder' and abs(srf.Radius - 1.0) < 1e-3:
                    out.append(V(srf.Center))
    return out

def ream_peg_holes(key, shape, normal):
    """The V5 peg holes (r 1.25) are up to ~0.08 mm off the axis of the pegs that enter them, which eats
    the 0.25 mm nominal wall down to ~0.17. Ream every peg hole to r = 1.0 + C + 0.01 about the PEG's own
    axis (the hole only grows on its close side, by the eccentricity), keeping >= 0.2 mm all around."""
    host_lbl = BONE_INST.get(key) or WRIST_INST[key]
    pegs = _peg_axes(host_lbl, shape.BoundBox)
    cuts = []
    for f in shape.Faces:
        srf = f.Surface
        if srf.TypeId != 'Part::GeomCylinder' or abs(srf.Radius - 1.25) > 0.02:
            continue
        if abs(abs(V(srf.Axis).normalize().dot(normal)) - 1.0) > 1e-6:
            continue
        c = V(srf.Center)
        best, bd = None, 1e9
        for p in pegs:
            d = (p - c) - normal * (p - c).dot(normal)      # both axes are parallel to normal
            if d.Length < bd:
                best, bd = p, d.Length
        if best is None or bd > 0.5:
            continue
        bb = f.BoundBox                  # not the vertices: a hole face can end in a vertex-free curve (wrist pegs)
        ts = [(V(x, y, z) - c).dot(normal) for x in (bb.XMin, bb.XMax) for y in (bb.YMin, bb.YMax) for z in (bb.ZMin, bb.ZMax)]
        t0, t1 = min(ts), max(ts)
        cuts.append(Part.makeCylinder(1.01 + C, (t1 + 0.01) - (t0 - 0.05), best + normal * (t0 - 0.05), normal))
    if not cuts:
        return shape
    return shape.cut(cuts)

# ---- modify ----------------------------------------------------------------------------------------------------------
def palm_bone(key, shape):
    s = shape.cut(_bone_pocket(key, shape.BoundBox))
    pil = bone_pillar_solids(key)
    if pil:
        s = s.fuse(pil)
    s = ream_peg_holes(key, s, V(0, 1, 0))
    return _clean(s)

def wrist_segment(key, shape):
    s = shape.cut(_wrist_pocket())
    if key in WRIST_BARS:
        s = s.fuse(wrist_pillar_solids(key, _wrist_seg_sources()[key]))
    s = ream_peg_holes(key, s, V(0, 0, 1))
    return _clean(s)

def modify(key, shape):
    if key in BONE_INST:
        return palm_bone(key, shape)
    if key in WRIST_INST:
        return wrist_segment(key, shape)
    if key in TPU_INST:
        return tpu_part(key, shape)
    return shape
