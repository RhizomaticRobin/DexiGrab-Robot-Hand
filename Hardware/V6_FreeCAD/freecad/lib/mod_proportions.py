"""V6.1 proportions (topic 'proportions'): human-like phalanx lengths, spliced into the finished finger bones.

User decision (2026-09-26): re-proportion the finger phalanges to the user's own hand, measured with the Leap Motion
(~/leap/reports/hand_size_vs_robot.json, bones x 1.25).  Palm, finger pitch and bone thickness stay as they are.
Joint-to-joint targets (mm): index 43.6 / 29.5 / 25.0, middle 50.6 / 35.6 / 24.5, ring 49.5 / 27.8 / 21.5,
pinky 32.9 / 25.0 / 21.0 (Leap distal 17.4 clamped to 21 for the M1.6 anchor + fingertip pad, middle phalanx grown
from 24.6 to 25.0), thumb: metacarpal unchanged, proximal (R5-R6) 33.0, distal (R6-tip) 22.0.

How.  This module runs after every module that shapes the bones (actuation: tendon channels, knot pockets, distal M1.6
anchor, open-roof flexor slots; tactile: pad recesses, folds, side curtains, dorsal spine channels; thumb: M2 bolts).
modify() captures the shared bone and phalanx-pad shapes, new_parts() makes one variant per finger by SPLICING a
uniform prismatic mid-section, in the part's own frame:
  metacarpal (lengthen): cut at x_s, move the far part +d along x, fill the gap with the section at x_s extruded by d
  proximal   (shorten):  remove the slab [a, b], move the far part back by b - a
  distal     (shorten):  remove slab(s) between the tip clamp and the J3 clevis, move the TIP portion +L along x
Each bone keeps its proximal-side joint frame (metacarpal J1 at x 7.5, proximal J2 at x 7.5, distal J3 connector at
x 29.3); only the far joint / the tip moves.  Splice planes are chosen at build time inside the x-uniform ranges of the
RECEIVED shape (faces that do not contain the x direction bound them) intersected with a semantic window (behind the
knot pocket, in front of the far clevis, behind the pad's loop anchor, ...), and the section just before / after every
splice is compared (PROOF, written to work/proportions/splice_proof.json).  A plane that no longer fits raises.
The phalanx pads (tactile) are spliced the same way (same slab when it fits the pad, else the pad's own uniform range)
so pad, recess, fold, curtain and spine stay matched, and each spine sheet still ends at its J2 / J3 loop anchor.

Instances.  relink(): every finger / thumb bone and phalanx-pad instance -> its variant (unconditional, so it survives
build_v6.write_manifest()'s module reload).  placements(): every downstream bone, pad and J2 / J3 loop is shifted along
its finger by the cumulative joint shift (pure translations: joint angles unchanged; the thumb distal composes on
mod_thumb's re-clocked rest).  Moved part-local joint connectors: data/v6_joints_v61.json "frames"
(write_joint_frames(); the orchestrator merges them into data/v6_joints.json when it loads V6.1 into the master, so
geometry and frames change together; this module never writes the shared file).
World-frame fastener requests seated on a moved bone (the thumb R6 bolt): move_world_request(), called by mod_hardware.
Notes: freecad/notes/proportions.md.  Checks: scripts/verify_proportions.py.
"""
import os, sys, json, math, time
import FreeCAD as App, Part

LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
if LIB not in sys.path:
    sys.path.insert(0, LIB)
from v6geom import box

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
FC = ROOT + '/freecad'
WORK = FC + '/work/proportions'
JOINTS_JSON = ROOT + '/data/v6_joints.json'       # shared, live (rom_sweep, Leap exporter): NOT written by this module
JOINTS_V61 = ROOT + '/data/v6_joints_v61.json'    # this module's frame overrides; the orchestrator merges them into
                                                  # v6_joints.json when V6.1 is loaded into the master (same format)
ORDER = ['j0', 'pillars', 'thumb', 'actuation', 'shells', 'electronics', 'fixes', 'tactile', 'proportions', 'hardware']

# ------------------------------------------------------------------------------------------------ targets
# joint-to-joint (distal: J3 axis to fingertip), mm.  None = unchanged.
TARGET = {
    'index':  dict(meta=43.6, prox=29.5, dist=25.0),
    'middle': dict(meta=50.6, prox=35.6, dist=24.5),
    'ring':   dict(meta=49.5, prox=27.8, dist=21.5),
    'pinky':  dict(meta=32.9, prox=25.0, dist=21.0),
    'thumb':  dict(meta=None, prox=33.0, dist=22.0),
}
LEAP = {    # the user's bones (Leap Hyperion medians, mm) x 1.25, for the notes
    'index': (34.9, 23.6, 20.0), 'middle': (40.5, 28.5, 19.6), 'ring': (39.6, 22.2, 17.2), 'pinky': (26.3, 19.7, 13.9),
    'thumb': (47.8, 26.4, 17.4)}
SCALE = 1.25
MIN_MID, MIN_DIST = 22.0, 20.0          # middle phalanx joint-to-joint, distal J3-to-tip (M1.6 anchor + pad)

# finger chains: (shared source part key, instance label) per segment
CHAIN = {
    'index':  dict(meta=('metacarpal', 'Metacarpal Bone_V02 <1>'), prox=('proximal', 'Proximal Phalanx Bone_V02 <1>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <1>')),
    'middle': dict(meta=('metacarpal', 'Metacarpal Bone_V02 <2>'),
                   prox=('proximal_middle', 'Proximal Phalanx Bone_V02_middlefinger <1>'), dist=('distal', 'Distal Phalanx Bone_V02 <2>')),
    'ring':   dict(meta=('metacarpal', 'Metacarpal Bone_V02 <3>'), prox=('proximal', 'Proximal Phalanx Bone_V02 <2>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <3>')),
    'pinky':  dict(meta=('metacarpal_pinky', 'Metacarpal Bone_V02_pinky <1>'),
                   prox=('proximal_pinky', 'Proximal Phalanx Bone_V02_pinky <1>'), dist=('distal', 'Distal Phalanx Bone_V02 <4>')),
    'thumb':  dict(meta=None, prox=('thumb_proximal', 'Proximal Phalanx Bone_V02 <3>'), dist=('thumb_distal', 'Distal Phalanx Bone_V02 <5>')),
}
FINGERS = ('index', 'middle', 'ring', 'pinky', 'thumb')
SEGS = ('meta', 'prox', 'dist')
SEG_NAME = {'meta': 'metacarpal', 'prox': 'proximal', 'dist': 'distal'}
# phalanx pad part of each shared bone key (mod_tactile: 'tactile_pad_' + key)
PAD_OF = {k: 'tactile_pad_' + k for k in ('metacarpal', 'metacarpal_pinky', 'proximal', 'proximal_middle', 'proximal_pinky',
                                          'thumb_proximal', 'distal', 'thumb_distal')}
# part-local joint x of the shared parts (data/mates.csv connectors; all on the local x axis)
JX = {'metacarpal': (7.5, 39.5), 'metacarpal_pinky': (7.5, 32.5),
      'proximal': (7.5, 49.2), 'proximal_middle': (7.5, 57.2), 'proximal_pinky': (7.5, 39.2), 'thumb_proximal': (7.5, 49.2),
      'distal': (29.3, None), 'thumb_distal': (29.3, None)}      # distal: J3 connector; the tip is measured on the shape
DIST_TIP_X = 0.0              # fingertip of the V5 distal (local x min, optimal bounding box; re-measured at build time)
# features that bound the splice windows (other modules' constants, part-local x)
KNOT_END = 19.0 + 1.5         # actuation knot pocket (dia 3 at x 19) in metacarpal / proximal
FLEX_SLOT = 7.7               # actuation open-roof flexor slot / clevis floor: j -/+ 7.7
NUT_SLOT_END = 8.33           # distal M1.6 nut pocket + side slot (x 6.53..8.33)
CLEVIS_BACK = 7.9             # distal J3 clevis slot back wall 7.9 from the axis (x 21.4)
KNUCKLE_EXIT = 17.5           # tactile: metacarpal spine sheet starts here (knuckle ribbon exit)
WRAP_TIP = 8.2                # tactile: distal pad wrap starts (tip centre 7.5 + 0.5 + C)
MARGIN = 0.25                 # splice planes stay this far inside a uniform range
NEAR_X_DEG = 0.5              # cylinders within this angle of x count as prismatic (V5 source tilts) - reported

def tactile_anchor_offsets():
    """(J2-loop anchor on the metacarpal: j2 + a2, J3-loop anchor on the distal: j3 - b3) offsets, from mod_tactile."""
    try:
        import mod_tactile
        a3, b3, a2, b2 = mod_tactile._anchors()
        return a2, b3
    except Exception:
        return -12.0645, 12.2145

def active_modules():
    env = [m for m in os.environ.get('V6_MODULES', '').split(',') if m]
    return env or ORDER

# ------------------------------------------------------------------------------------------------ assembly data
_DEF = None
def _definition():
    global _DEF
    if _DEF is None:
        d = json.load(open(ROOT + '/data/top_assembly_definition.json'))
        ra = d['rootAssembly']
        inst = {i['id']: i for i in ra['instances']}
        for s in d['subAssemblies']:
            for i in s['instances']:
                inst[i['id']] = i
        occ = {}
        for o in ra['occurrences']:
            t = o['transform']
            occ[inst[o['path'][-1]]['name']] = App.Placement(App.Matrix(
                t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000, t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))
        mates = {}
        for f in ra['features']:
            fd = f['featureData']
            if f['featureType'] != 'mate' or fd.get('mateType') != 'REVOLUTE':
                continue
            ents = []
            for e in fd['matedEntities']:
                cs = e['matedCS']
                ents.append((inst[e['matedOccurrence'][-1]]['name'], [c * 1000 for c in cs['origin']]))
            mates[fd['name']] = ents
        _DEF = dict(occ=occ, mates=mates)
    return _DEF

def mate_between(a, b):
    """(mate name, {label: part-local origin}) of the revolute mate joining instances a and b"""
    for n, ents in _definition()['mates'].items():
        labs = {e[0] for e in ents}
        if labs == {a, b}:
            return n, {lab: org for lab, org in ents}
    raise KeyError('no revolute mate between %s and %s' % (a, b))

_TREST = None
def _thumb_rest():
    """mod_thumb.placements() once per module load (build_v6 reloads every module at the start of a build)"""
    global _TREST
    if _TREST is None:
        _TREST = {}
        if 'thumb' in active_modules():
            try:
                import mod_thumb
                _TREST = {k: App.Placement(v) for k, v in mod_thumb.placements().items()}
            except Exception as e:
                print('[proportions] mod_thumb rest unavailable (%s): V5 CAD pose used for the thumb' % e)
    return _TREST

def rest_placement(label):
    """current rest pose of an instance: the thumb flex stack at mod_thumb's re-clocked rest, else the V5 CAD pose"""
    tr = _thumb_rest()
    if label in tr:
        return App.Placement(tr[label])
    return App.Placement(_definition()['occ'][label])

# ------------------------------------------------------------------------------------------------ plan (lengths)
def joint_span(finger, seg):
    """current joint-to-joint length of a segment from the mate connectors (metacarpal J1-J2, proximal J2-J3)"""
    c = CHAIN[finger]
    if seg == 'meta':
        base = 'Base Bone 1_V02 <%s>' % {'index': 1, 'middle': 2, 'ring': 4, 'pinky': 3}[finger]
        _, o1 = mate_between(c['meta'][1], base)
        _, o2 = mate_between(c['meta'][1], c['prox'][1])
        return o2[c['meta'][1]][0] - o1[c['meta'][1]][0]
    if seg == 'prox':
        if finger == 'thumb':
            _, o1 = mate_between(c['prox'][1], 'Metacarpal Bone_V02 <4>')
        else:
            _, o1 = mate_between(c['prox'][1], c['meta'][1])
        _, o2 = mate_between(c['prox'][1], c['dist'][1])
        return o2[c['prox'][1]][0] - o1[c['prox'][1]][0]
    raise ValueError(seg)

def plan(tip_x=None):
    """per finger: {'meta': d (insert, mm), 'prox': d (<0 remove), 'dist': L (removed), before/after lengths}"""
    tip_x = DIST_TIP_X if tip_x is None else tip_x
    out = {}
    for f in FINGERS:
        t = TARGET[f]
        p = {}
        if t['meta'] is not None:
            now = joint_span(f, 'meta')
            p['meta'] = dict(now=round(now, 4), target=t['meta'], d=round(t['meta'] - now, 4))
        now = joint_span(f, 'prox')
        p['prox'] = dict(now=round(now, 4), target=t['prox'], d=round(t['prox'] - now, 4))
        j3 = JX[CHAIN[f]['dist'][0]][0]
        now = j3 - tip_x
        p['dist'] = dict(now=round(now, 4), target=t['dist'], d=round(t['dist'] - now, 4))
        if t['prox'] < MIN_MID or t['dist'] < MIN_DIST:
            raise ValueError('%s: target below the minimum (middle >= %.0f, distal >= %.0f)' % (f, MIN_MID, MIN_DIST))
        out[f] = p
    return out

def shifts(f, pl=None):
    """world translations (t2: J2 shift = proximal / J2-loop / proximal pad, t23: J3 shift = distal, its pad, J3 loop)"""
    pl = pl or plan()
    c = CHAIN[f]
    t2 = V(0, 0, 0)
    if c['meta'] is not None and 'meta' in pl[f]:
        t2 = rest_placement(c['meta'][1]).Rotation.multVec(V(pl[f]['meta']['d'], 0, 0))
    t3 = rest_placement(c['prox'][1]).Rotation.multVec(V(pl[f]['prox']['d'], 0, 0))
    return t2, t2 + t3

# ------------------------------------------------------------------------------------------------ uniform ranges
def _prismatic(f, near_deg=NEAR_X_DEG):
    """(prismatic, tilt_deg) of a face along x"""
    s = f.Surface
    t = s.TypeId
    if t == 'Part::GeomPlane':
        return abs(s.Axis.x) < 1e-7, 0.0
    if t in ('Part::GeomCylinder', 'Part::GeomSurfaceOfExtrusion'):
        a = V(s.Axis) if t == 'Part::GeomCylinder' else V(s.Direction)
        a.normalize()
        tilt = math.degrees(math.acos(min(1.0, abs(a.x))))
        if tilt < 1e-5:
            return True, 0.0
        return tilt < near_deg, tilt
    return False, None

def uniform_ranges(shape, lo=-1e9, hi=1e9):
    """x ranges (open) where every section of shape is the same (up to near-x source tilts, listed in 'tilted')"""
    blocks, tilted = [], []
    for f in shape.Faces:
        ok, tilt = _prismatic(f)
        fb = f.BoundBox
        if not ok:
            blocks.append((fb.XMin, fb.XMax))
        elif tilt:
            tilted.append(dict(x=[round(fb.XMin, 3), round(fb.XMax, 3)], tilt_deg=round(tilt, 4),
                               radius=round(getattr(f.Surface, 'Radius', 0.0), 3)))
    blocks.sort()
    merged = []
    for a, b in blocks:
        if merged and a <= merged[-1][1] + 1e-6:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    bb = shape.BoundBox
    out, prev = [], bb.XMin
    for a, b in merged:
        if a > prev:
            out.append((prev, a))
        prev = max(prev, b)
    if bb.XMax > prev:
        out.append((prev, bb.XMax))
    res = []
    for a, b in out:
        a2, b2 = max(a, lo), min(b, hi)
        if b2 - a2 > 2 * MARGIN:
            res.append((a2, b2))
    return res, tilted

# ------------------------------------------------------------------------------------------------ splicing
BIG = 400.0

def _half(shape, x, side):
    tool = box((-BIG, -BIG, -BIG), (x, BIG, BIG)) if side < 0 else box((x, -BIG, -BIG), (BIG, BIG, BIG))
    return shape.common(tool)

def section(shape, x):
    wires = shape.slice(V(1, 0, 0), x)
    if not wires:
        raise ValueError('empty section at x %.3f' % x)
    return Part.makeFace(wires, 'Part::FaceMakerBullseye')

UNREFINED = set()  # parts whose fused result could not be refined (the received shape cannot be either, e.g. distal)

def _single(s, what):
    try:
        s = s.removeSplitter()
    except Exception:          # OCC UnifySameDomain fails on some received shapes (the V5 distal's sphere-capped tip
        UNREFINED.add(what)    # corners fail it before any splice); the fused solid is kept as it is (valid, split faces)
    if not s.isValid():
        s.fix(1e-7, 1e-7, 1e-7)
    if len(s.Solids) != 1:
        raise RuntimeError('%s: splice gave %d solids' % (what, len(s.Solids)))
    s = s.Solids[0]
    if not s.isValid():
        raise RuntimeError('%s: splice result is not valid' % what)
    return s

def splice(shape, ops, what):
    """ops, all in the ORIGINAL part coordinates: ('insert', x, d) or ('remove', a, b, move) with move 'far' (x > b
    moves back) or 'tip' (x < a moves forward).  Applied so that the coordinates of the ops still to come do not move:
    far-moving ops from the far end, tip-moving ops from the tip end."""
    s = shape
    tip = any(o[0] == 'remove' and o[3] == 'tip' for o in ops)
    for op in sorted(ops, key=lambda o: o[1] if tip else -o[1]):
        if op[0] == 'insert':
            _, x, d = op
            left, right = _half(s, x, -1), _half(s, x, +1)
            right.translate(V(d, 0, 0))
            slab = section(s, x).extrude(V(d, 0, 0))
            s = _single(left.fuse([slab, right]), what)
        else:
            _, a, b, move = op
            left, right = _half(s, a, -1), _half(s, b, +1)
            if move == 'far':
                right.translate(V(-(b - a), 0, 0))
            else:
                left.translate(V(b - a, 0, 0))
            s = _single(left.fuse(right), what)
    return s

def _symdiff(fa, fb):
    """area of the symmetric difference of two parallel section faces (fb moved into fa's plane)"""
    g = fb.copy()
    g.translate(V(fa.BoundBox.XMin - fb.BoundBox.XMin, 0, 0))
    return fa.cut(g).Area + g.cut(fa).Area

def prove(shape, ops, what, eps=0.05):
    """section comparison at every splice: the faces that meet (or the section that is extruded) must be identical,
    and a removed slab must be uniform (sections every <= 1 mm inside it equal the one before it)"""
    rows = []
    for op in ops:
        if op[0] == 'insert':
            x = op[1]
            f0, f1 = section(shape, x - eps), section(shape, x + eps)
            rows.append(dict(op='insert', x=round(x, 4), d=round(op[2], 4), compare=[round(x - eps, 4), round(x + eps, 4)],
                             area=round(f0.Area, 4), symdiff=round(_symdiff(f0, f1), 5),
                             edges=[len(f0.Edges), len(f1.Edges)]))
        else:
            a, b = op[1], op[2]
            f0, f1 = section(shape, a - eps), section(shape, b + eps)
            n = max(2, int(math.ceil((b - a) / 1.0)))
            inner = [a + (b - a) * k / n for k in range(n + 1)]
            inner[0] += eps; inner[-1] -= eps
            worst = max(_symdiff(f0, section(shape, x)) for x in inner)
            rows.append(dict(op='remove', a=round(a, 4), b=round(b, 4), L=round(b - a, 4), move=op[3],
                             compare=[round(a - eps, 4), round(b + eps, 4)], area=round(f0.Area, 4),
                             symdiff=round(_symdiff(f0, f1), 5), inside_max_symdiff=round(worst, 5), inside_n=len(inner),
                             edges=[len(f0.Edges), len(f1.Edges)]))
    return rows

def choose_insert(shape, window):
    """splice plane for an insertion: middle of the largest uniform range inside window; returns (x, ranges, tilted)"""
    rng, tilted = uniform_ranges(shape, *window)
    if not rng:
        raise RuntimeError('no x-uniform range inside %s' % (window,))
    a, b = max(rng, key=lambda r: r[1] - r[0])
    return 0.5 * (a + b), rng, tilted

def choose_remove(shape, window, L, move):
    """removal slabs (largest uniform ranges first) inside window, total L"""
    rng, tilted = uniform_ranges(shape, *window)
    rng = sorted(rng, key=lambda r: -(r[1] - r[0]))
    ops, left = [], L
    for a, b in rng:
        if left <= 1e-9:
            break
        cap = (b - a) - 2 * MARGIN
        if cap <= 0.2:
            continue
        take = min(left, cap)
        c = 0.5 * (a + b)
        ops.append(('remove', c - take / 2, c + take / 2, move))
        left -= take
    if left > 1e-6:
        raise RuntimeError('only %.2f of %.2f mm removable inside the uniform ranges %s of window %s'
                           % (L - left, L, [(round(a, 2), round(b, 2)) for a, b in rng], window))
    return ops, rng, tilted

# ------------------------------------------------------------------------------------------------ variants
def bone_key(f, seg):
    return 'proportions_%s_%s' % (SEG_NAME[seg], f)

def pad_key(f, seg):
    return 'proportions_pad_%s_%s' % (SEG_NAME[seg], f)

def pad_instance(f, seg):
    return 'tactile_pad_%s_%s' % (f, seg)

def loop_instance(f, kind):
    return 'tactile_loop_%s_%s' % (f, kind)

def thumb_source(seg):
    """shared key the thumb variant is spliced from (mod_thumb's variant when it ran)"""
    k = CHAIN['thumb'][seg][0]
    return k if k in _SRC else {'prox': 'proximal', 'dist': 'distal'}[seg]

def pad_source(f, seg):
    """tactile pad part the variant's pad is spliced from (the thumb always has its own, mirrored, pads)"""
    if f == 'thumb':
        return 'tactile_pad_thumb_' + SEG_NAME[seg]
    return PAD_OF.get(CHAIN[f][seg][0])

def windows(src_key, seg, pad=False):
    """semantic splice window (part-local x) of a shared bone / its pad"""
    a2, b3 = tactile_anchor_offsets()
    j = JX.get(src_key) or JX[{'thumb_metacarpal': 'metacarpal'}.get(src_key, src_key)]
    if seg == 'meta':
        return (KNUCKLE_EXIT, j[1] + a2) if pad else (KNOT_END, j[1] - FLEX_SLOT)
    if seg == 'prox':
        return (KNOT_END, j[1] - FLEX_SLOT)
    j3 = j[0]
    return (WRAP_TIP, j3 - b3) if pad else (NUT_SLOT_END, j3 - CLEVIS_BACK)

_SRC = {}          # shared key -> received shape (bones and pads)
PROOF = {}         # new key -> splice report
LAST = {}          # new key -> ops actually used

def build_variant(f, seg, pl):
    src = CHAIN[f][seg][0] if f != 'thumb' else thumb_source(seg)
    shape = _SRC[src]
    d = pl[f][seg]['d']
    rep = dict(finger=f, seg=seg, source=src, target=pl[f][seg]['target'], before=pl[f][seg]['now'], delta=d)
    if seg == 'meta':
        x, rng, tilted = choose_insert(shape, windows(src, seg))
        ops = [('insert', x, d)]
    elif seg == 'prox':
        ops, rng, tilted = choose_remove(shape, windows(src, seg), -d, 'far')
    else:
        ops, rng, tilted = choose_remove(shape, windows(src, seg), -d, 'tip')
    rep.update(ranges=[[round(a, 4), round(b, 4)] for a, b in rng], tilted=tilted, ops=[list(o) for o in ops])
    rep['proof'] = prove(shape, ops, bone_key(f, seg))
    new = splice(shape, ops, bone_key(f, seg))
    rep.update(volume_before=round(shape.Volume, 3), volume_after=round(new.Volume, 3),
               volume_expected=round(shape.Volume + sum(r['area'] * (r['d'] if r['op'] == 'insert' else -r['L'])
                                                         for r in rep['proof']), 3),
               bbox_x=[round(new.BoundBox.XMin, 4), round(new.BoundBox.XMax, 4)])
    PROOF[bone_key(f, seg)] = rep
    LAST[bone_key(f, seg)] = ops
    out = {bone_key(f, seg): new}
    # pad: same slab(s) when they lie inside the pad's uniform ranges, else the pad's own range
    pk = pad_source(f, seg)
    if pk and pk in _SRC:
        pshape = _SRC[pk]
        pw = windows(src, seg, pad=True)
        prng, ptilt = uniform_ranges(pshape, *pw)
        def inside(a, b):
            return any(ra + MARGIN - 1e-6 <= a and b <= rb - MARGIN + 1e-6 for ra, rb in prng)
        if seg == 'meta':
            pops = ops if inside(ops[0][1], ops[0][1]) else [('insert', choose_insert(pshape, pw)[0], d)]
        else:
            if all(inside(o[1], o[2]) for o in ops):
                pops = ops
            else:
                pops = choose_remove(pshape, pw, -d, ops[0][3])[0]
        prep = dict(finger=f, seg=seg, source=pk, delta=d, window=[round(v, 4) for v in pw],
                    ranges=[[round(a, 4), round(b, 4)] for a, b in prng], tilted=ptilt, ops=[list(o) for o in pops],
                    same_as_bone=(pops == ops))
        prep['proof'] = prove(pshape, pops, pad_key(f, seg))
        pnew = splice(pshape, pops, pad_key(f, seg))
        prep.update(volume_before=round(pshape.Volume, 3), volume_after=round(pnew.Volume, 3),
                    volume_expected=round(pshape.Volume + sum(r['area'] * (r['d'] if r['op'] == 'insert' else -r['L'])
                                                              for r in prep['proof']), 3))
        PROOF[pad_key(f, seg)] = prep
        LAST[pad_key(f, seg)] = pops
        out[pad_key(f, seg)] = pnew
    return out

# ------------------------------------------------------------------------------------------------ pipeline hooks
CAPTURE = set(JX) | set(PAD_OF.values())

def modify(key, shape):
    if key in CAPTURE:
        _SRC[key] = shape.copy()
    return shape

def new_parts():
    t0 = time.time()
    tip = None
    if 'distal' in _SRC:
        ob = _SRC['distal'].optimalBoundingBox(False, False) if hasattr(_SRC['distal'], 'optimalBoundingBox') else _SRC['distal'].BoundBox
        tip = ob.XMin
    pl = plan(tip)
    out = {}
    PROOF.clear(); LAST.clear()
    for f in FINGERS:
        for seg in SEGS:
            if seg not in pl[f]:
                continue
            src = CHAIN[f][seg][0] if f != 'thumb' else thumb_source(seg)
            if src not in _SRC:
                raise RuntimeError('proportions: shape %s was not captured (module order?)' % src)
            out.update(build_variant(f, seg, pl))
    try:
        os.makedirs(WORK, exist_ok=True)
        path = os.environ.get('PROP_PROOF', WORK + '/splice_proof.json')     # read by scripts/verify_proportions.py
        json.dump(dict(generated=time.strftime('%Y-%m-%d %H:%M:%S'), plan=pl, tip_x=tip, parts=PROOF,
                       seconds=round(time.time() - t0, 1)), open(path, 'w'), indent=1)
    except Exception as e:
        print('[proportions] proof not written: %s' % e)
    print('[proportions] %d variants in %.1fs' % (len(out), time.time() - t0))
    return out

def relink():
    out = {}
    tactile = 'tactile' in active_modules()
    for f in FINGERS:
        for seg in SEGS:
            if CHAIN[f][seg] is None or TARGET[f][seg] is None:
                continue
            out[CHAIN[f][seg][1]] = bone_key(f, seg)
            if tactile:
                out[pad_instance(f, seg)] = pad_key(f, seg)
    return out

def _tactile_specs():
    if 'tactile' not in active_modules():
        return {}
    try:
        import mod_tactile
        return {n: pl for n, p, pl, c, att in mod_tactile.instance_specs()}
    except Exception as e:
        print('[proportions] tactile instance poses unavailable: %s' % e)
        return {}

def moved_instances(pl=None):
    """{instance label: (translation, base placement)} for every re-posed instance"""
    pl = pl or plan()
    specs = _tactile_specs()
    out = {}
    for f in FINGERS:
        c = CHAIN[f]
        t2, t23 = shifts(f, pl)
        if f != 'thumb':
            out[c['prox'][1]] = (t2, rest_placement(c['prox'][1]))
        out[c['dist'][1]] = (t23, rest_placement(c['dist'][1]))
        for name, t in ((pad_instance(f, 'prox'), t2), (loop_instance(f, 'J2'), t2),
                        (pad_instance(f, 'dist'), t23), (loop_instance(f, 'J3'), t23)):
            if name in specs and t.Length > 1e-9:
                out[name] = (t, App.Placement(specs[name]))
    return out

def placements():
    out = {}
    for lab, (t, base) in moved_instances().items():
        out[lab] = App.Placement(t, App.Rotation()).multiply(base)
    return out

# ------------------------------------------------------------------------------------------------ tactile tables
SHARED_PHALANX = ('metacarpal', 'metacarpal_pinky', 'proximal', 'proximal_middle', 'proximal_pinky', 'distal',
                  'thumb_proximal', 'thumb_distal')

def patch_tactile(T):
    """Install the per-finger phalanx pads in mod_tactile's tables, for the fab-data generator and the checks (never
    during a build): one key per variant (= the bone part key), its pad spec / joints / spine width, the rays pointing
    at them, occ() returning the re-posed instance placements and canon_matrix() mapping a shortened distal's pad frame
    (fingertip kept at u 0, as mod_tactile assumes) onto the spliced bone (tip moved +L).  The shared finger keys are
    removed from PHALANX (no instance uses them any more); thumb_metacarpal is unchanged and stays.
    Returns {variant key: shared key it came from}."""
    pl = plan()
    moved = {lab: App.Placement(t, App.Rotation()).multiply(base) for lab, (t, base) in moved_instances(pl).items()}
    orig_occ, orig_canon = T.occ, T.canon_matrix
    shift, origin = {}, {}
    for f in FINGERS:
        for seg in SEGS:
            if CHAIN[f][seg] is None or seg not in pl[f]:
                continue
            src = CHAIN[f][seg][0]
            key = bone_key(f, seg)
            p, J = dict(T.PHALANX[src]), dict(T.BONE_J[src])
            d = pl[f][seg]['d']
            if seg in ('meta', 'prox'):
                jk = 'j2' if seg == 'meta' else 'j3'
                p['u1'] += d
                p['top'] = (p['top'][0], p['top'][1] + d)
                J[jk] += d
            else:
                L = -d
                p['u1'] -= L
                p['top'] = (p['top'][0], p['top'][1] - L)
                J['j3'] -= L
                shift[key] = L
            T.PHALANX[key], T.BONE_J[key], T.SPINE_N[key] = p, J, T.SPINE_N[src]
            T.RAYS[f][seg] = (key, CHAIN[f][seg][1])
            origin[key] = src
    for k in SHARED_PHALANX:
        T.PHALANX.pop(k, None)

    def occ(name):
        return App.Placement(moved[name]) if name in moved else orig_occ(name)

    def canon_matrix(key):
        if key in shift:
            return App.Matrix(1, 0, 0, shift[key], 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
        return orig_canon(key)
    T.occ, T.canon_matrix = occ, canon_matrix
    T._PARTS = None
    return origin

# ------------------------------------------------------------------------------------------------ joint frames
def joint_frames(pl=None):
    """{mate: {'frames': {label: {'origin': [x, y, z]}}}} for every connector whose part-local origin moved"""
    pl = pl or plan()
    out = {}
    for f in FINGERS:
        c = CHAIN[f]
        if c['meta'] is not None and 'meta' in pl[f]:
            name, org = mate_between(c['meta'][1], c['prox'][1])
            o = org[c['meta'][1]]
            out[name] = {'frames': {c['meta'][1]: {'origin': [round(o[0] + pl[f]['meta']['d'], 4), round(o[1], 4), round(o[2], 4)]}},
                         'note': '%s J2: metacarpal J2 connector moved %+.2f mm (V6.1 proportions)' % (f, pl[f]['meta']['d'])}
        name, org = mate_between(c['prox'][1], c['dist'][1])
        o = org[c['prox'][1]]
        out[name] = {'frames': {c['prox'][1]: {'origin': [round(o[0] + pl[f]['prox']['d'], 4), round(o[1], 4), round(o[2], 4)]}},
                     'note': '%s %s: proximal distal-end connector moved %+.2f mm (V6.1 proportions)'
                             % (f, 'R6' if f == 'thumb' else 'J3', pl[f]['prox']['d'])}
    return out

def write_joint_frames(path=JOINTS_V61):
    """write the frames to data/v6_joints_v61.json (same format as v6_joints.json; re-read right before writing, every
    other key and field kept).  The shared data/v6_joints.json is only changed by the orchestrator, together with the
    geometry (frames applied to un-spliced bones would put the pivots in the wrong place)."""
    new = joint_frames()
    cur = json.load(open(path)) if os.path.exists(path) else {}
    tgt = cur['joints'] if isinstance(cur.get('joints'), dict) else cur
    for mate, v in new.items():
        e = tgt.setdefault(mate, {})
        fr = e.setdefault('frames', {})
        fr.update(v['frames'])
        notes = [n for n in str(e.get('note', '')).split(' | ') if n and 'V6.1 proportions' not in n]
        e['note'] = ' | '.join(notes + [v['note']])
    tmp = path + '.proportions.tmp'
    json.dump(cur, open(tmp, 'w'), indent=1)
    os.replace(tmp, path)
    return new

def merged_joints():
    """data/v6_joints.json with this module's frames merged in (what the master will hold once V6.1 is loaded)"""
    cur = json.load(open(JOINTS_JSON)) if os.path.exists(JOINTS_JSON) else {}
    tgt = cur['joints'] if isinstance(cur.get('joints'), dict) else cur
    mine = json.load(open(JOINTS_V61)) if os.path.exists(JOINTS_V61) else joint_frames()
    for mate, v in mine.items():
        e = tgt.setdefault(mate, {})
        e.setdefault('frames', {}).update(v.get('frames', {}))
    return cur

def write_merged(path=WORK + '/v6_joints_merged.json'):
    """merged overrides for this agent's own ROM / export runs (work/proportions/rom_sweep_v61.py reads V6_JOINTS_JSON)"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(merged_joints(), open(path, 'w'), indent=1)
    return path

# ------------------------------------------------------------------------------------------------ attachments (hardware)
LOCAL_BOX = {   # part-local envelopes of the V5 bones (x range from the shapes, section 15 x 15), +1 mm
    'meta': (-1.0, 48.0), 'prox': (-1.0, 66.0), 'dist': (-1.0, 38.0)}

def _local_map(f, seg, q, pl):
    """old part-local point -> new part-local point (None inside a removed slab)"""
    key = bone_key(f, seg)
    ops = LAST.get(key)
    if ops is None:          # module state lost (reload): the same rule as new_parts, without the shapes
        d = pl[f][seg]['d']
        ops = [('insert', 0.5 * sum(windows(CHAIN[f][seg][0], seg)), d)] if seg == 'meta' else None
    x = q.x
    if ops is None:          # removal without state: points on the far side / tip side are unambiguous
        d = pl[f][seg]['d']
        if seg == 'prox':
            w = windows(CHAIN[f][seg][0], seg)
            return V(x + d, q.y, q.z) if x >= w[1] else (V(q) if x <= w[0] else None)
        w = windows(CHAIN[f][seg][0], seg)
        return V(x - d, q.y, q.z) if x <= w[0] else (V(q) if x >= w[1] else None)
    dx = 0.0
    for op in ops:
        if op[0] == 'insert':
            if x > op[1]:
                dx += op[2]
        else:
            a, b, move = op[1], op[2], op[3]
            if a < x < b:
                return None
            if move == 'far' and x >= b:
                dx -= b - a
            if move == 'tip' and x <= a:
                dx += b - a
    return V(x + dx, q.y, q.z)

def move_world_point(P, pl=None):
    """world point seated on a moved bone (old pose) -> its new world position; P itself when no bone moves it"""
    pl = pl or plan()
    hits = []
    for f in FINGERS:
        t2, t23 = shifts(f, pl)
        for seg, t in (('meta', V(0, 0, 0)), ('prox', t2), ('dist', t23)):
            if CHAIN[f][seg] is None or seg not in pl[f]:
                continue
            M = rest_placement(CHAIN[f][seg][1])
            q = M.inverse().multVec(P)
            x0, x1 = LOCAL_BOX[seg]
            if not (x0 <= q.x <= x1 and abs(q.y) <= 11.0 and abs(q.z) <= 8.5):
                continue
            q2 = _local_map(f, seg, q, pl)
            if q2 is None:
                continue
            hits.append((f, seg, M.multVec(q2) + t))
    if not hits:
        return V(P), None
    p0 = hits[0][2]
    for f, seg, p in hits[1:]:
        if (p - p0).Length > 0.01:
            print('[proportions] WARNING: point %s maps differently on %s %s and %s %s' % (P, hits[0][0], hits[0][1], f, seg))
    return p0, '%s %s' % (hits[0][0], hits[0][1])

def move_world_request(topic, entry, P, A):
    """mod_hardware hook for world-frame fastener requests: seat point and axis after the re-proportioning (the moves
    are pure translations, so the axis is unchanged)"""
    P2, host = move_world_point(V(P))
    if host and (P2 - V(P)).Length > 1e-6:
        print('[proportions] %s request %s rides on %s: moved %.2f mm' % (topic, entry.get('name', '?'), host, (P2 - V(P)).Length))
    return P2, V(A)
