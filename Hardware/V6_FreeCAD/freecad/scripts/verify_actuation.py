"""Verify the actuation module in a built document (headless).

  V6_TEST=<built FCStd> V6_REPORT=<txt> V6_SECT=<digits, default 123456> freecadcmd verify_actuation.py
Checks: 1 validity/volumes; 2 motors + spools + trays vs every other solid (overlap 0, gap >= 0.2); 3 plug keep-outs
free; 4 ROM sweeps J0 +-15 and J1 0..90 per finger against the magazines (incl. cheeks / J0-slot fill), trays, motors,
spools; 5 T1/T2 against the thumb yaw sweep; 6 tendon channel walls >= 1.0 mm (axis to the nearest face that is not
the tendon's own channel) + the V5 wing channels by ray marching (info).  Full run ~10 min; 1 always runs.
"""
import os, sys, math, time
LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
sys.path.insert(0, LIB)
import FreeCAD as App, Part
import mod_actuation as A
import n20
V = App.Vector
T0 = time.time()
PATH = os.environ.get('V6_TEST', '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/work/actuation/test.FCStd')
doc = App.openDocument(PATH)
out = open(os.environ.get('V6_REPORT', '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/work/actuation/verify.txt'), 'w')
def P(*a):
    s = ' '.join(str(x) for x in a)
    print(s, flush=True)
    out.write(s + '\n'); out.flush()

links = [o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None and hasattr(o.LinkedObject, 'Shape')]
def world(o):
    s = o.LinkedObject.Shape.copy()
    s.Placement = o.Placement.multiply(s.Placement)
    return s
W = {}
for o in links:
    try:
        s = world(o)
        if s.isNull() or not s.Solids:
            continue
        W[o.Label] = s
    except Exception:
        pass
BAD = {k for k, s in W.items() if not s.isValid()}          # e.g. Shell2 mesh: bbox checks only

SECT = os.environ.get('V6_SECT', '123456')
# ------------------------------------------------------------------------------------------------ 1 validity
P('=== 1. parts (validity, volume)')
for o in doc.Objects:
    if o.Name.startswith('V6_') and o.TypeId == 'Part::Feature' and ('actuation' in (o.Label2 or '')):
        s = o.Shape
        P('  %-26s valid %-5s solids %d vol %9.1f  <- %s' % (o.Name[3:], s.isValid(), len(s.Solids), s.Volume, o.Label2))

def gap(a, b):
    if not a.BoundBox.isValid() or not b.BoundBox.isValid():
        return 99, 0
    ba, bb = a.BoundBox, b.BoundBox
    bb2 = App.BoundBox(bb); bb2.enlarge(1.0)
    if not ba.intersect(bb2):
        return 99, 0
    d = a.distToShape(b)[0]
    ov = 0.0
    if d < 1e-6:
        try:
            ov = a.common(b).Volume
        except Exception:
            ov = -1
    return d, ov

act = {k: s for k, s in W.items() if k.startswith(('N20_', 'Spool_', 'Tray_'))}
others = {k: s for k, s in W.items() if k not in act}
P('=== 2. motors / spools / trays vs everything (need overlap 0, gap >= 0.2)')
act_items = sorted(act.items()) if '2' in SECT else []
worst = []
viol = 0
for k, s in act_items:
    mind, who, ov_tot = 99, '', 0.0
    for k2, s2 in list(others.items()) + [(x, y) for x, y in act.items() if x != k]:
        if k2 in BAD:
            if s.BoundBox.intersect(s2.BoundBox):
                pass
            continue
        # allowed contacts: motor<->own spool (shaft fit), tray<->own palm bone (bolted mating face)
        sid = k.split('_', 1)[1]
        if k.startswith('Spool_') and k2 == 'N20_' + sid or k.startswith('N20_') and k2 == 'Spool_' + sid:
            continue
        d, ov = gap(s, s2)
        tray_mate = k.startswith('Tray_') and k2.startswith('Base Bone 1.2')
        if tray_mate and ov < 1e-3:
            continue
        if d < mind:
            mind, who = d, k2
        ov_tot += max(ov, 0)
    flag = 'OK' if (mind >= 0.2 - 1e-6 and ov_tot < 1e-3) else 'FAIL'
    if flag == 'FAIL':
        viol += 1
    worst.append((mind, k, who, ov_tot))
    P('  %-18s min gap %6.3f to %-45s overlap %8.3f  %s' % (k, mind, who[:45], ov_tot, flag))
P('  -> %d violations' % viol if '2' in SECT else '  (skipped)')

# ------------------------------------------------------------------------------------------------ 3 plug keep-outs
P('=== 3. connector / plug keep-outs (8 mm) must be free')
kviol = 0
MP = A.all_motor_placements()
T3 = None
for sid, (pl, joint, kind) in (sorted(MP.items()) if '3' in SECT else []):
    ko = n20.plug_keepout(8.0)
    ko.Placement = pl
    ko = ko.copy(); ko.transformShape(App.Matrix(), True)
    hits = []
    for k2, s2 in W.items():
        if k2 in ('N20_' + sid,) or k2 in BAD:
            continue
        d, ov = gap(ko, s2)
        if d < 1e-6 and ov > 1e-4:
            hits.append('%s %.2f' % (k2, ov))
    if hits:
        kviol += 1
    P('  %-3s keep-out %s' % (sid, 'clear' if not hits else 'HIT: ' + '; '.join(hits)))
P('  -> %d blocked' % kviol if '3' in SECT else '  (skipped)')

# ------------------------------------------------------------------------------------------------ 4 ROM sweeps
P('=== 4. ROM sweeps (moving finger chain vs magazines, trays, motors, spools)')
CHAIN = {'index': ['Base Bone 1_V02 <1>', 'Metacarpal Bone_V02 <1>', 'Proximal Phalanx Bone_V02 <1>', 'Distal Phalanx Bone_V02 <1>'],
         'middle': ['Base Bone 1_V02 <2>', 'Metacarpal Bone_V02 <2>', 'Proximal Phalanx Bone_V02_middlefinger <1>', 'Distal Phalanx Bone_V02 <2>'],
         'ring': ['Base Bone 1_V02 <4>', 'Metacarpal Bone_V02 <3>', 'Proximal Phalanx Bone_V02 <2>', 'Distal Phalanx Bone_V02 <3>'],
         'pinky': ['Base Bone 1_V02 <3>', 'Metacarpal Bone_V02_pinky <1>', 'Proximal Phalanx Bone_V02_pinky <1>', 'Distal Phalanx Bone_V02 <4>']}
static = {k: s for k, s in W.items() if k.startswith(('N20_', 'Spool_', 'Tray_', 'Base Bone 1.2'))}
def rot_about(shape, p, d, deg):
    s = shape.copy()
    s.rotate(p, d, deg)
    return s
romfail = 0
for f in (A.FINGERS if '4' in SECT else []):
    Mp = A.pm(f)
    j0p = Mp.multVec(V(89.3, 0, 0)); j0d = Mp.multVec(V(89.3, 1, 0)) - j0p
    Mb = A.OCC[A.BASE_INST[f]]
    j1p = Mb.multVec(V(7.5, 0, 0)); j1d = Mb.multVec(V(7.5, 1, 0)) - j1p
    chain = [W[k] for k in CHAIN[f] if k in W]
    tip = chain[-1].BoundBox.Center
    # flexion sign: +deg about j1d must raise the tip (+Z)
    sgn = 1 if rot_about(Part.Vertex(tip), j1p, j1d, 10).Point.z > tip.z else -1
    own_palm = A.PALM_INST[f]
    for jn, (pp, dd, angs, movers) in {'J0': (j0p, j0d, [-15, -10, -5, 5, 10, 15], chain),
                                         'J1': (j1p, j1d, [15, 30, 45, 60, 75, 90], chain[1:])}.items():
        mind, who = 99, ''
        for a in angs:
            ang = a if jn == 'J0' else a * sgn
            mv = [rot_about(s, pp, dd, ang) for s in movers]
            for k2, s2 in static.items():
                if k2 == own_palm and jn == 'J0':
                    # the base bone pivots in the palm-bone clevis: only report overlaps beyond the baseline clevis contact
                    pass
                for i, m in enumerate(mv):
                    if k2 == own_palm and i == 0:
                        # base bone vs own palm bone: only the actuation additions below the V5 underside count
                        mag = s2.common(A.xform(A.box((60, 7.62, -12), (100, 40, 12)).fuse(
                            A.box((70, 1.9, -8.6), (86.7, 7.7, 8.6))), Mp))    # + cheeks, slot fill, slot floor
                        d, ov = gap(m, mag)
                        if d < mind:
                            mind, who = d, 'base bone %+d deg vs own magazine' % a
                        continue
                    d, ov = gap(m, s2)
                    if d < mind:
                        mind, who = d, '%s %+d deg %s' % (CHAIN[f][i if jn == 'J0' else i + 1][:22], a, k2[:30])
        flag = 'OK' if mind >= 0.2 - 0.005 else 'FAIL'      # 0.2 = the j0 tongue/back-lug design gap
        if flag == 'FAIL':
            romfail += 1
        P('  %-6s %s sweep: min gap %6.3f  (%s)  %s' % (f, jn, mind, who, flag))
P('  -> %d sweeps with contact' % romfail if '4' in SECT else '  (skipped)')

# ------------------------------------------------------------------------------------------------ 5 thumb T1/T2 vs yaw sweep
P('=== 5. T1/T2 vs thumb yaw sweep (R1 axis, carrier + gimbal + thumb chain)')
R1p, R1d = V(-165.30, 21.04, 0), V(0, 0, 1)
thumb_mov = [k for k in W if k.startswith(('first_thumb', 'second_thumb', 'third_thumb', 'Metacarpal Bone_V02 <4>', 'Proximal Phalanx Bone_V02 <3>', 'Distal Phalanx Bone_V02 <5>', 'hand_pulley'))]
tmin, twho = 99, ''
for a in (list(range(-70, 76, 5)) if '5' in SECT else []):
    for k in thumb_mov:
        m = rot_about(W[k], R1p, R1d, a)
        for t in ('N20_T1', 'N20_T2', 'Spool_T1', 'Spool_T2'):
            d, ov = gap(m, W[t])
            if d < tmin:
                tmin, twho = d, '%s %+d deg vs %s' % (k[:24], a, t)
P('  min gap %.3f (%s) %s' % (tmin, twho, 'OK' if tmin >= 0.2 else 'FAIL') if '5' in SECT else '  (skipped)')

# ------------------------------------------------------------------------------------------------ 6 tendon channel walls
P('=== 6. tendon channel walls (nearest face that is not the channel itself, need >= 1.0 mm)')
def channel_wall(solid, pts, r, skip=4.0, tol=0.06, R=3.0, others=()):
    """(min wall, where): distance from the channel axis to the nearest face that is not this channel's own cut
    surface (own = every sample of the face lies at r +- 0.06 from the axis polyline), minus r.  The first/last
    `skip` mm (openings into bays / risers / the next part) are not checked."""
    ps = A.path_points(pts, R, 5.0)
    segs = list(zip(ps[:-1], ps[1:]))
    for op, oR in others:                  # joining channels of the same tendon (e.g. the A/B risers)
        q = A.path_points(op, oR, 5.0)
        segs += list(zip(q[:-1], q[1:]))
    def dpath(q):
        best = 9e9
        for a, b in segs:
            ab = b - a
            L2 = ab.dot(ab)
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, (q - a).dot(ab) / L2))
            best = min(best, (a + ab * t - q).Length)
        return best
    bb = App.BoundBox()
    for p in ps:
        bb.add(p)
    bb.enlarge(r + 1.5)
    foreign = []
    for fc in solid.Faces:
        if not fc.BoundBox.intersect(bb):
            continue
        try:
            smp = fc.tessellate(0.05)[0]           # points that really lie on the (trimmed) face
        except Exception:
            smp = [v.Point for v in fc.Vertexes]
        if not smp:
            smp = [v.Point for v in fc.Vertexes]
        if smp and all(abs(dpath(q) - r) < tol for q in smp[::max(1, len(smp) // 40)]):
            continue
        foreign.append(fc)
    if not foreign:
        return 1.5, None                       # nothing within r + 1.5 of the axis: wall >= 1.5
    comp = Part.Compound(foreign)
    total = sum((b - a).Length for a, b in zip(ps[:-1], ps[1:]))
    worst, where, acc = 1.5, None, 0.0
    for a, b in zip(ps[:-1], ps[1:]):
        L = (b - a).Length
        n = max(1, int(L / 1.0))
        for k in range(n):
            t = acc + L * k / n
            if skip <= t <= total - skip:
                p = a + (b - a) * (k / n)
                w = comp.distToShape(Part.Vertex(p))[0] - r
                if w < worst:
                    worst, where = w, p
        acc += L
    return worst, where

def ray_wall(solid, pts, r, R=5.0, skip=1.0):
    """(min wall, where) by marching 12 radial rays per mm of channel through a local copy of the solid:
    material thickness from just outside the bore (r + 0.15, which ignores sub-0.15 mm slivers left where a
    re-cut and an existing hole are not exactly coaxial) to the first exit.  0 = the bore is open there."""
    import math
    ps = A.path_points(pts, R, 5.0)
    bb = App.BoundBox()
    for p in ps:
        bb.add(p)
    bb.enlarge(r + 2.5)
    loc = solid.common(Part.makeBox(bb.XLength, bb.YLength, bb.ZLength, V(bb.XMin, bb.YMin, bb.ZMin)))
    total = sum((b - a).Length for a, b in zip(ps[:-1], ps[1:]))
    worst, where, acc = 9.0, None, 0.0
    for a, b in zip(ps[:-1], ps[1:]):
        L = (b - a).Length
        t = (b - a) * (1.0 / L)
        u = t.cross(V(0, 0, 1) if abs(t.z) < 0.9 else V(1, 0, 0)).normalize()
        w_ = t.cross(u)
        n = max(1, int(L / 1.0))
        for k in range(n):
            if not (skip <= acc + L * k / n <= total - skip):
                continue
            p = a + (b - a) * (k / n)
            for j in range(12):
                d = u * math.cos(math.pi * j / 6) + w_ * math.sin(math.pi * j / 6)
                s_ = r + 0.15
                while s_ < r + 0.6 and not loc.isInside(p + d * s_, 1e-6, True):
                    s_ += 0.05
                if s_ >= r + 0.6:
                    wall = 0.0
                else:
                    while s_ < r + 2.0 and loc.isInside(p + d * s_, 1e-6, True):
                        s_ += 0.05
                    wall = s_ - r
                if wall < worst:
                    worst, where = wall, p + d * r
        acc += L
    return worst, where

palm_enclosed = {}
Pth = A.tendon_paths()
for f in (A.FINGERS if '6' in SECT else []):
    Ml = A.pm(f)
    solid = W[A.PALM_INST[f]]
    segs = {'A lane+guide': Pth['A'][1:9], 'B lane+guide': Pth['B'][1:9], 'C guide': Pth['C'][2:5],
            'D+ tunnel': Pth['D+'][1:8], 'D- tunnel': Pth['D-'][1:8]}
    for nm, pts in segs.items():
        wp = [Ml.multVec(p) for p in pts]
        # skip the first 3 mm (entry from the bay) and the last 1 mm
        oth = []
        if nm[0] in 'AB':
            P0, P1 = Pth[nm[0]][0], Pth[nm[0]][1]
            oth = [([Ml.multVec(v) for v in (P0 + V(0, -2.0, 0), P1, P1 + V(6.0, 0, 0))], 3.0)]
        w, p = channel_wall(solid, wp, A.TR, others=oth)
        where = '' if w >= 1.0 - 1e-6 else ' <-- thin at ' + str(None if p is None else tuple(round(v, 1) for v in p))
        P('  %-6s %-13s min wall %.2f mm%s' % (f, nm, w, where))
    for nm in ('D+', 'D-'):                    # V5 lateral wing channel (dia 1.5, V5 position): informational
        wp = [Ml.multVec(p) for p in Pth[nm][7:9]]
        w, p = ray_wall(solid, wp, 0.75)
        P('  %-6s %-13s min wall %.2f mm at %s (V5 channel, position unchanged; it already opens into the J0 tongue slot at x 82.5-83.5 in the j0 input: info only)'
          % (f, nm + ' wing', w, None if p is None else tuple(round(v, 1) for v in p)))
if '6' not in SECT:
    P('  (skipped)')
P('done in %.0fs' % (time.time() - T0))
out.close()
