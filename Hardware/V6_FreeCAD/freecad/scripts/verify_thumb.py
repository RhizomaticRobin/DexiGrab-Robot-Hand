"""Verify the V6 thumb (requirement 8) on a headless build (freecadcmd).

env: V6_IN (default work/thumb/test.FCStd), V6_STEPS (default 13), V6_FAST=1 (skip the R1 x R2 grid)
Checks: validity/volumes; overlap (must be 0) and min gap (>= 0.2 where a running clearance is designed) between the
thumb parts and their neighbours; the yaw load path (journal vs coupler float); range-of-motion sweeps of R1..R6
about the Onshape mate frames (data/top_assembly_definition.json, same convention as scripts/rom_sweep.py), with the
parts that ride along (R2 tube with the carrier; motor shaft turns with the coupler) and the two keeper screws
modelled as steel pins (yaw keeper fixed in cover3, roll keeper fixed in the cross).
"""
import os, sys, json, math, time
sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
import FreeCAD as App, Part
import mod_thumb as T

V = App.Vector
R = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
inp = os.environ.get('V6_IN', R + '/freecad/work/thumb/test.FCStd')
STEPS = int(os.environ.get('V6_STEPS', '13'))
TOL = 0.01                    # mm3 counted as a collision
t_start = time.time()
doc = App.openDocument(inp)
# rest placements of the re-clocked flex stack (stand-in until build_v6 applies mod_thumb.placements())
for lab, pl in T.placements().items():
    for o in doc.getObjectsByLabel(lab):
        if not o.Placement.isSame(pl, 1e-7):
            o.Placement = pl
            print('applied rest placement from mod_thumb.placements():', lab)

W = {}
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.LinkedObject is not None and o.LinkedObject.Shape.Solids:
        s = o.LinkedObject.Shape.copy()
        s.Placement = o.Placement.multiply(s.Placement)
        W[o.Label] = s

HUB, CAR, CRS, THD = 'hand_pulley_wheel_thumb <1>', 'first_thumb_hinge <1>', 'second_thumb_hinge <1>', 'third_thumb_hinge <1>'
MET, PRX, DST = 'Metacarpal Bone_V02 <4>', 'Proximal Phalanx Bone_V02 <3>', 'Distal Phalanx Bone_V02 <5>'
COV, MOT, TUB = 'driver_side_palm_cover3 <1>', 'T3_motor', 'thumb_R2_tube'
THUMB = [HUB, CAR, TUB, CRS, THD, MET, PRX, DST]

# steel pins modelled for the checks (not in the document: the hardware module places the real screws)
yk = V(T.R1X, T.R1Y + T.KEEP_A, T.Z_KEEP)
W['keeper_yaw (M2)'] = T.cyl(yk + V(-T.KEEP_HALF, 0, 0), yk + V(T.KEEP_HALF + 3.6, 0, 0), 1.0)
kx, kz = T.K3
W['keeper_roll (M2)'] = T.GF.w(T.cyl((kx, -T.CROSS_W + 2.2, kz), (kx, T.CROSS_W - 0.2, kz), 1.0))
W['R2 pin (M2)'] = T.YF.w(T.cyl((0, T.ARM_OUT - 1.0, 0), (0, T.ARM_OUT - 1.0 - 8.0, 0), 1.0))
r4 = T.GF.loc(T.R4P)
W['R4 bolt (M2)'] = T.GF.w(T.cyl((r4.x, -7.0, r4.z), (r4.x, 5.3, r4.z), 1.0))
W['R4 bolt (M2)'].Placement = T.reclock_placement().multiply(W['R4 bolt (M2)'].Placement)   # rides on the flex stack

def gap(a, b):
    A, B = W[a], W[b]
    d = A.distToShape(B)[0]
    ov = A.common(B).Volume if d < 1e-6 else 0.0
    return d, ov

print('=' * 100)
print('V6 THUMB VERIFY  %s' % inp)
print('=' * 100)
print('\n[1] parts')
for k in ('V6_cover3', 'V6_thumb_hub', 'V6_thumb_hinge1', 'V6_thumb_hinge2', 'V6_thumb_hinge3', 'V6_thumb_metacarpal',
          'V6_thumb_proximal', 'V6_thumb_distal', 'V6_thumb_n20', 'V6_thumb_r2_tube'):
    o = doc.getObject(k)
    if o is None:
        print('  %-22s MISSING' % k); continue
    s = o.Shape
    print('  %-22s vol %9.2f  valid %-5s solids %d' % (k, s.Volume, s.isValid(), len(s.Solids)))
for lab, key in ((MET, 'thumb_metacarpal'), (PRX, 'thumb_proximal'), (DST, 'thumb_distal')):
    o = doc.getObjectsByLabel(lab)[0]
    print('  relink %-30s -> %s  %s' % (lab, o.LinkedObject.Name, 'OK' if o.LinkedObject.Name == 'V6_' + key else 'WRONG'))

# ---------------------------------------------------------------------------------------------------- static pairs
print('\n[2] current pose: designed running clearances (need >= 0.2, overlap 0)')
RUN = [(HUB, CAR, 'coupler float in carrier pocket (0.2)'), (HUB, MOT, 'coupler D-bore on the N20 shaft (0.2)'),
       (CAR, COV, 'journal r9/9.2 + thrust ring + floor'), (CAR, 'keeper_yaw (M2)', 'yaw keeper in the rim groove'),
       (CAR, CRS, 'R2 arms / cross faces'), (CAR, TUB, 'tube glued in the -s arm (0.2 glue gap)'),
       (TUB, CRS, 'R2 tube pin in the cross'), ('R2 pin (M2)', CRS, 'R2 M2 pin in the cross'),
       (CRS, THD, 'R3 bore / collar thrust face'), ('keeper_roll (M2)', THD, 'roll keeper in the shaft groove'),
       (THD, MET, 'R4 tongue / thickened cheeks'), ('R4 bolt (M2)', THD, 'R4 bolt in the tongue'),
       (MET, PRX, 'R5 tongue / narrowed slot'), (PRX, DST, 'R6 tongue / narrowed slot'), (MOT, COV, 'N20 in its pocket')]
bad = 0
for a, b, what in RUN:
    d, ov = gap(a, b)
    ok = ov < TOL and d >= 0.2 - 1e-3
    bad += not ok
    print('  %-4s %-26s x %-26s gap %6.3f  overlap %7.3f   %s' % ('ok' if ok else 'FAIL', a[:26], b[:26], d, ov, what))
print('\n[3] current pose: no overlap with anything else (thumb parts + T3 motor vs every other link)')
others = [l for l in W if l not in THUMB + [COV, MOT] and not l.startswith(('keeper', 'R2 pin', 'R4 bolt'))]
pairs_done = set((a, b) for a, b, _ in RUN) | set((b, a) for a, b, _ in RUN)
worst = []
for a in THUMB + [MOT, COV]:
    for b in others + THUMB + [MOT, COV]:
        if a == b or (a, b) in pairs_done or (b, a) in pairs_done:
            continue
        pairs_done.add((a, b))
        if not W[a].BoundBox.intersect(W[b].BoundBox):
            continue
        d, ov = gap(a, b)
        if ov > TOL or d < 0.2:
            worst.append((ov, d, a, b))
import csv
base = set()
try:
    for r in csv.DictReader(open(R + '/freecad/work/audit_baseline_j0.csv')):
        base.add(frozenset((r['part_a'], r['part_b'])))
except Exception:
    pass
new_bad = 0
for ov, d, a, b in sorted(worst, reverse=True):
    pre = frozenset((a, b)) in base and MOT not in (a, b) and not set((a, b)) & set(THUMB)
    new_bad += not pre
    print('  %-8s %-30s x %-40s gap %6.3f overlap %8.3f  %s' % ('OVERLAP' if ov > TOL else 'gap<0.2', a, b, d, ov,
          'pre-existing in audit_baseline_j0.csv, outside the thumb region (orchestrator mod_fixes)' if pre else 'THUMB'))
print('  new thumb-related problems: %d' % new_bad)
if not worst:
    print('  none: every thumb part and the T3 motor clear everything else by >= 0.2 mm')

# ---------------------------------------------------------------------------------------------------- load path
print('\n[4] yaw load path')
s_journal = W[CAR].distToShape(W[COV])[0]
s_cpl_pocket = W[HUB].distToShape(W[CAR])[0]
s_cpl_shaft = W[HUB].distToShape(W[MOT])[0]
print('  carrier -> cover3: journal dia 18.0 x 6.3 in dia 18.4 cup, thrust ring r 9.2..11.0 (platform 0.2 above),')
print('                      yaw keeper M2 (1.2 mm in the rim groove, 0.2/0.4 axial), min gap carrier-cover3 %.3f' % s_journal)
print('  carrier -> motor  : only via the coupler: coupler-pocket %.3f + coupler-shaft %.3f = %.2f mm radial float'
      % (s_cpl_pocket, s_cpl_shaft, s_cpl_pocket + s_cpl_shaft))
print('                      > journal clearance 0.20 -> radial/axial/moment loads land on cover3; the shaft sees torque')
ov = W[CAR].common(W[MOT]).Volume
print('  carrier touches the motor: %s   (V5: the hub was carried by the shaft alone)' % ('YES' if ov > 0 else 'no'))

# ---------------------------------------------------------------------------------------------------- ROM
d = json.load(open(R + '/data/top_assembly_definition.json'))
ra = d['rootAssembly']
inst = {i['id']: i for i in ra['instances']}
for sa in d['subAssemblies']:
    for i in sa['instances']:
        inst[i['id']] = i
links = {o.Label: o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None}
J = {}
for f in ra['features']:
    fd = f['featureData']
    if f['featureType'] != 'mate' or fd['mateType'] != 'REVOLUTE':
        continue
    ents = []
    for e in fd['matedEntities']:
        name = inst[e['matedOccurrence'][-1]]['name']
        cs = e['matedCS']
        lk = links[name]
        pl = lk.Placement.multiply(lk.LinkedObject.Shape.Placement)
        ents.append((name, pl.multVec(V(*[c * 1000 for c in cs['origin']])), pl.Rotation.multVec(V(*cs['xAxis'])),
                     pl.Rotation.multVec(V(*cs['zAxis']))))
    lim = fd.get('mateLimits', {})
    J[fd['name']] = dict(ents=ents, lo=math.degrees(lim.get('limitAxialZMin', -math.pi)),
                         hi=math.degrees(lim.get('limitAxialZMax', math.pi)),
                         local=[(inst[e['matedOccurrence'][-1]]['name'], V(*[c * 1000 for c in e['matedCS']['origin']]),
                                 V(*e['matedCS']['xAxis']), V(*e['matedCS']['zAxis'])) for e in fd['matedEntities']])
J['Revolute 2']['lo'], J['Revolute 2']['hi'] = T.R3_LIMITS          # V6 limits (published in data/v6_joints.json)
TH0_REST = {'Revolute 2': T.ROLL_REST}   # rest value of re-placed joints (data/v6_joints.json "rest")
J['Revolute 5']['lo'], J['Revolute 5']['hi'] = T.YAW_LIMITS

CHAIN = {'Revolute 5': ('R1 yaw', HUB, [HUB, CAR, TUB, CRS, THD, MET, PRX, DST, 'R2 pin (M2)', 'keeper_roll (M2)', 'R4 bolt (M2)']),
         'Revolute 1': ('R2', CRS, [CRS, THD, MET, PRX, DST, 'keeper_roll (M2)', 'R4 bolt (M2)']),
         'Revolute 2': ('R3 roll', THD, [THD, MET, PRX, DST, 'R4 bolt (M2)']),
         'Revolute 3': ('R4', MET, [MET, PRX, DST, 'R4 bolt (M2)']),
         'Revolute 4': ('R5', PRX, [PRX, DST]),
         'Revolute 6': ('R6', DST, [DST])}
SKIP = {frozenset((HUB, MOT))}          # the N20 shaft turns with the coupler
RUNPAIRS = {'Revolute 5': [(CAR, COV), (CAR, 'Palm_bone1 <5>'), (CAR, 'keeper_yaw (M2)')],
            'Revolute 1': [(CRS, CAR), (CRS, TUB), (CRS, 'R2 pin (M2)')],
            'Revolute 2': [(THD, CRS), (THD, 'keeper_roll (M2)')],
            'Revolute 3': [(MET, THD)], 'Revolute 4': [(PRX, MET)], 'Revolute 6': [(DST, PRX)]}

def joint_rot(name, ang):
    j = J[name]
    (na, pa, xa, za), (nb, pb, xb, zb) = j['ents']
    th0 = TH0_REST.get(name, math.degrees(math.atan2(xa.cross(xb).dot(za), xa.dot(xb))))
    child = CHAIN[name][1]
    sign = 1.0 if child == na else -1.0
    delta = (ang - th0 + 180) % 360 - 180
    return App.Placement(V(0, 0, 0), App.Rotation(za, sign * delta), pa), th0

fixed_all = [l for l in W if l not in CHAIN['Revolute 5'][2]]

def hits_at(moving, pl, pre=None):
    """Collisions of the moved set against everything not moving (pre: extra placement applied first)."""
    out = []
    mset = set(moving)
    for m in moving:
        sm = W[m].copy()
        if pre is not None and m in pre[1]:
            sm.Placement = pre[0].multiply(sm.Placement)
        sm.Placement = pl.multiply(sm.Placement)
        bm = sm.BoundBox
        for f in W:
            if f in mset or frozenset((m, f)) in SKIP:
                continue
            sf = W[f]
            if pre is not None and f in pre[1]:
                sf = sf.copy(); sf.Placement = pre[0].multiply(sf.Placement)
            if not bm.intersect(sf.BoundBox):
                continue
            v = sm.common(sf).Volume
            if v > TOL:
                out.append('%s x %s %.2f' % (m.replace(' <1>', ''), f.replace(' <1>', ''), v))
    return out

print('\n[5] range of motion (each joint alone, others at the current pose; %d steps over the mate limits)' % STEPS)
summary = {}
for name in ('Revolute 5', 'Revolute 1', 'Revolute 2', 'Revolute 3', 'Revolute 4', 'Revolute 6'):
    label, child, moving = CHAIN[name]
    lo, hi = J[name]['lo'], J[name]['hi']
    if hi - lo >= 359:
        lo, hi = -180, 180
    angs = [lo + (hi - lo) * k / (STEPS - 1) for k in range(STEPS)]
    res = []
    th0 = None
    gmin = {}
    for a in angs:
        pl, th0 = joint_rot(name, a)
        res.append((a, hits_at(moving, pl)))
        for m, f in RUNPAIRS.get(name, []):
            sm = W[m].copy(); sm.Placement = pl.multiply(sm.Placement)
            g = sm.distToShape(W[f])[0]
            if g < gmin.get((m, f), (9e9,))[0]:
                gmin[(m, f)] = (g, a)
    clear = [a for a, h in res if not h]
    summary[name] = (label, lo, hi, th0, res)
    print('  %-8s %-11s limits [%6.1f, %6.1f] now %6.1f : %s' % (label, name, lo, hi, th0,
          'CLEAR over the full range' if len(clear) == len(res) else 'collides at %s' % [round(a, 1) for a, h in res if h]))
    for a, h in res:
        if h:
            print('           %6.1f: %s' % (a, '; '.join(h[:4])))
    for (m, f), (g, a) in gmin.items():
        print('           running gap %-24s x %-24s min %.3f (at %.1f)' % (m[:24], f[:24], g, a))

if os.environ.get('V6_FAST') != '1':
    print('\n[6] R1 yaw band vs R2 pitch (5 deg steps; collision-free yaw intervals)')
    r2moving = CHAIN['Revolute 1'][2]
    for r2 in (0.0, -45.0, -90.0):
        pl2, _ = joint_rot('Revolute 1', r2)
        pre = (pl2, set(r2moving))
        lo, hi = J['Revolute 5']['lo'], J['Revolute 5']['hi']
        n = int(round((hi - lo) / 5.0)) + 1
        ok = []
        first_hit = {}
        for k in range(n):
            a = lo + 5.0 * k
            pl, _ = joint_rot('Revolute 5', a)
            h = hits_at(CHAIN['Revolute 5'][2], pl, pre)
            ok.append((a, not h))
            if h and not first_hit:
                first_hit = {a: h[:3]}
        # intervals
        iv, cur = [], None
        for a, good in ok:
            if good and cur is None:
                cur = [a, a]
            elif good:
                cur[1] = a
            elif cur is not None:
                iv.append(tuple(cur)); cur = None
        if cur is not None:
            iv.append(tuple(cur))
        print('  R2 %6.1f : yaw clear on %s' % (r2, ', '.join('[%.1f, %.1f]' % x for x in iv) or 'nothing'))

print('\n[7] hard stops (keeper screws; stops sit %.1f deg (yaw) / %.1f deg (roll) outside the mate limits)' % (T.YAW_STOP, T.R3_STOP))
for name, label, pin, part, m in (('Revolute 5', 'yaw', 'keeper_yaw (M2)', CAR, T.YAW_STOP), ('Revolute 2', 'roll', 'keeper_roll (M2)', THD, T.R3_STOP)):
    lo, hi = J[name]['lo'], J[name]['hi']
    for a in (lo - m - 1.0, lo, hi, hi + m + 1.0):
        pl, _ = joint_rot(name, a)
        s = W[part].copy(); s.Placement = pl.multiply(s.Placement)
        d_ = s.distToShape(W[pin])[0]
        v = s.common(W[pin]).Volume if d_ < 1e-6 else 0.0
        print('  %-5s %6.1f deg: keeper gap %6.3f overlap %6.3f  %s' % (label, a, d_, v,
              'stop engaged' if v > 0 or d_ < 1e-3 else 'free'))
print('\nstatic running-clearance failures: %d    done in %.0fs' % (bad, time.time() - t_start))

# ==================================================================================================== V6 opposition
import numpy as np
LINKS = {o.Label: o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None}
def base_pl(lab):
    lk = LINKS[lab]
    return lk.Placement.multiply(lk.LinkedObject.Shape.Placement)

def joint_axis(name):
    """(point, unit axis, sign, theta0) of a revolute mate at the rest pose (rom_sweep convention)."""
    (na, oa, xa, za), (nb, ob, xb, zb) = J[name]['local']
    pa_, pb_ = base_pl(na), base_pl(nb)
    p, x1, z1 = pa_.multVec(oa), pa_.Rotation.multVec(xa), pa_.Rotation.multVec(za)
    x2 = pb_.Rotation.multVec(xb)
    th0 = TH0_REST.get(name, math.degrees(math.atan2(x1.cross(x2).dot(z1), x1.dot(x2))))
    return p, z1, na, nb, th0

def rotmat(p, a, deg):
    """4x4 numpy rotation about axis a through p."""
    a = np.array([a.x, a.y, a.z]); a = a / np.linalg.norm(a)
    t = math.radians(deg); c, s_ = math.cos(t), math.sin(t)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    Rm = np.eye(3) + s_ * K + (1 - c) * K @ K
    M = np.eye(4); M[:3, :3] = Rm
    pv = np.array([p.x, p.y, p.z]); M[:3, 3] = pv - Rm @ pv
    return M

THUMB_CH = [('Revolute 5', HUB), ('Revolute 1', CRS), ('Revolute 2', THD), ('Revolute 3', MET), ('Revolute 4', PRX), ('Revolute 6', DST)]
FINGERS = {'index': [('Revolute 22', 'Base Bone 1_V02 <1>'), ('Revolute 15', 'Metacarpal Bone_V02 <1>'),
                     ('Revolute 16', 'Proximal Phalanx Bone_V02 <1>'), ('Revolute 17', 'Distal Phalanx Bone_V02 <1>')],
           'middle': [('Revolute 21', 'Base Bone 1_V02 <2>'), ('Revolute 18', 'Metacarpal Bone_V02 <2>'),
                      ('Revolute 13', 'Proximal Phalanx Bone_V02_middlefinger <1>'), ('Revolute 14', 'Distal Phalanx Bone_V02 <2>')]}
AX = {}
for name, child in THUMB_CH + FINGERS['index'] + FINGERS['middle']:
    p, z1, na, nb, th0 = joint_axis(name)
    AX[name] = (p, z1, 1.0 if child == na else -1.0, th0)

def jm(name, val):
    p, z1, sg, th0 = AX[name]
    return rotmat(p, z1, sg * ((val - th0 + 180) % 360 - 180))

def chain_mats(names_vals):
    """Cumulative transforms after each joint (product of exponentials about the rest axes)."""
    out, M = [], np.eye(4)
    for name, val in names_vals:
        M = M @ jm(name, val)
        out.append(M.copy())
    return out

def to_pl(M):
    m = App.Matrix(*[float(v) for v in M.reshape(-1)])
    return App.Placement(m)

# which labels ride after which joint of the thumb chain
RIDE = {0: [HUB, CAR, TUB, 'R2 pin (M2)'], 1: [CRS, 'keeper_roll (M2)'], 2: [THD], 3: [MET, 'R4 bolt (M2)'], 4: [PRX], 5: [DST]}
FRIDE = {0: 0, 1: 1, 2: 2, 3: 3}

def thumb_moved(vals):
    mats = chain_mats(list(zip([n for n, _ in THUMB_CH], vals)))
    out = {}
    for k, labs in RIDE.items():
        pl = to_pl(mats[k])
        for l in labs:
            sh = W[l].copy(); sh.Placement = pl.multiply(sh.Placement); out[l] = sh
    return out

def finger_moved(fname, vals):
    names = [n for n, _ in FINGERS[fname]]
    mats = chain_mats(list(zip(names, vals)))
    out = {}
    for k, (n, lab) in enumerate(FINGERS[fname]):
        pl = to_pl(mats[k]); sh = W[lab].copy(); sh.Placement = pl.multiply(sh.Placement); out[lab] = sh
    return out

FLEXSTACK = [THD, MET, PRX, DST, 'R4 bolt (M2)']
UPSTREAM = [HUB, CAR, TUB, CRS, 'R2 pin (M2)', 'keeper_roll (M2)']
NONADJ = [(MET, CRS), (MET, CAR), (PRX, THD), (PRX, CRS), (PRX, CAR), (DST, MET), (DST, THD), (DST, CRS), (DST, CAR)]
FINGER_LABS = set(l for f in FINGERS.values() for _, l in f)

def collide(moved, extra_fixed=None, skip=()):
    """Hits of moved thumb parts vs fixed parts (and non-adjacent thumb pairs)."""
    hits = []
    fixed = {l: W[l] for l in W if l not in moved and l not in ('keeper_yaw (M2)',) and not l.startswith('R2 pin')}
    fixed['keeper_yaw (M2)'] = W['keeper_yaw (M2)']
    if extra_fixed:
        fixed.update(extra_fixed)
    for m in FLEXSTACK + [CAR, CRS]:
        sm = moved[m]
        for f, sf in fixed.items():
            if f in skip or frozenset((m, f)) in SKIP:
                continue
            if not sm.BoundBox.intersect(sf.BoundBox):
                continue
            v = sm.common(sf).Volume
            if v > TOL:
                hits.append((m, f, v))
    for a, b in NONADJ:
        if moved[a].BoundBox.intersect(moved[b].BoundBox):
            v = moved[a].common(moved[b]).Volume
            if v > TOL:
                hits.append((a, b, v))
    return hits

REST = T.ROLL_REST
th_now = {n: AX[n][3] for n, _ in THUMB_CH}
print('\n[8] thumb flexion at the re-clocked rest (roll %.1f), flex synergy R3 = 90s, R4 = -90s, R6 = -90s' % REST)
print('    rest mate values now: ' + ', '.join('%s %.1f' % (n, th_now[n]) for n, _ in THUMB_CH))
for yaw in (-70.0, -45.0, -17.0, 0.0):
    for pitch in (0.0, -45.0):
        line, first_struct = [], None
        for sflex in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
            vals = [yaw, pitch, REST, 90 * sflex, -90 * sflex, -90 * sflex]
            mv = thumb_moved(vals)
            h = collide(mv)
            fing = sorted(set(f for m, f, v in h if f in FINGER_LABS))
            struct = [(m, f, v) for m, f, v in h if f not in FINGER_LABS]
            if struct and first_struct is None:
                first_struct = (sflex, struct)
            line.append('%.1f:%s' % (sflex, 'ok' if not h else ('FINGER ' + '/'.join(x.split(' <')[0].replace('Phalanx Bone_V02', '').replace('Bone_V02', '') + x[-3:] for x in fing) if fing and not struct else 'STRUCT')))
        print('  yaw %6.1f pitch %6.1f : %s' % (yaw, pitch, '  '.join(line)))
        if first_struct:
            sfl, st = first_struct
            print('        first structure hit at s=%.1f: %s' % (sfl, '; '.join('%s x %s %.1f' % (m.split(' <')[0], f.split(' <')[0], v) for m, f, v in st[:4])))

print('\n[9] opposition: thumb pad vs index / middle pad (grid over R1,R2,R3(+-25),flex; finger flex synergy)')
PAD_LOC = [V(3.0, 0, 7.5), V(6.0, 0, 7.5), V(9.0, 0, 7.5), V(1.0, 0, 5.0)]
def pad_pts(lab):
    pl = base_pl(lab)
    return np.array([[q.x, q.y, q.z, 1.0] for q in (pl.multVec(v) for v in PAD_LOC)])
def pad_n(lab):
    d = base_pl(lab).Rotation.multVec(V(0, 0, 1))
    return np.array([d.x, d.y, d.z, 0.0])
tp, tn = pad_pts(DST), pad_n(DST)
yaws = np.arange(-75, 75.01, 7.5); pitches = np.arange(-125, 25.01, 10.0)
rolls = np.array([T.R3_LIMITS[0], REST - 12.5, REST, REST + 12.5, T.R3_LIMITS[1]]); flexs = np.linspace(0, 1, 11)
TY = np.stack([jm('Revolute 5', a) for a in yaws]); TP = np.stack([jm('Revolute 1', a) for a in pitches])
TR = np.stack([jm('Revolute 2', a) for a in rolls])
TF = np.stack([jm('Revolute 3', 90 * f) @ jm('Revolute 4', -90 * f) @ jm('Revolute 6', -90 * f) for f in flexs])
TT = (TY[:, None, None, None] @ TP[None, :, None, None] @ TR[None, None, :, None] @ TF[None, None, None, :])  # (21,16,5,11,4,4)
Tt = TT.reshape(-1, 4, 4)
thumb_p = np.einsum('nij,pj->npi', Tt, tp)[..., :3]          # (N, 4, 3)
thumb_n = np.einsum('nij,j->ni', Tt, tn)[..., :3]
idx_shape = TT.shape[:4]
results = {}
for fname, ch in FINGERS.items():
    fl = np.linspace(0, 1, 11)
    fp, fnrm = pad_pts(ch[3][1]), pad_n(ch[3][1])
    FM = np.stack([jm(ch[0][0], 0.0) @ jm(ch[1][0], 90 * f) @ jm(ch[2][0], -90 * f) @ jm(ch[3][0], -90 * f) for f in fl])
    fing_p = np.einsum('nij,pj->npi', FM, fp)[..., :3]         # (11, 4, 3)
    fing_n = np.einsum('nij,j->ni', FM, fnrm)[..., :3]
    d = np.linalg.norm(thumb_p[:, None, :, None, :] - fing_p[None, :, None, :, :], axis=-1).min(axis=(2, 3))   # (N, 11)
    facing = np.einsum('ni,mi->nm', thumb_n, fing_n)              # pad normals: < 0 = opposed
    score = d + np.where(facing < -0.2, 0.0, 15.0)
    # diversify: best 4 per pitch value (the closest pads overall sit at low pitch where the thumb base hits cover3)
    pidx = np.unravel_index(np.arange(score.shape[0]), idx_shape)[1]
    cands = []
    for ip in range(len(pitches)):
        sub = np.where(pidx == ip)[0]
        sc = score[sub]
        flat = np.argsort(sc.reshape(-1))[:4]
        for o in flat:
            n_, m = divmod(int(o), sc.shape[1])
            n = int(sub[n_])
            iy, ip2, ir, iflex = np.unravel_index(n, idx_shape)
            cands.append((float(d[n, m]) + (0 if facing[n, m] < -0.2 else 15), float(facing[n, m]), float(yaws[iy]), float(pitches[ip2]), float(rolls[ir]), float(flexs[iflex]), float(fl[m])))
    cands.sort()
    # shape check: take the closest structure-clean candidates, then back the finger off (bisection on its flex)
    # until the two distal pads just touch; the grid step alone lands in interpenetration
    def status(c, fv):
        dd, fc, yv, pv, rv, sv, _ = c
        mv = thumb_moved([yv, pv, rv, 90 * sv, -90 * sv, -90 * sv])
        fm = finger_moved(fname, [0.0, 90 * fv, -90 * fv, -90 * fv])
        h = collide(mv, extra_fixed=fm)
        other = [(m, f, v) for m, f, v in h if f not in fm]
        tvf = [(m, f, v) for m, f, v in h if f in fm and not (m == DST and f == ch[3][1])]
        fh = []
        for fl_lab, fsh in fm.items():
            for f, sf in W.items():
                if f in fm or f in mv or f in FINGER_LABS or f.startswith(('keeper', 'R2 pin', 'R4 bolt')):
                    continue
                if fsh.BoundBox.intersect(sf.BoundBox) and fsh.common(sf).Volume > TOL:
                    fh.append((fl_lab, f))
        dres = mv[DST].distToShape(fm[ch[3][1]])
        ov = mv[DST].common(fm[ch[3][1]]).Volume if dres[0] < 1e-6 else 0.0
        return other, tvf, fh, dres, ov
    best, tried = None, 0
    for c in cands:
        if tried >= 12 or best is not None:
            break
        dd, fc, yv, pv, rv, sv, fv = c
        other, tvf, fh, dres, ov = status(c, fv)
        if other or tvf or fh:
            continue
        tried += 1
        lo_f, hi_f = max(fv - 0.4, 0.0), fv          # lo: apart (checked), hi: overlapping or touching
        o_lo = status(c, lo_f)
        if o_lo[4] > 0 or o_lo[3][0] < 1e-6:
            continue
        for _ in range(9):
            mid = 0.5 * (lo_f + hi_f)
            st = status(c, mid)
            if st[0] or st[1] or st[2]:
                break
            if st[4] > 0 or st[3][0] < 1e-6:
                hi_f = mid
            else:
                lo_f, last = mid, st
        st = status(c, lo_f)
        if not (st[0] or st[1] or st[2]) and st[4] < 1.0 and st[3][0] <= 0.5:
            p1, p2 = st[3][1][0]
            best = (st[3][0], c, lo_f, (p1 + p2) * 0.5)
    if best:
        g, (dd, fc, yv, pv, rv, sv, _), fv, cp = best
        print('  %-6s PINCH: thumb pad touches the %s pad (gap %.2f mm, no other collision); pads facing %.2f' % (fname, fname, g, fc))
        print('         thumb  R5 yaw %.1f  R1 pitch %.1f  R2 roll %.1f (rest %+.1f)  flex %.0f%%: R3 %.0f R4 %.0f R6 %.0f'
              % (yv, pv, rv, rv - REST, 100 * sv, 90 * sv, -90 * sv, -90 * sv))
        print('         %-6s J0 0  flex %.0f%%: J1 %.0f J2 %.0f J3 %.0f   contact at (%.1f, %.1f, %.1f)' % (fname, 100 * fv, 90 * fv, -90 * fv, -90 * fv, cp.x, cp.y, cp.z))
    else:
        print('  %-6s no clean pinch found among the best candidates' % fname)
    results[fname] = best
print('\n[10] tendon path through the gimbal over the roll range (dia 0.8 tendon: tube -> entry fan -> axial bore -> pad branch)')
def tendon_parts(pl_roll):
    """Fixed entry segment (tube inner end -> gimbal centre, along R2) and the rolled axial + pad-branch segments."""
    r = 0.4
    ent = T.cyl(T.G + T.E2 * -14.0, T.G + T.E2 * -0.3, r)
    ax = T.GF.w(T.cyl((0.4, 0, 0), (T.TENDON_X, 0, 0), r))
    br = T.GF.w(T.cyl((T.TENDON_X, 0, 0), (T.TENDON_X + 6.5, 0, 6.5), r))
    moved = []
    for sh in (ax, br):
        sh.Placement = pl_roll.multiply(T.reclock_placement().multiply(sh.Placement)); moved.append(sh)
    return ent, moved
for ang in (T.R3_LIMITS[0], -75.5, T.ROLL_REST, -45.5, T.R3_LIMITS[1]):
    pl, _ = joint_rot('Revolute 2', ang)
    thd = W[THD].copy(); thd.Placement = pl.multiply(thd.Placement)
    ent, (ax, br) = tendon_parts(pl)
    hits = []
    for nm, seg in (('entry', ent), ('axial', ax), ('branch', br)):
        for lab, sh in (('roll shaft', thd), ('cross', W[CRS]), ('tube', W[TUB])):
            if seg.BoundBox.intersect(sh.BoundBox):
                v = seg.common(sh).Volume
                if v > 1e-4:
                    hits.append('%s x %s %.3f' % (nm, lab, v))
    print('  roll %6.1f: %s' % (ang, 'tendon path clear' if not hits else 'BLOCKED: ' + '; '.join(hits)))
print('\nall sections done in %.0fs' % (time.time() - t_start))
