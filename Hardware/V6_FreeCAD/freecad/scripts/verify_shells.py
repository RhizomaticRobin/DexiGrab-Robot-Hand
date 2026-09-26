"""Verify the shells module on a built model (headless).

env: V6_IN = built FCStd (default work/shells/test.FCStd), SHELLS_QUICK=1 for a reduced sweep.
Checks: validity/volumes, Shell2 fidelity to the 3MF mesh, nominal overlaps + min gaps, the relative-motion
sweep of the thumb group (Shell2 + cover3 + thumb base) against the pinky group (Shell1 + cover_back + cover2 +
retention stacks) about the joint centre C, retention engagement, and the travel-stop angles.
Prints PASS/FAIL lines and a final summary.
"""
import os, sys, math, time, itertools
sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
import numpy as np
import FreeCAD as App, Part
import mod_shells as M

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'
inp = os.environ.get('V6_IN', os.path.join(ROOT, 'work', 'shells', 'test.FCStd'))
QUICK = os.environ.get('SHELLS_QUICK', '1') == '1'          # default: 12-pose sweep (fits one 10-min run)
SWEEP_ONLY = os.environ.get('SHELLS_SWEEP_ONLY') == '1'
ROLLS_ENV = os.environ.get('SHELLS_ROLLS')                    # e.g. "-1,0,5" to split the full sweep over runs
TILTS_ENV = os.environ.get('SHELLS_TILTS')                    # "3" or "5" tilt corners
results = []
import resource


def rss(tag):
    print('     [mem] %-28s max RSS so far %.0f MB' % (tag, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6))


def check(name, ok, msg):
    results.append((name, ok))
    print('%-4s %-46s %s' % ('PASS' if ok else 'FAIL', name, msg))


doc = App.openDocument(inp)
links = {o.Label: o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None}


def W(lbl):
    o = links[lbl]
    s = o.LinkedObject.Shape.copy()
    s.Placement = o.Placement.multiply(s.Placement)
    return s


P1 = links['Shell1 <1>'].Placement
# ---------------------------------------------------------------- 1. validity, volumes, provenance
print('== parts')
for lbl in ('Shell1 <1>', 'Shell2 <1>', 'cover_back <1>', 'driver_side_palm_cover2 <1>'):
    o = links[lbl].LinkedObject
    s = o.Shape
    check('valid ' + lbl, s.isValid() and len(s.Solids) == 1,
          'object %s  vol %.1f  solids %d  (%s)' % (o.Name, s.Volume, len(s.Solids), o.Label2))
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.Label.startswith('shells '):
        check('valid ' + o.Label, o.LinkedObject.Shape.isValid(), 'vol %.2f' % o.LinkedObject.Shape.Volume)
src2 = doc.getObject('SRC_Shell2').Shape.copy()
check('Shell2 source is the mesh', len(src2.Solids) == 0, 'SRC_Shell2: %d faces, %d solids, vol %.1f (inverted)' % (len(src2.Faces), len(src2.Solids), src2.Volume))

# ---------------------------------------------------------------- 2. Shell2 rebuild fidelity (native frame)
S1, S2, CB, C2, C3 = W('Shell1 <1>'), W('Shell2 <1>'), W('cover_back <1>'), W('driver_side_palm_cover2 <1>'), W('driver_side_palm_cover3 <1>')
thumb_base = [W(l) for l in ('hand_pulley_wheel_thumb <1>', 'first_thumb_hinge <1>') if l in links]
stacks = [W(o.Label) for o in doc.Objects if o.TypeId == 'App::Link' and o.Label.startswith('shells ')]
stack_lbl = [o.Label for o in doc.Objects if o.TypeId == 'App::Link' and o.Label.startswith('shells ')]
import gc
App.closeDocument(doc.Name)
links = None
gc.collect()
rss('after doc open + parts')
if not SWEEP_ONLY:
    print('== Shell2 B-rep vs 3MF mesh (native frame, before the joint features)')
    nat = M.rebuild_shell2(src2)
    pts, tris = src2.tessellate(0.1)
    P = np.array([[p.x, p.y, p.z] for p in pts])
    rng = np.random.default_rng(1)
    idx = rng.choice(len(P), 80 if QUICK else 200, replace=False)
    sh = nat.Shells[0]
    d = np.array([sh.distToShape(Part.Vertex(V(*P[i])))[0] for i in idx])
    check('rebuild valid, volume within 1.5 %', nat.isValid() and abs(nat.Volume / 34681.8 - 1) < 0.015,
          'vol %.1f vs mesh 34681.8 (%.2f %%)' % (nat.Volume, 100 * (nat.Volume / 34681.8 - 1)))
    print('     vertex->B-rep distance (%d random mesh vertices): median %.3f  p90 %.3f  p95 %.3f  max %.3f mm; %.0f %% within 0.2 mm'
          % (len(d), np.median(d), np.percentile(d, 90), np.percentile(d, 95), d.max(), 100 * (d <= 0.2).mean()))

    # ---------------------------------------------------------------- 3. nominal pose
    rss('after fidelity')
    print('== nominal pose: overlaps and gaps')


    def gap(a, b):
        ba, bb = a.BoundBox, b.BoundBox
        if not ba.intersect(bb) and ba.DiagonalLength > 0:
            # cheap reject: boxes more than 5 mm apart
            dx = max(ba.XMin - bb.XMax, bb.XMin - ba.XMax, 0)
            dy = max(ba.YMin - bb.YMax, bb.YMin - ba.YMax, 0)
            dz = max(ba.ZMin - bb.ZMax, bb.ZMin - ba.ZMax, 0)
            if max(dx, dy, dz) > 5:
                return max(dx, dy, dz), 0.0
        dd = a.distToShape(b)[0]
        ov = a.common(b).Volume if dd < 1e-6 else 0.0
        return dd, ov


    pairs = [('Shell1 x Shell2', S1, S2, 0.2), ('Shell1 x cover2 (was 699.6 mm3)', S1, C2, 0.2),
             ('cover_back x cover3 (was 0.44 mm3)', CB, C3, 0.2), ('Shell2 x cover3 (lip)', S2, C3, 0.2),
             ('Shell1 x cover3', S1, C3, 0.2), ('Shell2 x cover_back', S2, CB, 0.2), ('Shell2 x cover2', S2, C2, 0.2),
             ('Shell1 x cover_back', S1, CB, 0.0)]
    for nm, a, b, need in pairs:
        dd, ov = gap(a, b)
        ok = ov < 1e-3 and dd >= need - 1e-3
        check(nm, ok, 'min gap %.3f mm, overlap %.3f mm3 (need >= %.1f)' % (dd, ov, need))
    for lbl, st in zip(stack_lbl, stacks):
        for nm, other, need in (('Shell2', S2, 0.0), ('Shell1', S1, 0.0)):
            dd, ov = gap(st, other)
            check('%s x %s' % (lbl, nm), ov < 1e-3, 'min gap %.3f, overlap %.3f mm3 (contact allowed: it bears on it)' % (dd, ov))

    # ---------------------------------------------------------------- 4. motion sweep about C

rss('after nominal pose')
print('== sweep: thumb group vs pinky group about C (D-frame axes through C)')
Cw = P1.multVec(V(*M.C))
ax_roll = P1.Rotation.multVec(V(0, 1, 0))
ax_pitch = P1.Rotation.multVec(V(1, 0, 0))
ax_yaw = P1.Rotation.multVec(V(0, 0, 1))


def moved(shape, roll, pitch, yaw):
    s = shape.copy()
    if roll:
        s.rotate(Cw, ax_roll, roll)
    if pitch:
        s.rotate(Cw, ax_pitch, pitch)
    if yaw:
        s.rotate(Cw, ax_yaw, yaw)
    return s


def stud_offset(ys, ts, roll, pitch, yaw):
    """Stud centre on Shell2's top, expressed in Shell2's moving frame, relative to the slot centreline."""
    o, e = M._stud_axis(ys, ts)
    rt = M._shell2_top_r(ys, ts)
    p = P1.multVec(o + e * rt)
    # inverse motion of Shell2 applied to the (fixed) stud point
    q = App.Placement()
    for axis, ang in ((ax_yaw, -yaw), (ax_pitch, -pitch), (ax_roll, -roll)):
        if ang:
            q = App.Placement(V(), App.Rotation(axis, ang), Cw).multiply(q)
    pl = P1.inverse().multVec(q.multVec(p))
    th = math.degrees(math.atan2(pl.x, pl.z))
    return pl.y - ys, th - ts


pinky = {'Shell1': S1, 'cover_back': CB, 'cover2': C2}
pinky.update(dict(zip(stack_lbl, stacks)))
thumb = {'Shell2': S2, 'cover3': C3}
if len(thumb_base) == 2:
    thumb.update({'thumb hub': thumb_base[0], 'thumb yoke': thumb_base[1]})
PAIRS = [('Shell2', k) for k in pinky if not k.startswith('shells ') or 'slider' in k] + [('cover3', 'Shell1'), ('cover3', 'cover_back')] + \
        [(t, p) for t in ('thumb hub', 'thumb yoke') if t in thumb for p in ('Shell1', 'cover_back')]
rolls = [float(x) for x in ROLLS_ENV.split(',')] if ROLLS_ENV else ([M.ROLL[0], 0.0, 7.5, M.ROLL[1]] if QUICK else [M.ROLL[0], 0.0, 5.0, 10.0, M.ROLL[1]])
tilts = [(0, 0), (M.TILT, M.TILT), (M.TILT, -M.TILT), (-M.TILT, M.TILT), (-M.TILT, -M.TILT)]
if (TILTS_ENV or ('3' if QUICK else '5')) == '3':
    tilts = [(0, 0), (M.TILT, M.TILT), (-M.TILT, -M.TILT)]


def overlap(a, b):
    ba, bb = a.BoundBox, b.BoundBox
    if not ba.intersect(bb):
        return 0.0
    return a.common(b).Volume


worst = {}
t0 = time.time()
n_poses = 0
min_margin = 1e9
min_arc = 1e9
for roll in rolls:
    for pitch, yaw in tilts:
        n_poses += 1
        mv = {n: moved(sh_, roll, pitch, yaw) for n, sh_ in thumb.items()}
        gc.collect()
        for tn, pn in PAIRS:
            ov = overlap(mv[tn], pinky[pn])
            k = tn + ' x ' + pn
            if k not in worst or ov > worst[k][0]:
                worst[k] = (ov, roll, pitch, yaw)
        for ys, tsd in M.STUDS:
            dy, dth = stud_offset(ys, tsd, roll, pitch, yaw)
            min_margin = min(min_margin, 6.5 - (M.SLOT_W + abs(dy)))
        lo = max(M.TH_LAND[0], -40.0 + roll)
        hi = min(M.TH_LAND[1], 53.0 + roll)
        min_arc = min(min_arc, hi - lo)
print('     %d poses x %d pairs in %.0fs' % (n_poses, len(PAIRS), time.time() - t0))
for k, (ov, r, p, y) in sorted(worst.items()):
    check('sweep ' + k, ov < 1e-3, 'max overlap %.4f mm3 (worst at roll %.1f pitch %.1f yaw %.1f)' % (ov, r, p, y))
gmin = 1e9
for roll in (M.ROLL[0], M.ROLL[1]):
    for pitch, yaw in tilts[1:]:
        gmin = min(gmin, moved(S2, roll, pitch, yaw).distToShape(S1)[0])
check('Shell1 x Shell2 gap at range corners', gmin > 0.0, 'min gap %.3f mm at roll %s x tilt +-%.1f corners' % (gmin, M.ROLL, M.TILT))
check('retention: slider washer overlaps slot sides', min_margin >= 1.0, 'min overlap %.2f mm over the sweep (>= 1.0)' % min_margin)
check('bearing: seat stays over the land', min_arc >= 30.0, 'min common arc %.1f deg over the roll range' % min_arc)

# ---------------------------------------------------------------- 5. travel stops (stud vs slot ends)
rss('after sweep')
print('== travel stops')
S2d = S2.copy()
S2d.Placement = P1.inverse().multiply(S2d.Placement)
S1d = S1.copy()
S1d.Placement = P1.inverse().multiply(S1d.Placement)


def stud_hits(roll):
    s2 = S2d.copy()
    s2.rotate(V(*M.C), V(0, 1, 0), roll)
    return s2.common(S1d).Volume


for sign, lim in ((-1, M.ROLL[0]), (1, M.ROLL[1])):
    a_ok, a_bad = lim, lim + sign * 4.0
    free_at_limit = stud_hits(lim) < 1e-3
    for _ in range(8):
        mid = 0.5 * (a_ok + a_bad)
        if stud_hits(mid) < 1e-3:
            a_ok = mid
        else:
            a_bad = mid
    check('stop %s' % ('flattening' if sign < 0 else 'cupping'), free_at_limit and abs(a_ok - lim) < 3.0,
          'design limit %.1f deg free: %s; hard stop (first Shell1/Shell2 contact = stud at slot end) at %.2f deg' % (lim, free_at_limit, a_ok))

rss('after stops')
n_fail = sum(1 for _, ok in results if not ok)
print('== SUMMARY: %d checks, %d FAIL' % (len(results), n_fail))
