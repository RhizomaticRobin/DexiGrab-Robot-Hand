"""Verify the V6 pillars module (requirement 6 + TPU clearance fixes) on a headless build.

env: V6_IN = FCStd built with V6_MODULES=j0,pillars (default work/pillars/test.FCStd)
     V6_BASE = the unmodified input (default work/pillars/base.FCStd), for before/after numbers
Checks (all in the assembled pose, via the link placements):
  1 validity / volume of every part pillars touches
  2 halves connected: mid-span piece of every bone (x 7..70, J0 block removed) is ONE solid (V5: two), pillar
    cross-sections in the TPU mid-plane (count, areas, min)
  3 print continuity: every pillar footprint fully backed by material just below and just above the TPU layer
  4 TPU x PLA: overlap volume = 0 for every TPU instance against every palm bone / wrist segment, in-plane clearance
    >= 0.2 (TPU mid-plane section), gap pillar-to-TPU >= 0.2, pegs-to-holes >= 0.2; the coordinator's audit list
  5 TPU palm: every TPU part one solid, webs unchanged, all 8 TPU instances still form one connected sheet
  6 pillars clear of the actuation draft motors (>= 1.2 mm) and of the hardware bracket-fastener keep-outs (>= 1.2 mm)
Exit code 1 if a hard check fails.
"""
import os, sys, json, functools
print = functools.partial(print, flush=True)
sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
import FreeCAD as App, Part
import mod_pillars as MP, n20
V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'
INP = os.environ.get('V6_IN', ROOT + '/work/pillars/test.FCStd')
BASE = os.environ.get('V6_BASE', ROOT + '/work/pillars/base.FCStd')
TOL = 1e-6
FAIL = []

def check(ok, msg):
    print(('  ok   ' if ok else '  FAIL ') + msg)
    if not ok:
        FAIL.append(msg)

# ---------------------------------------------------------------------------------------------- base volumes (sources)
base = App.openDocument(BASE)
import partkeys
bsrc = partkeys.resolve(base)
KEYS = list(MP.BONE_INST) + list(MP.WRIST_INST) + list(MP.TPU_INST)
vol0 = {k: bsrc[k].Shape.Volume for k in KEYS}
import mod_j0
vol_j0 = {k: mod_j0.modify(k, bsrc[k].Shape.copy()).Volume for k in MP.BONE_INST}
App.closeDocument(base.Name)
MP._cache.clear()

doc = App.openDocument(INP)
links = {o.Label: o for o in doc.Objects if o.TypeId == 'App::Link'}
SRC_NAME = dict(MP.TPU_SRC, **MP.WRIST_SRC)
SRC_NAME.update({k: partkeys.PARTS[k][0] for k in MP.BONE_INST})
def part(key):
    o = doc.getObject('V6_' + key)
    return o.Shape if o is not None else doc.getObject(SRC_NAME[key]).Shape
def world(label):
    l = links[label]
    s = l.LinkedObject.Shape.copy(); s.Placement = l.Placement.multiply(s.Placement)
    return s
BONE_LBL = MP.BONE_INST; WRIST_LBL = MP.WRIST_INST
TPU_LBL = [i for v in MP.TPU_INST.values() for i in v]

print('== 1. validity and volumes (mm3)')
for k in KEYS:
    s = part(k)
    ref = vol_j0.get(k, vol0[k])
    check(s.isValid() and len(s.Solids) == 1, '%-13s valid %s solids %d  vol %9.2f  (before %9.2f, %+8.2f)' % (
        k, s.isValid(), len(s.Solids), s.Volume, ref, s.Volume - ref))
for lbl in list(BONE_LBL.values()) + list(WRIST_LBL.values()) + TPU_LBL:
    lk = links[lbl].LinkedObject.Name
    ok = lk.startswith('V6_') or (lbl.startswith('Palm2 <') and lk == 'SRC_Palm2')
    check(ok, 'link %-58s -> %s' % (lbl, lk))

print('== 2. palm and back halves connected (bone-local)')
for key, lbl in BONE_LBL.items():
    b = part(key)
    mid = b.common(Part.makeBox(63.0, 40, 40, V(7.0, -20, -20)))          # x 7..70: no wrist end, no J0 block
    sec = MP._section(b, 'y', 0.5 * (MP.BY0 + MP.BY1))
    faces = sorted(sec.Faces, key=lambda f: f.CenterOfMass.x)
    pil = [f for f in faces if f.CenterOfMass.x < 71.0]
    j0b = [f for f in faces if f.CenterOfMass.x >= 71.0]
    check(len(mid.Solids) == 1, '%-12s mid-span x 7..70 is %d solid(s) (V5: 2 = split halves)' % (key, len(mid.Solids)))
    print('       TPU mid-plane: %d PLA cross-sections left of the J0 block, areas %s mm2 (min %.1f); J0 block %.1f mm2' % (
        len(pil), ', '.join('%.1f@x%.0f' % (f.Area, f.CenterOfMass.x) for f in pil), min(f.Area for f in pil) if pil else 0,
        sum(f.Area for f in j0b)))
for key, lbl in WRIST_LBL.items():
    s = part(key)
    sec = MP._section(s, 'z', 0.5 * (MP.WZ0 + MP.WZ1))
    bar = [f for f in sec.Faces if f.CenterOfMass.y > 4.0]
    print('       %-12s TPU mid-plane PLA %.1f mm2 total; finger-side bar pillar %s' % (
        key, sec.Area, ', '.join('%.1f mm2' % f.Area for f in bar) if bar else 'none (outer side solid)'))

print('== 3. print continuity: pillar footprints fully backed just below / above the TPU layer')
for key in MP.PILLARS:
    b = part(key)
    below = MP._section(b, 'y', MP.BY1 + 0.05); above = MP._section(b, 'y', MP.BY0 - 0.05)
    below.translate(V(0, -(MP.BY1 + 0.05), 0)); above.translate(V(0, -(MP.BY0 - 0.05), 0))
    for i, sp in enumerate(MP.PILLARS[key]):
        f = MP.pillar_face(sp, y=0.0)
        fb = f.common(below).Area / f.Area; fa = f.common(above).Area / f.Area
        check(fb > 0.999 and fa > 0.999, '%-12s pillar %d area %.1f mm2: back half covers %.4f, palm half covers %.4f' % (key, i, f.Area, fb, fa))
segs = MP._wrist_seg_sources()
for key in MP.WRIST_BARS:
    s = part(key)
    f = MP.wrist_bar_face(key, segs[key])
    z0 = f.BoundBox.ZMin
    lo = f.copy(); lo.translate(V(0, 0, (MP.WZ0 - 0.10) - z0)); lo = lo.extrude(V(0, 0, 0.05))
    hi = f.copy(); hi.translate(V(0, 0, (MP.WZ1 + 0.05) - z0)); hi = hi.extrude(V(0, 0, 0.05))
    cl = lo.common(s).Volume / lo.Volume; ch = hi.common(s).Volume / hi.Volume
    check(cl > 0.999 and ch > 0.999, '%-12s bar pillar area %.1f mm2 backed below %.4f / above %.4f' % (key, f.Area, cl, ch))

print('== 4. TPU against PLA (assembled pose)')
W = {l: world(l) for l in list(BONE_LBL.values()) + list(WRIST_LBL.values()) + TPU_LBL}
pairs = []
for t in TPU_LBL:
    for p in list(BONE_LBL.values()) + list(WRIST_LBL.values()):
        if W[t].BoundBox.intersect(W[p].BoundBox):
            pairs.append((t, p))
ZMID = -1.323
def zsec(s):
    return MP._section(s, 'z', ZMID)
S2 = {l: zsec(W[l]) for l in W}
worst_ov, worst_ip = 0.0, 99.0
for t, p in pairs:
    d = W[t].distToShape(W[p])[0]
    ov = W[t].common(W[p]).Volume if d < TOL else 0.0
    ip = S2[t].distToShape(S2[p])[0] if (S2[t].Faces and S2[p].Faces) else 99.0
    worst_ov = max(worst_ov, ov); worst_ip = min(worst_ip, ip)
    if ov > 1e-6 or ip < 0.2 - 1e-4:
        check(False, '%-20s x %-58s overlap %.4f in-plane gap %.3f' % (t, p, ov, ip))
check(worst_ov <= 1e-6, 'all %d TPU x PLA pairs: max overlap %.2e mm3' % (len(pairs), worst_ov))
check(worst_ip >= 0.2 - 1e-4, 'all TPU x PLA pairs: min in-plane gap in the TPU layer %.3f mm (>= 0.2)' % worst_ip)
AUDIT = [('Base Bone 1.2_V02_ringfing <1>', 'palm_bone_flex <1>', 44.46), ('Base Bone 1.2_V02_ringfing <1>', 'Palm3_2 <1>', 23.57),
         ('Base Bone 1.2_V02_pinky <1>', 'palm_bone_flex <1>', 15.25), ('Base Bone 1.2_V02_middlefinger <1>', 'Palm2 <1>', 5.22),
         ('Base Bone 1.2_V02_ringfing <1>', 'Palm2 <2>', 2.04), ('Base Bone 1.2_V02_middlefinger <1>', 'Palm2_2 <1>', 1.44),
         ('Base Bone 1.2_V02_middlefinger <1>', 'Palm1 <1>', 0.09), ('Base Bone 1.2_V02_ringfing <1>', 'Palm2_2 <1>', 0.025),
         ('Palm_bone1 <1>', 'palm_bone_flex <1>', 0.032), ('Palm_bone1 <2>', 'palm_bone_flex <1>', 0.080),
         ('Palm_bone1 <4>', 'palm_bone_flex <1>', 0.047), ('Palm_bone1 <5>', 'palm_bone_flex <1>', 0.031),
         ('Base Bone 1.2_V02_pinky <1>', 'Palm2 <3>', 0.0), ('Base Bone 1.2_V02_pinky <1>', 'Palm3_2 <1>', 0.0)]
print('   coordinator audit list (was -> now):')
for a, b, was in AUDIT:
    d = W[a].distToShape(W[b])[0]
    ov = W[a].common(W[b]).Volume if d < TOL else 0.0
    ip = S2[a].distToShape(S2[b])[0] if (S2[a].Faces and S2[b].Faces) else 99.0
    check(ov <= 1e-6 and ip >= 0.2 - 1e-4, '%-40s x %-18s overlap %6.3f -> %.4f mm3, in-plane gap %.3f' % (a[:40], b, was, ov, ip))
# pillar-to-TPU gap and peg clearance
pl = MP._placements()
gmin = 99.0
for key in MP.PILLARS:
    for sol in MP.bone_pillar_solids(key, y0=MP.BY0, y1=MP.BY1):
        sw = sol.copy(); sw.transformShape(pl[MP.BONE_INST[key]].toMatrix())
        for t in TPU_LBL:
            if sw.BoundBox.intersect(W[t].BoundBox) or True:
                gmin = min(gmin, sw.distToShape(W[t])[0])
for key in MP.WRIST_BARS:
    sol = MP.wrist_pillar_solids(key, segs[key])[0]
    sw = sol.copy(); sw.transformShape(pl[MP.WRIST_INST[key]].toMatrix())
    for t in TPU_LBL:
        gmin = min(gmin, sw.distToShape(W[t])[0])
check(gmin >= 0.2 - 1e-4, 'gap between every pillar and every TPU instance: min %.3f mm (>= 0.2)' % gmin)
pmin = 99.0; npeg = 0
for t in TPU_LBL:
    tl = W[t]
    for f in links[t].LinkedObject.Shape.Faces:
        pass
    # pegs = TPU material more than 0.05 mm beyond the sheet (towards the back, world -z)
    pegs = tl.common(Part.makeBox(400, 400, 5.0, V(-300, -200, -1.623 - 0.05 - 5.0)))
    for pg in pegs.Solids:
        npeg += 1
        for p in list(BONE_LBL.values()) + list(WRIST_LBL.values()):
            if pg.BoundBox.intersect(W[p].BoundBox):
                pmin = min(pmin, pg.distToShape(W[p])[0])
check(pmin >= 0.2 - 1e-4, '%d TPU pegs: min clearance to their holes (radial and tip) %.3f mm (>= 0.2)' % (npeg, pmin))

print('== 5. TPU palm integrity')
for k in MP.TPU_INST:
    s = part(k)
    check(len(s.Solids) == 1 and s.isValid(), '%-10s one valid solid (vol %.2f, was %.2f)' % (k, s.Volume, vol0[k]))
check(abs(part('tpu_web').Volume - vol0['tpu_web']) < 1e-6, 'webs (Palm2 x3) unchanged: vol %.2f' % part('tpu_web').Volume)
sheet = W[TPU_LBL[0]].fuse([W[t] for t in TPU_LBL[1:]])
check(len(sheet.Solids) == 1, 'all %d TPU instances still form one connected sheet (%d solid)' % (len(TPU_LBL), len(sheet.Solids)))

print('== 6. clearance to other workstreams (bone-local, 3D)')
act = json.load(open(ROOT + '/regions/actuation_motors.json'))
env = n20.body_envelope(0.2).fuse([n20.plug_keepout(8.0), Part.makeCylinder(1.7, 11.0, V(-11.0, 0, 0), V(1, 0, 0))]).removeSplitter()
envs = []
for m in act['motors']:
    e = env.copy(); e.transformShape(App.Matrix(*m['matrix'])); envs.append((m['seat'], e))
hw = json.load(open(ROOT + '/regions/hardware.json'))
for key in MP.PILLARS:
    sols = MP.bone_pillar_solids(key, y0=MP.BY0, y1=MP.BY1)
    M = pl[MP.BONE_INST[key]].toMatrix()
    dm, dh = 99.0, 99.0; wm, wh = '', ''
    for i, sol in enumerate(sols):
        sw = sol.copy(); sw.transformShape(M)
        for seat, e in envs:
            if sw.BoundBox.intersect(e.BoundBox) or True:
                d = sw.distToShape(e)[0]
                if d < dm: dm, wm = d, '%s pillar %d' % (seat, i)
        for h in hw:
            if h.get('part') == key and h.get('frame') == 'local':
                b = h['box']; bx = Part.makeBox(b[3] - b[0], b[4] - b[1], b[5] - b[2], V(b[0], b[1], b[2]))
                d = sol.distToShape(bx)[0]
                if d < dh: dh, wh = d, 'pillar %d vs %s' % (i, h['what'][:28])
    check(dm >= 1.2 - 1e-3, '%-12s pillars to actuation draft motor envelopes: min %.2f mm (%s)' % (key, dm, wm))
    check(dh >= 1.2 - 1e-3, '%-12s pillars to hardware keep-outs: min %.2f mm (%s)' % (key, dh, wh or 'none in this bone'))
for key in MP.WRIST_BARS:
    sw = MP.wrist_pillar_solids(key, segs[key])[0].copy(); sw.transformShape(pl[MP.WRIST_INST[key]].toMatrix())
    dm = min(sw.distToShape(e)[0] for _, e in envs)
    check(dm >= 1.2 - 1e-3, '%-12s bar pillar to actuation draft motor envelopes: min %.2f mm' % (key, dm))
print('RESULT: %s (%d failures)' % ('PASS' if not FAIL else 'FAIL', len(FAIL)))
sys.exit(1 if FAIL else 0)
