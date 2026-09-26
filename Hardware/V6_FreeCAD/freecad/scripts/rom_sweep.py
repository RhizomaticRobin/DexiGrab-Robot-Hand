"""Range-of-motion sweep for every revolute joint of the placed model (headless).

Joint frames come from the Onshape assembly definition (matedCS of both entities, part-local, with x axes),
mapped to world with the links' placements. The joint value is the rotation of entity 2's frame relative to
entity 1's about entity 1's z; the current value theta0 is measured from the placed model. The palm
(groups Palm_rigid, Palm_TPU, Shells, Wrist_segments) is the fixed root; each joint moves its downstream
subtree through [limit min, limit max] (others held) and is checked against every other part.
env: V6_IN, V6_ROM_OUT (csv), V6_STEPS (default 9), V6_TOL (overlap mm3 counted as collision, default 0.5),
     V6_JOINTS (comma list of mate names, optional)
"""
import os, json, csv, math, time
import FreeCAD as App, Part
R = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
inp = os.environ.get('V6_IN', R + '/freecad/DexiGrab_V6.FCStd')
out = os.environ.get('V6_ROM_OUT', R + '/freecad/work/rom.csv')
STEPS = int(os.environ.get('V6_STEPS', '9'))
TOL = float(os.environ.get('V6_TOL', '0.5'))
ONLY = set(filter(None, os.environ.get('V6_JOINTS', '').split(',')))
doc = App.openDocument(inp)
links = {}
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.LinkedObject is not None and o.LinkedObject.Shape.Solids:
        s = o.LinkedObject.Shape.copy(); s.Placement = o.Placement.multiply(s.Placement)
        grp = next((g.Name for g in o.InList if g.TypeId == 'App::DocumentObjectGroup'), '')
        links[o.Label] = (o, s, grp)
d = json.load(open(R + '/data/top_assembly_definition.json'))
ra = d['rootAssembly']
inst = {i['id']: i for i in ra['instances']}
for sa in d['subAssemblies']:
    for i in sa['instances']:
        inst[i['id']] = i
def V(v): return App.Vector(*v)
joints = []
for f in ra['features']:
    fd = f['featureData']
    if f['featureType'] != 'mate' or fd['mateType'] != 'REVOLUTE':
        continue
    ents = []
    for e in fd['matedEntities']:
        name = inst[e['matedOccurrence'][-1]]['name']
        cs = e['matedCS']
        ents.append((name, V([c * 1000 for c in cs['origin']]), V(cs['xAxis']), V(cs['zAxis'])))
    lim = fd.get('mateLimits', {})
    joints.append(dict(name=fd['name'], ents=ents, lo=math.degrees(lim.get('limitAxialZMin', -math.pi)), hi=math.degrees(lim.get('limitAxialZMax', math.pi))))
# V6 overrides (shared with the Leap session's exporter): data/v6_joints.json, keyed by mate name, optionally under
# "joints"; per joint {"lo": deg, "hi": deg} (or limit_min/limit_max) in Onshape mate-value degrees.
_ov_path = R + '/data/v6_joints.json'
if os.path.exists(_ov_path):
    _ov = json.load(open(_ov_path))
    _ov = _ov.get('joints', _ov)
    for j in joints:
        o = _ov.get(j['name'])
        if isinstance(o, dict):
            j['lo'] = float(o.get('lo', o.get('limit_min', j['lo'])))
            j['hi'] = float(o.get('hi', o.get('limit_max', j['hi'])))
            fr = o.get('frames', {})
            unknown = set(fr) - {e[0] for e in j['ents']}
            if unknown:      # same policy as the Leap exporter: a typo is a hard error, not a silent skip
                raise KeyError('v6_joints.json: %s frames name unknown instance(s) %s (expected %s)' % (j['name'], sorted(unknown), [e[0] for e in j['ents']]))
            if fr:
                ents = []
                for (en, eo, ex, ez) in j['ents']:
                    f = fr.get(en)
                    if f:
                        eo = V(f['origin']) if 'origin' in f else eo
                        ex = V(f['x']) if 'x' in f else ex
                        ez = V(f['z']) if 'z' in f else ez
                    ents.append((en, eo, ex, ez))
                j['ents'] = ents
            print('override', j['name'], j['lo'], j['hi'], 'frames' if fr else '')

_OCC = None
def theta_cad(j):
    """atan2 joint angle at the original Onshape pose (occurrence transforms in the definition JSON)."""
    global _OCC
    if _OCC is None:
        _OCC = {}
        for o in ra['occurrences']:
            t = o['transform']
            _OCC[inst[o['path'][-1]]['name']] = App.Placement(App.Matrix(t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000, t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))
    (na, oa, xa, za), (nb, ob, xb, zb) = j['ents']
    pa, pb = _OCC[na], _OCC[nb]
    xaw, zaw, xbw = pa.Rotation.multVec(xa), pa.Rotation.multVec(za), pb.Rotation.multVec(xb)
    return math.degrees(math.atan2(xaw.cross(xbw).dot(zaw), xaw.dot(xbw)))

def world_frame(label, o, x, z):
    lk = links[label][0]
    pl = lk.Placement.multiply(lk.LinkedObject.Shape.Placement)
    return pl.multVec(o), pl.Rotation.multVec(x), pl.Rotation.multVec(z)
ROOT_GROUPS = {'Palm_rigid', 'Palm_TPU', 'Shells', 'Wrist_segments', 'Motors', 'Electronics', 'Wiring', 'Battery'}
SKIP_GROUPS = {'Hardware'}          # fasteners are validated by the hardware/thumb checks, not by the sweep
# attachments: non-mated instances that ride with a mated one; exclusions: pairs that are not collisions
ATTACH, EXCLUDE = {}, set()
import glob
for fpath in glob.glob(R + '/freecad/regions/attachments_*.json'):
    try:
        a = json.load(open(fpath))
        ATTACH.update(a.get('attach', {}))
        EXCLUDE.update(tuple(sorted(p)) for p in a.get('exclude', []))
    except Exception as e:
        print('bad attachments file', fpath, e)
links = {l: v for l, v in links.items() if v[2] not in SKIP_GROUPS}
adj = {}
# mate groups: members move rigidly together (e.g. Group 1 = the thumb hub + yoke of Assem5)
for f in ra['features']:
    if f['featureType'] == 'mateGroup':
        members = []
        for occ in f['featureData']['occurrences']:
            path = occ['occurrence']
            for o2 in ra['occurrences']:
                if o2['path'][:len(path)] == path and inst[o2['path'][-1]]['type'] == 'Part':
                    members.append(inst[o2['path'][-1]]['name'])
        for m1 in members:
            for m2 in members:
                if m1 != m2:
                    adj.setdefault(m1, []).append((m2, {'name': 'group', 'rigid': True}))
for j in joints:
    a, b = j['ents'][0][0], j['ents'][1][0]
    adj.setdefault(a, []).append((b, j)); adj.setdefault(b, []).append((a, j))
root = {l for l, (_, _, g) in links.items() if g in ROOT_GROUPS}
parent_joint, seen, frontier = {}, set(root), list(root)
while frontier:
    n = frontier.pop()
    for m, j in adj.get(n, []):
        if m not in seen:
            seen.add(m); parent_joint[m] = (j, n); frontier.append(m)
children = {}
for c, (j, p) in parent_joint.items():
    children.setdefault(p, []).append(c)
for c, p in ATTACH.items():
    if c in links:
        children.setdefault(p, []).append(c)
def subtree(n):
    res, st = [], [n]
    while st:
        x = st.pop(); res.append(x); st.extend(children.get(x, []))
    return res
rows = []
t0 = time.time()
for child, (j, par) in sorted(parent_joint.items(), key=lambda kv: int(''.join(c for c in kv[1][0]['name'] if c.isdigit()) or 0)):
    if j.get('rigid') or (ONLY and j['name'] not in ONLY):
        continue
    (na, oa, xa, za), (nb, ob, xb, zb) = j['ents']
    pa, xaw, zaw = world_frame(na, oa, xa, za)
    pb, xbw, zbw = world_frame(nb, ob, xb, zb)
    atan_now = math.degrees(math.atan2(xaw.cross(xbw).dot(zaw), xaw.dot(xbw)))
    # V6 joint-value convention (shared with the Leap exporter): value = theta_CAD + child rotation from its CAD pose
    # (in this script's sign convention), theta_CAD = atan2 at the ORIGINAL Onshape pose. For a re-posed joint the
    # current value is therefore 2*theta_CAD - atan2_now (re-measuring atan2 alone gives the wrong sign).
    tc = theta_cad(j)
    theta0 = 2 * tc - atan_now
    theta0 = (theta0 + 180) % 360 - 180
    sign = 1.0 if child == na else -1.0     # Onshape: value = rotation of entity 1 relative to entity 2 (checked on the finger flexion joints)
    lo, hi = j['lo'], j['hi']
    if hi - lo >= 359:
        lo, hi = -180.0, 180.0
    moving = [l for l in subtree(child) if l in links]
    fixed = [l for l in links if l not in moving]
    bad = []
    for k in range(STEPS):
        ang = lo + (hi - lo) * k / (STEPS - 1)
        delta = ang - theta0
        # bring angles like 270..360 onto the current branch
        delta = (delta + 180) % 360 - 180
        Rm = App.Placement(App.Vector(0, 0, 0), App.Rotation(zaw, sign * delta), pa)
        hits = []
        for m in moving:
            sm = links[m][1].copy(); sm.Placement = Rm.multiply(sm.Placement)
            bm = sm.BoundBox
            for f2 in fixed:
                sf = links[f2][1]
                if not bm.intersect(sf.BoundBox) or tuple(sorted((m, f2))) in EXCLUDE:
                    continue
                v = sm.common(sf).Volume
                if v > TOL:
                    hits.append('%s x %s %.1f' % (m, f2, v))
        rows.append([j['name'], child, round(ang, 1), len(hits), '; '.join(hits)])
        if hits:
            bad.append((round(ang, 1), hits))
    print('%-12s %-42s now %6.1f  range [%6.1f,%6.1f]  moving %d  %s' % (j['name'], child, theta0, lo, hi, len(moving), 'CLEAR' if not bad else 'COLLIDES at %s' % [a for a, _ in bad]))
    for a, h in bad[:3]:
        print('      %6.1f deg: %s' % (a, '; '.join(h[:3])))
with open(out, 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['joint', 'child', 'angle_deg', 'n_hits', 'hits']); w.writerows(rows)
print('done in %.0fs' % (time.time() - t0))
