"""Verify the V6.1 proportions (topic 'proportions') on a built model, headless.

  V6_IN=<built.FCStd> V6_SECT=123456 freecadcmd scripts/verify_proportions.py
env: V6_IN    built model (default work/proportions/test.FCStd)
     V6_SECT  sections to run (default 123456; 7 = combined curl, slow): 1 parts + relinks, 2 assembled joint-to-joint lengths, 3 splice proof,
              4 overlaps and gaps at rest, 5 tactile pads (seated, equal to mod_tactile's own pad at the new length,
              loops on the new joints), 6 hardware (thumb R6 bolt; statuses against the reference build),
              7 combined curl: J1 = J2 = J3 at 45 / 67.5 / 90 deg per finger (fist), against everything
     V6_OUT   json results (default work/proportions/verify_<sections>.json)
     V6_HW_TEST / V6_HW_REF  hardware reports of the test build / of the reference build (section 6)
Range of motion: work/proportions/rom_sweep_v61.py (= scripts/rom_sweep.py reading V6_JOINTS_JSON) with the merged
overrides from mod_proportions.write_merged(); the shared data/v6_joints.json is not written by this topic.
"""
import os, sys, json, math, time, glob
LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
sys.path.insert(0, LIB)
os.environ.setdefault('V6_MODULES', 'j0,pillars,thumb,actuation,shells,fixes,tactile,proportions,hardware')
import FreeCAD as App, Part
import mod_proportions as P
V = App.Vector
FC = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'
WORK = FC + '/work/proportions'
inp = os.environ.get('V6_IN', WORK + '/test.FCStd')
SECT = os.environ.get('V6_SECT', '123456')
out_path = os.environ.get('V6_OUT', WORK + '/verify_%s.json' % SECT)
T0 = time.time()
doc = App.openDocument(inp)
links = {o.Label: o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None}
res = {'model': inp, 'sections': SECT}

def pl_of(label):
    o = links[label]
    return o.Placement.multiply(o.LinkedObject.Shape.Placement)

def world(label):
    o = links[label]
    s = o.LinkedObject.Shape.copy()
    s.Placement = o.Placement.multiply(s.Placement)
    return s

def local(label):
    return links[label].LinkedObject.Shape

def obj(key):
    return doc.getObject('V6_' + key)

def say(*a):
    print(*a); sys.stdout.flush()

def bx(x0, x1):
    return Part.makeBox(x1 - x0, 400, 400, V(x0, -200, -200))

def symdiff_vol(a, b):
    return a.cut(b).Volume + b.cut(a).Volume

PL = P.plan()
PROOF = json.load(open(WORK + '/splice_proof.json'))
VARIANTS = sorted(PROOF['parts'])

# ================================================================================================ 1 parts
if '1' in SECT:
    rows, bad = [], []
    for k in VARIANTS:
        o = obj(k)
        if o is None:
            bad.append('%s missing' % k); continue
        s = o.Shape
        ob = s.optimalBoundingBox(False, False)
        rows.append(dict(key=k, valid=s.isValid(), solids=len(s.Solids), volume=round(s.Volume, 3),
                         x=[round(ob.XMin, 4), round(ob.XMax, 4)]))
        if not (s.isValid() and len(s.Solids) == 1):
            bad.append(k)
    shared = ['V6_' + k for k in P.SHARED_PHALANX] + ['V6_tactile_pad_' + k for k in P.SHARED_PHALANX]
    still = sorted(l for l, o in links.items() if o.LinkedObject.Name in shared)
    relinked = {lab: links[lab].LinkedObject.Name[3:] for lab in P.relink() if lab in links}
    wrong = {lab: k for lab, k in P.relink().items() if lab in links and relinked[lab] != k}
    res['parts'] = dict(n=len(rows), all_valid_single=not bad, bad=bad, rows=rows, instances_on_shared_parts=still,
                        relinked=len(relinked), relink_mismatch=wrong)
    say('[1] %d new parts, all valid single solids: %s %s; instances still on shared phalanx parts: %s; relink mismatches: %s'
        % (len(rows), not bad, bad, still, wrong))

# ================================================================================================ 2 lengths
if '2' in SECT:
    frames = P.joint_frames(PL)
    pub = json.load(open(P.JOINTS_V61)) if os.path.exists(P.JOINTS_V61) else {}     # this module's frames file
    pub = pub.get('joints', pub)
    pub_ok = all(pub.get(m, {}).get('frames', {}).get(lab, {}).get('origin') == v['frames'][lab]['origin']
                 for m, v in frames.items() for lab in v['frames'])
    def conn(a, b, label):
        name, org = P.mate_between(a, b)
        o = org[label]
        fr = frames.get(name, {}).get('frames', {}).get(label)
        if fr:
            o = fr['origin']
        return pl_of(label).multVec(V(*o)), name
    rows, worst = [], 0.0
    for f in P.FINGERS:
        c = P.CHAIN[f]
        t = P.TARGET[f]
        r = dict(finger=f)
        if f != 'thumb':
            base = 'Base Bone 1_V02 <%s>' % {'index': 1, 'middle': 2, 'ring': 4, 'pinky': 3}[f]
            J1, _ = conn(c['meta'][1], base, c['meta'][1])
            J2a, m2 = conn(c['meta'][1], c['prox'][1], c['meta'][1])
            J2b, _ = conn(c['meta'][1], c['prox'][1], c['prox'][1])
            r['meta'] = round((J2a - J1).Length, 4)
            r['J2_mismatch'] = round((J2a - J2b).Length, 5)
            ref_J1 = P.rest_placement(c['meta'][1]).multVec(V(*P.mate_between(c['meta'][1], base)[1][c['meta'][1]]))
            r['J1_moved'] = round((J1 - ref_J1).Length, 5)
        else:
            J2b, _ = conn('Metacarpal Bone_V02 <4>', c['prox'][1], c['prox'][1])
            J2a, _ = conn('Metacarpal Bone_V02 <4>', c['prox'][1], 'Metacarpal Bone_V02 <4>')
            J1, _ = conn('Metacarpal Bone_V02 <4>', 'third_thumb_hinge <1>', 'Metacarpal Bone_V02 <4>')
            r['meta'] = round((J2a - J1).Length, 4)
            r['J2_mismatch'] = round((J2a - J2b).Length, 5)
        J3a, _ = conn(c['prox'][1], c['dist'][1], c['prox'][1])
        J3b, _ = conn(c['prox'][1], c['dist'][1], c['dist'][1])
        r['prox'] = round((J3a - J2b).Length, 4)
        r['J3_mismatch'] = round((J3a - J3b).Length, 5)
        dshape = local(c['dist'][1])
        xmin = dshape.optimalBoundingBox(False, False).XMin
        j3x = P.mate_between(c['prox'][1], c['dist'][1])[1][c['dist'][1]][0]
        r['dist'] = round(j3x - xmin, 4)          # J3 connector to the fingertip (optimal bounding box, part frame)
        # joint angles unchanged: downstream rotations equal the rest rotations
        r['rot_change_deg'] = round(max(math.degrees(pl_of(c[s][1]).Rotation.multiply(P.rest_placement(c[s][1]).Rotation.inverted()).Angle)
                                        for s in ('prox', 'dist')), 6)
        r['target'] = [t['meta'] if t['meta'] is not None else r['meta'], t['prox'], t['dist']]
        errs = [abs(r['meta'] - r['target'][0]), abs(r['prox'] - t['prox']), abs(r['dist'] - t['dist'])]
        r['max_err'] = round(max(errs), 4)
        worst = max(worst, r['max_err'], r['J2_mismatch'], r['J3_mismatch'])
        rows.append(r)
        say('[2] %-6s meta %7.3f  prox %7.3f  dist %7.3f  target %s  err %.4f  J2/J3 mismatch %.5f/%.5f  rot %.1e'
            % (f, r['meta'], r['prox'], r['dist'], r['target'], r['max_err'], r['J2_mismatch'], r['J3_mismatch'], r['rot_change_deg']))
    res['lengths'] = dict(rows=rows, worst=round(worst, 5), pass_0p1=worst <= 0.1, frames_published=pub_ok)
    say('[2] worst %.4f mm (<= 0.1: %s); frames in data/v6_joints_v61.json match: %s' % (worst, worst <= 0.1, pub_ok))

# ================================================================================================ 3 splice proof
def pieces(ops, xlo=-150.0, xhi=150.0):
    """(x0, x1, disp) of the source pieces kept, plus the inserted slabs (x, d)"""
    ins = [o for o in ops if o[0] == 'insert']
    rem = sorted([o for o in ops if o[0] == 'remove'], key=lambda o: o[1])
    if ins:
        x, d = ins[0][1], ins[0][2]
        return [(xlo, x, 0.0), (x, xhi, d)], [(x, d)]
    cuts = [xlo] + [v for o in rem for v in (o[1], o[2])] + [xhi]
    tot = sum(o[2] - o[1] for o in rem)
    out = []
    for i in range(len(rem) + 1):
        a, b = cuts[2 * i], cuts[2 * i + 1]
        if rem[0][3] == 'far':
            disp = -sum(o[2] - o[1] for o in rem[:i])
        else:
            disp = tot - sum(o[2] - o[1] for o in rem[:i])
        out.append((a, b, disp))
    return out, []

def junctions(ops):
    ps, slabs = pieces(ops)
    if slabs:
        x, d = slabs[0]
        return [x, x + d]
    return [p[1] + p[2] for p in ps[:-1]]

if '3' in SECT:
    rows, worst_v, worst_a = [], 0.0, 0.0
    for k in VARIANTS:
        pr = PROOF['parts'][k]
        src = obj(pr['source']).Shape if obj(pr['source']) else None
        new = obj(k).Shape
        ops = [tuple(o) for o in pr['ops']]
        r = dict(key=k, source=pr['source'], ops=pr['ops'])
        ps, slabs = pieces(ops)
        dv = []
        for (a, b, disp) in ps:
            A = src.common(bx(a, b)); A.translate(V(disp, 0, 0))
            B = new.common(bx(a + disp, b + disp))
            dv.append(round(symdiff_vol(A, B), 5))
        for (x, d) in slabs:
            S = P.section(src, x).extrude(V(d, 0, 0))
            B = new.common(bx(x, x + d))
            dv.append(round(symdiff_vol(S, B), 5))
        da = []
        for x in junctions(ops):
            f0, f1 = P.section(new, x - 0.05), P.section(new, x + 0.05)
            da.append(round(P._symdiff(f0, f1), 5))
        r.update(piece_symdiff_mm3=dv, junction_section_symdiff_mm2=da, proof_at_build=[(p.get('symdiff'), p.get('inside_max_symdiff')) for p in pr['proof']],
                 tilted=pr['tilted'])
        worst_v = max(worst_v, max(dv)); worst_a = max(worst_a, max(da))
        rows.append(r)
        say('[3] %-36s pieces %s  junction sections %s  tilted %s' % (k, dv, da, sorted(set(t['tilt_deg'] for t in pr['tilted']))))
    res['splices'] = dict(rows=rows, worst_piece_symdiff_mm3=worst_v, worst_junction_symdiff_mm2=worst_a)
    say('[3] worst piece symmetric difference %.5f mm3, worst junction section difference %.5f mm2' % (worst_v, worst_a))

# ================================================================================================ 4 overlaps / gaps
if '4' in SECT:
    att = {}
    exc = set()
    for fpath in glob.glob(FC + '/regions/attachments_*.json'):
        a = json.load(open(fpath))
        att.update(a.get('attach', {}))
        exc.update(tuple(sorted(p)) for p in a.get('exclude', []))
    moved = set(P.placements()) | set(P.relink())
    SKIP = ('Hardware',)
    cand = {}
    for lab, o in links.items():
        grp = next((g.Name for g in o.InList if g.TypeId == 'App::DocumentObjectGroup'), '')
        if grp in SKIP or not o.LinkedObject.Shape.Solids:
            continue
        cand[lab] = o
    wshape = {}
    def W(lab):
        if lab not in wshape:
            wshape[lab] = world(lab)
        return wshape[lab]
    hits, contacts = [], []
    for m in sorted(moved):
        if m not in cand:
            continue
        a = W(m)
        for lab in cand:
            if lab == m or (lab in moved and lab < m):
                continue
            b_bb = cand[lab].LinkedObject.Shape.BoundBox.transformed(cand[lab].Placement.toMatrix())
            if not a.BoundBox.intersect(b_bb):
                continue
            b = W(lab)
            v = a.common(b).Volume
            if v > 1e-4:
                pair = tuple(sorted((m, lab)))
                ray = lambda n: n.split('_')[2] if n.startswith('tactile_') and len(n.split('_')) > 2 else None
                fpc = (all(n.startswith('tactile_') for n in pair) and ray(pair[0]) == ray(pair[1]) and v <= 0.02
                       and any(n.startswith(('tactile_loop_', 'tactile_knuckle_')) for n in pair))
                kind = ('excluded (flexible crossing)' if pair in exc else 'FPC splice butt joint (loop / ribbon at its pad\'s sheet end)' if fpc
                        else ('pad on its bone' if att.get(m) == lab or att.get(lab) == m else 'COLLISION'))
                (hits if kind == 'COLLISION' else contacts).append(dict(pair=pair, mm3=round(v, 4), kind=kind))
    # joint gaps: new pose vs the reference relative pose (the shared source shapes at the V6 rest poses)
    SRC_OF = {c[1]: c[0] for f2 in P.FINGERS for c in P.CHAIN[f2].values() if c}
    def refshape(lab):
        if lab in SRC_OF and obj(SRC_OF[lab]) is not None:
            s = obj(SRC_OF[lab]).Shape.copy()
            s.Placement = P.rest_placement(lab)
            return s
        return W(lab)                          # instance not touched by this module
    gaps = []
    for f in P.FINGERS:
        c = P.CHAIN[f]
        chain = ([('Base Bone 1_V02 <%s>' % {'index': 1, 'middle': 2, 'ring': 4, 'pinky': 3}[f], None)] if f != 'thumb'
                 else [('Metacarpal Bone_V02 <4>', None)])
        pairs = []
        if f != 'thumb':
            pairs.append((chain[0][0], c['meta'][1]))
            pairs.append((c['meta'][1], c['prox'][1]))
        else:
            pairs.append(('Metacarpal Bone_V02 <4>', c['prox'][1]))
        pairs.append((c['prox'][1], c['dist'][1]))
        for a_lab, b_lab in pairs:
            dn = W(a_lab).distToShape(W(b_lab))[0]
            dr = refshape(a_lab).distToShape(refshape(b_lab))[0]
            gaps.append(dict(pair=[a_lab, b_lab], gap_new=round(dn, 4), gap_reference=round(dr, 4), same=abs(dn - dr) < 1e-3))
    res['overlaps'] = dict(collisions=hits, designed_contacts=contacts, joint_gaps=gaps)
    say('[4] collisions of moved / relinked instances: %d %s' % (len(hits), hits[:6]))
    say('[4] designed contacts (pads on bones, excluded flexible crossings): %d, max %.4f mm3' % (len(contacts), max([c['mm3'] for c in contacts] or [0])))
    for g in gaps:
        say('[4] gap %-44s new %.4f  reference %.4f  %s' % (' x '.join(g['pair']), g['gap_new'], g['gap_reference'], 'same' if g['same'] else 'CHANGED'))

# ================================================================================================ 5 tactile
if '5' in SECT:
    import mod_tactile as T
    origin = P.patch_tactile(T)
    rows = []
    for f in P.FINGERS:
        for seg in P.SEGS:
            if P.CHAIN[f][seg] is None or P.TARGET[f][seg] is None:
                continue
            key = P.bone_key(f, seg)
            pk = P.pad_key(f, seg)
            inst = P.pad_instance(f, seg)
            if pk not in [o.Name[3:] for o in doc.Objects if o.Name.startswith('V6_proportions_pad')]:
                continue
            spliced = obj(pk).Shape
            regen = T.pad_part(key)
            dv = symdiff_vol(spliced, regen)
            pad_w, bone_w = world(inst), world(P.CHAIN[f][seg][1])
            ov = pad_w.common(bone_w).Volume
            dist = pad_w.distToShape(bone_w)[0]
            rows.append(dict(pad=inst, part=pk, symdiff_vs_mod_tactile_mm3=round(dv, 5), vol=round(spliced.Volume, 3),
                             vol_mod_tactile=round(regen.Volume, 3), overlap_with_bone_mm3=round(ov, 5), dist_to_bone=round(dist, 5)))
            say('[5] %-26s vs mod_tactile pad at the new length: %.5f mm3 (vol %.2f / %.2f); on bone: overlap %.5f mm3, distance %.5f'
                % (inst, dv, spliced.Volume, regen.Volume, ov, dist))
    loops = []
    for f in P.FINGERS:
        for kind in ('J2', 'J3'):
            lab = P.loop_instance(f, kind)
            if lab not in links:
                continue
            want, _ = T.joint_frame(f, kind)
            got = links[lab].Placement
            dd = (got.Base - want.Base).Length
            da = math.degrees(got.Rotation.multiply(want.Rotation.inverted()).Angle)
            loops.append(dict(loop=lab, offset=round(dd, 5), angle=round(da, 6)))
            say('[5] %-24s on the new %s joint frame: offset %.5f mm, angle %.6f deg' % (lab, kind, dd, da))
    res['tactile'] = dict(pads=rows, loops=loops,
                          worst_pad_symdiff=max(r['symdiff_vs_mod_tactile_mm3'] for r in rows) if rows else None,
                          worst_pad_overlap=max(r['overlap_with_bone_mm3'] for r in rows) if rows else None,
                          worst_loop_offset=max(l['offset'] for l in loops) if loops else None)

# ================================================================================================ 6 hardware
if '6' in SECT:
    tp = os.environ.get('V6_HW_TEST', WORK + '/hardware_report_test.json')
    rp = os.environ.get('V6_HW_REF', WORK + '/hw_report_reference.json')
    t = {j['id']: j for j in json.load(open(tp))['joints']}
    r = {j['id']: j for j in json.load(open(rp))['joints']}
    diff = []
    for jid in sorted(set(t) | set(r)):
        a, b = t.get(jid), r.get(jid)
        if not a or not b or a['status'] != b['status']:
            diff.append(dict(id=jid, test=a and a['status'], reference=b and b['status']))
    r6t, r6r = t.get('R_thumb_R6_bolt'), r.get('R_thumb_R6_bolt')
    moved = (V(*r6t['seat_world']) - V(*r6r['seat_world'])).Length if r6t and r6r else None
    # the moved request must sit on the new R6 axis: distance from the thumb distal's R6 axis line
    dl = 'Distal Phalanx Bone_V02 <5>'
    pl = pl_of(dl)
    p0, ax = pl.multVec(V(29.3, 0, 0)), pl.Rotation.multVec(V(0, 1, 0))
    seat = V(*r6t['seat_world']) if r6t else None
    off_axis = ((seat - p0) - ax * (seat - p0).dot(ax)).Length if seat else None
    t2, t23 = P.shifts('thumb')
    res['hardware'] = dict(status_differences=diff, R6_seat_moved=round(moved, 4) if moved else None, R6_expected_move=round(t23.Length, 4),
                           R6_seat_off_new_axis=round(off_axis, 4) if off_axis is not None else None,
                           R6_status=(r6t or {}).get('status'), R6_status_reference=(r6r or {}).get('status'),
                           R6_problems=(r6t or {}).get('problems'))
    say('[6] joints with a different status than the reference build: %s' % diff)
    say('[6] thumb R6 bolt: seat moved %.3f mm (expected %.3f), %.4f mm off the moved R6 axis; status %s (reference %s)'
        % (moved or -1, t23.Length, off_axis if off_axis is not None else -1, res['hardware']['R6_status'], res['hardware']['R6_status_reference']))

# ================================================================================================ 7 combined curl (fist)
if '7' in SECT:
    # every finger J1 = J2 = J3 = s * 90 deg of flexion together (rom_sweep moves one joint at a time); the pads and loops
    # ride on their bones (regions/attachments_*.json); checked against every other instance (hardware skipped, like
    # rom_sweep) and within the finger (non-adjacent bones).  Joint frames: definition + merged frames (v6_joints.json + v6_joints_v61.json).
    d = json.load(open(P.ROOT + '/data/top_assembly_definition.json'))
    ra = d['rootAssembly']
    inst = {i['id']: i for i in ra['instances']}
    for sa in d['subAssemblies']:
        for i in sa['instances']:
            inst[i['id']] = i
    ov = P.merged_joints(); ov = ov.get('joints', ov)     # shared overrides + data/v6_joints_v61.json
    if os.environ.get('V6_NO_FRAMES'):             # reference (pre-proportions) model: V5 connector origins
        ov = {}
    MATES = {}
    for f_ in ra['features']:
        fd = f_['featureData']
        if f_['featureType'] != 'mate' or fd.get('mateType') != 'REVOLUTE':
            continue
        ents = []
        for e in fd['matedEntities']:
            cs = e['matedCS']
            nm = inst[e['matedOccurrence'][-1]]['name']
            org = V(*[c * 1000 for c in cs['origin']])
            fr = ov.get(fd['name'], {}).get('frames', {}).get(nm)
            if fr and 'origin' in fr:
                org = V(*fr['origin'])
            ents.append((nm, org, V(*cs['zAxis'])))
        lim = fd.get('mateLimits', {})
        MATES[fd['name']] = dict(ents=ents, lo=math.degrees(lim.get('limitAxialZMin', 0)), hi=math.degrees(lim.get('limitAxialZMax', 0)))
    att = {}
    exc = set()
    for fpath in glob.glob(FC + '/regions/attachments_*.json'):
        a = json.load(open(fpath))
        att.update(a.get('attach', {}))
        exc.update(tuple(sorted(p)) for p in a.get('exclude', []))
    fixed_all = {}
    for lab, o in links.items():
        grp = next((g.Name for g in o.InList if g.TypeId == 'App::DocumentObjectGroup'), '')
        if grp != 'Hardware' and o.LinkedObject.Shape.Solids:
            fixed_all[lab] = o
    def flex_value(name):
        m = MATES[name]
        lo, hi = m['lo'], m['hi']
        if hi - lo >= 359 or hi > 180:                  # 270..360 style: flexed end is 270 = -90
            return -90.0
        return hi if abs(hi) > abs(lo) else lo
    rows = []
    for f in [x for x in P.FINGERS if x != 'thumb']:
        c = P.CHAIN[f]
        base = 'Base Bone 1_V02 <%s>' % {'index': 1, 'middle': 2, 'ring': 4, 'pinky': 3}[f]
        chain = [(P.mate_between(c['meta'][1], base)[0], c['meta'][1]),
                 (P.mate_between(c['meta'][1], c['prox'][1])[0], c['prox'][1]),
                 (P.mate_between(c['prox'][1], c['dist'][1])[0], c['dist'][1])]
        riders = {lab: [r for r, host in att.items() if host == lab and r in links] for _, lab in chain}
        for s_ in (0.5, 0.75, 1.0):
            pls = {lab: links[lab].Placement for lab in [b for _, b in chain] + sum(riders.values(), [])}
            for k, (mname, child) in enumerate(chain):
                m = MATES[mname]
                (na, oa, za), (nb, ob, zb) = m['ents']
                pa = pls.get(na, links[na].Placement)
                piv, ax = pa.multVec(oa), pa.Rotation.multVec(za)
                sign = 1.0 if child == na else -1.0
                R = App.Placement(V(0, 0, 0), App.Rotation(ax, sign * s_ * flex_value(mname)), piv)
                for _, lab in chain[k:]:
                    for x in [lab] + riders[lab]:
                        pls[x] = R.multiply(pls[x])
            moving = list(pls)
            ws = {}
            for x in moving:
                sh = links[x].LinkedObject.Shape.copy(); sh.Placement = pls[x].multiply(sh.Placement); ws[x] = sh
            hits = []
            for x in moving:
                for y in fixed_all:
                    if y in pls or tuple(sorted((x, y))) in exc:
                        continue
                    yb = links[y].LinkedObject.Shape.BoundBox.transformed(links[y].Placement.toMatrix())
                    if not ws[x].BoundBox.intersect(yb):
                        continue
                    v = ws[x].common(world(y)).Volume
                    if v > 0.5:
                        hits.append('%s x %s %.1f' % (x, y, v))
            for i, x in enumerate(moving):             # within the finger (joint partners included, as in rom_sweep)
                for y in moving[i + 1:]:
                    pair = tuple(sorted((x, y)))
                    if pair in exc or att.get(x) == y or att.get(y) == x or att.get(x, x) == att.get(y, y):
                        continue
                    if ws[x].BoundBox.intersect(ws[y].BoundBox):
                        v = ws[x].common(ws[y]).Volume
                        if v > 0.5:
                            hits.append('%s x %s %.1f' % (x, y, v))
            # fingertip clearance to the palm pad / palm bone at this curl
            tip = ws[c['dist'][1]]
            palm_lab = {'index': 'tactile_pad_index_palm', 'middle': 'tactile_pad_middle_palm', 'ring': 'tactile_pad_ring_palm',
                        'pinky': 'tactile_pad_pinky_palm'}[f]
            dpalm = tip.distToShape(world(palm_lab))[0] if palm_lab in links else None
            rows.append(dict(finger=f, curl=s_, deg=round(90 * s_, 1), hits=hits, tip_to_palm_pad=round(dpalm, 2) if dpalm is not None else None))
            say('[7] %-6s J1=J2=J3=%4.1f deg: %s  distal to its palm pad %.2f mm' % (f, 90 * s_, 'CLEAR' if not hits else 'HITS %s' % hits[:3], dpalm if dpalm is not None else -1))
    res['fist'] = rows

res['seconds'] = round(time.time() - T0, 1)
json.dump(res, open(out_path, 'w'), indent=1)
say('written %s (%.0fs)' % (out_path, time.time() - T0))
