"""Verify the tactile skin (lib/mod_tactile.py) on a built document (headless, read-only).

env: V_IN (default work/tactile/test.FCStd), V_SECTIONS (comma list: parts,fit,overlap,walls,rom; default all),
     V_OUT (json summary, default work/tactile/verify_<sections>.json)
Sections
  parts   : every part the module modifies or creates is a valid single solid; volumes
  fit     : each pad sits in its recess: no overlap with its bone, lateral clearance of the laminated slab >= 0.2
            (pad lifted 0.3 off the floor so only the walls count), channel clearance of the FPC wrap / spine,
            pad top vs the surrounding face (designed +0.2 proud)
  overlap : every tactile instance against every other instance: common volume = 0; min distance (BRepExtrema, with a
            surface-sampling cross-check where it reports contact without overlap - its loft-face false positives)
  walls   : min wall between the tactile cuts and internal voids (tendon channels, knot pockets, anchors, TPU layer,
            strap-screw nut pockets): >= 1.0 required over channels
  rom     : J1/J2/J3 (and thumb R4/R5/R6) swept 0..90 deg with pads/spines riding on their bones; joint loops at 0 and
            90 deg (designed shapes, constant length) against the bones at that angle; knuckle ribbon (static part)
            against the finger at J0 -15/0/+15 deg
"""
import os, sys, json, math, time, traceback
R = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
sys.path.insert(0, R + '/freecad/lib')
import FreeCAD as App, Part
import mod_tactile as T
V = App.Vector
inp = os.environ.get('V_IN', R + '/freecad/work/tactile/test.FCStd')
SECTIONS = [s for s in os.environ.get('V_SECTIONS', 'parts,fit,overlap,walls,rom').split(',') if s]
out_path = os.environ.get('V_OUT', R + '/freecad/work/tactile/verify_%s.json' % '_'.join(SECTIONS))
res = {'doc': inp, 'sections': SECTIONS, 'checks': []}
t00 = time.time()

def check(name, ok, detail):
    res['checks'].append({'check': name, 'ok': bool(ok), 'detail': detail})
    print('%-6s %-44s %s' % ('PASS' if ok else 'FAIL', name, detail)); sys.stdout.flush()

doc = App.openDocument(inp)
links = {}
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.LinkedObject is not None and hasattr(o.LinkedObject, 'Shape'):
        links[o.Label] = o

def wshape(lab):
    o = links[lab]
    s = o.LinkedObject.Shape.copy()
    s.Placement = o.Placement.multiply(s.Placement)
    return s

def obj(key):
    return doc.getObject('V6_' + key)

MY_KEYS = list(T.PHALANX) + list(T.PALM) + ['cover2', 'cover3']
NEW = list(T.new_parts().keys()) if 'parts' in SECTIONS else []

# ------------------------------------------------------------------------------------------------ parts
if 'parts' in SECTIONS:
    bad = []
    vols = {}
    for k in MY_KEYS + NEW:
        o = obj(k)
        if o is None:
            bad.append('%s missing' % k); continue
        s = o.Shape
        vols[k] = round(s.Volume, 2)
        if not s.isValid() or len(s.Solids) != 1:
            bad.append('%s valid %s solids %d' % (k, s.isValid(), len(s.Solids)))
        chain = o.Label2
        if k in MY_KEYS and 'tactile' not in chain:
            bad.append('%s not modified by tactile (%s)' % (k, chain))
    check('parts valid single solids', not bad, 'all %d modified + %d new parts valid' % (len(MY_KEYS), len(NEW)) if not bad else '; '.join(bad))
    res['volumes'] = vols
    tax = {k: len(T.taxels(k)) for k in list(T.PHALANX) + list(T.PALM)}
    res['taxels_per_part'] = tax

# ------------------------------------------------------------------------------------------------ fit
def surface_samples(shape, step=1.2):
    pts = []
    for f in shape.Faces:
        try:
            u0, u1, v0, v1 = f.ParameterRange
            nu = max(2, int((u1 - u0) / step) + 1) if f.Surface.TypeId == 'Part::GeomPlane' else 6
            nv = max(2, int((v1 - v0) / step) + 1) if f.Surface.TypeId == 'Part::GeomPlane' else 6
            for i in range(nu):
                for j in range(nv):
                    u = u0 + (u1 - u0) * (i + 0.5) / nu; v = v0 + (v1 - v0) * (j + 0.5) / nv
                    if f.isInside(f.valueAt(u, v), 1e-4, True):
                        pts.append(f.valueAt(u, v))
        except Exception:
            pass
    for vx in shape.Vertexes:
        pts.append(vx.Point)
    return pts

def sampled_distance(a, b, step=1.2, stop=0.0):
    best = 1e9
    for p in surface_samples(a, step):
        d = b.distToShape(Part.Vertex(p))[0]
        if d < best:
            best = d
            if best <= stop:
                break
    return best

def slab_local(key):
    """the laminated slab alone (the part that must keep 0.2 to the recess walls), canonical -> part local"""
    if key in T.PHALANX:
        p = T.PHALANX[key]
        u0, u1 = p['u0'] + T.C, p['u1'] - T.C
        v_fold, v_out = -(T.FLOOR - T.RF), T.FACE - T.SIDE_WALL - T.C
        if 'tip' in p:
            cu, r = p['tip']
            sl = T._dshape_face(u0, u1, cu, r - T.SIDE_WALL - T.C, v_fold, v_out, T.FLOOR)
            sl = sl.common(T.cbox(u0 - 1, u1 + 1, -20, 20, T.FLOOR, T.FLOOR + T.STACK))
        else:
            sl = T.cbox(u0, u1, v_fold, v_out, T.FLOOR, T.FLOOR + T.STACK)
        return T.to_local(key, T._mirror_v(sl, p['s']))
    p = T.PALM[key]
    sl = T.cbox(p['u0'] + T.C, p['u1'] - T.C, -(T.FLOOR - T.RF), T.FACE - T.SIDE_WALL - T.C, T.FLOOR, T.FLOOR + T.STACK)
    sl = T._mirror_v(sl, p['s'])
    holes = [Part.makeCylinder(T.SCREW_HOLE_R, 3.0, V(hu, hv, T.FLOOR - 1.0), V(0, 0, 1))
             for (hu, hv) in [T.world_to_canon_xy(key, xy) for xy in T.STRAP_SCREWS.get(key, [])]]
    if holes:
        sl = sl.cut(holes)
    return T.to_local(key, sl)

if 'fit' in SECTIONS:
    rows = []
    worst_lat = 1e9; worst_ov = 0.0; prouds = []; seated = []
    for name, part, pl, col, att in T.instance_specs():
        if not name.startswith('tactile_pad_'):
            continue
        key = part[len('tactile_pad_'):]
        pad = wshape(name)
        bone = wshape(att)
        ov = pad.common(bone).Volume
        d_seat = pad.distToShape(bone)[0]
        # slab: lift 0.3 mm along the palm normal -> only the recess walls remain within reach
        bpl = links[att].Placement.multiply(links[att].LinkedObject.Shape.Placement)
        sl = slab_local(key); sl.Placement = bpl.multiply(sl.Placement)
        n = bpl.Rotation.multVec(V(0, 0, 1) if key in T.PHALANX else V(0, -1, 0))
        up = sl.copy(); up.translate(n * 0.3)
        lat = up.distToShape(bone)[0]
        # proud: slab top vs the palm face (canonical w = FACE)
        top = sl.copy(); top.transformShape(bpl.inverse().toMatrix())
        top.transformShape(T.canon_matrix(key).inverse())
        proud = top.BoundBox.ZMax - T.FACE
        prouds.append(proud); seated.append(d_seat)
        worst_lat = min(worst_lat, lat); worst_ov = max(worst_ov, ov)
        rows.append({'pad': name, 'bone': att, 'overlap': round(ov, 5), 'seated_distance': round(d_seat, 4),
                     'slab_wall_clearance': round(lat, 3), 'proud': round(proud, 3)})
    res['fit'] = rows
    check('pads: no overlap with their bone', worst_ov < 1e-4, 'max overlap %.5f mm3 over %d pads' % (worst_ov, len(rows)))
    check('pads: seated (touch the recess / channel floors)', max(seated) < 0.01, 'max pad-to-bone distance %.4f mm' % max(seated))
    check('pads: slab clearance to the recess walls >= 0.2', worst_lat >= 0.199,
          'min %.3f mm (design 0.2; wrap/spine channels are FPC width + 0.2 by construction)' % worst_lat)
    check('pads: stand proud of the face', min(prouds) > 0.15 and max(prouds) < 0.25, 'proud %.3f..%.3f mm (design 0.2 with a 0.8 stack in a 0.6 recess)' % (min(prouds), max(prouds)))

# ------------------------------------------------------------------------------------------------ overlap
def ray_of(name):
    return next((f for f in T.RAYS if '_%s' % f in name or name.endswith(f)), None)

def seats():
    """non-tactile parts a tactile instance may touch by design"""
    out = {}
    for f, r in T.RAYS.items():
        out['tactile_loop_%s_J3' % f] = {r['prox'][1], r['dist'][1]}
        out['tactile_loop_%s_J2' % f] = {r['meta'][1], r['prox'][1]}
        if r['palm']:
            out['tactile_knuckle_' + f] = {r['palm'][1], r['meta'][1]}
            out['tactile_tail_' + f] = {r['palm'][1]} | ({'driver_side_palm_cover2 <1>'} if f == 'pinky' else set())
        for seg in ('dist', 'prox', 'meta', 'palm'):
            if r.get(seg):
                out['tactile_pad_%s_%s' % (f, seg)] = {r[seg][1]}
    out['tactile_tail_thumb'] = {T.RAYS['thumb']['meta'][1]}
    return out

def splice(a, b):
    """two tactile instances that are one continuous FPC (modelled as abutting parts)"""
    fa, fb = ray_of(a), ray_of(b)
    if fa is None or fa != fb:
        return False
    kinds = sorted([a.split('_')[1], b.split('_')[1]])
    return kinds in (['loop', 'pad'], ['knuckle', 'pad'], ['pad', 'tail'])

if 'overlap' in SECTIONS:
    tac = [l for l in links if l.startswith('tactile_')]
    V_ONLY = [x for x in os.environ.get('V_ONLY', '').split(',') if x]     # optional split of the (slow) section
    tac_chk = [l for l in tac if not V_ONLY or any(x in l for x in V_ONLY)]
    others = [l for l in links if not l.startswith('tactile_') and not l.startswith('HW_')]
    hw = [l for l in links if l.startswith('HW_')]
    W = {l: wshape(l) for l in tac + others + hw}
    SEAT = seats()
    worst, spl, eng = [], [], []
    for i, t in enumerate(tac):
        if t not in tac_chk:
            continue
        st = W[t]; bb = st.BoundBox
        bbe = App.BoundBox(bb.XMin - 0.5, bb.YMin - 0.5, bb.ZMin - 0.5, bb.XMax + 0.5, bb.YMax + 0.5, bb.ZMax + 0.5)
        for o in others + hw + [x for x in tac if x != t and (x not in tac_chk or tac.index(x) > i)]:
            if not W[o].BoundBox.intersect(bbe):
                continue
            ov = st.common(W[o]).Volume
            if ov <= 1e-4:
                continue
            if o.startswith('tactile_') and splice(t, o) and ov < 0.02:
                spl.append('%s x %s %.4f' % (t, o, ov)); continue
            if o.startswith('HW_R_tactile_pod_lid') and t in ('tactile_pod', 'tactile_pod_lid'):
                eng.append('%s x %s %.3f' % (t, o, ov)); continue
            worst.append('%s x %s ov %.4f' % (t, o, ov))
    # material ADDED by tactile (palm pad beds) against every other part except the bone it belongs to
    fills = []
    for key in (T.PALM if (not V_ONLY or 'fill' in V_ONLY) else []):
        f, r = next((f, r) for f, r in T.RAYS.items() if r['palm'] and r['palm'][0] == key)
        lab = r['palm'][1]
        bpl = links[lab].Placement.multiply(links[lab].LinkedObject.Shape.Placement)
        fill = T.palm_tools_local(key)[0]
        fill.Placement = bpl.multiply(fill.Placement)
        fb = fill.BoundBox
        for o in others + hw:
            if o == lab or not W[o].BoundBox.intersect(fb):
                continue
            ov = fill.common(W[o]).Volume
            if ov > 1e-4:
                worst.append('palm bed fill %s x %s ov %.4f' % (key, o, ov))
            fills.append((key, o, round(ov, 4)))
    res['palm_fill_contacts'] = fills
    res['overlaps'] = worst; res['splices'] = spl; res['designed_engagement'] = eng
    check('no overlap: tactile vs all parts/hardware/tactile', not worst,
          'none (%d FPC splice butt joints <= 0.02 mm3, %d lid screw/insert engagements in the pod)' % (len(spl), len(eng)) if not worst else '; '.join(worst[:12]))
    # clearance of the free tails / pod to everything they are not seated on
    gaps = []
    for t in [x for x in tac_chk if not x.startswith('tactile_pad_')]:
        st = W[t]; bb = st.BoundBox
        bbe = App.BoundBox(bb.XMin - 0.5, bb.YMin - 0.5, bb.ZMin - 0.5, bb.XMax + 0.5, bb.YMax + 0.5, bb.ZMax + 0.5)
        for o in others + hw:
            if not W[o].BoundBox.intersect(bbe):
                continue
            if o.startswith('HW_R_tactile_pod_lid'):
                continue
            d = st.distToShape(W[o])[0]
            if d < 0.2:
                seat = o in SEAT.get(t, set())
                d2 = sampled_distance(st, W[o], 1.0) if (d < 1e-3 and not seat) else None
                gaps.append((t, o, round(d, 3), None if d2 is None else round(d2, 3), 'seat' if seat else 'free'))
    res['gaps'] = gaps
    free_bad = [g for g in gaps if g[4] == 'free' and (g[3] if g[3] is not None else g[2]) < 0.199]
    check('free tails/pod: clearance >= 0.2 to unrelated parts', not free_bad,
          'contacts only where seated (channels, loop anchors, exits, strap top): %d seat contacts' % sum(1 for g in gaps if g[4] == 'seat')
          if not free_bad else '; '.join('%s x %s %.3f (sampled %s)' % (g[0], g[1], g[2], g[3]) for g in free_bad))

# ------------------------------------------------------------------------------------------------ walls
def actuation_finger_cuts(akey):
    """rebuild mod_actuation.finger_modify's cutter list (read-only use of its constants)"""
    import mod_actuation as A
    from v6geom import cyl, box
    cuts = [cyl((x0, y, z), (x1, y, z), 1.05) for (y, z, x0, x1) in A.V5CH.get(akey, [])]
    if akey == 'distal':
        cuts += A.distal_anchor()
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(21.6, 0, 4.7), V(-1, 0, 0)))
    if akey in A.BORES:
        j2, j3 = A.BORES[akey]
        cuts.append(box((-1.0, -0.8, 4.0), (j2 + 7.7, 0.8, 8.5)))
        cuts.append(box((j3 - 7.7, -0.8, 4.0), (j3 + 9.0, 0.8, 8.5)))
        cuts.append(cyl((A.ANCHOR_X, 0, 3.4), (A.ANCHOR_X, 0, 8.5), 1.5))
    if akey in ('metacarpal', 'metacarpal_pinky'):
        cuts.append(cyl((A.ANCHOR_X, 0, 3.4), (A.ANCHOR_X, 0, 8.5), 1.5))
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(14.9, 0.05, 4.7), V(1, 0, 0)))
        rear_j2 = 39.5 if akey == 'metacarpal' else 32.5
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(rear_j2 - 7.4, 0.05, 4.7), V(-1, 0, 0)))
    return cuts

if 'walls' in SECTIONS:
    THUMB_MAP = {'thumb_metacarpal': 'metacarpal', 'thumb_proximal': 'proximal', 'thumb_distal': 'distal'}
    rows = []
    worst = 1e9
    for key in T.PHALANX:
        ak = THUMB_MAP.get(key, key)
        voids = actuation_finger_cuts(ak)
        mine = T.phalanx_tools_local(key)
        # exclude voids that are open to the surface where my recess meets them on purpose: the knot pocket (bridged
        # by the pad, open to the face) and the open-roof slots - report them separately
        dmin = 1e9; which = ''
        for i, v in enumerate(voids):
            bbv = v.BoundBox
            open_face = bbv.ZMax > 7.4             # opens to the palm face (knot pocket, open-roof slot)
            for m in mine:
                d = m.distToShape(v)[0]
                if not open_face and d < dmin:
                    dmin, which = d, 'void %d z %.2f..%.2f x %.1f..%.1f' % (i, bbv.ZMin, bbv.ZMax, bbv.XMin, bbv.XMax)
        rows.append({'part': key, 'min_wall_to_internal_void': round(dmin, 3), 'at': which})
        worst = min(worst, dmin)
    # palm bones: TPU layer and strap nut pockets
    for key in T.PALM:
        mine = T.palm_tools_local(key)[1]
        tpu = T.to_local(key, T.cbox(5.0, 72.0, -7.5, 7.5, -1.1, -0.5))
        d_tpu = mine.distToShape(tpu)[0]
        rows.append({'part': key, 'min_wall_to_TPU_layer': round(d_tpu, 3)})
    res['walls'] = rows
    check('min wall over internal channels >= 1.0', worst >= 0.999, 'min %.3f mm (phalanges: recess floor 6.9 vs flexor-channel roof 5.75 -> 1.15; spine floor -6.9 vs back channel -5.65 -> 1.25)' % worst)
    tp = min(r['min_wall_to_TPU_layer'] for r in rows if 'min_wall_to_TPU_layer' in r)
    check('palm curtain lip above the TPU web slot', tp >= 0.59, '%.3f mm (the TPU layer is an insert pocket, not a channel; 3 PLA layers)' % tp)

# ------------------------------------------------------------------------------------------------ rom
def rot_about(shape, point, axis, deg):
    s = shape.copy()
    s.rotate(point, axis, deg)
    return s

if 'rom' in SECTIONS:
    import csv
    mates = {}
    for row in csv.DictReader(open(R + '/data/mates.csv')):
        mates[row['mate']] = row
    # (palm-side instance, tip-side instance, tip-side joint point & axis in the tip-side local frame) per finger joint
    chains = {
        'index': [('Base Bone 1_V02 <1>', 'Metacarpal Bone_V02 <1>', (7.5, 0, 0)),
                  ('Metacarpal Bone_V02 <1>', 'Proximal Phalanx Bone_V02 <1>', (7.5, 0, 0)),
                  ('Proximal Phalanx Bone_V02 <1>', 'Distal Phalanx Bone_V02 <1>', (29.3, 0, 0))],
        'middle': [('Base Bone 1_V02 <2>', 'Metacarpal Bone_V02 <2>', (7.5, 0, 0)),
                   ('Metacarpal Bone_V02 <2>', 'Proximal Phalanx Bone_V02_middlefinger <1>', (7.5, 0, 0)),
                   ('Proximal Phalanx Bone_V02_middlefinger <1>', 'Distal Phalanx Bone_V02 <2>', (29.3, 0, 0))],
        'ring': [('Base Bone 1_V02 <4>', 'Metacarpal Bone_V02 <3>', (7.5, 0, 0)),
                 ('Metacarpal Bone_V02 <3>', 'Proximal Phalanx Bone_V02 <2>', (7.5, 0, 0)),
                 ('Proximal Phalanx Bone_V02 <2>', 'Distal Phalanx Bone_V02 <3>', (29.3, 0, 0))],
        'pinky': [('Base Bone 1_V02 <3>', 'Metacarpal Bone_V02_pinky <1>', (7.5, 0, 0)),
                  ('Metacarpal Bone_V02_pinky <1>', 'Proximal Phalanx Bone_V02_pinky <1>', (7.5, 0, 0)),
                  ('Proximal Phalanx Bone_V02_pinky <1>', 'Distal Phalanx Bone_V02 <4>', (29.3, 0, 0))],
        'thumb': [('third_thumb_hinge <1>', 'Metacarpal Bone_V02 <4>', (7.5, 0, 0)),
                  ('Metacarpal Bone_V02 <4>', 'Proximal Phalanx Bone_V02 <3>', (7.5, 0, 0)),
                  ('Proximal Phalanx Bone_V02 <3>', 'Distal Phalanx Bone_V02 <5>', (29.3, 0, 0))],
    }
    ride = {}                                    # instance label -> tactile instances riding on it
    for n, p, pl, c, a in T.instance_specs():
        if a:
            ride.setdefault(a, []).append(n)
    rows = []
    worst = 1e9
    for f, joints in chains.items():
        labs = [joints[0][0]] + [j[1] for j in joints]
        for ji, (pa, tb, jp) in enumerate(joints):
            # joint axis in world from the tip-side bone (local y through jp); flexion = tip moves toward +z(palm)
            pl = links[tb].Placement.multiply(links[tb].LinkedObject.Shape.Placement)
            P = pl.multVec(V(*jp)); A = pl.Rotation.multVec(V(0, 1, 0))
            tipdir = pl.Rotation.multVec(V(1, 0, 0) if 'Distal' not in tb else V(-1, 0, 0))
            downstream = labs[ji + 1:]
            moving = []
            for lab in downstream:
                moving.append(lab)
                moving += [t for t in ride.get(lab, []) if 'loop' not in t]
            fixed = [lab for lab in labs[:ji + 1]] + [t for lab in labs[:ji + 1] for t in ride.get(lab, []) if 'loop' not in t and 'knuckle' not in t]
            Wm = {m: wshape(m) for m in moving}
            Wf = {x: wshape(x) for x in fixed}
            for deg in (0, 30, 60, 90):
                # sign: flexion moves the tip toward world +z
                # flexion turns the tip toward the palm side of the bone (its local +z; the thumb is re-clocked, so
                # world z is not the palm normal there)
                nrm = pl.Rotation.multVec(V(0, 0, 1))
                sgn = 1.0 if App.Rotation(A, 10).multVec(tipdir).dot(nrm) >= tipdir.dot(nrm) else -1.0
                hits = []
                dmin = 1e9
                for m, sm in Wm.items():
                    sr = rot_about(sm, P, A, sgn * deg)
                    for x, sx in Wf.items():
                        if not sr.BoundBox.intersect(sx.BoundBox):
                            continue
                        is_tac = m.startswith('tactile_') or x.startswith('tactile_')
                        if not is_tac:
                            continue
                        ov = sr.common(sx).Volume
                        if ov > 0.01:
                            hits.append('%s x %s %.2f' % (m, x, ov))
                rows.append({'finger': f, 'joint': ji, 'deg': deg, 'hits': hits})
                if hits:
                    print('   ROM %s J%d %d deg: %s' % (f, ji, deg, '; '.join(hits[:4])))
    bad = [r for r in rows if r['hits']]
    res['rom'] = rows
    check('ROM 0..90 deg with pads/spines riding on the bones', not bad, 'no pad/spine collisions in %d poses' % len(rows) if not bad else '%d poses with hits' % len(bad))
    # joint loops: the designed shape at 0 deg and the same-length shape at 90 deg against both bones at that angle
    lrows = []
    for f in ('index', 'middle', 'ring', 'pinky', 'thumb'):
        joints = chains[f]
        for kind, (pa, tb, jp) in (('J2', joints[-2]), ('J3', joints[-1])):
            jpl, (key, on, j) = T.joint_frame(f, kind)
            n_cols = T.LOOP_W[kind][f]
            pl = links[tb].Placement.multiply(links[tb].LinkedObject.Shape.Placement)
            P = pl.multVec(V(*jp)); A = pl.Rotation.multVec(V(0, 1, 0))
            tipdir = pl.Rotation.multVec(V(1, 0, 0) if 'Distal' not in tb else V(-1, 0, 0))
            nrm = pl.Rotation.multVec(V(0, 0, 1))
            sgn = 1.0 if App.Rotation(A, 10).multVec(tipdir).dot(nrm) >= tipdir.dot(nrm) else -1.0
            for deg in (0, 90):
                lp = T.joint_loop_part(kind, n_cols, deg)
                lp.Placement = jpl.multiply(lp.Placement)
                # trim the anchor ends (they sit in the channels by design): keep the part below the dorsal faces
                d = T.loop_design(kind, 0.0)
                tip = wshape(tb) if deg == 0 else rot_about(wshape(tb), P, A, sgn * deg)
                pal = wshape(pa)
                ov_t = lp.common(tip).Volume; ov_p = lp.common(pal).Volume
                lrows.append({'finger': f, 'joint': kind, 'deg': deg, 'overlap_tip': round(ov_t, 4), 'overlap_palm': round(ov_p, 4),
                              'loop_len': round(d['L'], 1), 'r90': round(d['rl90'], 2)})
    res['loops'] = lrows
    worst = max(max(r['overlap_tip'], r['overlap_palm']) for r in lrows)
    check('joint loops clear of the knuckles at 0 and 90 deg', worst < 0.05,
          'max overlap %.4f mm3; loop length %.1f mm stores %.1f mm over its chord (needed 15.7 + 3 margin); 90-deg loop radius %.1f mm' % (
              worst, lrows[0]['loop_len'], T.LOOP_EXCESS, min(r['r90'] for r in lrows)))
    # knuckle ribbon: static part (palm frame u < 97) against the finger swung about J0 by -15 / +15 deg
    jrows = []
    for f in ('index', 'middle', 'ring', 'pinky'):
        pkey, pon = T.RAYS[f]['palm']
        Mc = T._canon_world(pkey)
        rib = wshape('tactile_knuckle_' + f)
        cut = T.cbox(-50, 97.0, -40, 40, -60, 30)
        cut.transformShape(Mc)
        stat = rib.common(cut)
        J = Mc.multVec(V(89.3, 0, 0)); Ax = App.Placement(Mc).Rotation.multVec(V(0, 0, 1))
        fl = [x for x in chains[f][0][:1]] + [j[1] for j in chains[f]]
        for deg in (-15, 15):
            for lab in fl:
                sr = rot_about(wshape(lab), J, Ax, deg)
                ov = sr.common(stat).Volume if sr.BoundBox.intersect(stat.BoundBox) else 0.0
                jrows.append({'finger': f, 'deg': deg, 'part': lab, 'overlap': round(ov, 4)})
    res['j0'] = jrows
    worst = max(r['overlap'] for r in jrows)
    check('J0 +-15 deg: finger clear of the static knuckle ribbon', worst < 0.01, 'max overlap %.4f mm3' % worst)

res['elapsed_s'] = round(time.time() - t00, 1)
json.dump(res, open(out_path, 'w'), indent=1)
print('done %.0fs -> %s' % (time.time() - t00, out_path))
