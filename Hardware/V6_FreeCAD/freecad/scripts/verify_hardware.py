"""Verify the V6 hardware (topic 'hardware') on a built model, headless.

  V6_IN=<built.FCStd> freecadcmd scripts/verify_hardware.py
env: V6_IN        built model (default work/hardware/test.FCStd)
     V6_HW_REPORT report path (default regions/hardware_report.json; written with stage "verify")
     V6_HW_REGIONS claimed-regions path (default regions/hardware.json; "" = do not write)
     V6_HW_SWEEP  J0 angles in degrees (default "-15,-10,-5,0,5,10,15")
Checks: library validity and seat frames; every joint re-validated against the built parts (hole + 0.2/side,
seats, pocket clearance, engagement); exact overlap volume of every hardware instance against every other
instance; measured J0 clearances and the stack-up along the J0 axis; J0 range-of-motion sweep (finger chain
rotated about the J0 axis of mod_j0.MC_PALM, cross-checked against data/mates.csv).
"""
import sys, os, math, json, csv, time
LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
sys.path.insert(0, LIB)
import FreeCAD as App, Part
import importlib
import mod_hardware as H
import mod_j0
from v6geom import cyl, hex_prism
importlib.reload(H)
V = App.Vector
FC = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
inp = os.environ.get('V6_IN', FC + '/work/hardware/test.FCStd')
regions_out = os.environ.get('V6_HW_REGIONS', FC + '/regions/hardware.json')
angles = [float(a) for a in os.environ.get('V6_HW_SWEEP', '-15,-10,-5,0,5,10,15').split(',')]
T0 = time.time()
doc = App.openDocument(inp)
out = {'model': inp}
fails = []

def say(*a):
    print(*a)

# ---------------------------------------------------------------- 1. library
say('== 1. fastener library (V6_hardware_*)')
lib = {}
for o in doc.Objects:
    if o.Name.startswith('V6_hardware_'):
        s = o.Shape
        ob = s.optimalBoundingBox(True, False)
        ok = s.isValid() and len(s.Solids) == 1 and s.Volume > 0 and o.Placement.isIdentity()
        lib[o.Name[3:]] = dict(valid=s.isValid(), solids=len(s.Solids), volume=round(s.Volume, 3),
                               placement_identity=o.Placement.isIdentity(),
                               z=[round(ob.ZMin, 3), round(ob.ZMax, 3)], xy=[round(ob.XMax, 3), round(ob.YMax, 3)])
        say('  %-24s valid %-5s solids %d vol %8.3f  z %7.3f..%7.3f  x/y +-%.3f/%.3f %s' % (
            o.Name[3:], s.isValid(), len(s.Solids), s.Volume, ob.ZMin, ob.ZMax, ob.XMax, ob.YMax, '' if ok else '  <-- BAD'))
        if not ok:
            fails.append('library part %s invalid' % o.Name)
out['library'] = lib

# ---------------------------------------------------------------- 2. joints re-validated
say('\n== 2. joints (resolve + validate against the built parts)')
joints, open_items, files, world = H.plan(doc, True)
for j in joints:
    e = j['checks'].get('engagement', {})
    say('  %-11s %-11s + %-9s %-4s  grip %s  nut face %s  tip past nut %s (%s threads)%s' % (
        j['id'], H.screw_type(j['size'], j['L']), j['nut'], j['status'].upper(), j['checks'].get('grip_parts'),
        e.get('nut_face'), e.get('protrusion'), e.get('threads_past'),
        ''.join('\n        %s: %s' % p for p in j['problems'])))
    if j['status'] == 'fail':
        fails.append('%s: %s' % (j['id'], '; '.join(m for s, m in j['problems'] if s == 'FAIL')))
say('  open items (not placed): %d' % len(open_items))
for f in files:
    say('  request file %s: %s, %d entries, %d placed, rejected %s' % (f['file'], f['status'], f['entries'], f['placed'], f['rejected']))

# ---------------------------------------------------------------- 3. exact collision audit of the instances
say('\n== 3. collision audit: every hardware instance vs every other instance (exact common volume)')
hw = [o for o in doc.Objects if o.TypeId == 'App::Link' and o.Name.startswith('I_HW_')]
def placed(o):
    s = o.LinkedObject.Shape.copy()     # obj.Shape is immutable in FreeCAD 1.1 (fasteners are tiny)
    s.Placement = o.Placement
    return s
hws = [(o.Label, placed(o)) for o in hw]
hw_bb = [h.BoundBox for _, h in hws]
others = [(it['label'], world.shape(it)) for it in world.items if any(it['bb'].intersect(b) for b in hw_bb)]
audit = []
worst = 0.0
for la, a in hws:
    for lb, b in others + [x for x in hws if x[0] != la]:
        if not a.BoundBox.intersect(b.BoundBox):
            continue
        d = a.distToShape(b)[0]
        if d > 0.2:
            continue
        v = a.common(b).Volume if d < 1e-6 else 0.0
        if d < 1e-6 and v < 1e-6:          # boundaries apart but maybe fully inside
            v = 0.0
        same_joint = la.rsplit('_', 1)[0] == lb.rsplit('_', 1)[0]
        audit.append(dict(a=la, b=lb, min_gap=round(d, 4), overlap_mm3=round(v, 5)))
        worst = max(worst, v)
        if v > 1e-4:
            fails.append('overlap %s x %s = %.4f mm3' % (la, lb, v))
audit.sort(key=lambda r: (-r['overlap_mm3'], r['min_gap']))
for r in audit:
    say('  %-26s %-58s gap %.4f  overlap %.5f' % (r['a'], r['b'], r['min_gap'], r['overlap_mm3']))
say('  pairs within 0.2 mm: %d, max overlap %.5f mm3 (instances %d)' % (len(audit), worst, len(hw)))
out['audit'] = dict(instances=len(hw), pairs_within_0p2=len(audit), max_overlap_mm3=round(worst, 6), pairs=audit)

# ---------------------------------------------------------------- 4. J0: measured clearances + stack-up
say('\n== 4. J0 clearances (measured) and stack-up along the axis (z from the head seat, toward the nut)')
by_label = {it['label']: it for it in world.items}
j0 = [j for j in joints if j['source'] == 'j0']
j0_out = {}
for j in j0:
    pl = H._placement(j['P'], j['A'], j['X'])
    palm = world.shape(by_label[j['palm']])
    base = world.shape(by_label[j['base']])
    zn = j['zn']
    shank = H._local(cyl((0, 0, 0.01), (0, 0, zn - 0.01), 1.0), pl)
    head = H._local(cyl((0, 0, -2.0), (0, 0, -0.25), 1.9), pl)
    nut = H._local(hex_prism((0, 0, zn + 0.25), (0, 0, 1), (1, 0, 0), 4.0, 0.7), pl)
    m = dict(shank_to_palm_hole=round(shank.distToShape(palm)[0], 4), shank_to_base_bone_hole=round(shank.distToShape(base)[0], 4),
             head_to_counterbore_wall=round(head.distToShape(palm)[0], 4), nut_flats_to_pocket=round(nut.distToShape(palm)[0], 4),
             base_bone_to_palm_bone=round(base.distToShape(palm)[0], 4))
    # stack-up: material intervals per part on 4 lines at r = 1.6 around the axis
    stack = {}
    for lab, s in (('palm bone', palm), ('Base Bone 1', base)):
        ivs, cur = [], None
        z = -2.5
        while z <= 17.0:
            ins = any(s.isInside(pl.multVec(V(1.6 * math.cos(t), 1.6 * math.sin(t), z)), 1e-6, True) for t in (0.3, 1.9, 3.5, 5.1))
            if ins and cur is None:
                cur = z
            if not ins and cur is not None:
                ivs.append([round(cur, 2), round(z - 0.05, 2)])
                cur = None
            z = round(z + 0.05, 4)
        if cur is not None:
            ivs.append([round(cur, 2), 17.0])
        stack[lab] = ivs
    e = j['checks']['engagement']
    stack['screw head'] = [-2.0, 0.0]
    stack['nyloc (h 2.8)'] = [e['nut_face'], e['nut_top']]
    stack['screw tip'] = j['L']
    m['stack'] = stack
    lens = {}
    for L in (12, 14, 16, 18, 20):
        p = L - e['nut_top']
        lens['M2x%d' % L] = dict(protrusion=round(p, 2), threads_past=round(p / 0.4, 1),
                                 ok=p >= 0.8 and L - (e['nut_face'] + 3.0) >= 0.6)
    m['length_choice'] = lens
    j0_out[j['id']] = m
    say('  %-10s shank-palm hole %.3f  shank-Base Bone hole %.3f  head-counterbore %.3f  nut flats-pocket %.3f  Base Bone-palm %.3f' % (
        j['id'], m['shank_to_palm_hole'], m['shank_to_base_bone_hole'], m['head_to_counterbore_wall'], m['nut_flats_to_pocket'], m['base_bone_to_palm_bone']))
    say('             stack: %s' % json.dumps(stack))
    say('             length: %s' % ', '.join('%s %+.2f (%s thr)%s' % (k, v['protrusion'], v['threads_past'], '' if v['ok'] else ' no') for k, v in lens.items()))
    for key in ('shank_to_palm_hole', 'shank_to_base_bone_hole', 'head_to_counterbore_wall', 'nut_flats_to_pocket'):
        if m[key] < 0.2 - 0.005:
            fails.append('%s %s = %.3f < 0.2' % (j['id'], key, m[key]))
out['j0_clearances'] = j0_out

# ---------------------------------------------------------------- 5. J0 range-of-motion sweep
say('\n== 5. J0 sweep: finger chain rotated about the J0 axis (mod_j0.MC_PALM), hardware re-checked')
mates = list(csv.DictReader(open(ROOT + '/data/mates.csv')))
def mate_axis(palm_label):
    """Palm-side J0 connector (origin, z) in palm-local coordinates from mates.csv."""
    base = palm_label.replace(' <1>', '')
    for r in mates:
        if not r['limits(deg or mm)'].startswith("{'limitAxialZMax': 15.0"):
            continue
        for occ, o, z in (('occ1', 'cs1_origin_mm_local', 'cs1_z_local'), ('occ2', 'cs2_origin_mm_local', 'cs2_z_local')):
            if base in r[occ]:
                return r['mate'], V(*json.loads(r[o])), V(*json.loads(r[z]))
    return None, None, None
FINGER_GROUP = {'index': 'Index', 'middle': 'Middle', 'ring': 'Ring', 'pinky': 'Pinky'}
sweep = {}
for j in j0:
    palm_link = by_label[j['palm']]['link']
    o, x, z = mod_j0.MC_PALM
    ax_pt = palm_link.Placement.multVec(V(*o))
    ax_dir = palm_link.Placement.Rotation.multVec(V(*z))
    name, mo, mz = mate_axis(j['palm'])
    w_mo = palm_link.Placement.multVec(mo)
    w_mz = palm_link.Placement.Rotation.multVec(mz)
    off = ((w_mo - ax_pt) - ax_dir * (w_mo - ax_pt).dot(ax_dir)).Length
    ang = math.degrees(w_mz.getAngle(ax_dir))
    chain = [(o2, o2.LinkedObject.Shape.copy()) for o2 in doc.getObject(FINGER_GROUP[j['finger']]).Group if o2.TypeId == 'App::Link']
    hw_j = [(la, s) for la, s in hws]
    palm = world.shape(by_label[j['palm']])
    rows = []
    for a in angles:
        rot = App.Placement(V(0, 0, 0), App.Rotation(ax_dir, a), ax_pt)
        worst_v, min_gap, base_palm_v, shank_gap = 0.0, 1e9, 0.0, None
        pl = H._placement(j['P'], j['A'], j['X'])
        shank = cyl((0, 0, 0.01), (0, 0, j['zn'] - 0.01), 1.0)
        shank.Placement = pl
        for lk, s in chain:
            s.Placement = rot.multiply(lk.Placement)
            for la, h in hw_j:
                if not h.BoundBox.intersect(s.BoundBox):
                    continue
                d = h.distToShape(s)[0]
                v = h.common(s).Volume if d < 1e-6 else 0.0
                if la.startswith('HW_%s_' % j['id']):
                    min_gap = min(min_gap, d)
                worst_v = max(worst_v, v)
                if v > 1e-4:
                    fails.append('J0 sweep %s %+.0f deg: %s x %s overlap %.4f' % (j['id'], a, la, lk.Label, v))
            if lk.Label == j['base']:
                shank_gap = shank.distToShape(s)[0]
                if s.BoundBox.intersect(palm.BoundBox):
                    base_palm_v = s.common(palm).Volume
        rows.append(dict(angle=a, hardware_overlap_mm3=round(worst_v, 5), own_hw_min_gap_to_chain=round(min_gap, 4),
                         shank_to_base_bone=round(shank_gap, 4) if shank_gap is not None else None,
                         base_bone_x_palm_overlap_mm3=round(base_palm_v, 4)))
        if shank_gap is not None and shank_gap < 0.2 - 0.005:
            fails.append('J0 sweep %s %+.0f deg: shank to Base Bone %.3f < 0.2' % (j['id'], a, shank_gap))
    sweep[j['id']] = dict(axis_point=[round(c, 4) for c in ax_pt], axis_dir=[round(c, 5) for c in ax_dir],
                          mate=name, mate_axis_offset=round(off, 4), mate_axis_angle_deg=round(ang, 4), rows=rows)
    say('  %-10s axis %s dir %s  vs %s: offset %.4f mm, angle %.4f deg' % (j['id'], sweep[j['id']]['axis_point'], sweep[j['id']]['axis_dir'], name, off, ang))
    for r in rows:
        say('      %+5.0f deg: hw overlap %.5f  own hw gap to finger %.3f  shank-Base Bone %.3f  Base Bone x palm %.3f mm3' % (
            r['angle'], r['hardware_overlap_mm3'], r['own_hw_min_gap_to_chain'], r['shank_to_base_bone'] or -1, r['base_bone_x_palm_overlap_mm3']))
out['j0_sweep'] = sweep

# ---------------------------------------------------------------- 6. claimed regions vs the other agents', report
say('\n== 6. claimed regions vs other agents\' regions/*.json')
regs = H.claimed_regions(joints, doc)
conf = H.region_conflicts(regs, doc)
for c in conf:
    say('  %s %s: OURS %s  <->  THEIRS %s' % (c['file'], 'exact' if c.get('exact') else 'approx(world AABB)', c.get('ours'), c.get('theirs') or c.get('error')))
say('  %d conflicts' % len(conf))
out['claimed_regions'] = regs
out['region_conflicts'] = conf
out['verdict'] = 'PASS' if not fails else 'FAIL'
out['verify_failures'] = fails
rep = H.write_report(joints, open_items, files, doc, stage='verify', extra=dict(verify=out), elapsed=time.time() - T0)
say('\n== report written:', H._report_path())
if regions_out:
    with open(regions_out, 'w') as fh:
        json.dump(regs, fh, indent=1)
    say('== regions written: %s (%d boxes)' % (regions_out, len(regs)))
say('\nBOM (placed):', rep['bom'])
say('SUMMARY:', rep['summary'])
say('VERDICT: %s%s' % (out['verdict'], ''.join('\n  - ' + f for f in fails)))
say('elapsed %.0fs' % (time.time() - T0))
