"""V6 actuation (topic 'actuation', requirements 1, 2, 4): straight N20 gearmotors with encoders for the 16 finger
joints + thumb T1/T2, their seats, retention, spools and tendon channels.  mm, clearance C = 0.2 per side.

FINGERS: a 2x2 motor magazine under every palm bone (reasoning and joint map in notes/actuation.md).
  * all four motors lie along the palm bone (local x), flats vertical, PCB tab + 6-pin connector toward local +z
    (thumb side), encoder end toward the fingertips: every plug points into the open inter-finger gap beside the
    magazine and its cable drops straight down to the harness (V5's trapped index/pinky wiring is gone).
  * tier 1 (directly under the bone): A = J1 flexor (front), B = J2 flexor (rear), spool r 3.75 (tendon axis).
  * tier 2 (under tier 1, shifted 3.5-7 mm back): C = J3 flexor (front), D = J0 abduction loop (rear, 2 grooves),
    spool r 5.65 so their tendons clear tier 1 in the magazine side lanes.
  * magazine = PLA fused to the palm-bone underside (printed on the back half after the TPU pause): open-bottom bays,
    gearbox fork ribs (axial + torque), bend bosses, an under-J0 tendon guide block.  Old worm seats are filled
    (outside the TPU insert layer, the TPU peg holes and the hardware keep-outs).
  * tray (new part per finger) holds tier 2 in its own gearbox forks and carries tier 1 on the fork tops;
    2x M2x16 SHCS + M2 hex nut through side ears.
  * tendons (0.5 mm braided UHMWPE, channels dia 1.2, bends R >= 3): A/B rise into two lanes in the bone's back half
    (y 5.8, under the pillars), run to the J0 block, drop through the back lug beside the J0 nut into the guide
    block; C runs in the -z side lane to the guide block; A, B, C pass the J0 axis *under* the joint (guide 3.7 mm in
    front of the axis -> <0.1 mm abduction coupling) and enter the base bone from below (V5 connector position),
    rise behind the J1 bore and wrap the J1 head.  D's two ends run in tunnels (y 6.2, z +-5.65) and go up into the
    V5 lateral J0 wing channels (moment arm ~10.3) to knot pockets in the base bone's sides.
THUMB: T1 (flexor R4) and T2 (flexor R5+R6) stand in their V5 cover3 seat towers (centred, turned to the seat angle),
  shaft up, spool at the top, gearbox face screwed to a floor with the N20's own 2x M1.6x3 (T1 in a collar on its
  tower); tendon hand-off to the thumb gimbal (mod_thumb).
Palm-bone local frame (README): x along the bone (J0 at 89.3), y = back side (world -Z), z = world +Y side.
"""
import os, sys, json, math
import FreeCAD as App, Part

LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
if LIB not in sys.path:
    sys.path.insert(0, LIB)
from v6geom import cyl, box, hex_prism, bone_section, clean
import n20

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
REGIONS = ROOT + '/freecad/regions'
C = 0.2
TR = 0.6                        # tendon channel radius (dia 1.2 for 0.5 mm line)

# ------------------------------------------------------------------------------------------------ frames
def _occ_matrices():
    d = json.load(open(os.path.join(ROOT, 'data/top_assembly_definition.json')))
    ra = d['rootAssembly']
    inst = {i['id']: i for i in ra['instances']}
    for s in d['subAssemblies']:
        for i in s['instances']:
            inst[i['id']] = i
    out = {}
    for o in ra['occurrences']:
        leaf = inst[o['path'][-1]]
        t = o['transform']
        out[leaf['name']] = App.Matrix(t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000,
                                       t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1)
    return out

OCC = _occ_matrices()
FINGERS = ['index', 'middle', 'ring', 'pinky']
PALM_KEY = {f: 'palm_' + f for f in FINGERS}
PALM_INST = {'index': 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>',
             'middle': 'Base Bone 1.2_V02_middlefinger <1>',
             'ring': 'Base Bone 1.2_V02_ringfing <1>',
             'pinky': 'Base Bone 1.2_V02_pinky <1>'}
BASE_INST = {'index': 'Base Bone 1_V02 <1>', 'middle': 'Base Bone 1_V02 <2>', 'ring': 'Base Bone 1_V02 <4>',
             'pinky': 'Base Bone 1_V02 <3>'}
LETTER = {'index': 'I', 'middle': 'M', 'ring': 'R', 'pinky': 'P'}

def pm(f):
    return OCC[PALM_INST[f]]

# ------------------------------------------------------------------------------------------------ layout (palm local)
T1Y, T2Y = 13.7, 27.1           # tier axes: bone underside y 7.5 + 0.2 + 6.0 ; tier 1 bottom 19.7 + fork 1.0 + 0.4 + 6
HW, HH = 5.0, 6.0               # motor half width (flats, lateral) / half height (round, vertical)
RS = {'small': 3.75, 'big': 5.65, 'double': 5.65}    # tendon-axis radius on the spool
RF = {'small': 4.9, 'big': 6.6, 'double': 6.6}       # flange radius
SPAN = {'small': (-1.0, -7.0), 'big': (-1.0, -7.0), 'double': (-1.0, -7.6)}   # spool span, motor x
GROOVE = {'small': [-5.0], 'big': [-5.0], 'double': [-3.6, -6.0]}           # groove centres, motor x
# name: (tier, gearbox-face x, spool, joint, tendon side (+1 = +z))
MOTORS = {'A': (1, 55.0, 'small', 'J1', -1), 'B': (1, 14.0, 'small', 'J2', +1),
          'C': (2, 53.5, 'big', 'J3', -1), 'D': (2, 9.0, 'double', 'J0', 0)}
LANE_Y = 5.8                    # bone lanes (A, B) : under the pillars (<= 4.3) and 1.1 mm above the bone underside
LANE_Z = 1.6
TUN_Y, TUN_Z = 5.9, 6.3         # D tunnels (>= 1 mm to the peg holes, tab windows, lane ceiling and outer face)
SIDE_Y = 14.0                   # C side lane height (clear of the A/B drops into the guide)
BAY_X0, BAY_X1 = 2.8, 86.6      # tier-1 bay extent (x)
WALL = 1.2
MWALL = 1.7                     # magazine side walls (carry the D tunnels)
BAY_Z = HW + C + 1.0            # bay half width incl. the 1.0 side lane (6.2)
TRAY_Z = RF['big'] + C          # tray inner half width (6.8)
CEIL = 7.7                      # bay ceiling (bone underside 7.5 + C)
J0X = 89.3
J0KEEP = (86.69, 91.91, 10.9, 2.61)   # hardware: J0 nut + screw tip keep-out (x0, x1, y_max, |z|)
GUIDE = (86.6, 93.4, 11.1, 20.1)      # guide block x0, x1, y0, y1 (down to the tray plane: prints on the bed)
BASE_ENTRY_X = 13.0             # base-bone local x of the flexor entry (V5 vertical connector sits at 12.2)

def motor_local(m):
    t, x0, kind, joint, side = MOTORS[m]
    return V(x0, T1Y if t == 1 else T2Y, 0.0)

TAB_FLIP = {('index', 'C')}     # tab/plug toward -z (index C: its +z plug keep-out would touch Shell1)

def motor_matrix_local(m, f=None):
    """n20 frame of motor m in the palm-bone frame (axes parallel; tab +z, or -z for TAB_FLIP)."""
    o = motor_local(m)
    if (f, m) in TAB_FLIP:
        return App.Matrix(1, 0, 0, o.x, 0, -1, 0, o.y, 0, 0, -1, o.z, 0, 0, 0, 1)
    return App.Matrix(1, 0, 0, o.x, 0, 1, 0, o.y, 0, 0, 1, o.z, 0, 0, 0, 1)

def motor_placement(f, m):
    return App.Placement(pm(f).multiply(motor_matrix_local(m, f)))

def ml(m, x, y=0.0, z=0.0):
    """motor-frame point of finger motor m -> palm local."""
    o = motor_local(m)
    return V(o.x + x, o.y + y, o.z + z)

# ------------------------------------------------------------------------------------------------ geometry helpers
def path_points(pts, R=3.0, step_deg=10.0):
    """Polyline with rounded corners (bend radius R) -> dense point list (arcs sampled every step_deg)."""
    pts = [V(p) for p in pts]
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        u = b - a; u.normalize()
        w = c - b; w.normalize()
        ang = math.acos(max(-1.0, min(1.0, u.dot(w))))
        if ang < 2e-3:
            continue
        t = R * math.tan(ang / 2)
        t = min(t, 0.49 * (b - a).Length, 0.49 * (c - b).Length)
        Rr = t / math.tan(ang / 2)
        p1 = b - u * t
        n = u.cross(w); n.normalize()
        inward = n.cross(u); inward.normalize()
        ctr = p1 + inward * Rr
        r0 = p1 - ctr
        k = max(2, int(math.degrees(ang) / step_deg) + 1)
        for j in range(k + 1):
            th = ang * j / k
            rot = App.Rotation(n, math.degrees(th))
            out.append(ctr + rot.multVec(r0))
    out.append(pts[-1])
    clean_pts = [out[0]]
    for p in out[1:]:
        if (p - clean_pts[-1]).Length > 1e-3:
            clean_pts.append(p)
    return clean_pts

def tube(pts, r=TR, R=3.0):
    """Solid tube of radius r along polyline pts with rounded corners: a chain of cylinders + joint spheres
    (robust in booleans; arc faceting error < 0.05 mm)."""
    ps = path_points(pts, R)
    cyls = [Part.makeCylinder(r, (q - p).Length, p, q - p) for p, q in zip(ps[:-1], ps[1:]) if (q - p).Length > 1e-3]
    sph = [Part.makeSphere(r, p) for p in ps[1:-1]]
    parts = cyls + sph
    if len(parts) == 1:
        return parts[0]
    expect = sum(math.pi * r * r * (q - p).Length for p, q in zip(ps[:-1], ps[1:]))
    out = parts[0].multiFuse(parts[1:]).removeSplitter()
    if out.isValid() and abs(out.Volume - expect) < 0.25 * expect:
        return out
    out = cyls[0]                                  # multiFuse glitch: fuse step by step
    for i, c in enumerate(cyls[1:]):
        out = out.fuse([sph[i], c]) if i < len(sph) else out.fuse(c)
    out = out.removeSplitter()
    if not (out.isValid() and abs(out.Volume - expect) < 0.25 * expect):
        raise RuntimeError('tube build failed (vol %.1f, expected %.1f)' % (out.Volume, expect))
    return out

def xform(shape, M):
    s = shape.copy()
    s.transformShape(M, True)
    return s

def local_of(f, shape_world):
    return xform(shape_world, pm(f).inverse())

# ------------------------------------------------------------------------------------------------ spools (motor frame)
def spool(kind):
    """Spool in the n20 motor frame (axis x, on the shaft x -1.0 .. -7.0/-7.6).  D-bore is nominal (dia 3.0, flat
    1.0 from the axis on +z): FDM holes print ~0.1 mm small, so it presses on; torque is carried by the flat (form
    lock), the interference only keeps it on axially (drop of CA optional).  Axial tendon-anchor hole dia 1.0 through
    the outer flange of each groove."""
    rf, rg = RF[kind], RS[kind] - 0.25
    if kind == 'double':
        prof = [(-1.0, 3.4), (-2.0, 3.4), (-2.0, rf), (-2.8, rf), (-2.8, rg), (-4.4, rg), (-4.4, rf), (-5.2, rf),
                (-5.2, rg), (-6.8, rg), (-6.8, rf), (-7.6, rf)]
    else:
        prof = [(-1.0, 3.4), (-3.0, 3.4), (-3.0, rf), (-3.8, rf), (-3.8, rg), (-6.2, rg), (-6.2, rf), (-7.0, rf)]
    pts = [V(prof[0][0], 0, 0)] + [V(x, 0, r) for x, r in prof] + [V(prof[-1][0], 0, 0)]
    s = Part.Face(Part.makePolygon(pts + [pts[0]])).revolve(V(0, 0, 0), V(1, 0, 0), 360)
    bore = Part.makeCylinder(1.5, 20, V(-15, 0, 0), V(1, 0, 0)).cut(box((-16, -2, 1.0), (6, 2, 2)))
    s = s.cut(bore)
    ra = rg + 0.8
    holes = [cyl((-6.1, 0, ra), (prof[-1][0] - 0.1, 0, ra), 0.5)]
    if kind == 'double':
        holes = [cyl((-1.9, 0, ra), (-2.9, 0, ra), 0.5), cyl((-6.7, 0, -ra), (-7.7, 0, -ra), 0.5)]
    return clean(s.cut(holes))

def spool_clear(kind, c=C):
    """Rotating-spool clearance envelope (motor frame)."""
    x0, x1 = SPAN[kind]
    return Part.makeCylinder(RF[kind] + c, (x0 - x1) + 2 * c, V(x1 - c, 0, 0), V(1, 0, 0))

def motor_env_local(m, c=C, f=None):
    """Motor body envelope + plug keep-out + spool clearance + protruding shaft, palm local."""
    kind = MOTORS[m][2]
    s = n20.body_envelope(c).fuse([n20.plug_keepout(8.0), spool_clear(kind, c),
                                    Part.makeCylinder(1.5 + c, 11.2, V(-11.0, 0, 0), V(1, 0, 0)),
                                    Part.makeCylinder(2.0 + c, 1.2, V(-1.0, 0, 0), V(1, 0, 0)),
                                    Part.makeCylinder(n20.MAGNET[0] / 2 + c, 1.4, V(25.3, 0, 0), V(1, 0, 0))])
    return xform(s, motor_matrix_local(m, f))


# ------------------------------------------------------------------------------------------------ tendon paths (palm local)
def tendon_paths():
    """name -> polyline (palm local) from the spool tangent point to where the tendon leaves the palm-bone assembly
    (under-J0 guide exit for the flexors, J0 wing exit for the abduction loop)."""
    gA = ml('A', GROOVE['small'][0]).x
    gB = ml('B', GROOVE['small'][0]).x
    gC = ml('C', GROOVE['big'][0]).x
    gDp, gDm = ml('D', GROOVE['double'][0]).x, ml('D', GROOVE['double'][1]).x
    P = {}
    for nm, g, s in (('A', gA, -1), ('B', gB, 1)):
        P[nm] = [V(g, T1Y, s * RS['small']), V(g, LANE_Y, s * LANE_Z), V(78.0, LANE_Y, s * LANE_Z),
                 V(81.2, 5.8, s * 3.0), V(82.2, 6.15, s * 3.25), V(82.9, 6.6, s * 3.45), V(86.0, 8.6, s * 4.28),
                 V(88.4, 10.2, s * 4.28), V(88.4, 11.8, s * 4.28), V(91.2, 12.8, s * 0.9), V(GUIDE[1] + 0.2, 12.5, s * 0.9)]
    P['C'] = [V(gC, T2Y, -RS['big']), V(gC, SIDE_Y, -RS['big']), V(86.9, SIDE_Y, -RS['big']),
              V(89.2, 15.6, -2.6), V(91.4, 15.8, 0.0), V(GUIDE[1] + 0.2, 15.5, 0.0)]
    wa, wb = V(74.94, 2.75, 6.88), V(94.77, -0.14, 10.55)       # V5 lateral J0 wing channel (dia 1.5)
    wk = wa + (wb - wa) * 0.2
    for nm, gx, sz in (('D+', gDp, 1), ('D-', gDm, -1)):
        # risers up the side lanes, forward under the TPU wrist-plate pocket (y <= 5.15 for x < 7.2) at y 6.8,
        # then up into the tunnel from x 12.5
        P[nm] = [V(gx, T2Y, sz * RS['double']), V(gx, 8.9, sz * RS['double']), V(gx + 3.0, 6.8, sz * TUN_Z),
                 V(9.5, 6.8, sz * TUN_Z), V(12.5, TUN_Y, sz * TUN_Z),
                 V(71.2, TUN_Y, sz * TUN_Z), V(74.5, 3.9, sz * 6.55), V(wk.x, wk.y, sz * wk.z), V(wb.x, wb.y, sz * wb.z),
                 V(97.2, -0.46, sz * 11.0)]
    return P

def tendon_cutters_palm():
    P = tendon_paths()
    out = []
    for nm in ('A', 'B'):
        out.append(tube(P[nm][1:], TR, 3.0))                         # lane -> J0 block -> guide
        out.append(tube([P[nm][0] + V(0, -2.0, 0), P[nm][1], P[nm][1] + V(6.0, 0, 0)], TR, 3.0))  # riser + bend
    out.append(tube(P['C'][1:], TR, 3.0))
    for nm in ('D+', 'D-'):
        out.append(tube([P[nm][0] + V(0, -10, 0)] + P[nm][1:8], TR, 3.0))
        out.append(tube(P[nm][7:], 0.75, 5.0))                      # V5 wing channel (dia 1.5) re-cut + exit
    # flared exits of the guide block (J0 swings +-15 deg: the tendon to the base bone fans +-4 mm)
    for nm in ('A', 'B', 'C'):
        p = P[nm][-2]
        q = P[nm][-1]
        cone = Part.makeCone(TR, 2.2, (q - p).Length + 0.4, p, q - p)
        out.append(cone)
    return out

# ------------------------------------------------------------------------------------------------ keep-outs of other agents
def _json(p):
    try:
        return json.load(open(p))
    except Exception:
        return []

def keepouts_local(key):
    out = []
    for fn in ('pillars.json', 'hardware.json'):
        for r in _json(REGIONS + '/' + fn):
            if r.get('part') == key and r.get('frame') == 'local':
                out.append(r['box'])
    return out

def hardware_world_boxes():
    return [r['box'] for r in _json(REGIONS + '/hardware.json') if r.get('frame') == 'world']

# ------------------------------------------------------------------------------------------------ palm bone
SEATS_LOCAL = [(13.9, 3.34), (33.83, 3.34), (53.76, 3.34)]   # V5 worm-seat trough axis points (palm local x, y)
SEAT_AX = V(0.858, 0.514, 0)
TPU_LAYER = (0.5, 1.1)          # pillars: TPU insert layer exactly y 0.5..1.1 - the seat fills stop at its faces
SLOT_X = 83.2                     # base-bone tongue reaches back to x 83.49 at +-15 deg (y -2.3..4.5, |z| <= 6.72)

def seat_fill(key, f):
    """V5 worm seats filled inside the 15x15 bone envelope (x 7.8..71.2), except the TPU layer, the TPU peg holes
    (pillars.json) and the hardware keep-outs (cover3 bracket screws)."""
    env = bone_section(7.8, 71.2)
    tro = [Part.makeCylinder(6.9, 50, V(x, y, 0) - SEAT_AX * 25, SEAT_AX) for x, y in SEATS_LOCAL]
    fill = env.common(tro[0].fuse(tro[1:]))
    ex = [box((-1, TPU_LAYER[0], -12), (98, TPU_LAYER[1], 12))]
    for b in keepouts_local(key):
        if b[1] >= 1.0 and b[4] - b[1] < 3.5:                         # TPU peg holes (dia 2.42, y 1.1..4.1): keep open
            cx, cz = (b[0] + b[3]) / 2, (b[2] + b[5]) / 2
            ex.append(cyl((cx, b[1] - 0.2, cz), (cx, b[4], cz), 1.21 + C))
    Mi = pm(f).inverse()
    for b in hardware_world_boxes():
        bl = xform(box((b[0] - 0.05, b[1] - 0.05, b[2] - 0.05), (b[3] + 0.05, b[4] + 0.05, b[5] + 0.05)), Mi)
        if bl.BoundBox.intersect(env.BoundBox):
            ex.append(bl)
    for r in _json(REGIONS + '/hardware.json'):            # e.g. the index cover3 bracket screws + nut pockets
        if r.get('part') == key and r.get('frame') == 'local':
            b = r['box']
            hb = box((b[0] - 0.05, b[1] - 0.05, b[2] - 0.05), (b[3] + 0.05, b[4] + 0.05, b[5] + 0.05))
            if hb.BoundBox.intersect(env.BoundBox):
                ex.append(hb)
    return fill.cut(ex[0].multiFuse(ex[1:]) if len(ex) > 1 else ex[0])

def fork_ribs(m, y0, y1, open_down=True):
    """Fork plates trapping the gearbox of motor m (palm local): 0.6 mm plate in front of the gearbox face with a
    U-slot for the boss/shaft, 0.8 mm plate behind the gearbox with the can profile; both open toward the insertion
    side and limited to the motor width (side lanes stay free)."""
    o = motor_local(m)
    lo, hi = min(y0, y1) - o.y, max(y0, y1) - o.y
    zc = HW + C
    front = box((-0.8, lo, -zc), (-0.2, hi, zc))
    rear = box((9.2, lo, -zc), (10.0, hi, zc))
    sd = 1 if open_down else -1
    slot = cyl((-2, 0, 0), (1, 0, 0), 2.2).fuse(box((-2, 0, -2.2), (1, sd * 12, 2.2)))
    can = Part.makeCylinder(HH + C, 3, V(8.5, 0, 0), V(1, 0, 0)).common(box((8, -8, -zc), (11, 8, zc)))
    can = can.fuse(box((8, 0, -zc), (11, sd * 12, zc)))
    return xform(front.cut(slot).fuse(rear.cut(can)), motor_matrix_local(m))

def _step(tag, s, ref=None):
    """Guard for the long boolean chain: raise with a clear message if a step breaks the solid."""
    if os.environ.get('ACT_DEBUG'):
        print('   palm step %-10s valid %s solids %d vol %.1f' % (tag, s.isValid(), len(s.Solids), s.Volume), flush=True)
    if ref is not None and (len(s.Solids) != 1 or s.Volume < 0.5 * ref):
        raise RuntimeError('palm_modify: step %s broke the part (%d solids, vol %.1f vs %.1f)' % (tag, len(s.Solids), s.Volume, ref))
    return s

def _cut(s, tool, tag, ref):
    """Cut, and if OCC returns garbage retry with the tool slightly fuzzed (Part fuzzy boolean)."""
    r = s.cut(tool)
    if len(r.Solids) == 1 and r.Volume > 0.5 * ref and r.isValid():
        return r
    try:
        r2 = s.cut(tool, 1e-4)
        if len(r2.Solids) == 1 and r2.Volume > 0.5 * ref and r2.isValid():
            return r2
    except Exception:
        pass
    return _step(tag, r, ref)

def _fuse_safe(s, p, depth=0):
    r = s.fuse(p)
    if len(r.Solids) == 1 and r.isValid() and r.Volume > s.Volume - 1e-3:
        return r
    if depth >= 3:
        raise RuntimeError('palm_modify: fuse failed (piece vol %.2f, result %d solids vol %.1f)' % (p.Volume, len(r.Solids), r.Volume))
    bb = p.BoundBox                                       # split the piece in two along its longest side
    if bb.XLength >= max(bb.YLength, bb.ZLength):
        h = [box((bb.XMin - 1, bb.YMin - 1, bb.ZMin - 1), (bb.Center.x, bb.YMax + 1, bb.ZMax + 1)),
             box((bb.Center.x, bb.YMin - 1, bb.ZMin - 1), (bb.XMax + 1, bb.YMax + 1, bb.ZMax + 1))]
    elif bb.YLength >= bb.ZLength:
        h = [box((bb.XMin - 1, bb.YMin - 1, bb.ZMin - 1), (bb.XMax + 1, bb.Center.y, bb.ZMax + 1)),
             box((bb.XMin - 1, bb.Center.y, bb.ZMin - 1), (bb.XMax + 1, bb.YMax + 1, bb.ZMax + 1))]
    else:
        h = [box((bb.XMin - 1, bb.YMin - 1, bb.ZMin - 1), (bb.XMax + 1, bb.YMax + 1, bb.Center.z)),
             box((bb.XMin - 1, bb.YMin - 1, bb.Center.z), (bb.XMax + 1, bb.YMax + 1, bb.ZMax + 1))]
    for hb in h:
        q = p.common(hb)
        if q.Volume > 1e-4:
            s = _fuse_safe(s, q, depth + 1)
    return s

def palm_modify(key, shape):
    f = key[5:]
    body = box((0.8, 5.2, -(BAY_Z + MWALL)), (8.0, 20.1, BAY_Z + MWALL)).fuse(
        box((8.0, 4.6, -(BAY_Z + MWALL)), (SLOT_X, 20.1, BAY_Z + MWALL))).fuse(   # y 4.6: hugs the bone corners
        box((SLOT_X, 4.7, -(BAY_Z + MWALL)), (BAY_X1, 20.1, BAY_Z + MWALL)))      # J0 slot floor = back-lug face
    # cheeks carry the J0 ramps; in front of SLOT_X they stay outside the base-bone tongue's +-15 deg sweep
    cheeks = [box((70.0, 2.0, sz * 5.0), (SLOT_X, 8.2, sz * 8.5)) for sz in (-1, 1)]
    cheeks += [box((SLOT_X - 0.1, 2.0, sz * 6.95), (BAY_X1, 8.2, sz * 8.5)) for sz in (-1, 1)]
    cheeks += [box((81.5, 2.0, -5.5), (SLOT_X, 4.8, 5.5))]    # fills the unused rear of the J0 slot over the A/B drops
    cheeks += [box((70.0, 2.0, sz * 4.0), (76.0, 5.2, sz * 5.0)) for sz in (-1, 1)]    # close a V5 notch by the D ramps
    strips = [box((8.0, 3.6, sz * 5.0), (70.0, 5.2, sz * (BAY_Z + MWALL))) for sz in (-1, 1)]  # tunnel roof strips
    pegs = [cyl(((b[0] + b[3]) / 2, 0.9, (b[2] + b[5]) / 2), ((b[0] + b[3]) / 2, b[4], (b[2] + b[5]) / 2), 1.21 + C)
            for b in keepouts_local(key) if b[1] >= 1.0 and b[4] - b[1] < 3.5]
    cheeks += [st.cut(pegs) if pegs else st for st in strips]
    guide = box((GUIDE[0], GUIDE[2], -7.0), (GUIDE[1], GUIDE[3], 5.6))
    legs = [box((GUIDE[0], 5.2, sz * 2.62), (90.2, GUIDE[2] + 0.1, sz * (5.9 if sz > 0 else 6.2))) for sz in (-1, 1)]
    v5j0 = box((70.0, 1.6, -6.8), (72.6, 7.7, 6.8))       # closes the V5 J0 entry holes (the loop now uses tunnels)
    fill = seat_fill(key, f)
    slabs = [fill.common(box((x0, -9, -9), (x0 + 7.0, 9, 9))) for x0 in range(7, 72, 7)]
    s = shape
    for p in [body, guide, v5j0] + cheeks + legs + [sl for sl in slabs if sl.Volume > 1e-3]:
        s = _fuse_safe(s, p)                              # one piece at a time: OCC dropped the bone when the
    s = _step('fuse', s, shape.Volume)                    # whole seat fill was fused at once (index bone)
    ref = s.Volume
    # tier-1 bays (open bottom) + tab/connector/plug windows through the +z wall
    bay = box((BAY_X0, CEIL, -(HW + C)), (BAY_X1, 20.3, HW + C)).fuse(
        box((BAY_X0, CEIL + 0.8, -BAY_Z), (BAY_X1, 20.3, BAY_Z)))           # side lanes: ceiling 0.8 lower
    win = [xform(box((24.3, -6.2, 6.0), (37.3, 6.4, 12.0)), motor_matrix_local(m, f)) for m in ('A', 'B')]
    s = _cut(s, bay.fuse(win), 'bays', ref)
    s = _cut(s, box((0.7, 16.0, -4.0), (1.7, 17.4, 4.0)), 'hook', ref)     # rear-hook groove for the tray lip
    # fork ribs (tier 1, open downward) + bend bosses behind the A/B risers
    adds = [fork_ribs(m, CEIL - 0.1, 20.1, True) for m in ('A', 'B')]
    # descent boss over A's magnet (carved by its envelope), from just after A's PCB envelope; +z stops at A's plug
    adds.append(box((ml('A', 25.75).x, CEIL - 0.1, -BAY_Z), (BAY_X1, GUIDE[2] + 0.1, 5.9)))
    for sz in (-1, 1):
        adds.append(box((BAY_X0, CEIL - 0.1, sz * 4.0), (12.5, 8.6, sz * BAY_Z)))    # rear corner under the D tunnels
    for m in ('A', 'B'):
        t, x0, kind, joint, side = MOTORS[m]
        gx = ml(m, GROOVE[kind][0]).x
        adds.append(box((gx - 3.6, CEIL - 0.1, side * 0.9), (gx - 0.8, 8.9, side * 4.5)))
    s = _step('ribs', s.fuse(adds), ref)
    env = [motor_env_local(m, f=f) for m in ('A', 'B', 'C', 'D')]
    s = _cut(s, env[0].multiFuse(env[1:]), 'motors', ref)
    for i, t in enumerate(tendon_cutters_palm()):       # one tool at a time (multi-tool cuts left slivers)
        s = _cut(s, t, 'tendon%d' % i, ref)
    ts = tray_screw(f, tray=False)
    s = _cut(s, ts[0].fuse(ts[1:]), 'screw', ref)
    return clean(s)

# ------------------------------------------------------------------------------------------------ tray (palm local frame)
TRAY_Y0, TRAY_Y1 = 20.1, 35.1
SCREW_XZ = (91.6, -4.4)         # front tray screw (x, z): M2x12 SHCS, vertical, nut in the guide block

def tray_screw(f, tray):
    x, z = SCREW_XZ
    if tray:
        return [cyl((x, 18, z), (x, 40, z), 1.0 + C), cyl((x, 29.1, z), (x, 40, z), 1.9 + C)]  # head recessed 6 mm
    # palm side: clearance hole + hex nut pocket (M2 4.0 AF) side-loaded from -z, seated y 17.3..19.3, below the
    # tendon channels of the guide block (M2x12: head at y 29.1, tip at 17.1 through the whole nut)
    return [cyl((x, 16.0, z), (x, 20.4, z), 1.0 + C),
            hex_prism((x, 19.3, z), (0, -1, 0), (1, 0, 0), 4.0 + 2 * C, 1.6 + 2 * C + 0.4),
            box((x - 2.0 - C, 17.3, z), (x + 2.0 + C, 19.3, z - 6))]

def tray(f):
    x0 = ml('D', -11.4).x - WALL
    x1 = SCREW_XZ[0] + 2.8
    s = box((x0, TRAY_Y0, -(TRAY_Z + WALL)), (x1, TRAY_Y1, TRAY_Z + WALL))
    s = s.cut(box((x0 + WALL, TRAY_Y0 - 1, -TRAY_Z), (86.8, TRAY_Y1 - WALL, TRAY_Z)))
    # front block under the guide (screw), keep clear of C's plug keep-out (+z)
    zs = -1 if (f, 'C') in TAB_FLIP else 1
    s = s.cut(box((86.8, TRAY_Y0 - 1, zs * 5.3), (x1 + 1, TRAY_Y1 + 1, zs * 9)))
    # rear hook: upright behind the magazine rear wall (0.2 gap) + tongue into its groove (0.2 all round)
    s = s.fuse([box((x0, 16.2, -3.8), (0.6, TRAY_Y0 + 0.1, 3.8)), box((0.5, 16.2, -3.8), (1.5, 17.2, 3.8))])
    s = s.fuse([fork_ribs(m, 19.9, TRAY_Y1 - WALL + 0.1, False) for m in ('C', 'D')])
    # tier-1 carry pads under the A/B gearboxes
    shelves = []
    for m in ('A', 'B'):                  # carry pads: inside the magazine walls above the mating plane, full width below
        x = motor_local(m).x
        shelves += [box((x + 0.2, 19.9, -(BAY_Z - C)), (x + 8.8, TRAY_Y0 + 0.01, BAY_Z - C)),
                    box((x + 0.2, TRAY_Y0, -(TRAY_Z + 0.1)), (x + 8.8, TRAY_Y0 + 1.0, TRAY_Z + 0.1))]
    s = s.fuse(shelves)
    env = [motor_env_local(m, f=f) for m in ('A', 'B', 'C', 'D')]
    s = s.cut(env[0].multiFuse(env[1:]))
    s = s.cut([xform(box((24.3, -6.4, 6.0), (37.3, 6.4, 12.0)), motor_matrix_local(m, f)) for m in ('C', 'D')])
    P = tendon_paths()
    s = s.cut([tube([P['C'][0] + V(0, 6, 0), P['C'][1] + V(0, 1, 0)], TR + 0.2, 3.0)] +
              [tube([P[k][0] + V(0, 6, 0), P[k][1] + V(0, 12, 0)], TR + 0.2, 3.0) for k in ('D+', 'D-')])
    ts = tray_screw(f, tray=True)
    s = s.cut(ts[0].multiFuse(ts[1:]))
    tri = Part.Face(Part.makePolygon([V(82.0, TRAY_Y1 + 0.2, -12), V(96.0, TRAY_Y1 + 0.2, -12), V(96.0, 28.0, -12),
                                      V(82.0, TRAY_Y1 + 0.2, -12)])).extrude(V(0, 0, 24))
    s = s.cut(tri)
    return clean(s)

# ------------------------------------------------------------------------------------------------ finger bones
V5CH = {   # V5 dia-2 channels (data/sw_bone_channels.csv): (y, z, x0, x1) in the Onshape part frame
    'metacarpal': [(0.05, 4.70, 15.2, 31.8), (0.05, -4.60, 15.2, 31.8)],
    'metacarpal_pinky': [(0.05, -4.60, 15.2, 24.8), (0.05, 4.70, 15.2, 24.8)],
    'proximal': [(0.05, 4.70, 1.1, 55.6), (0.05, -4.60, 0.1, 56.7)],
    'proximal_middle': [(0.25, 4.75, 1.1, 63.6), (0.20, -4.55, 0.1, 64.7)],
    'proximal_pinky': [(0.05, 4.70, 1.1, 45.6), (0.05, -4.60, 0.1, 46.7)],
    'distal': [(-0.05, -4.60, 0.5, 21.4), (0.20, -0.95, 1.1, 13.3), (0.15, 0.95, 1.1, 13.3), (0.95, -2.70, 0.5, 8.3),
               (-0.10, 2.55, 0.5, 10.0), (0.00, 4.70, 0.7, 21.4), (-0.55, -2.80, 0.3, 21.4)],
}
BORES = {'proximal': (7.5, 49.2), 'proximal_middle': (7.5, 57.2), 'proximal_pinky': (7.5, 39.2)}
ANCHOR_X = 19.0                 # knot pocket (dia 3.0 from the palm face) for the J1 (metacarpal) / J2 (proximal) flexor

def distal_anchor():
    """V5 M1.6 tendon anchor (sw_instances Mirrorm1_screw_assemb_fingies-1), in the distal part frame."""
    Mi = OCC['Distal Phalanx Bone_V02 <1>'].inverse()
    w = lambda x, y, z: Mi.multVec(V(x, y, z))
    yc, zc = -0.2539, -0.5230
    parts = [cyl(w(44.17, yc, zc), w(52.17, yc, zc), 1.92),              # head counterbore from the tip
             cyl(w(36.17, yc, zc), w(44.17, yc, zc), 1.05)]              # M1.6 hole
    a, b = w(41.17, yc, zc), w(42.97, yc, zc)
    parts.append(hex_prism(a, b - a, V(0, 1, 0), 3.2 + 2 * C, (b - a).Length))   # M1.6 nut
    c = w(42.07, 8.29, -1.12)
    parts.append(box((min(a.x, b.x), c.y - 8.0, c.z - 1.85), (max(a.x, b.x), c.y + 8.0, c.z + 1.85)))  # nut slot
    return parts

def finger_modify(key, shape):
    cuts = [cyl((x0, y, z), (x1, y, z), 1.05) for (y, z, x0, x1) in V5CH.get(key, [])]
    if key == 'distal':
        cuts += distal_anchor()
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(21.6, 0, 4.7), V(-1, 0, 0)))
    if key in BORES:
        j2, j3 = BORES[key]
        # open-roof flexor slots: roof removed from the tongue ends to 7.7 past each axis (no flexor reversal)
        cuts.append(box((-1.0, -0.8, 4.0), (j2 + 7.7, 0.8, 8.5)))
        cuts.append(box((j3 - 7.7, -0.8, 4.0), (j3 + 9.0, 0.8, 8.5)))
        cuts.append(cyl((ANCHOR_X, 0, 3.4), (ANCHOR_X, 0, 8.5), 1.5))   # J2 flexor knot pocket
    if key in ('metacarpal', 'metacarpal_pinky'):
        cuts.append(cyl((ANCHOR_X, 0, 3.4), (ANCHOR_X, 0, 8.5), 1.5))   # J1 flexor knot pocket
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(14.9, 0.05, 4.7), V(1, 0, 0)))  # flared rear entry
        rear_j2 = 39.5 if key == 'metacarpal' else 32.5
        cuts.append(Part.makeCone(2.0, 1.05, 1.4, V(rear_j2 - 7.4, 0.05, 4.7), V(-1, 0, 0)))
    if not cuts:
        return shape
    tool = cuts[0].multiFuse(cuts[1:]) if len(cuts) > 1 else cuts[0]
    return clean(shape.cut(tool))

# ------------------------------------------------------------------------------------------------ base bone
def base_modify(shape):
    """Flexor entry from below (behind the J1 bore, V5 connector position), groove over the J1 head, J0 loop anchors."""
    x = BASE_ENTRY_X
    cuts = [box((x - 0.7, -2.2, -8.5), (x + 0.7, 2.2, 5.5)),                               # riser slot (3 tendons)
            Part.makeCone(4.0, 0.7, 2.5, V(x, 0, -9.0), V(0, 0, 1)).common(box((x - 3, -4.5, -10), (x + 3, 4.5, 0)))]
    # groove over the J1 head round (axis y at (7.5, z 0)): floor R 6.4, width 2.4, from the riser exit over the top
    ring = Part.makeCylinder(9.0, 2.4, V(7.5, -1.2, 0), V(0, 1, 0)).cut(Part.makeCylinder(6.4, 2.4, V(7.5, -1.2, 0), V(0, 1, 0)))
    wedge = box((-2.0, -1.3, 1.0), (x + 0.1, 1.3, 10.0))
    cuts.append(ring.common(wedge))
    # J0 loop knot anchors: lateral dia 1.2 through-hole + dia 2.6 knot pockets from the underside
    cuts.append(cyl((16.8, -8, -2.0), (16.8, 8, -2.0), TR))
    for sy in (-1, 1):
        cuts.append(cyl((16.8, sy * 4.4, -8.5), (16.8, sy * 4.4, -1.4), 1.3))
    return clean(shape.cut(cuts))

# ------------------------------------------------------------------------------------------------ thumb T1 / T2 (world)
# T1/T2 sit in their V5 seat towers on cover3 (the stepped towers, each with a turned 13.0 x 10.5 gearbox pocket):
# centred on the pocket and turned to its angle, the 13.0 side taking the gearbox's 12 mm width.  T2 is turned so its
# tab and plug stay inside cover3's outline (toward T3).  The V5 pockets are open at the top: T2's is filled down to
# the gearbox face as its floor; T1 stands 14 mm above its tower top (spool level with T2's for the tendon hand-off),
# so its tower gets a collar (pocket + T_WALL) up to the floor.  Seat centres/angles measured on the V5 cover3.
THUMB = {'T1': dict(xy=(-186.02, 21.09), rot=-7.56, zf=-24.2, top=-38.61, joint='thumb R4 flexor (thumb metacarpal)'),
         'T2': dict(xy=(-147.93, 26.52), rot=-65.56, zf=-25.8, top=-23.10, joint='thumb R5+R6 flexor (thumb distal)')}
T_SPOOL_DX = -2.0               # T1/T2 spool sits 2 mm further out: 2.8 mm floor between gearbox face and spool
T_FLOOR = 2.8
T_POCKET = (13.0, 10.5)         # V5 gearbox pocket: width (along the seat angle) x thickness
T_WALL, T_CR = 1.6, 2.0         # T1 collar wall and corner radius

def _rotz(deg):
    a = math.radians(deg)
    return App.Matrix(math.cos(a), -math.sin(a), 0, 0, math.sin(a), math.cos(a), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

def thumb_placement(t, turned=True):
    d = THUMB[t]
    x, y = d['xy']
    # n20 frame: +X (encoder) = -Z world (shaft up), +Y = +X world, +Z (tab) = -Y world; then turned by the seat angle
    M = _rotz(d['rot'] if turned else 0.0).multiply(App.Matrix(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1))
    M.move(V(x, y, d['zf']))
    return App.Placement(M)

def thumb_env_motor(c=C):
    """T1/T2 cutter in the motor frame: body + plug keep-out, open downward (inserted from below), floor
    x -2.8..0 left in place except the boss/shaft hole and the 2x M1.6 holes."""
    env = n20.body_envelope(c).fuse([n20.plug_keepout(8.0), box((0.2, -6.0 - c, -5.0 - c), (40.0, 6.0 + c, 5.0 + c)),
                                      box((24.0, -6.0 - c, -5.0 - c), (40.0, 6.0 + c, 11.5 + c))])
    env = env.fuse([cyl((-3.0, 0, 0), (0.5, 0, 0), 1.5 + c), cyl((-1.0, 0, 0), (0.5, 0, 0), 2.0 + c)])
    for (hx, hy, hz) in n20.MOUNT_HOLES:
        env = env.fuse([cyl((-3.0, hy, hz), (0.5, hy, hz), 0.8 + c), cyl((-3.2, hy, hz), (-1.0, hy, hz), 1.6 + c)])
    return env

def thumb_env_well():
    """Spool well above the floor + tendon exit window from the spool groove toward -Y world and the gimbal side.
    Placed unturned (thumb_placement(t, turned=False)): the window keeps the tendon hand-off whatever the seat angle."""
    x0, x1 = SPAN['small']
    well = Part.makeCylinder(RF['small'] + 0.4, (x0 - x1) + 0.6, V(x1 + T_SPOOL_DX - 0.4, 0, 0), V(1, 0, 0))
    well = well.fuse(cyl((-12.0, 0, 0), (-2.9, 0, 0), 1.5 + 0.4))
    gx = GROOVE['small'][0] + T_SPOOL_DX
    return well.fuse(box((gx - 1.6, -RF['small'] - 0.4, 0.0), (gx + 1.6, RF['small'] + 0.4, 30.0)))

def thumb_seat_fill(t):
    """World solid added to cover3 at seat t: T1 collar (tower extended up to the floor top), T2 floor (open pocket
    top filled from the gearbox face to the tower top)."""
    d = THUMB[t]
    hu, hv = T_POCKET[0] / 2, T_POCKET[1] / 2
    if t == 'T1':
        hu, hv, r, z0, z1 = hu + T_WALL, hv + T_WALL, T_CR, d['top'] - 0.6, d['zf'] + T_FLOOR
    else:
        hu, hv, r, z0, z1 = hu + 0.05, hv + 0.05, 0.0, d['zf'], d['top']
    b = Part.makeBox(2 * hu, 2 * hv, z1 - z0, V(-hu, -hv, 0))
    if r > 0:
        b = b.makeFillet(r, [e for e in b.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[-1].Point.z) > 1e-6])
    M = _rotz(d['rot'])
    M.move(V(d['xy'][0], d['xy'][1], z0))
    return xform(b, M)

def cover3_modify(shape):
    Mi = OCC['driver_side_palm_cover3 <1>'].inverse()
    fills = [xform(thumb_seat_fill(t), Mi) for t in ('T1', 'T2')]
    cuts = []
    for t in ('T1', 'T2'):
        cuts.append(xform(xform(thumb_env_motor(), thumb_placement(t).toMatrix()), Mi))
        cuts.append(xform(xform(thumb_env_well(), thumb_placement(t, turned=False).toMatrix()), Mi))
    return clean(shape.fuse(fills).cut(cuts))

# ------------------------------------------------------------------------------------------------ module interface
THUMB_BONES = {'thumb_metacarpal': 'metacarpal', 'thumb_proximal': 'proximal', 'thumb_distal': 'distal'}

def modify(key, shape):
    if os.environ.get('ACT_DEBUG'):
        print('actuation.modify start', key, flush=True)
    try:
        return _modify(key, shape)
    except Exception as e:
        import traceback
        print('ACTUATION modify(%s) FAILED: %r' % (key, e), flush=True)
        traceback.print_exc()
        raise

def _modify(key, shape):
    if key in PALM_KEY.values():
        return palm_modify(key, shape)
    if key == 'base_bone':
        return base_modify(shape)
    if key in V5CH:
        return finger_modify(key, shape)
    if key in THUMB_BONES:
        return finger_modify(THUMB_BONES[key], shape)
    if key == 'cover3':
        return cover3_modify(shape)
    return shape

def new_parts():
    out = {'actuation_n20': n20.shape(), 'actuation_spool_small': spool('small'),
           'actuation_spool_big': spool('big'), 'actuation_spool_double': spool('double')}
    for f in FINGERS:
        out['actuation_tray_' + f] = tray(f)
    return out

SPOOL_KEY = {'small': 'actuation_spool_small', 'big': 'actuation_spool_big', 'double': 'actuation_spool_double'}
MOTOR_COL = (0.75, 0.75, 0.78)
SPOOL_COL = (0.95, 0.75, 0.2)

def all_motor_placements():
    """seat id -> (placement, joint text, spool kind)."""
    out = {}
    for f in FINGERS:
        L = LETTER[f]
        for m in ('A', 'B', 'C', 'D'):
            t, x0, kind, joint, side = MOTORS[m]
            out[L + m] = (motor_placement(f, m), '%s %s' % (f, joint), kind)
    for t in ('T1', 'T2'):
        out[t] = (thumb_placement(t), THUMB[t]['joint'], 'small')
    return out

def instances():
    out = []
    for sid, (pl, joint, kind) in all_motor_placements().items():
        out.append(dict(name='N20_' + sid, part='actuation_n20', placement=pl, group='Actuation', color=MOTOR_COL))
        spl = pl.multiply(App.Placement(V(T_SPOOL_DX, 0, 0), App.Rotation())) if sid in THUMB else pl
        out.append(dict(name='Spool_' + sid, part=SPOOL_KEY[kind], placement=spl, group='Actuation', color=SPOOL_COL))
    for f in FINGERS:
        out.append(dict(name='Tray_' + f, part='actuation_tray_' + f, placement=App.Placement(pm(f)),
                        group='Actuation', color=(0.3, 0.55, 0.85)))
    return out

# ------------------------------------------------------------------------------------------------ published data
SIGN_NOTE = ('positive motor command = spool turns +X_motor (right hand about the n20 +X axis); the firmware '
             'direction column is the spool sense that SHORTENS the flexor (or rotates J0 toward +Y / the thumb)')

def _rowmajor(M):
    return [round(v, 6) for v in (M.A11, M.A12, M.A13, M.A14, M.A21, M.A22, M.A23, M.A24,
                                  M.A31, M.A32, M.A33, M.A34, 0.0, 0.0, 0.0, 1.0)]

def wind_sense(sid):
    """Spool rotation sense (right hand about +X_motor) that ACTUATES the joint: winds the flexor in (A, B, C, T1,
    T2) or turns J0 toward +Y world / the thumb side (D).  Surface speed at the take-off point (0, 0, s*r) of a spool
    turning +w about +x is w * r * (-y_motor*s ...): a tendon leaving on +z going -y (toward the bone) pays out."""
    if sid == 'T1':
        return +1               # T1 tendon leaves on +Z_motor heading +Y_motor (+X world, toward the gimbal)
    if sid == 'T2':
        return -1               # T2 tendon leaves on +Z_motor heading -Y_motor (-X world)
    m = sid[1]
    side = MOTORS[m][4]
    if side == 0:               # D: +w winds the -z end in -> finger turns toward -z local (-Y world side)
        return -1
    return +1 if side < 0 else -1

# firmware channel proposal (electronics owns the final driver assignment) and tendon anchors
CHANNEL = {'ID': 0, 'IA': 1, 'IB': 2, 'IC': 3, 'MD': 4, 'MA': 5, 'MB': 6, 'MC': 7, 'RD': 8, 'RA': 9, 'RB': 10, 'RC': 11,
           'PD': 12, 'PA': 13, 'PB': 14, 'PC': 15, 'T1': 16, 'T2': 17, 'T3': 18}
ANCHOR = {'A': 'metacarpal knot pocket (x 19)', 'B': 'proximal knot pocket (x 19)', 'C': 'distal M1.6 clamp',
          'D': 'base-bone lateral knot anchors (x 16.8), loop via the palm tunnels + V5 wing channels',
          'T1': 'thumb metacarpal knot pocket', 'T2': 'thumb distal M1.6 clamp'}

def write_motors_json(path=REGIONS + '/actuation_motors.json', draft=False):
    rows = []
    MP = all_motor_placements()
    for sid, (pl, joint, kind) in MP.items():
        M = pl.toMatrix()
        cc = M.multVec(V(*n20.REF['connector_centre']))
        cd = M.multVec(V(1, 0, 0)) - M.multVec(V(0, 0, 0))
        td = M.multVec(V(0, 0, 1)) - M.multVec(V(0, 0, 0))
        ko = n20.plug_keepout(8.0)
        ko.transformShape(M, True)
        bb = ko.BoundBox
        rows.append(dict(seat=sid, matrix=_rowmajor(M), joint=joint, spool=kind,
                         spool_tendon_radius_mm=RS[kind], connector=[round(v, 3) for v in cc],
                         connector_dir=[round(v, 5) for v in cd], tab_dir=[round(v, 5) for v in td],
                         plug_keepout_world_aabb=[round(v, 2) for v in (bb.XMin, bb.YMin, bb.ZMin, bb.XMax, bb.YMax, bb.ZMax)],
                         wind_sense=wind_sense(sid), channel_proposal=CHANNEL.get(sid),
                         tendon_anchor=ANCHOR.get(sid if sid.startswith('T') else sid[-1])))
    try:
        import mod_thumb
        M3 = mod_thumb.motor_matrix()
        M3 = M3.toMatrix() if hasattr(M3, 'toMatrix') else M3
        cc = M3.multVec(V(*n20.REF['connector_centre']))
        rows.append(dict(seat='T3', matrix=_rowmajor(M3), joint='thumb R1 yaw, direct drive (owned by mod_thumb)',
                         channel_proposal=CHANNEL['T3'], spool=None, connector=[round(v, 3) for v in cc],
                         connector_dir=[round(v, 5) for v in (M3.multVec(V(1, 0, 0)) - M3.multVec(V(0, 0, 0)))],
                         tab_dir=[round(v, 5) for v in (M3.multVec(V(0, 0, 1)) - M3.multVec(V(0, 0, 0)))]))
    except Exception as e:
        rows.append(dict(seat='T3', matrix=None, joint='thumb R1 yaw (mod_thumb.motor_matrix unavailable: %s)' % e))
    d = dict(note='World 4x4 (row-major) of the lib/n20.py frame per motor: origin = gearbox front face centre, '
                  '+X toward the encoder (shaft -X), +Z = PCB tab / connector / D-flat side. connector = connector '
                  'centre (world mm), connector_dir = plug mating direction; plug_keepout_world_aabb = the 8 mm plug '
                  '+ first-bend space the harness must leave free.  Finger seats: <finger letter><A..D>; A=J1, B=J2, '
                  'C=J3 flexors, D=J0 loop.  channel_proposal = driver channel suggested by actuation (electronics '
                  'owns the final assignment).  ' + SIGN_NOTE, motors=sorted(rows, key=lambda r: r['seat']))
    if draft:
        d['draft'] = True
    json.dump(d, open(path, 'w'), indent=1)
    return d

if __name__ == '__main__' or os.environ.get('ACT_JSON'):
    d = write_motors_json(draft=bool(os.environ.get('ACT_DRAFT')))
    print('wrote actuation_motors.json', len(d['motors']))
