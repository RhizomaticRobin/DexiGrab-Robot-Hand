"""V6 thumb (topic 'thumb', requirement 8): a real yaw bearing and steel-pinned gimbal joints.

Why: in V5 the thumb yaw hub (dia 7.4 stem, dia 3.4 D-bore) hung on the T3 motor's dia 3 output shaft alone, with a
~2.5 mm gap all round in the tower's square window, so every thumb load went into the shaft and gearbox. The gimbal
pins (R2 dia 6 snap pins, R3 dia 6 roll shaft with a split snap head, R4 metacarpal pin) were printed.

What this module builds (all joint axes and part frames unchanged, so data/mates.csv stays valid):
  cover3        T3 tower rebuilt: old seat/window filled, a turret (r 11.0) with a dia 18.4 journal cup up to Z -15.0,
                a 2.8 mm floor that the N20 gearbox face is screwed to (2x M1.6x3), N20 pocket open at the bottom
                (motor goes in from below, encoder tab toward -Y), plug keep-out, one M2 keeper screw (north side)
                that runs in a partial groove of the turntable = lift retention + yaw hard stops (2.5 deg outside
                the Revolute 5 limits).
  thumb_hinge1  yaw carrier (was the yoke): dia 18 x 6.3 journal turntable + platform (thrust face on the turret top)
                + two 6 mm arms (tube-side arm trimmed to r 14.2 about R1 for the index wrist segment, all of it
                inside r 16.0 for cover3's wall strip). R2: M2x8 + nyloc clamped in the +s arm (screw end = pin);
                a 4x3 mm steel tube glued in the -s arm (tube end = pin; the tendon runs through it along R2).
  thumb_hub     D-coupler (was the hub): 7 x 7 square sleeve, D-bore for the N20 shaft, floats 0.2 in its pocket so
                only torque reaches the motor shaft (the journal has the tighter clearance).
  thumb_hinge2  gimbal cross: closed barrel (r 7.5) around the R3 bore (dia 7.4) with a front thrust boss, blind R2 pin
                holes, tendon hole along R2 and tendon exit slots, one M2 keeper (+hex nut) for the R3 roll shaft.
  thumb_hinge3  roll shaft: dia 7 shaft (printed lying down), keeper groove = axial retention + roll stops (rest -58, -93..-23, stops 5 deg outside),
                collar thrust face, 6 mm tongue with an M2 hole at R4, Ø2 tendon cross holes at the gimbal centre.
  thumb_metacarpal (new key, relinked for 'Metacarpal Bone_V02 <4>'): R4 proximal clevis cheeks thickened to
                4.3 mm, integral pin removed, M2x14 + nyloc through-bolt at R4 (head counterbore, nyloc pocket);
                R5 distal clevis: integral pin removed, slot narrowed 10.2 -> 9.86, cheek tip relieved r7.6
                (0.2 over the proximal tongue's r7.4 drum), M2x16 + nyloc through-bolt.
  thumb_proximal (new key, relinked for 'Proximal Phalanx Bone_V02 <3>'): both O6.8 tongue bores plugged and
                redrilled O2.4 (M2 clearance) -> R5 and R6 pivot on steel M2 bolts, no more 1.15 mm pin slop.
  thumb_distal (new key, relinked for 'Distal Phalanx Bone_V02 <5>'): R6 clevis: integral O5 pin removed,
                slot narrowed to 9.8, tip relieved r7.6, M2x16 + nyloc through-bolt.
  thumb_n20 / T3_motor, thumb_r2_tube: purchased parts placed by this module (fasteners: regions/fasteners_thumb.json).

Opposition re-clock (2026-09-26, from the user's Leap-measured thumb): the flex stack (roll shaft, metacarpal, proximal,
distal) rests re-clocked about the roll axis R3 at Revolute 2 = -58 (was +4.4) so the thumb curls across the palm toward the
fingers; passive roll -93..-23 (rest -58) with keeper stops 5 deg outside; yaw -85..+75 (stops 2.5 deg outside). New instance placements come from
placements() (needs build_v6 support; work/thumb/apply_rest.py applies them to a test build); part-local frames unchanged.
Limits/rest published in data/v6_joints.json. Tendons: entry fan (roll range) at the gimbal centre, axial bore on the roll axis, pad/back branches.

Gimbal frame used below: origin G = R1 x R2 x R3 intersection; x = thumb direction (R3), y = R2 axis (e2), z = up.
"""
import os, sys, json, math
import FreeCAD as App, Part

LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
if LIB not in sys.path:
    sys.path.insert(0, LIB)
from v6geom import cyl, box, hex_prism, clean
import n20

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
C = 0.2                              # clearance per side (requirement 7)

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
M_HUB = OCC['hand_pulley_wheel_thumb <1>']
M_YOKE = OCC['first_thumb_hinge <1>']
M_CROSS = OCC['second_thumb_hinge <1>']
M_THIRD = OCC['third_thumb_hinge <1>']
M_META = OCC['Metacarpal Bone_V02 <4>']
M_COVER3 = OCC['driver_side_palm_cover3 <1>']

def _dir(M, v):
    d = M.multVec(V(*v)) - M.multVec(V(0, 0, 0))
    d.normalize()
    return d

def _closest(p1, d1, p2, d2):
    w = p1 - p2
    a, b, c, dd, e = d1.dot(d1), d1.dot(d2), d2.dot(d2), d1.dot(w), d2.dot(w)
    den = a * c - b * b
    s, t = (b * e - c * dd) / den, (a * e - b * dd) / den
    return (p1 + d1 * s + p2 + d2 * t) * 0.5

EZ = V(0, 0, 1)
E2 = _dir(M_YOKE, (0, 0, 1))                          # R2 axis (yoke bore), world, current yaw
G = _closest(M_HUB.multVec(V(0, 0, 0)), _dir(M_HUB, (0, 0, 1)), M_YOKE.multVec(V(0, 0, 0)), E2)
R1X, R1Y = G.x, G.y                                   # yaw axis R1 (vertical)
E3 = _dir(M_CROSS, (1, 0, 0))                         # R3 axis (cross bore), toward the thumb tip
E2C = _dir(M_CROSS, (0, 0, -1))                       # R2 axis as seen by the cross (== E2)
R4P = M_META.multVec(V(7.5, 0, 0))                    # R4 axis point (world)

class Frame:
    """Right-handed frame (origin, x, y); z = x cross y. Shapes are built in frame coordinates and mapped to world."""
    def __init__(self, origin, x, y):
        x = V(x); x.normalize()
        y = V(y) - x * V(y).dot(x); y.normalize()
        z = x.cross(y)
        self.o, self.x, self.y, self.z = V(origin), x, y, z
        self.M = App.Matrix(x.x, y.x, z.x, origin.x, x.y, y.y, z.y, origin.y, x.z, y.z, z.z, origin.z, 0, 0, 0, 1)
        self.Minv = self.M.inverse()
    def w(self, s):
        return s.transformed(self.M, True)
    def pt(self, x, y, z):
        return self.M.multVec(V(x, y, z))
    def loc(self, p):
        return self.Minv.multVec(V(p))

YF = Frame(G, E2.cross(EZ), E2)                       # yaw frame: x = horizontal thumb dir, y = R2, z = up
GF = Frame(G, E3, E2C)                                # gimbal frame of the cross / roll shaft (pitched with R2)

def to_local(shape_world, M):
    return shape_world.transformed(M.inverse(), True)

def zw(zworld):
    """YF z coordinate of a world height."""
    return zworld - G.z

# ------------------------------------------------------------------------------------------------ parameters
# yaw bearing / T3 stack (world Z)
Z_TOWER_TOP = -27.504      # V5 T3 tower top (cover3 local z -6.113)
Z_F = -24.5                # N20 gearbox front face = underside of the cup floor
Z_FLOOR_TOP = -21.7        # floor 2.8 thick (M1.6 heads counterbored 1.8)
Z_TT_BOT = -21.3           # turntable underside (0.4 above the floor; the thrust face is the platform)
Z_RIM = -15.0              # turret top = thrust ring
Z_PLAT_BOT = Z_RIM + C     # platform underside
Z_PLAT_TOP = -12.3         # platform top between the arms (pitch +25 clearance of the re-clocked metacarpal)
R_TT = 9.0                 # turntable (journal) radius
R_CUP = R_TT + C
R_TURRET = 11.0            # T2 gearbox pocket (actuation draft) starts 11.2 from R1
Z_KEEP = -18.3             # yaw keeper screw axis height
KEEP_A = 8.8               # keeper axis distance from R1 (north side, +Y): 1.2 mm engagement in the rim
KEEP_HALF = 5.2            # head seat / nut face at +-5.2 along the keeper chord
R_GROOVE = KEEP_A - 1.0 - C
YAW_LIMITS = (-85.0, 75.0)  # Revolute 5 limits (deg); V5 had -70; Leap: 6 % of frames at a -75 stop
YAW_STOP = 2.5             # hard stops 2.5 deg outside the limits (keeper >= 0.2 free at the limits)
# coupler
CPL = 7.0                  # square sleeve across flats
Z_CPL = (-21.1, -13.5)
Z_POCKET_TOP = -13.3
# arms / cross / roll shaft (gimbal frame, mm from G)
ARM_IN, ARM_OUT = 8.4, 14.4   # arm faces along R2 (|y|)
ARM_R = 7.5                   # arm disc radius about R2
R_TRIM = 14.2                 # tube-side arm trimmed to this radius about R1
R_OUTER = 16.0                # whole carrier inside this radius about R1
CROSS_W = ARM_IN - C          # cross half width along R2
BARREL_R = 7.5
BORE_R = 3.5 + C              # R3 bore (shaft dia 7)
SHAFT_R = 3.5
SHAFT_X = (-7.3, 7.7)
COLLAR = (7.7, 8.6, 5.0)      # x0, x1, r
TONGUE_H = 3.0                # tongue half thickness (6.0)
TONGUE_R = 4.85               # R4 end radius
BOSS_R = 5.5                  # front thrust boss of the cross (x 0..7.5)
K3 = (4.0, 3.6)               # R3 keeper axis (x, z) in GF, along y
TENDON_X = 9.0                # axial tendon bore ends / branches here (beyond the barrel front face 7.5)
K3_R_BOSS = 3.34
PIN_END = 4.9                 # R2 blind holes end |y|
# Opposition re-clock (Leap fit of the user's hand, ~/leap/reports/thumb_rom_from_leap.md): the flex stack (roll shaft,
# metacarpal, proximal, distal) is re-placed about the roll axis R3 so the thumb flexion axis turns ~62 deg toward the
# measured one; the passive roll range stays +-15 about the new rest. Part-local joint frames are unchanged.
ROLL_REST = -58.0             # Revolute 2 rest (Onshape mate value, deg); V5 CAD pose was +4.4
ROLL_HALF = 35.0              # passive roll +35 above the rest (Leap: 8 % of frames at a -33 stop)
ROLL_NEG = 35.0               # and -35 below it (Leap: 15 % of frames sat at the -83 stop of a +-25 range)
R3_LIMITS = (ROLL_REST - ROLL_NEG, ROLL_REST + ROLL_HALF)     # Revolute 2 limits (deg): -93 .. -23
R3_LIMIT = 15.0
R3_STOP = 5.0                 # roll hard stops 5 deg outside the limits (keeper >= 0.2 free at +-15)
TUBE_OD, TUBE_ID = 4.0, 3.0
# metacarpal R4 clevis (metacarpal local)
META_SLOT = 3.2               # new cheek inner face |y| (slot 6.4 for the 6.0 tongue)
META_CB = (5.3, 2.1)          # head counterbore: seat |y|, radius
META_NUT_FACE = 4.2           # nyloc bearing face |y|

# ------------------------------------------------------------------------------------------------ helpers
def _cyl_axis(frame_pts, r):
    a, b = frame_pts
    return cyl(a, b, r)

def _prism_xz(pts_xz, y0, y1):
    """Polygon in the (x, z) plane extruded along y from y0 to y1 (frame coordinates)."""
    w = Part.makePolygon([V(x, y0, z) for x, z in pts_xz] + [V(pts_xz[0][0], y0, pts_xz[0][1])])
    return Part.Face(w).extrude(V(0, y1 - y0, 0))

def _sector_yz(r, a0, a1, x0, x1, n=24):
    """Pie sector in the (y, z) plane (angles from +z toward +y, deg), extruded along x."""
    pts = [V(x0, 0, 0)]
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        pts.append(V(x0, r * math.sin(a), r * math.cos(a)))
    pts.append(V(x0, 0, 0))
    return Part.Face(Part.makePolygon(pts)).extrude(V(x1 - x0, 0, 0))

def _sector_xy(r, a0, a1, z0, z1, n=48):
    """Pie sector in the (x, y) plane (angles from +x toward +y, deg), extruded along z."""
    pts = [V(0, 0, z0)]
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        pts.append(V(r * math.cos(a), r * math.sin(a), z0))
    pts.append(V(0, 0, z0))
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, 0, z1 - z0))

def _fuse(shapes):
    s = shapes[0]
    if len(shapes) > 1:
        s = s.fuse(shapes[1:])
    return s.removeSplitter()

def _cut(s, shapes):
    return s.cut(shapes).removeSplitter()

# ------------------------------------------------------------------------------------------------ motor pose
def motor_matrix():
    """World pose of the n20.py frame for T3: shaft up (+Z) on R1, encoder tab (+Z local) toward -Y."""
    x, y, z = V(0, 0, -1), V(1, 0, 0), V(0, -1, 0)
    o = V(R1X, R1Y, Z_F)
    return App.Matrix(x.x, y.x, z.x, o.x, x.y, y.y, z.y, o.y, x.z, y.z, z.z, o.z, 0, 0, 0, 1)

def yaw_theta0():
    return mate_theta0('Revolute 5')

def mate_theta0(name):
    """Current value (deg) of a revolute mate, computed like scripts/rom_sweep.py (entity 2 frame vs entity 1 frame)."""
    d = json.load(open(os.path.join(ROOT, 'data/top_assembly_definition.json')))
    for f in d['rootAssembly']['features']:
        fd = f['featureData']
        if f['featureType'] == 'mate' and fd.get('name') == name:
            (e1, e2) = fd['matedEntities']
            inst = {i['id']: i for i in d['rootAssembly']['instances']}
            for sa in d['subAssemblies']:
                for i in sa['instances']:
                    inst[i['id']] = i
            def wf(e):
                M = OCC[inst[e['matedOccurrence'][-1]]['name']]
                cs = e['matedCS']
                return _dir(M, cs['xAxis']), _dir(M, cs['zAxis'])
            xa, za = wf(e1)
            xb, zb = wf(e2)
            return math.degrees(math.atan2(xa.cross(xb).dot(za), xa.dot(xb)))
    raise KeyError(name)

def roll_reclock():
    """Re-clock of the flex stack about R3 relative to the V5 CAD pose, deg, in the Revolute 2 mate sense
    (mate value = theta0 + delta; child = third hinge = entity 1, so it turns by +delta about entity 1's z = -E3)."""
    return ROLL_REST - mate_theta0('Revolute 2')

def reclock_placement():
    """World rotation that takes the flex stack from the V5 CAD pose to the re-clocked rest."""
    return App.Placement(V(0, 0, 0), App.Rotation(E3 * -1, roll_reclock()), G)

FLEX_STACK = ('third_thumb_hinge <1>', 'Metacarpal Bone_V02 <4>', 'Proximal Phalanx Bone_V02 <3>', 'Distal Phalanx Bone_V02 <5>')

def rest_matrix(label):
    """World placement matrix of a thumb instance at the V6 rest (flex stack re-clocked)."""
    M = OCC[label]
    return reclock_placement().toMatrix().multiply(M) if label in FLEX_STACK else M

# ------------------------------------------------------------------------------------------------ cover3 (T3 tower)
T3L = V(21.906, -5.198, 0)          # R1 axis in cover3 local coordinates

def cover3_fill_local():
    """Solid that fills the V5 T3 seat, window, +Y slot and nut trap (cover3 local frame)."""
    lo = cyl((T3L.x, T3L.y, -38.6), (T3L.x, T3L.y, -17.3), 10.0)
    hi = cyl((T3L.x, T3L.y, -17.3), (T3L.x, T3L.y, -6.113), 10.0).common(box((-100, -100, -18), (30.2, 100, 0)))
    slot = box((14.9, -16.5, -35.5), (28.5, -5.0, -30.3))
    nut = cyl((16.267, -13.167, -10.6), (16.267, -13.167, -6.113), 2.9)
    return _fuse([lo, hi, slot, nut])

def turret_world():
    t = cyl((R1X, R1Y, Z_TOWER_TOP - 0.3), (R1X, R1Y, Z_RIM), R_TURRET)
    y = R1Y + KEEP_A                                         # boss that carries the keeper head seat and nut
    kb = cyl((R1X - 7.2, y, Z_KEEP), (R1X + 7.2, y, Z_KEEP), 3.4)
    kb = kb.fuse(box((R1X - 7.2, y - 3.4, Z_TOWER_TOP - 0.3), (R1X + 7.2, y, Z_KEEP)))   # buttress down to the tower
    kb = kb.common(box((R1X - 20, y - 20, Z_TOWER_TOP - 1), (R1X + 20, y + 20, Z_RIM)))  # nothing above the thrust ring
    return t.fuse(kb).removeSplitter()

def motor_pocket_world(plug=8.0):
    """N20 envelope + insertion path from below (PCB section extruded down) + plug keep-out (world).

    n20.body_envelope() leaves an uncovered 0.35 mm ring at the motor's end cap (local x 23.95..24.3,
    between the can box and the PCB box), so a cap sleeve is fused in here explicitly."""
    M = motor_matrix()
    env = n20.body_envelope(C)
    cap = cyl((n20.CAP_X[0] - 0.1, 0, 0), (n20.CAP_X[1] + 0.1, 0, 0), 5.0 + C)
    path = box((n20.PCB_X[0] - C, -6 - C, -5 - C), (45.0, 6 + C, 11.5 + C))
    keep = n20.plug_keepout(plug)
    return _fuse([env, cap, path, keep]).transformed(M, True)

def keeper_world():
    """(hole, head spot-face, nut pocket) of the yaw keeper: chord along X at Y = R1Y + KEEP_A, Z = Z_KEEP."""
    y, z = R1Y + KEEP_A, Z_KEEP
    hole = cyl((R1X - 14, y, z), (R1X + 14, y, z), 1.0 + C)
    head = cyl((R1X - 14, y, z), (R1X - KEEP_HALF, y, z), 1.9 + C)
    nut = hex_prism((R1X + KEEP_HALF, y, z), (1, 0, 0), (0, 0, 1), 4.0 + 2 * C, 10.0)
    return hole, head, nut

def cover3_modify(shape):
    fill = cover3_fill_local()
    add_w = [turret_world()]
    cuts_w = [cyl((R1X, R1Y, Z_FLOOR_TOP), (R1X, R1Y, Z_RIM + 1.0), R_CUP),                    # journal cup
              cyl((R1X, R1Y, Z_F - 0.1), (R1X, R1Y, Z_FLOOR_TOP + 0.1), 2.0 + C),             # shaft boss dia 4.4
              motor_pocket_world()]
    for sx in (-1, 1):                                                                        # 2x M1.6x3 into the gearbox face
        px = R1X + sx * 4.5
        cuts_w.append(cyl((px, R1Y, Z_F - 0.1), (px, R1Y, Z_FLOOR_TOP + 0.1), 0.8 + C))
        cuts_w.append(cyl((px, R1Y, Z_FLOOR_TOP - 1.8), (px, R1Y, Z_FLOOR_TOP + 0.1), 1.5 + C))
    cuts_w += list(keeper_world())
    Minv = M_COVER3.inverse()
    s = shape.fuse([fill] + [a.transformed(Minv, True) for a in add_w]).removeSplitter()
    s = s.cut([c.transformed(Minv, True) for c in cuts_w])
    return clean(s)

# ------------------------------------------------------------------------------------------------ yaw carrier (thumb_hinge1)
def carrier_world():
    zt0, zt1 = zw(Z_TT_BOT), zw(Z_PLAT_TOP)
    zp0 = zw(Z_PLAT_BOT)
    parts = [cyl((0, 0, zt0), (0, 0, zt1), R_TT),
             box((-ARM_R, -ARM_OUT, zp0), (ARM_R, ARM_OUT, zt1))]
    for sg in (-1, 1):
        y0, y1 = sorted((sg * ARM_IN, sg * ARM_OUT))
        parts.append(box((-ARM_R, y0, zp0), (ARM_R, y1, 0)))
        parts.append(cyl((0, y0, 0), (0, y1, 0), ARM_R))
    s = _fuse(parts)
    cuts = []
    # coupler pocket (square, 0.2 clearance) and shaft-tip clearance
    h = CPL / 2 + C
    cuts.append(box((-h, -h, zt0 - 1), (h, h, zw(Z_POCKET_TOP))))
    # yaw keeper groove: partial ring on the rim, radial end walls = hard stops
    th0 = yaw_theta0()
    lo, hi = YAW_LIMITS
    half = math.degrees(math.acos((KEEP_A - 1.0) / R_TT))
    phi_k = math.degrees(math.atan2(YF.loc(V(R1X, R1Y + 1, G.z)).y, YF.loc(V(R1X, R1Y + 1, G.z)).x))
    a0 = phi_k + (lo - YAW_STOP - th0) - half
    a1 = phi_k + (hi + YAW_STOP - th0) + half
    ring = cyl((0, 0, zw(Z_KEEP) - 1.0 - C), (0, 0, zw(Z_KEEP) + 1.0 + 2 * C), R_TT + 1).cut(
        cyl((0, 0, zw(Z_KEEP) - 2), (0, 0, zw(Z_KEEP) + 2), R_GROOVE))
    cuts.append(ring.common(_sector_xy(R_TT + 2, a0, a1, zw(Z_KEEP) - 2, zw(Z_KEEP) + 2)))
    # R2 +y: M2 clearance hole, nyloc pocket on the inner face, head counterbore (1.0) on the outer face
    cuts.append(cyl((0, ARM_IN - 0.1, 0), (0, ARM_OUT + 0.1, 0), 1.0 + C))
    cuts.append(hex_prism((0, ARM_IN - 0.1, 0), (0, 1, 0), (1, 0, 0), 4.0 + 2 * C, 3.0 + 0.1))
    cuts.append(cyl((0, ARM_OUT - 1.0, 0), (0, ARM_OUT + 0.1, 0), 1.9 + C))
    # R2 -y: steel tube (OD 4) glued in the arm
    cuts.append(cyl((0, -ARM_OUT - 0.1, 0), (0, -ARM_IN + 0.1, 0), TUBE_OD / 2 + C))
    # pitch +25 with the re-clocked flex stack: the metacarpal's rounded clevis end dips to Z -12.7 about 9 mm in front
    # of G -> top-front of the turntable/platform lowered to Z -13.3 between the arms (coupler pocket roof untouched)
    cuts.append(box((CPL / 2 + C + 0.8, -ARM_IN + 0.1, zw(Z_POCKET_TOP)), (20, ARM_IN - 0.1, 5)))
    cuts.append(box((6.5, -ARM_IN + 0.1, zw(-14.2)), (20, ARM_IN - 0.1, 5)))       # rim front: pitch +25 contact at x 8.8
    # the -y arm swings past the index wrist segment (>= 14.5 from R1 at Z -8..+2): keep it inside r 14.2
    trim = box((-20, -30, zp0 - 1), (20, -0.001, 20)).cut(cyl((0, 0, zp0 - 2), (0, 0, 25), R_TRIM))
    cuts.append(trim)
    # the +y arm corner passes cover3's wall strip (>= 16.3 from R1, near yaw +70): keep everything inside r 16.0
    cuts.append(box((-30, -30, zp0 - 1), (30, 30, 25)).cut(cyl((0, 0, zp0 - 2), (0, 0, 26), R_OUTER)))
    s = s.cut(cuts)
    return clean(YF.w(s))

# ------------------------------------------------------------------------------------------------ coupler (thumb_hub)
def coupler_world():
    z0, z1 = zw(Z_CPL[0]), zw(Z_CPL[1])
    h = CPL / 2
    sq = box((-h, -h, z0), (h, h, z1))
    for sx in (-1, 1):
        for sy in (-1, 1):                                   # 0.5 mm corner chamfers
            p = [V(sx * h, sy * h, z0 - 1), V(sx * (h - 0.7), sy * h, z0 - 1), V(sx * h, sy * (h - 0.7), z0 - 1)]
            sq = sq.cut(Part.Face(Part.makePolygon(p + [p[0]])).extrude(V(0, 0, z1 - z0 + 2)))
    s = YF.w(sq)
    # D-bore: dia 3.4, flat 1.2 from the axis on the motor's D-flat side (world -Y at the current pose)
    d = cyl((R1X, R1Y, Z_CPL[0] - 1), (R1X, R1Y, Z_CPL[1] + 1), 1.5 + C)
    d = d.common(box((R1X - 3, R1Y - (n20.SHAFT_FLAT + C), Z_CPL[0] - 1), (R1X + 3, R1Y + 3, Z_CPL[1] + 1)))
    cham = Part.makeCone(1.5 + C + 0.4, 1.5 + C, 0.4, V(R1X, R1Y, Z_CPL[0]), V(0, 0, 1))
    return clean(s.cut([d, cham]))

# ------------------------------------------------------------------------------------------------ gimbal cross (thumb_hinge2)
def cross_world():
    w = CROSS_W
    parts = [cyl((0, -w, 0), (0, w, 0), BARREL_R),
             cyl((0, 0, 0), (BARREL_R, 0, 0), BOSS_R),
             cyl((K3[0], -w, K3[1]), (K3[0], w, K3[1]), K3_R_BOSS)]
    s = _fuse(parts).common(box((-20, -w, -20), (20, w, 20)))
    cuts = [cyl((-12, 0, 0), (12, 0, 0), BORE_R),                                   # R3 bore
            cyl((0, PIN_END, 0), (0, w + 0.1, 0), 1.0 + C),                          # R2 +y M2 pin (blind)
            cyl((0, -w - 0.1, 0), (0, -PIN_END, 0), TUBE_OD / 2 + C),               # R2 -y tube pin (blind)
            cyl((0, -PIN_END - 0.1, 0), (0, 0, 0), 1.0)]                             # tendon along R2 into the bore
    # (V5-style radial tendon exits removed: after the opposition re-clock they would point into the yoke arms;
    #  the flexion tendons now leave the gimbal centre forward through the roll shaft's axial bore)
    kx, kz = K3
    cuts.append(cyl((kx, -w - 0.1, kz), (kx, w + 0.1, kz), 1.0 + C))               # R3 keeper M2 hole
    cuts.append(cyl((kx, -w - 0.1, kz), (kx, -6.0, kz), 1.9 + C))                    # head counterbore (-y face)
    fl = V(-kx, 0, -kz); fl.normalize()
    cuts.append(hex_prism((kx, 6.0, kz), (0, 1, 0), (fl.x, fl.y, fl.z), 4.0 + 2 * C, w - 6.0 + 0.1))   # hex nut pocket (+y face)
    s = s.cut(cuts)
    return clean(GF.w(s))

# ------------------------------------------------------------------------------------------------ roll shaft (thumb_hinge3)
def third_world():
    r4 = GF.loc(R4P)
    x4, z4 = r4.x, r4.z
    parts = [cyl((SHAFT_X[0], 0, 0), (SHAFT_X[1], 0, 0), SHAFT_R),
             cyl((COLLAR[0], 0, 0), (COLLAR[1], 0, 0), COLLAR[2]),
             box((COLLAR[1] - 0.01, -TONGUE_H, z4 - TONGUE_R), (x4, TONGUE_H, z4 + TONGUE_R)),
             cyl((x4, -TONGUE_H, z4), (x4, TONGUE_H, z4), TONGUE_R)]
    s = _fuse(parts)
    kx, kz = K3
    groove = cyl((kx - 1.0 - C, 0, 0), (kx + 1.0 + C, 0, 0), SHAFT_R + 1).cut(
        cyl((kx - 2, 0, 0), (kx + 2, 0, 0), kz - 1.0 - C))
    half = math.degrees(math.acos((kz - 1.0) / SHAFT_R))
    th0 = mate_theta0('Revolute 2')
    a0 = th0 - (R3_LIMITS[1] + R3_STOP) - half
    a1 = th0 - (R3_LIMITS[0] - R3_STOP) + half
    groove = groove.common(_sector_yz(SHAFT_R + 2, a0, a1, kx - 2, kx + 2))
    # tendon pass-through at the gimbal centre: entry hole that lies along R2 at the re-clocked rest, an axial bore on the
    # roll axis (decoupled from roll), then branches to the pad (+z of the flex stack) and back sides of the tongue
    rc = math.radians(roll_reclock())
    ey, ez_ = math.cos(rc), math.sin(rc)          # R2 direction seen in the shaft frame (old-pose GF), re-clocked
    cuts = [groove,
            cyl((x4, -5, z4), (x4, 5, z4), 1.0 + C),                                 # R4 M2 hole
            tendon_entry_fan(),                                                       # tendon entry from the tube side
            cyl((0, 0, 0), (TENDON_X, 0, 0), 1.0),                                    # axial bore on the roll axis
            cyl((TENDON_X, 0, 0), (TENDON_X + 7, 0, 7), 1.0),                         # branch to the pad side (flexor)
            cyl((TENDON_X, 0, 0), (TENDON_X + 7, 0, -7), 1.0),                        # branch to the back side (extensor)
            Part.makeSphere(1.0, V(TENDON_X, 0, 0))]
    s = s.cut(cuts)
    return clean(GF.w(s))


def tendon_entry_fan():
    """Dia 2 entry slot from the gimbal centre toward the tube (-s) side, fanned over the whole roll range so the
    tendon coming along R2 always meets it (shaft frame = V5 CAD pose; roll value v puts R2 at angle v - theta0)."""
    th0 = mate_theta0('Revolute 2')
    lo, hi = R3_LIMITS
    parts = []
    n = int(math.ceil((hi - lo) / 7.5))
    for i in range(n + 1):
        b = math.radians(lo + (hi - lo) * i / n - th0)
        parts.append(cyl((0, 0, 0), (0, -5 * math.cos(b), -5 * math.sin(b)), 1.0))
    return _fuse(parts)

# ------------------------------------------------------------------------------------------------ thumb metacarpal / proximal / distal (R4, R5, R6 bolts)
_SRC = {}

# V5 clevis layout (measured on the sources): cheek fingers |y| 5.1..7.5, tip rounded r7.5 about the joint
# axis, slot back wall 7.7 (R5) / 7.9 (R6) from the axis; mating tongue 9.4 wide with a r7.4 drum end; bores
# O6.8 (pin O4.5/O5.0, 0.9..1.15 loose) and nothing retaining the tongue against sliding out along y.
# V6: pin out, M2 + nyloc through-bolt in (pivot + y-retention), slot narrowed 10.2 -> 9.8 (0.2/side).
SLOT_IN = 4.9                  # narrowed slot inner face |y| (tongue 4.7 + 0.2)
CB_SEAT = 5.5                  # head counterbore seat |y| (2.0 deep from the 7.5 finger face)

def _cheek_profile(shape, y, x0, x1):
    wires = shape.slice(V(0, 1, 0), y)
    faces = [Part.Face(w) for w in wires if w.isClosed()]
    f = max(faces, key=lambda q: q.Area)
    return f.common(box((x0, y - 1, -10), (x1, y + 1, 10)))

def _clevis_bolt_cuts(xa):
    """M2 + nyloc through-bolt features on the axis (xa, y, 0): hole, head CB, nyloc pocket."""
    return [cyl((xa, -8, 0), (xa, 8, 0), 1.0 + C),
            cyl((xa, CB_SEAT, 0), (xa, 8, 0), 1.9 + C),
            hex_prism((xa, -CB_SEAT, 0), (0, -1, 0), (1, 0, 0), 4.0 + 2 * C, 3.0)]

def _tip_shave(xa):
    """Round the clevis finger tips to r7.6 about the axis (tongue drum is r7.4 -> 0.2 radial)."""
    return cyl((xa, -8, 0), (xa, 8, 0), 7.6).common(box((xa + 5.4, -9, -9), (xa + 9, 9, 9)))

def _narrow_slot(src, x0, x1, slot_in=None):
    """0.2 slabs on the clevis fingers' inner faces: slice at |y|=6 (inside the finger), land 4.9..5.1."""
    adds = []
    for sg in (1, -1):
        prof = _cheek_profile(src, sg * 6.0, x0, x1)
        si = SLOT_IN if slot_in is None else slot_in
        prof.translate(V(0, sg * (si - 6.0), 0))
        adds.append(prof.extrude(V(0, sg * (5.1 - si), 0)))
    return adds

def metacarpal_thumb(src):
    s = src
    adds = []
    for sg in (1, -1):                                                          # R4 cheeks 6.0 -> 3.2
        prof = _cheek_profile(src, sg * 6.0, -5, 15.2)
        prof.translate(V(0, sg * (5.1 - 6.0), 0))
        adds.append(prof.extrude(V(0, -sg * (5.1 - META_SLOT), 0)))
    adds += _narrow_slot(src, 31.0, 46.9, 4.93)          # R5 slot 5.1 -> 4.93 (proximal <3> sits 0.025 off-centre)
    s = s.fuse(adds).removeSplitter()
    s = s.cut([box((-1, -META_SLOT, -9), (15.2, META_SLOT, 9)),                       # R4 slot 6.4 (drops the printed pin)
               cyl((7.5, -9, 0), (7.5, 9, 0), 1.0 + C),                                # R4 M2 through hole
               cyl((7.5, META_CB[0], 0), (7.5, 9, 0), META_CB[1]),                      # R4 head counterbore (+y)
               hex_prism((7.5, -META_NUT_FACE, 0), (0, -1, 0), (1, 0, 0), 4.0 + 2 * C, 9 - META_NUT_FACE),
               cyl((39.5, -8, 0), (39.5, 8, 0), 2.25 + 0.05),                          # R5 integral pin out
               _tip_shave(39.5)] + _clevis_bolt_cuts(39.5))
    return clean(s)

def proximal_thumb(src):
    """Both O6.8 tongue bores (R5 at x 7.5, R6 at x 49.2) plugged and redrilled O2.4."""
    plugs = [cyl((xa, -4.7, 0), (xa, 4.7, 0), 3.42) for xa in (7.5, 49.2)]
    s = src.fuse(plugs).removeSplitter()
    s = s.cut([cyl((xa, -6, 0), (xa, 6, 0), 1.0 + C) for xa in (7.5, 49.2)])
    return clean(s)

def distal_thumb(src):
    """R6 clevis (axis x 29.3): pin out, slot narrowed, M2 + nyloc."""
    s = src.fuse(_narrow_slot(src, 21.5, 36.8)).removeSplitter()
    s = s.cut([cyl((29.3, -8, 0), (29.3, 8, 0), 2.5 + 0.05), _tip_shave(29.3)] + _clevis_bolt_cuts(29.3))
    return clean(s)

# ------------------------------------------------------------------------------------------------ purchased parts
TUBE_OUT = 14.0                # tube outer end |y| (inside the r 14.2 trim)

def tube_local():
    L = TUBE_OUT - PIN_END - C
    return cyl((0, 0, 0), (0, 0, L), TUBE_OD / 2).cut(cyl((0, 0, -1), (0, 0, L + 1), TUBE_ID / 2))

def tube_placement():
    o = YF.pt(0, -TUBE_OUT, 0)
    z = E2
    x = EZ
    y = z.cross(x)
    return App.Placement(App.Matrix(x.x, y.x, z.x, o.x, x.y, y.y, z.y, o.y, x.z, y.z, z.z, o.z, 0, 0, 0, 1))

# ------------------------------------------------------------------------------------------------ fastener requests
def fastener_requests():
    """Every thumb screw/nut for regions/fasteners_thumb.json (world frame; position = head seat, axis head -> tip)."""
    M_DST = rest_matrix('Distal Phalanx Bone_V02 <5>')
    M_META = rest_matrix('Metacarpal Bone_V02 <4>')          # R4/R5/R6 bolts ride on the re-clocked flex stack
    def P(v):
        return [round(v.x, 4), round(v.y, 4), round(v.z, 4)]
    out = []
    y = R1Y + KEEP_A
    out.append(dict(name='yaw_keeper', type='M2x14 SHCS', nut='M2 nyloc', nut_offset=2 * KEEP_HALF,
                    position=P(V(R1X - KEEP_HALF, y, Z_KEEP)), axis=[1, 0, 0], frame='world',
                    note='R1 yaw bearing keeper: chord through the cover3 turret, runs in the turntable rim groove '
                         '(lift retention + yaw hard stops). Fixed to cover3.'))
    out.append(dict(name='R2_pin', type='M2x8 SHCS', nut='M2 nyloc', nut_offset=2.0,
                    position=P(YF.pt(0, ARM_OUT - 1.0, 0)), axis=P(E2 * -1), frame='world',
                    note='R2 +s trunnion: clamped in the carrier arm (nyloc in the inner-face pocket); the 3.2 mm tip past '
                         'the nut IS the pivot pin in the cross (blind dia 2.4 hole). Length is intentional.'))
    kx, kz = K3
    out.append(dict(name='R3_keeper', type='M2x14 SHCS', nut='M2 hex', nut_offset=12.0,
                    position=P(GF.pt(kx, -6.0, kz)), axis=P(GF.y), frame='world',
                    note='R3 roll keeper: across the gimbal cross, runs in the roll-shaft groove (axial retention + '
                         'roll hard stops). Fixed to the cross; head on the -s face, hex nut on the +s face.'))
    out.append(dict(name='R4_bolt', type='M2x14 SHCS', nut='M2 nyloc', nut_offset=META_CB[0] + META_NUT_FACE,
                    position=P(M_META.multVec(V(7.5, META_CB[0], 0))), axis=P(_dir(M_META, (0, -1, 0))), frame='world',
                    note='R4 pivot: thumb metacarpal cheeks + roll-shaft tongue (J0-style through-bolt).'))
    out.append(dict(name='R5_bolt', type='M2x16 SHCS', nut='M2 nyloc', nut_offset=2 * CB_SEAT,
                    position=P(M_META.multVec(V(39.5, CB_SEAT, 0))), axis=P(_dir(M_META, (0, -1, 0))), frame='world',
                    note='R5 pivot: thumb metacarpal distal clevis + thumb proximal tongue (J0-style through-bolt).'))
    out.append(dict(name='R6_bolt', type='M2x16 SHCS', nut='M2 nyloc', nut_offset=2 * CB_SEAT,
                    position=P(M_DST.multVec(V(29.3, CB_SEAT, 0))), axis=P(_dir(M_DST, (0, -1, 0))), frame='world',
                    note='R6 pivot: thumb distal clevis + thumb proximal tongue (J0-style through-bolt).'))
    for i, sx in enumerate((-1, 1), 1):
        out.append(dict(name='T3_mount_%d' % i, type='M1.6x3 SHCS', nut=None,
                        position=P(V(R1X + sx * 4.5, R1Y, Z_FLOOR_TOP - 1.8)), axis=[0, 0, -1], frame='world',
                        note='T3 N20 gearbox face to the cup floor (tapped M1.6 holes of the gearbox front plate; '
                             'if absent, glue the gearbox face to the floor).'))
    return out

# ------------------------------------------------------------------------------------------------ pipeline hooks
def modify(key, shape):
    if key in ('metacarpal', 'proximal', 'distal'):
        _SRC[key] = shape.copy()
        return shape
    if key == 'cover3':
        return cover3_modify(shape)
    if key == 'thumb_hub':
        return to_local(coupler_world(), M_HUB)
    if key == 'thumb_hinge1':
        return to_local(carrier_world(), M_YOKE)
    if key == 'thumb_hinge2':
        return to_local(cross_world(), M_CROSS)
    if key == 'thumb_hinge3':
        return to_local(third_world(), M_THIRD)
    return shape

def new_parts():
    out = {'thumb_n20': n20.shape(), 'thumb_r2_tube': tube_local()}
    if 'metacarpal' in _SRC:
        out['thumb_metacarpal'] = metacarpal_thumb(_SRC['metacarpal'])
    if 'proximal' in _SRC:
        out['thumb_proximal'] = proximal_thumb(_SRC['proximal'])
    if 'distal' in _SRC:
        out['thumb_distal'] = distal_thumb(_SRC['distal'])
    return out

def instances():
    return [{'name': 'T3_motor', 'part': 'thumb_n20', 'placement': App.Placement(motor_matrix()), 'group': 'Motors',
             'color': (0.75, 0.62, 0.25)},
            {'name': 'thumb_R2_tube', 'part': 'thumb_r2_tube', 'placement': tube_placement(), 'group': 'Thumb',
             'color': (0.7, 0.7, 0.72)}]

def placements():
    """NEW HOOK (needs build_v6.py support): {instance label: App.Placement} for existing instances whose rest pose
    changes. The opposition re-clock turns the flex stack about the roll axis; part-local frames stay as they are."""
    return {lab: App.Placement(rest_matrix(lab)) for lab in FLEX_STACK}

def relink():
    out = {}
    if 'metacarpal' in _SRC:
        out['Metacarpal Bone_V02 <4>'] = 'thumb_metacarpal'
    if 'proximal' in _SRC:
        out['Proximal Phalanx Bone_V02 <3>'] = 'thumb_proximal'
    if 'distal' in _SRC:
        out['Distal Phalanx Bone_V02 <5>'] = 'thumb_distal'
    return out
