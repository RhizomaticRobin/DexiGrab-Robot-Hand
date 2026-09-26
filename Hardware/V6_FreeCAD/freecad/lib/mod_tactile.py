"""DexiGrab V6 tactile skin (topic `tactile`): FlexiTac-style piezoresistive pads on every grasp contact surface.

Reasoning, numbers, BOM and the fabrication data: freecad/notes/tactile.md.  Checks: scripts/verify_tactile.py.
FlexiTac (Huang & Li, Columbia 2026, CC BY-NC 4.0): two 0.2 mm FPCs + Velostat + lamination + Kapton edge seal,
~0.8 mm stack (budget), 2 mm taxel pitch, gold-finger tails into 0.5 mm FFC (16 rows / 32 columns), one
"Reading Board 32x16" (Arduino Nano, 1 mux + 4 shift registers) per 16 x 32 matrix, USB serial 2 Mbaud.

Architecture ("finger ray"): each finger is ONE custom FlexiTac sensor (top FPC = rows, bottom FPC = columns):
  distal + proximal + metacarpal pads (row lines shared along the finger, own column lines) + the palm pad on that
  finger's palm bone (own row lines, shares the column lines) -> 12 rows x <= 32 columns -> one standard Reading
  Board 32x16 per finger, no junction PCB.  The thumb is a fifth ray.
Geometry (all mm, clearance 0.2 per side):
  * pads sit in 0.6 mm recesses -> a 0.8 mm stack stands 0.2 proud (flush if the real stack is 0.6); the recess floor
    keeps >= 1.06 mm to every internal void (flexor channels, their flared entries, knot pockets, the distal anchor).
  * each phalanx pad's FPCs wrap the "strap" edge (R 1.0 fold), run down that side face and around the R6 back corner
    onto the back (0.6 mm channels, FPC 0.4 = both layers), where a dorsal spine channel carries the ray to the joints.
    J2/J3 are crossed on the back with free omega slack loops (the dorsal path grows ~15.7 mm at 90 deg; the palm side
    would shrink and pinch).  J1 + J0: the knuckle ribbon (free service loop under the knuckle -> lane beside the motor
    magazine -> Z step up through the web-end window -> jog into the palm bone's side curtain channel).  No pad crosses
    a joint.
  * palm bones: the section corners are squared above w 2.7 (pad bed) and the open V5 trough-3 notch (u 51.5..62.5) and
    trough-1/2 slivers are filled from the TPU layer up, so the palm pad is flat; its FPCs fold over the strap-side
    edge into a 0.6 mm curtain channel (w 0.1..5.9, a 0.6 mm lip above the TPU web slot) that continues as the digit
    ribbon at u 62.5; the ray tail leaves the pad's wrist end flat and runs over the wrist segment to the readout pod
    strapped on the arm.  The thumb ray follows mod_thumb.placements() (opposition re-clock).
  * Base Bone 1 gets no pad: its palm side is under the palm bone's J0 lug and its J1 head is swept by the
    metacarpal clevis floor (0.2 mm running gap) and carries the flexor groove.
Canonical pad frame per part: u along the bone, v across, w toward the palm (face at w = 7.5).
  phalanges: (u, v, w) = local (x, y, z);  palm bones: local (x, y, z) = (u, -w, v).
"""
import os, sys, json, math
import FreeCAD as App, Part

LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
if LIB not in sys.path:
    sys.path.insert(0, LIB)
from v6geom import cyl, box, clean

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
FC = ROOT + '/freecad'
REGIONS = FC + '/regions'
EXP = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/v5_step/Robotic Hand_V5_simulacra - '
VERSION = 'tactile-1.0'

# ------------------------------------------------------------------------------------------------ design constants
C = 0.2              # clearance per side
STACK = 0.8          # FlexiTac stack budget (2 x 0.2 FPC + Velostat + lamination + Kapton)
DEPTH = 0.6          # recess depth -> pad 0.2 proud
FPC2 = 0.4           # both FPC layers stacked (tails, curtains, spines, loops)
CH = 0.6             # tail channel depth
FACE = 7.5           # palm face at w = 7.5 in the canonical frame
FLOOR = FACE - DEPTH # 6.9
RF = 1.0             # fold radius (inner) at the strap edge
WALL = 1.0           # end walls
SIDE_WALL = 0.8      # non-strap side wall
PITCH = 2.0          # taxel pitch
ELEC = 1.5           # electrode width (0.5 gap)
SEAL = 0.5           # laminated seal margin beyond the outermost electrode edge
TRACE = 0.2          # trace pitch in tails (0.1 / 0.1 mm)
TAIL_MARGIN = 0.5    # tail edge margin per side
EPS = 0.3
TEAL = (0.0, 0.62, 0.62)
TAIL_COL = (0.85, 0.55, 0.10)     # polyimide amber for tails / loops
BOARD_COL = (0.05, 0.35, 0.60)

def tail_width(n_lines):
    return n_lines * TRACE + 2 * TAIL_MARGIN

# ------------------------------------------------------------------------------------------------ occurrences
_OCC = None
_TREST = None
def _thumb_rest():
    """V6 rest placements of the re-clocked thumb flex stack (mod_thumb.placements(); build_v6 applies them to the
    links), so the thumb pads, loops and tail follow the opposition re-clock."""
    global _TREST
    if _TREST is None:
        _TREST = {}
        try:
            import mod_thumb
            if hasattr(mod_thumb, 'placements'):
                _TREST = dict(mod_thumb.placements())
        except Exception as e:
            print('[tactile] thumb rest poses unavailable (%s): using the V5 CAD pose' % e)
    return _TREST

def occ(name):
    """World placement of an Onshape occurrence (equal to the document link placement; the thumb flex stack at the
    V6 rest pose of mod_thumb.placements())."""
    global _OCC
    tr = _thumb_rest()
    if name in tr:
        return App.Placement(tr[name])
    if _OCC is None:
        d = json.load(open(ROOT + '/data/top_assembly_definition.json'))
        ra = d['rootAssembly']
        inst = {i['id']: i for i in ra['instances']}
        for s in d['subAssemblies']:
            for i in s['instances']:
                inst[i['id']] = i
        _OCC = {}
        for o in ra['occurrences']:
            t = o['transform']
            _OCC[inst[o['path'][-1]]['name']] = App.Placement(App.Matrix(
                t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000, t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))
    return _OCC[name]

def xform(shape, M):
    s = shape.copy()
    s.transformShape(M if isinstance(M, App.Matrix) else M.toMatrix())
    return s

# ------------------------------------------------------------------------------------------------ canonical frames
PALM_KEYS = ('palm_index', 'palm_middle', 'palm_ring', 'palm_pinky')
M_PALM = App.Matrix(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1)      # canonical (u,v,w) -> palm local (u,-w,v)

def canon_matrix(key):
    return M_PALM if key in PALM_KEYS else App.Matrix()

def to_local(key, shape):
    return xform(shape, canon_matrix(key))

def cbox(u0, u1, v0, v1, w0, w1):
    return box((u0, v0, w0), (u1, v1, w1))

def ucyl(u0, u1, v, w, r):
    """cylinder along u through (v, w)"""
    return Part.makeCylinder(r, u1 - u0, V(u0, v, w), V(1, 0, 0))

# ------------------------------------------------------------------------------------------------ pad specifications
# s = strap side (sign of canonical v) where the bottom FPC folds over the edge into the side curtain.
# u0/u1 = recess ends (walls 1.0 to the flat-top ends measured on the pre-tactile bones), cu = curtain u range.
# spine: list of (u_from, u_to, n_lines) dorsal spine segments (phalanges only); n_lines sets the channel width.
# metacarpal / distal recess ends are held 0.3 inside the flat top so the recess corner keeps >= 1.06 mm to the flared
# flexor-channel entries of mod_actuation (cones r 2.0 -> 1.05 at x 14.9 / j2 - 7.4 / 21.6).  The thumb distal wraps to
# the same side as the finger distals: its other side carries the M1.6 anchor nut slot (open to that face).
PHALANX = {
    #  key                s     u0     u1    flat-top ends
    'metacarpal':        dict(s=-1, u0=16.5, u1=30.5, top=(15.2, 31.8)),
    'metacarpal_pinky':  dict(s=+1, u0=16.5, u1=23.5, top=(15.2, 24.8)),
    'thumb_metacarpal':  dict(s=+1, u0=16.5, u1=30.5, top=(15.2, 31.8)),
    'proximal':          dict(s=-1, u0=16.5, u1=40.2, top=(15.5, 41.2)),
    'proximal_middle':   dict(s=-1, u0=16.5, u1=48.2, top=(15.5, 49.2)),
    'proximal_pinky':    dict(s=+1, u0=16.5, u1=30.2, top=(15.5, 31.2)),
    'thumb_proximal':    dict(s=+1, u0=16.5, u1=40.2, top=(15.5, 41.2)),
    'distal':            dict(s=+1, u0=0.8, u1=20.0, top=(0.0, 21.3), tip=(7.5, 7.5)),
    'thumb_distal':      dict(s=+1, u0=0.8, u1=20.0, top=(0.0, 21.3), tip=(7.5, 7.5)),
}
PALM = {
    'palm_index':  dict(s=-1, u0=8.5, u1=79.0, bed=(7.5, 80.0), cover='cover3'),
    'palm_middle': dict(s=-1, u0=8.5, u1=79.0, bed=(7.5, 80.0)),
    'palm_ring':   dict(s=-1, u0=8.5, u1=79.0, bed=(7.5, 80.0)),
    'palm_pinky':  dict(s=+1, u0=8.5, u1=79.0, bed=(7.5, 80.0), cover='cover2'),
}
BED_W0 = 2.7          # pad bed only above w 2.7 (clear of the J0 tongue slot, w <= 2.5 at x > 79.4)
NOTCH = (51.5, 62.5)  # V5 trough-3 notch left open by the seat fill: filled from the TPU layer (w -0.5) up
SLIVERS = [(11.5, 20.0), (31.5, 40.0)]   # thin trough-1/2 slivers beside the seat fill (|v| 4.6..6.5): filled above w -0.5
TPU_TOP_W = -0.5      # palm-side face of the TPU insert layer (bone y 0.5)
CURTAIN_W0 = 0.1      # palm curtain stops 0.6 above the TPU web slot (w -0.5): a 3-layer PLA lip
# strap screws (world XY) under the palm pad: pad holes r 2.3 (head dia 3.8 + key access), taxels kept 0.5 off
STRAP_SCREWS = {'palm_index': [(-91.7951, 1.4878), (-114.15, 1.10), (-133.20, 1.41)],                  # V5 BRK_I1..I3
                'palm_pinky': [(-100.97, -71.16), (-120.43, -65.81), (-139.12, -59.80)]}   # BRK_P1..P3 (no V6 screw at P4:
                                                                          # hardware open item; the ray tail crosses that strap)
SCREW_HOLE_R = 2.3

# finger rays: which part keys and which occurrences
RAYS = {
    'index':  dict(meta=('metacarpal', 'Metacarpal Bone_V02 <1>'), prox=('proximal', 'Proximal Phalanx Bone_V02 <1>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <1>'), base='Base Bone 1_V02 <1>',
                   palm=('palm_index', 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>'), j2=39.5, j3=49.2),
    'middle': dict(meta=('metacarpal', 'Metacarpal Bone_V02 <2>'), prox=('proximal_middle', 'Proximal Phalanx Bone_V02_middlefinger <1>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <2>'), base='Base Bone 1_V02 <2>',
                   palm=('palm_middle', 'Base Bone 1.2_V02_middlefinger <1>'), j2=39.5, j3=57.2),
    'ring':   dict(meta=('metacarpal', 'Metacarpal Bone_V02 <3>'), prox=('proximal', 'Proximal Phalanx Bone_V02 <2>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <3>'), base='Base Bone 1_V02 <4>',
                   palm=('palm_ring', 'Base Bone 1.2_V02_ringfing <1>'), j2=39.5, j3=49.2),
    'pinky':  dict(meta=('metacarpal_pinky', 'Metacarpal Bone_V02_pinky <1>'), prox=('proximal_pinky', 'Proximal Phalanx Bone_V02_pinky <1>'),
                   dist=('distal', 'Distal Phalanx Bone_V02 <4>'), base='Base Bone 1_V02 <3>',
                   palm=('palm_pinky', 'Base Bone 1.2_V02_pinky <1>'), j2=32.5, j3=39.2),
    'thumb':  dict(meta=('thumb_metacarpal', 'Metacarpal Bone_V02 <4>'), prox=('thumb_proximal', 'Proximal Phalanx Bone_V02 <3>'),
                   dist=('thumb_distal', 'Distal Phalanx Bone_V02 <5>'), base=None, palm=None, j2=39.5, j3=49.2),
}

# ------------------------------------------------------------------------------------------------ taxel grids
def _centered(a, b, n_max=None):
    """electrode centres at PITCH inside [a, b] (already inset), centred"""
    n = int(math.floor((b - a) / PITCH + 1e-9)) + 1
    if n_max:
        n = min(n, n_max)
    mid = 0.5 * (a + b)
    return [mid + (i - 0.5 * (n - 1)) * PITCH for i in range(n)]

def row_positions(s):
    """v of the 6 along-u (row) electrodes: outer edge 0.5 inside the non-strap pad edge (v = 6.5), strap-side edge
    0.4 before the fold start (v = 5.9)"""
    vs = [5.25 - 2.0 * k for k in range(6)]
    return [-s * v for v in vs]

def pad_outline_u(key):
    if key in PHALANX:
        p = PHALANX[key]
    else:
        p = PALM[key]
    return p['u0'] + C, p['u1'] - C

def col_positions(key):
    a, b = pad_outline_u(key)
    inset = ELEC / 2 + SEAL + 0.05
    if key in PALM:
        return _centered(a + inset, b - inset, 32)
    if 'tip' in PHALANX[key]:
        # fingertip: from the J3-end wall toward the tip at PITCH
        us, u = [], b - inset
        while u > a + 0.5:
            us.append(u)
            u -= PITCH
        return sorted(us)
    return _centered(a + inset, b - inset)

def _inside_pad(key, u, v):
    """electrode crossing square (ELEC x ELEC) fully inside the pad outline with SEAL margin (canonical).
    Rows (v) and columns (u) are laid out to fit the rectangular part; only the D-shaped fingertip needs a test."""
    p = PHALANX.get(key)
    if p and 'tip' in p:
        cu, r = p['tip']
        rp = r - SIDE_WALL - C                      # pad tip radius
        if u < cu:
            du = abs(u - cu) + ELEC / 2
            dv = abs(v) + ELEC / 2
            if math.hypot(du, dv) > rp - SEAL + 1e-9:
                return False
    return True

def taxels(key):
    """list of dicts: u, v (canonical), row index (0..5 across), col index (along), flags"""
    out = []
    rows = row_positions((PHALANX.get(key) or PALM.get(key))['s'])
    cols = col_positions(key)
    holes = [world_to_canon_xy(key, xy) for xy in STRAP_SCREWS.get(key, [])]
    knots = []
    if key in PHALANX and 'tip' not in PHALANX[key]:
        knots = [(19.0, 0.0)]                                   # actuation knot pocket (dia 3 from the palm face)
    for j, u in enumerate(cols):
        for i, v in enumerate(rows):
            if not _inside_pad(key, u, v):
                continue
            flags = []
            skip = False
            for (hu, hv) in holes:
                du = max(abs(u - hu) - ELEC / 2, 0); dv = max(abs(v - hv) - ELEC / 2, 0)
                if math.hypot(du, dv) < SCREW_HOLE_R + SEAL:
                    skip = True
            if skip:
                continue
            for (ku, kv) in knots:
                du = max(abs(u - ku) - ELEC / 2, 0); dv = max(abs(v - kv) - ELEC / 2, 0)
                if math.hypot(du, dv) < 1.5:
                    flags.append('over knot pocket (bridged, reduced backing)')
            out.append(dict(u=round(u, 3), v=round(v, 3), row=i, col=j, flags=flags))
    return out

_PALM_INST = {'palm_index': 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>',
              'palm_middle': 'Base Bone 1.2_V02_middlefinger <1>', 'palm_ring': 'Base Bone 1.2_V02_ringfing <1>',
              'palm_pinky': 'Base Bone 1.2_V02_pinky <1>'}

def world_to_canon_xy(key, xy):
    """a world (x, y) on the palm face of a palm bone -> canonical (u, v)"""
    pl = occ(_PALM_INST[key])
    p = pl.inverse().multVec(V(xy[0], xy[1], 7.0))
    # local (x, y, z) = (u, -w, v)
    return (p.x, p.z)

# ------------------------------------------------------------------------------------------------ phalanx geometry
def _mirror_v(shape, s):
    """build for strap side s = -1; mirror across v = 0 for s = +1"""
    if s < 0:
        return shape
    return shape.mirror(V(0, 0, 0), V(0, 1, 0))

def _dshape_face(u0, u1, cu, r, v_lo, v_hi, w):
    """planar 'D' region in the plane w: u in [u0, u1], v in [v_lo, v_hi], rounded tip = disc r about (cu, 0)"""
    rect = cbox(max(u0, cu), u1, v_lo, v_hi, w, w + 1.0)
    disc = Part.makeCylinder(r, 1.0, V(cu, 0, w), V(0, 0, 1))
    lim = cbox(u0, cu + 0.001, v_lo, v_hi, w, w + 1.0)
    return rect.fuse(disc.common(lim)).removeSplitter()

def phalanx_recess(key):
    """cut tool (canonical, built for s=-1 then mirrored): palm-face recess + strap-edge fold + side curtain +
    back-corner + back band over the curtain range"""
    p = PHALANX[key]
    s = p['s']
    u0, u1 = p['u0'], p['u1']
    parts = []
    if 'tip' in p:
        cu, r = p['tip']
        top = _dshape_face(u0, u1, cu, r - SIDE_WALL, -(FACE + EPS), FACE - SIDE_WALL, FLOOR)
        top = top.common(cbox(u0 - 1, u1 + 1, -20, 20, FLOOR, FACE + EPS))
        # open the strap side along the straight part
        top = top.fuse(cbox(cu, u1, -(FACE + EPS), -(FACE - SIDE_WALL - 1.0), FLOOR, FACE + EPS))
        cu0 = cu + 0.5
    else:
        top = cbox(u0, u1, -(FACE + EPS), FACE - SIDE_WALL, FLOOR, FACE + EPS)
        cu0 = u0
    parts.append(top)
    fu0, fu1 = cu0, u1
    # fold: corner square minus the R 1.0 rounding of the floor corner
    fold = cbox(fu0, fu1, -(FACE + EPS), -(FLOOR - RF), FLOOR - RF, FACE + EPS)
    fold = fold.cut(ucyl(fu0 - 1, fu1 + 1, -(FLOOR - RF), FLOOR - RF, RF))
    parts.append(fold)
    # side curtain: v in [-7.5-eps, -6.9], w from the back-corner start (-1.5) up to the fold
    parts.append(cbox(fu0, fu1, -(FACE + EPS), -FLOOR, -1.5, FLOOR - RF + 0.01))
    # back corner annulus R 6 -> 5.4 about (v -1.5, w -1.5), quadrant v <= -1.5, w <= -1.5
    ann = ucyl(fu0, fu1, -1.5, -1.5, 6.0 + EPS).cut(ucyl(fu0 - 1, fu1 + 1, -1.5, -1.5, 6.0 - CH))
    parts.append(ann.common(cbox(fu0, fu1, -(FACE + 1), -1.5, -(FACE + 1), -1.5)))
    t = parts[0].multiFuse(parts[1:]).removeSplitter()
    return _mirror_v(t, s)

def back_channel(u0, u1, width):
    """dorsal spine channel on the D-back (flat |v| <= 1.5 at w -7.5, R6 corners about (+-1.5, -1.5)): CH deep,
    width + 2*0.1 wide (0.1 clearance per side)"""
    h = width / 2 + 0.1
    parts = [cbox(u0, u1, -min(h, 1.5), min(h, 1.5), -(FACE + EPS), -(FACE - CH))]
    if h > 1.5:
        for sv in (-1, 1):
            ann = ucyl(u0, u1, sv * 1.5, -1.5, 6.0 + EPS).cut(ucyl(u0 - 1, u1 + 1, sv * 1.5, -1.5, 6.0 - CH))
            lim = cbox(u0, u1, min(sv * 1.5, sv * h), max(sv * 1.5, sv * h), -(FACE + 1), -1.5)
            parts.append(ann.common(lim))
    return parts[0].multiFuse(parts[1:]).removeSplitter() if len(parts) > 1 else parts[0]

def back_sheet(u0, u1, width, t=FPC2):
    """FPC lying on the spine channel floor (thickness t outward from the floor), |v| <= width/2"""
    h = width / 2
    fl = FACE - CH                     # floor at w = -6.9
    parts = [cbox(u0, u1, -min(h, 1.5), min(h, 1.5), -(fl + t), -fl)]
    if h > 1.5:
        rin = 6.0 - CH
        for sv in (-1, 1):
            ann = ucyl(u0, u1, sv * 1.5, -1.5, rin + t).cut(ucyl(u0 - 1, u1 + 1, sv * 1.5, -1.5, rin))
            lim = cbox(u0, u1, min(sv * 1.5, sv * h), max(sv * 1.5, sv * h), -(FACE + 1), -1.5)
            parts.append(ann.common(lim))
    return parts[0].multiFuse(parts[1:]).removeSplitter() if len(parts) > 1 else parts[0]

def phalanx_pad(key):
    """pad part (canonical): laminated slab (STACK) on the recess floor + the bottom-FPC wrap (FPC2): fold, side
    curtain, back corner and the back band up to the spine centre"""
    p = PHALANX[key]
    s = p['s']
    u0, u1 = p['u0'] + C, p['u1'] - C
    v_fold = -(FLOOR - RF)            # -5.9: slab ends where the fold begins
    v_out = FACE - SIDE_WALL - C      # 6.5
    if 'tip' in p:
        cu, r = p['tip']
        slab = _dshape_face(u0, u1, cu, r - SIDE_WALL - C, v_fold, v_out, FLOOR)
        slab = slab.common(cbox(u0 - 1, u1 + 1, -20, 20, FLOOR, FLOOR + STACK))
        cu0 = cu + 0.5 + C
    else:
        slab = cbox(u0, u1, v_fold, v_out, FLOOR, FLOOR + STACK)
        cu0 = u0
    fu0, fu1 = cu0, u1
    parts = [slab]
    fold = ucyl(fu0, fu1, v_fold, FLOOR - RF, RF + FPC2).cut(ucyl(fu0 - 1, fu1 + 1, v_fold, FLOOR - RF, RF))
    parts.append(fold.common(cbox(fu0, fu1, -(FACE + 1), v_fold, FLOOR - RF, FACE + 1)))
    parts.append(cbox(fu0, fu1, -(FLOOR + FPC2), -FLOOR, -1.5, FLOOR - RF))
    # back corner + centre band: only where the spine sheet runs (beyond the loop anchors / the knuckle exit the FPC
    # continues as the loop or ribbon, modelled as separate parts; the traces of the end columns turn along the side
    # face to reach the corner inside this range)
    _, (sa, sb) = spine_ranges(key)
    ba, bb = max(fu0, sa), min(fu1, sb)
    if bb - ba > 0.1:
        rin = 6.0 - CH
        ann = ucyl(ba, bb, -1.5, -1.5, rin + FPC2).cut(ucyl(ba - 1, bb + 1, -1.5, -1.5, rin))
        parts.append(ann.common(cbox(ba, bb, -(FACE + 1), -1.5, -(FACE + 1), -1.5)))
        parts.append(back_sheet(ba, bb, 3.0))
    t = parts[0].multiFuse(parts[1:]).removeSplitter()
    return _mirror_v(t, s)

# ------------------------------------------------------------------------------------------------ dorsal spines
# per bone: own joint positions (canonical u); the spine channel runs out through the knuckle end faces, the FPC sheet
# stops at the loop anchors (see loop_design; values for LOOP_EXCESS 18.7: J3 anchors j+3.29 / j+12.21, J2 anchors
# j-12.06 / j-3.14) and at the knuckle-loop exit (metacarpal u 17.5).
BONE_J = {'distal': dict(j3=29.3), 'thumb_distal': dict(j3=29.3),
          'proximal': dict(j2=7.5, j3=49.2), 'proximal_middle': dict(j2=7.5, j3=57.2),
          'proximal_pinky': dict(j2=7.5, j3=39.2), 'thumb_proximal': dict(j2=7.5, j3=49.2),
          'metacarpal': dict(j1=7.5, j2=39.5), 'metacarpal_pinky': dict(j1=7.5, j2=32.5),
          'thumb_metacarpal': dict(j1=7.5, j2=39.5)}
# columns carried by each spine (bottom FPC; rows = 6 on the top FPC); shared parts sized for the widest user
SPINE_N = {'distal': 9, 'thumb_distal': 9, 'proximal': 20, 'proximal_middle': 24, 'proximal_pinky': 15,
           'thumb_proximal': 20, 'metacarpal': 30, 'metacarpal_pinky': 18, 'thumb_metacarpal': 26}
KNUCKLE_EXIT = 17.5

def _anchors():
    d3 = loop_design('J3', 0.0); d2 = loop_design('J2', 0.0)
    return d3['pa'][0], d3['pb'][0], d2['pa'][0], d2['pb'][0]

def spine_ranges(key):
    """(channel u0, u1), (sheet u0, u1) on the back of phalanx `key`"""
    a3, b3, a2, b2 = _anchors()
    J = BONE_J[key]
    if 'j3' in J and 'j2' not in J:                    # distal: tip-side of J3, u runs toward J3
        j3 = J['j3']
        return (8.0, j3 - 8.0 + 0.5), (8.0, j3 - b3)
    if 'j3' in J:                                       # proximal: tip-side of J2, palm-side of J3
        return (-0.5, J['j3'] + 8.0), (J['j2'] + b2, J['j3'] + a3)
    return (J['j1'] + 7.2, J['j2'] - 7.2), (KNUCKLE_EXIT, J['j2'] + a2)   # metacarpal

def spine_width(key):
    return tail_width(SPINE_N[key])

# ------------------------------------------------------------------------------------------------ palm geometry
def rounded_envelope(u0, u1):
    """V5 palm-bone section (canonical): 15 x 15 with R6 corners centred at (+-1.5, +-1.5)"""
    s = cbox(u0, u1, -7.5, 7.5, -1.5, 1.5).fuse(cbox(u0, u1, -1.5, 1.5, -7.5, 7.5))
    for sv in (-1, 1):
        for sw in (-1, 1):
            s = s.fuse(ucyl(u0, u1, sv * 1.5, sw * 1.5, 6.0))
    return s.removeSplitter()

_STRAPS = {}
def strap_tools(key):
    """footprints (+C) of the cover3 / cover2 bracket straps lying over the palm face of palm_index / palm_pinky,
    canonical frame, extruded w 2.0 .. 8.6.  From the V5 geometry (same source and method as mod_fixes)."""
    if key in _STRAPS:
        return _STRAPS[key]
    cov = PALM[key].get('cover')
    if not cov:
        _STRAPS[key] = []
        return []
    fname = {'cover3': 'driver_side_palm_cover3.step', 'cover2': 'driver_side_palm_cover2.step'}[cov]
    occn = {'cover3': 'driver_side_palm_cover3 <1>', 'cover2': 'driver_side_palm_cover2 <1>'}[cov]
    s = Part.read(EXP + fname)
    s.Placement = occ(occn).multiply(s.Placement)
    # to the bone's canonical frame
    Mc = canon_matrix(key)
    Mb = occ(_PALM_INST[key]).toMatrix().multiply(Mc)          # canonical -> world
    s.transformShape(Mb.inverse())
    region = s.common(cbox(-2, 98, -7.7, 7.7, 2.0, 8.6))
    tools = []
    for sol in region.Solids:
        if sol.Volume < 1.0:
            continue
        bb = sol.BoundBox
        faces = []
        for w in [bb.ZMin + 0.05 + k * (bb.ZMax - bb.ZMin - 0.1) / 5 for k in range(6)]:
            for wr in sol.slice(V(0, 0, 1), w):
                f = Part.Face(wr)
                if f.Area > 0.3:
                    f.translate(V(0, 0, -w))
                    faces.append(f)
        if not faces:
            continue
        foot = faces[0]
        for f in faces[1:]:
            foot = foot.fuse(f)
        foot = foot.removeSplitter()
        outer = [Part.Face(ff.OuterWire) for ff in foot.Faces]
        foot = outer[0]
        for f in outer[1:]:
            foot = foot.fuse(f)
        foot = foot.removeSplitter()
        off = foot.makeOffset2D(C, 0, False, False, False)
        off.translate(V(0, 0, 2.0))
        tools.append(off.extrude(V(0, 0, 6.6)))
    _STRAPS[key] = tools
    return tools

def palm_bed(key):
    """material added (canonical), all above the TPU layer (w >= -0.5) and minus the strap keep-outs:
    squared section corners above w BED_W0 (pad bed), the open trough-3 notch, the trough-1/2 side slivers"""
    p = PALM[key]
    b0, b1 = p['bed']
    corners = cbox(b0, b1, -7.5, 7.5, BED_W0, FACE).cut(rounded_envelope(b0 - 1, b1 + 1))
    fills = [corners]
    n0, n1 = NOTCH
    notch = rounded_envelope(n0, n1).common(cbox(n0, n1, -8, 8, TPU_TOP_W, BED_W0 + 0.01))
    fills.append(notch.fuse(cbox(n0, n1, -7.5, 7.5, BED_W0, FACE)))
    for (a, b) in SLIVERS:
        for sv in (-1, 1):
            lim = cbox(a, b, min(sv * 4.0, sv * 7.6), max(sv * 4.0, sv * 7.6), TPU_TOP_W, FACE)
            fills.append(rounded_envelope(a, b).common(lim))
    add = fills[0].multiFuse(fills[1:]).removeSplitter()
    st = strap_tools(key)
    if st:
        add = add.cut(st[0].multiFuse(st[1:]) if len(st) > 1 else st[0])
    return add

def palm_recess_top(key):
    """palm-face recess (canonical) incl. the ray-tail channel over the wrist end: everything above the floor w 6.9
    (used to lower the cover2/cover3 bracket-strap tops that lie in these areas)"""
    p = PALM[key]
    t = cbox(p['u0'], p['u1'], -(FACE + EPS), FACE - SIDE_WALL, FLOOR, FACE + EPS)
    hw = tail_width(TAIL_N) / 2 + 0.1
    t = t.fuse(cbox(TAIL_CH[0], TAIL_CH[1] + 0.5, TAIL_V0 - hw, TAIL_V0 + hw, FLOOR, FACE + EPS))
    return _mirror_v(t.removeSplitter(), p['s'])

def palm_recess(key):
    """top recess + strap-edge fold + side curtain down to CURTAIN_W0 (canonical)"""
    p = PALM[key]
    u0, u1 = p['u0'], p['u1']
    parts = [cbox(u0, u1, -(FACE + EPS), FACE - SIDE_WALL, FLOOR, FACE + EPS)]
    fold = cbox(u0, u1, -(FACE + EPS), -(FLOOR - RF), FLOOR - RF, FACE + EPS)
    fold = fold.cut(ucyl(u0 - 1, u1 + 1, -(FLOOR - RF), FLOOR - RF, RF))
    parts.append(fold)
    parts.append(cbox(u0, WING_U0, -(FACE + EPS), -FLOOR, CURTAIN_W0, FLOOR - RF + 0.01))
    # ray-tail channel from the pad's wrist end over the rounded end of the bone (tail 7.4 + 0.2)
    hw = tail_width(TAIL_N) / 2 + 0.1
    parts.append(cbox(TAIL_CH[0], TAIL_CH[1] + 0.5, TAIL_V0 - hw, TAIL_V0 + hw, FLOOR, FACE + EPS))
    t = parts[0].multiFuse(parts[1:]).removeSplitter()
    return _mirror_v(t, p['s'])

def palm_pad(key):
    """palm pad part (canonical): laminated slab with strap-screw holes + fold + side curtain (FPC2)"""
    p = PALM[key]
    u0, u1 = p['u0'] + C, p['u1'] - C
    v_fold = -(FLOOR - RF)
    s = p['s']
    slab = cbox(u0, u1, v_fold, FACE - SIDE_WALL - C, FLOOR, FLOOR + STACK)
    fold = ucyl(u0, u1, v_fold, FLOOR - RF, RF + FPC2).cut(ucyl(u0 - 1, u1 + 1, v_fold, FLOOR - RF, RF))
    fold = fold.common(cbox(u0, u1, -(FACE + 1), v_fold, FLOOR - RF, FACE + 1))
    side = cbox(u0, CURTAIN_U1, -(FLOOR + FPC2), -FLOOR, CURTAIN_W0 + 0.1, FLOOR - RF)
    t = slab.fuse([fold, side]).removeSplitter()
    t = _mirror_v(t, s)
    holes = []
    for xy in STRAP_SCREWS.get(key, []):
        hu, hv = world_to_canon_xy(key, xy)
        holes.append(Part.makeCylinder(SCREW_HOLE_R, 3.0, V(hu, hv, FLOOR - 1.0), V(0, 0, 1)))
    if holes:
        t = t.cut(holes)
    return t

# ------------------------------------------------------------------------------------------------ module interface
_CACHE = {}

def _cut(shape, tools, key):
    tool = tools[0].multiFuse(tools[1:]) if len(tools) > 1 else tools[0]
    res = shape.cut(tool)
    return _single(res, key)

def _single(res, key):
    res = res.removeSplitter()
    if len(res.Solids) > 1:
        sols = sorted(res.Solids, key=lambda x: -x.Volume)
        if sols[1].Volume > 1.0:
            print('[tactile] WARNING %s: %d solids after the cut (2nd %.2f mm3)' % (key, len(sols), sols[1].Volume))
        res = sols[0]
    elif len(res.Solids) == 1:
        res = res.Solids[0]
    if not res.isValid():
        res.fix(1e-7, 1e-7, 1e-7)
    return res

def phalanx_tools_local(key):
    t = [phalanx_recess(key)]
    (a, b), _ = spine_ranges(key)
    t.append(back_channel(a, b, spine_width(key)))
    return [to_local(key, x) for x in t]

def palm_tools_local(key):
    return to_local(key, palm_bed(key)), to_local(key, palm_recess(key))

def cover_recess_local(cover):
    """the palm-pad top recess of the bone(s) this cover straps, in the cover's local frame"""
    occn = {'cover3': 'driver_side_palm_cover3 <1>', 'cover2': 'driver_side_palm_cover2 <1>'}[cover]
    tools = []
    for key, p in PALM.items():
        if p.get('cover') != cover:
            continue
        t = to_local(key, palm_recess_top(key))
        t = xform(t, occ(_PALM_INST[key]))
        t = xform(t, occ(occn).inverse())
        tools.append(t)
    return tools

def modify(key, shape):
    if key in PHALANX:
        return _cut(shape, phalanx_tools_local(key), key)
    if key in PALM:
        add, cut = palm_tools_local(key)
        s = shape.fuse(add)
        s = _single(s, key)
        return _cut(s, [cut], key)
    if key in ('cover3', 'cover2'):
        tools = cover_recess_local(key)
        if tools:
            return _cut(shape, tools, key)
    return shape

def pad_part(key):
    if key in PHALANX:
        t = [phalanx_pad(key)]
        _, (a, b) = spine_ranges(key)
        t.append(back_sheet(a, b, spine_width(key) - 0.0))
        s = t[0].multiFuse(t[1:]).removeSplitter()
    else:
        s = palm_pad(key)
    s = to_local(key, s)
    return _single(s, 'pad_' + key)

# ================================================================================================ tails
# All FPC tails are modelled as strips of both FPC layers (FPC2 thick) swept along a centre line.
R_MIN = 3.0          # minimum bend radius of any dynamically flexed tail (FPC 0.2 mm: >= 15x thickness)
LOOP_MARGIN = 3.0    # spare length at 90 deg so a loop is never pulled taut around a knuckle

def strip_solid(pts, wdirs, width, thick=FPC2):
    """ruled loft of rectangles (width along wdirs[i], thickness along t x w) through the centre-line points"""
    wires = []
    n = len(pts)
    for i in range(n):
        p = pts[i]
        t = (pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)])
        t.normalize()
        w = V(wdirs[i]); w = w - t * w.dot(t); w.normalize()
        nn = t.cross(w); nn.normalize()
        a, b = w * (width / 2), nn * (thick / 2)
        q = [p + a + b, p - a + b, p - a - b, p + a - b]
        wires.append(Part.makePolygon(q + [q[0]]))
    return Part.makeLoft(wires, True, True)

def _arc_pts(c, r, a0, a1, ccw, step=0.6):
    """points on a circle arc (2D tuples) from angle a0 to a1 (radians), direction ccw/cw"""
    if ccw:
        while a1 < a0:
            a1 += 2 * math.pi
    else:
        while a1 > a0:
            a1 -= 2 * math.pi
    n = max(2, int(abs(a1 - a0) * r / step) + 1)
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * k / (n - 1)), c[1] + r * math.sin(a0 + (a1 - a0) * k / (n - 1))) for k in range(n)]

def three_arc(pa, ha, pb, hb, rf, rl, side=-1):
    """fillet (cw, rf) at pa with heading ha -> loop circle (ccw, rl) -> fillet (cw, rf) into pb with heading hb.
    2D (s, n); returns (points, length) or None.  side picks the loop centre (-1: toward -n / away from the bone)."""
    right = lambda h: (math.sin(h), -math.cos(h))
    fa = (pa[0] + rf * right(ha)[0], pa[1] + rf * right(ha)[1])
    fb = (pb[0] + rf * right(hb)[0], pb[1] + rf * right(hb)[1])
    dx, dy = fb[0] - fa[0], fb[1] - fa[1]
    d = math.hypot(dx, dy)
    R = rf + rl
    if d > 2 * R or d < 1e-9:
        return None
    m = ((fa[0] + fb[0]) / 2, (fa[1] + fb[1]) / 2)
    h = math.sqrt(max(R * R - d * d / 4, 0))
    px, py = -dy / d, dx / d
    cands = [(m[0] + h * px, m[1] + h * py), (m[0] - h * px, m[1] - h * py)]
    c = min(cands, key=lambda q: side * -q[1]) if side else cands[0]
    ang = lambda p, o: math.atan2(p[1] - o[1], p[0] - o[0])
    pts = _arc_pts(fa, rf, ang(pa, fa), ang(c, fa), False)
    pts += _arc_pts(c, rl, ang(fa, c), ang(fb, c), True)[1:]
    pts += _arc_pts(fb, rf, ang(c, fb), ang(pb, fb), False)[1:]
    L = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    return pts, L

def loop_for_length(pa, ha, pb, hb, L, rf=R_MIN, side=-1):
    """3-arc curve of length L (bisection on the loop radius); returns (pts, rl) or (None, None)"""
    lo, hi = 0.3, 60.0
    best = None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        r = three_arc(pa, ha, pb, hb, rf, mid, side)
        if r is None:
            lo = mid
            continue
        if r[1] > L:
            hi = mid
        else:
            lo = mid
        best = (r[0], mid)
    if best is None:
        return None, None
    return best

def rot2(p, c, deg):
    a = math.radians(deg)
    x, y = p[0] - c[0], p[1] - c[1]
    return (c[0] + x * math.cos(a) - y * math.sin(a), c[1] + x * math.sin(a) + y * math.cos(a))

# ------------------------------------------------------------------------------------------------ ribbon turtle
class Ribbon:
    """3D ribbon path: tracks position p, heading T and width direction W (N = T x W).  bend() is out-of-plane
    (about W, radius >= R_MIN for moving sections), twist() rotates W about T while advancing, curve() is an in-plane
    turn (about N) that only static, pre-shaped sections may use (the FPC outline is cut curved)."""
    def __init__(self, p, T, W, step=0.5):
        self.pts = [V(p)]
        self.T = V(T).normalize()
        w = V(W); w = w - self.T * w.dot(self.T); self.W = w.normalize()
        self.ws = [V(self.W)]
        self.step = step
        self.log = []
        self.length = 0.0
    @property
    def p(self):
        return self.pts[-1]
    @property
    def N(self):
        return self.T.cross(self.W)
    def _add(self, p):
        self.length += (p - self.pts[-1]).Length
        self.pts.append(V(p)); self.ws.append(V(self.W))
    def straight(self, L):
        n = max(1, int(abs(L) / self.step))
        p0 = V(self.p)
        for k in range(1, n + 1):
            self._add(p0 + self.T * (L * k / n))
        self.log.append(('straight', L))
        return self
    def _rotate(self, axis, deg, center):
        r = App.Rotation(axis, deg)
        return r, (lambda q: center + r.multVec(q - center))
    def bend(self, R, deg, toward_n=True):
        """turn T toward +N (toward_n) or -N by deg on radius R, about the W axis"""
        sgn = 1 if toward_n else -1
        center = self.p + self.N * (sgn * R)
        axis = self.W * (-sgn)
        n = max(2, int(abs(math.radians(deg)) * R / self.step))
        p0, T0 = V(self.p), V(self.T)
        for k in range(1, n + 1):
            r = App.Rotation(axis, deg * k / n)
            q = center + r.multVec(p0 - center)
            self.T = r.multVec(T0)
            self._add(q)
        self.log.append(('bend', R, deg))
        return self
    def curve(self, R, deg, left=True):
        """in-plane turn about N (static sections only): left = toward +W"""
        sgn = 1 if left else -1
        center = self.p + self.W * (sgn * R)
        axis = self.N * sgn
        n = max(2, int(abs(math.radians(deg)) * R / self.step))
        p0, T0, W0 = V(self.p), V(self.T), V(self.W)
        for k in range(1, n + 1):
            r = App.Rotation(axis, deg * k / n)
            self.T = r.multVec(T0); self.W = r.multVec(W0)
            self._add(center + r.multVec(p0 - center))
        self.log.append(('curve', R, deg))
        return self
    def twist(self, deg, L):
        n = max(2, int(abs(L) / self.step))
        p0, W0 = V(self.p), V(self.W)
        for k in range(1, n + 1):
            self.W = App.Rotation(self.T, deg * k / n).multVec(W0)
            self._add(p0 + self.T * (L * k / n))
        self.log.append(('twist', deg, L))
        return self
    def to(self, q, n=None):
        """straight segment to point q (sets T)"""
        d = V(q) - self.p
        if d.Length < 1e-6:
            return self
        self.T = V(d).normalize()
        w = self.W - self.T * self.W.dot(self.T)
        if w.Length > 1e-6:
            self.W = w.normalize()
        return self.straight(d.Length)
    def solid(self, width, thick=FPC2):
        return strip_solid(self.pts, self.ws, width, thick)

# ------------------------------------------------------------------------------------------------ joint loops
# Joint plane (s, n): s along the finger toward the tip, n toward the palm, in the palm-side bone's frame.  The strip
# centre runs at n = NS in the dorsal channels.  All knuckles (J1/J2/J3 and the thumb's R5/R6) share the layout:
#   palm-side channel may carry the strip up to s = j + A_OFF, tip-side channel from s = j + B_OFF (minimal anchors,
#   ramps of R 3 from NS to -7.9 so the strip is dorsal of the facing knuckle before the channel ends);
#   the 90-deg wrap corner is the tongue's dorsal end corner at (j + 7.5, -7.5).
NS = -(FACE - CH) - FPC2 / 2          # -7.1
ANCHORS = {'J3': (5.8, 9.7), 'J2': (-9.7, -5.5), 'J1': (5.5, 9.7)}
LOOP_EXCESS = 15.7 + LOOP_MARGIN      # dorsal path growth at 90 deg (wrap of the tongue corner) + margin

def omega_alpha(excess, R=R_MIN):
    """contact angle of the symmetric omega (fillets and loop all radius R) storing `excess` over its chord"""
    lo, hi = 1e-4, math.pi / 2
    for _ in range(80):
        a = 0.5 * (lo + hi)
        e = 4 * R * (math.pi - a - math.sin(a))
        if e > excess:
            lo = a
        else:
            hi = a
    return 0.5 * (lo + hi)

def loop_design(kind, j):
    """(P_A, P_B, L, pts0, pts90, rl90) for a knuckle at s = j (2D joint plane)"""
    a_off, b_off = ANCHORS[kind]
    alpha = omega_alpha(LOOP_EXCESS)
    chord = 4 * R_MIN * math.sin(alpha)
    ext = max(0.0, (chord - (b_off - a_off)) / 2)
    pa = (j + a_off - ext, NS)
    pb = (j + b_off + ext, NS)
    r0 = three_arc(pa, 0.0, pb, 0.0, R_MIN, R_MIN, side=-1)
    pts0, L = r0
    pb90 = rot2(pb, (j, 0.0), 90.0)
    pts90, rl90 = loop_for_length(pa, 0.0, pb90, math.pi / 2, L, R_MIN, side=-1)
    return dict(pa=pa, pb=pb, L=L, pts0=pts0, pts90=pts90, rl90=rl90, pb90=pb90, chord=pb[0] - pa[0])

def loop_solid(kind, j, width, deg=0, frame='phalanx'):
    """loop strip in the palm-side bone's local frame (phalanx: s = x, n = z; base bone: s = -x, n = z)"""
    d = loop_design(kind, j)
    pts = d['pts0'] if deg == 0 else d['pts90']
    sx = -1.0 if frame == 'base' else 1.0
    P = [V(sx * q[0], 0.0, q[1]) for q in pts]
    return strip_solid(P, [V(0, 1, 0)] * len(P), width)

# ------------------------------------------------------------------------------------------------ spline ribbons
def spline_ribbon(way, t0=None, t1=None, step=0.5):
    """free tail: B-spline centre line through waypoints [(point, W)], width direction interpolated between the
    waypoint W's (projected perpendicular to the tangent).  Returns (pts, ws)."""
    P = [V(p) for p, _ in way]
    c = Part.BSplineCurve()
    if t0 is not None and t1 is not None:
        c.interpolate(Points=P, InitialTangent=V(t0), FinalTangent=V(t1))
    else:
        c.interpolate(P)
    L = c.length()
    n = max(4, int(L / step))
    ps = [c.value(c.parameterAtDistance(L * k / n)) for k in range(n + 1)]
    # waypoint parameters (arc length along the curve)
    sw = []
    for p, w in way:
        prm = c.parameter(V(p))
        sw.append((c.length(c.FirstParameter, prm), V(w).normalize()))
    ws = []
    for k, q in enumerate(ps):
        s = L * k / n
        i = 0
        while i < len(sw) - 2 and sw[i + 1][0] < s:
            i += 1
        (s0, w0), (s1, w1) = sw[i], sw[i + 1]
        f = 0.0 if s1 - s0 < 1e-9 else min(max((s - s0) / (s1 - s0), 0.0), 1.0)
        f = 0.5 - 0.5 * math.cos(math.pi * f)          # smooth
        w = w0 * (1 - f) + w1 * f
        ws.append(w)
    return ps, ws

def ribbon_metrics(pts, ws):
    """min out-of-plane bend radius, min in-plane radius, max twist (deg/mm), length"""
    import numpy as np
    P = np.array([[p.x, p.y, p.z] for p in pts]); W = np.array([[w.x, w.y, w.z] for w in ws])
    d = np.diff(P, axis=0); seg = np.linalg.norm(d, axis=1); T = d / seg[:, None]
    Tm = (T[1:] + T[:-1]); Tm /= np.linalg.norm(Tm, axis=1)[:, None]
    dT = (T[1:] - T[:-1]) / (0.5 * (seg[1:] + seg[:-1]))[:, None]
    Wm = W[1:-1] - (np.sum(W[1:-1] * Tm, axis=1))[:, None] * Tm
    Wm /= np.linalg.norm(Wm, axis=1)[:, None]
    Nm = np.cross(Tm, Wm)
    k_out = np.abs(np.sum(dT * Nm, axis=1)); k_in = np.abs(np.sum(dT * Wm, axis=1))
    # twist: rotation of W about T between samples
    tw = []
    for i in range(1, len(P) - 1):
        a = W[i] - np.dot(W[i], T[i - 1]) * T[i - 1]; b = W[i + 1] - np.dot(W[i + 1], T[i - 1]) * T[i - 1]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            continue
        cang = np.clip(np.dot(a, b) / (na * nb), -1, 1)
        tw.append(np.degrees(np.arccos(cang)) / seg[i])
    return dict(r_out=float(1.0 / max(k_out.max(), 1e-9)), r_in=float(1.0 / max(k_in.max(), 1e-9)),
                twist=float(max(tw) if tw else 0.0), length=float(seg.sum()))

# ------------------------------------------------------------------------------------------------ ray routes
LOOP_W = {'J3': {'index': 9, 'middle': 9, 'ring': 9, 'pinky': 9, 'thumb': 9},
          'J2': {'index': 20, 'middle': 24, 'ring': 20, 'pinky': 15, 'thumb': 20}}
RAY_N = {'index': 26, 'middle': 30, 'ring': 26, 'pinky': 18, 'thumb': 26}      # digit columns (knuckle loop)
TAIL_N = 32                                                                     # ray tail: 32 shared columns

def joint_loop_part(kind, n_cols, deg=0):
    """loop strip in its joint frame: origin at the joint axis, x toward the fingertip, z toward the palm, y = axis"""
    d = loop_design(kind, 0.0)
    pts = d['pts0'] if deg == 0 else d['pts90']
    P = [V(q[0], 0.0, q[1]) for q in pts]
    return strip_solid(P, [V(0, 1, 0)] * len(P), tail_width(n_cols))

def joint_frame(f, kind):
    """world placement of the joint frame of finger f's J3 or J2 (palm-side bone occurrence + offset along u)"""
    r = RAYS[f]
    if kind == 'J3':
        key, on = r['prox']
        j = BONE_J[key]['j3']
    else:
        key, on = r['meta']
        j = BONE_J[key]['j2']
    return occ(on).multiply(App.Placement(V(j, 0, 0), App.Rotation())), (key, on, j)

PALM_J1_U = 115.144 - 7.5          # J1 in the palm-bone canonical frame (base bone x 7.5 at J0 = 0)
FINE = 0.15                        # trace pitch (0.075/0.075 mm) from the knuckle loop onward (window is 6 mm)
LANE_V = 9.2                       # lane beside the magazine (|v| 7.9) below the TPU web, clear of the J0 ramps
LANE_W = -11.0                     # lane centre height (ribbon w -8.35..-13.65): under the base bone (-8.0) and the wings
RISE_U = 81.0                      # rise past the TPU web: slot between the wing (|v| <= 10.5, u 73.5..94.5) and
SLOT_V = 11.1                      #   the neighbour's motor-plug keep-outs (|v| >= 12.45, u >= 77.8, w <= -7.5)
CURT_V = 7.1                       # curtain FPC mid-plane (channel floor |v| 6.9, FPC 0.4)
SIDE_W = 3.0                       # digit ribbon centre height above the web (ribbon w 0.35..5.65 for 5.3 mm)
CURTAIN_U1 = 62.5                  # the digit ribbon becomes the curtain's knuckle-end continuation here (one FPC)
WING_U0 = 73.3                     # wing blend starts ~73.5: the ribbon only moves in toward the bone below this
TAIL_V0 = 0.3                      # ray tail centre on the palm pad's wrist end (canonical v, strap side -)
TAIL_CH = (-0.8, 8.5)              # ray-tail channel over the palm bone's rounded wrist end (u range, 0.6 deep)

def fine_width(n):
    return n * FINE + 2 * 0.4

def catmull(P, alpha=0.5, step=0.5):
    """centripetal Catmull-Rom through points (FreeCAD vectors); returns sampled points"""
    Q = [P[0] * 2 - P[1]] + list(P) + [P[-1] * 2 - P[-2]]
    out = []
    for i in range(1, len(Q) - 2):
        p0, p1, p2, p3 = Q[i - 1], Q[i], Q[i + 1], Q[i + 2]
        t0 = 0.0
        t1 = t0 + (p1 - p0).Length ** alpha
        t2 = t1 + (p2 - p1).Length ** alpha
        t3 = t2 + (p3 - p2).Length ** alpha
        m = max(4, int((p2 - p1).Length / step))
        for k in range(m):
            t = t1 + (t2 - t1) * k / m
            A1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0))
            A2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1))
            A3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2))
            B1 = A1 * ((t2 - t) / (t2 - t0)) + A2 * ((t - t0) / (t2 - t0))
            B2 = A2 * ((t3 - t) / (t3 - t1)) + A3 * ((t - t1) / (t3 - t1))
            out.append(B1 * ((t2 - t) / (t2 - t1)) + B2 * ((t - t1) / (t2 - t1)))
    out.append(V(P[-1]))
    return out

def rmf_twist(pts, w0, w1):
    """width directions along pts: rotation-minimising frame from w0, plus a smoothly distributed twist so the last
    one equals w1 (projected)"""
    n = len(pts)
    T = []
    for i in range(n):
        t = pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)]
        T.append(t.normalize())
    W = [V(w0) - T[0] * V(w0).dot(T[0])]
    W[0].normalize()
    for i in range(1, n):                        # double reflection (Wang et al. 2008)
        v1 = pts[i] - pts[i - 1]
        c1 = v1.dot(v1)
        if c1 < 1e-12:
            W.append(V(W[-1])); continue
        rL = W[-1] - v1 * (2.0 / c1 * v1.dot(W[-1]))
        tL = T[i - 1] - v1 * (2.0 / c1 * v1.dot(T[i - 1]))
        v2 = T[i] - tL
        c2 = v2.dot(v2)
        w = rL - v2 * (2.0 / c2 * v2.dot(rL)) if c2 > 1e-12 else rL
        W.append(w.normalize())
    we = V(w1) - T[-1] * V(w1).dot(T[-1]); we.normalize()
    ang = math.degrees(math.atan2(T[-1].dot(W[-1].cross(we)), W[-1].dot(we)))
    if ang > 90.0:                               # W and -W are the same ribbon: take the short way round
        ang -= 180.0
    elif ang < -90.0:
        ang += 180.0
    L = [0.0]
    for i in range(1, n):
        L.append(L[-1] + (pts[i] - pts[i - 1]).Length)
    out = []
    for i in range(n):
        f = L[i] / L[-1] if L[-1] > 0 else 1.0
        f = 0.5 - 0.5 * math.cos(math.pi * f)
        out.append(App.Rotation(T[i], ang * f).multVec(W[i]))
    return out

def free_span(p_start, t_start, w_start, way, p_end, t_end, w_end):
    """free tail between two frames: Catmull-Rom through [start, way..., end] (end tangents enforced by short
    lead-in / lead-out points) with an RMF + distributed twist"""
    P = [V(p_start), V(p_start) + V(t_start).normalize() * 1.5] + [V(q) for q in way] +         [V(p_end) - V(t_end).normalize() * 1.5, V(p_end)]
    pts = catmull(P)
    return pts, rmf_twist(pts, w_start, w_end)

def _canon_world(key):
    """canonical (u, v, w) of palm bone `key` -> world matrix"""
    return occ(_PALM_INST[key]).toMatrix().multiply(M_PALM)

def _mv(M, p):
    return M.multVec(V(p))

def _mr(M, d):
    return App.Placement(M).Rotation.multVec(V(d))

def _jog(rl, s, v_from, v_to, R):
    """out-of-plane S-bend (about W) moving the ribbon from |v| v_from to |v| v_to"""
    d = abs(v_from - v_to)
    if d < 1e-6:
        return rl
    ang = math.degrees(math.acos(1 - d / (2 * R)))
    goal = V(0, -s, 0) if v_to < v_from else V(0, s, 0)
    first = rl.N.dot(goal) > 0
    rl.bend(R, ang, toward_n=first)
    rl.bend(R, ang, toward_n=not first)
    return rl

def _slot_rise(rl, s, v_slot):
    """lane (heading -u, W vertical) -> jog out to the wing/plug slot -> in-plane 'Z' step up past the TPU web
    (vertical section at u RISE_U) -> heading -u above the web at SIDE_W, beside the wing -> to u 73"""
    rl.to(V(95.5, rl.p.y, rl.p.z))
    _jog(rl, s, abs(rl.p.y), v_slot, 8.0)
    if rl.p.x > RISE_U + 4.0:
        rl.to(V(RISE_U + 4.0, rl.p.y, rl.p.z))
    rl.curve(4.0, 90.0, left=rl.W.dot(V(0, 0, 1)) > 0)
    rl.straight((SIDE_W - rl.p.z) - 4.0)
    rl.curve(4.0, 90.0, left=rl.W.dot(V(-1, 0, 0)) > 0)
    rl.to(V(WING_U0, rl.p.y, rl.p.z))
    return rl

def knuckle_loop_path(f, world=False, segments=False):
    """digit ribbon from the metacarpal's dorsal exit to the palm bone's wrist end: exit bend R3 -> free service loop
    (J1 slack + 90 deg twist, clear of the flexor span; free span 51.5 mm (pinky 53.2) vs the taut R3 path at J1 90 deg
    combined with J0 15 deg: >= 3.9 mm to spare) -> lane beside the plug-free side of a magazine under the TPU
    web -> in-plane 'Z' step up through the web-end window -> jog into the palm pad's curtain channel, where it
    continues as the curtain (same FPC) at u CURTAIN_U1.
    Returns (pts, ws) in finger f's palm-bone canonical frame (or world when world=True).
    The pinky's own motor plugs sit on its +v side, so its ribbon shares the ring's lane and window (stacked 1.0 / 0.6 mm
    outboard of the ring ribbon) and crosses the ring/pinky gap above the web into the pinky's curtain channel."""
    key = RAYS[f]['palm'][0]
    s = PALM[key]['s']
    Mf = _canon_world(key)
    u_exit = PALM_J1_U + (KNUCKLE_EXIT - 7.5)          # 117.64
    rb = Ribbon(V(u_exit, 0, NS), V(-1, 0, 0), V(0, 1, 0))
    rb.bend(R_MIN, 90.0, toward_n=True)                  # dorsal
    p1, w1 = V(rb.p), V(rb.W)
    if f != 'pinky':
        lane0 = V(96.0, s * LANE_V, LANE_W)
        way = [p1, p1 + V(0, 0, -1.5), V(p1.x - 0.894, s * 0.618, -23.53), V(111.11, s * 4.41, -30.01),
               V(105.99, s * LANE_V, -27.83), V(102.18, s * LANE_V, -12.58), V(97.5, s * LANE_V, LANE_W), lane0]
        fp = catmull(way)
        fw = rmf_twist(fp, w1, V(0, 0, 1))
        pts, ws = rb.pts[:-1] + fp, rb.ws[:-1] + fw
        free = (len(rb.pts) - 1, len(rb.pts) - 1 + len(fp) - 1)     # free service loop (indices into pts)
        rl = Ribbon(lane0, V(-1, 0, 0), V(fw[-1]))
        _slot_rise(rl, s, SLOT_V)
        _jog(rl, s, SLOT_V, CURT_V, 8.0)
        rl.to(V(CURTAIN_U1, s * CURT_V, SIDE_W))
        pts, ws = pts + rl.pts[1:], ws + rl.ws[1:]
        if world:
            pts, ws = [_mv(Mf, q) for q in pts], [_mr(Mf, w) for w in ws]
        return (pts, ws, free) if segments else (pts, ws)
    # pinky: world-frame route through the ring lane
    Mr = _canon_world('palm_ring')
    Mri = Mr.inverse()
    PV = LANE_V + 1.0                                    # stacked outboard of the ring ribbon (ring frame v -10.2)
    ex = [_mv(Mf, q) for q in rb.pts]; exw = [_mr(Mf, w) for w in rb.ws]
    p1w = ex[-1]
    lane0 = _mv(Mr, V(96.0, -PV, LANE_W))
    pr = _mv(Mri, p1w)                                   # exit in the ring frame
    way_r = [pr, pr + _mr(Mri, _mr(Mf, V(0, 0, -1.5))), V(pr.x - 2.0, pr.y + 3.0, -23.5), V(110.0, -18.5, -30.5),
             V(104.5, -PV - 1.2, -27.0), V(101.0, -PV, -13.0), V(97.5, -PV, LANE_W), V(96.0, -PV, LANE_W)]
    fp = catmull([_mv(Mr, q) for q in way_r])
    fw = rmf_twist(fp, exw[-1], _mr(Mr, V(0, 0, 1)))
    pts, ws = ex[:-1] + fp, exw[:-1] + fw
    free = (len(ex) - 1, len(ex) - 1 + len(fp) - 1)
    # lane + slot rise in the ring frame (1.0 outboard of the ring ribbon in the lane, 0.6 in the slot: |v| 11.7)
    rl = Ribbon(V(96.0, -PV, LANE_W), V(-1, 0, 0), _mr(Mri, fw[-1]))
    _slot_rise(rl, -1, SLOT_V + 0.6)                     # stacked 0.2 mm outboard of the ring ribbon
    lp = [_mv(Mr, q) for q in rl.pts[1:]]; lw = [_mr(Mr, w) for w in rl.ws[1:]]
    # above the web: cross the ring/pinky gap (pinky frame v 12.3 -> 7.1) into the pinky's curtain plane
    Mfi = Mf.inverse()
    a = _mv(Mfi, lp[-1])
    rs = Ribbon(a, _mr(Mfi, _mr(Mr, rl.T)), _mr(Mfi, lw[-1]))
    rs.to(V(a.x - 0.2, a.y, SIDE_W))
    _jog(rs, +1, abs(a.y), CURT_V, 6.0)                  # static, pre-formed S (R 6): in the curtain plane by u 63.1
    rs.to(V(CURTAIN_U1, CURT_V, SIDE_W))
    sp = [_mv(Mf, q) for q in rs.pts[1:]]; sw = [_mr(Mf, w) for w in rs.ws[1:]]
    pts = pts + lp + sp
    ws = ws + lw + sw
    if not world:
        pts, ws = [_mv(Mfi, q) for q in pts], [_mr(Mfi, w) for w in ws]
    return (pts, ws, free) if segments else (pts, ws)

def wrist_tail_path(f):
    """ray tail (32 columns + 12 rows, 7.4 wide): leaves the palm pad's wrist end flat in a 0.6 mm channel over the
    bone's rounded end, ramps up over the TPU wrist plate, crosses the wrist segment at z 7.2..7.6 and runs to the
    readout pod on the arm (world frame).  No twist: the pad, channel and wrist crossing are all palm-side flat."""
    key, on = RAYS[f]['palm']
    s = PALM[key]['s']
    M = _canon_world(key)
    vc = -s * TAIL_V0
    # centre in the channel: floor w 6.9 -> strip w 6.9..7.3, centre 7.1
    rb = Ribbon(V(PALM[key]['u0'] + C, vc, FLOOR + FPC2 / 2), V(-1, 0, 0), V(0, 1, 0))
    rb.to(V(TAIL_CH[0] + 0.3, vc, FLOOR + FPC2 / 2))
    p0 = [_mv(M, q) for q in rb.pts]; w0 = [_mr(M, w) for w in rb.ws]
    e = p0[-1]
    d = _mr(M, V(-1, 0, 0)); d.z = 0; d.normalize()      # continue along the ray for 9 mm, then ease to world -x
    q9 = e + d * 9.0
    way = [e, e + d * 1.5, V(e.x + d.x * 4.5, e.y + d.y * 4.5, 7.3), V(q9.x, q9.y, 7.55),
           V(q9.x - 9.0, q9.y + d.y * 4.0, 7.6), V(-189.0, q9.y + d.y * 5.0, 7.6),
           V(-205.0, 0.5 * (q9.y + d.y * 5.0 + pod_entry(f, 8.0).y), 7.5),
           pod_entry(f, 8.0), pod_entry(f, -3.0)]
    fp = catmull(way)
    fw = rmf_twist(fp, w0[-1], V(0, 1, 0))
    return p0[:-1] + fp, w0[:-1] + fw


# ------------------------------------------------------------------------------------------------ thumb route
THUMB_RUN_Y, THUMB_RUN_Z = 39.5, -12.0  # corridor outside cover3 (y <= 34.8), the shells' bowl claim (y <= 35.2), the
                                        # thumb-side seam claim (top z -18.4) and the yaw carrier (>= 18.5 from R1)
THUMB_ANCHOR = V(-166.0, THUMB_RUN_Y, THUMB_RUN_Z)   # first standoff clip on cover3's thumb-side wall

def thumb_service_path(segments=False):
    """thumb metacarpal dorsal exit -> free service loop (bulges outward and down to z -35, 88 mm free) -> anchor
    clip on cover3's thumb-side wall -> corridor y 39.5 / z -12 to the wrist -> readout pod (world frame).
    Returns (pts, ws)."""
    on = RAYS['thumb']['meta'][1]
    pl = occ(on)
    R = pl.Rotation
    p0 = pl.multVec(V(KNUCKLE_EXIT, 0, NS))
    rb = Ribbon(p0, R.multVec(V(-1, 0, 0)), R.multVec(V(0, 1, 0)))
    rb.bend(R_MIN, 90.0, toward_n=True)
    p1 = V(rb.p)
    # service loop at the V6 rest (thumb flex stack re-clocked -62.4 deg about R3, mod_thumb.placements()): waypoints
    # from a keep-out-constrained search (work/tactile/scratch/th_search4.py): free span (exit-bend end -> anchor clip)
    # >= 80 mm for the R1/R2/R3/R4 extremes (worst reach 48.8 mm + U-turn + margin), out-of-plane bend R >= 3, clear of
    # the thumb-side seam claim, the shells' bowl claim, cover3, the yaw-carrier sweep and the metacarpal
    way1 = [p1, p1 + rb.T * 2.5, V(-155.546, 52.369, -27.251), V(-147.232, 55.974, -31.707),
            V(-152.904, 66.597, -23.211), V(-159.241, 58.908, -13.734), V(-162.614, 43.940, -8.304),
            V(THUMB_ANCHOR), V(-180.0, THUMB_RUN_Y, THUMB_RUN_Z)]
    fp1 = catmull(way1)
    fw1 = rmf_twist(fp1, rb.W, V(0, 0, 1))
    way2 = [V(-180.0, THUMB_RUN_Y, THUMB_RUN_Z), V(-195.0, THUMB_RUN_Y, THUMB_RUN_Z), V(-205.0, THUMB_RUN_Y - 4.0, -9.0),
            V(-213.0, 28.5, -4.0), V(-214.0, 21.0, 1.5), pod_entry('thumb', 12.0) + V(0, 6.0, -1.0),
            pod_entry('thumb', 3.0), pod_entry('thumb', -3.0)]
    fp2 = catmull(way2)
    fw2 = rmf_twist(fp2, fw1[-1], V(0, 1, 0))
    pts, ws = rb.pts[:-1] + fp1 + fp2[1:], rb.ws[:-1] + fw1 + fw2[1:]
    i0 = len(rb.pts) - 1
    ia = i0 + min(range(len(fp1)), key=lambda i: (fp1[i] - THUMB_ANCHOR).Length)
    return (pts, ws, (i0, ia)) if segments else (pts, ws)

# ------------------------------------------------------------------------------------------------ readout pod
# Default readout = 5 x FlexiTac "Reading Board 32x16" (Arduino Nano + 1 mux + 4 shift registers) + a 7-port USB 2.0
# hub, in a pod strapped on TOP (palm side, +z) of the arm link: the mirror of the electronics pod underneath (same
# 42 mm offset from the cover_back backplate, same R 22.5 arm).  Board size and connector layout are NOT published:
# ASSUMED 50 x 25 PCB, Nano on headers (15 mm total), 32-pin + 16-pin 0.5 mm ZIFs along one long edge (the tail edge),
# the Nano's mini-USB at one short end (16 mm plug allowance).
# Pod layout (pod frame: x toward the hand, origin = front face centre on the bottom plane, z up): two columns of
# three bays; every board's ZIF edge faces the side wall, where a 5 mm aisle carries the ray tails (on edge) from the
# front slot to the ZIFs; the USB plugs sit in the 16 mm gaps between bays; the hub is the last bay of the -y column
# and its upstream cable leaves through the rear wall.
ARM_AXIS_PT = V(-189.54, -24.58, -24.52)     # Dorna tool axis on the backplate outer face (mod_electronics.pod_matrix)
ARM_OUT = V(-0.99131, 0.13153, 0.0)          # arm direction (away from the hand)
ARM_R = 22.5
POD_OUT = 42.0
POD = dict(L=204.0, W=66.0, H=28.0, wall=2.0, floor=5.0, lid=2.0, saddle=3.0)
BOARD = (50.0, 25.0, 1.6)                   # reading board PCB (assumed)
NANO = (45.0, 18.0, 13.4)                   # Arduino Nano on headers: board + Nano = 15 mm (assumed envelope)
ZIF32, ZIF16 = (20.5, 3.5, 2.0), (12.5, 3.5, 2.0)   # 0.5 mm FFC ZIF connectors (length, depth, height; assumed)
HUB = (50.0, 25.0, 10.0)                    # 7-port USB 2.0 hub board (assumed; multi-TT preferred, see notes)
PLUG = 16.0                                 # mini-USB plug + strain relief allowance behind each Nano
AISLE = 5.0                                 # tail aisle between the ZIF edge and the side wall
BOARD_Z = POD['floor'] + 2.0                # boards on 2 mm floor rails (VHB tape)
COL_Y = POD['W'] / 2 - POD['wall'] - AISLE - BOARD[1] / 2          # 13.5: column centres at +-13.5
# bays: (name, front x of the board envelope, column sign, rotated 180 deg about z?)  the -y column is turned so its
# ZIF edge also faces its side wall (its USB end then faces the front: plug gap in front of the board)
_X1 = -POD['wall'] - 2.0
BAYS = [('thumb', _X1, +1, False), ('index', _X1 - BOARD[0] - PLUG, +1, False),
        ('middle', _X1 - 2 * (BOARD[0] + PLUG), +1, False),
        ('ring', _X1 - PLUG, -1, True), ('pinky', _X1 - 2 * PLUG - BOARD[0], -1, True),
        ('hub', _X1 - 3 * PLUG - 2 * BOARD[0], -1, True)]
# ray-tail entry positions across the front slot (pod-frame y), at the tails' crossing height (world z 7.4)
POD_ENTRY_Y = {'thumb': 22.0, 'index': 13.5, 'middle': 5.0, 'ring': -5.0, 'pinky': -13.5}
TAIL_Z_WORLD = 7.4
LID_LUGS = [(-12.0, -1), (-12.0, +1), (-POD['L'] + 12.0, -1), (-POD['L'] + 12.0, +1)]
LUG_R, LUG_OFF = 3.0, 1.5                   # external lid lugs: r 3 about y = +-(W/2 + 1.5), M2 heat-set insert

def pod_matrix():
    """pod frame: origin = front face (hand end) centre at the pod bottom plane, x toward the hand, z up"""
    out = V(ARM_OUT).normalize()
    front = ARM_AXIS_PT + out * POD_OUT
    o = V(front.x, front.y, ARM_AXIS_PT.z + (ARM_R + 0.5) - POD['saddle'])      # bottom plane; saddle cuts 3 up
    x = out * -1
    z = V(0, 0, 1)
    y = z.cross(x)
    return App.Matrix(x.x, y.x, z.x, o.x, x.y, y.y, z.y, o.y, x.z, y.z, z.z, o.z, 0, 0, 0, 1)

def pod_point(x, y, z):
    return pod_matrix().multVec(V(x, y, z))

def pod_entry(f, x):
    """world point of ray f's tail at pod-frame x (x > 0: in front of the pod) on its entry line"""
    zp = TAIL_Z_WORLD - pod_matrix().multVec(V(0, 0, 0)).z
    return pod_point(x, POD_ENTRY_Y[f], zp)

def _lug_xy(x, sy):
    return x, sy * (POD['W'] / 2 + LUG_OFF)

def pod_case():
    L, W, H, w, fl = POD['L'], POD['W'], POD['H'] - POD['lid'], POD['wall'], POD['floor']
    body = box((-L, -W / 2, 0), (0, W / 2, H))
    for (x, sy) in LID_LUGS:                                    # external lid lugs, full height, merged into the wall
        lx, ly = _lug_xy(x, sy)
        body = body.fuse(Part.makeCylinder(LUG_R, H, V(lx, ly, 0), V(0, 0, 1)))
    cav = box((-L + w, -W / 2 + w, fl), (-w, W / 2 - w, H + 1))
    s = body.cut(cav)
    # board rails (2 x 2 mm, 2 per bay) on the floor
    rails = []
    for (name, xf, sy, rot) in BAYS:
        ln = HUB[0] if name == 'hub' else BOARD[0]
        for dy in (-8.0, 8.0):
            rails.append(box((xf - ln + 3.0, sy * COL_Y + dy - 1.0, fl - 0.01), (xf - 3.0, sy * COL_Y + dy + 1.0, BOARD_Z)))
    s = s.fuse(rails)
    # saddle: arm cylinder + 0.5 clearance, axis 20.0 below the bottom plane
    s = s.cut(Part.makeCylinder(ARM_R + 0.5, L + 2, V(1, 0, -(ARM_R + 0.5 - POD['saddle'])), V(-1, 0, 0)))
    # front slot for the five ray tails (pod z 9.2..13.8 = world ~4.7..9.3)
    s = s.cut(box((-w - 1, -W / 2 + w + 0.5, 9.2), (1, W / 2 - w - 0.5, 13.8)))
    # rear exit for the hub's upstream USB cable
    hx = [b for b in BAYS if b[0] == 'hub'][0]
    s = s.cut(box((-L - 1, hx[2] * COL_Y - 6.0, BOARD_Z), (-L + w + 1, hx[2] * COL_Y + 6.0, BOARD_Z + 12.0)))
    # two strap tunnels through the floor (hook-and-loop straps around the arm, like the electronics pod)
    for x in (-50.0, -150.0):
        s = s.cut(box((x - 13, -W / 2 - LUG_R - 3, 0.8), (x + 13, W / 2 + LUG_R + 3, 3.0)))
    for (x, sy) in LID_LUGS:                                    # M2 heat-set insert holes dia 3.0 x 4
        lx, ly = _lug_xy(x, sy)
        s = s.cut(Part.makeCylinder(1.5, 4.0, V(lx, ly, H - 4.0), V(0, 0, 1)))
    return clean(s)

def pod_lid():
    L, W, H = POD['L'], POD['W'], POD['H']
    lid = box((-L, -W / 2, H - POD['lid']), (0, W / 2, H))
    for (x, sy) in LID_LUGS:
        lx, ly = _lug_xy(x, sy)
        lid = lid.fuse(Part.makeCylinder(LUG_R, POD['lid'], V(lx, ly, H - POD['lid']), V(0, 0, 1)))
    for (x, sy) in LID_LUGS:
        lx, ly = _lug_xy(x, sy)
        lid = lid.cut(Part.makeCylinder(1.2, 5.0, V(lx, ly, H - 3.0), V(0, 0, 1)))    # M2 clearance
    return clean(lid.removeSplitter())

def board_model():
    """reading board 32x16 (assumed): PCB + Nano on headers + 32/16-pin 0.5 mm FFC ZIFs along the +y long edge.
    Board frame: x toward the ZIF end (front edge at x = 0), +y = the ZIF (tail) edge, z up from the PCB bottom."""
    bx, by, bt = BOARD
    s = box((-bx, -by / 2, 0), (0, by / 2, bt))
    s = s.fuse(box((-bx + 2.5, -by / 2 + 1.0, bt), (-bx + 2.5 + NANO[0], -by / 2 + 1.0 + NANO[1], bt + NANO[2])))
    s = s.fuse(box((-3.0 - ZIF32[0], by / 2 - ZIF32[1], bt), (-3.0, by / 2, bt + ZIF32[2])))
    s = s.fuse(box((-5.0 - ZIF32[0] - ZIF16[0], by / 2 - ZIF16[1], bt), (-5.0 - ZIF32[0], by / 2, bt + ZIF16[2])))
    return clean(s.removeSplitter())

def hub_model():
    return box((-HUB[0], -HUB[1] / 2, 0), (0, HUB[1] / 2, HUB[2]))

def bay_placement(name):
    b = [x for x in BAYS if x[0] == name][0]
    _, xf, sy, rot = b
    ln = HUB[0] if name == 'hub' else BOARD[0]
    if not rot:
        return App.Placement(pod_matrix().multiply(App.Placement(V(xf, sy * COL_Y, BOARD_Z), App.Rotation()).toMatrix()))
    # rotated 180 deg about z: the board frame's front edge (x = 0) is at pod x = xf - ln
    return App.Placement(pod_matrix().multiply(App.Placement(V(xf - ln, sy * COL_Y, BOARD_Z), App.Rotation(V(0, 0, 1), 180)).toMatrix()))

# ================================================================================================ module interface
def _tail(pts, ws, n_cols, name):
    sol = strip_solid(pts, ws, tail_width(n_cols))
    return _single(sol, name)

_PARTS = None
def _build_parts():
    global _PARTS
    if _PARTS is not None:
        return _PARTS
    out = {}
    for key in list(PHALANX) + list(PALM):
        out['tactile_pad_' + key] = pad_part(key)
    for kind, widths in LOOP_W.items():
        for n in sorted(set(widths.values())):
            out['tactile_loop_%s_n%d' % (kind, n)] = _single(joint_loop_part(kind, n), 'loop')
    for f in ('index', 'middle', 'ring', 'pinky'):
        key = RAYS[f]['palm'][0]
        pts, ws = knuckle_loop_path(f)
        out['tactile_knuckle_' + f] = to_local(key, _single(strip_solid(pts, ws, fine_width(RAY_N[f])), 'knuckle_' + f))
        pts, ws = wrist_tail_path(f)
        out['tactile_tail_' + f] = _tail(pts, ws, TAIL_N, 'tail_' + f)
    pts, ws = thumb_service_path()
    out['tactile_tail_thumb'] = _tail(pts, ws, RAY_N['thumb'], 'tail_thumb')
    out['tactile_pod'] = pod_case()
    out['tactile_pod_lid'] = pod_lid()
    out['tactile_board'] = board_model()
    out['tactile_usb_hub'] = hub_model()
    _PARTS = out
    return out

def new_parts():
    return dict(_build_parts())

POD_PL = None
def pod_placement(dx=0.0, dy=0.0, dz=0.0):
    return App.Placement(pod_matrix().multiply(App.Placement(V(dx, dy, dz), App.Rotation()).toMatrix()))

def instance_specs():
    """(name, part, placement, colour, attach-to occurrence label or None)"""
    out = []
    for f, r in RAYS.items():
        for seg in ('dist', 'prox', 'meta', 'palm'):
            if r.get(seg):
                k, on = r[seg]
                out.append(('tactile_pad_%s_%s' % (f, seg), 'tactile_pad_' + k, occ(on), TEAL, on))
        for kind in ('J3', 'J2'):
            pl, (key, on, j) = joint_frame(f, kind)
            out.append(('tactile_loop_%s_%s' % (f, kind), 'tactile_loop_%s_n%d' % (kind, LOOP_W[kind][f]), pl, TAIL_COL, on))
        if f != 'thumb':
            key, on = r['palm']
            out.append(('tactile_knuckle_' + f, 'tactile_knuckle_' + f, occ(on), TAIL_COL, on))
            out.append(('tactile_tail_' + f, 'tactile_tail_' + f, App.Placement(), TAIL_COL, None))
    out.append(('tactile_tail_thumb', 'tactile_tail_thumb', App.Placement(), TAIL_COL, None))
    out.append(('tactile_pod', 'tactile_pod', pod_placement(), (0.25, 0.25, 0.28), None))
    out.append(('tactile_pod_lid', 'tactile_pod_lid', pod_placement(), (0.35, 0.35, 0.38), None))
    for name, xf, sy, rot in BAYS:
        if name == 'hub':
            out.append(('tactile_usb_hub', 'tactile_usb_hub', bay_placement(name), (0.1, 0.1, 0.1), None))
        else:
            out.append(('tactile_board_' + name, 'tactile_board', bay_placement(name), BOARD_COL, None))
    return out

def instances():
    return [{'name': n, 'part': p, 'placement': pl, 'group': 'Tactile', 'color': c} for n, p, pl, c, _ in instance_specs()]

def fastener_requests():
    """regions/fasteners_tactile.json: pod lid 4x M2x5 SHCS into M2 heat-set inserts (seat = underside of the head on
    the lid top, axis head -> tip)"""
    out = []
    for i, (x, sy) in enumerate(LID_LUGS):
        pl = pod_placement(*_lug_xy(x, sy), POD['H'])
        p = pl.Base
        out.append({'name': 'pod_lid_%d' % i, 'type': 'M2x5 SHCS', 'nut': 'M2 insert', 'nut_offset': POD['lid'],
                    'position': [round(p.x, 3), round(p.y, 3), round(p.z, 3)], 'axis': [0.0, 0.0, -1.0], 'frame': 'world',
                    'note': 'tactile readout pod lid (M2 heat-set insert in the corner post)'})
    return out

def attachments():
    """regions/attachments_tactile.json for scripts/rom_sweep.py: pads ride on their bones, each joint loop rides on its
    palm-side bone; the loops / knuckle ribbons / thumb tail are flexible where they cross a joint, so those pairs are
    excluded (their 0/90-deg shapes are checked by verify_tactile.py instead)."""
    att, exc = {}, []
    thumb_chain = ['hand_pulley_wheel_thumb <1>', 'first_thumb_hinge <1>', 'second_thumb_hinge <1>', 'third_thumb_hinge <1>',
                   'thumb_R2_tube']
    for f, r in RAYS.items():
        for seg in ('dist', 'prox', 'meta', 'palm'):
            if r.get(seg):
                att['tactile_pad_%s_%s' % (f, seg)] = r[seg][1]
        att['tactile_loop_%s_J3' % f] = r['prox'][1]
        att['tactile_loop_%s_J2' % f] = r['meta'][1]
        exc += [['tactile_loop_%s_J3' % f, r['dist'][1]], ['tactile_loop_%s_J3' % f, 'tactile_pad_%s_dist' % f],
                ['tactile_loop_%s_J2' % f, r['prox'][1]], ['tactile_loop_%s_J2' % f, 'tactile_pad_%s_prox' % f],
                ['tactile_loop_%s_J2' % f, 'tactile_loop_%s_J3' % f]]
        if r['palm']:
            k = 'tactile_knuckle_' + f
            exc += [[k, r['meta'][1]], [k, r['base']], [k, 'tactile_pad_%s_meta' % f], [k, 'tactile_loop_%s_J2' % f]]
        else:
            k = 'tactile_tail_thumb'
            for lab in thumb_chain + [r['meta'][1], r['prox'][1], r['dist'][1], 'tactile_pad_thumb_meta',
                                      'tactile_pad_thumb_prox', 'tactile_pad_thumb_dist', 'tactile_loop_thumb_J2', 'tactile_loop_thumb_J3']:
                exc.append([k, lab])
    return {'attach': att, 'exclude': exc,
            'note': 'tactile: pads ride on their bones, joint loops on their palm-side bone; loop x tip-side bone, '
                    'knuckle ribbon x finger and thumb tail x thumb chain are flexible crossings (checked at 0/90 deg '
                    'by verify_tactile.py, thumb tail by its length budget), not collisions'}
