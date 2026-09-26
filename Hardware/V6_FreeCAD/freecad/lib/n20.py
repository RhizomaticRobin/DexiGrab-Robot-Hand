"""Straight (coaxial) N20 gearmotor with magnetic Hall encoder, modelled from the measured N20.sat
(sw_source/N20.sat, see data/sw_motor_models.json -> "N20").  Shared, read-only for all modules.

Local frame (mm):
  origin  = centre of the gearbox front face (mounting face)
  +X      = motor axis, from the gearbox toward the encoder (the output shaft points to -X)
  +Z      = side of the encoder-PCB tab and of the D-flat on the shaft
Overall 40.8 long (x -10.8 .. 30.0), 12 wide (y), 16 tall (z -5 .. 11 incl. the PCB tab).
"""
import FreeCAD as App, Part
V = App.Vector

SHAFT_D = 3.0
SHAFT_FLAT = 1.0          # flat 1.0 mm from the axis on the +Z side (2.5 across the flat)
SHAFT_TIP_X = -10.8       # 10.0 exposed beyond a 4.0 x 0.8 boss
BOSS = (4.0, 0.8)
GEARBOX_X = (0.0, 9.0)    # 12 (y) x 10 (z)
CAN_X = (9.0, 23.75)      # dia 12 with flats at z = +-5
CAP_X = (23.75, 24.5)
PCB_X = (24.5, 25.5)      # 12 (y) x 16 (z: -5 .. 11), tab 6 mm above the can's +Z flat
MAGNET = (9.0, 26.5, 30.0)
CONNECTOR = ((25.5, 29.0), (-5.25, 5.25), (6.45, 10.95))   # 6-pin 1.5 mm pitch (JST ZH style), mates along +X
MOUNT_HOLES = [(0.0, -4.5, 0.0), (0.0, 4.5, 0.0)]         # typical N20 front plate: 2x M1.6, 9 mm apart (verify on the real part)
MOUNT_HOLE_D, MOUNT_HOLE_DEPTH = 1.6, 3.0                  # tapped M1.6 (modelled at the major diameter) into the gearbox front

def _box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))

def parts():
    """Dict of named solids (local frame)."""
    shaft = Part.makeCylinder(SHAFT_D / 2, -SHAFT_TIP_X, V(SHAFT_TIP_X, 0, 0), V(1, 0, 0))
    shaft = shaft.cut(_box(SHAFT_TIP_X - 1, -BOSS[1], -2, 2, SHAFT_FLAT, 2))
    boss = Part.makeCylinder(BOSS[0] / 2, BOSS[1], V(-BOSS[1], 0, 0), V(1, 0, 0))
    gearbox = _box(GEARBOX_X[0], GEARBOX_X[1], -6, 6, -5, 5)
    for hx, hy, hz in MOUNT_HOLES:
        gearbox = gearbox.cut(Part.makeCylinder(MOUNT_HOLE_D / 2, MOUNT_HOLE_DEPTH, V(hx, hy, hz), V(1, 0, 0)))
    can = Part.makeCylinder(6.0, CAN_X[1] - CAN_X[0], V(CAN_X[0], 0, 0), V(1, 0, 0)).common(_box(CAN_X[0], CAN_X[1], -6, 6, -5, 5))
    cap = Part.makeCylinder(5.0, CAP_X[1] - CAP_X[0], V(CAP_X[0], 0, 0), V(1, 0, 0))
    pcb = _box(PCB_X[0], PCB_X[1], -6, 6, -5, 11)
    magnet = Part.makeCylinder(MAGNET[0] / 2, MAGNET[2] - MAGNET[1], V(MAGNET[1], 0, 0), V(1, 0, 0))
    rear_shaft = Part.makeCylinder(0.5, MAGNET[1] - CAP_X[0] + 0.5, V(CAP_X[0], 0, 0), V(1, 0, 0))   # dia 1 rear shaft through the PCB to the magnet
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = CONNECTOR
    connector = _box(cx0, cx1, cy0, cy1, cz0, cz1)
    return {'shaft': shaft, 'boss': boss, 'gearbox': gearbox, 'can': can, 'cap': cap, 'pcb': pcb, 'magnet': magnet, 'rear_shaft': rear_shaft, 'connector': connector}

def shape():
    """One solid (fused) for fit checks and display."""
    p = parts()
    s = p['gearbox'].fuse([p[k] for k in ('shaft', 'boss', 'can', 'cap', 'pcb', 'magnet', 'rear_shaft', 'connector')])
    return s.removeSplitter()

def body_envelope(clearance=0.2):
    """Seat cut tool: gearbox + can + cap + PCB + connector grown by `clearance` (no shaft)."""
    c = clearance
    env = _box(GEARBOX_X[0] - c, CAN_X[1] + c, -6 - c, 6 + c, -5 - c, 5 + c)
    env = env.fuse(_box(PCB_X[0] - c, PCB_X[1] + c, -6 - c, 6 + c, -5 - c, 11 + c))
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = CONNECTOR
    env = env.fuse(_box(cx0 - c, cx1 + c, cy0 - c, cy1 + c, cz0 - c, cz1 + c))
    env = env.fuse(Part.makeCylinder(MAGNET[0] / 2 + c, MAGNET[2] - MAGNET[1] + c, V(MAGNET[1], 0, 0), V(1, 0, 0)))
    return env.removeSplitter()

def plug_keepout(plug_len=8.0):
    """Space behind the connector for the mating plug and the first wire bend (mating direction +X)."""
    (cx0, cx1), (cy0, cy1), (cz0, cz1) = CONNECTOR
    return _box(cx1, cx1 + plug_len, cy0 - 0.5, cy1 + 0.5, cz0 - 0.5, cz1 + 0.5)

REF = {'shaft_tip': (SHAFT_TIP_X, 0, 0), 'front_face': (0, 0, 0), 'can_centre': ((CAN_X[0] + CAN_X[1]) / 2, 0, 0),
       'rear_end': (MAGNET[2], 0, 0), 'connector_centre': (27.25, 0, 8.7), 'axis': (1, 0, 0), 'tab_dir': (0, 0, 1)}
