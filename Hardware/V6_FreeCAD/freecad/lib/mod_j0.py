"""V6 J0: finger-to-palm pivot for an M2 socket-head cap screw + M2 nyloc nut (replaces the hammered pin).

Palm bones (local): J0 axis through (89.3, y, 0) along y, palm side = -y, lugs at |y| 5.0..7.5.
Base Bone 1 (local): J0 axis through (25.844, 0, z) along z, palm side = +z, tongue z -4.5..4.5.
Same design as the Onshape feature 'V6 J0 screw joint (M2)'; clearances default to 0.2 mm per side.
"""
import FreeCAD as App, Part
from v6geom import cyl, box, hex_prism, bone_section, planar_face, clean
V = App.Vector
C = 0.2                         # clearance per side
HOLE_R = 1.0 + C                # M2 clearance hole  (2.4)
CB_R = 1.9 + C                  # ISO 4762 M2 head 3.8 dia -> 4.2 counterbore
CB_DEPTH = 2.0 + C              # head 2.0 high -> flush
NUT_AF = 4.0 + 2 * C            # DIN 985 M2 nyloc 4.0 AF -> 4.4
NUT_DEPTH = 1.0
AX = 89.3
TONGUE_HALF = 4.5
PALM_INNER = 2.5                # palm-side lug inner face: y -5.0 -> -2.5 (lug 2.5 -> 5.0 mm)
BACK_INNER = TONGUE_HALF + C    # back lug inner face: y 5.0 -> 4.7 (lug 2.5 -> 2.8 mm)
TONGUE_TOP = PALM_INNER - C     # tongue trimmed to z <= 2.3
BAX = 25.844

def palm_bone(shape):
    palm_lug = planar_face(shape, (AX, -5, 0), (0, 1, 0), 95)
    back_lug = planar_face(shape, (AX, 5, 0), (0, -1, 0), 95)
    add = [palm_lug.extrude(V(0, 5 - PALM_INNER, 0)), back_lug.extrude(V(0, -(5 - BACK_INNER), 0))]
    plug = cyl((AX, -8, 0), (AX, 8, 0), 3.45).common(bone_section(AX - 5, AX + 5))
    plug = plug.cut(box((AX - 10, -PALM_INNER, -10), (AX + 10, BACK_INNER, 10)))
    s = shape.fuse(add + [plug])
    cuts = [cyl((AX, -8, 0), (AX, 8, 0), HOLE_R),
            cyl((AX, -8, 0), (AX, -7.5 + CB_DEPTH, 0), CB_R),
            hex_prism((AX, 8, 0), (0, -1, 0), (1, 0, 0), NUT_AF, 0.5 + NUT_DEPTH)]
    return clean(s.cut(cuts))

def base_bone(shape):
    sh = [f for f in shape.Faces if f.Surface.TypeId == 'Part::GeomCylinder'
          and abs(f.Surface.Radius - 8.75) < 0.01 and abs(abs(f.Surface.Axis.z) - 1) < 1e-6]
    if not sh:
        raise ValueError('Base Bone 1 shoulder recess R8.75 not found')
    c = sh[0].Surface.Center
    s = shape.fuse(cyl((BAX, 0, -TONGUE_HALF), (BAX, 0, TONGUE_HALF), 3.3))
    s = s.cut([cyl((c.x, c.y, TONGUE_TOP), (c.x, c.y, TONGUE_HALF + 0.1), 8.75),
               cyl((BAX, 0, -8), (BAX, 0, 8), HOLE_R)])
    return clean(s)

# J0 connector frames (part-local): origin, x axis, z axis  (identical to the Onshape J0 mates)
MC_PALM = ((AX, 0, 0), (-1, 0, 0), (0, -1, 0))
MC_BASE = ((BAX, 0, 0), (1, 0, 0), (0, 0, 1))

PALM_KEYS = ('palm_index', 'palm_middle', 'palm_ring', 'palm_pinky')

def modify(key, shape):
    if key in PALM_KEYS:
        return palm_bone(shape)
    if key == 'base_bone':
        return base_bone(shape)
    return shape
