"""Shared geometry helpers for DexiGrab V6 work in FreeCAD (GUI or headless freecadcmd).

Conventions (hand/world frame, mm): +X toward the fingertips, +Y toward the thumb, +Z = palm side.
Each part lives in its own local frame (as exported from the Onshape Part Studio); App::Link
objects in the document place them. Modifications are pure functions shape -> shape.
"""
import FreeCAD as App, Part, math
V = App.Vector

def cyl(a, b, r):
    """Solid cylinder between points a and b (App.Vector or tuple), radius r."""
    a, b = V(*a), V(*b)
    return Part.makeCylinder(r, (b - a).Length, a, (b - a).normalize())

def box(c1, c2):
    x0, y0, z0 = (min(c1[i], c2[i]) for i in range(3))
    x1, y1, z1 = (max(c1[i], c2[i]) for i in range(3))
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))

def hex_prism(center, axis, xdir, af, depth):
    """Hexagon (across-flats af) in the plane through center normal to axis, extruded depth along axis."""
    center, axis, xdir = V(*center), V(*axis).normalize(), V(*xdir).normalize()
    ydir = axis.cross(xdir)
    r = af / math.sqrt(3)
    pts = [center + xdir * (r * math.cos(math.radians(30 + 60 * i))) + ydir * (r * math.sin(math.radians(30 + 60 * i))) for i in range(6)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    return face.extrude(axis * depth)

def bone_section(x0, x1):
    """15 x 15 mm bone section (R6 corners centred at +-1.5) as a prism along local x."""
    s = box((x0, -7.5, -1.5), (x1, 7.5, 1.5)).fuse(box((x0, -1.5, -7.5), (x1, 1.5, 7.5)))
    for sy in (-1, 1):
        for sz in (-1, 1):
            s = s.fuse(cyl((x0, sy * 1.5, sz * 1.5), (x1, sy * 1.5, sz * 1.5), 6.0))
    return s.removeSplitter()

def planar_face(shape, point, normal, min_max_x=None, tol=1e-4):
    """The unique planar face of shape on plane (point, normal) with outward normal = normal."""
    point, normal = V(*point), V(*normal)
    hits = []
    for f in shape.Faces:
        if f.Surface.TypeId != 'Part::GeomPlane':
            continue
        n = f.normalAt(*f.Surface.parameter(f.CenterOfMass)) if hasattr(f, 'CenterOfMass') else f.Surface.Axis
        if n.dot(normal) < 0.999:
            continue
        if abs((f.Surface.Position - point).dot(normal)) > tol:
            continue
        if min_max_x is not None and f.BoundBox.XMax < min_max_x:
            continue
        hits.append(f)
    if len(hits) != 1:
        raise ValueError('expected one face on plane %s n=%s, found %d' % (point, normal, len(hits)))
    return hits[0]

def clean(s):
    """Refine and check a result solid."""
    s = s.removeSplitter()
    if not s.isValid():
        s.fix(1e-7, 1e-7, 1e-7)
    if len(s.Solids) == 1:
        s = s.Solids[0]
    return s

def placed(link):
    """World-space copy of a link's shape."""
    s = link.LinkedObject.Shape.copy()
    s.Placement = link.Placement
    return s

def min_gap(a, b):
    """Minimum distance between two shapes (0 if touching/overlapping) and the overlap volume."""
    d = a.distToShape(b)[0]
    ov = a.common(b).Volume if d < 1e-6 else 0.0
    return d, ov
