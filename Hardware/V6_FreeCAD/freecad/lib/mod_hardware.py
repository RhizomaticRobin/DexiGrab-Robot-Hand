"""V6 hardware (topic 'hardware'): every screw and nut, placed and validated.

new_parts()  Fastener library in seat frames. M2 socket-head cap screws are the sw_source STEP heads
             (with the real 1.5 mm hex socket) on plain major-diameter shanks; M1.6 cap screws, DIN 985 M2
             nylocs, M1.6 nuts, ISO 7089 washers and the M2 heat-set insert are parametric. The M2 hex nut
             is the STEP body with its helical thread replaced by a plain bore. Nothing has helical faces.
instances()  (1) J0 pivots: 4 x M2x16 ISO 4762 + M2 DIN 985 nyloc (stack from mod_j0);
             (2) the V5 fastener sites (data/sw_fasteners_onshape.csv) whose holes exist in the built geometry;
             (3) every request in regions/fasteners_*.json, read at build time.
             Each joint is snapped to its modelled hole, its nut seat is inferred when not given, and it is
             validated against the built geometry: hole with 0.2 mm/side clearance, head and nut seated,
             counterbore / nut pocket clearance, thread engagement, collisions. Problems and open items go
             to regions/hardware_report.json (rewritten on every build).

Seat frame of every fastener part: origin = seat point (underside of a screw head; bearing face of a nut or
washer; top face of an insert), +Z from the head toward the tip. Screws occupy z in [-k, L]; nuts z in [0, h]
with their hex flats normal to local X.

Request protocol (README): {"type": "M2x8 SHCS", "nut": "M2 hex"|"M2 nyloc"|"M2 insert"|null,
"position": seat point, "axis": head->tip, "frame": "world", "note": ...}. Optional extras understood here:
"name", "nut_offset" (mm from the seat to the face the nut/insert bears on; inferred when absent),
"washer": "head"|"nut"|"both", "frame": "local" with "part": <part key> (one joint per instance of that
part) or "instance": <link label>.

env (testing): V6_HW_REQUESTS (glob, default regions/fasteners_*.json), V6_HW_REPORT (report path),
V6_HW_VALIDATE=0 (skip geometry checks).
"""
import os, sys, re, json, math, glob, time, datetime, hashlib
import FreeCAD as App, Part

LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
if LIB not in sys.path:
    sys.path.insert(0, LIB)
from v6geom import cyl, box, hex_prism, clean

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD'
FC = ROOT + '/freecad'
SW = os.path.join(ROOT, '..', 'Biomemetic Robot Hand, Prosthetic')   # this repo's V5 SolidWorks folder (M2 screw/nut STEP models)
REGIONS = FC + '/regions'
C = 0.2            # clearance per side (requirement 7)
TOL = 0.02         # check tolerance: a modelled 0.2 mm gap passes, 0.18 fails
GROUP = 'Hardware'
VERSION = 'hardware-1.0'

def _request_glob():
    return os.environ.get('V6_HW_REQUESTS', REGIONS + '/fasteners_*.json')

def _report_path():
    return os.environ.get('V6_HW_REPORT', REGIONS + '/hardware_report.json')

# ------------------------------------------------------------------------------------------------ catalogue
SHCS = {    # ISO 4762: head dk max, head height k max, socket s, socket depth t min, pitch P
    'M2':   dict(d=2.0, dk=3.8, k=2.0, s=1.5, t=1.0, P=0.40),
    'M1.6': dict(d=1.6, dk=3.0, k=1.6, s=1.5, t=0.7, P=0.35),
}
LENGTHS = {'M2': (3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 25, 30),       # all exist as sw_source STEP files
           'M1.6': (2, 2.5, 3, 4, 5, 6, 8, 10, 12, 16)}                 # ISO 4762 preferred lengths
NUTS = {
    'M2 hex':    dict(size='M2', kind='hex', std='ISO 4032', s=4.0, h=1.6, e=4.32),
    'M2 nyloc':  dict(size='M2', kind='nyloc', std='DIN 985 (M2 size per supplier tables)', s=4.0, h=2.8, h_max=3.0,
                      m=1.6, e=4.32, hex_h=1.8),
    'M1.6 hex':  dict(size='M1.6', kind='hex', std='ISO 4032', s=3.2, h=1.3, e=3.41),
    'M2 insert': dict(size='M2', kind='insert', std='brass heat-set insert M2 x 3 x OD 3.2 (generic)', od=3.2, h=3.0, hole=3.0),
    'M2 long insert': dict(size='M2', kind='insert', std='brass heat-set insert ruthex RX-M2x4 (OD 3.6 x 4)', od=3.6, h=4.0, hole=3.2),
}
WASHERS = {'M2': dict(std='ISO 7089', d1=2.2, d2=5.0, h=0.3),
           'M1.6': dict(std='ISO 7089', d1=1.7, d2=4.0, h=0.3)}
BASE_LIBRARY = ['M2x5 SHCS', 'M2x8 SHCS', 'M2x10 SHCS', 'M2x12 SHCS', 'M2x14 SHCS', 'M2x16 SHCS',
                'M2 hex', 'M2 nyloc', 'M2 washer', 'M1.6x3 SHCS', 'M1.6x5 SHCS', 'M1.6 hex']
COLORS = {'screw': (0.16, 0.16, 0.18), 'nut': (0.78, 0.78, 0.80), 'washer': (0.86, 0.86, 0.88),
          'insert': (0.80, 0.64, 0.30), 'fail': (0.95, 0.10, 0.10)}

def _fmt(v):
    return ('%g' % v).replace('.', 'p')

def _sz(size):
    return size.lower().replace('.', 'p')

def screw_type(size, L):
    return '%sx%g SHCS' % (size, L)

def key_for(t):
    """Part key of a canonical type name ('M2x16 SHCS', 'M2 nyloc', 'M2 washer', ...)."""
    if t.endswith('SHCS'):
        size, L = parse_screw(t)
        return 'hardware_%sx%s_shcs' % (_sz(size), _fmt(L))
    if t.endswith('washer'):
        return 'hardware_%s_washer' % _sz(t.split()[0])
    size, kind = t.split(' ', 1)
    return 'hardware_%s_%s' % (_sz(size), {'hex': 'hex_nut', 'nyloc': 'nyloc_nut', 'insert': 'insert',
                                          'long insert': 'long_insert'}[kind])

def parse_screw(t):
    """'M2x16 SHCS' / 'M2 x 16' / 'M1.6x5' -> ('M2', 16.0). Only ISO 4762 socket-head cap screws."""
    s = str(t)
    m = re.search(r'M\s*(\d+(?:\.\d+)?)\s*[x×X*]\s*(\d+(?:\.\d+)?)', s)
    if not m:
        raise ValueError('cannot parse screw type %r (expected e.g. "M2x8 SHCS")' % t)
    size, L = 'M%g' % float(m.group(1)), float(m.group(2))
    if size not in SHCS:
        raise ValueError('screw size %s not in the library (M2, M1.6)' % size)
    low = s.lower()
    if any(w in low for w in ('button', 'countersunk', 'flat head', 'pan head', 'low head', 'set screw')):
        raise ValueError('only ISO 4762 socket-head cap screws are modelled, got %r' % t)
    return size, L

def parse_nut(t, size):
    if t is None or str(t).strip().lower() in ('', 'none', 'null', 'no', 'false'):
        return None
    low = str(t).lower()
    m = re.search(r'm\s*(\d+(?:\.\d+)?)', low)
    sz = 'M%g' % float(m.group(1)) if m else size
    if 'insert' in low or 'heat' in low:
        kind = 'long insert' if any(w in low for w in ('long', 'ruthex', 'x4', '4mm', '4 mm', '3.6')) else 'insert'
    else:
        kind = 'nyloc' if ('nyl' in low or 'lock' in low) else 'hex'
    name = '%s %s' % (sz, kind)
    if name not in NUTS:
        raise ValueError('nut %r is not in the library (%s)' % (t, ', '.join(NUTS)))
    if sz != size:
        raise ValueError('nut %r does not fit a %s screw' % (t, size))
    return name

# ------------------------------------------------------------------------------------------------ geometry
def _revolve(profile):
    """Solid of revolution about +Z from a closed (r, z) profile (consecutive duplicates dropped)."""
    pts = []
    for r, z in profile:
        p = V(r, 0, z)
        if not pts or (p - pts[-1]).Length > 1e-9:
            pts.append(p)
    if (pts[0] - pts[-1]).Length < 1e-9:
        pts.pop()
    return Part.Face(Part.makePolygon(pts + [pts[0]])).revolve(V(0, 0, 0), V(0, 0, 1), 360)

def _moved(shape, v):
    """Copy of shape translated by v with the transform baked into the geometry (identity Placement)."""
    s = shape.copy()
    m0 = s.Placement.toMatrix()
    s.Placement = App.Placement()
    m = App.Matrix()
    m.move(V(*v))
    s.transformShape(m.multiply(m0), True)
    return s

def _step(name):
    for d in (SW + '/M2 nuts and screws', SW):
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return None

def _shank(d, L):
    r, ch, rt = d / 2.0, 0.18 * d, 0.243 * d          # tip chamfer as on the STEP screws (M2: 0.36 long to r 0.486)
    return _revolve([(0, 0), (r, 0), (r, L - ch), (rt, L), (0, L)])

def _param_head(sp):
    dk, k, s, t = sp['dk'], sp['k'], sp['s'], sp['t']
    c = 0.05 * dk
    head = _revolve([(0, -k), (dk / 2 - c, -k), (dk / 2, -k + c), (dk / 2, -c), (dk / 2 - c, 0), (0, 0)])
    return head.cut(hex_prism((0, 0, -k - 0.1), (0, 0, 1), (1, 0, 0), s, t + 0.1))

def _screw_shape(size, L):
    sp = SHCS[size]
    path = _step('M2 x 0.4mm Thread %gmm LONG SOCKET HEAD CAP SCREW.STEP' % L) if size == 'M2' else None
    if path:
        raw = _moved(Part.read(path), (0, 0, L))        # STEP frame: tip at z=0, head underside at z=-L
        head = raw.common(box((-3, -3, -sp['k'] - 1), (3, 3, 0)))
        how = 'head from sw_source "%s" + plain shank' % os.path.basename(path)
    else:
        head, how = _param_head(sp), 'parametric ISO 4762'
    return clean(head.fuse(_shank(sp['d'], L))), how

def _hex_body(s, h, top_chamfer=True):
    """Hex prism (flats normal to X) with the usual 30 deg corner chamfer (chamfer circle = s) on the bearing
    face and, optionally, on the top face."""
    body = hex_prism((0, 0, 0), (0, 0, 1), (1, 0, 0), s, h)
    t = math.tan(math.radians(60))
    if top_chamfer:
        prof = [(0, 0), (s / 2, 0), (s / 2 + (h / 2) * t, h / 2), (s / 2, h), (0, h)]
    else:
        prof = [(0, 0), (s / 2, 0), (s / 2 + h * t, h), (0, h)]
    return body.common(_revolve(prof))

def _nut_shape(name):
    n = NUTS[name]
    d = SHCS[n['size']]['d']
    bore = cyl((0, 0, -1), (0, 0, n['h'] + 1), d / 2 + 0.01)
    if n['kind'] == 'hex' and n['size'] == 'M2' and _step('M2 x 0.4mm Thread HEX NUT.STEP'):
        raw = _moved(Part.read(_step('M2 x 0.4mm Thread HEX NUT.STEP')), (0, 0, n['h'] / 2))   # STEP is centred
        s = raw.fuse(cyl((0, 0, 0), (0, 0, n['h']), d / 2 + 0.03)).cut(bore)               # helical thread -> plain bore
        return clean(s), 'sw_source "M2 x 0.4mm Thread HEX NUT.STEP", thread replaced by a plain bore'
    if n['kind'] == 'hex':
        return clean(_hex_body(n['s'], n['h']).cut(bore)), 'parametric %s' % n['std']
    if n['kind'] == 'nyloc':
        s, h, hh = n['s'], n['h'], n['hex_h']
        rc = 0.475 * s
        collar = _revolve([(0, hh - 0.05), (rc, hh - 0.05), (rc, h - 0.3), (rc - 0.3, h), (0, h)])
        return clean(_hex_body(s, hh, top_chamfer=False).fuse(collar).cut(bore)), 'parametric %s' % n['std']
    # heat-set insert: knurl envelope with two relief grooves, top at z=0
    od, L = n['od'], n['h']
    body = _revolve([(0, 0), (od / 2 - 0.15, 0), (od / 2, 0.15), (od / 2, L - 0.1), (od / 2 - 0.1, L), (0, L)])
    for zc in (L * 0.35, L * 0.7):
        body = body.cut(_revolve([(od / 2 - 0.12, zc - 0.25), (od / 2 + 0.1, zc - 0.25), (od / 2 + 0.1, zc + 0.25), (od / 2 - 0.12, zc + 0.25)]))
    return clean(body.cut(bore)), 'parametric %s' % n['std']

def _washer_shape(size):
    w = WASHERS[size]
    s = cyl((0, 0, 0), (0, 0, w['h']), w['d2'] / 2).cut(cyl((0, 0, -1), (0, 0, 1), w['d1'] / 2))
    return clean(s), 'ISO 7089 (the sw_source washer STEP is 0.4 thick; ISO 7089 M2 is 0.3)' if size == 'M2' else 'ISO 7089'

def build_type(t):
    """(part key, shape in its seat frame, provenance) of a canonical type name."""
    if t.endswith('SHCS'):
        shape, how = _screw_shape(*parse_screw(t))
    elif t.endswith('washer'):
        shape, how = _washer_shape(t.split()[0])
    else:
        shape, how = _nut_shape(t)
    if not shape.Placement.isIdentity():
        shape = _moved(shape, (0, 0, 0))
    return key_for(t), shape, how

# ------------------------------------------------------------------------------------------------ data
PALM_LABELS = {'index': 'Base Bone 1.2_V02_pointerfinger_and_thumb_attachment <1>',
               'middle': 'Base Bone 1.2_V02_middlefinger <1>',
               'ring': 'Base Bone 1.2_V02_ringfing <1>',
               'pinky': 'Base Bone 1.2_V02_pinky <1>'}
BASE_LABELS = {'index': 'Base Bone 1_V02 <1>', 'middle': 'Base Bone 1_V02 <2>', 'ring': 'Base Bone 1_V02 <4>',
               'pinky': 'Base Bone 1_V02 <3>'}

def j0_local():
    """J0 stack in palm-bone local coordinates, from mod_j0: seat, nut face, axis, nut flat normal."""
    try:
        import mod_j0
        ax, cb, nd = mod_j0.AX, mod_j0.CB_DEPTH, mod_j0.NUT_DEPTH
    except Exception:
        ax, cb, nd = 89.3, 2.2, 1.0
    return dict(seat=(ax, -7.5 + cb, 0.0), nut=(ax, 7.5 - nd, 0.0), axis=(0.0, 1.0, 0.0), x=(1.0, 0.0, 0.0))

_JSON = {}
def json_placement(label):
    """Occurrence placement from data/top_assembly_definition.json (same source as the FCStd links)."""
    if 'occ' not in _JSON:
        d = json.load(open(ROOT + '/data/top_assembly_definition.json'))
        ra = d['rootAssembly']
        inst = {i['id']: i for i in ra['instances']}
        for s in d['subAssemblies']:
            for i in s['instances']:
                inst[i['id']] = i
        _JSON['occ'] = {inst[o['path'][-1]]['name']: o['transform'] for o in ra['occurrences']}
    t = _JSON['occ'][label]
    return App.Placement(App.Matrix(t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000,
                                    t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1))

V5_CATEGORY = [   # (instance regex, category, id prefix, what, owner hint)
    (r'^m2x5_assemb-', 'bracket', 'BRK', 'M2x5 cover bracket to palm bone', 'owner of cover2/cover3 + palm bones'),
    (r'^m2x10_assemb-', 'driver', 'DRV', 'M2x8 driver-board mount', 'electronics'),
    (r'^M2 x 0\.4mm Thread (5mm LONG SOCKET HEAD CAP SCREW\.step-3|HEX NUT\.step-4)$', 'driver', 'DRV', 'M2x5 driver mount in cover_back', 'electronics'),
    (r'^Battery_case-1/m2x5_batt_assemb-', 'battery', 'BAT', 'M2x5 battery cover', 'electronics (battery case)'),
    (r'^Mirrorm1_screw_assemb_fingies-', 'fingertip', 'TIP', 'M1.6 fingertip tendon anchor', 'actuation'),
    (r'^(Mirrorm1_screw_assemb_copy-|m1_screw_assemb_copy-)', 'thumb', 'THB', 'M1.6 thumb gimbal', 'thumb'),
    (r'^(m1_screw_assemb-[1245]/|TR Fastenings Ltd-M1_6X3mm-[34]$|m1\.6 Nut \.35mm pitch-[34]$)', 'cover_back', 'CBK', 'M1.6 cover_back / Shell1', 'shells'),
]

def _vec(t):
    return V(*[float(x) for x in t.split()])

def v5_sites():
    """V5 SolidWorks fastener joints (screw + its nut), in the hand frame, grouped by category."""
    import csv
    rows = list(csv.DictReader(open(ROOT + '/data/sw_fasteners_onshape.csv')))
    screws, nuts = [], []
    for r in rows:
        if r['hidden'] == 'True' or r['coincident_duplicate_of'] or r['hosts_inferred'].startswith('(none'):
            continue
        cat = next((c for c in V5_CATEGORY if re.search(c[0], r['instance'])), None)
        if cat is None:
            continue
        (screws if r['kind'].startswith('screw') else nuts).append((r, cat))
    joints = []
    for r, cat in screws:
        H, A, T = _vec(r['head_or_centre_os']), _vec(r['axis_os(head->tip)']), _vec(r['tip_os'])
        A.normalize()
        m2 = r['type'].startswith('M2')
        k = 2.0 if m2 else 1.3                          # V5 M1.6x3 were TR Fastenings pan heads (k 1.3)
        P = H + A * k
        L = round((T - H).Length - k, 1)
        best = None
        for n, ncat in nuts:
            c = _vec(n['head_or_centre_os'])
            w = c - P
            along = w.dot(A)
            off = (w - A * along).Length
            if off < 0.6 and -0.5 < along < L + 1 and (best is None or off < best[0]):
                best = (off, n, along)
        zn = None
        if best:
            m = 1.6 if best[1]['type'].startswith('M2') else 1.3
            zn = best[2] - m / 2
        size = 'M2' if m2 else 'M1.6'
        joints.append(dict(v5=r['instance'], category=cat[1], prefix=cat[2], what=cat[3], owner=cat[4],
                           size=size, L=L if m2 else 3.0, nut='%s hex' % size, P=P, A=A, zn=zn,
                           hosts=r['hosts_inferred']))
    # stable ids: brackets by host bone and x (knuckle first), the rest by category order
    out, count = [], {}
    for j in sorted(joints, key=lambda j: (j['category'], ('pointer' not in j['hosts']), -j['P'].x)):
        if j['category'] == 'bracket':
            host = 'I' if 'pointerfinger' in j['hosts'] and 'thumb_hinge' not in j['hosts'] else ('P' if 'pinky' in j['hosts'] else 'T')
            tag = '%s_%s' % (j['prefix'], host)
        elif j['category'] == 'driver':
            tag = '%s_%s' % (j['prefix'], 'C3' if 'cover3' in j['hosts'] else ('C2' if 'cover2' in j['hosts'] else 'CB'))
        else:
            tag = j['prefix']
        count[tag] = count.get(tag, 0) + 1
        j['id'] = '%s%d' % (tag, count[tag])
        out.append(j)
    return out

# ------------------------------------------------------------------------------------------------ document
def find_doc():
    docs = [App.ActiveDocument] if App.ActiveDocument else []
    for n in App.listDocuments():
        d = App.getDocument(n)
        if d not in docs:
            docs.append(d)
    for d in docs:
        if any(o.TypeId == 'App::Link' and o.Label == PALM_LABELS['index'] for o in d.Objects):
            return d
    return None

class World:
    """World-placed solids of every instance in the document except this module's own hardware."""
    def __init__(self, doc, exclude=None):
        self.doc = doc
        self.items = []
        for o in doc.Objects:
            if o.TypeId != 'App::Link' or o.LinkedObject is None:
                continue
            if o.Name.startswith('I_HW_') or o.LinkedObject.Name.startswith('V6_hardware_'):
                continue
            if exclude and o.Label in exclude:
                continue
            s = o.LinkedObject.Shape
            if s.isNull() or not s.Solids:
                continue                                   # mesh-only parts (Shell2 as imported) cannot be checked
            self.items.append(dict(label=o.Label, name=o.Name, link=o, local=s,
                                   bb=s.BoundBox.transformed(o.Placement.toMatrix()), _w=None))

    def shape(self, it):
        if it['_w'] is None:
            w = it['local'].copy()                        # obj.Shape is immutable in FreeCAD 1.1; copied lazily,
            w.Placement = it['link'].Placement            # only for parts near a fastener (same convention as v6geom.placed)
            it['_w'] = w
        return it['_w']

    def release(self):
        for it in self.items:
            it['_w'] = None
            it.pop('_f', None)

    def faces(self, it):
        """[(face, world bbox)] of an item, cached."""
        if '_f' not in it:
            it['_f'] = [(f, f.BoundBox) for f in self.shape(it).Faces]
        return it['_f']

    def near(self, bb, margin=0.5):
        b = App.BoundBox(bb)
        b.enlarge(margin)
        return [it for it in self.items if it['bb'].intersect(b)]

    def fingerprint(self):
        h = hashlib.sha1()
        for it in sorted(self.items, key=lambda i: i['name']):
            p = it['link'].Placement
            h.update(('%s|%s|%.4f|%s|%s' % (it['name'], it['link'].LinkedObject.Name, it['local'].Volume,
                                           tuple(round(x, 4) for x in (it['bb'].XMin, it['bb'].YMin, it['bb'].ZMin, it['bb'].XMax, it['bb'].YMax, it['bb'].ZMax)),
                                           tuple(round(x, 5) for x in p.toMatrix().A))).encode())
        return h.hexdigest()

def _placement(P, A, X=None):
    A = V(A)
    A.normalize()
    X = V(X) if X is not None else V(1, 0, 0)
    X = X - A * X.dot(A)
    if X.Length < 1e-6:
        t = V(1, 0, 0) if abs(A.x) < 0.9 else V(0, 1, 0)
        X = t - A * t.dot(A)
    X.normalize()
    Y = A.cross(X)
    return App.Placement(App.Matrix(X.x, Y.x, A.x, P.x, X.y, Y.y, A.y, P.y, X.z, Y.z, A.z, P.z, 0, 0, 0, 1))

def _local(shape, pl):
    s = shape.copy()
    s.Placement = pl
    return s

def _tube(r0, r1, z0, z1):
    s = cyl((0, 0, z0), (0, 0, z1), r1)
    return s.cut(cyl((0, 0, z0 - 1), (0, 0, z1 + 1), r0)) if r0 > 0 else s

def _hexenv(af, z0, z1):
    return hex_prism((0, 0, z0), (0, 0, 1), (1, 0, 0), af, z1 - z0)

def overlap(a, b, volume=True, b_faces=None):
    """(overlaps, volume, distance) of two solids. distToShape reports the boundary distance even when one
    solid lies completely inside the other, so containment is tested explicitly. b_faces = [(face, bbox)] of b:
    the boundary distance is then measured only to b's faces near a (much faster on big parts)."""
    if b_faces is not None:
        ab = App.BoundBox(a.BoundBox)
        ab.enlarge(0.05)
        near = [f for f, fb in b_faces if fb.intersect(ab)]
        d = a.distToShape(Part.Compound(near))[0] if near else 1e9
    else:
        d = a.distToShape(b)[0]
    if d > 1e-7:
        pa = a.Vertexes[0].Point if a.Vertexes else a.CenterOfMass
        pb = b.Vertexes[0].Point if b.Vertexes else b.CenterOfMass
        if b.isInside(pa, 1e-6, True) or a.isInside(pb, 1e-6, True):
            return True, (min(a.Volume, b.Volume) if volume else -1.0), 0.0
        return False, 0.0, d
    if not volume:
        return True, -1.0, 0.0
    try:
        v = a.common(b).Volume
    except Exception:
        v = -1.0
    return (v > 1e-5 or v < 0), v, 0.0

# ------------------------------------------------------------------------------------------------ joints
def _joint(id, source, size, L, nut, P, A, X=None, zn=None, washer=None, snap=False, note='', **extra):
    j = dict(id=id, source=source, size=size, L=float(L), nut=nut, P=V(P), A=V(A), X=V(X) if X is not None else None,
             zn=zn, washer=washer, snap=snap, note=note, problems=[], checks={}, status='unvalidated')
    j['A'].normalize()
    j.update(extra)
    return j

def j0_joints(doc):
    loc = j0_local()
    out = []
    for f, lab in PALM_LABELS.items():
        src = 'document link'
        pl = None
        if doc is not None:
            objs = doc.getObjectsByLabel(lab)
            if objs:
                pl = objs[0].Placement
        if pl is None:
            pl, src = json_placement(lab), 'top_assembly_definition.json'
        P = pl.multVec(V(*loc['seat']))
        N = pl.multVec(V(*loc['nut']))
        A = pl.Rotation.multVec(V(*loc['axis']))
        X = pl.Rotation.multVec(V(*loc['x']))
        out.append(_joint('J0_%s' % f, 'j0', 'M2', 16, 'M2 nyloc', P, A, X, zn=(N - P).dot(A),
                          note='J0 pivot, %s finger; palm-bone placement from %s' % (f, src),
                          palm=lab, base=BASE_LABELS[f], finger=f))
    return out

def _read_requests():
    """[(file, status, entries)] for every regions/fasteners_*.json; tolerant of partial / in-progress writes."""
    out = []
    active = [m for m in os.environ.get('V6_MODULES', '').split(',') if m]   # orchestrator: only requests from modules in this build
    for f in sorted(glob.glob(_request_glob())):
        topic = os.path.basename(f)[len('fasteners_'):-len('.json')]
        if active and topic not in active:
            continue
        data, err = None, None
        for attempt in range(3):
            try:
                with open(f) as fh:
                    txt = fh.read()
                if not txt.strip():
                    raise ValueError('empty file')
                data, err = json.loads(txt), None
                break
            except Exception as e:
                err = '%s: %s' % (type(e).__name__, e)
                time.sleep(0.3)
        if err:
            out.append((f, 'unreadable (%s); skipped this build' % err, []))
            continue
        if isinstance(data, dict):
            data = data.get('fasteners', data.get('requests', [data]))
        if not isinstance(data, list):
            out.append((f, 'unexpected top-level %s; expected a list of requests' % type(data).__name__, []))
            continue
        out.append((f, 'ok', data))
    return out

def _links_for(doc, e):
    if doc is None:
        return []
    if e.get('instance'):
        return list(doc.getObjectsByLabel(e['instance']))
    key = e.get('part')
    if not key:
        return []
    import partkeys
    names = {'V6_' + key}
    if key in partkeys.PARTS and partkeys.PARTS[key][0]:
        names.add(partkeys.PARTS[key][0])
    return [o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None
            and o.LinkedObject.Name in names and not o.Name.startswith('I_HW_')]

def _safe(s):
    return re.sub(r'[^A-Za-z0-9_]', '_', str(s))

# V6.1 proportions: world-frame requests were written at the V6 rest pose; mod_proportions re-poses the downstream
# finger / thumb bones (e.g. the thumb R6 bolt now sits 8.7 mm closer to R5), so a request seated on a moved bone is
# moved with it.  Identity when mod_proportions is not part of this build.
def _proportions_move(topic, e, P, A):
    active = [m for m in os.environ.get('V6_MODULES', '').split(',') if m]
    if active and 'proportions' not in active:
        return P, A
    try:
        import mod_proportions
    except ImportError:
        return P, A
    return mod_proportions.move_world_request(topic, e, P, A)

def request_joints(doc):
    joints, files = [], []
    for f, status, entries in _read_requests():
        topic = os.path.basename(f)[len('fasteners_'):-len('.json')] if os.path.basename(f).startswith('fasteners_') else os.path.basename(f)
        info = dict(file=os.path.relpath(f, FC), status=status, entries=len(entries), placed=0, rejected=[])
        for i, e in enumerate(entries, 1):
            rid = 'R_%s_%s' % (_safe(topic), _safe(e.get('name') or e.get('id') or i) if isinstance(e, dict) else i)
            try:
                if not isinstance(e, dict):
                    raise ValueError('entry is not an object')
                size, L = parse_screw(e['type'])
                nut = parse_nut(e.get('nut'), size)
                P, A = V(*[float(x) for x in e['position']]), V(*[float(x) for x in e['axis']])
                if A.Length < 1e-9:
                    raise ValueError('zero axis')
                frame = e.get('frame', 'world')
                washer = e.get('washer')
                if washer not in (None, 'head', 'nut', 'both'):
                    raise ValueError('washer must be "head", "nut", "both" or null')
                zn = e.get('nut_offset')
                zn = float(zn) if zn is not None else None
                common = dict(size=size, L=L, nut=nut, zn=zn, washer=washer, snap=True, note=str(e.get('note', '')),
                              request=dict(file=info['file'], index=i, entry=e))
                if frame == 'world':
                    P, A = _proportions_move(topic, e, P, A)     # V6.1 proportions: ride along with a re-posed bone
                    joints.append(_joint(rid, 'request', P=P, A=A, **common))
                elif frame == 'local':
                    links = _links_for(doc, e)
                    if not links:
                        raise ValueError('frame "local" needs "part" (part key) or "instance" (link label) that exists in the document')
                    for n, lk in enumerate(links, 1):
                        pl = lk.Placement
                        joints.append(_joint(rid + ('_%d' % n if len(links) > 1 else ''), 'request',
                                             P=pl.multVec(P), A=pl.Rotation.multVec(A), host_instance=lk.Label, **common))
                else:
                    raise ValueError('frame must be "world" or "local"')
                info['placed'] += 1
            except KeyError as ex:
                info['rejected'].append('entry %d: missing key %s' % (i, ex))
            except Exception as ex:
                info['rejected'].append('entry %d: %s' % (i, ex))
        files.append(info)
    return joints, files

# ------------------------------------------------------------------------------------------------ resolve
def _faces_near(world, items, P, A, zlo, zhi, rmax):
    """Faces of the given parts within the cylinder (P, A, r<=rmax, z in [zlo, zhi])."""
    probe = _local(cyl((0, 0, zlo), (0, 0, zhi), rmax), _placement(P, A))
    pb = probe.BoundBox
    out = []
    for it in items:
        for f, fb in world.faces(it):
            if fb.intersect(pb):
                out.append((it, f))
    return out, probe

def _snap_axis(world, items, j, sp):
    """Move the joint onto the axis of a modelled bore (concave cylinder, dia d+0.04..d+1.6) within 0.35 mm / 2 deg."""
    P, A = j['P'], j['A']
    best = None
    faces, _ = _faces_near(world, items, P, A, -sp['k'] - 1, j['L'] + 1, sp['dk'])
    for it, f in faces:
        su = f.Surface
        if su.TypeId != 'Part::GeomCylinder':
            continue
        if not (sp['d'] / 2 + 0.02 <= su.Radius <= sp['d'] / 2 + 0.8):
            continue
        ax = V(su.Axis)
        if abs(ax.dot(A)) < math.cos(math.radians(2)):
            continue
        w = su.Center - P
        off = (w - ax * w.dot(ax)).Length
        if off > 0.35:
            continue
        pt = f.Vertexes[0].Point if f.Vertexes else f.CenterOfMass
        n = f.normalAt(*f.Surface.parameter(pt))
        c = su.Center + ax * (pt - su.Center).dot(ax)
        if n.dot(c - pt) <= 0:                            # convex (a pin), not a hole
            continue
        zs = [(v.Point - P).dot(A) for v in f.Vertexes] or [0.0]
        ext = max(zs) - min(zs)
        if best is None or ext > best[0]:
            best = (ext, V(ax) if ax.dot(A) > 0 else V(ax) * -1, V(su.Center), su.Radius, it['label'])
    if best is None:
        return None
    ext, ax, c, r, lab = best
    Pn = c + ax * (P - c).dot(ax)
    shift = (Pn - P).Length
    ang = math.degrees(ax.getAngle(A))
    j['P'], j['A'] = Pn, ax
    return dict(bore_dia=round(2 * r, 3), bore_part=lab, lateral_shift=round(shift, 3), tilt_deg=round(ang, 3))

def _plane_faces(world, items, P, A, zlo, zhi, facing, r_ring):
    """Planar faces normal to A whose outward normal is facing*A, lying under the ring (r_ring) around the axis;
    returns sorted [(z, label)]."""
    out = []
    faces, _ = _faces_near(world, items, P, A, zlo, zhi, r_ring + 0.3)
    for it, f in faces:
        if f.Surface.TypeId != 'Part::GeomPlane':
            continue
        n = f.normalAt(*f.Surface.parameter(f.CenterOfMass))
        if n.dot(A) * facing < 0.999:
            continue
        z = (f.Surface.Position - P).dot(A)
        if not (zlo <= z <= zhi):
            continue
        ring = Part.Circle(P + A * z, A, r_ring).toShape()
        if f.distToShape(ring)[0] < 1e-6:
            out.append((z, it['label']))            # exact: the head/nut must land on the face, not 1e-4 into it
    uniq = {}
    for z, lab in out:
        uniq.setdefault((round(z, 6), lab), (z, lab))
    return sorted(uniq.values())

def _flats_x(world, items, j, af, z0, z1):
    """Normal of a pocket flat (axis-parallel plane at af/2 +- 0.35 from the axis) around the nut, or None."""
    P, A = j['P'], j['A']
    faces, _ = _faces_near(world, items, P, A, z0, z1, af / 2 * 1.2 + 0.5)
    for it, f in faces:
        if f.Surface.TypeId != 'Part::GeomPlane':
            continue
        n = V(f.Surface.Axis)
        if abs(n.dot(A)) > 0.01:
            continue
        dist = abs((f.Surface.Position - P).dot(n))
        if abs(dist - af / 2) > 0.35:
            continue
        zs = [(v.Point - P).dot(A) for v in f.Vertexes]
        if max(zs) < z0 or min(zs) > z1:
            continue
        x = n - A * n.dot(A)
        x.normalize()
        return x
    return None

def _nut_h(j):
    return NUTS[j['nut']]['h'] if j['nut'] else 0.0

def _washers(j):
    w = WASHERS[j['size']]['h']
    return (w if j['washer'] in ('head', 'both') else 0.0), (w if j['washer'] in ('nut', 'both') else 0.0)

def resolve(j, world):
    """Snap to the modelled hole, locate the head seat and the nut face, orient the nut flats."""
    sp = SHCS[j['size']]
    info = {}
    items = world.near(_joint_bb(j, sp), 1.0)
    j['_items'] = items
    if j['snap']:
        s = _snap_axis(world, items, j, sp)
        if s:
            info['snap'] = s
        wh, _ = _washers(j)
        rb = (sp['d'] / 2 + C + sp['dk'] / 2) / 2          # mid radius of the head bearing ring
        seats = [z - wh for z, lab in _plane_faces(world, items, j['P'], j['A'], -0.8, 0.8, -1, rb)]
        if seats:
            z = min(seats, key=abs)
            if abs(z) > 1e-3:
                j['P'] = j['P'] + j['A'] * z
                if j['zn'] is not None:
                    j['zn'] -= z
                info['seat_shift'] = round(z, 3)
    if j['nut'] and j['zn'] is None:
        j['zn'] = _infer_nut(world, items, j, sp)
        info['nut_face'] = 'inferred' if j['zn'] is not None else 'not found'
    if j['nut'] and NUTS[j['nut']]['kind'] in ('hex', 'nyloc') and j['zn'] is not None:
        n = NUTS[j['nut']]
        _, wn = _washers(j)
        x = _flats_x(world, items, j, n['s'] + 2 * C, j['zn'] + wn, j['zn'] + wn + n['h'])
        if x is not None and j['X'] is None:
            j['X'] = x
            info['nut_flats'] = 'aligned to pocket'
    j['resolve'] = info
    return j

def _joint_bb(j, sp):
    r = max(sp['dk'], 5.2) / 2 + 1.0
    env = _local(cyl((0, 0, -sp['k'] - 0.5), (0, 0, j['L'] + 1.0), r), _placement(j['P'], j['A']))
    return env.BoundBox

def _infer_nut(world, items, j, sp):
    n = NUTS[j['nut']]
    _, wn = _washers(j)
    shapes = [(world.shape(it), world.faces(it)) for it in items]
    pl = _placement(j['P'], j['A'], j['X'])
    if n['kind'] == 'insert':
        # the insert bore: coaxial concave cylinder of about the insert hole diameter; top = its end nearest the head
        faces, _ = _faces_near(world, items, j['P'], j['A'], 0.1, j['L'], n['hole'] / 2 + 0.5)
        tops = []
        for it, f in faces:
            su = f.Surface
            if su.TypeId == 'Part::GeomCylinder' and abs(su.Radius - n['hole'] / 2) < 0.25 and abs(V(su.Axis).dot(j['A'])) > 0.999:
                tops.append(min((v.Point - j['P']).dot(j['A']) for v in f.Vertexes))
        return min(tops) if tops else None
    rn = (sp['d'] / 2 + C + n['s'] / 2) / 2
    for z, lab in _plane_faces(world, items, j['P'], j['A'], 0.3, j['L'] - 0.3, +1, rn):
        x = _flats_x(world, items, j, n['s'] + 2 * C, z + wn, z + wn + n['h']) or j['X']   # test hex aligned to the pocket
        env = _local(_hexenv(n['s'] + 2 * C - 2 * TOL, z + wn + TOL, z + wn + n['h'] - TOL), _placement(j['P'], j['A'], x))
        if not any(env.BoundBox.intersect(s.BoundBox) and overlap(env, s, volume=False, b_faces=fs)[0] for s, fs in shapes):
            return z
    return None

# ------------------------------------------------------------------------------------------------ validate
def _engagement(j):
    """Thread engagement numbers and verdicts for the chosen length."""
    sp = SHCS[j['size']]
    P_ = sp['P']
    L = j['L']
    res = dict(pitch=P_)
    if not j['nut']:
        res['verdict'] = ('WARN', 'no nut: the screw must thread into a pilot hole (%s: dia %.1f-%.1f) over >= %.1f mm; '
                          'not checkable from the model' % (j['size'], sp['d'] * 0.8, sp['d'] * 0.85, 2 * sp['d']))
        return res
    n = NUTS[j['nut']]
    if j['zn'] is None:
        res['verdict'] = ('FAIL', 'no nut seat found along the screw (give "nut_offset" or model the nut pocket)')
        return res
    _, wn = _washers(j)
    face = j['zn'] + wn
    if n['kind'] == 'insert':
        eng = min(L, face + n['h']) - face
        res.update(insert_top=round(face, 3), engaged=round(eng, 3), tip_below_insert=round(L - face - n['h'], 3))
        need = min(n['h'], 1.5 * sp['d'])
        res['verdict'] = ('OK', '') if eng >= need - 1e-6 else ('FAIL', 'only %.2f mm engaged in the insert (need %.1f)' % (eng, need))
        res['required_min'] = need
        std_ok = [l for l in LENGTHS[j['size']] if min(l, face + n['h']) - face >= need]
        res['shortest_ok'] = min(std_ok) if std_ok else None
        return res
    h = n['h']
    p = L - (face + h)
    res.update(nut_face=round(face, 3), nut_top=round(face + h, 3), protrusion=round(p, 3), threads_past=round(p / P_, 2))
    if n['kind'] == 'nyloc':
        p_worst = L - (face + n['h_max'])
        res.update(protrusion_worst=round(p_worst, 3), threads_past_worst=round(p_worst / P_, 2))
        need = 2 * P_
        if p < 1.5 * P_:
            res['verdict'] = ('FAIL', 'tip only %.2f mm (%.1f threads) past the nyloc top: nylon not fully engaged (need >= 2)' % (p, p / P_))
        elif p < need or p_worst < 1.5 * P_:
            res['verdict'] = ('WARN', 'tip %.2f mm past the nyloc (%.1f threads; %.1f with a 3.0 mm tall nut)' % (p, p / P_, p_worst / P_))
        else:
            res['verdict'] = ('OK', '')
    else:
        need = 1 * P_
        if p < 0:
            res['verdict'] = ('FAIL', 'partial engagement: tip %.2f mm short of the nut top (%.0f%% of the nut engaged)' % (-p, 100 * max(0, h + p) / h))
        elif p < need:
            res['verdict'] = ('WARN', 'tip only %.2f mm past the nut (< 1 thread)' % p)
        else:
            res['verdict'] = ('OK', '')
    res['required_min'] = round(need, 3)
    std_ok = [l for l in LENGTHS[j['size']] if l - (face + h) >= need - 1e-9]
    res['shortest_ok'] = min(std_ok) if std_ok else None
    if res['shortest_ok'] is not None and res['shortest_ok'] < L and p > need + 2.0:
        res['verdict'] = ('WARN', 'screw %.1f mm longer than needed (tip %.2f past the nut); %s would do'
                          % (L - res['shortest_ok'], p, screw_type(j['size'], res['shortest_ok'])))
    return res

def validate(j, world):
    """Geometric checks of one joint against the placed parts; fills j['checks'], j['problems'], j['status']."""
    sp = SHCS[j['size']]
    d, dk, k, L = sp['d'], sp['dk'], sp['k'], j['L']
    pl = _placement(j['P'], j['A'], j['X'])
    items = j.get('_items') or world.near(_joint_bb(j, sp), 1.0)
    shapes = [(it['label'], world.shape(it), world.faces(it)) for it in items]
    wh, wn = _washers(j)
    n = NUTS[j['nut']] if j['nut'] else None
    zn = j['zn']
    face = (zn + wn) if zn is not None else None
    probs, chk = [], {}

    def clash(name, env_local, fail_msg, sev='FAIL'):
        env = _local(env_local, pl)
        hits = []
        for lab, s, fs in shapes:
            if not env.BoundBox.intersect(s.BoundBox):
                continue
            o, v, dist = overlap(env, s, b_faces=fs)
            if o:
                hits.append('%s (%.3f mm3)' % (lab, v) if v >= 0 else lab)
        chk[name] = 'clear' if not hits else hits
        if hits:
            probs.append((sev, fail_msg % ', '.join(hits)))

    def present(name, env_local):
        env = _local(env_local, pl)
        hits = [lab for lab, s, fs in shapes if env.BoundBox.intersect(s.BoundBox) and overlap(env, s, volume=False, b_faces=fs)[0]]
        chk[name] = hits
        return hits

    r_sh = d / 2 + C - TOL
    grip_end = face if face is not None else (L if not n else None)
    # head space: counterbore >= dk + 0.4 (or open), nothing above the seat
    clash('head_space', cyl((0, 0, -k), (0, 0, -TOL), dk / 2 + C - TOL),
          'head space: less than %.1f mm dia / %.1f mm free around the head, blocked by %%s' % (dk + 2 * C, k))
    if wh:
        clash('head_washer_space', cyl((0, 0, TOL), (0, 0, wh - TOL), WASHERS[j['size']]['d2'] / 2 + C - TOL),
              'washer under the head collides with %s')
    # clearance hole through the clamped stack
    if grip_end is not None and grip_end > wh + 2 * TOL:
        clash('hole', cyl((0, 0, wh + TOL), (0, 0, grip_end - TOL), r_sh),
              'clearance hole: less than %.1f mm dia (0.2 mm/side) along the screw in %%s' % (d + 2 * C))
    # head seat: material under the bearing ring
    seat = present('head_seat', _tube(d / 2 + C + 0.05, dk / 2 - 0.15, wh + 0.02, wh + 0.3))
    if not seat:
        probs.append(('FAIL', 'head not seated: no material under the head bearing ring'))
    # counterbore depth = nearest surface facing the head just outside the counterbore
    ups = [z for z, lab in _plane_faces(world, items, j['P'], j['A'], -k - 3, -0.05, -1, dk / 2 + C + 0.4)]
    depth = -max(ups) if ups else 0.0
    chk['counterbore_depth'] = round(depth, 3)
    chk['head_top_vs_surface'] = round(k - depth, 3)          # > 0: head stands proud
    if depth > 0.05 and k - depth > 0.05:
        probs.append(('WARN', 'head stands %.2f mm proud of the surrounding surface (counterbore %.2f deep, head %.1f)'
                      % (k - depth, depth, k)))
    # nut / insert
    if n and face is not None:
        if n['kind'] in ('hex', 'nyloc'):
            clash('nut_space', _hexenv(n['s'] + 2 * C - 2 * TOL, face + TOL, face + n['h'] - TOL),
                  'nut pocket/space: less than %.1f AF (0.2 mm/side) or blocked, by %%s' % (n['s'] + 2 * C))
            nseat = present('nut_seat', _tube(d / 2 + C + 0.05, n['s'] / 2 - 0.15, zn - 0.3, zn - 0.02))
            if not nseat:
                probs.append(('FAIL', 'nut not seated: no material under the nut bearing face'))
            if wn:
                clash('nut_washer_space', cyl((0, 0, zn + TOL), (0, 0, face - TOL), WASHERS[j['size']]['d2'] / 2 + C - TOL),
                      'washer under the nut collides with %s')
            tip0 = face + n['h'] + TOL
        else:                                             # heat-set insert: pilot hole present, wall around it
            clash('insert_hole', cyl((0, 0, face + TOL), (0, 0, face + n['h'] + 0.5), n['hole'] / 2 - TOL),
                  'insert hole: less than %.1f mm dia x %.1f deep in %%s' % (n['hole'], n['h'] + 0.5))
            wall = present('insert_wall', _tube(n['od'] / 2 + 0.05, n['od'] / 2 + 1.0, face + 0.3, face + n['h'] - 0.3))
            if not wall:
                probs.append(('FAIL', 'no material around the heat-set insert'))
            tip0 = face + n['h'] + TOL
        if L + C > tip0:
            clash('tip_space', cyl((0, 0, tip0), (0, 0, L + C), r_sh),
                  'screw tip (beyond the nut) collides with / has < 0.2 mm around it: %s')
    # (nut given but no seat found: reported once, by the engagement check below)
    # which parts does the screw pass through (the joint must join something)
    if grip_end is not None and grip_end > 0.2:
        grip = present('grip_parts', _tube(d / 2 + C + 0.05, d / 2 + C + 0.8, 0.05, grip_end - 0.05))
        if len(set(grip)) < 2:
            probs.append(('WARN', 'the screw passes through %s only: it joins nothing' % (', '.join(grip) or 'no part')))
    eng = _engagement(j)
    chk['engagement'] = eng
    if eng['verdict'][0] != 'OK':
        probs.append(eng['verdict'])
    j['checks'] = chk
    j['problems'] = probs
    j['status'] = 'fail' if any(p[0] == 'FAIL' for p in probs) else ('warn' if probs else 'ok')
    return j

def hole_exists(j, world):
    """Cheap test for V5 sites: a through-path (dia 0.8 core) along the grip, room for head and nut, and at least
    one part around the screw. Returns (ok, reasons)."""
    sp = SHCS[j['size']]
    items = j.get('_items') or world.near(_joint_bb(j, sp), 1.0)
    shapes = [(it['label'], world.shape(it), world.faces(it)) for it in items]
    pl = _placement(j['P'], j['A'], j['X'])
    zend = j['zn'] if j['zn'] is not None else j['L']
    tests = [('screw path', cyl((0, 0, 0.05), (0, 0, max(zend - 0.05, 0.1)), 0.4)),
             ('head space', cyl((0, 0, -sp['k'] + 0.1), (0, 0, -0.1), sp['dk'] / 2 - 0.25))]
    if j['zn'] is not None:
        n = NUTS[j['nut']]
        tests.append(('nut space', _hexenv(n['s'] - 0.4, j['zn'] + 0.1, j['zn'] + n['h'] - 0.1)))
    blocked = []
    for what, env in tests:
        e = _local(env, pl)
        for lab, s, fs in shapes:
            if e.BoundBox.intersect(s.BoundBox) and overlap(e, s, volume=False, b_faces=fs)[0]:
                blocked.append('%s blocked by %s' % (what, lab))
    tube = _local(_tube(sp['d'] / 2 + C + 0.05, sp['d'] / 2 + C + 0.8, 0.05, max(zend - 0.05, 0.1)), pl)
    if not any(tube.BoundBox.intersect(s.BoundBox) and overlap(tube, s, volume=False, b_faces=fs)[0] for lab, s, fs in shapes):
        blocked.append('no part around the screw (the V5 host is not in this model)')
    return (not blocked), blocked

# ------------------------------------------------------------------------------------------------ plan
V5_SUGGEST = {
    'bracket': 'cut a %.1f mm through hole + %.1f mm counterbore in the strap (bone side already has a dia 2.5 bore '
               'and an M2 nut pocket); or drop the site' % (2.0 + 2 * C, 3.8 + 2 * C),
    'driver': 'driver boards move in V6: request the board screws in regions/fasteners_electronics.json',
    'battery': 'battery case is redesigned in V6: request its lid screws in a fasteners_*.json',
    'fingertip': 'no anchor bore / nut slot in the simulacra distal phalanx: request M1.6 anchors when the tendon design is fixed',
    'thumb': 'thumb is being redesigned: request its screws in regions/fasteners_thumb.json',
    'cover_back': 'no through holes in cover_back (Shell1 keeps 2 partial M1.6 pockets): request if the shells keep screws',
}

def plan(doc, validate_geometry=True, log=print):
    """All joints with resolution + validation, the V5 open items and the request-file status."""
    world = World(doc) if doc is not None else None
    joints = j0_joints(doc)
    open_items = []
    for s in v5_sites():
        j = _joint('V5_' + s['id'], 'v5', s['size'], s['L'], s['nut'], s['P'], s['A'], zn=None, snap=True,
                   note='V5 %s (%s)' % (s['what'], s['v5']), v5=s['v5'], category=s['category'], owner=s['owner'])
        if world is None:
            ok = s['category'] == 'bracket' and 'pointerfinger' in s['hosts'] and 'thumb_hinge' not in s['hosts']
            why = ['no document: static decision']
            j['zn'] = s['zn']
        else:
            resolve(j, world)                    # snap to the bore, find the seat, infer the nut face
            ok, why = hole_exists(j, world)
            if s['zn'] is not None:
                zv5 = s['zn'] - j['resolve'].get('seat_shift', 0.0)
                j['zn_v5'] = round(zv5, 3)
                if j['zn'] is None or abs(j['zn'] - zv5) > 1.0:
                    ok = False
                    why.append('the V5 nut seat (%.2f mm from the head seat) does not exist in this model%s'
                               % (zv5, '' if j['zn'] is None else ' (nearest free seat at %.2f mm)' % j['zn']))
        if ok:
            joints.append(j)
        else:
            open_items.append(dict(site=j['id'], category=s['category'], v5_instance=s['v5'], what=s['what'],
                                   fastener='%s + %s' % (screw_type(s['size'], s['L']), s['nut']),
                                   seat_world=[round(c, 3) for c in s['P']], axis_world=[round(c, 4) for c in s['A']],
                                   reason='; '.join(why), suggestion=V5_SUGGEST.get(s['category'], ''), owner=s['owner']))
    rj, files = request_joints(doc)
    joints += rj
    if world is not None and validate_geometry:
        for j in joints:
            if 'resolve' not in j:
                resolve(j, world)
    joints, dropped = dedupe(joints)
    for j, by in dropped:
        if j['source'] == 'v5':
            open_items.append(dict(site=j['id'], category=j.get('category'), v5_instance=j.get('v5'), what=j['note'],
                                   fastener='%s + %s' % (screw_type(j['size'], j['L']), j['nut']),
                                   reason='superseded by %s (same hole)' % by, suggestion='', owner=j.get('owner')))
        else:
            for f in files:
                if 'request' in j and f['file'] == j['request']['file']:
                    f['rejected'].append('entry %d (%s): duplicates %s, not placed' % (j['request']['index'], j['id'], by))
    kept = {}
    for j in joints:
        if 'request' in j:
            kept.setdefault(j['request']['file'], set()).add(j['request']['index'])
    for f in files:
        f['placed'] = len(kept.get(f['file'], ()))
    if world is not None and validate_geometry:
        for j in joints:
            validate(j, world)
    return joints, open_items, files, world

def dedupe(joints):
    """Drop joints that sit in the same hole as a higher-priority one (j0 > request > v5; first request wins)."""
    rank = {'j0': 0, 'request': 1, 'v5': 2}
    order = sorted(range(len(joints)), key=lambda i: (rank.get(joints[i]['source'], 3), i))
    keep, dropped = [], []
    for i in order:
        j = joints[i]
        sp = SHCS[j['size']]
        dup = None
        for k in keep:
            if math.degrees(k['A'].getAngle(j['A'])) > 2.0:
                continue
            w = j['P'] - k['P']
            if (w - k['A'] * w.dot(k['A'])).Length > 0.5:
                continue
            z = w.dot(k['A'])                                  # j's seat along k's axis
            if z - sp['k'] < k['L'] and z + j['L'] > -SHCS[k['size']]['k']:
                dup = k['id']
                break
        if dup:
            dropped.append((j, dup))
        else:
            keep.append(j)
    keep.sort(key=lambda j: joints.index(j))
    return keep, dropped

def bom(joints):
    count = {}
    for j in joints:
        for t in [screw_type(j['size'], j['L'])] + ([j['nut']] if j['nut'] else []) + \
                 ['%s washer' % j['size']] * (2 if j['washer'] == 'both' else (1 if j['washer'] else 0)):
            count[t] = count.get(t, 0) + 1
    return dict(sorted(count.items()))

def _jsonable(j):
    out = dict(id=j['id'], source=j['source'], screw=screw_type(j['size'], j['L']), nut=j['nut'], washer=j['washer'],
               status=j['status'], seat_world=[round(c, 4) for c in j['P']], axis_world=[round(c, 5) for c in j['A']],
               nut_offset=round(j['zn'], 4) if j['zn'] is not None else None, note=j['note'],
               problems=['%s: %s' % p for p in j['problems']], resolve=j.get('resolve', {}))
    for k in ('v5', 'category', 'finger', 'request', 'host_instance'):
        if k in j:
            out[k] = j[k] if k != 'request' else dict(file=j[k]['file'], index=j[k]['index'])
    chk = {}
    for k, v in j['checks'].items():
        chk[k] = v
    out['checks'] = chk
    return out

def write_report(joints, open_items, files, doc, stage='build', extra=None, elapsed=None):
    probs = [dict(id=j['id'], severity=s, message=m) for j in joints for s, m in j['problems']]
    rep = dict(generated=datetime.datetime.now().isoformat(timespec='seconds'), stage=stage, module=VERSION,
               model=(doc.FileName or doc.Name) if doc is not None else None,
               summary=dict(joints=len(joints), ok=sum(j['status'] == 'ok' for j in joints),
                            warn=sum(j['status'] == 'warn' for j in joints), fail=sum(j['status'] == 'fail' for j in joints),
                            unvalidated=sum(j['status'] == 'unvalidated' for j in joints),
                            open_items=len(open_items), request_files=len(files)),
               problems=probs, requests=files, open_items=open_items, bom=bom(joints),
               joints=[_jsonable(j) for j in joints])
    if elapsed is not None:
        rep['elapsed_s'] = round(elapsed, 1)
    if extra:
        rep.update(extra)
    path = _report_path()
    tmp = path + '.tmp'
    with open(tmp, 'w') as fh:
        json.dump(rep, fh, indent=1)
    os.replace(tmp, path)
    return rep

def claimed_regions(joints, doc=None, key_access=10.0):
    """Space this module needs kept clear (regions/hardware.json format). J0 in each palm bone's local frame:
    the fastener envelope (counterbore, hole, pocket, nut and tip standing out of the back lug) and the
    straight hex-key approach on the palm side. Other placed V5 joints: the same in world coordinates."""
    regs = []
    loc = j0_local()
    for j in joints:
        sp = SHCS[j['size']]
        n = NUTS.get(j['nut']) if j['nut'] else None
        rn = (n['s'] / math.sqrt(3) if n and 's' in n else (n['od'] / 2 if n else 0)) + C + 0.1
        r = max(sp['dk'] / 2 + C, rn)
        if j['source'] == 'j0':
            ax, ys = loc['seat'][0], loc['seat'][1]
            y_out = -7.5                                   # palm-side face of the palm lug
            tip = ys + j['L'] + C
            regs.append(dict(part='palm_' + j['finger'], frame='local', box=[round(ax - r, 2), y_out, round(-r, 2), round(ax + r, 2), round(tip, 2), round(r, 2)],
                             what='J0 %s M2x16 SHCS + M2 nyloc: counterbore, hole, nut pocket, and nut + screw tip standing %.1f mm out of the back lug (+0.2); keep clear'
                             % (j['finger'], tip - 7.5)))
            ra = sp['dk'] / 2 + C
            regs.append(dict(part='palm_' + j['finger'], frame='local', box=[round(ax - ra, 2), y_out - key_access, round(-ra, 2), round(ax + ra, 2), y_out, round(ra, 2)],
                             what='J0 %s: straight 1.5 mm hex-key approach to the cap screw from the palm side (%.0f mm); keep open' % (j['finger'], key_access)))
        elif j['source'] == 'v5' and j['status'] != 'unvalidated':
            what = '%s %s + %s (%s)' % (j['id'], screw_type(j['size'], j['L']), j['nut'], j.get('v5', ''))
            pl = _placement(j['P'], j['A'], j['X'])
            parts = [('fastener: head, hole, nut%s, tip (+0.2); keep clear' % (' pocket' if j['nut'] else ''),
                      cyl((0, 0, -sp['k']), (0, 0, j['L'] + C), r)),
                     ('straight %.0f mm hex-key approach; keep open' % key_access,
                      cyl((0, 0, -sp['k'] - key_access), (0, 0, -sp['k']), sp['dk'] / 2 + C))]
            # palm-bone hosted joints are claimed in that bone's local frame (the frame the pillars work in)
            host = j.get('resolve', {}).get('snap', {}).get('bore_part')
            key = next((('palm_' + f) for f, lab in PALM_LABELS.items() if lab == host), None)
            link = doc.getObjectsByLabel(host)[0] if (key and doc is not None and doc.getObjectsByLabel(host)) else None
            for text, solid in parts:
                env = _local(solid, pl)
                if link is not None:
                    env = _local(solid, link.Placement.inverse().multiply(pl))
                b = env.BoundBox
                regs.append(dict(part=key if link is not None else 'world', frame='local' if link is not None else 'world',
                                 box=[round(b.XMin, 2), round(b.YMin, 2), round(b.ZMin, 2), round(b.XMax, 2), round(b.YMax, 2), round(b.ZMax, 2)],
                                 what='%s %s' % (what, text)))
    return regs

def _world_boxes(reg, doc):
    """World AABBs of a region (one per instance of its part)."""
    if reg.get('frame') == 'world' or reg.get('part') == 'world':
        return [list(reg['box'])]
    out = []
    for lk in _links_for(doc, {'part': reg.get('part')}):
        b = App.BoundBox(*reg['box']).transformed(lk.Placement.toMatrix())
        out.append([b.XMin, b.YMin, b.ZMin, b.XMax, b.YMax, b.ZMax])
    return out

def region_conflicts(mine, doc, regions_dir=REGIONS):
    """Overlaps between our claimed boxes and every other agent's regions/*.json. Same part + local frame:
    exact box test; anything else: world AABBs (conservative for rotated parts, marked approx)."""
    def hit(a, c):
        return all(a[i] < c[i + 3] and c[i] < a[i + 3] for i in range(3))
    out = []
    mine_w = [(m, _world_boxes(m, doc)) for m in mine]
    for f in sorted(glob.glob(regions_dir + '/*.json')):
        b = os.path.basename(f)
        if b.startswith('hardware') or b.startswith('fasteners_'):
            continue
        try:
            other = json.load(open(f))
        except Exception as e:
            out.append(dict(file=b, error='unreadable: %s' % e))
            continue
        if isinstance(other, dict):
            other = other.get('regions', [other])
        for o in other if isinstance(other, list) else []:
            try:
                ow = None
                for m, mw in mine_w:
                    if m['frame'] == 'local' and o.get('frame') == 'local':
                        if m['part'] != o.get('part'):
                            continue
                        if hit(m['box'], o['box']):
                            out.append(dict(file=b, exact=True, part=m['part'], ours=m['what'][:100], theirs=str(o.get('what'))[:140],
                                            ours_box=m['box'], theirs_box=o['box']))
                        continue
                    if ow is None:
                        ow = _world_boxes(o, doc)
                    if any(hit(x, y) for x in mw for y in ow):
                        out.append(dict(file=b, exact=False, part='world', ours=m['what'][:100], theirs=str(o.get('what'))[:140],
                                        ours_box=m['box'], theirs_box=o['box']))
            except Exception:
                continue
    return out

# ------------------------------------------------------------------------------------------------ interface
def needed_types():
    """Every fastener type the library must contain: base set + J0 + V5 + all current requests (file-based,
    so new_parts() and instances() agree without the geometry)."""
    ts = set(BASE_LIBRARY) | {'M2x16 SHCS', 'M2 nyloc'}
    for s in v5_sites():
        ts.add(screw_type(s['size'], s['L']))
        ts.add(s['nut'])
    for f, status, entries in _read_requests():
        for e in entries:
            try:
                size, L = parse_screw(e['type'])
                ts.add(screw_type(size, L))
                nut = parse_nut(e.get('nut'), size)
                if nut:
                    ts.add(nut)
                if e.get('washer'):
                    ts.add('%s washer' % size)
            except Exception:
                pass
    return sorted(ts)

def new_parts():
    out = {}
    for t in needed_types():
        key, shape, how = build_type(t)
        out[key] = shape
    return out

_CACHE = globals().get('_CACHE', {})      # survives importlib.reload(): build() and write_manifest() share it

def _specs(joints, doc):
    have = None
    if doc is not None:
        have = {o.Name[3:] for o in doc.Objects if o.Name.startswith('V6_hardware_')}
    specs = []
    for j in joints:
        pl = _placement(j['P'], j['A'], j['X'])
        fail = j['status'] == 'fail'
        wh, wn = _washers(j)
        items = [('screw', screw_type(j['size'], j['L']), 0.0)]
        if wh:
            items.append(('washer_head', '%s washer' % j['size'], -0.0))
        if j['nut'] and j['zn'] is not None:
            if wn:
                items.append(('washer_nut', '%s washer' % j['size'], j['zn']))
            items.append(('nut', j['nut'], j['zn'] + wn))
        for role, t, z in items:
            key = key_for(t)
            if have is not None and key not in have:
                j['problems'].append(('FAIL', '%s not in this build\'s library (request changed during the build); rebuild' % t))
                continue
            if role == 'washer_head':
                place = _placement(j['P'], j['A'], j['X'])       # washer seat frame: bearing face = head seat
            else:
                place = pl.multiply(App.Placement(V(0, 0, z), App.Rotation()))
            kind = 'screw' if role == 'screw' else ('insert' if t.endswith('insert') else ('washer' if 'washer' in role else 'nut'))
            specs.append(dict(name='HW_%s_%s' % (_safe(j['id']), role), part=key, placement=place, group=GROUP,
                              color=COLORS['fail'] if fail else COLORS[kind]))
    return specs

def instances():
    t0 = time.time()
    doc = find_doc()
    validate_geometry = os.environ.get('V6_HW_VALIDATE', '1') != '0'
    key = None
    if doc is not None:
        world = World(doc)
        reqs = []
        for f in sorted(glob.glob(_request_glob())):
            try:
                reqs.append((f, os.path.getmtime(f), open(f).read()))
            except Exception:
                reqs.append((f, 0, ''))
        key = hashlib.sha1((VERSION + world.fingerprint() + repr(reqs) + str(validate_geometry)).encode()).hexdigest()
    if key is not None and key in _CACHE:
        joints, open_items, files = _CACHE[key]
    else:
        joints, open_items, files, _ = plan(doc, validate_geometry)
        for j in joints:
            j.pop('_items', None)
        if key is not None:
            _CACHE.clear()
            _CACHE[key] = (joints, open_items, files)
    specs = _specs(joints, doc)
    stage = 'build' if (doc is not None and validate_geometry) else 'build (geometry not validated)'
    write_report(joints, open_items, files, doc, stage=stage, elapsed=time.time() - t0)
    return specs
