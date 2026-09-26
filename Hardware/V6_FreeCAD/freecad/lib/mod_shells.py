"""V6 shells: Shell2 as a valid B-rep, the Shell1<->Shell2 spherical joint, and the shell interference fixes.

Design frame D = Shell1's local frame. Shell1's skins are exact surfaces of revolution about the D y-axis,
which in the hand runs along the middle of the palm in its back plane, so the axis is the palm-cupping axis.
The user's requirement (README item 10) is read as follows:
  - the "palm shells" are Shell1 (dorsal bowl) and Shell2 (outer cap).
  - Shell1 + cover_back ride on the pinky side (cover2); Shell2 is driven in roll by the thumb side through
    a lip that straddles cover3's dorsal wall end.
  - when the palm cups, the thumb side rolls about the axis relative to the pinky side, so the shells roll
    against each other; palm twist adds small tilts.
Joint:
  - bearing: raised spherical land on Shell1 and a concave seat in Shell2, both centred on the axis at C, with
    0.2 mm running clearance.
  - retention and travel limits: 2 studs on Shell1 through arc slots in Shell2. The slot ends are the roll
    stops and the slot sides are the tilt stops.
  - elastic: on each stud, a slider washer, a silicone O-ring and a cap washer under an M2 screw. The squeezed
    O-ring holds the seat on the land.
See notes/shells.md.
"""
import os, json, math, hashlib
import numpy as np
import FreeCAD as App, Part

V = App.Vector
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'
WORK = os.path.join(ROOT, 'work', 'shells')
BAKE = os.path.join(WORK, 'bake')
CACHE = os.path.join(WORK, 'cache')
REBUILD_VERSION = 'v4-harmonic-fill'

# ---------------------------------------------------------------- design parameters (mm, deg)
CLR = 0.2                      # running clearance per side (README rule 7)
Z2 = 5.1187                    # V5 relationship: Shell2 = Shell1 frame shifted +5.1187 along D +z (world -Z)
C = (0.0, -60.0, 0.0)          # joint centre: on the axis, at Shell1's equator (outer R 66.50)
R_LAND = 70.0                  # Shell1 land sphere radius (land height 3.5 at the equator)
R_SEAT = R_LAND + CLR          # Shell2 seat sphere radius
W_LAND = 8.0                   # land half-width along y
W_SEAT = 10.0                  # seat band half-width along y (land + tilt travel)
TH_LAND = (-36.0, 38.0)        # land extent in theta
ROLL = (-1.0, 15.0)            # design roll range of Shell2 about the axis (deg, + = cupping, thumb side palm-ward)
TILT = 1.0                     # design tilt range about the other two axes through C (+-deg)
STUDS = [(-95.0, 5.0), (-20.0, 5.0)]   # (y, theta) of the retention studs at roll 0
STUD_R = 2.5                   # stud radius (dia 5)
SLOT_W = STUD_R + CLR + 1.3    # slot half-width (clearance + tilt excursion at the stud)
ORING_ID, ORING_CS = 5.0, 2.5   # silicone 50A O-ring on each stud
ORING_H = 0.9 * ORING_CS        # O-ring height at the highest seat point: ~10 % squeeze (gentle hold-down, low friction)
STUD_ABOVE = 1.5 + ORING_H      # stud top above the slider seat: slider washer 1.5 + squeezed O-ring
PILOT_R = 0.85                 # M2 thread-forming pilot (dia 1.7) in the stud
PILOT_DEPTH = 7.0
THUMB_TRIM = 2.8               # Shell1 thumb edge / cover_back thumb end pulled back (roll -1 deg + tilt +-1 deg)
LIP_Y = (-72.0, -30.0)         # Shell2 lip over cover3's dorsal wall end (clean plate edge: clear of the R6 fillet at y < -73.9)
LIP_T = 1.6                    # lip wall thickness
NT = (0.99144, -0.13053, 0.0)  # cover3 wall normal (outward, thumb side) in D
D_C3_IN, D_C3_OUT = 53.973, 58.473     # cover3 wall inner / outer face: NT . p
D_C3_BLOCK = 46.473                    # cover3 thumb-block inner face (wrist region, y < -85)
Y_NOTCH = -77.0                        # 3 mm past the end of cover3's R6 notch corner (block face -> wall face, y -80.3), for tilt
NP = (-0.99144, -0.13053, 0.0)  # cover2 wall normal (outward, pinky side) in D
D_C2_IN, D_C2_OUT = 51.0, 55.77      # cover2 wall inner / outer face: NP . p
Z_C2_END = 42.4752                    # cover2 dorsal wall end (local z -24.5) in D
C2_SCREWS_X = (-62.0, -22.0)   # cover2 local x of the Shell1<->cover2 screws (wall end z_local = -24.5)


def _placements():
    d = json.load(open(os.path.join(BAKE, 'placements.json')))
    return {k: App.Placement(V(*v[0]), App.Rotation(*v[1])) for k, v in d.items()}


def _lemon_out(y):
    """Shell1 outer skin radius at y (measured arc: centre (y -61.40, r -109.56), R 176.04)."""
    return -109.56 + math.sqrt(176.04 ** 2 - (y + 61.40) ** 2)


def _lemon_in(y):
    return -121.92 + math.sqrt(184.39 ** 2 - (y + 60.0) ** 2)


def _pol(r, th_deg, y):
    t = math.radians(th_deg)
    return V(r * math.sin(t), y, r * math.cos(t))


# ================================================================ Shell2 rebuild from the 3MF mesh
def _prep(P, T):
    T = np.asarray(T)[:, ::-1]
    P = np.asarray(P, float)
    v0, v1, v2 = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    n = np.cross(v1 - v0, v2 - v0)
    n /= np.linalg.norm(n, axis=1)[:, None]
    c = (v0 + v1 + v2) / 3
    rh = np.c_[c[:, 0], np.zeros(len(c)), c[:, 2]]
    rh /= np.linalg.norm(rh, axis=1)[:, None]
    d = (n * rh).sum(1)
    lab = np.where(d > 0.7, 1, np.where(d < -0.7, -1, 0))
    return {'P': P, 'T': T, 'lab': lab, 'th': np.arctan2(P[:, 0], P[:, 2]), 'r': np.hypot(P[:, 0], P[:, 2]), 'y': P[:, 1]}


def _loops(m, la, lb):
    T, lab = m['T'], m['lab']
    E = np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]])
    tid = np.tile(np.arange(len(T)), 3)
    key = np.sort(E, 1)
    o = np.lexsort((key[:, 1], key[:, 0]))
    ks, ts = key[o], tid[o]
    pairs, tp = ks.reshape(-1, 2, 2)[:, 0, :], ts.reshape(-1, 2)
    sel = ((lab[tp[:, 0]] == la) & (lab[tp[:, 1]] == lb)) | ((lab[tp[:, 0]] == lb) & (lab[tp[:, 1]] == la))
    adj = {}
    for a, b in pairs[sel]:
        adj.setdefault(int(a), []).append(int(b))
        adj.setdefault(int(b), []).append(int(a))
    seen, out = set(), []
    for s in list(adj):
        if s in seen:
            continue
        loop, prev, cur = [s], None, s
        seen.add(s)
        while True:
            nx = [v for v in adj[cur] if v != prev and v not in seen]
            if not nx:
                break
            prev, cur = cur, nx[0]
            loop.append(cur)
            seen.add(cur)
        out.append(loop)
    return sorted([l for l in out if len(l) >= 20], key=len, reverse=True)


def _basis(t, y, deg):
    u = (np.asarray(t, float) - 0.12) / 0.9
    w = (np.asarray(y, float) + 60.0) / 62.0
    cols = []
    for i in range(deg + 1):
        ci = np.polynomial.chebyshev.chebval(u, [0] * i + [1])
        for j in range(deg + 1 - i):
            cols.append(ci * np.polynomial.chebyshev.chebval(w, [0] * j + [1]))
    return np.array(cols).T


def _inside(poly, q, shrink=0.0):
    x, y = q[:, 0], q[:, 1]
    px, py = poly[:, 0], poly[:, 1]
    qx, qy = np.roll(px, -1), np.roll(py, -1)
    res = np.zeros(len(q), bool)
    dmin = np.full(len(q), np.inf)
    for ax, ay, bx, by in zip(px, py, qx, qy):
        cond = (ay > y) != (by > y)
        xint = ax + (y - ay) * (bx - ax) / np.where(by - ay == 0, 1e-12, by - ay)
        res ^= cond & (x < xint)
        if shrink:
            ex, ey = bx - ax, by - ay
            L2 = ex * ex + ey * ey + 1e-12
            tt = np.clip(((x - ax) * ex + (y - ay) * ey) / L2, 0, 1)
            dmin = np.minimum(dmin, np.hypot(x - ax - tt * ex, y - ay - tt * ey))
    return res & (dmin > shrink) if shrink else res


def _uv(m, loop):
    P = m['P'][loop]
    return np.c_[np.degrees(np.arctan2(P[:, 0], P[:, 2])), P[:, 1]]


def _fit(m, label, poly, holes, deg, shrink, extra=None, reject=None):
    idx = np.unique(m['T'][m['lab'] == label].ravel())
    q = np.c_[np.degrees(m['th'][idx]), m['y'][idx]]
    sel = _inside(poly, q, shrink)
    for h in holes:
        sel &= ~_inside(h, q, 0.0)
        sel &= ~_near(h, q, 0.8)
    if extra is not None:
        sel &= extra(idx)
    t, y, r = m['th'][idx][sel], m['y'][idx][sel], m['r'][idx][sel]
    keep = np.ones(len(t), bool)
    for _ in range(8):
        coef, *_ = np.linalg.lstsq(_basis(t[keep], y[keep], deg), r[keep], rcond=None)
        res = r - _basis(t, y, deg) @ coef
        if reject is None:
            break
        nk = np.abs(res - np.median(res[keep])) < reject
        if (nk == keep).all():
            break
        keep = nk
    res = r - _basis(t, y, deg) @ coef
    return coef, float(np.abs(res[keep]).max()), float(np.sqrt((res[keep] ** 2).mean())), int(keep.sum())


def _near(poly, q, dist):
    x, y = q[:, 0], q[:, 1]
    px, py = poly[:, 0], poly[:, 1]
    qx, qy = np.roll(px, -1), np.roll(py, -1)
    dmin = np.full(len(q), np.inf)
    for ax, ay, bx, by in zip(px, py, qx, qy):
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey + 1e-12
        tt = np.clip(((x - ax) * ex + (y - ay) * ey) / L2, 0, 1)
        dmin = np.minimum(dmin, np.hypot(x - ax - tt * ex, y - ay - tt * ey))
    return dmin < dist


def _resample_polar(pts, n, centre, k=1.15):
    """Resample a closed loop given as (theta_deg, y, r) rows at n equal polar angles around centre."""
    cx, cy = centre
    a = np.arctan2(pts[:, 1] - cy, (pts[:, 0] - cx) * k)
    o = np.argsort(a)
    a, p = a[o], pts[o]
    a3 = np.r_[a - 2 * np.pi, a, a + 2 * np.pi]
    p3 = np.r_[p, p, p]
    tgt = np.linspace(-np.pi, np.pi, n, endpoint=False)
    return np.c_[[np.interp(tgt, a3, p3[:, i]) for i in range(p.shape[1])]].T


def _simplify(poly, tol):
    """Douglas-Peucker on a closed polygon (k,2)."""
    def dp(pts):
        if len(pts) < 3:
            return pts
        a, b = pts[0], pts[-1]
        ab = b - a
        L = np.hypot(*ab) + 1e-12
        d = np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / L
        i = int(np.argmax(d))
        if d[i] > tol:
            return np.r_[dp(pts[:i + 1])[:-1], dp(pts[i:])]
        return np.r_[[a], [b]]
    i0 = int(np.argmax(poly[:, 0]))
    p = np.r_[poly[i0:], poly[:i0 + 1]]
    j = int(np.argmin(p[:, 0]))
    out = np.r_[dp(p[:j + 1])[:-1], dp(p[j:])[:-1]]
    return out


def _section_loft(f_lo, f_hi, th_range, y_range, n_th=28, dy=4.0):
    """Solid between two radial height fields r = f(theta_rad, y) over a (theta, y) box, by lofting y-sections."""
    th = np.radians(np.linspace(th_range[0], th_range[1], n_th))
    ys = np.arange(y_range[0], y_range[1] + 1e-6, dy)
    wires = []
    for y in ys:
        lo = f_lo(th, np.full_like(th, y))
        hi = f_hi(th, np.full_like(th, y))
        top = Part.BSplineCurve()
        top.interpolate([V(r * math.sin(t), y, r * math.cos(t)) for t, r in zip(th, hi)])
        bot = Part.BSplineCurve()
        bot.interpolate([V(r * math.sin(t), y, r * math.cos(t)) for t, r in zip(th[::-1], lo[::-1])])
        e1 = top.toShape()
        e2 = Part.LineSegment(top.EndPoint, bot.StartPoint).toShape()
        e3 = bot.toShape()
        e4 = Part.LineSegment(bot.EndPoint, top.StartPoint).toShape()
        wires.append(Part.Wire([e1, e2, e3, e4]))
    return Part.makeLoft(wires, True, False)


def _central_prism(loop_uv_r, h1=12.0, h2=120.0, smooth=True, directions=None):
    """Solid bounded by rulings through a closed loop, capped by the planes z=h1, z=h2 (planar caps).

    loop_uv_r: rows (theta_deg, y, r) of points on the loop; each ruling is radial through the point unless
    `directions` (k,3) gives the ruling direction (e.g. bottom->top rim points)."""
    A, B = [], []
    for i, (thd, y, r) in enumerate(loop_uv_r):
        p = _pol(r, thd, y)
        d = V(*directions[i]) if directions is not None else V(math.sin(math.radians(thd)), 0, math.cos(math.radians(thd)))
        s1 = (h1 - p.z) / d.z
        s2 = (h2 - p.z) / d.z
        A.append(p + d * s1)
        B.append(p + d * s2)
    if smooth:
        ca, cb = Part.BSplineCurve(), Part.BSplineCurve()
        ca.interpolate(A, PeriodicFlag=True)
        cb.interpolate(B, PeriodicFlag=True)
        wa, wb = Part.Wire([ca.toShape()]), Part.Wire([cb.toShape()])
    else:
        wa, wb = Part.makePolygon(A + [A[0]]), Part.makePolygon(B + [B[0]])
    return Part.makeLoft([wa, wb], True, True)


def _ray_heights(m, th_deg, ys):
    """Outermost radius where radial rays (from the y axis) hit the mesh; nan where they miss."""
    P, T = m['P'], m['T']
    v0, e1, e2 = P[T[:, 0]], P[T[:, 1]] - P[T[:, 0]], P[T[:, 2]] - P[T[:, 0]]
    out = np.full((len(ys), len(th_deg)), np.nan)
    tr = np.radians(th_deg)
    for j, y in enumerate(ys):
        o = np.array([0.0, y, 0.0])
        near = np.abs(P[T].mean(1)[:, 1] - y) < 6.0
        V0, E1, E2 = v0[near], e1[near], e2[near]
        for i, t in enumerate(tr):
            d = np.array([math.sin(t), 0.0, math.cos(t)])
            h = np.cross(d, E2)
            a = (E1 * h).sum(1)
            ok = np.abs(a) > 1e-12
            f = np.where(ok, 1.0 / np.where(ok, a, 1), 0)
            sv = o - V0
            u = f * (sv * h).sum(1)
            q = np.cross(sv, E1)
            v = f * (q @ d)
            tt = f * (E2 * q).sum(1)
            hit = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (tt > 0)
            if hit.any():
                out[j, i] = tt[hit].max()
    return out


def _grid_loft(R_lo, R_hi, th_deg, ys):
    """Solid between two sampled radial height fields (rows = y sections) by lofting y-sections."""
    tr = np.radians(th_deg)
    wires = []
    for j, y in enumerate(ys):
        top = Part.BSplineCurve()
        top.interpolate([V(r * math.sin(t), y, r * math.cos(t)) for t, r in zip(tr, R_hi[j])])
        bot = Part.BSplineCurve()
        bot.interpolate([V(r * math.sin(t), y, r * math.cos(t)) for t, r in zip(tr[::-1], R_lo[j][::-1])])
        wires.append(Part.Wire([top.toShape(), Part.LineSegment(top.EndPoint, bot.StartPoint).toShape(),
                                bot.toShape(), Part.LineSegment(bot.EndPoint, top.StartPoint).toShape()]))
    s = Part.makeLoft(wires, True, False)
    if not s.isValid():
        print('mod_shells: loft invalid before fix')
        s.fix(1e-6, 1e-6, 1e-5)
    return s


def _radial_prism(uv, h1=12.0, h2=140.0):
    """Prism of rays from the y axis through a closed (theta_deg, y) polygon, capped by planes z=h1, z=h2."""
    A = [V(h1 * math.tan(math.radians(t)), y, h1) for t, y in uv]
    B = [V(h2 * math.tan(math.radians(t)), y, h2) for t, y in uv]
    return Part.makeLoft([Part.makePolygon(A + [A[0]]), Part.makePolygon(B + [B[0]])], True, True)


def _smooth_radial_prism(loop_xyz, n=140, h1=12.0, h2=140.0):
    """Radial prism through a smooth closed loop (arc-length resampled, periodic B-spline, planar caps)."""
    B = np.asarray(loop_xyz)
    seg = np.linalg.norm(np.diff(np.r_[B, B[:1]], axis=0), axis=1)
    sl = np.r_[0, np.cumsum(seg)]
    tgt = np.linspace(0, sl[-1], n, endpoint=False)
    Bx = np.c_[[np.interp(tgt, sl, np.r_[B, B[:1]][:, k]) for k in range(3)]].T
    uv = np.c_[np.degrees(np.arctan2(Bx[:, 0], Bx[:, 2])), Bx[:, 1]]
    wires = []
    for h in (h1, h2):
        c = Part.BSplineCurve()
        c.interpolate([V(h * math.tan(math.radians(t)), y, h) for t, y in uv], PeriodicFlag=True)
        wires.append(Part.Wire([c.toShape()]))
    return Part.makeLoft(wires, True, True)


def rebuild_shell2(mesh_shape=None, log=None):
    """Clean B-rep of Shell2 in its native frame (= Shell1 design frame) from the 3MF mesh (cached).

    Outer skin incl. the edge chamfers: radial ray heights of the mesh on a (theta, y) grid (logo area:
    Chebyshev fit of the surrounding top skin); underside: Chebyshev fit (the underside is a surface of
    revolution to <0.001 mm); outline: radial prism through the underside rim loop; logo: recess floor fit,
    radial walls through the mesh's logo rim loop."""
    if mesh_shape is not None:
        pts, tris = mesh_shape.tessellate(0.1)
        P, T = np.array([[p.x, p.y, p.z] for p in pts]), np.array(tris)
    else:
        z = np.load(os.path.join(BAKE, 's2_mesh.npz'))
        P, T = z['P'], z['T']
    key = hashlib.sha1(np.round(P, 4).tobytes() + np.asarray(T).tobytes() + REBUILD_VERSION.encode()).hexdigest()[:12]
    os.makedirs(CACHE, exist_ok=True)
    cpath = os.path.join(CACHE, 'shell2_native_%s.brep' % key)
    if os.path.exists(cpath):
        s = Part.Shape()
        s.read(cpath)
        return s.Solids[0] if s.Solids else s
    m = _prep(P, T)
    Lt, Lb = _loops(m, 1, 0), _loops(m, -1, 0)
    plogo, pbot = _uv(m, Lt[1]), _uv(m, Lb[0])
    pisl = [_uv(m, l) for l in Lt[2:]]
    DT, DB, DF = 8, 6, 4
    ct, mt, rt, nt = _fit(m, 1, _uv(m, Lt[0]), [plogo] + pisl, DT, 0.8)
    cb, mb, rb, nb = _fit(m, -1, pbot, [], DB, 0.8)
    ftop = lambda t, y: _basis(t, y, DT) @ ct
    fbot = lambda t, y: _basis(t, y, DB) @ cb
    deep = lambda idx: (ftop(m['th'][idx], m['y'][idx]) - m['r'][idx]) > 0.6
    cf, mf, rf, nf = _fit(m, 1, plogo, pisl, DF, 0.5, extra=deep, reject=0.25)
    ffloor = lambda t, y: _basis(t, y, DF) @ cf
    # outer height field from the mesh itself
    th_deg = np.arange(-46.0, 60.01, 1.25)
    ys = np.arange(-122.0, 2.01, 2.5)
    H = _ray_heights(m, th_deg, ys)
    TT, YY = np.meshgrid(np.radians(th_deg), ys)
    q = np.c_[np.degrees(TT.ravel()), YY.ravel()]
    logo_zone = _inside(plogo, q, 0.0) | _near(plogo, q, 1.5)
    for pi in pisl:
        logo_zone |= _inside(pi, q, 0.0) | _near(pi, q, 1.5)
    Hf = H.ravel()
    Hf[logo_zone] = ftop(TT.ravel()[logo_zone], YY.ravel()[logo_zone])
    Rbot = fbot(TT.ravel(), YY.ravel())                  # underside: surface of revolution, extrapolates benignly
    # rays that miss the cap: smooth harmonic continuation of the measured outer skin (no step at the outline,
    # so the lofted skin does not ring); the outline prism removes this margin afterwards
    miss = ~np.isfinite(Hf)
    G = Hf.reshape(H.shape).copy()
    known = ~miss.reshape(H.shape)
    G[~known] = np.nanmean(G[known])
    for _ in range(400):
        Gp = np.pad(G, 1, mode='edge')
        avg = 0.25 * (Gp[:-2, 1:-1] + Gp[2:, 1:-1] + Gp[1:-1, :-2] + Gp[1:-1, 2:])
        G = np.where(known, G, avg)
    Hf = np.maximum(G.ravel(), Rbot + 0.3)
    R_hi = Hf.reshape(H.shape)
    R_lo = Rbot.reshape(H.shape)
    slab = _grid_loft(R_lo, R_hi, th_deg, ys)
    if log:
        log('shell2 slab loft valid %s vol %.0f' % (slab.isValid(), slab.Volume))
    # outline: radial prism through the underside rim loop (the rim walls are radial to within the chamfer)
    outline = _radial_prism(_simplify(pbot, 0.05))
    body = slab.common(outline)
    # logo recess
    ring = [_simplify(plogo, 0.08)] + [_simplify(pi, 0.08) for pi in pisl]
    lprism = _radial_prism(ring[0])
    for r_ in ring[1:]:
        lprism = lprism.cut(_radial_prism(r_))
    tb = (plogo[:, 0].min() - 3, plogo[:, 0].max() + 3)
    yb = (plogo[:, 1].min() - 3, plogo[:, 1].max() + 3)
    lth = np.arange(tb[0], tb[1] + 0.01, 1.5)
    lys = np.arange(yb[0], yb[1] + 0.01, 3.0)
    LT, LY = np.meshgrid(np.radians(lth), lys)
    FL = ffloor(LT.ravel(), LY.ravel()).reshape(LT.shape)
    TP = ftop(LT.ravel(), LY.ravel()).reshape(LT.shape) + 2.0
    recess = _grid_loft(FL, TP, lth, lys).common(lprism)
    s = body.cut(recess).removeSplitter()
    if not s.isValid():
        s.fix(1e-7, 1e-7, 1e-6)
    if len(s.Solids) == 1:
        s = s.Solids[0]
    if log:
        log('shell2 rebuild: top fit max %.3f rms %.3f | underside max %.3f | logo floor max %.3f rms %.3f | valid %s vol %.1f'
            % (mt, rt, mb, mf, rf, s.isValid(), s.Volume))
    if s.isValid() and s.Solids and s.Volume > 1000:
        s.exportBrep(cpath)
    return s


# ================================================================ helpers (design frame D)
def _load(path):
    sh = Part.Shape()
    sh.read(path)
    return sh


def _revband(r_lo, r_hi, y0, y1, n=40):
    """Solid of revolution about the D y-axis between radius functions r_lo(y) < r_hi(y), y0..y1."""
    ys = np.linspace(y0, y1, n)
    lo = Part.BSplineCurve()
    lo.interpolate([V(0, y, float(r_lo(y))) for y in ys])
    hi = Part.BSplineCurve()
    hi.interpolate([V(0, y, float(r_hi(y))) for y in ys[::-1]])
    w = Part.Wire([lo.toShape(), Part.LineSegment(lo.EndPoint, hi.StartPoint).toShape(), hi.toShape(),
                   Part.LineSegment(hi.EndPoint, lo.StartPoint).toShape()])
    return Part.Face(w).revolve(V(0, 0, 0), V(0, 1, 0), 360)


def _wedge(th1, th2, y0, y1, R=160.0):
    """Region between the half-planes through the y-axis at theta1 < theta2 (deg, from +z toward +x)."""
    pts = [V(0, y0, 0)]
    for t in np.linspace(th1, th2, 9):
        pts.append(V(R * math.sin(math.radians(t)), y0, R * math.cos(math.radians(t))))
    pts.append(V(0, y0, 0))
    return Part.Face(Part.makePolygon(pts)).extrude(V(0, y1 - y0, 0))


def _yslab(y0, y1, R=200.0):
    return Part.makeBox(2 * R, y1 - y0, 2 * R, V(-R, y0, -R))


def _halfspace(n, d, R=400.0):
    """Solid {p : n.p >= d} (n unit), as a big box."""
    n = V(*n)
    n.normalize()
    a = V(0, 0, 1) if abs(n.z) < 0.9 else V(1, 0, 0)
    u = a.cross(n)
    u.normalize()
    w = n.cross(u)
    box = Part.makeBox(R, 2 * R, 2 * R)
    box.Placement = App.Placement(n * d - u * R - w * R, App.Rotation(n, u, w, 'XYZ'))
    return box


def _grown(shape, dirs, amount=CLR):
    """Translated copies of a solid (original + one per direction); subtracting them all one by one
    approximates a Minkowski growth by `amount` (fusing near-coincident copies upsets OCC)."""
    tools = []                      # the shifted copies cover the original; cutting it too leaves coincident faces
    for dvec in dirs:
        c = shape.copy()
        v = V(*dvec)
        v.normalize()
        c.translate(v * amount)
        tools.append(c)
    return tools


def _cut_seq(s, tools):
    for t in tools:
        s = s.cut(t)
    return s


def _clean(sol):
    sol = sol.removeSplitter()
    if not sol.isValid():
        sol.fix(1e-7, 1e-7, 1e-6)
    if len(sol.Solids) == 1:
        sol = sol.Solids[0]
    return sol


_NATIVE = []


def _shell2_D():
    """Rebuilt Shell2 (cached) placed in the V5 relationship to Shell1 (design frame D)."""
    if not _NATIVE:
        _NATIVE.append(rebuild_shell2(None))
    s = _NATIVE[0].copy()
    s.translate(V(0, 0, Z2))
    return s


def _stud_axis(ys, ts):
    e = V(math.sin(math.radians(ts)), 0, math.cos(math.radians(ts)))
    return V(0, ys, 0), e


_TOPR = {}


def _shell2_top_r(ys, ts):
    """Height (along the stud axis, from the y axis) of the slider-washer seat at stud (ys, ts): the highest
    point of Shell2's top skin under the washer footprint (r 6.5) over the whole roll range."""
    k = (ys, ts)
    if k not in _TOPR:
        o, e = _stud_axis(ys, ts)
        u = V(0, 1, 0)
        w = e.cross(u)
        rt0 = max(math.hypot(v.X, v.Z) for v in _shell2_D().common(Part.makeLine(o + e * 55, o + e * 95)).Vertexes)
        piece = Part.makeCylinder(20.0, 16.0, o + e * (rt0 - 10.0), e)          # local neighbourhood of the stud
        base = _shell2_D().common(piece)
        best = 0.0
        poses = [(r, 0, 0) for r in (ROLL[0], 0.0, 0.5 * (ROLL[0] + ROLL[1]), ROLL[1])]
        poses += [(r, a, b) for r in (ROLL[0], 0.5 * (ROLL[0] + ROLL[1]), ROLL[1]) for a in (-TILT, TILT) for b in (-TILT, TILT)]
        for roll, pitch, yaw in poses:
            s2 = base.copy()
            s2.rotate(V(0, 0, 0), V(0, 1, 0), roll)
            if pitch:
                s2.rotate(V(*C), V(1, 0, 0), pitch)
            if yaw:
                s2.rotate(V(*C), V(0, 0, 1), yaw)
            for i in range(13):
                off = V() if i == 12 else (u * math.cos(i * math.pi / 6) + w * math.sin(i * math.pi / 6)) * 6.5
                c = s2.common(Part.makeLine(o + off + e * 55, o + off + e * 95))
                for vx in c.Vertexes:
                    best = max(best, (vx.Point - o - off).dot(e))
        _TOPR[k] = best + 0.1
    return _TOPR[k]


# ================================================================ Shell1 features
def _land():
    ball = Part.makeSphere(R_LAND, V(*C))
    band = ball.common(_yslab(C[1] - W_LAND, C[1] + W_LAND)).common(_wedge(TH_LAND[0], TH_LAND[1], -200, 200))
    shell = _revband(lambda y: _lemon_out(y) - 1.0, lambda y: R_LAND + 5.0, C[1] - 12, C[1] + 12)
    return band.common(shell)


def _studs():
    out, pilots = [], []
    for ys, ts in STUDS:
        o, e = _stud_axis(ys, ts)
        r0 = _lemon_out(ys) - 1.5
        r1 = _shell2_top_r(ys, ts) + STUD_ABOVE
        out.append(Part.makeCylinder(STUD_R, r1 - r0, o + e * r0, e))
        pilots.append(Part.makeCylinder(PILOT_R, PILOT_DEPTH + 0.5, o + e * (r1 - PILOT_DEPTH), e))
    return out, pilots


def _c2_screw_points():
    """Shell1<->cover2 screw axes in D: seat point on the cover2 wall-end centre line, axis -z (D)."""
    pl = _placements()
    M = pl['Shell1 <1>'].inverse().multiply(pl['driver_side_palm_cover2 <1>'])
    return [M.multVec(V(x, -2.25, -24.5)) for x in C2_SCREWS_X]


def shell1(shape):
    s = shape.fuse(_land())
    studs, pilots = _studs()
    s = s.fuse(studs)
    # pinky side: rebate over cover2's wall end, 0.2 mm all round (fixes the 2.2 mm interference)
    # cover2's wall is a plate between NP.p = 51.0 (inner, incl. small ribs) and 55.77 (outer, coplanar with Shell1's
    # pinky trim face); its dorsal end is flat at D z 42.47 (local z -24.5). Remove that slab grown by 0.2 below the end.
    wall = _halfspace(NP, D_C2_IN - CLR).cut(_halfspace(NP, D_C2_OUT + CLR))
    wall = wall.common(_halfspace((0, 0, -1), -(Z_C2_END + CLR))).common(_yslab(-107.5 - CLR, 5.0))
    cuts = [wall]
    # thumb side: edge pulled back THUMB_TRIM from cover3's wall and block faces
    cuts.append(_halfspace(NT, D_C3_IN - THUMB_TRIM).common(_yslab(Y_NOTCH, 20.0)))
    cuts.append(_halfspace(NT, D_C3_BLOCK - THUMB_TRIM).common(_yslab(-140.0, Y_NOTCH)))
    # stud pilots + Shell1<->cover2 screw holes (clearance 2.4, counterbore 4.4 to a flat seat 2.2 above the wall end)
    cuts += pilots
    for pnt in _c2_screw_points():
        cuts.append(Part.makeCylinder(1.2, 40, V(pnt.x, pnt.y, pnt.z - 5), V(0, 0, 1)))
        cuts.append(Part.makeCylinder(2.2, 40, V(pnt.x, pnt.y, pnt.z + CLR + 2.2), V(0, 0, 1)))
    s = s.cut(cuts)
    return _clean(s)


# ================================================================ Shell2 features
_K_WALL = [(-80.0, 9.459), (-78.0, 9.459), (-76.0, 8.686), (-74.0, 8.191), (-72.0, 8.037), (-70.0, 7.956),
           (-68.0, 7.882), (-66.0, 7.816), (-64.0, 7.756), (-62.0, 7.704), (-60.0, 7.659), (-58.0, 7.621),
           (-56.0, 7.592), (-54.0, 7.571), (-52.0, 7.558), (-50.0, 7.57), (-48.0, 7.606), (-46.0, 7.669),
           (-44.0, 7.758), (-42.0, 7.874), (-40.0, 7.991), (-38.0, 8.101), (-36.0, 8.204), (-34.0, 8.3),
           (-32.0, 8.387), (-30.0, 8.462), (-28.0, 8.53), (-26.0, 8.616), (-24.0, 8.616)]
# cover3 dorsal wall-end top above Shell1's outer skin (max over +-2 mm windows, measured every 1 mm)


def _wall_k(y):
    ys, ks = zip(*_K_WALL)
    return float(np.interp(y, ys, ks))


def _lip_block():
    y0, y1 = LIP_Y
    band = _revband(lambda y: _lemon_out(y) + 4.0, lambda y: _lemon_out(y) + _wall_k(y) + CLR + LIP_T, y0, y1)
    slab = _halfspace(NT, D_C3_IN - 2.0).cut(_halfspace(NT, D_C3_OUT + CLR + LIP_T))
    return band.common(slab).common(_halfspace((0, 0, 1), 10.0))      # dorsal side only (the band is 360 deg)


def _c3_clear(c=CLR + 0.05):
    """Analytic clearance tools around cover3 (thumb side), from its measured faces (not its B-rep):
    the dorsal wall plate NT.p 53.97..58.47 up to the wall-end envelope lemon(y)+k(y), and the thumb block
    (NT.p >= 46.47) for y < Y_NOTCH (all heights: Shell2 stops at the block face, no sliver over it)."""
    dorsal = _halfspace((0, 0, 1), 10.0)
    wall = _halfspace(NT, D_C3_IN - c).cut(_halfspace(NT, D_C3_OUT + c))
    wall = wall.common(_revband(lambda y: 1.0, lambda y: _lemon_out(y) + _wall_k(y) + c, Y_NOTCH - 3.0, -6.0))
    wall = wall.common(_yslab(Y_NOTCH - 3.0, -6.0)).common(dorsal)
    block = _halfspace(NT, D_C3_BLOCK - c).common(_yslab(-140.0, Y_NOTCH)).common(dorsal)
    return [wall, block]


def _slots():
    out = []
    for ys, ts in STUDS:
        # Shell2 rolling +phi carries the slot +phi, so the fixed stud travels -phi in Shell2's frame
        a0, a1 = ts - ROLL[1], ts - ROLL[0]
        body = _wedge(a0, a1, ys - SLOT_W, ys + SLOT_W).common(_revband(lambda y: 55.0, lambda y: 95.0, ys - 8, ys + 8))
        ends = []
        for a in (a0, a1):
            o, e = _stud_axis(ys, a)
            ends.append(Part.makeCylinder(SLOT_W, 40, o + e * 55, e))
        out.append(body.fuse(ends))
    return out


def shell2_D():
    """Final Shell2 in the design frame D (nominal pose)."""
    c = CLR + 0.05
    dorsal = _halfspace((0, 0, 1), 10.0)
    s = _shell2_D()
    # 1. thumb end: stop 0.25 short of cover3's wall inner face (y > Y_NOTCH) and block face (y < Y_NOTCH)
    s = s.cut([_halfspace(NT, D_C3_IN - c).common(_yslab(Y_NOTCH, 20.0)).common(dorsal),
               _halfspace(NT, D_C3_BLOCK - c).common(_yslab(-140.0, Y_NOTCH)).common(dorsal)])
    # 2. lip = U-cap straddling cover3's dorsal wall end (analytic solids only), fused on
    lip = _lip_block().cut(_c3_clear(c)[0])
    s = s.fuse(lip)
    # 3. spherical seat band and the stud slots
    seat = Part.makeSphere(R_SEAT, V(*C)).common(_yslab(C[1] - W_SEAT, C[1] + W_SEAT))
    s = s.cut([seat] + _slots())
    s = _clean(s)
    if len(s.Solids) > 1:
        s = max(s.Solids, key=lambda x: x.Volume)
    return s


# ================================================================ cover_back, cover2
M16_HOLES = [(-33.65, 43.26), (-11.01, 53.47)]   # Shell1's existing M1.6 bosses (D x, z), screw along +y


def cover_back_D(shape_D):
    s = shape_D.cut(_halfspace(NT, D_C3_BLOCK - THUMB_TRIM))
    holes = []
    for x, z in M16_HOLES:
        holes.append(Part.makeCylinder(0.8 + CLR, 12, V(x, -130, z), V(0, 1, 0)))
        holes.append(Part.makeCylinder(1.5 + CLR, 5.5 + 1.2, V(x, -130, z), V(0, 1, 0)))   # head seat 1.2 deep
    return _clean(s.cut(holes))


def cover2_D(shape_D):
    pil = []
    for pnt in _c2_screw_points():
        pil.append(Part.makeCylinder(0.85, 8.0 + 0.2, V(pnt.x, pnt.y, pnt.z - 8.0), V(0, 0, 1)))
    return _clean(shape_D.cut(pil))


# ================================================================ frames + module interface
def _to_local(shape_D, link_label):
    """D-frame geometry -> part-local frame of the part shown by link_label (baked into the geometry)."""
    pl = _placements()
    M = pl[link_label].inverse().multiply(pl['Shell1 <1>'])
    s = shape_D.copy()
    s.transformShape(M.toMatrix(), True)
    return s


def _to_D(shape_local, link_label):
    pl = _placements()
    M = pl['Shell1 <1>'].inverse().multiply(pl[link_label])
    s = shape_local.copy()
    s.transformShape(M.toMatrix(), True)
    return s


def modify(key, shape):
    if key == 'shell1':
        return shell1(shape)                     # Shell1's local frame is D
    if key == 'shell2':
        rebuild_shell2(shape)                    # warms the cache from the actual input mesh
        return _to_local(shell2_D(), 'Shell2 <1>')
    if key == 'cover_back':
        return _to_local(cover_back_D(_to_D(shape, 'cover_back <1>')), 'cover_back <1>')
    if key == 'cover2':
        return _to_local(cover2_D(_to_D(shape, 'driver_side_palm_cover2 <1>')), 'driver_side_palm_cover2 <1>')
    return shape


# retention stack parts (D frame at the nominal pose), instanced with Shell1's placement
def _stack(ys, ts):
    o, e = _stud_axis(ys, ts)
    rt = _shell2_top_r(ys, ts)
    slider = Part.makeCylinder(6.5, 1.5, o + e * rt, e).cut(Part.makeCylinder(STUD_R + CLR, 3, o + e * (rt - 1), e))
    # modelled 0.05 mm short of both washers: a torus tangent to a plane upsets OCC booleans (audit false positive)
    oring = Part.makeTorus(ORING_ID / 2 + ORING_CS / 2, ORING_H / 2 - 0.05, o + e * (rt + 1.5 + ORING_H / 2), e)
    capw = Part.makeCylinder(6.0, 1.2, o + e * (rt + STUD_ABOVE), e).cut(Part.makeCylinder(1.0 + CLR, 3, o + e * (rt + STUD_ABOVE - 1), e))
    return slider, oring, capw


def new_parts():
    out = {}
    for i, (ys, ts) in enumerate(STUDS):
        sl, orr, cw = _stack(ys, ts)
        out['shells_slider_%d' % (i + 1)] = sl
        out['shells_oring_%d' % (i + 1)] = orr
        out['shells_capwasher_%d' % (i + 1)] = cw
    return out


def instances():
    p1 = _placements()['Shell1 <1>']
    col = {'slider': (0.85, 0.85, 0.8), 'oring': (0.08, 0.08, 0.08), 'capwasher': (0.85, 0.85, 0.8)}
    out = []
    for i in range(len(STUDS)):
        for kind in ('slider', 'oring', 'capwasher'):
            out.append({'name': 'shells %s %d' % (kind, i + 1), 'part': 'shells_%s_%d' % (kind, i + 1),
                        'placement': App.Placement(p1), 'group': 'Shells', 'color': col[kind]})
    return out
