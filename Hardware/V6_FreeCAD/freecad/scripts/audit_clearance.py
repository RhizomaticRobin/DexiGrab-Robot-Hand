"""Global interference / clearance audit of the placed model (headless).

env: V6_IN (FCStd, default DexiGrab_V6.FCStd), V6_AUDIT_OUT (csv path), V6_GAP (default 0.2)
Reports every pair of placed instances whose bounding boxes come within GAP: overlap volume (interference),
touching (distance 0, no volume), or gap < GAP.
"""
import os, sys, csv, time, itertools
sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
import FreeCAD as App, Part
GAP = float(os.environ.get('V6_GAP', '0.2'))
inp = os.environ.get('V6_IN', '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/DexiGrab_V6.FCStd')
out = os.environ.get('V6_AUDIT_OUT', '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/work/audit.csv')
doc = App.openDocument(inp)
items = []
for o in doc.Objects:
    if o.TypeId != 'App::Link' or o.LinkedObject is None:
        continue
    s = o.LinkedObject.Shape.copy()
    if not s.Solids:          # mesh-only (Shell2 as imported) - skip, reported separately
        print('skip (no solid):', o.Label)
        continue
    s.Placement = o.Placement.multiply(s.Placement)
    grp = next((g.Label for g in o.InList if g.TypeId == 'App::DocumentObjectGroup'), '')
    items.append((o.Label, grp, s))
print('instances', len(items))
rows = []
t0 = time.time()
for (la, ga, a), (lb, gb, b) in itertools.combinations(items, 2):
    ba, bb = a.BoundBox, b.BoundBox
    if (ba.XMin > bb.XMax + GAP or bb.XMin > ba.XMax + GAP or ba.YMin > bb.YMax + GAP or bb.YMin > ba.YMax + GAP
            or ba.ZMin > bb.ZMax + GAP or bb.ZMin > ba.ZMax + GAP):
        continue
    d = a.distToShape(b)[0]
    if d >= GAP:
        continue
    vol = 0.0
    if d < 1e-6:
        try:
            vol = a.common(b).Volume
        except Exception as e:
            vol = -1.0
    kind = 'INTERFERENCE' if vol > 0.01 else ('touch' if d < 1e-6 else 'gap<%.1f' % GAP)
    rows.append((kind, la, ga, lb, gb, round(d, 3), round(vol, 3)))
rows.sort(key=lambda r: (r[0] != 'INTERFERENCE', -r[6], r[5]))
with open(out, 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['kind', 'part_a', 'group_a', 'part_b', 'group_b', 'min_gap_mm', 'overlap_mm3']); w.writerows(rows)
print('pairs flagged', len(rows), 'in %.0fs' % (time.time() - t0))
for r in rows:
    print(r)
