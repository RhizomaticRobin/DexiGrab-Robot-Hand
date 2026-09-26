# Per-instance colours: neutral source parts, per-group link overrides (re-run after every load).
import FreeCAD as App
doc = App.getDocument('DexiGrab_V6')
NEUTRAL = (0.82, 0.82, 0.80)
COL = {'Index': NEUTRAL, 'Middle': NEUTRAL, 'Ring': NEUTRAL, 'Pinky': NEUTRAL,
       'Thumb': (0.55, 0.65, 0.82), 'Palm_rigid': (0.78, 0.78, 0.74), 'Wrist_segments': (0.72, 0.72, 0.66),
       'Palm_TPU': (1.0, 0.55, 0.1), 'Shells': (0.25, 0.25, 0.28), 'Motors': (0.80, 0.66, 0.25),
       'Hardware': (0.12, 0.12, 0.14), 'Electronics': (0.10, 0.45, 0.20), 'Wiring': (0.85, 0.15, 0.15),
       'Battery': (0.20, 0.35, 0.75), 'Tactile': (0.10, 0.70, 0.70), 'Actuation': (0.80, 0.66, 0.25)}
TRANSP = {'Palm_TPU': 0.3, 'Shells': 0.55}
for o in doc.Objects:                       # sources / V6 parts: neutral (they are hidden; links show them)
    if o.TypeId == 'Part::Feature' and o.ViewObject is not None:
        try:
            o.ViewObject.ShapeAppearance = [App.Material(DiffuseColor=NEUTRAL)]
        except Exception:
            pass
n, groups = 0, {}
hand = doc.getObject('Hand')
for g in hand.Group:
    for o in getattr(g, 'Group', []):
        if o.TypeId != 'App::Link' or o.ViewObject is None:
            continue
        c = COL.get(g.Name, (0.7, 0.7, 0.7))
        try:
            vo = o.ViewObject
            vo.OverrideMaterial = True
            m = vo.ShapeMaterial
            m.DiffuseColor = c
            m.Transparency = TRANSP.get(g.Name, 0.0)
            vo.ShapeMaterial = m
            n += 1
            groups[g.Name] = groups.get(g.Name, 0) + 1
        except Exception:
            pass
doc.recompute()
print('coloured', n, groups)
