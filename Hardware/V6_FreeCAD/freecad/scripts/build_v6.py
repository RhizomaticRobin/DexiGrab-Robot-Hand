"""Rebuild every V6 part from its V5 source by composing the modification modules, headless or in the GUI.

Usage (headless):  freecadcmd build_v6.py  -- see ARGS below (environment variables, since freecadcmd eats argv):
  V6_MODULES = comma list, e.g. "j0,pillars"          (order = composition order; default: ORDER below)
  V6_IN      = input FCStd  (default freecad/DexiGrab_V6.FCStd)
  V6_OUT     = output FCStd (default: overwrite input when running in the GUI, required headless)
  V6_BREP    = directory to also write <key>.brep of every built part (optional)
Module interface (freecad/lib/mod_<name>.py):
  modify(key, shape) -> shape          change existing parts (return the shape unchanged if not yours)
  new_parts() -> {key: shape}          new parts in their own local frames (keys must be unique, prefix with module name)
  instances() -> [ {name, part, placement(App.Placement), group, color?, transparency?} ]  new placed instances
  relink() -> {instance_label: part_key}   point existing instances at a different part (e.g. thumb-only variants)
"""
import sys, os, importlib, time
LIB = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib'
sys.path.insert(0, LIB)
import FreeCAD as App, Part
import partkeys
importlib.reload(partkeys)
ORDER = ['j0', 'pillars', 'thumb', 'actuation', 'shells', 'electronics', 'fixes', 'tactile', 'proportions', 'splay', 'hardware']
ROOT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad'

def load_modules(names):
    mods = []
    for n in names:
        if not os.path.exists(os.path.join(LIB, 'mod_%s.py' % n)):
            continue
        m = importlib.import_module('mod_' + n)
        importlib.reload(m)
        mods.append((n, m))
    return mods

_LAST_MODS = None
_TOUCHED = set()

def _baked(shape):
    """Shape with its own placement folded into the geometry. A part whose shape carries a placement (e.g. made with
    rotate()) gets that placement as its object Placement, and App::Link (LinkTransform False) ignores it: the part
    would be displayed without its rotation. Rigid transform copy, safe on tori (unlike transformGeometry)."""
    if shape.Placement.isIdentity():
        return shape
    m = shape.Placement.toMatrix()
    c = shape.copy()
    c.Placement = App.Placement()
    c.transformShape(m, True)
    return c

def build(doc, names, brep_dir=None, log=print):
    global _LAST_MODS
    mods = load_modules(names)
    _LAST_MODS = mods
    _TOUCHED.clear()
    grp = doc.getObject('V6_parts') or doc.addObject('App::DocumentObjectGroup', 'V6_parts')
    src = partkeys.resolve(doc)
    # module-major composition: each module modifies every part that exists at that point, including
    # new parts from earlier modules (e.g. actuation adds tendon channels to mod_thumb's thumb-only bones)
    shapes = {k: s.Shape.copy() for k, s in src.items()}
    changed = {k: [] for k in shapes}
    origin = {k: s for k, s in src.items()}
    for n, m in mods:
        if hasattr(m, 'modify'):
            for key in list(shapes):
                t0 = time.time()
                try:
                    new = m.modify(key, shapes[key])
                except Exception as e:
                    dump = os.path.join(ROOT, 'work', 'failed_%s_%s.brep' % (n, key))
                    shapes[key].exportBrep(dump)
                    log('!! %s.modify(%s) FAILED: %s  (input saved to %s)' % (n, key, e, dump))
                    raise
                if new is not None and not new.isSame(shapes[key]):
                    changed[key].append('%s(%.1fs)' % (n, time.time() - t0))
                    shapes[key] = new
        if hasattr(m, 'new_parts'):
            for key, shape in m.new_parts().items():
                shapes[key] = shape
                changed[key] = [n]
                origin[key] = None
    built = {k: (_baked(shapes[k]) if changed[k] else shapes[k], changed[k], origin[k]) for k in shapes}
    # write V6 objects and re-point links
    objs = {}
    for key, (shape, changed, s) in built.items():
        if not changed and s is not None:
            objs[key] = s               # untouched part: instances keep pointing at the V5 source
            continue
        name = 'V6_' + key
        o = doc.getObject(name) or doc.addObject('Part::Feature', name)
        o.Shape = shape
        o.Label = 'V6 ' + key
        o.Label2 = ' + '.join(changed)
        if o not in grp.Group:
            grp.addObject(o)
        o.Visibility = False
        objs[key] = o
        if brep_dir:
            shape.exportBrep(os.path.join(brep_dir, key + '.brep'))
        log('%-18s vol %9.2f  valid %s  solids %d  <- %s' % (key, shape.Volume, shape.isValid(), len(shape.Solids), ' + '.join(changed)))
    by_src = {s.Name: objs[k] for k, (_, _, s) in built.items() if s is not None}
    for o in doc.Objects:
        if o.TypeId == 'App::Link' and o.LinkedObject is not None:
            cur = o.LinkedObject.Name
            base = cur
            if cur.startswith('V6_'):   # find the source this V6 object came from
                k = cur[3:]
                base = src[k].Name if k in src else None
            if base in by_src and o.LinkedObject is not by_src[base]:
                o.LinkedObject = by_src[base]
    made = set()
    for n, m in mods:
        if hasattr(m, 'relink'):
            for label, key in m.relink().items():
                for o in doc.getObjectsByLabel(label):
                    o.LinkedObject = objs[key]
        if hasattr(m, 'placements'):      # modules may re-pose existing instances (e.g. the thumb opposition re-clock)
            for label, pl in m.placements().items():
                for o in doc.getObjectsByLabel(label):
                    o.Placement = pl
                    _TOUCHED.add(label)
        if hasattr(m, 'instances'):
            for spec in m.instances():
                made.add('I_' + spec['name'])
                g = doc.getObject(spec['group']) or doc.addObject('App::DocumentObjectGroup', spec['group'])
                hand = doc.getObject('Hand')
                if hand and g not in hand.Group:
                    hand.addObject(g)
                lname = 'I_' + spec['name']
                lnk = doc.getObject(lname) or doc.addObject('App::Link', lname)
                lnk.Label = spec['name']
                lnk.LinkedObject = objs[spec['part']]
                lnk.Placement = spec['placement']
                if lnk not in g.Group:
                    g.addObject(lnk)
        if hasattr(m, 'placements_rel'):  # world transform pre-multiplied onto the current placement (composes with earlier moves)
            for label, T in m.placements_rel().items():
                for o in doc.getObjectsByLabel(label):
                    o.Placement = T.multiply(o.Placement)
                    _TOUCHED.add(label)
    _prune_instances(doc, made)
    doc.recompute()
    return built

def _prune_instances(doc, keep):
    """Remove module-created instance links (I_*) that the current module set no longer produces."""
    for o in list(doc.Objects):
        if o.TypeId == 'App::Link' and o.Name.startswith('I_') and o.Name not in keep:
            doc.removeObject(o.Name)

def write_manifest(built, doc, out_dir, names):
    """Headless side: dump built shapes as BREP + a manifest the GUI loader can apply quickly."""
    import json
    os.makedirs(out_dir, exist_ok=True)
    man = {'modules': names, 'parts': {}, 'instances': [], 'relink': {}}
    for key, (shape, changed, s) in built.items():
        if not changed and s is not None:
            continue
        path = os.path.join(out_dir, key + '.brep')
        shape.exportBrep(path)
        man['parts'][key] = {'brep': path, 'changed': changed, 'source': s.Name if s is not None else None,
                             'volume': shape.Volume, 'valid': shape.isValid()}
    man['placements'] = {}
    # reuse the module objects from build(): reloading would wipe state captured in modify() (thumb/proportions)
    for n, m in (_LAST_MODS if _LAST_MODS is not None else load_modules(names)):
        if hasattr(m, 'relink'):
            man['relink'].update(m.relink())
        if hasattr(m, 'placements'):
            man['placements'].update({lab: list(pl.toMatrix().A) for lab, pl in m.placements().items()})
        if hasattr(m, 'instances'):
            for spec in m.instances():
                d = dict(spec)
                d['placement'] = list(spec['placement'].toMatrix().A)
                man['instances'].append(d)
    for label in sorted(_TOUCHED):       # final (composed) placements override the per-module values
        for o in doc.getObjectsByLabel(label):
            man['placements'][label] = list(o.Placement.toMatrix().A)
    json.dump(man, open(os.path.join(out_dir, 'manifest.json'), 'w'), indent=1)
    return man

def load_manifest(doc, out_dir, log=print):
    """GUI side: create/update V6_<key> objects from BREPs, re-point links, add instances (fast)."""
    import json
    man = json.load(open(os.path.join(out_dir, 'manifest.json')))
    grp = doc.getObject('V6_parts') or doc.addObject('App::DocumentObjectGroup', 'V6_parts')
    src = partkeys.resolve(doc)
    objs = {k: s for k, s in src.items()}
    for key, info in man['parts'].items():
        name = 'V6_' + key
        o = doc.getObject(name) or doc.addObject('Part::Feature', name)
        sh = Part.Shape(); sh.read(info['brep'])
        sh = _baked(sh)
        o.Shape = sh
        o.Label = 'V6 ' + key
        o.Label2 = ' + '.join(info['changed'])
        if o not in grp.Group:
            grp.addObject(o)
        o.Visibility = False
        objs[key] = o
        log('%-18s vol %9.2f  <- %s' % (key, sh.Volume, ' + '.join(info['changed'])))
    src_to_obj = {s.Name: objs[k] for k, s in src.items()}
    for o in doc.Objects:
        if o.TypeId == 'App::Link' and o.LinkedObject is not None:
            cur = o.LinkedObject.Name
            base = cur
            if cur.startswith('V6_'):
                k = cur[3:]
                base = src[k].Name if k in src else None
            if base in src_to_obj and o.LinkedObject is not src_to_obj[base]:
                o.LinkedObject = src_to_obj[base]
    hand = doc.getObject('Hand')
    for spec in man['instances']:
        g = doc.getObject(spec['group']) or doc.addObject('App::DocumentObjectGroup', spec['group'])
        if hand and g not in hand.Group:
            hand.addObject(g)
        lname = 'I_' + spec['name']
        lnk = doc.getObject(lname) or doc.addObject('App::Link', lname)
        lnk.Label = spec['name']
        lnk.LinkedObject = objs[spec['part']]
        lnk.Placement = App.Placement(App.Matrix(*spec['placement']))
        if lnk not in g.Group:
            g.addObject(lnk)
        if spec.get('color') and lnk.ViewObject:
            try:
                lnk.ViewObject.OverrideMaterial = True
                lnk.ViewObject.ShapeMaterial.DiffuseColor = tuple(spec['color'])
            except Exception:
                pass
    # relinks and placements AFTER the instances, so later modules (e.g. proportions) win over instance specs
    for label, key in man['relink'].items():
        for o in doc.getObjectsByLabel(label):
            o.LinkedObject = objs[key]
    for label, mat in man.get('placements', {}).items():
        for o in doc.getObjectsByLabel(label):
            o.Placement = App.Placement(App.Matrix(*mat))
    _prune_instances(doc, {'I_' + spec['name'] for spec in man['instances']})
    doc.recompute()
    return man

if __name__ == '__main__' or os.environ.get('V6_RUN'):
    names = [n for n in os.environ.get('V6_MODULES', ','.join(ORDER)).split(',') if n]
    inp = os.environ.get('V6_IN', os.path.join(ROOT, 'DexiGrab_V6.FCStd'))
    out = os.environ.get('V6_OUT')
    doc = App.openDocument(inp)
    built = build(doc, names, None)
    if os.environ.get('V6_BREP'):
        write_manifest(built, doc, os.environ['V6_BREP'], names)
        print('manifest written to', os.environ['V6_BREP'])
    if out:
        doc.saveAs(out)
        print('saved', out)
