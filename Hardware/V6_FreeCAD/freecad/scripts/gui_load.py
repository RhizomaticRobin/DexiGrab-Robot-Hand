# Load the headless build (build/manifest.json + BREPs) into the live GUI document.
import sys, importlib
for p in ('/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib', '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/scripts'):
    if p not in sys.path: sys.path.insert(0, p)
import FreeCAD as App, FreeCADGui as Gui
import partkeys, build_v6
importlib.reload(partkeys); importlib.reload(build_v6)
doc = App.getDocument('DexiGrab_V6')
# retire first-pass objects (from 02_apply_j0.py) by pointing their links back at the V5 sources
OLD = {'V6_PalmBone_index': 'palm_index', 'V6_PalmBone_middle': 'palm_middle', 'V6_PalmBone_ring': 'palm_ring',
       'V6_PalmBone_pinky': 'palm_pinky', 'V6_BaseBone1': 'base_bone'}
src = partkeys.resolve(doc)
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.LinkedObject is not None and o.LinkedObject.Name in OLD:
        o.LinkedObject = src[OLD[o.LinkedObject.Name]]
for n in OLD:
    if doc.getObject(n):
        doc.removeObject(n)
build_v6.load_manifest(doc, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/build')
doc.save()
print('V6 objects:', sorted(o.Name for o in doc.Objects if o.Name.startswith('V6_') and o.TypeId == 'Part::Feature'))
