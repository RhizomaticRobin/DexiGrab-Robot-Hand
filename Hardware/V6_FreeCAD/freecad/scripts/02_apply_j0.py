import sys, importlib
sys.path.insert(0, '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/lib')
import FreeCAD as App, FreeCADGui as Gui
import v6geom, mod_j0
importlib.reload(v6geom); importlib.reload(mod_j0)
doc = App.getDocument('DexiGrab_V6')
grp = doc.getObject('V6_parts') or doc.addObject('App::DocumentObjectGroup', 'V6_parts')

def v6part(name, shape, label):
    o = doc.getObject(name) or doc.addObject('Part::Feature', name)
    o.Shape = shape
    o.Label = label
    if o not in grp.Group:
        grp.addObject(o)
    o.Visibility = False
    return o

palm = {'SRC_Base_Bone_1_2_V02_pointerfinger_and_thumb_attachment': ('V6_PalmBone_index', 11032.74),
        'SRC_Base_Bone_1_2_V02_middlefinger': ('V6_PalmBone_middle', 12163.07),
        'SRC_Base_Bone_1_2_V02_ringfing': ('V6_PalmBone_ring', 12263.04),
        'SRC_Base_Bone_1_2_V02_pinky': ('V6_PalmBone_pinky', 10373.71)}
new = {}
for src, (name, onshape_vol) in palm.items():
    s = mod_j0.palm_bone(doc.getObject(src).Shape)
    new[src] = v6part(name, s, name.replace('V6_', 'V6 ') + ' (J0 M2)')
    print(name, round(s.Volume, 2), 'Onshape', onshape_vol, 'valid', s.isValid(), 'solids', len(s.Solids))
s = mod_j0.base_bone(doc.getObject('SRC_Base_Bone_1_V02').Shape)
new['SRC_Base_Bone_1_V02'] = v6part('V6_BaseBone1', s, 'V6 Base Bone 1 (J0 M2)')
print('V6_BaseBone1', round(s.Volume, 2), 'Onshape', 3064.41, 'valid', s.isValid(), 'solids', len(s.Solids))

# re-point every instance link from the V5 source part to its V6 part
n = 0
for o in doc.Objects:
    if o.TypeId == 'App::Link' and o.LinkedObject and o.LinkedObject.Name in new:
        o.LinkedObject = new[o.LinkedObject.Name]
        n += 1
doc.recompute()
doc.save()
print('links re-pointed', n)
