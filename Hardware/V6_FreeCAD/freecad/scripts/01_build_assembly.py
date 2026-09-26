# Build the DexiGrab V6 FreeCAD document from the Onshape STEP export (part-local frames)
# and the exact occurrence transforms from the Onshape assembly definition.
import FreeCAD as App, FreeCADGui as Gui, Part, json, os, re
EXP = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/v5_step'
PFX = 'Robotic Hand_V5_simulacra - '
OUT = '/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad/DexiGrab_V6.FCStd'
defn = json.load(open('/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/data/top_assembly_definition.json'))
ra = defn['rootAssembly']
inst = {i['id']: i for i in ra['instances']}
for s in defn['subAssemblies']:
    for i in s['instances']:
        inst[i['id']] = i

if 'DexiGrab_V6' in App.listDocuments():
    App.closeDocument('DexiGrab_V6')
doc = App.newDocument('DexiGrab_V6')
src_grp = doc.addObject('App::DocumentObjectGroup', 'V5_source_parts')
hand = doc.addObject('App::DocumentObjectGroup', 'Hand')
groups = {}
def group(name):
    if name not in groups:
        g = doc.addObject('App::DocumentObjectGroup', name)
        hand.addObject(g)
        groups[name] = g
    return groups[name]

FINGER = {'Base Bone 1_V02 <1>': 'Index', 'Metacarpal Bone_V02 <1>': 'Index', 'Proximal Phalanx Bone_V02 <1>': 'Index', 'Distal Phalanx Bone_V02 <1>': 'Index',
          'Base Bone 1_V02 <2>': 'Middle', 'Metacarpal Bone_V02 <2>': 'Middle', 'Proximal Phalanx Bone_V02_middlefinger <1>': 'Middle', 'Distal Phalanx Bone_V02 <2>': 'Middle',
          'Base Bone 1_V02 <4>': 'Ring', 'Metacarpal Bone_V02 <3>': 'Ring', 'Proximal Phalanx Bone_V02 <2>': 'Ring', 'Distal Phalanx Bone_V02 <3>': 'Ring',
          'Base Bone 1_V02 <3>': 'Pinky', 'Metacarpal Bone_V02_pinky <1>': 'Pinky', 'Proximal Phalanx Bone_V02_pinky <1>': 'Pinky', 'Distal Phalanx Bone_V02 <4>': 'Pinky',
          'Metacarpal Bone_V02 <4>': 'Thumb', 'Proximal Phalanx Bone_V02 <3>': 'Thumb', 'Distal Phalanx Bone_V02 <5>': 'Thumb',
          'first_thumb_hinge <1>': 'Thumb', 'second_thumb_hinge <1>': 'Thumb', 'third_thumb_hinge <1>': 'Thumb', 'hand_pulley_wheel_thumb <1>': 'Thumb'}
TPU = ('Palm1', 'Palm2', 'Palm2_2', 'Palm3_2', 'Palm_pinky', 'palm_bone_flex')
def category(name):
    if name in FINGER: return FINGER[name]
    base = name.rsplit(' <', 1)[0]
    if base in TPU: return 'Palm_TPU'
    if base.startswith('Shell') or base == 'cover_back': return 'Shells'
    if base.startswith('Palm_bone1'): return 'Wrist_segments'
    return 'Palm_rigid'

def safe(n): return re.sub(r'[^A-Za-z0-9_]', '_', n)

# one source object per unique part (document + partId); Palm_bone1 segments share a name, so load all 5 files
files = {os.path.basename(f)[len(PFX):-5]: os.path.join(EXP, f) for f in os.listdir(EXP) if f.endswith('.step')}
src = {}
def source_for(occ_leaf, part_key):
    if part_key in src: return src[part_key]
    base = occ_leaf['name'].rsplit(' <', 1)[0]
    cands = sorted(k for k in files if k == base or re.fullmatch(re.escape(base) + r' \(\d+\)', k))
    if base == 'Palm_bone1':   # 5 different parts with the same name and the same (in-place) transform
        used = {o.Label2 for o in src.values() if o is not None}
        cands = [k for k in cands if k not in used]
    shape = Part.read(files[cands[0]])
    o = doc.addObject('Part::Feature', 'SRC_' + safe(base))
    o.Shape = shape
    o.Label2 = cands[0]
    src_grp.addObject(o)
    o.Visibility = False
    src[part_key] = o
    return o

seen = set()
log = []
for occ in ra['occurrences']:
    leaf = inst[occ['path'][-1]]
    if leaf['type'] != 'Part':
        continue
    key = (leaf['documentId'], leaf['partId'], leaf['name'].rsplit(' <', 1)[0])
    t = occ['transform']
    if leaf['name'].startswith('palm_bone_flex') and 'palm_bone_flex' in seen:
        log.append('skipped duplicate ' + leaf['name']); continue
    seen.add(leaf['name'].rsplit(' <', 1)[0])
    s = source_for(leaf, key)
    if s.Shape.isValid() and s.Shape.Solids and s.Shape.Volume < 1.0:
        log.append('skipped junk sliver ' + leaf['name'] + ' (' + s.Label2 + ')'); continue
    M = App.Matrix(t[0], t[1], t[2], t[3] * 1000, t[4], t[5], t[6], t[7] * 1000, t[8], t[9], t[10], t[11] * 1000, 0, 0, 0, 1)
    lnk = doc.addObject('App::Link', safe(leaf['name']))
    lnk.Label = leaf['name']
    lnk.LinkedObject = s
    lnk.Placement = App.Placement(M)
    group(category(leaf['name'])).addObject(lnk)

doc.recompute()
# colours: PLA light grey, thumb blue-grey, TPU orange (translucent), shells dark
COL = {'Palm_TPU': (1.0, 0.55, 0.1), 'Shells': (0.25, 0.25, 0.28), 'Thumb': (0.55, 0.65, 0.8), 'Wrist_segments': (0.75, 0.75, 0.7)}
for gname, g in groups.items():
    for o in g.Group:
        vo = o.ViewObject
        if vo is None: continue
        c = COL.get(gname, (0.82, 0.82, 0.8))
        try:
            vo.ShapeAppearance = [App.Material(DiffuseColor=c)]
        except Exception:
            try: vo.ShapeColor = c
            except Exception: pass
        if gname == 'Palm_TPU':
            vo.Transparency = 30
        if gname == 'Shells':
            vo.Transparency = 60
doc.saveAs(OUT)
Gui.ActiveDocument.ActiveView.viewIsometric()
Gui.SendMsgToActiveView('ViewFit')
print('parts', len(src), 'links', sum(len(g.Group) for g in groups.values()))
print('\n'.join(log))
for k, o in sorted(src.items(), key=lambda kv: kv[1].Name):
    print(o.Name, o.Label2, round(o.Shape.Volume, 2), o.Shape.isValid(), len(o.Shape.Solids))
