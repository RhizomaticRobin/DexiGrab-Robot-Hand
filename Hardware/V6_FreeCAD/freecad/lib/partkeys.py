"""Canonical part keys for the DexiGrab V6 FreeCAD model.

Every existing part has a key; modules modify parts by key. Instances (App::Link objects in the
document) point at one part each; several instances can share a part (e.g. 4x Base Bone 1).
"""
# key -> (V5 source object name in DexiGrab_V6.FCStd, description, material)
PARTS = {
    'palm_index':      ('SRC_Base_Bone_1_2_V02_pointerfinger_and_thumb_attachment', 'index palm bone (Onshape KFbB)', 'PLA'),
    'palm_middle':     ('SRC_Base_Bone_1_2_V02_middlefinger', 'middle palm bone (KFXB)', 'PLA'),
    'palm_ring':       ('SRC_Base_Bone_1_2_V02_ringfing', 'ring palm bone (KFPB)', 'PLA'),
    'palm_pinky':      ('SRC_Base_Bone_1_2_V02_pinky', 'pinky palm bone (KFTB)', 'PLA'),
    'base_bone':       ('SRC_Base_Bone_1_V02', 'Base Bone 1, J0/J1 knuckle, all 4 fingers (JFP)', 'PLA'),
    'metacarpal':      ('SRC_Metacarpal_Bone_V02', 'first phalanx, index/middle/ring AND thumb (JFH)', 'PLA'),
    'metacarpal_pinky': ('SRC_Metacarpal_Bone_V02_pinky', 'pinky first phalanx (JFb)', 'PLA'),
    'proximal':        ('SRC_Proximal_Phalanx_Bone_V02', 'middle phalanx, index/ring AND thumb (JFD)', 'PLA'),
    'proximal_middle': ('SRC_Proximal_Phalanx_Bone_V02_middlefinger', 'middle finger middle phalanx (JFT)', 'PLA'),
    'proximal_pinky':  ('SRC_Proximal_Phalanx_Bone_V02_pinky', 'pinky middle phalanx (JFX)', 'PLA'),
    'distal':          ('SRC_Distal_Phalanx_Bone_V02', 'distal phalanx, all 4 fingers AND thumb (JFL)', 'PLA'),
    'cover2':          ('SRC_driver_side_palm_cover2', 'pinky-side wall (KFLB)', 'PLA'),
    'cover3':          ('SRC_driver_side_palm_cover3', 'thumb-side wall + thumb mount + T1-T3 seats (KFDB)', 'PLA'),
    'cover_back':      ('SRC_cover_back', 'wrist-end back plate (KFfB)', 'PLA'),
    'shell1':          ('SRC_Shell1', 'back-of-hand dome (KFHB)', 'PLA'),
    'shell2':          ('SRC_Shell2', 'outer cap, MESH from Shell2.3MF (inverted, not a valid solid yet)', 'PLA'),
    'thumb_hub':       ('SRC_hand_pulley_wheel_thumb', 'thumb yaw hub / coupler on T3 shaft (KF3B)', 'PLA'),
    'thumb_hinge1':    ('SRC_first_thumb_hinge', 'thumb yoke, R2 axis (KF7B)', 'PLA'),
    'thumb_hinge2':    ('SRC_second_thumb_hinge', 'thumb gimbal cross, R2 pins + R3 bore (JFj)', 'PLA'),
    'thumb_hinge3':    ('SRC_third_thumb_hinge', 'thumb roll shaft, R3 + R4 cross bore (JFf)', 'PLA'),
    'tpu_index':       ('SRC_Palm1', 'TPU strip inside the index palm bone (JFv)', 'TPU'),
    'tpu_middle':      ('SRC_Palm2_2', 'TPU strip inside the middle palm bone (JFz)', 'TPU'),
    'tpu_ring':        ('SRC_Palm3_2', 'TPU strip inside the ring palm bone (JF3)', 'TPU'),
    'tpu_pinky':       ('SRC_Palm_pinky', 'TPU strip inside the pinky palm bone (JF7)', 'TPU'),
    'tpu_web':         ('SRC_Palm2', 'TPU 0.6 mm web between palm bones, x3 (JFr)', 'TPU'),
    'tpu_wrist':       ('SRC_palm_bone_flex', 'TPU plate across the 4 wrist segments (JF/)', 'TPU'),
    # wrist segments (Palm_bone1 sub-assembly, 4 parts, one frame): assigned to fingers by position
    'wrist_index':     (None, 'index wrist segment, seat IW', 'PLA'),
    'wrist_middle':    (None, 'middle wrist segment, seat MW', 'PLA'),
    'wrist_ring':      (None, 'ring wrist segment, seat RW', 'PLA'),
    'wrist_pinky':     (None, 'pinky wrist segment, seat PW', 'PLA'),
}
WRIST_SOURCES = ['SRC_Palm_bone1', 'SRC_Palm_bone001', 'SRC_Palm_bone002', 'SRC_Palm_bone003', 'SRC_Palm_bone004']

def resolve(doc):
    """Map every key to its V5 source object in doc (wrist segments by world Y of their centroid)."""
    out = {}
    for k, (src, _, _) in PARTS.items():
        if src:
            out[k] = doc.getObject(src)
    # wrist segments: all share one placement; sort by world Y (index is the most +Y)
    # any wrist-segment instance gives the (shared) placement; links may point at V5 sources or at V6_wrist_* parts
    link = next(o for o in doc.Objects if o.TypeId == 'App::Link' and o.LinkedObject is not None
                and (o.LinkedObject.Name in WRIST_SOURCES or o.LinkedObject.Name.startswith('V6_wrist_') or o.Label.startswith('Palm_bone1 <')))
    segs = []
    for n in WRIST_SOURCES:
        o = doc.getObject(n)
        if o is None or o.Shape.Volume < 1.0:
            continue
        c = link.Placement.multVec(o.Shape.CenterOfMass)
        segs.append((c.y, o))
    segs.sort(key=lambda t: -t[0])
    for k, (_, o) in zip(['wrist_index', 'wrist_middle', 'wrist_ring', 'wrist_pinky'], segs):
        out[k] = o
    return out
