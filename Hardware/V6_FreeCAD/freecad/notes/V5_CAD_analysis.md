# Robot hand CAD in Onshape: research report

Document **"Robotic Hand_V5_simulacra_with_missing_shell.zip"** (`8c4224ebcfece55ba8586cf9`, workspace Main `f48268e37e9db0ae8088e6d3`), explored on 2026-09-24 with the `onshape` MCP tools. Everything was read-only (list/get/find, `check_assembly_interference`, `eval_featurescript`, screenshots). Nothing in the model was changed.

**Tags.** **(M)** = measured: read from the model, or computed directly from measured geometry. **(I)** = inferred: an interpretation or estimate. **(R)** = taken from your `~/notes/hand-speedrun` repo (for example `cad_bom.csv`, which was extracted from the SolidWorks `Robotic Hand_V5.SLDASM`), not from this Onshape document.

**Frame and units.** All coordinates are in millimetres, in the top-level assembly frame. +X points toward the fingertips. +Y points toward the thumb. **+Z is the palm side**: the fingers flex toward +Z. The shells, and the pockets where the motors would sit, are on the −Z side, which is the back of the hand.

![whole hand, annotated](screenshots/hand_top_annotated.png)

---

## 0. Headline findings (most surprising first)

1. **The joints already exist as mates: 22 revolutes (M).** The top-level assembly has 22 revolute mates, one planar mate and one mate group, in folders named Thumb, Pointer, Middle, Ring and Pinky. All are in state OK. The geometry shows **4 joints per finger** (abduction J0, then J1–J3 flexion) and **6 on the thumb**: a 3-axis gimbal (yaw R1, pitch R2, roll R3) followed by 3 flexion joints (R4–R6). That is **22 kinematic DOF, not 16 (M/I).**
2. **There are 19 N20 motor seats, not 16, and no motors or pulleys in the model (M).** Each finger has 4 seats: 3 in its palm bone and 1 in its wrist segment. The thumb has 3 vertical seats in `driver_side_palm_cover3`. The CAD therefore puts **4 motors on every finger and 3 on the thumb**, not 3 per finger and 4 on the thumb. The SolidWorks V5 in your repo has the same 16 finger motor positions and no thumb motors (R).
3. **The only "pulley" is not a pulley (M).** `hand_pulley_wheel_thumb` sits directly on a vertical motor seat (the axes are 0.4 mm apart). It has a Ø3.4 D-bore and a 135° lobe of radius 8.5–13.4 mm that keys into the thumb yoke. It is a direct-drive coupler for thumb yaw, and it has no rope groove. None of the 16 tendon pulleys from V5 (R) are in this document.
4. **The missing shell is Shell2, and it has already been added (M).** `Palm_rigid` already contains an instance of `Shell2` linked from the **Shell2.3MF** document. It is the outer back-of-hand cap with the open-source-hardware gear logo, and it is a mesh body. It shares `Shell1`'s design frame, but it does not fit as placed. It hangs about **6 mm too low**, clear of Shell1. Even in the shared frame, its underside would sit **1.3–1.9 mm inside** Shell1's outer skin.
5. **The thumb is the right way round (M).** Its pad (flexion side) faces +Z, the palm side, the same as the fingers. So it flexes toward the fingertips and is not 180° off. Its pronation (roll) joint only travels about −10°…+18° before tabs collide, so the pad direction is essentially fixed by how the thumb is assembled.
6. **Only 5 of the checker's 58 bounding-box hits are real, and 3 more real overlaps sit outside what it checks (M).** Verified with exact geometry:
   - cover2 overlaps Shell1 by 2.2 mm.
   - The cover3 brackets sink 1.0 mm into the index palm bone.
   - The thumb hub overlaps the yoke by 2.2 mm.
   - `palm_bone_flex` is instanced twice at the same place.
   - `Palm_bone1 <1>` is a 0.006 mm³ junk sliver.
   - Outside the checker, which only compares parts within one assembly: the palm TPU web runs 2.5–2.8 mm into the ring and pinky palm bones (3 pairs).

   All 19 top-level hits (knuckles and neighbouring fingers) are clean.
7. **The tendon routing is unfinished (M).** No distal phalanx has a tendon anchor. Only the four Base Bone 1 knuckles, the pinky metacarpal and proximal phalanx, and the middle proximal phalanx have Ø2 channels. The thumb bones have none. The four abduction pins (J0) are not modelled, so their holes are empty.

---

## 1. Structure

### 1.1 Elements (M)
| Element | ID | What it is |
|---|---|---|
| Robotic Hand_V5_simulacra (Assembly) | a6fc7e8b… | **The complete hand.** 24 instances: 21 parts plus 3 sub-assemblies, with all 24 mate features (22 revolute, Planar 1, Group 1, plus folders) |
| Palm_rigid (Assembly) | eeb41023… | Rigid palm: 4 palm bones (`Base Bone 1.2_*`), side walls `driver_side_palm_cover2/3`, `cover_back`, `Shell1`, **`Shell2` linked from the Shell2.3MF document**, and the `Palm_bone1` sub-assembly. No mates |
| Palm_Deformable (Assembly) | 8477dc45… | The TPU palm (I, the RUNBOOK says "The palm is TPU", R): 4 strips (`Palm1`, `Palm2_2`, `Palm3_2`, `Palm_pinky`), 3 webs `Palm2`, and `palm_bone_flex` ×2. No mates |
| Palm_bone1 (Assembly) | 3e49bb9e… | Wrist-end chain of 4 segments, each holding one N20 seat, plus a junk sliver body. Nested in Palm_rigid. No mates |
| Assem5^Robotic Hand_V5_simulacra (Assembly) | 19efb9c2… | Thumb base: `hand_pulley_wheel_thumb` (yaw hub) + `first_thumb_hinge` (yoke). No mates (rigid pair) |
| Robotic Hand_V5_simulacra parts (Part Studio) | 5d0124b4… | One `Import 1` feature, **state WARNING** (import options: allowFaultyParts on, flatten off). 31 solid bodies |
| Robotic Hand_V5_simulacra.zip (Blob) + 5 BOM tables | | Not readable with these tools |

Tree (M):
```
Robotic Hand_V5_simulacra
├── Palm_rigid (identity in top level)
│   ├── Base Bone 1.2_V02_pointerfinger_and_thumb_attachment / _middlefinger / _ringfing / _pinky
│   ├── driver_side_palm_cover3, driver_side_palm_cover2, cover_back, Shell1
│   ├── Shell2  ← linked from document "Shell2.3MF" (7e601f7d…, Part Studio b3e90e39…, part JFD)
│   └── Palm_bone1 (sub-assembly): 4 wrist segments + 1 junk sliver
├── Palm_Deformable (identity in top level): Palm1, Palm2_2, Palm3_2, Palm_pinky, Palm2 ×3, palm_bone_flex ×2
├── Assem5 (rotated −47.15° about the thumb yaw axis = current R1 angle): hub + yoke
└── 21 parts: 4 fingers × (Base Bone 1, Metacarpal, Proximal, Distal) + thumb (Metacarpal, Proximal, Distal, second_thumb_hinge, third_thumb_hinge)
```

### 1.2 How the Part Studio is laid out (M)
Parts sit at their **own local origins** (see `screenshots/partstudio_iso.png`, where they overlap in a jumble). For example, every bone runs along local +X from x = 0 with a ±7.5 mm cross-section. The exception is the four `Palm_bone1` segments, which are modelled in place. Hand coordinates therefore need the assembly instance transforms. The MCP tools only expose each instance's translation and world bounding box. I recovered the full rotations by fitting them to those boxes, and checked them three ways:

- **Bounding-box fit (M).** The box the tool reports equals FeatureScript's non-tight `evBox3d`. The fit residual is ≤0.012 mm for every instance I solved.
- **Roll ambiguity (M).** Square-section bones fit the box equally well at any 90° roll about their own axis. I chose the roll whose knuckle windows open toward +Z, which is what the renders show.
- **Joint alignment (M).** With those transforms, all 22 joints line up: each pin or bore meets its partner's bore within **≤0.04 mm and ≤0.3°**.

Sub-assembly placement:
- **Palm_rigid and Palm_Deformable are at identity** (M). The palm clevis holes land on the finger J0 bores within 0.05 mm. The TPU strip windows land on the palm-bone motor seats within 0.07 mm.
- **Assem5 is rotated about the thumb yaw axis** (M, derived). Its yoke bores only line up with the second hinge's pins after a −47.15° rotation. That rotation is just R1's current angle, and it matches the render (`screenshots/thumb_base_top.png`).

### 1.3 Overall size (M)
The hand is 255.7 mm long, from the wrist end of cover3 (X −198.0) to the middle fingertip (X +57.7). The palm is about 129 mm wide, from cover2 at Y −94 to the thumb mount at Y +35. It is about 77 mm deep, from +7 down to the Shell1 crown at −69.9. The fingers fan out at 0° (index), 5°, 10° and 15° (pinky) toward −Y. The thumb points 46.5° toward +Y, lying flat in the palm plane.

| View | |
|---|---|
| Iso ![](screenshots/hand_top_iso.png) | Top ![](screenshots/hand_top_top.png) |
| Bottom (back of hand; grey = Shell2, black = Shell1) ![](screenshots/hand_top_bottom.png) | Front (−Y), shows Shell2 hanging below Shell1 ![](screenshots/hand_top_front.png) |
| From the fingertips (+X) ![](screenshots/hand_top_right.png) | Palm close-up ![](screenshots/palm_top.png) |

---

## 2. Parts (31 Part Studio bodies + Shell2)

Sizes are **local tight bounding boxes**, as length × width × height in the part's own axes. Volumes are from `evVolume`. Everything in this table is (M) unless marked.

### Finger and thumb bones (all 15 × 15 mm section, R6 rounded on the back side, square on the palm side)
| Part (ID) | Instances | Size (mm) | Vol (mm³) | Joint features |
|---|---|---|---|---|
| Base Bone 1_V02 (JFP) | 4 (index, middle, ring, pinky) | 31.5×15×15 | 3173 | Knuckle block: vertical Ø6.5 bore (J0) and horizontal Ø6.8 bore (J1), 18.3 mm apart. Ø2 tendon holes at z = +4.7 and −5.6 |
| Metacarpal Bone_V02 (JFH) | 4 (index, middle, ring, **thumb**) | 47×15×15 | 5158 | Clevis at both ends with **integral Ø4.5 pins** 32.0 mm apart. No tendon holes |
| Metacarpal Bone_V02_pinky (JFb) | 1 | 40×15×15 | 3645 | Integral Ø5.0 pins 25.0 mm apart. Ø2 back-side channel and a Ø3 knot hole (tendon anchor) |
| Proximal Phalanx Bone_V02 (JFD) | 3 (index, ring, **thumb**) | 56.6×15×15 | 8485 | Tongues with Ø6.8 bores 41.7 mm apart. **No tendon holes** |
| Proximal Phalanx Bone_V02_middlefinger (JFT) | 1 | 64.6×15×15 | 9887 | Bores 49.7 mm apart. Ø2 palm-side guides (z +4.75) at both joints, and a back-side through-channel (z −4.56) |
| Proximal Phalanx Bone_V02_pinky (JFX) | 1 | 46.6×15×15 | 6171 | Bores 31.7 mm apart. Same channels (z +4.68 / −4.62) |
| Distal Phalanx Bone_V02 (JFL) | 5 (4 fingers + thumb) | 36.8×15×15 | 5096 | Clevis with **integral Ø5.0 pin** 29.3 mm from the tip. **No tendon holes or anchor** |

Naming note: as positioned, "Metacarpal" is the first phalanx and "Proximal" the middle one. The palm's own metacarpals are the `Base Bone 1.2` parts (I).

### Thumb mechanism
| Part (ID) | Inst. | Size | Vol | Role |
|---|---|---|---|---|
| hand_pulley_wheel_thumb (KF3B) | 1 (Assem5) | 17.2×19.5×22 | 817 | Yaw hub/coupler. **D-bore Ø3.4, 3.0 across the flat**, hub Ø7.4, 135° lobe r 8.5–13.4 mm. **No rope groove** |
| first_thumb_hinge (KF7B) | 1 (Assem5) | 21.0×17×25 | 2043 | Yoke. Two R8.5 arms with Ø6.5 bores (R2 axis) |
| second_thumb_hinge (JFj) | 1 | 15.9×13.7×30 | 1063 | Gimbal cross. Integral Ø6 pins (R2) with Ø7.5 heads, bore for the third hinge (R3), Ø2 hole along the R2 axis |
| third_thumb_hinge (JFf) | 1 | 9.7×32×9.7 | 1317 | Roll shaft (Ø6/Ø9.7). Ø5.5 cross bore for R4 at y −25.8, Ø2 cross holes at the gimbal centre, roll-stop tabs |
| thumb_hollow_for_third_hinge (JFn) | **0 (unused)** | 6×10×6 | 283 | Helper/cut-tool body. A Ø6×10 cylinder the size of the third hinge's shaft |

### Palm and structure
| Part (ID) | Inst. | Size | Vol | Role |
|---|---|---|---|---|
| Base Bone 1.2_V02_pointerfinger_and_thumb_attachment (KFbB) | 1 | 96.8×15×21 | 10478 | Index palm bone: 3 N20 seats (I1–I3), J0 clevis (Ø5.5 lugs at x 89.3), Ø1.5 abduction channels, 3 M2 screw bores (Ø2.5 / Ø4.3 counterbore) for the cover3 brackets |
| Base Bone 1.2_V02_middlefinger (KFXB) | 1 | 96.8×15×21 | 11608 | Middle palm bone (M1–M3) |
| Base Bone 1.2_V02_ringfing (KFPB) | 1 | 96.8×15×21 | 11708 | Ring palm bone (R1–R3) |
| Base Bone 1.2_V02_pinky (KFTB) | 1 | 96.8×15×21 | 9819 | Pinky palm bone (P1–P3), plus screw bores to cover2 |
| Palm_bone1 ×4 (KFnB, KFrB, KFvB, KFzB) | 1 each (Palm_bone1 asm) | ≈15–16×18.4×15–16 | 1233–1514 | Wrist segments, one per finger, each with an N20 seat (IW, MW, RW, PW). Curved rolling faces between segments |
| driver_side_palm_cover3 (KFDB) | 1 | 128.7×36.9×68.0 | 49671 | Thumb-side wall and thumb mount. **3 vertical N20 seats (T1–T3)**, lug with a square window, 3 bracket straps over the index palm bone, grid of 24 Ø2.1 holes, Ø8 wire channel |
| driver_side_palm_cover2 (KFLB) | 1 | 108.1×19.8×52.9 | 22018 | Pinky-side wall. Ø8 wire channel, 4 Ø2.1 holes |
| cover_back (KFfB) | 1 | 71.1×4.5×56.4 | 15337 | Wrist-end back plate, in the same frame as Shell1 |

### Deformable palm (TPU, I)
| Part (ID) | Inst. | Size | Vol | Role |
|---|---|---|---|---|
| Palm1 / Palm2_2 / Palm3_2 (JFv/JFz/JF3) | 1 each | 18×65×3.6 | 545 | Strip inside the index, middle and ring palm bones. 3 motor windows and 6 Ø2 pegs each |
| Palm_pinky (JF7) | 1 | 15×65×3.6 | 428 | Same, pinky |
| Palm2 (JFr) | 3 | 8.65×64.9×0.6 | 110 | 0.6 mm web between palm bones, running through 0.6 mm slots in them |
| palm_bone_flex (JF/) | **2 (same place)** | 20.7×3.6(3.8)×50 | 502 | Wrist plate across the 4 wrist segments. 4 motor windows, 7 Ø2 pegs |

### Shells
| Part | Inst. | Size | Vol | Role |
|---|---|---|---|---|
| Shell1 (KFHB) | 1 | 110.5×120×66.5 | 48784 | Black dome over the back of the hand |
| **Shell2** (Shell2.3MF doc) | 1 (Palm_rigid) | 93.2×120×36.6 | 34682 | Outer cap with the OSH gear logo. **Single-face mesh solid** (3MF). See §5 |

### Everything else (M)
- **Motors: 0.** There are 19 seats (§4).
- **Tendon pulleys: 0.**
- **Electronics: 0**, including drivers and the Mega.
- **Battery and case: 0.**
- **Fasteners and pins: 0.** In particular there is no pin for any J0.
- **Junk:** `Palm_bone1` (KFjB) is a 0.006 mm³ sliver (0.007 × 2.3 × 0.6 mm) instanced as `Palm_bone1 <1>`.
- **Nothing is suppressed** in any assembly.

---

## 3. Joints

### 3.1 Mates (M)
| Folder | Mates |
|---|---|
| Thumb | Group 1 (mate group), Revolute 1, 2, 3, 4, 5, 6 |
| Pointer | Revolute 15, 16, 17, 22 |
| Middle | Revolute 13, 14, 18, 21 |
| Ring | Revolute 7, 11, 12, 20 |
| Pinky | Revolute 8, 9, 10, 19 |
| (no folder) | Planar 1 |

All mates are in state OK. The MCP tools do **not** expose mate details: the connected instances, mate connectors, and limits. So I matched them to joints from geometry. 4 revolutes per finger folder correspond to J0–J3, and 6 in the thumb folder to R1–R6 (I). I could not tell what **Planar 1** and **Group 1** attach to (open question). Your RUNBOOK notes that "CAD limit signs were uniformly inverted vs physical flexion" (R), so the mate limits exist but could not be read here.

### 3.2 Joint table
Axis points and directions come from the pins and bores (M). Ranges are geometric hard stops, found by rotating the child body about the measured axis and testing collisions with FeatureScript (M). They are 1–5° brackets, with tendons and motors absent. For flexion joints, **+ means flexion toward the palm (+Z)**.

| Joint | Parent → child | Axis point (mm) | Axis direction | Range (°) | Driven by (I) |
|---|---|---|---|---|---|
| **Index J0** (abduction) | index palm bone → Base Bone 1 <1> | (−71.8, −0.2, −0.5) | vertical (0,0,1) | ±20 (clevis stop); about ±10 relative to a still neighbour before the fingertips touch (I, 2D estimate) | one of IW/I1–I3; lateral Ø1.5 channel pair |
| Index J1 | Base Bone 1 <1> → Metacarpal <1> | (−53.5, −0.3, −0.5) | (0, 1, 0) | −2.5 … +93 | one of IW/I1–I3 |
| Index J2 | Metacarpal <1> → Proximal <1> | (−21.5, −0.3, −0.5) | (0, 1, 0) | −3 … +93 | " |
| Index J3 | Proximal <1> → Distal <1> | (20.2, −0.3, −0.5) | (0, 1, 0) | −3 … +94 | " (no anchor on the distal) |
| Middle J0 | middle palm bone → Base Bone 1 <2> | (−71.8, −25.6, −0.5) | vertical | ±20 (same design) | MW/M1–M3 |
| Middle J1 | Base Bone 1 <2> → Metacarpal <2> | (−53.6, −27.2, −0.5) | (0.089, 0.996, 0) | −2.5 … +93 | " |
| Middle J2 | Metacarpal <2> → Proximal _middlefinger | (−21.7, −30.0, −0.5) | (0.089, 0.996, 0) | −3 … +93 | " |
| Middle J3 | Proximal _middlefinger → Distal <2> | (27.8, −34.4, −0.5) | (0.089, 0.996, 0) | −3 … +94 | " |
| Ring J0 | ring palm bone → Base Bone 1 <4> | (−75.0, −50.7, −0.5) | vertical | ±20 | RW/R1–R3 |
| Ring J1 | Base Bone 1 <4> → Metacarpal <3> | (−57.0, −53.9, −0.5) | (0.175, 0.985, 0) | −2.5 … +93 | " |
| Ring J2 | Metacarpal <3> → Proximal <2> | (−25.5, −59.5, −0.5) | (0.175, 0.985, 0) | −3 … +93 | " |
| Ring J3 | Proximal <2> → Distal <3> | (15.6, −66.8, −0.5) | (0.175, 0.985, 0) | −3 … +94 | " |
| Pinky J0 | pinky palm bone → Base Bone 1 <3> | (−80.4, −75.5, −0.5) | vertical | ±20 | PW/P1–P3 |
| Pinky J1 | Base Bone 1 <3> → Metacarpal _pinky | (−62.7, −80.2, −0.5) | (0.26, 0.966, 0) | −2.5 … +93 | " |
| Pinky J2 | Metacarpal _pinky → Proximal _pinky | (−38.6, −86.7, −0.5) | (0.26, 0.966, 0) | −3 … +93 | " |
| Pinky J3 | Proximal _pinky → Distal <4> | (−8.0, −95.0, −0.5) | (0.26, 0.966, 0) | −3 … +94 | " |
| **Thumb R1** (yaw) | cover3 / Palm_rigid → Assem5 (hub + yoke) | (−165.3, 21.0, −17.6) | vertical | toward the index: +30 (thumb metacarpal hits cover3 by +40). Away: at least −100 with no stop found | **T3, direct drive** (hub on the seat axis, 0.4 mm offset) |
| Thumb R2 (pitch) | yoke → second_thumb_hinge | (−165.3, 20.9, −0.5) | (−0.725, 0.689, 0) | free −60 (palm side) … +20 (back side). Stops between −60/−90 and +20/+40 | none dedicated (I) |
| Thumb R3 (roll / pronation) | second → third_thumb_hinge | gimbal centre | along the thumb (0.689, 0.724, 0) | **−10 … +18** (tabs on the third hinge hit the second) | none dedicated (I) |
| Thumb R4 | third_thumb_hinge → Metacarpal <4> | (−154.0, 32.9, −0.5) | (−0.725, 0.689, 0) | about −110 … +100 (slim shaft end, no early stop) | T1/T2 tendons (I) |
| Thumb R5 | Metacarpal <4> → Proximal <3> | (−132.0, 56.1, −0.5) | (−0.725, 0.689, 0) | −3 … +93 | T1/T2 tendons (I) |
| Thumb R6 | Proximal <3> → Distal <5> | (−103.3, 86.3, −0.5) | (−0.725, 0.689, 0) | −3 … +94 | T1/T2 tendons (I) |

R1, R2 and R3 meet within 0.06 mm at (−165.3, 20.95, −0.5) and are mutually perpendicular to within 0.3° (M). **The thumb base is a 3-axis gimbal.**

Why the flexion joints stop where they do (M): each knuckle is a clevis (two 2.4 mm cheeks, 10.2 mm gap, integral pin) around a 9.4 mm tongue. On the palm side the tongue is rounded (R7.4). On the back side it is square and reaches to within 0.3 mm of the clevis block. So a finger can flex about 93° toward the palm, but hyperextends only 2–3°. The fingers are posed straight, at the back-side stop.

Pin and bore fits (M): the Ø4.5 metacarpal pins and Ø5.0 pinky/distal pins sit in **Ø6.8** bores, leaving 0.9–1.15 mm of radial clearance. The J0 clevis has Ø5.5 lug holes and a Ø6.5 bore in the base bone, with **no pin modelled**.

### 3.3 Link lengths (M)
| | J0→J1 | J1→J2 | J2→J3 | J3→tip | J0→tip |
|---|---|---|---|---|---|
| Index | 18.3 | 32.0 | 41.7 | 29.3 | 121.3 |
| Middle | 18.3 | 32.0 | 49.7 | 29.3 | 129.3 |
| Ring | 18.3 | 32.0 | 41.7 | 29.3 | 121.3 |
| Pinky | 18.3 | 25.0 | 31.7 | 29.3 | 104.3 |
| Thumb (gimbal→R4→R5→R6→tip) | 16.4 | 32.0 | 41.7 | 29.3 | 119.3 |

### 3.4 DOF count
- **In the CAD: 22 revolute DOF (M/I).** That is 4 per finger (abduction + 3 flexion) and 6 on the thumb (3-DOF gimbal + 3 flexion). One gimbal DOF (roll) is limited to about 28° of travel. The planar mate may add more if it leaves something floating (open question).
- **Actuator seats: 19 (M).** 4 per finger and 3 on the thumb.
- **So "16 DOF, 3 per finger and 4 on the thumb" is not what this CAD shows (M/I).** Every finger joint, including abduction, has a motor seat available: 4 seats for 4 joints. The thumb has 3 seats for 6 joints. The SolidWorks V5 source in your repo (R) has 16 GM12-N20 worm motors, **all in the finger rows (4 per finger) and none at the thumb**. Their centres sit on this CAD's finger seats, offset about 10 mm along the seat axis where the gearbox hangs (I).

---

## 4. Actuation

### 4.1 Motor seats (M)
All 19 seats have the same N20 signature: a Ø12.28 trough 15–19 mm long, with Ø13.0 relief arcs and a Ø9.5 × 1 mm end recess.

- **Finger seats** are inclined **30.7°** down toward the fingertips, with the can axis along (0.86, 0, −0.51) in each palm bone's frame. The motors hang into the cavity at the back of the hand. Each seat breaks through the palm face as the crescent window visible in the top view. Seats are spaced 19.9–20.0 mm along each palm bone. A fourth seat per finger sits in the wrist segment, about 29 mm further toward the wrist.
- **Thumb seats T1–T3** are vertical, in cover3.

| Seat | Finger | Host | Seat mid (X, Y, Z) |
|---|---|---|---|
| IW, I1, I2, I3 | index | wrist segment; index palm bone ×3 | (−171.5,−0.6,−5.2)*, (−142.2,−0.6,−6.9), (−122.3,−0.6,−6.9), (−102.4,−0.6,−6.9) |
| MW, M1, M2, M3 | middle | wrist segment; middle palm bone ×3 | (−171.0,−17.3,−5.3)*, (−142.0,−19.8,−6.9), (−122.1,−21.5,−6.9), (−102.3,−23.3,−6.9) |
| RW, R1, R2, R3 | ring | wrist segment; ring palm bone ×3 | (−173.2,−33.8,−5.2)*, (−144.4,−38.8,−6.9), (−124.7,−42.3,−6.9), (−105.1,−45.8,−6.9) |
| PW, P1, P2, P3 | pinky | wrist segment; pinky palm bone ×3 | (−176.6,−49.7,−5.2)*, (−148.4,−57.6,−6.9), (−129.2,−62.8,−6.9), (−109.9,−67.9,−6.9) |
| T1 | thumb | cover3 (behind the gimbal, under the lug window) | (−185.8, 20.7, −54.9), vertical, Z −61.0…−48.8 |
| T2 | thumb | cover3 (in front of the gimbal) | (−147.9, 26.9, −42.9), vertical, Z −52.4…−33.3 |
| T3 | thumb | cover3 (**on the R1 yaw axis**) | (−165.7, 21.3, −47.3), vertical, Z −56.8…−37.8 |

\*Wrist seats: point where the seat axis crosses `palm_bone_flex` (the Palm_bone1 placement inside Palm_rigid could not be recovered). Full list in `data/motor_seats.csv`.

### 4.2 Pulleys (M)
There are no tendon pulleys in this document. `hand_pulley_wheel_thumb` is a yaw coupler (§2). With the N20 standing in T3, its shaft would reach the hub's D-bore at Z −26.5 (I). That needs a **coaxial (spur-gear) output**, whereas a worm N20's output shaft is perpendicular to the motor. See the open questions.

Your repo's V5 BOM lists 8 × `hand_pulley_wheel1` (23.75 × 5.0 × 22.5 mm) and 8 × `hand_pulley_wheel22` (23.75 × 12.5 × 22.5 mm) (R). That puts the outer radius at about 11.9 mm, with the groove radius unknown.

### 4.3 Tendon channels and moment arms (M)
| Where | Channel | Offset from joint axis = moment arm |
|---|---|---|
| Palm bones (all 4) | Ø1.5 pair, from x≈71–75 in the palm bone to the clevis sides, exiting **±10.3 mm lateral**, 5.4 mm beyond the J0 axis. Also Ø1.5 entries at x≈71.2 from the back-side face, and Ø2.9 half-grooves (vertical, ±6.05 lateral, x = 83.25) | **J0: about 10.3 mm** (antagonistic pair) |
| Base Bone 1 (all 4) | Ø2 along x at z **+4.68** (palm side) over J1; Ø2 at z **−5.6** (back side, tilted 14°) near J1; a vertical Ø2 connector; two 45° Ø2 exits at the J1 end face | **J1 flexion 4.7 mm**, extension about 5.6 mm |
| Metacarpal _pinky only | Ø2 back-side channel at z −4.62, ending in a Ø3 knot hole (anchor) at x 18.95 | pinky J1 extension 4.6 mm |
| Proximal _middlefinger / _pinky only | Ø2 palm-side guides at z **+4.75 / +4.68** spanning both joints; a Ø2 back-side through-channel at z **−4.56 / −4.62** | **J2/J3 flexion 4.7 mm**, extension 4.6 mm |
| Metacarpal V02, Proximal V02, all Distal V02 | none | none: **index/ring/thumb J2–J3 and every J3 anchor are unmodelled** |
| Thumb third hinge / second hinge | Ø2 cross holes **at the gimbal centre** (within 0.34 mm); Ø2 hole **along the R2 pin axis** | about 0 about R1/R2/R3 by design: pass-throughs that decouple the thumb tendons from the gimbal (I) |

With straight fingers, the flexion moment arm is about 4.7 mm. Taking a V5 pulley groove of about 10 mm radius (I), 90° of joint flexion needs about 7.4 mm of tendon, which is about 42° of pulley rotation (I).

### 4.4 Motor-to-joint map (I)
| Finger / thumb | Seats | Joints | Most plausible assignment |
|---|---|---|---|
| Each finger | W, 1, 2, 3 | J0, J1, J2, J3 | **One seat per joint, with no coupling needed.** J0 most likely uses seat 3, the one nearest the lateral J0 channels. The order of the others is **not determinable**: no spools, tendons or anchors are modelled |
| Thumb | T1, T2, T3 | R1–R6 | **T3 → R1 yaw (direct drive).** T1/T2 → tendons through the gimbal centre to the flexion chain R4–R6, either coupled or a flexor/extensor pair. R2 pitch and R3 roll have no dedicated actuator |

Your plan's channel preset "thumb_m1…pinky_m3" (R, BUILD_PLAN) assumes 4 thumb and 3 finger motors, which does not match the CAD's seats.

---

## 5. What's missing, and Shell2

### 5.1 Absent from this document (M)
Compared against the V5 BOM in your repo (R):
- 16 GM12-N20 worm gearmotors
- 16 pulleys
- 8 DC motor driver boards
- the Mini Mega2560 PRO
- the 6 V battery and battery case
- all M1.6 and M2 screws and nuts
- `thumb_ball_joint_cap` and `Metacarpal Bone_V02_thumb` (both hidden in V5)
- all of V5's cut-tool bodies (about 25 kinds, for example `motor_cav_tolerances` ×16, `string_3dhole1/2` and `side_and_knuckle_stringholes` ×4). Only `thumb_hollow_for_third_hinge` survives here

Never modelled anywhere: tendons, J0 pins, and motors for T1–T3.

Several shared parts differ from the V5 BOM: cover3 is 128.7 mm long here against 79.9 mm there, V5 has no thumb seats or thumb hub, and the bones here have no M1.6 holes or nut slots. So this is **a different revision (I)**, probably the sim-ready "simulacra" asset: kinematics-only, with mates added and purchased parts stripped.

### 5.2 Shell2
- **It is the missing shell (M).** "Shell2" is a single mesh solid (34,682 mm³) in the Shell2.3MF document (`7e601f7d…`, Part Studio `b3e90e39…`, imported from a 3MF). It is the outer cap of the back of the hand, 2.4–5.4 mm thick including the gear-logo relief (`screenshots/shell2_doc_iso.png`).
- **It has already been inserted (M).** It is instanced in **Palm_rigid** as `Shell2 <1>`. The imported zip's own Part Studio has no Shell2, which is where the document name comes from (I).
- **It uses Shell1's design frame (M).** Both span exactly Y −120…0. I cast 150 vertical rays through each part. In that shared frame, Shell2's underside lies **1.3–1.9 mm below Shell1's outer surface** everywhere, so it would overlap rather than seat.
- **As placed, it is about 6 mm too far out (M, from renders ±1 mm).** Its footprint matches the shared frame within a few mm in X/Y (measured Y −66.7…16.6 against about −66…15.5 predicted). But its lowest point is at Z ≈ −78.6 instead of −72.4, so it floats about 4–5 mm clear of Shell1 (`screenshots/raw/palm_rigid_front_2400.png`, `palm_rigid_right_2400.png`).
- **Would it fit? (I)** It is the right part, with the right footprint and curvature. To seat it, either offset it outward about 1.5–2 mm from its native frame, or recess Shell1 by that much where it sits. Its current placement is wrong either way. Because it is a mesh, it cannot be offset or booleaned parametrically.

---

## 6. Problems

### 6.1 Interference
`check_assembly_interference` reported 58 bounding-box hits (19 top level, 14 Palm_rigid, 23 Palm_Deformable, 1 Palm_bone1, 1 Assem5). **I checked every one with exact geometry.** Points were sampled on each body (vertices, edge points, 3×3 face points 0.05 mm inside), moved into the other body's frame with the reconstructed transforms, and tested for containment. Penetration of 0.15 mm or less counts as touching. Per-pair results are in `data/interference.csv`.

**Real interference (M):**
| Pair | Depth | Where |
|---|---|---|
| Shell1 × driver_side_palm_cover2 | **2.2 mm** | cover2's outer wall against Shell1's side wall, along most of its length |
| driver_side_palm_cover3 × index palm bone | **1.0 mm** | the three bracket straps sink into the palm face of the bone (x 24–75 mm) |
| hand_pulley_wheel_thumb × first_thumb_hinge (Assem5) | **2.2 mm** | the hub's lobe runs into the yoke underside. Might be an intended press-fit |
| palm_bone_flex <1> × palm_bone_flex <2> | 100 % | duplicate instance at an identical transform |
| Palm3_2 × ring palm bone *(outside the checker)* | 2.5 mm | the strip's 0.6 mm web layer runs into the bone's wrist-end wall; the web slot stops about 2.5 mm short |
| palm_bone_flex × ring / pinky palm bones *(outside the checker)* | 2.5 / 2.8 mm | same web level, wrist-end walls |
| Palm_bone1 <1> | n/a | 0.006 mm³ junk sliver touching segment <4> |

**Not real:**
- All 19 top-level hits: 12 finger knuckles, 2 neighbouring-finger hits caused by the splay, and 5 thumb pairs.
- Shell1 against the palm bones and cover_back.
- cover2 against its palm bones, and the palm bones against each other.
- All 22 TPU-to-TPU hits.
- cover3 × Shell1 and cover3 × cover_back are touching (≤0.1 mm).

### 6.2 States (M)
- Every assembly feature is OK.
- No instance is suppressed.
- The only warning is the Part Studio's **`Import 1` feature, which is in WARNING**. Its message is not exposed by these tools, and "allow faulty parts" is on.

### 6.3 Thumb orientation (M)
The thumb bones sit with their local +z, which is their flexion or pad side, facing **world +Z**. This is the same as every finger bone. Evidence:
- In the render (`screenshots/thumb_top.png`), the R5 and R6 knuckle windows open to +Z. That only happens when the proximal phalanx's rounded tongue side faces +Z.
- The metacarpal's square-cornered block faces +Z.
- The stop geometry lets these joints flex only toward that side (§3.2).
- R4, the third hinge's cross bore, is horizontal and perpendicular to the thumb.

So R4–R6 curl the thumb toward the palm, where the flexed fingertips go, and the back of the thumb faces the shells. **This is not the "nail faces the fingers/palm" 180° error described in RUNBOOK G4 (R).** In the flat pose the pad faces the palm normal. It is not turned toward the index until R1 and R2 move. With only −10…+18° of roll, a wrong roll could not be corrected by the joint, so re-check it on the physical hand anyway.

### 6.4 Other issues
- Tendon routing is incomplete and inconsistent across fingers (§4.3). No distal anchors.
- The J0 pins are missing. Integral pins are 0.9–1.15 mm loose in Ø6.8 bores.
- The "deformable" palm is rigid in CAD: there are no mates in Palm_rigid, Palm_Deformable or Palm_bone1.
- The helper body `thumb_hollow_for_third_hinge` is left in the Part Studio and not instanced.
- Motor type conflict at T3 (§4.2).

---

## 7. Open questions
1. **Mate details.** Which instances do Planar 1 and Group 1 act on, and what limits do the 22 revolutes have? The MCP only lists names, types and states. A read-only REST `GET /assemblies/.../e/...?includeMateFeatures=true`, or a look in the Onshape UI, would answer it. I did not do this because it is outside your tool allowlist.
2. **Motor count and split.** Is the build 16 motors (V5: 4 per finger, none on the thumb) or 19 (this CAD: 4 per finger plus 3 thumb)? Your plan's 3 per finger + 4 thumb matches neither.
3. Which seat pulls which joint, and are any joints coupled? That needs the tendon routing, which is not in this model.
4. **T3 drive.** A worm N20 has a perpendicular output shaft. Is T3 meant for a spur N20, or is the hub/pocket arrangement from an older concept?
5. **Shell2.** Should it seat directly on Shell1 (with Shell1 recessed about 1.5–2 mm), or sit offset? Who placed it about 6 mm low?
6. **J0 pin spec** (Ø5–5.5 × about 15 mm?), and the intent of the Ø6.8 bores around Ø4.5/Ø5.0 pins (bushings? tolerance?).
7. Are the index, ring and thumb bones older V02 variants that should get the middle/pinky tendon channels, and the distals an anchor?
8. What is the cause of the `Import 1` WARNING?
9. Is the "Palm_Deformable" meant to be articulated (the palm arch) for simulation?

---

## 8. Method and limits
- **Structure.** `get_document_summary`, `get_elements`, `get_assembly` (all 5), `get_assembly_features` (all 5), `get_features` and `get_parts` on both Part Studios.
- **Geometry.** `eval_featurescript` for tight/loose boxes, volumes, every cylindrical face (axis, radius, convex/concave, extent), planar faces, ray casts and point containment. `get_body_details` was never used.
- **Transforms.** From `get_assembly_positions`, fitted in `tools/solve.py`, and saved to `data/instance_transforms_topframe.json`.
- **Interference and ranges.** FeatureScript point sampling (`tools/fsgen*.py`, `romgen.py`, `chaingen.py`).
- **Limits of what I could see.**
  - Screenshots always auto-fit the whole assembly, so the per-finger images are crops of a 2400 px top view.
  - The tools cannot read BOM elements, mate definitions, or the Palm_bone1 and Shell2 instance transforms. Shell2's placement is from calibrated renders (about ±1 mm).
  - Range-of-motion stops ignore tendons and motors, and each sweep considered only the pair or neighbours listed.
- **Screenshots** (in `screenshots/`):
  - Whole hand: `hand_top_{iso,top,bottom,front,right}.png`, `hand_top_annotated.png`
  - Per finger: `finger_{index,middle,ring,pinky}_top.png`, `finger_side_front.png`, `thumb_top.png`, `thumb_base_top.png`
  - Palm: `palm_top.png`, `palm_rigid_{iso,bottom}.png`, `palm_deformable_iso.png`, `palm_bone1_iso.png`
  - Thumb base: `assem5_iso.png`
  - Part Studios: `partstudio_iso.png`, `shell2_doc_iso.png`
  - Full-resolution originals: `raw/`

| Finger close-ups (top, palm side) | |
|---|---|
| Index ![](screenshots/finger_index_top.png) | Middle ![](screenshots/finger_middle_top.png) |
| Ring ![](screenshots/finger_ring_top.png) | Pinky ![](screenshots/finger_pinky_top.png) |
| Thumb ![](screenshots/thumb_top.png) | Thumb base: yoke, gimbal, lug ![](screenshots/thumb_base_top.png) |
| Pinky side profile (palm side up; the circles are the R7.4–7.5 knuckle ends; the fingertip is rounded on the back side) ![](screenshots/finger_side_front.png) | Palm_rigid from below: Shell2 (grey) inside Shell1 (black) ![](screenshots/palm_rigid_bottom.png) |
