# DexiGrab V6: design report (draft, 2026-09-26)

- **Master model:** `freecad/DexiGrab_V6.FCStd`. This FreeCAD file is now the master. The Onshape working copy only has the J0 change.
- **Rebuild:** `scripts/build_v6.py` applies the modules in this order: j0 → pillars → thumb → actuation → shells → electronics → fixes → tactile → proportions → hardware.
- **Checks:**
  - `scripts/audit_clearance.py` tests every pair of placed parts.
  - `scripts/rom_sweep.py` sweeps all 22 joints.
  - Each workstream has its own `scripts/verify_<topic>.py`.
- **Detailed notes:** `notes/<topic>.md`.

Status: everything below is built, verified and loaded in the FreeCAD window, **except Electronics (in progress) and the V6.2 pinky/index splay (in progress)**.

## Requirements → what was done

| # | Your requirement | V6 design | Status |
|---|---|---|---|
| 1 | Straight (coaxial) N20 motors, not the worm GM12 | 18 straight N20s with encoders, modelled from `N20.sat`. Each finger has a 2×2 magazine under its palm bone (A→J1, B→J2, C→J3, D→J0). T1/T2 stand upright in their V5 cover3 seat towers, centred and turned to each seat's angle and screwed to a printed floor (T1 in a collar on its tower). T3 drives the thumb yaw | ✅ |
| 2 | 19 motors, 10 drivers, 19-channel firmware | Motor-to-joint map is in `regions/actuation_motors.json`. The driver and pin map is part of Electronics | ⏳ Electronics |
| 3 | Screws, nuts, boards, battery, everything missing | 12 fastener types, 50 placed so far. Board, battery and case are part of Electronics | ✅ / ⏳ |
| 4 | Re-analyse and route wiring; index and pinky motors had no room | Every motor plug now points sideways into the gap beside its magazine, and its cable drops straight down, so the index/pinky problem is gone. Harness and tunnels are part of Electronics | ✅ / ⏳ |
| 5 | Case for 2× 6 V 2800 mAh NiMH AA packs in parallel | Part of Electronics | ⏳ |
| 6 | Fragile palm bones (TPU insert splits them) | PLA pillars through holes in the TPU, 19–46 mm² each, at every rib between seats, plus bars in the wrist segments. Every palm bone is now one piece along its whole length | ✅ |
| 7 | 0.2 mm clearance wherever parts need spacing | Global audit: **0 moving part pairs under 0.2 mm**. Clamped, bolted and inserted parts are deliberately in contact | ✅ |
| 8 | Thumb breaks under load; thumb joints too weak | Real yaw bearing: Ø18 journal plus thrust ring in cover3, so the motor carries torque only. Closed gimbal, steel pin + tube at R2, steel keeper at R3, M2 bolts at R4–R6 | ✅ |
| 9 | J0 hammered rod snaps 35 % of the time | M2×16 cap screw and M2 nyloc. Palm lug thickened to 5 mm with a flush counterbore; nut sits in a hex pocket; old pin holes filled; 0.2 mm running gaps | ✅ |
| 10 | Palm shells: spherical freedom, held together elastically | Shell2 rides on a spherical land on Shell1 (R 70.0/70.2). Two studs pass through arc slots, each with a silicone O-ring and washer under an M2 screw. Stops at −2.2°/+16.2° | ✅ |
| + | FlexiTac tactile skin, full coverage | 19 pads, 1421 taxels at 2 mm pitch; one flex circuit per finger with slack loops at every joint; 5 FlexiTac readout boards + USB hub in a forearm pod | ✅ |
| + | Thumb opposition (from your Leap data) | Flex stack re-clocked about its long axis: roll rest −58°, passive range −93…−23; yaw −85…+75 | ✅ |
| + | Human-like finger proportions (your Leap bones × 1.25) | V6.1: 14 finger-specific bones, spliced so every channel, anchor and pad recess is kept. Joint-to-joint targets hit exactly (index 43.6/29.5/25.0, middle 50.6/35.6/24.5, ring 49.5/27.8/21.5, pinky 32.9/25.0/21.0; thumb R5-R6 33, R6-tip 22). Leap re-fit: middle-joint (PIP) position error 2-3x smaller; the hand is ~1.28x your size | ✅ |
| + | Resting pinky spread (from Leap) | V6.2: the pinky Base Bone 1 gets a built-in 16° outward bend, and the index +8° (or +6°) toward the thumb. J0 joint unchanged | ⏳ V6.2 |

## Verification highlights

**J0 screw joint**
- 0.200 mm gaps all round.
- Clear over ±15°.
- The screw tip passes the nyloc by 3.5 threads.

**Palm pillars**
- Every palm bone is now a single solid through the TPU layer.
- TPU/PLA overlap is 0 in the assembled pose, with ≥ 0.2 mm around every pillar.

**Thumb**
- All 15 running clearances are ≥ 0.2 mm.
- Pinch pads meet with gaps of 0.11 mm (index) and 0.06 mm (middle), with no other collisions.
- Re-fit against your recorded thumb session (Leap session):

  | Measure | Before | After |
  |---|---|---|
  | Pad direction error, median | 48.5° | 0.1° |
  | Bone direction error, median | 7.9° | 0.4° |
  | Frames with roll at a limit | 83 % | 24 % |

  That was before the stops were widened. With the final stops (roll −93…−23, yaw −85…+75), yaw is essentially free and roll sits at a limit in a small minority of frames.

**Actuation**
- 0 motor/spool overlaps.
- All 18 plug keep-outs clear.
- Channel walls ≥ 1.0 mm.
- J1–J3 clear over 0–90°.

**Shells**
- 48/48 checks pass.
- The 25-pose roll/tilt sweep shows no collision and no loss of retention.
- Shell1×cover2 overlap (700 mm³) and cover_back×cover3 overlap are fixed.

**Tactile**
- Pads seated 0.2 mm proud with 0.2 mm edge clearance.
- 60 finger/thumb poses are clear, and every flex bend is ≥ 3 mm radius.

**Global audit and range-of-motion sweep** (`work/integrate_v61/`, V6.1)
- The only volume overlaps are intended: self-tapping screws into Ø1.7 pilot holes, and heat-set inserts.
- **Coupled limits the firmware must respect:**
  - **Finger abduction (J0):** a finger hits its neighbour at about 11° when abducting alone. When neighbours converge, keep their J0 difference under about 10°.
  - **Thumb yaw:** with pitch 0 it is clear only on −85…−20. At pitch −45 or lower the whole −85…+75 range is clear. Pitch (R2) is passive, not motor-driven, so firmware cannot set it: either cap yaw at −20 or add a pitch motor (see open items).
  - **Thumb R4 flexion at the rest yaw:** clear to 60°. At full flexion it meets cover3 unless the thumb is yawed or pitched for opposition.

## Print and assembly (details in each note)

**Palm bones**
- Print them back side down.
- Pause at **19.6 mm** to insert the TPU palm (pause at 8.5 mm too when printing them with the wrist segments). Lay the TPU in flush with the pillar tops.
- Print the TPU palm as a flat 0.6 mm part.

**J0**
1. Drop the nyloc into the back pocket.
2. Drive the M2×16 from the palm side with a 1.5 mm hex key, stopping at contact.
3. Back it off slightly so the finger swings freely.

**Index bracket straps**
- They now sit in pockets with 0.2 mm clearance.
- Use low-head or button-head M2×5 screws. A standard ISO 4762 head stands 0.25 mm proud.

**Shells**
1. Screw Shell1 onto cover2's wall end.
2. Lower Shell2 onto the hand, lip over cover3's wall edge.
3. On each stud fit a slider, then a 5×2.5 mm silicone O-ring, a cap washer and an M2×8.

**Thumb**
- Fit the nuts first.
- Motor in from below, then the coupler, carrier, roll shaft with its keeper (insert near −60° roll), the cross and tube, and finally the bones with M2 bolts.

**Tactile**
- Laminate each circuit as in the FlexiTac guide.
- Stick the pads down with 0.05 mm transfer tape and leave the tails free.

## Parts to buy (so far)

**Motors**
- 19 straight N20 gearmotors with 6-pin Hall encoders.
- The design assumes 1:298 gearing and 7 PPR. **Measure your motors' ratio, stall torque and encoder.**
- Confirm the gearbox face has 2 × M1.6 tapped holes 9 mm apart; T1–T3 are screwed through them, or glue them if not.

**Fasteners**
- M2×16 + M2 nyloc ×4 for J0.
- M2×5 low-head + M2 hex nut ×6 for the brackets.
- M2 and M1.6 screws for the thumb, trays and shells.
- Heat-set inserts for the tactile pod lid.
- Full counts and sizes: `notes/hardware.md` and `regions/hardware_report.json`.

**Other**
- Silicone O-rings, 5×2.5 mm, 40–50A, ×2.
- 0.5 mm braided tendon line.
- A 4×3 mm steel tube for the thumb R2.

**Tactile**
- 5 FlexiTac Reading Boards (Arduino Nano) and a 7-port USB hub.
- FPC pairs made from `work/tactile/dxf/`, plus Velostat and laminating sheets.
- About $150–300 per hand at prototype quantities.

## Open items / decisions for you

1. **Electronics:** boards, battery case, harness and pin map are still in progress. The Mega 2560's pin budget for 19 motors plus 38 encoder lines is being validated.
2. **V6.1 finger proportions:** in progress.
3. **Motor figures are assumptions:** the 1:298 ratio, 0.4 N·m stall and 7 PPR drive the spool sizing and firmware.
4. **Thumb pitch and roll are passive.** Of the thumb's 6 joints, 3 motors drive R1 yaw (T3), R4 (T1) and R5 + R6 coupled (T2). Pitch (R2) and roll (R3) have no motor.
   - Roll holds by friction; a light torsion spring at R3 would help it stay near −58°.
   - A pitch motor would be motor 20. The 10 dual drivers have exactly one spare channel, and it would need 2 more encoder pins on the Mega.
   - Hand totals: 22 joints, 19 motors, 19 independently controlled DOF. Each finger is fully actuated (4 joints, 4 motors).
5. **FlexiTac licence:** CC BY-NC 4.0, non-commercial, attribution required.
6. **Onshape:** the V6 geometry lives in FreeCAD. To bring it back, export STEP from FreeCAD and drag it into Onshape in the browser, which uses no API quota.
