# V6 thumb (requirement 8 + opposition re-clock): real yaw bearing, steel-pinned joints, thumb that curls across the palm

Owner: thumb agent. Files:
- `lib/mod_thumb.py`
- `scripts/verify_thumb.py` (last full run: `work/thumb/verify.log`)
- `regions/thumb.json`
- `regions/fasteners_thumb.json`
- `data/v6_joints.json` (Revolute 2 and 5; written on the coordinator's request)
- work files in `work/thumb/`

Test build: `V6_MODULES=j0,thumb`, then `work/thumb/apply_rest.py` (see "Placements hook" below), giving `work/thumb/test.FCStd`.

All part-local joint frames are unchanged, so `data/mates.csv` stays valid. The only frames that move are the instance rest placements of the flex stack.

## 1. Why the thumb broke (V5)

- **The yaw hub hung on the motor shaft.** The hub had a Ø7.4 printed stem with a Ø3.4 D-bore, sitting only on the T3 N20's Ø3 shaft. It floated about 1 mm above the tower with a gap of about 2.5 mm all round, so there was no bearing. 20 N on the pad, about 119 mm out, puts about 2.4 N·m on the stem and shaft, which is about 900 MPa in a Ø3 shaft.
- **The gimbal pins were printed:**
  - R2: Ø6 snap pins.
  - R3: a Ø6 roll shaft with a split snap head.
  - R4: the metacarpal's integral Ø4.5 pin.
  - R5 and R6: the finger-style print-in-place pins, with 0.9–1.15 mm of slop.
- **The hub and yoke** overlapped by 2.2 mm.

## 2. Load path

**Before:** thumb → printed snap pins → yoke → hub lobe → Ø7.4 printed stem → **Ø3 N20 shaft → gearbox**. 100 % of the forces and moments went into the motor.

**After:**
1. thumb bones → **M2 through-bolts** at R4, R5 and R6
2. → Ø7 roll shaft in the closed gimbal barrel's 15 mm bore. The R3 steel keeper takes the axial load and provides the roll stops.
3. → R2 trunnions: an M2 screw on one side and a 4×3 mm steel tube on the other
4. → one-piece yaw carrier
5. → **cover3:**
   - Ø18 × 6.3 journal: radial load and moment
   - thrust ring, r 9.2–11.0: downward load and moment
   - M2 keeper screw in the rim groove: lift, moment and the yaw stops

The N20 drives only through a floating D-coupler. The coupler has 0.4 mm of radial float against the journal's 0.2 mm, so the shaft sees **torque only**. `verify_thumb.py` [4] shows the carrier doesn't touch the motor.

## 3. Opposition re-clock (2026-09-26)

The Leap Motion recording of the author's hand (1411 bent-thumb frames) showed that the V5 thumb flexes toward +Z like a fifth finger. Its flexion axis is (0.72, −0.69, 0), about 55–65° away from the user's (0.40, −0.19, −0.89). Roll is passive, so instead of widening it I re-clocked the flex stack.

- **Rest placement.** The instances `third_thumb_hinge <1>`, `Metacarpal Bone_V02 <4>`, `Proximal Phalanx Bone_V02 <3>` and `Distal Phalanx Bone_V02 <5>` get a new rest placement: rotated about R3 through G, to Revolute 2 = **−58**. In mate terms that is a rotation of −62.4° about the third hinge's connector z (= −e3).
  - New flexion axis: **(0.33, −0.32, −0.89)**. The pad now faces the index side, 27.6° above −e2.
  - Part-local geometry and joint frames are unchanged. The roll shaft's R4 tongue and bore are re-clocked with the placement.
- **Roll keeper groove.** Re-cut on the roll shaft for the new range. The passive roll is **±35 about −58, i.e. −93…−23**. It started at ±25 (−83…−33). The Leap re-fits then found 15 % of frames at −83 and 8 % at −33, so both ends were extended. Index-tip opposition sits at −58.4…−52.9. Hard stops are 5° outside, at −98 and −18.
- **Tendons.**
  - The V5 radial exits through the barrel are removed. At the new clocking they would point straight into the yoke arms.
  - New path: tube along R2 → Ø2 hole in the cross → Ø2 **entry fan** in the roll shaft at the gimbal centre, open toward the tube side over the whole roll range → Ø2 **axial bore on the roll axis**, which decouples it from roll → 45° branches to the tongue's pad (flexor) and back (extensor) edges, 9–14 mm in front of G, into the open metacarpal clevis slot.
  - The fan is a union of Ø2 bores at 7.5° steps. It keeps the R2 entry open at every roll from −93 to −23; `verify_thumb.py` [10] runs a Ø0.8 tendon through the whole path at 5 roll values, all clear.
  - The Ø2 pass-throughs at the gimbal centre are kept.
- **Yaw.** Negative travel is extended from −70 to −75 (Leap p2 −71.8), then to **−85** after the re-fit found 6 % of frames at the −75 stop. Yaw stops are at −87.5 and +77.5. The carrier is checked clear of cover3 and the index wrist segment at pitch 0, −45 and −90.
- **Carrier clearance for the rolled metacarpal.** The rolled metacarpal's rounded clevis end reaches lower at pitch +25, so the carrier was relieved:
  - platform top between the arms: Z −11.5 → −12.3
  - front of the turntable rim: −13.3, and −14.2 for x > 6.5
  - The journal and the coupler pocket roof are untouched.
- **Published in `data/v6_joints.json`:**
  - `Revolute 2`: `{"lo": -93, "hi": -23, "rest": -58}`
  - `Revolute 5`: `{"lo": -85, "hi": 75, "rest": -43.48}`
- **Rest convention.** "rest" is the rom_sweep/Leap value, θ0 + sign·rotation. Re-measuring the re-placed pose with atan2 gives 2·4.4 − (−58) = +66.8, so **use "rest", not the atan2 re-measure**. `verify_thumb.py` does this.
- **Bone lengths are not changed here.** The V6.1 agent will re-proportion the fingers and shorten `thumb_proximal` and `thumb_distal` by splicing them after all modules. This module keeps the V5 lengths and every part-local joint frame.

## 4. What changed (parts)

| Part (key) | V6 design |
|---|---|
| cover3 (T3 region only) | <ul><li>V5 T3 seat, window, slot and nut trap filled.</li><li>Turret r 11.0 with a Ø18.4 cup up to the thrust ring at Z −15.0.</li><li>2.8 mm floor at Z −24.5…−21.7. The N20 gearbox face is screwed to it with 2× M1.6×3.</li><li>N20 pocket: envelope + 0.2, open through the bottom (the motor goes in from below), 8 mm plug keep-out.</li><li>North keeper boss: M2 chord at Y R1+8.8, Z −18.3.</li></ul> |
| thumb_hinge1 → **yaw carrier** | <ul><li>Turntable r 9.0 (Z −21.3…−12.3).</li><li>Platform with its underside 0.2 mm above the thrust ring.</li><li>6 mm arms at \|s\| 8.4–14.4.</li><li>7.4 mm square coupler pocket.</li><li>Keeper groove giving yaw stops at −87.5 and +77.5.</li><li>R2 +s: M2 hole, nyloc pocket, head counterbore.</li><li>R2 −s: Ø4.4 bore for the tube.</li><li>The tube-side arm is trimmed to r 14.2 about R1 (index wrist segment); the whole carrier fits inside r 16.0 (cover3 wall strip).</li><li>Front relief for the re-clocked metacarpal (section 3).</li></ul> |
| thumb_hub → **D-coupler** | 7.0 × 7.0 × 7.6 square sleeve. D-bore Ø3.4 with the flat 1.2 from the axis, and a 0.4 lead-in chamfer. |
| thumb_hinge2 → **closed gimbal barrel** | <ul><li>Barrel r 7.5, 16.4 wide, with a front thrust boss r 5.5.</li><li>R3 bore Ø7.4.</li><li>Blind R2 holes: Ø2.4 for the M2, Ø4.4 for the tube.</li><li>Ø2 tendon hole along R2.</li><li>R3 keeper: M2 plus hex nut.</li></ul> |
| thumb_hinge3 → **roll shaft** | <ul><li>Ø7 shaft, x −7.3…7.7.</li><li>Collar Ø10 as the thrust face.</li><li>6.0 mm tongue with an M2 hole at R4.</li><li>Keeper groove for −93…−23 plus the stops.</li><li>Tendon entry fan, axial bore and pad/back branches (section 3).</li></ul> |
| thumb_metacarpal (relinked `Metacarpal Bone_V02 <4>`) | <ul><li>R4 cheeks thickened to a 6.4 slot, pin removed, M2×14 with a nyloc.</li><li>R5 pin removed, slot 9.86, tips relieved to r 7.6, M2×16 with a nyloc.</li></ul> |
| thumb_proximal (relinked `<3>`) | Both Ø6.8 bores plugged and redrilled Ø2.4. |
| thumb_distal (relinked `<5>`) | R6 pin removed, slot 9.8, tips relieved to r 7.6, M2×16 with a nyloc. |
| T3_motor (Motors), thumb_R2_tube (Thumb) | Placed by this module. The N20 origin is (R1X, R1Y, −24.5), shaft up, encoder tab toward −Y. |

## 5. Verification

All results are from `work/thumb/verify.log` at the re-clocked rest.

**Static checks.**
- All 10 parts are valid single solids (the N20 model is 2 solids by design).
- All relinks are OK.
- All 15 designed running clearances are ≥ 0.2 mm with 0 overlap. R5 is 0.205; everything else is 0.200.
- There are 0 thumb-related overlaps with anything else. The three flagged cover3 pairs are pre-existing in `audit_baseline_j0.csv`: the index-bone straps, cover_back and Shell1.

**Range of motion.** Each joint alone, 13 steps, 0.01 mm³ collision tolerance, keepers modelled as steel pins. Values are mate degrees.

| Joint | Limits | Result |
|---|---|---|
| R1 yaw (Rev 5) | **−85…+75** | <ul><li>Carrier running gaps ≥ 0.200 over the whole range: cover3 0.200, index wrist segment 0.286, keeper 0.200.</li><li>Thumb clear on −85…−20 at R2 = 0. Beyond that, the V5 metacarpal outline meets cover3's wall strip, as in the baseline.</li><li>Thumb clear on the full −85…+75 at R2 = −45 and −90.</li></ul> |
| R2 (Rev 1) | −125…+25 | Clear. Gap ≥ 0.200; 0.53 at +25 after the relief. |
| R3 roll (Rev 2) | **−93…−23**, rest −58 | Clear, gap ≥ 0.200. The keeper is 0.200 free at both limits; stops engage at −98 and −18. The tendon path is clear over the whole range ([10]). |
| R4 (Rev 3) | 0…90 | At the CAD yaw and pitch, clear to 60°. From 67.5° the metacarpal curls into cover3's wall strip on the palm side. See [8] for other poses. |
| R5 (Rev 4), R6 (Rev 6) | −90…0 | Clear, gaps ≥ 0.205 and ≥ 0.200. |

**Flexion at the new clocking.** Synergy s: R3 = 90s, R4 = −90s, R6 = −90s. Fingers straight. First hit shown:

| Yaw \ pitch | 0 | −45 |
|---|---|---|
| −70 | clean to s 0.8; at 1.0 the proximal/distal reach the index and middle palm bones | clean to s 0.6; at 0.8 the proximal meets the palm bones and cover3 strip |
| −45 | clean to s 0.6; at 0.8 the metacarpal meets the cover3 strip | clean to s 0.6; at 0.8 the distal meets the ring and middle palm bones |
| −17 | 0.2 mm³ touch, metacarpal vs cover3, even unflexed (the yaw limit at pitch 0) | clean to s 0.6; at 0.8 the distal meets cover_back and the index wrist segment |
| 0 | blocked: the thumb lies over the cover3 strip | clean to s 0.6; at 0.8 the distal meets cover_back |

Leap needs about s ≤ 0.3 (R3 p98 27°, R4 −26°, R6 −22°). So the thumb is clean over the whole used range at yaw ≤ −45, and with pitch −45 at every yaw tested. The hits at s ≥ 0.8 are the thumb curling into the palm: a natural soft end of travel, which firmware can cap at about 60°.

**Flexion at the roll + limit** (roll −23, `work/thumb/roll_hi_check.log`):
- No hits on the index finger or the carrier arms at any pose.
- Clean to s 0.8 at yaw −70 and −45 (pitch 0), and at yaw −17 and 0 (pitch −45).
- Clean to s 0.6 at yaw −70 and −45 with pitch −45. The first hits at 0.8 are the distal reaching the ring and middle palm bones and the index wrist segment.
- At yaw −17 and pitch 0, a tiny metacarpal–cover3 touch appears from s 0.2. Yaw 0 at pitch 0 is blocked, as at the rest roll.
- Pitch sweep −125…+25 at roll −23: clear, with the flex stack ≥ 0.655 mm from the carrier.

**Opposition.** Grid over R1, R2, R3 (−93…−23) and the thumb flex synergy × the finger flex synergy. Each candidate is refined by backing the finger off until the distal pads just touch, then re-checked against everything.

| Pinch | Thumb | Finger | Contact | Pad gap | Other collisions |
|---|---|---|---|---|---|
| **Index pad** | yaw −60, pitch −5, roll −70.5, flex 50% (R3 45 / R4 −45 / R6 −45) | 82% (J1 74 / J2 −74 / J3 −74) | (−96.3, 4.0, 27.5) | 0.11 mm | none |
| **Middle pad** | yaw −60, pitch −55, roll −70.5, flex 50% | 79% | (−100.2, −17.0, 39.9) | 0.06 mm | none |

The yaw of −60 matches the Leap median of −57.3. The index pinch's roll (−70.5) sits within the passive range; Leap's index opposition fell at −58…−53. The grid step is 12.5°, so a nearby roll probably works too.

**Hard stops.**
- Yaw: engaged at −88.5 and +78.5, free by 0.200 at −85 and +75.
- Roll: engaged at −99 and −17, free by 0.200 at −93 and −23.

## 6. Strength (short-term estimates)

- **Yaw bearing.** 2.4 N·m gives at most about 20 MPa of journal edge pressure, even ignoring the thrust ring. The keeper takes about 300 N of lift.
- **R2 trunnions.** A 10 N sideways tip load gives about 92 N per trunnion.
  - M2 cantilever: about 400 MPa. **Use 12.9 alloy SHCS.**
  - 4×3 tube: about 40 MPa.
- **R4–R6.** A 10 N tip load gives about 200 N of joint reaction.
  - The M2 bolts in double shear see about 32 MPa.
  - PLA bearing is 17 MPa at R4 and 11 MPa at R5/R6.

## 7. Fasteners and purchased parts

`regions/fasteners_thumb.json` is in the world frame at the V6 rest. The R4–R6 bolts follow the re-clocked flex stack; their axis is (0.33, −0.32, −0.89).

| Name | Screw | Nut | Role |
|---|---|---|---|
| yaw_keeper | M2×14 SHCS | nyloc | cover3 |
| R2_pin | M2×8 SHCS | nyloc | The tip is the pivot pin; the length is intentional. |
| R3_keeper | M2×14 SHCS | hex nut | through the cross |
| R4_bolt | M2×14 SHCS | nyloc | R4 pivot |
| R5_bolt | M2×16 SHCS | nyloc | R5 pivot |
| R6_bolt | M2×16 SHCS | nyloc | R6 pivot |
| T3_mount ×2 | M1.6×3 SHCS | none | into the gearbox face |

Not in the hardware library, so modelled here:
- **R2 tendon-side pin:** a steel or brass tube, 4 × 3 × 8.9 mm, glued into the carrier arm only.
- **T3:** a straight N20 with an encoder.

## 8. Print and assembly

**Print orientation:**
- Carrier: lying on the flat +s arm face, with R2 vertical and supports.
- Cross: R2 vertical.
- Roll shaft: lying down, with the R4 hole vertical.
- Coupler: upright, PETG if you have it.
- cover3: as in V5.

**Assembly order:**
1. Seat the nuts.
2. Push the N20 up from below and fix it with 2× M1.6 through the cup.
3. Fit the coupler on the shaft.
4. Lower the carrier into the cup and fit the yaw keeper.
5. Insert the roll shaft into the cross and fit the R3 keeper at mid-roll.
6. Fit the cross between the arms: R2 pin on one side, glued tube on the other. Thread the tendons through the tube, cross, axial bore and branches.
7. Bones:
   1. Assemble the metacarpal with the R4 bolt so that its pad faces the tongue's pad-side branch. That is the re-clocked orientation, and it is fixed by the tongue.
   2. Fit the R5 and R6 bolts.
   3. Tighten each nyloc until the joint swings freely with no end play.

## 9. Coordination and open items

- **Placements hook (orchestrator).** `mod_thumb.placements()` returns {label: App.Placement} for the four flex-stack instances. `build_v6.py` has no hook for moving existing instances yet. Please add this after the relink step:

  ```python
  for label, pl in m.placements().items():
      for o in doc.getObjectsByLabel(label):
          o.Placement = pl
  ```

  Put the same in `write_manifest`/`load_manifest` for the GUI. Until then, `work/thumb/apply_rest.py` applies it to a built file.
- **rom_sweep (orchestrator).**
  - Use `"rest"` from `data/v6_joints.json` as θ0 for Revolute 2. The atan2 re-measure of the re-placed pose reads +66.8.
  - `attachments_orchestrator.json` attaches thumb_R2_tube to the cross. It is really glued in the carrier arm, but it is coaxial with R2, so the sweep gives the same result either way.
- **Actuation.**
  - T1/T2 tendons enter along R2 through the tube bore. The tube's outer end rides on r 14.0 about R1 at Z −0.47.
  - They run along the roll axis and out through the tongue's pad/back branches, 9–14 mm in front of G.
  - The flexion side of the thumb bones is now their local +z, which in world at the rest is 27.6° above −e2, toward the index. Channels you add to the `thumb_*` keys through `modify()` land on my shapes correctly, because build_v6 now composes module-major.
  - Anything on the T2 shaft must stay below Z −15.0 and within r 7.1 of the T2 axis.
- **Electronics.**
  - The draft battery and harness boxes overlap the T3 tower, which is solid there.
  - The T3 cable leaves through the open bottom of the pocket. The connector is at about (−165.3, 12.3, −51.8) and mates along −Z.
- **Open question for the user: M1.6 holes.** Check that your N20 gearbox has the 2 tapped M1.6 holes 9 mm apart. If not, glue the face to the floor.
- **Open question for the user: pitch and roll.** R2 (pitch) and R3 (roll) remain unactuated. The passive roll range is now 70° wide (−93…−23) and the index pinch uses roll −70.5. If the roll flops under load, a light torsion spring toward −58, or friction at R3, would hold it.
