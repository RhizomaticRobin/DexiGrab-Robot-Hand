# V6.1 proportions: human-like phalanx lengths

Owner: proportions agent. Files: `lib/mod_proportions.py`, `scripts/verify_proportions.py`, `regions/proportions.json`,
`data/v6_joints_v61.json` (frame overrides, to be merged by the orchestrator), fabrication data and test builds in
`work/proportions/`. One minimal edit in `lib/mod_hardware.py` (section 6).

## 1. What changed and why

The user decided to re-proportion the finger phalanges to their own hand. The source is the Leap Motion bones × 1.25
(`~/leap/reports/hand_size_vs_robot.json`). Palm, finger pitch and bone thickness are unchanged; the thickness is
intentional.

Joint-to-joint lengths in mm (distal = J3 axis to the fingertip), as built and measured in the assembled pose:

| Finger | Metacarpal J1→J2 | Proximal J2→J3 | Distal J3→tip | J1→tip |
|---|---|---|---|---|
| Index | 32.0 → **43.6** | 41.7 → **29.5** | 29.3 → **25.0** | 103.0 → 98.1 |
| Middle | 32.0 → **50.6** | 49.7 → **35.6** | 29.3 → **24.5** | 111.0 → 110.7 |
| Ring | 32.0 → **49.5** | 41.7 → **27.8** | 29.3 → **21.5** | 103.0 → 98.8 |
| Pinky | 25.0 → **32.9** | 31.7 → **25.0** | 29.3 → **21.0** | 86.0 → 78.9 |
| Thumb | 32.0 (unchanged) | R5→R6 41.7 → **33.0** | R6→tip 29.3 → **22.0** | R5→tip 71.0 → 55.0 |

Choices behind the table:
- **Pinky.** The Leap gives 26.3 / 19.7 / 13.9 mm, which is 32.9 / 24.6 / 17.4 after scaling. The distal is clamped to
  21.0, because the M1.6 anchor and the fingertip pad need 20–22 mm. The middle phalanx is grown slightly, from 24.6 to
  25.0, as the brief's table (~25) asks.
- **Thumb.** The brief's "R4→R6" is the `thumb_proximal` key (R5→R6, 41.7 mm now). The Leap gives 26.4 × 1.25 = 33.0
  for it, and 17.4 × 1.25 = 21.75 for the distal. I used the table's rounded 22.0. The thumb metacarpal is unchanged.
- **Minimums.** The shortest middle phalanx is 25.0 (limit 22). The shortest distal is 21.0 (limit 20–22).

## 2. Method: splicing the finished bones

`mod_proportions` runs after every module that shapes the bones:
- actuation: tendon channels, knot pockets, open-roof flexor slots and the distal M1.6 anchor
- tactile: pad recesses, folds, side curtains and dorsal spine channels
- thumb: the M2 bolts

`modify()` captures the shared bones and phalanx pads. `new_parts()` makes one variant per finger by splicing each part
in its own frame:
- **Metacarpal (lengthen):** cut at x_s, move the far part +d along x, and fill the gap with the section at x_s,
  extruded by d.
- **Proximal (shorten):** remove the slab [a, b] and move the far part back.
- **Distal (shorten):** remove slabs between the tip clamp and the J3 clevis, and move the tip portion +L toward J3.

Each bone keeps its proximal-side joint frame, and only the far joint or the tip moves:
- metacarpal J1 stays at x 7.5
- proximal J2 stays at x 7.5
- the distal J3 connector stays at x 29.3

**How the splice planes are chosen (at build time, from the received shape).** A face that does not contain the x
direction is anything other than:
- a plane whose normal is perpendicular to x
- a cylinder or extrusion along x

Any edge between two x-prismatic faces is parallel to x, so every change of section along x lies inside the x-extent of
such a face. The x-uniform ranges are the complement of those extents; `work/proportions/scan_sections.py` cross-checks
them with sections sampled every 0.25 mm. Each range is intersected with a semantic window:
- **Metacarpal:** between the knot pocket (ends at x 20.5) and the J2 clevis floor (j2 − 7.7).
- **Metacarpal pad:** before its J2-loop anchor (j2 − 12.06), so that the anchor moves with J2.
- **Proximal:** between the knot pocket and the J3 flexor slot (j3 − 7.7).
- **Distal:** between the M1.6 nut slot (ends at x 8.33) and the J3 clevis back wall (x 21.4).
- **Distal pad:** between its wrap start (8.2) and its J3-loop anchor (17.09).

A plane that no longer fits makes the build fail loudly. Every splice is also proven at build time by comparing the
section just before and just after it (`work/proportions/splice_proof.json`).

Splice locations (part-local x, mm):

| Part(s) | Uniform range used | Splice |
|---|---|---|
| metacarpal → index / middle / ring | 20.5–30.5 | insert 11.6 / 18.6 / 17.5 at x 25.5 |
| metacarpal_pinky → pinky | 20.66–23.5 | insert 7.9 at x 22.08 |
| proximal → index / ring | 20.5–40.2 | remove 24.25–36.45 (12.2) / 23.4–37.3 (13.9) |
| proximal_middle → middle | 20.5–48.2 | remove 27.3–41.4 (14.1) |
| proximal_pinky → pinky | 20.5–30.2 | remove 22.0–28.7 (6.7) |
| thumb_proximal → thumb | 20.5–40.2 | remove 26.0–34.7 (8.7) |
| distal → index / middle | 13.32–20.0 | remove 14.51–18.81 (4.3) / 14.26–19.06 (4.8) |
| distal → ring / pinky; thumb_distal → thumb | 13.32–20.0 and 10.0–13.3 | remove 13.57–19.75 (6.18) plus 10.84–12.46 / 10.59–12.71 / 11.09–12.21 |
| metacarpal pads (index / middle / ring) | 17.5–27.44 | same plane as the bone (x 25.5) |
| metacarpal_pinky pad | 17.5–20.44 | insert at x 18.97 (the bone's plane lies beyond the pad's loop anchor) |
| proximal pads | 16.7–40.0 (middle 48.0, pinky 30.0) | same slab as the bone |
| distal pads (all five) | 8.2–17.09 | own slab, centred at x 12.64 (4.3 / 4.8 / 7.8 / 8.3 / 7.3) |

**Why the ring, pinky and thumb distals need a second slab.** Only 6.68 mm of the distal is uniform between the tip clamp
and the recess end, but those three need 7.3–8.3 mm removed. The second slab lies in x 10.0–13.3. No transverse feature
crosses that range: it lies beyond the nut slot and between the ends of the blind channels at x 10.0 and 13.3. Removing
it only shortens two blind tendon passages (V5 channels at z ±0.95) and the Ø2.1 bore beyond the screw tip, from 13.3 to
11.2–12.2. The clamp itself is untouched: M1.6×3 screw seated at x 5.3, nut and side slot at x 6.5–8.3.

**Two near-x features in the sources.** Their tilt makes the sections at the two sides of a splice differ very slightly
(section symmetric difference):
- **Middle proximal:** the V5 back channel (r 1.0, inside actuation's Ø2.1 extensor channel) is tilted 0.28°. Removing
  14.1 mm leaves a 0.07 mm step on a crescent under 0.11 mm thick inside the extensor channel. Section difference
  0.127 mm², volume bookkeeping +0.85 mm³. Functionally irrelevant for the elastic cord.
- **Distal:** the M1.6 bore is tilted 0.058°. At the second slab this gives 0.002–0.004 mm².

The UnifySameDomain refinement (`removeSplitter`) fails on the received distal before any splice. It has two
sphere-capped dorsal corners at the tip. So the spliced distals keep split coplanar faces at the junction; they are valid
single solids.

## 3. Instances, poses and joint frames

**Part keys.** There are 28 new keys:
- `proportions_{metacarpal,proximal,distal}_{index,middle,ring,pinky}`
- `proportions_{proximal,distal}_thumb`
- `proportions_pad_…` for the matching phalanx pads

**`relink()`.** It points these instances at their variants:
- every finger bone instance: Metacarpal <1..3>, Metacarpal_pinky, Proximal <1>/<2>, _middlefinger, _pinky, Distal <1..4>
- the thumb's Proximal <3> and Distal <5> (this overrides mod_thumb's relink, which comes earlier)
- the 14 `tactile_pad_<finger>_{meta,prox,dist}` instances

It is unconditional. So it survives `build_v6.write_manifest()`, which reloads the modules and so loses state kept from
`modify()`.

**`placements()`.** These are pure translations, so no joint angle changes. For each finger:
- **J2 shift t2** = metacarpal rotation × (Δmeta, 0, 0). It moves the proximal, its pad and the J2 loop.
- **J3 shift t23** = t2 + proximal rotation × (Δprox, 0, 0). It moves the distal, its pad and the J3 loop.
- **Thumb:** t23 is composed on mod_thumb's re-clocked rest, and moves Distal <5>, its pad and the J3 (R6) loop by
  8.70 mm toward R5.

The tactile base poses come from `mod_tactile.instance_specs()`. So the knuckle ribbons, the wrist tails and the thumb
tail are untouched: they start at the metacarpal's knuckle exit (u 17.5), which does not move.

**Joint frames.** Nine mates have part-local connector origins that moved: Revolute 16, 13, 11 and 9 (the metacarpal J2
connector, +Δmeta), Revolute 17, 14, 12 and 10 (the proximal J3 connector, +Δprox), and Revolute 6 (the thumb proximal
R6, −8.7). They are in `data/v6_joints_v61.json`, in the same format as `data/v6_joints.json`.
- At the coordinator's request, the shared `data/v6_joints.json` was restored byte for byte, to the thumb entries only.
  The orchestrator merges the v61 file when it loads V6.1 into the master, so that geometry and frames change together.
- For Revolute 10 (pinky J3) the moved connector is entity 1, which is the pivot `rom_sweep` uses. Applied to the old
  geometry, it gave false collisions.
- `mod_proportions.write_merged()` writes a merged copy to `work/proportions/v6_joints_merged.json`.
  `work/proportions/rom_sweep_v61.py` is `scripts/rom_sweep.py` with only its overrides path configurable
  (`V6_JOINTS_JSON`), for this topic's ROM runs.

## 4. Other modules

**Tactile.**
- The 14 phalanx pads are spliced the same way as their bones. Each one is identical, to 0.00000 mm³ of symmetric
  difference, to the pad `mod_tactile` itself makes at the new length. The check is `patch_tactile()` +
  `pad_part()`, in verify [5].
- The pads stay seated: 0 overlap, 0 distance to the recess floor.
- Every spine sheet still ends at its J2/J3 loop anchor. All 23 loop-to-pad and ribbon-to-pad contacts of the moved
  and relinked parts are the tactile design's FPC butt joints, with exactly the tactile agent's volumes (≤ 0.0097 mm³).
- The fabrication data were regenerated into `work/proportions/tactile_fab/` by `work/proportions/make_fab.py`. It is the
  tactile agent's generator, unchanged except that `mod_proportions.patch_tactile()` installs the per-finger pads, the
  moved joints and the re-posed instances, and it writes into this folder. The outputs are:
  - `dxf/` (32 files)
  - `taxel_map.csv`, `readout_table.csv`, `fab_summary.json`, `taxel_map.png`

  The distal pads keep their fingertip at u 0 in the pad frame, as the generator assumes; `canon_matrix()` maps them onto
  the shortened distal.

Taxels (FlexiTac 2 mm pitch, 6 rows per digit pad):

| Board | Ray | Taxels (was) | Rows | Digit columns: distal + proximal + metacarpal | Lines available (J3 loop / J2 loop / ribbon) |
|---|---|---|---|---|---|
| 1 | thumb | 101 (148) | R1–R6 | 5 + 7 + 6 = 18 | 9 / 20 / 26 |
| 2 | index | 295 (308) | R1–R12 | 7 + 5 + 12 = 24 | 9 / 20 / 26 |
| 3 | middle | 361 (364) | R1–R12 | 7 + 8 + 15 = 30 | 9 / 24 / 30 |
| 4 | ring | 329 (340) | R1–R12 | 5 + 4 + 15 = 24 | 9 / 20 / 26 |
| 5 | pinky | 236 (261) | R1–R12 | 5 + 3 + 6 = 14 | 9 / 15 / 18 |

- The total is **1322 taxels** (was 1421).
- Every ray still fits one 16 × 32 reading board: at most 12 rows and 32 columns.
- Loops, spines, knuckle ribbons and tails keep the tactile widths, which now carry spare lines. The middle finger uses
  exactly the 30 lines of its ribbon and spine.
- Flat ray lengths: thumb 299, index 458, middle 547, ring 443, pinky 480 mm (were 315 / 463 / 547 / 447 / 487).
- No ray flat pattern overlaps itself.

**Actuation.** The moment arms are unchanged, so the excursions and spools are unchanged. Straight-finger flexor lengths
are below. The palm and base portions come from `notes/actuation.md` §7; the finger portions come from the new joint
positions (clevis floors 7.7 from the axes, knot pockets at x 19, distal clamp about 15.6 mm from the J3 floor).

| Finger | A (J1) | B (J2) | C (J3) |
|---|---|---|---|
| index | 92.8 (unchanged) | 165.8 → 177.4 | 187.3 → 182.4 |
| middle | 92.8 | 165.8 → 184.4 | 195.3 → 195.0 |
| ring | 92.8 | 165.8 → 183.3 | 187.3 → 183.1 |
| pinky | 92.8 | 158.8 → 166.7 | 170.3 → 163.2 |

- **Thumb.** T1 is unchanged. T2 (R5 + R6 flexor) is 16.0 mm shorter in the thumb: its finger portion goes from 57.3 to
  41.3 mm.
- **Elastic extensors.** The cords in the back channels change by the same amounts. They were not modelled before either.

**Thumb and hardware.**
- The R6 bolt request in `regions/fasteners_thumb.json` is a world-frame request written at the old rest pose. It is
  moved by `mod_proportions.move_world_request()`, which `mod_hardware` calls (section 6). The seat moved 8.700 mm and
  lies 0.0000 mm off the moved R6 axis.
- The R4 and R5 bolts do not move.
- `attachments_orchestrator.json`: the R2 tube rides on the yaw carrier, which is not moved.

## 5. Verification

The test build is the full chain j0, pillars, thumb, actuation, shells, fixes, tactile, proportions, hardware, run in
three stages so that each freecadcmd run stays under the 8-minute rule. The builder is
`work/proportions/build_staged.py`: stage A1 took 5:31, A2 1:20 and B 1:40. It composes modules exactly as
`build_v6.build()` does, and runs every module's relink, placements and instances hooks live, in pipeline order, on the
finished document.
- Output: `work/proportions/test.FCStd`.
- A twin without proportions, built from the same stage-A shapes, is `test_noprop.FCStd`. It is the reference for
  hardware and range of motion.
- **Electronics was left out.** Its stage alone takes about 12 minutes, its agent is still changing it, and it does not
  touch the phalanges.

Results (`scripts/verify_proportions.py`; logs and JSON in `work/proportions/verify_*`):

| Check | Result |
|---|---|
| [1] New parts | 28, all valid single solids. No instance is left on a shared phalanx part. Relinks match. |
| [2] Lengths in the assembled pose | Every target is hit with **0.0000 mm** error. The J2 and J3 connectors of the two mates coincide (0.00000 mm). Rotation change of every moved instance: 0°. J1 unchanged. The frames in `v6_joints_v61.json` match. |
| [3] Splice proof | Every kept source piece equals its translated counterpart in the variant: **0.00000 mm³** symmetric difference. Every inserted slab equals the extruded section: 0.00000 mm³. The junction sections are identical (0.00000 mm²), except for the two near-x tilts in §2 (0.127 mm² middle proximal; ≤ 0.0043 mm² distal second slab). |
| [4] Overlaps and gaps at rest | The moved and relinked instances collide with nothing. There are 23 designed contacts, all ≤ 0.0097 mm³: FPC butt joints, and pads and ribbons at their excluded flexible crossings. Joint running gaps are unchanged from the reference relative pose: fingers J1 0.300, J2 0.300, J3 0.400 mm; thumb R5 0.205, R6 0.200 mm. |
| [5] Tactile | Pads equal mod_tactile's own pads at the new length (0.00000 mm³). Pads are seated: overlap 0, distance 0. The 10 loops sit on the new joint frames: offset 0.00000 mm, angle 0°. |
| [6] Hardware | All 28 joints have the same status and the same problem texts as the no-proportions twin. The R6 bolt seat moved 8.700 mm, 0.0000 mm off the new axis. |
| [7] Combined curl (J1 = J2 = J3) | All four fingers are clear at 45°, 67.5° and 90°. At 90°/90°/90° the fingertip stops short of its palm pad by 10.9 (index), 18.4 (middle), 20.3 (ring) and 4.5 (pinky) mm. With the V5 proportions, every distal drove into the palm pad and palm bone at 90°/90°/90° (up to 1430 mm³). |

**Range of motion.** `rom_sweep_v61.py` was run on `test.FCStd` with the merged overrides:
- **Finger joints.** All finger J1–J3 (Revolute 7–18) are **clear over 0–90°**, both at the standard 9 steps and at 5°
  steps (`rom_fingers_5deg.log`).
- **Thumb.** R1, R2 (−93 to −23), R5 (Revolute 4) and R6 (Revolute 6) are clear. R4 ≥ 67.5° (metacarpal against
  cover3) and the yaw hits are identical to the twin without proportions. They are pre-existing, on the unchanged thumb
  metacarpal. In the yaw sweep with the thumb unflexed, the shorter thumb's proximal and distal intrude less at most
  angles.
- **J0 neighbour contacts.** These are the first contact above 0.5 mm³, from 1° sweeps before → after. The partner is
  still the neighbour's fingertip; the interference at first contact is smaller (for example middle −11°: 162 → 100 mm³).

| J0 | toward the thumb (+) | toward the pinky (−) |
|---|---|---|
| index | none within +15 → none | −11 → −11 (now the index distal touches the middle proximal first) |
| middle | +11 → +11 | −11 → −11 |
| ring | +11 → **+12** | −11 → −11 |
| pinky | +12 → +12 | none within −15 → none |

## 6. Edits outside this topic (and restorations)

- **`lib/mod_hardware.py`**, a minimal edit commented `# V6.1 proportions:`. It adds `_proportions_move()` and one call in
  `request_joints()` for world-frame requests. When `proportions` is in the build, a request seated on a bone that this
  module moved is moved with it. Today that is only the thumb R6 bolt. Without `proportions` it is the identity.

  It is needed because hardware creates its instances after this module's `placements()` run, and it snaps each request
  to the hole near the requested point. A pre-moved world request in another file would have left the thumb's own R6
  entry as a failing phantom bolt.
- **`regions/hardware_report.json` was overwritten by mistake** by my first test build: mod_hardware writes it on every
  build unless `V6_HW_REPORT` is set. I regenerated it with `mod_hardware.plan()` on `work/integrate_v6b/built.FCStd` —
  the build that had written the previous version — and marked the stage accordingly. Another agent's build has since
  rewritten it. My builds now write to `work/proportions/`.
- **`data/v6_joints.json`.** My frames were first merged here as the brief said. At the coordinator's request they were
  moved to `data/v6_joints_v61.json`, and the file was restored byte-identical to its previous content. It has since been
  updated by the thumb agent (Revolute 2 hi −23).

## 7. Assumptions and open questions

1. **`build_v6.load_manifest()` (the GUI path) applies relink and placements before the instance specs.** For the
   instances that modules create — here the 14 tactile pads and 9 tactile loops — tactile's specs then overwrite this
   module's relinks and placements. In the GUI those pads would snap back to the shared pads at the old positions.
   - `build()` (headless) is correct, because hooks run per module in order.
   - Fix (orchestrator's file): in `load_manifest`, apply `man['relink']` and `man['placements']` after the
     `for spec in man['instances']` loop.
   - Also, `write_manifest()` reloads the modules, so `mod_thumb.relink()` (state from `modify()`) comes out empty in
     the manifest (`"relink": {}` in `build/manifest.json`). This module's relink is unconditional for that reason.
2. **Pre-existing hardware failures.** The thumb R5 and R6 bolts fail hardware validation in the reference build too, with
   and without this module: "nut not seated: no material under the nut bearing face" and "the screw passes through
   Proximal Phalanx Bone_V02 <3> only". This is for the thumb and hardware owners. The R6 bolt keeps its exact relative
   position.
3. **Stale tactile region boxes.** The world boxes of the tactile loops in `regions/tactile.json` are at the old joint
   positions; `regions/proportions.json` has the new ones. The remaining box-level overlaps are with the same objects
   (tactile loops, thumb chain) and one AABB corner of the pinky knuckle ribbon's static segment; there is no solid
   contact (verify [4]).
4. **Opposition changes.** Thumb-to-finger opposition changes, because the thumb is 16 mm shorter and the finger J2
   joints are 8–19 mm further out. The thumb agent's opposition fit (index pad at R3 45 / R4 −45 / R6 −45) should be
   re-run with V6.1.
5. **Palm contact at full curl.** With human-like proportions the fingertips no longer reach the palm at J1 = J2 = J3 =
   90° (4.5–20 mm short). This is fine for grasping, and it removes the V5 self-collision. A power grasp that presses
   the fingertips into the palm would need J1 beyond 90°.
6. **Tactile tables.** `mod_tactile`'s own tables still describe the shared pads, and the variants exist only in this
   module. If tactile later changes a pad or recess, this module re-splices whatever it receives. The build fails loudly
   if a splice window no longer holds a uniform range. The fab data must then be regenerated by running
   `work/proportions/make_fab.py`.

## 8. Print and assembly notes

**Printing.**
- Print the new bones like the V5 ones, with the palm face up where the other owners allow it. The recess floors then
  print as top surfaces.
- They are 28 distinct parts now. Every phalanx is finger-specific, and so is every phalanx pad.
- Nothing about the joints changes: the same clevises and tongues, the integral pins at J1–J3, and the thumb M2 bolts
  (R6 is still M2×16 with a nyloc).

**Tendons and elastics.**
- Cut tendons B and C to the new lengths in §4, plus your knot allowance.
- The distal clamp is unchanged: an M1.6×3 screw with an M1.6 nut in the side slot.
- In the ring, pinky and thumb distals the blind tendon-tail passage beside the screw is 1.1–2.1 mm shorter. Trim the
  tail after clamping.
- The elastic extensor cords change by the same amounts as tendon B (metacarpal) and C.

**Tactile.**
- The FPC rays must be re-ordered from `work/proportions/tactile_fab/dxf/`: all phalanx pads changed, and so did the ray
  flat patterns. The loops, knuckle ribbons and tails are unchanged.
- The host map for the boards is `work/proportions/tactile_fab/taxel_map.csv`.
- FlexiTac-derived data remain CC BY-NC 4.0; see `notes/tactile.md`.

## 9. Files

**Deliverables:**
- `lib/mod_proportions.py`: the module. `modify` / `new_parts` / `relink` / `placements`, plus these helpers:
  - `write_joint_frames`, `write_merged`
  - `patch_tactile`
  - `move_world_request`
- `scripts/verify_proportions.py`: sections 1–7, selected with `V6_SECT`.
- `notes/proportions.md` (this file).
- `regions/proportions.json`: 5 world boxes of the re-posed segments and 28 local boxes of the new parts.
- `data/v6_joints_v61.json`: 9 frame overrides, for the orchestrator to merge into `data/v6_joints.json`.
- `work/proportions/tactile_fab/`: `dxf/`, `taxel_map.csv`, `readout_table.csv`, `fab_summary.json`,
  `taxel_map.png`.
- `lib/mod_hardware.py`: the `# V6.1 proportions:` hook (section 6).

**Tools in `work/proportions/`:**
- `build_staged.py`: staged builder, same composition and hook order as `build_v6.build()`.
- `scan_sections.py`, `inspect_faces.py`: the uniform-range analysis.
- `unit_splice.py`: module run on BREPs.
- `make_fab.py`: the tactile generator, adapted.
- `publish_joints.py`
- `make_regions.py`
- `hw_report.py`: `mod_hardware.plan()` into this folder.
- `rom_sweep_v61.py`: `rom_sweep.py` with a configurable overrides path.

**Test builds and results in `work/proportions/`:**
- `test.FCStd`: the final build. `test_noprop.FCStd`: the twin without proportions.
- `stageA1/`, `stageA2/`: stage BREPs, for a quick rebuild of stage B.
- `splice_proof.json`
- `verify_*.json` / `.log`, `rom_*.csv` / `.log`, `hardware_report_{test,noprop}.json`.

**Rebuild:**

```
P_STAGE=B P_IN=work/proportions/base.FCStd P_DIR=work/proportions/stageA2 P_OUT=work/proportions/test.FCStd \
  freecadcmd work/proportions/build_staged.py
```
