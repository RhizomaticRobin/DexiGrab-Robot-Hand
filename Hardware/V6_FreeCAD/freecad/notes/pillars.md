# Pillars: palm-bone reinforcement through the TPU palm (requirement 6) and TPU clearance fixes

Module `lib/mod_pillars.py`. Check `scripts/verify_pillars.py`, which **passes with 0 failures** on `work/pillars/test.FCStd`. Claimed regions are in `regions/pillars.json`.

Pipeline order: `j0, pillars, …`. Build: `V6_MODULES=j0,pillars`, about 90 s, 0.45 GB peak.

## 1. Problem (measured)

The TPU palm is 8 pieces that touch edge to edge and fuse into **one connected sheet** (verified):
- 4 strips (`Palm1`, `Palm2_2`, `Palm3_2`, `Palm_pinky`)
- 3 webs (`Palm2`)
- the wrist plate (`palm_bone_flex`)

The sheet is 0.6 mm thick and lies in one plane, world z −1.623 … −1.023. In bone-local coordinates that is y 0.5 … 1.1, with the palm side at −y. Its Ø2 × 3 pegs point toward the back of the hand.

In every palm bone this layer is empty from the wrist end to x = 71, where the J0 block starts. The palm half and the back half of the bone therefore joined only at the J0 block, so the full ~70 mm length acted as two thin beams. That is the break at the strip ends the user reported.

The wrist segments had the same split across their finger-side bar. There the back half-bar is about 2 mm thick.

TPU interferences in V5 + j0: the coordinator's audit list. Two causes:
- The ring and pinky wrist ends were never slotted for the plate and the ring strip.
- The webs ran through 0.07–0.17 mm skins on the +z sides of the middle and ring bones.

## 2. What the module does

All dimensions are in mm. Clearance is C = 0.2 per side.

### Palm bones (only the TPU layer, the peg holes and the pillars change)

1. **Pocket.** Each TPU instance near the bone is sectioned at 3 heights in the layer. The union of those sections is offset by C and cut through the layer (y 0.5 … 1.1). This does four things:
   - slots the ring and pinky wrist ends;
   - removes the web skins;
   - gives the insert 0.2 mm in-plane clearance to every pocket wall (the J0 block face at x ≈ 71.2, and the solid index/pinky end caps);
   - does not touch the halves' faces. The sheet is sandwiched: contact top and bottom, no overlap.
2. **Pillars.** PLA fills the layer over each footprint. The footprint is fused 0.15 mm into both halves, where the material already exists.

| bone | pillars (bone-local) | PLA cross-section in the TPU mid-plane, left of the J0 block (mm²) |
|---|---|---|
| index | rib 1 polygon x 22.4–25.4, z −4.4…4.2 (a band on the wrist side of the cover3 nut pocket I3); rib 3 polygon x 62.1–67.1, z −6.2…5.8 | 19.3, 45.7; plus the existing solid wrist end 32.1 |
| middle | stadiums at x 25.6 (5.8 × 5.4), 45.5 (5.75 × 5.4), 66.0 (6.95 × 5.4); wrist end x 1.55–4.5 × z ±4.8 (through the plate) | 25.1, 24.8, 31.3, 26.5 |
| ring | the same as middle, shifted by z +0.1 … 0.15; wrist end x 1.65–4.5 × z ±4.95 | 24.8, 24.9, 31.3, 26.5 |
| pinky | polygons at rib 1 (x 22.4–27.1), rib 2 (x 42.3–48.3) and rib 3 (x 62.2–68.3) | 26.6, 27.5, 46.3; plus the existing solid wrist end 44.9 |

The J0 block is 221–237 mm² in every bone.

**Result:** the mid-span piece of each bone (x 7…70, with the wrist end and the J0 block removed) is now **one solid**. In V5 + j0 it was 2: the split halves.

### Placement rules

Placement was done on a 0.05 mm raster (`work/pillars/scratch/raster_opt2.py`, `poly_pillars.py`). It uses the V5 bone after the pocket cut and 30 section levels 0…1.2 mm deep into each half.

- **3D wall ≥ 1.2 mm** (raster margin 1.25) from every void of either half: seat windows and troughs, peg holes, cover2/cover3 screw bores. This includes the **hardware keep-outs of the three index bracket M2×5 screws with captive nut pockets** (`regions/hardware.json`). No pillar and no TPU hole lies over a nut pocket; the minimum 3D distance is 1.26 mm.
- **TPU ligament ≥ 1.2 mm** around every hole (hole = pillar + 0.2): to windows, sheet edges and every peg.
  - The wrist-end holes stop at x ≤ 4.7. That keeps 1.2 mm to the plate/strip seam at x ≈ 5.93, so the design also works if the TPU is printed as separate pieces.
- **Pillar width.** Stadiums are at least 5.4 mm wide. Polygons are at least 2.4 mm wide (opening radius 1.2, then a 0.1 mm inset).
- **Index rib 2 has no pillar.** Its nut pocket I2 fills the rib centre, and nothing ≥ 2.4 mm wide fits with 1.2 mm walls.
- **Actuation.** The current magazine design (`regions/actuation_motors.json` of 21:28) keeps every motor envelope, with `n20.body_envelope(0.2)`, cap, 8 mm plug keep-out and shaft, **≥ 6.4 mm** from all pillars.

### Peg holes

The V5 holes were Ø2.5 but up to 0.08 mm off the peg axes. They are reamed to r = 1.21 about each peg's own axis.

### Wrist segments (Palm_bone1 frame, TPU layer at z −1.1 … −0.5)

- **Pocket.** The same C-offset pocket rule. It fixes the 0.03–0.08 mm³ plate slivers.
- **Bar pillars.** A bar pillar fills the layer in the finger-side bar between the two plate pegs:
  - `wrist_middle`: 14.1 mm²
  - `wrist_ring`: 17.0 mm²

  The pillar stops 0.2 mm inside the finger face, so the plate's exposed hinge strip keeps its full width.
- **`wrist_index` and `wrist_pinky`** get no bar pillar. Their two pegs are 3.9 mm apart. Their outer sides are already solid through the layer, since the plate spans only x 6…56.
- PLA in the layer per segment: 92.0, 103.6, 103.0 and 97.6 mm².

### TPU parts

- **Holes** are the pillar footprint plus 0.2.
- **Pegs** are shortened by 0.25, so each peg tip has ≥ 0.2 mm to the bottom of its hole.
- **Webs are unchanged.** `tpu_web` is not rebuilt; it is shared by 3 instances and no hole reaches it.
- Volumes before → after (mm³):

| part | before | after |
|---|---|---|
| index | 545.0 | 495.0 |
| middle | 545.0 | 484.7 |
| ring | 545.0 | 484.8 |
| pinky | 428.0 | 353.9 |
| wrist | 502.3 | 436.9 |

## 3. Verification (`verify_pillars.py`, assembled pose, link placements)

- **Validity.** All 14 parts pillars touches are valid single solids. Bone volumes change by +37 / +57 / −6 / +44 mm³ versus j0; the ring loses its non-physical overlapping end cap.
- **Halves joined.** The mid-span is 1 solid for all 4 bones. Every pillar footprint is 100 % backed by material just below and just above the layer, so it prints continuously.
- **TPU × PLA:** 24 pairs, **maximum overlap 0.000 mm³**, **minimum in-plane gap 0.200 mm**.

Coordinator audit list, before → after:

| pair | overlap before (mm³) | overlap after (mm³) | in-plane gap after (mm) |
|---|---|---|---|
| ring × palm_bone_flex | 44.46 | 0 | 0.200 |
| ring × Palm3_2 | 23.57 | 0 | 0.200 |
| pinky × palm_bone_flex | 15.25 | 0 | 0.200 |
| middle × Palm2 <1> | 5.22 | 0 | 0.294 |
| ring × Palm2 <2> | 2.04 | 0 | 0.284 |
| middle × Palm2_2 | 1.44 | 0 | 0.200 |
| middle × Palm1 | 0.09 | 0 | 3.5 |
| ring × Palm2_2 | 0.025 | 0 | 3.4 |
| Palm_bone1 <1>/<2>/<4>/<5> × plate | 0.03–0.08 | 0 | 0.200 |
| pinky × Palm2 <3> | near-touch | 0 | 0.273 |
| pinky × Palm3_2 | near-touch | 0 | 5.1 |

- **Pillars to TPU:** gap ≥ 0.200 mm.
- **32 pegs to their holes:** ≥ 0.204 mm, radial and at the tip.
- **TPU sheet:** still 1 connected solid, and the webs are unchanged.
- **Range of motion:** not affected. No joint geometry changed; the J0 region beyond x 71.2 is untouched.

## 4. Printing: orientation and pause (decision)

**Print the palm bones and the wrist segments back-side down (palm face up).**

The pegs then drop into their pre-printed Ø2.42 holes. That is the V5 intent: clearance holes the same depth as the pegs. The pillars are printed from the back half up through the TPU layer.

- **Pause** after the layer whose top is the **TPU palm-side face**: world z −1.023, bone y 0.5, wrist-frame z −0.5. At that point the pillars stand exactly 0.6 mm tall, flush with where the sheet's top will be.
- **Pocket floor:** world z −1.623.
- **Layer grid:** keep 0.2 mm layers across the pocket, so both heights fall on layer boundaries.

| part (as bed face) | bed plane | pocket floor at | **pause after** | notes |
|---|---|---|---|---|
| palm bone, V5 back (j0 + pillars) | bone y +7.5 (world z −8.023) | 6.40 mm | **7.00 mm** (layer 35 at 0.2) | pillars = layers 6.4–7.0 |
| palm bone + actuation magazine (its region reaches bone y 35.3) | bone y +35.3 | 34.20 mm | **34.80 mm** | re-check once actuation is final |
| wrist segment | wrist z −9.0 (world −9.523) | 7.90 mm | **8.50 mm** | use a 0.3 mm first layer so 7.9 and 8.5 fall on layer boundaries |
| all palm parts in one job (needed if the TPU palm is one piece) | world z −9.523 | 7.90 mm | **8.50 mm** | V5 palm bones then start 1.5 mm above the bed and need support |

General rule: pause height = −1.023 − z_bed (world). Pocket floor = −1.623 − z_bed.

**Pause procedure:**
1. Remove strings from the pillar tops.
2. Lay the TPU palm in, pegs down. The holes slide over the pillars with 0.2 mm clearance. Press flat; the sheet must sit flush with the pillar tops.
3. Resume within a few minutes, so the pillar tops are still warm.
4. Print the first layer after the pause about 5 °C hotter and at about 50 % speed, for PLA-on-TPU adhesion and pillar-top fusion.
5. If your TPU prints thicker than 0.60 mm, add that excess as a z-offset for the resume.

**TPU palm:**
- Print it as one flat TPU part (the 8 bodies fuse into one), sheet down, pegs up. Pegs are 2.75 mm after the trim.
- The sheet must be 0.60 mm: 3 × 0.2 mm layers.

## 5. Open items for the orchestrator and user

1. **Print orientation vs actuation.** `lib/mod_actuation.py` says the magazine is "printed on the back half after the TPU pause", which implies palm-side down. Palm-down makes the TPU pegs stand 2.75 mm up during the pause, where the nozzle hits them.
   - Either keep back-side down (my decision above), with the magazine on the bed, which actuation must check for bridging and supports;
   - or flip the pegs to the palm side. That is a TPU and peg-hole change I can make, but the 3 index +z pegs would then sit about 0.3 mm from the cover3 nut pockets. They would have to move or be dropped; the pillars already lock the strip.
2. **Window pillars (big gain).** Actuation fills the old worm seats, but leaves y 0.25…1.35 open (from my draft region). That leaves a 1.1 mm void to bridge in each TPU window.
   - If the fill reaches exactly y 0.5 and 1.1 (my final region), the whole TPU window can become a pillar: about 3 × 100 mm² per bone, with no new TPU holes and 0.2 mm to the window edges.
   - It must be added after actuation in the pipeline, or by actuation.
3. **Index rib 2 has no pillar** (cover3 nut pocket I2). The index palm half is clamped to cover3 by the three bracket screws. Its halves are joined at the wrist end, rib 1, rib 3 and the J0 block.
4. **TPU assumed one printed piece.** The holes also respect the plate/strip seam, so separate pieces work too.

## 6. Files

- `lib/mod_pillars.py`
- `scripts/verify_pillars.py`
- `regions/pillars.json`
- `notes/pillars.md`
- `work/pillars/` (`test.FCStd`, `build.log`, `verify.log`, `scratch/` rasters, plots, and the placement scripts `raster_opt2.py` and `poly_pillars.py`)
