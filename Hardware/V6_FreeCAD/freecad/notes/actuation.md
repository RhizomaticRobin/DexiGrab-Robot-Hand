# Actuation (V6): straight N20 gearmotors, motor magazines, spools and tendons

Module `lib/mod_actuation.py`. Check `scripts/verify_actuation.py` (results in section 9). Published data:
`regions/actuation_motors.json` (world pose of all 19 motors incl. T3 from mod_thumb), `regions/actuation.json`
(claimed regions), `regions/fasteners_actuation.json`. Test build: `V6_MODULES=j0,pillars,thumb,actuation`
(about 3 min, one freecadcmd), then `freecadcmd scripts/verify_actuation.py`.

## 1. Why the V5 seats could not be reused

The V5 seats hold a worm GM12-N20 inclined 30.7° with its can in the palm bone and its perpendicular output shaft
(and the Ø21 pulley) hanging into the cavity. A straight N20 puts the spool on the motor axis, so:

- **In the old seat, the encoder end sits inside the bone.** The 6-pin plug mates along the axis, so its 8 mm plug
  space would point up through the palm face. That is exactly the "no room for the wiring" the user saw on the
  index and pinky; on the middle and ring it only looked better because the cable could escape sideways.
- **Reversed in the old seat**, the spool ends up inside the bone and its tendon leaves the spool at 60° down.
- **Vertical motors** (axis along the old worm shaft) need about 50 mm of depth each and fill the whole bowl.

## 2. Layout chosen: one 2×2 magazine under every palm bone

Each finger's four motors lie **along its own palm bone** (bone-local x), in two tiers:

| motor | tier | joint | spool (tendon axis radius) | notes |
|---|---|---|---|---|
| A | 1 (under the bone), front | **J1** flexor | small, r 3.75 | encoder toward the fingertips |
| B | 1, rear | **J2** flexor | small, r 3.75 | |
| C | 2 (under tier 1), front | **J3** flexor | big, r 5.65 | index C: tab turned to −z |
| D | 2, rear | **J0** loop (2 grooves) | big, r 5.65 | |

- **Why per finger:** the palm bones float on TPU (the palm arches). With every motor, spool and guide on the bone it
  drives, the tendon path from spool to finger never changes when the palm deforms. It also keeps the side walls free
  for the driver boards.
- **Orientation:** motor axis along the bone, flats vertical, encoder end toward the fingertips, PCB tab and plug toward
  bone-local +z (the thumb side). The index C motor has its tab toward −z because its +z plug space touched Shell1.
  - Every plug therefore points into the open inter-finger gap beside the magazine, 6–11 mm off the motor axis. Its
    cable drops straight down to the harness.
  - The adjacent finger's magazine is on the other side of that gap, and no two tabs face each other in the same gap.
- **Magazine:** PLA fused to the underside of the palm bone. It is printed as part of the bone (section 8).
  - Bays open downward. Each gearbox sits between two 0.6/0.8 mm **fork ribs** (a U-slot for the boss, and the can
    profile behind), which fix it axially and against the motor torque.
  - Two 1 mm **side lanes** run beside tier 1, and there is a 1.7 mm wall with the J0 tunnels in it. Roof strips
    (y 3.6–5.2, cut round the TPU peg holes) close the tunnels where the bone's rounded corners are thin (pinky).
  - An **under-J0 guide block** sits in front of motor A, below the J0 nut. A descent boss over A's magnet carries the
    A/B drops.
  - Near J0 the magazine stays out of the base-bone tongue's ±15° sweep: the slot floor is at y 4.7 (the j0 back-lug
    face), the cheeks under the wing channels are at |z| ≥ 6.95 for x > 83.1, and only the unused rear of the slot
    (x 81.5–83.2) is filled.
- **Tray:** a new part per finger (`actuation_tray_<finger>`).
  - It holds tier 2 in its own fork ribs, and its shelves carry the tier-1 gearboxes (0.2 mm gaps).
  - It hooks into a groove in the rear wall of the magazine and is held at the front by **1× M2×12 SHCS + M2 hex
    nut**. The nut is side-loaded into the guide block, below its tendon channels, and the head is recessed 6 mm
    in the tray.
  - The tray's front-bottom edge is chamfered to clear Shell1.
- **Old worm seats are filled.** This covers the troughs and the palm-face crescents, inside the 15×15 envelope for
  x 7.8–71.2. The fill stops **exactly at the TPU insert layer (y 0.5 and 1.1)**. It leaves every TPU peg hole and the
  index bracket-screw keep-outs open.
  - The TPU motor windows inside the layer are **left empty**. They are pillars' to fill if they want large pillars
    there.
  - The pillars stay ≥ 6.4 mm from every motor envelope, according to pillars.
- **Depth:** tier 1 is at world z −8.2 … −20.2, tier 2 at −21.6 … −33.6, and the tray floor bottom at about −35.6.
- **Bone-local x positions of the gearbox faces:** A 55.0, B 14.0, C 53.5, D 9.0. The motors run from x −1.8 (D shaft
  tip) to 85.0 (A magnet), and the tray from −3.6 to 94.4.
  - The magazines sit between world x ≈ −165 (the D shaft tip) and the knuckles. They stay behind the J0 nut
    keep-out and the metacarpal's J1 sweep.

## 3. Motor, spools, torque and speed

**Assumed motor:** 6 V N20, **1:298** (about 100 rpm no-load at 6 V), with a magnetic Hall encoder (7 PPR per channel).
- Stall torque ≈ 0.4 N·m; safe continuous ≈ 0.18 N·m (gearbox-limited).
- These are catalogue values. Measure the real ratio and stall current with BUILD_PLAN 1.6–1.8.
- 1:298 was chosen because the spur gearbox is back-drivable, unlike the V5 worms, and the highest common ratio gives
  the most holding friction and torque. Speed is still ample.

| spool | tendon radius | tendon force (continuous / stall) | tendon speed |
|---|---|---|---|
| small (A, B, T1, T2) | 3.75 mm (groove floor Ø7.0, flanges Ø9.8) | 48 N / 107 N | 39 mm/s |
| big (C, D) | 5.65 mm (floor Ø10.8, flanges Ø13.2) | 32 N / 71 N | 59 mm/s |

The groove radius is a trade-off:
- **Torque:** smaller is better.
- **Hub wall:** the hub needs at least 1.75 mm of wall around the Ø3 D-shaft, so the minimum floor radius is 3.25 mm.
- **Bend ratio:** D/d = 15 for 0.5 mm line.
- **Excursion:** the J3 flexor needs 33 mm, which is 0.9 turn on the big spool; the 2.4 mm groove holds about 4 turns.

Tier 2 needs the big radius so that its tendons clear tier 1 in the side lanes. It therefore drives the two
lowest-load joints, J3 and J0.

Resulting joint capability (continuous, no-load speed):

| joint | moment arm | torque | speed |
|---|---|---|---|
| J1 | 6.65 mm (tendon in a groove over the J1 head, constant) | 0.32 N·m | 336 °/s |
| J2 | 4.7 → 8.8 mm (open-roof knuckle, grows with flexion) | 0.23 → 0.42 N·m | 475 → 255 °/s |
| J3 | 4.7 → 8.8 mm | 0.15 → 0.28 N·m | 720 → 385 °/s |
| J0 | about 11 mm (lateral wing channels, antagonistic) | 0.35 N·m | 310 °/s |

At the continuous rating that is roughly 4–5 N of fingertip force with the finger straight, and more when the finger
is curled.

**Encoder:** 8344 counts per spool turn (7 × 4 × 298). That is 0.024° of J1 per count, or about 0.05° of J3.

**D-bore fit:** Ø3.0 nominal with the flat 1.0 mm from the axis, zero allowance on purpose. FDM holes print about
0.1 mm small, so the spool presses on.
- Torque is carried by the flat (form lock), not by friction.
- The flat contact stress at 0.18 N·m is about 14 MPa, and about 32 MPa at stall, against about 60 MPa for PLA.
- If the spool is too tight, ream it with a 3.0 mm drill. Add a drop of CA only if it walks off the shaft.
- A set screw does not fit: the smallest screw in the kit is M2×5, whose head would stand beyond the flange.

**Tendon anchor on the spool:** an axial Ø1.0 hole through the outer flange of each groove. The tendon is knotted
behind it.

## 4. Flexor-only drive for J1–J3, antagonistic loop for J0

**Bidirectional loops on J1–J3 are not usable with this knuckle geometry.**
- The back side of every knuckle is square: it is the hyperextension stop.
- An extensor through the V5 back channel (z −4.6) therefore lengthens by 12.3 mm over 90°, while the flexor shortens
  by 10.4–11.2 mm.
- A loop gains about 1–2 mm per joint, which is about 5 mm across J1–J3 for the J3 loop. UHMWPE stretches about 3 %,
  so the loop would over-tension and pull the anchors out.

**So each joint gets one flexor, and extension is passive.**
- Use one elastic per joint through the V5 back channels (z −4.6, added to every bone). A suggested cord is 1 mm
  silicone or latex, knotted, stiffer toward the palm: J1 > J2 > J3, so every flexor stays taut.
- Joint angles are set by the flexor lengths through the coupling matrix: flexor k shortens by the sum of its moment
  arms over the joints it crosses.

**J0 uses one spool with two grooves.** Its two ends run in mirror-image paths to the lateral wings. The loop length
changes only to second order, less than 0.3 mm at ±15°.

## 5. Tendon routing (0.5 mm braided UHMWPE, channels Ø1.2, bends R ≥ 3 except where stated)

**Palm bone and magazine (bone-local):**
- **A and B** rise from their spools through bend bosses (R 3) into two lanes in the bone's back half, at y 5.8 and
  z ∓1.6. The lanes run below the pillars and TPU pegs with at least 1 mm of wall.
- At x ≈ 81–88 they descend through the filled rear of the J0 slot and the boss over A's magnet. They pass beside the
  J0 nut, 4.3 mm off the centre plane, drop through the guide-block legs, and turn in under the nut.
- **C** rises beside tier 1 in the −z side lane, at y 14.0, and enters the guide block at the front, below the A/B
  drops.
- **The J0 axis is crossed underneath.** The guide exit is 4.3 mm in front of the J0 axis and the base-bone entry is
  12.8 mm in front, so the flexor length changes by less than 0.2 mm at ±15° of abduction. This is decoupled without
  touching the M2 J0 screw, which occupies the axis.
- **D (J0 loop):** the two ends rise at z ±5.65 and jog out into tunnels in the magazine wall (y 5.9, z ±6.3; y 6.8 under the TPU wrist-plate
  pocket for x < 9.5). They run forward to x 71.2, then ramp (via x 74.5, y 3.9) into the existing V5 lateral wing
  channels (Ø1.5), 20 % of the way along. They exit at the wing notches, ±10.5 mm lateral and 5.4 mm in front of the
  J0 axis.
  - The V5 wing channel is kept where V5 put it. Over x 82.5–83.5 it is already open into the J0 tongue slot in the
    j0 input (section 10).

**Base bone:**
- The three flexors enter from below at base-local x 13, the V5 connector position, 5.5 mm behind the J1 bore.
- They rise in a 1.4 × 4.4 slot and wrap the J1 head in a groove (floor R 6.4, 2.4 wide) down to the metacarpal's
  clevis floor.
- The J0 loop ends go to two knot anchors: a lateral Ø1.2 through-hole at x 16.8 and z −2, with Ø2.6 knot pockets
  from below at y ±4.4.

**Finger bones.** The same features go on the thumb's metacarpal, proximal and distal (mod_thumb's new keys).
- **The V5 Ø2 channels** are added from `data/sw_bone_channels.csv` (cut at Ø2.1 so they do not coincide with
  existing ones). This covers the metacarpal and pinky metacarpal, and all proximals and the distal at z +4.7/−4.6.
  The distal also gets its seven tip channels.
- **The V5 distal M1.6 anchor** is taken from `sw_instances.csv` (Mirrorm1_*): a Ø3.84 counterbore from the tip, a
  Ø2.1 hole x 5.3–13.3, an M1.6 nut pocket at x 6.5–8.3, and the side slot for the nut.
- **Found and fixed: the V5 palm channels reverse the flexor.**
  - The V5 channels run through the knuckle tongues past the joint axis. At J1 and J2 the flexor's line of action then
    crosses the axis after about 8° of flexion and the "flexor" extends the joint.
  - Fix: at J2 and J3 the channel roof is opened (a 1.6 mm slot down to z 4.0) from the tongue end to 7.7 mm past the
    axis. The effective exits are then the two clevis floors, ±7.7 mm from the axis.
  - The moment arm now grows from 4.7 to 8.8 mm over 90°, and the reversal angle is 117°.
  - At J1 the tendon wraps the head (above).
- **Knot pockets** (Ø3 from the palm face, x 19) anchor the J1 flexor in the metacarpal and the J2 flexor in the
  proximal. The J3 flexor ends at the distal's M1.6 clamp. Channel entries at the clevis floors are flared (Ø4 → Ø2.1).

**Thumb.**
- T1 (R4 flexor, anchored in the thumb metacarpal's knot pocket) and T2 (R5 + R6 flexor, anchored at the thumb
  distal's M1.6 clamp) stand in their V5 seat towers on cover3. Each tower has a 13.0 × 10.5 gearbox pocket, turned
  at its own angle.
- **Re-seated (orchestrator, 2026-09-26).** The first version stood T1/T2 square to the world axes, about 0.4 mm off
  their pocket centres. T1 floated 14 mm above its tower with no floor, and T2's floor and M1.6 holes fell inside the
  V5 pocket's open top. Now:
  - **T1** is centred on its pocket at (−186.02, 21.09) and turned −7.56°, gearbox face at z −24.2. Its tower is
    extended by a collar (pocket + 1.6 wall, R2 corners) from the tower top at z −38.6 to the floor top at z −21.4.
  - **T2** is centred at (−147.93, 26.52) and turned −65.56°, face at −25.8. The tab and plug point toward T3
    (0.3 mm to T3's body), so they stay inside cover3's outline. Turned the other way, the plug would break through
    the outer wall. The pocket's open top is filled from the face up to the tower top at z −23.1: that is its floor.
- The shaft stays below z −15.0 and within r 7.1, per the thumb agent.
- Both have their shaft up, the spool 2 mm out on the shaft above a 2.8 mm floor, and the gearbox face screwed with
  the N20's own 2× M1.6×3 SHCS. Each head sits in a Ø3.6 × 1.8 counterbore with 0.8 mm of floor under it. The spool
  well and tendon window keep their world orientation, so the tendon hand-off is unchanged.
- Their tendons leave the spools horizontally, at z ≈ −17 / −18.8.
- **Hand-off:** these tendons go to the thumb agent's Ø2 path along R2 (the steel tube), the roll-shaft cross holes and
  the ±15° barrel slots. The segment from the spools up to the R2 tube is **not modelled**: see open questions.

## 6. Motor / joint map for the firmware

`wind_sense` is the spool rotation, right-handed about the motor's +X (toward the encoder), that actuates the joint:
it winds the flexor in, or for D it turns J0 toward +Y world (the thumb side). The motor lead polarity that produces it
has to be recorded per channel on the bench, because it depends on the M1/M2 wiring. Channel numbers are a proposal:
the electronics agent owns the driver assignment. The same map is machine-readable in `regions/actuation_motors.json`
(`seat`, `joint`, `channel_proposal`, `wind_sense`, `spool_tendon_radius_mm`, `tendon_anchor`, and the world pose,
connector and plug keep-out of every motor).

| ch | seat | finger / joint | spool r | wind_sense | tendon anchor |
|---|---|---|---|---|---|
| 0 | ID | index J0 (loop) | 5.65 | −1 | base-bone side knots |
| 1 | IA | index J1 flexor | 3.75 | +1 | metacarpal knot pocket |
| 2 | IB | index J2 flexor | 3.75 | −1 | proximal knot pocket |
| 3 | IC | index J3 flexor | 5.65 | +1 | distal M1.6 clamp |
| 4–7 | MD, MA, MB, MC | middle J0, J1, J2, J3 | same | same | same |
| 8–11 | RD, RA, RB, RC | ring J0, J1, J2, J3 | same | same | same |
| 12–15 | PD, PA, PB, PC | pinky J0, J1, J2, J3 | same | same | same |
| 16 | T1 | thumb R4 flexor | 3.75 | +1 | thumb metacarpal knot pocket |
| 17 | T2 | thumb R5+R6 flexor | 3.75 | −1 | thumb distal M1.6 clamp |
| 18 | T3 | thumb R1 yaw, direct drive | none | n/a | mod_thumb coupler |

**Coupling.** With θ in rad, the flexor lengths are:
- L_A = L_A0 − 6.65·θ1
- L_B = L_B0 − 6.65·θ1 − r2(θ2)·θ2
- L_C = L_C0 − 6.65·θ1 − r2·θ2 − r3·θ3

where r2 and r3 grow from 4.7 to 8.8 mm with flexion (chord geometry, `tendon_paths` + the knuckle slots). J0 couples
into the flexors by less than 0.2 mm.

## 7. Tendon lengths and excursions (index; the others are within a few mm)

| tendon | path (spool → anchor) | approx. length | excursion for 0 → 90° | spool turn |
|---|---|---|---|---|
| A (J1) | palm 55 + under-J0 10 + base 24 + metacarpal 4 | 93 mm | 10.4 mm (J1) | 159° |
| B (J2) | palm 96 + 10 + 24 + metacarpal 17 + J2 span 15 + proximal 4 | 166 mm | 21.6 mm (J1 + J2) | 330° |
| C (J3) | palm 64 + 10 + 24 + 17 + 15 + proximal 26 + J3 span 15 + distal 16 | 187 mm | 32.8 mm (J1 + J2 + J3) | 333° |
| D ± (J0) | riser 21 + tunnel 66 + ramp 8 + wing 17 + to knot 4 | ≈ 116 mm each | ±2.9 mm (±15°) | ±29° |
| T1 / T2 | spool → R2 tube → gimbal → thumb | not modelled past the spool | R4: 10.4 mm; R5+R6: 22.4 mm | — |

## 8. Print and assembly notes

**Palm bones: print back side down, as pillars asked.** The flat bottom of the magazine (bone-local y 20.1) is on the
bed, so the magazine needs no other orientation.
- With the magazine the bone is 12.6 mm taller, so **the TPU pause moves to 19.6 mm** (it was 7.0 for the bare bone).
- Its TPU plane no longer matches the wrist segments' 8.5 mm. Print the palm bones alone, or on one plate with the
  wrist segments using two pauses (8.5 and 19.6 mm).
- The bays are open toward the bed and their ceilings bridge 12.4 mm. The fork ribs and the guide block stand on the
  bed.
- Use tree supports only under the blocks that hang from the bay ceilings: the two bend bosses and the descent boss
  over A's magnet. They are inside the open bays, so the supports come out easily. The rear corner ledges (1 mm) need
  none.

**Trays:** print floor down. No supports are needed.

**Spools:** print with the axis vertical, at 0.1 mm layers.

**Assembly:**
1. Press the spools on (the D-flat toward the tab), then tie the tendons through the flange holes.
2. Pre-thread each tendon from the spool along its channel before seating the motor. Channel order along the bone: A/B
   up into the bone lanes and out through the guide; C up the −z lane into the guide; D up through the rear risers into
   the tunnels, then the wing channels.
3. Seat A and B in the upper bays, with the gearbox between the forks and the tab through the +z window.
4. Drop C and D into the tray.
5. Hook the tray's tongue into the rear groove, swing it up, and fit the front M2×12. The nut is slid in from −z before
   the tray goes on.
6. Plug the cables before closing. The plugs are reachable through the side windows.

**T1 / T2:** insert from below (the pockets are turned to the seat angles; T2's tab faces T3), then screw the gearbox face to the floor with 2× M1.6×3 SHCS through the spool well. Press
the spool on last.

## 9. Verification (`scripts/verify_actuation.py` on `work/actuation/test.FCStd`)

Build: master `DexiGrab_V6.FCStd` with `V6_MODULES=j0,pillars,thumb,actuation` (about 4 min, one freecadcmd). The
check runs in parts (`V6_SECT=235`, `46`, `6`) to keep each run under 5 min.

| check | result |
|---|---|
| 1. validity | all 23 changed or new parts are valid single solids: 4 palm bones, base bone, cover3, 3 thumb bones, 6 finger bones, N20, 3 spools, 4 trays |
| 2. motors, spools and trays vs every part | 0 overlaps, minimum gap 0.200 mm (to the magazines, trays and cover3) |
| 3. plug keep-outs (8 mm, 18 motors) | 0 blocked |
| 4. ROM, J0 ±15° | base bone ≥ 0.200 mm from the magazine, cheeks and slot fill (0.2 mm is j0's own tongue/back-lug gap) |
| 4. ROM, J1 0–90° | metacarpal, proximal and distal: nothing within 1 mm of the magazines, trays, motors or spools |
| 5. T1/T2 vs thumb yaw −70…+75° | minimum 0.40 mm (first_thumb_hinge at +15° vs Spool_T1) |
| 6. channel walls (need ≥ 1.0 mm) | A lanes 1.09, B lanes 1.02, C guide 1.50, D tunnels 1.00 mm, the same on all four fingers |
| 6. V5 wing channels (information only) | 0.00: open into the J0 slot at x 82.5–83.5, the same in the j0 input |

The wall check measures from each channel's axis to the nearest face that is not part of that tendon's own channel
(risers included). The wing channels are measured by ray marching, because the V5 hole and the re-cut leave slivers
under 0.15 mm.

## 10. Open questions and conflicts

1. **Depth versus battery and harness.** The magazines need the space down to world z ≈ −35.6 (the tray floors) under
   all four palm bones. Electronics' draft has the harness layer at −32 … −22 and the battery-case top at −31. They
   were drafted against the old inclined-seat poses. Both have to move below −36, or the battery goes elsewhere.
   - 2 stacked 5×AA packs are 29 mm thick. From −36 down to the bowl's bottom (−64) leaves 28 mm at the centre, so the
     case must bulge through Shell1 (V5 did that too) or move out of the palm.
2. **Rear end.** The D shaft tips reach bone-local x −1.8, which is world x ≈ −163 (index) … −168 (pinky), at z −28.
   That is inside the draft Mega box (x −187 … −158); the board itself, on cover_back, is clear.
3. **The T1/T2 tendon path** from the spools (z ≈ −17) up to the thumb agent's R2 tube (z −0.5, swinging with yaw) is
   not modelled.
   - A fixed guide near R1 (for example an eyelet on the turret rim) would give the least yaw coupling.
   - Yaw couples into both thumb flexors, which the firmware can compensate using T3's encoder.
4. The **extension elastics** are not modelled: section 4 gives the recommendation.
5. **Motor data** (ratio, stall torque, encoder PPR) are assumptions. Measure them (BUILD_PLAN 1.6–1.8) before tuning
   the gains.
6. **The 0.5 mm line is an assumption** (braided UHMWPE, about 40 kg). Thicker line needs Ø1.4 channels, a 0.1 mm
   change of `TR`.
7. **V5 lateral wing channels (J0 loop).**
   - The Ø1.5 channel from V5 runs 0–0.65 mm from the side wall of the J0 tongue slot. It opens into the slot over
     x 82.5–83.5, 0.4 mm behind the tongue's rear end. This is the same in the j0 input, and it was kept so the V5 exit
     notches still match.
   - The taut tendon stays about 0.4 mm inside the bore line, so the tongue should not touch it. Check it on the bench.
   - If it frays, move the wing channel 0.7 mm outward (fill the V5 hole first) or glue in a Ø1.5/1.0 PTFE sleeve.
8. **Cables and the Shell2/cover3 seam.**
   - T1/T2 plug downward (−Z) into the cavity under cover3.
   - Their cables must run inside the cavity to the harness, never through the moving seam (shells' rule).
   - The finger motors' cables drop from the +z windows (−z for index C) straight down beside each magazine.
