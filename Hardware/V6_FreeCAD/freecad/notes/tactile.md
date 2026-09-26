# V6 tactile skin: FlexiTac pads on every grasp surface

Owner: tactile agent. Files: `lib/mod_tactile.py`, `scripts/verify_tactile.py`, `regions/tactile.json`,
`regions/attachments_tactile.json`, `regions/fasteners_tactile.json`, fabrication data in `work/tactile/`
(`dxf/`, `taxel_map.csv`, `readout_table.csv`, `taxel_map.png`, `fab_summary.json`, generator `make_fab.py`).
Test build: `j0,pillars,thumb,actuation | shells,electronics | fixes,tactile,hardware` in three stages
(`work/tactile/staged_build.py`, same composition and hook order as `build_v6.py`, including the thumb's
`placements()` re-clock) → `work/tactile/test.FCStd`. The electronics stage alone takes about 12 minutes now.

## 1. FlexiTac in one paragraph (and the licence)

FlexiTac (Huang & Li, Columbia, 2026; https://flexitac.github.io) is a piezoresistive matrix: two 0.2 mm FPCs
(top = row electrodes, bottom = column electrodes) with Velostat/Linqstat between them, each FPC laminated and the
edges sealed with polyimide tape, about 0.6–0.8 mm thick, taxels at 2 mm pitch. A "Reading Board 32x16" (Arduino Nano,
one 16-channel analog mux for the rows, four shift registers for the columns, 0.5 mm FFC connectors, 16-pin rows and
32-pin columns) scans 512 taxels and streams frames `0xAA 0x55` + 512 bytes at 2 Mbaud over USB serial.

**Licence: FlexiTac is © 2026 Columbia University, CC BY-NC 4.0.** Attribution is required and commercial use is
not allowed. The FPC outlines in `work/tactile/dxf/` are adaptations of the FlexiTac sensor design and the readout
reuses the FlexiTac board and firmware unchanged, so the same terms apply to anything built from them.

## 2. Coverage plan

One pad per rigid segment, on its palm face. No pad crosses a joint; every joint is crossed by a flexible tail.

| Segment | Part key (instances) | Taxels | Pad |
|---|---|---|---|
| distal ×4 | `distal` (index, middle, ring, pinky) | 46 | D-shaped, 12.4 × 18.8 mm, follows the R 7.5 tip to 1.2 mm from the edge |
| proximal | `proximal` (index, ring) / `proximal_middle` / `proximal_pinky` | 66 / 90 / 36 | 12.4 × 23.3 / 31.3 / 13.3 mm |
| metacarpal | `metacarpal` (index, middle, ring) / `metacarpal_pinky` | 36 / 18 | 12.4 × 13.6 / 6.6 mm |
| thumb | `thumb_distal`, `thumb_proximal`, `thumb_metacarpal` | 46 + 66 + 36 | as the finger parts; proximal and metacarpal wrap to the other side |
| palm | `palm_index`, `palm_middle`, `palm_ring`, `palm_pinky` | 160, 192, 192, 161 | 12.4 × 70.1 mm on each palm bone |

**Total: 1421 taxels** (thumb 148, index 308, middle 364, ring 340, pinky 261).

Not covered, and why:
- **Base Bone 1** (between J0 and J1): its palm side sits under the palm bone's J0 lug, the metacarpal clevis floor
  sweeps its J1 head with a 0.2 mm running gap, and it carries the flexor groove. There is no flat, static face.
- **The webs between the palm bones.** The palm between the bones is the flexible TPU insert. A pad spanning a web
  would put a laminated stack across the bending TPU and tear or delaminate. So the palm gets one pad per palm bone,
  and the webs stay bare (about 2–3 mm between the pads).
- **Joints and knuckles**, by rule. The loops cross them on the back of the hand.
- **Side faces.** The strap-side face of every bone carries the FPC wrap (fold and curtain), so it is not sensitive.
  I did not add index or thumb side pads: the side faces are 15 mm tall with R6 corners and already carry the wrap on
  one side. A side pad would need its own tail around the joint, and there is no free channel for it.
- **Fingertip wrap.** The pad follows the R 7.5 fingertip in plan (D shape) but does not wrap the tip bevel. The tip is
  doubly curved: a 0.8 mm laminated stack cannot follow it without darts, and darts would cost more taxels than the
  wrap gains.

## 3. Pad, recess and wrap geometry (canonical pad frame u, v, w; w toward the palm, face at w 7.5)

- **Stack 0.8 mm (budget) in a 0.6 mm recess: pads stand 0.2 mm proud** of the PLA face, so a contact loads the pad
  before the rim. If your laminated stack comes out at 0.6 mm, the pads sit flush. The recess keeps 0.2 mm clearance
  to its walls (end walls ≥ 1.0 mm, non-strap side wall 0.8 mm).
- **Wall over internal channels:** recess floor at w 6.9 against the flexor-channel roofs at w 5.75 in all phalanges,
  so at least 1.10–1.15 mm. The metacarpal and distal recesses stop 0.3 mm short of the flat top so their corners keep
  1.06 mm to the flared flexor-channel entries (cones r 2.0 → 1.05). The actuation knot pockets (Ø3 from the palm face)
  are bridged by the pad. The thumb distal wraps to the same side as the finger distals, away from the M1.6 anchor nut
  slot.
- **The FPC wrap (both FPCs):** each phalanx pad's FPCs fold over the strap edge (R 1.0 inner radius), run down the side
  face in a 0.6 mm channel, around the dorsal R6 corner and onto the back, where a dorsal **spine** channel
  (0.6 deep, FPC width + 0.2 wide) carries the ray to the joints.
- **Palm bones:** above w 2.7 the rounded section corners are squared off to give a flat pad bed, and the open V5
  trough-3 notch (u 51.5–62.5) and the trough-1/2 slivers are filled from the TPU layer up. The palm pad's FPCs fold over
  the strap edge into a 0.6 mm **curtain channel** on the side face (w 0.1–5.9, i.e. a 0.6 mm, three-layer PLA lip
  above the TPU web slot). The curtain runs to u 62.5 and the channel continues to u 73.3 to take the digit ribbon.
- **Strap screws:** the palm pads have Ø4.6 holes at the V5 bracket screws BRK_I1–I3 (index) and BRK_P1–P3 (pinky);
  heads and key stay reachable. The cover3 and cover2 strap tops are lowered 0.57 mm over the pad recess.
  **P4 has no V6 screw** (it is listed as unplaced by hardware); the pinky ray tail crosses that strap top in a
  lowered channel.

## 4. Routing: one continuous FPC pair per finger ("ray")

Each finger is one custom FlexiTac sensor: the distal, proximal and metacarpal pads share their six row lines (R1–R6)
and have their own columns; the palm pad has its own rows (R7–R12) and shares the column lines with the digits. So a
ray is at most 12 rows × 32 columns and fits one standard reading board, with no junction PCB anywhere.

| Crossing | Solution | Numbers |
|---|---|---|
| J3, J2 (and thumb R6, R5) | Dorsal Ω slack loop between the spine anchors: fillet R3 – loop – fillet R3 | length 27.6 mm stores 18.7 mm over its 8.9 mm chord (90° wrap growth 15.7 + 3 margin); widths 2.8 (J3, 9 lines), 4.0–5.8 (J2, 15–24 lines) |
| J1 and J0 | **Knuckle ribbon** (0.15 mm trace pitch, 3.5–5.3 mm wide): exit bend R3 off the metacarpal spine → free service loop under the knuckle → lane beside the plug-free side of the motor magazine under the TPU web (|v| 9.2, w −11) → in-plane Z step (R4, pre-cut) up through the web-end window beside the wing → S-jog (R8) into the palm bone's curtain channel, where it becomes the curtain at u 62.5 | length 101.9 (pinky 104.4) mm; out-of-plane R ≥ 3.0; twist ≤ 1.56°/mm; free span 51.5 mm (pinky 53.2) vs the taut R3 path at J1 90° + J0 15°: ≥ 3.9 mm to spare. The pinky ribbon shares the ring's lane and window (stacked 1.0 / 0.6 mm outboard) because the pinky's own motor plugs sit on its free side |
| wrist | **Ray tail**, 32 columns + 12 rows, 7.4 mm wide: leaves the palm pad's wrist end flat in a 0.6 mm channel over the bone's rounded end, rises over the TPU wrist plate, crosses the wrist segment at z 7.2–7.6 and runs to the readout pod | R ≥ 4.1; no twist (≤ 0.09°/mm); 81–82 mm to the pod plus the in-pod run |
| thumb R1–R4 | **Thumb tail** (26 columns + 6 rows, 6.2 mm): exit bend off the thumb metacarpal spine → free service loop (hangs outboard to y 67 and down to z −32 at the V6 rest, i.e. after the −62.4° opposition re-clock) → anchor clip on cover3's thumb-side wall (y 39.5, z −12) → corridor outside cover3 and above the thumb-side seam claim (top z −18.4) → pod | R ≥ 3.0; nominal twist ≤ 2.3°/mm; free span 87.9 mm against a worst reach of 48.8 mm over R1 −70…+75, R2 −125…+25, R3 −83…−33, R4 0…90 (plus a U-turn at 3 mm radius); 186 mm to the pod |

All cables stay on the palm side or outside the shells; nothing crosses the moving Shell2/cover3 thumb-side seam.
The knuckle loops hang under the knuckles (world z down to −30.7) inside the space the shells agent left free.

## 5. Readout: taxel budget, boards, lines, USB and power

- **5 × FlexiTac Reading Board 32x16**, one per ray, running `32_16_fast.ino` unchanged, plus a 7-port USB 2.0 hub,
  in a pod strapped on top (+z) of the arm link (the mirror of the electronics pod underneath). The Mega is saturated
  and the tactile readout does not touch it: it has its own USB link to the host.
- **Lines per board:** rows R1–R6 digit pads (shared along the finger), R7–R12 palm pad; R13–R16 spare.
  Columns: digits C1… from the fingertip (distal, then proximal, then metacarpal), palm C1–C32 from the knuckle end.
  Digit column k and palm column k are the same line.

| Board | Ray | Taxels | Rows used | Columns used |
|---|---|---|---|---|
| 1 | thumb | 148 | R1–R6 | C1–C26 |
| 2 | index | 308 | R1–R12 | C1–C32 |
| 3 | middle | 364 | R1–R12 | C1–C32 |
| 4 | ring | 340 | R1–R12 | C1–C32 |
| 5 | pinky | 261 | R1–R12 | C1–C32 |

  The per-pad table (pad → taxels → board / rows / columns → FFC pins) is `work/tactile/readout_table.csv`, and every
  taxel with its board, row, column, frame byte (`2 + 32·(R−1) + (C−1)`), pad-local index and world position/normal is
  in `work/tactile/taxel_map.csv`.
- **FFC / connectors:** each ray tail ends in two gold-finger paddles (0.5 mm pitch, polyimide stiffener to 0.3 mm):
  the bottom FPC in the board's 32-pin ZIF, the top FPC in its 16-pin ZIF. That is 10 connections and **no loose FFC
  cables**. The pin order printed by the real board may be reversed relative to `32_16_fast.ino`'s scan order: do a
  press test per pad and correct the host-side map (the CSV is the map).
- **Scan rate:** the firmware converts every taxel at ADC clock 1 MHz (prescaler 16), about 16 µs per taxel, so about
  8 ms per 512-taxel frame, **roughly 120 frames/s per board, all five in parallel**. With `ROW_COUNT 12` the frame
  shrinks to 384 taxels and about 160 frames/s (the host must then expect 2 + 384 bytes).
- **USB:** five full-speed USB-serial devices behind one hub, one cable to the host. Data rate about 64 kB/s per board
  at 120 Hz, so 2.6 Mbit/s in total: fine even on a single-TT hub, but prefer a multi-TT hub (for example FE1.1s or
  USB2517 based). Clone Nanos (CH340) have no USB serial numbers, so identify the boards by hub port
  (`/dev/serial/by-path`) and keep each board in its port.
- **Power:** from the host USB port only, about 5 × 35–40 mA plus the hub, about **0.25 A at 5 V**. No connection to
  the hand battery or the Mega. A self-powered hub is the robust choice.
- **Bigger-MCU option (not modelled):** one custom board with a Teensy 4.0 (USB high speed), four 74HC4067 muxes for 64
  rows and four 74HC595 for 32 columns. The five rays' column lines are driven in parallel and each ray keeps its own
  12 rows (5 × 12 = 60 rows), so it reads all 1421 taxels as one 64 × 32 matrix over a single USB port, at about
  150–250 Hz. It needs a new PCB and a firmware change (`MUX_COUNT 4`, `ROW_COUNT 64`).

## 6. What `mod_tactile.py` does

- **modify():** 9 phalanx parts get the recess, fold, side curtain, back corner and spine channel. 4 palm bones get the
  pad bed fill, recess, fold, curtain channel and ray-tail channel. cover3 and cover2 strap tops are lowered over the
  pad recess.
- **new_parts():** 13 pad parts (0.8 mm, teal, FPC wraps and spines included), 4 loop parts, 4 knuckle ribbons,
  5 ray tails (swept strips, amber), the pod, pod lid, reading board and USB hub.
- **instances():** group `Tactile`: 19 pads, 10 loops, 4 knuckle ribbons, 5 tails, pod, lid, 5 boards and the hub.
  The thumb items follow `mod_thumb.placements()` (the −62.4° opposition re-clock about R3).
- **Pod:** 204 × 66 × 28 mm (plus lid lugs), saddle on the R 22.5 arm, two strap tunnels, front slot for the five
  tails, rear exit for the hub cable. Two columns of three bays: each board's ZIF edge faces a 5 mm tail aisle along the
  side wall, and each Nano's USB plug has a 16 mm gap. Boards sit on 2 mm floor rails (VHB tape). The lid is held by
  4 × M2×5 SHCS into M2 heat-set inserts in external lugs (`regions/fasteners_tactile.json`).
- `regions/attachments_tactile.json` lets `scripts/rom_sweep.py` move the pads with their bones and the loops with
  their palm-side bones, and excludes the flexible crossings (loop × tip-side bone, knuckle ribbon × finger, thumb
  tail × thumb chain), which `verify_tactile.py` checks instead.

## 7. Verification (`scripts/verify_tactile.py` on the full build)

All sections were run on `work/tactile/test.FCStd` (the full module chain; see the header). Logs and JSON are in
`work/tactile/logs/` and `work/tactile/verify_*.json`.

| Check | Result |
|---|---|
| parts | 15 modified parts (9 phalanges, 4 palm bones, cover2, cover3) and 30 new parts: all valid single solids |
| pads in their recesses | 19 pads: overlap with the bone 0.00000 mm³; seated (distance 0.0000); slab clearance to the recess walls 0.200 mm minimum; 0.200 mm proud |
| overlaps | pads, palm-bed fill, tails, pod, boards and hub: none against every part, the hardware and each other. The only contacts are designed ones: 26 FPC splice butt joints ≤ 0.02 mm³ (one FPC modelled as abutting parts), 4 lid screw/insert engagements, and seats (loop anchors in their channels, ribbons at their exits and in their channels, tails in their channels and on the pinky strap top). The palm-bed fill re-fills nothing: point sampling found 0 of 7,900 fill samples inside material another module had removed |
| **conflict (electronics)** | `W_motor_PA`, the pinky J1 motor lead, runs through the ring/pinky knuckle service loops: 3.41 mm³ with the pinky ribbon and 0.03 mm from the ring ribbon (ring palm frame u 100–108, v −8…−14, w −22…−34). See §11 |
| walls | ≥ 1.064 mm between every tactile cut and the internal voids (flexor channels, flares, knot pockets, anchor); curtain lip above the TPU web slot 0.600 mm (an insert pocket, not a channel) |
| ROM, pads riding on the bones | J1/J2/J3 of all fingers and R4/R5/R6 of the thumb at 0, 30, 60 and 90°: 60 poses, no pad or spine collision |
| joint loops | J2 and J3 loops of all five rays at 0° and 90°: 0.0000 mm³ against both bones; length 27.6 mm stores 18.7 mm over the chord; 90° loop radius 8.6 mm |
| J0 ±15° | static part of every knuckle ribbon against the swung finger: 0.0000 mm³ |
| tails | knuckle ribbons R ≥ 3.0, twist ≤ 1.56°/mm (edge strain ≤ 0.22 %), J1 slack ≥ 3.9 mm at J1 90° + J0 15°; wrist tails R ≥ 4.1, twist ≤ 0.09°/mm; thumb tail R ≥ 3.0, free span 87.9 mm (worst reach 48.8 mm) |
| `scripts/rom_sweep.py` with `regions/attachments_tactile.json` (index J0–J3, thumb R4/R5) | the pads ride along, and there is no tactile-only collision. Two existing bone collisions also catch a pad: thumb R4 ≥ 67.5° (metacarpal × cover3 6.2 mm³, its pad 1.0 mm³) and index J0 −15° (index × middle finger 726 mm³, the middle proximal pad 3.5 mm³) |
| flat patterns | no ray overlaps itself; every pad, loop, ribbon and tail has a DXF |

The test build needed two workarounds for other modules, both listed in §11: `mod_fixes.modify(palm_pinky)` fails on
the current chain (its input shape was kept), and `mod_electronics` returns an empty cover3 (the pre-electronics cover3
was used).

## 8. Fabrication data (`work/tactile/`)

- `dxf/pad_<key>.dxf` (13): bottom FPC (columns) and, below it, top FPC (rows): outline, bend zones with radius,
  electrodes, taxel centres with pad-local r/c labels, strap-screw holes, splice edges to the loops / ribbons / tails.
  The two layers differ only in their bend allowances (mid-surface radii 1.1 and 5.5 for the bottom FPC, 1.3 and 5.7 for
  the top).
- `dxf/loop_<J>_n<N>.dxf` (4), `dxf/knuckle_<finger>.dxf` (4), `dxf/tail_<ray>.dxf` (5, including the in-pod run and
  both gold-finger paddles), and `dxf/ray_<ray>.dxf` (5): each ray's bottom FPC as **one** flat pattern with the
  pieces joined at their splice edges. None of them overlaps itself.
- Everything is drawn **as seen from outside** (contact or dorsal face). For the top FPC the electrodes are on the far
  side, so mirror that layer when you make its Gerbers.
- Flat patterns of the **free** spans (joint loops, the knuckle and thumb service loops) are cut **straight**: a free
  loop takes whatever bend-plus-twist shape its ends impose, and only its length is designed. The static sections
  (lanes, Z steps, curtains, corridors) are developed from the model (geodesic curvature only).
- **FPC build-up:** 2-layer flex, 0.2 mm, ENIG. Layer 1 carries the electrodes (bare, toward the Velostat); layer 2
  carries the routing under coverlay, with vias where a digit line meets its palm line. In the loops and the service
  loops use a single copper layer at the neutral plane (balanced coverlay), RA copper, no vias. Trace pitch 0.2 mm
  (0.1/0.1) in pads, spines and tails, 0.15 mm (0.075/0.075) in the knuckle ribbons and curtains; check the fab's
  minimum.
- **Flat lengths:** thumb 315, index 463, middle 547, ring 447, pinky 487 mm (including the in-pod runs of 36–185 mm).
  If your FPC fab cannot make them, end the tails at the pod front and bridge to the boards with standard 0.5 mm FFCs
  and FFC extenders (FlexiTac's own interconnect); every ray is then 280–362 mm.

## 9. Print and assembly notes

- The recesses and channels are shallow (0.6 mm): print the bones with the palm face up where the other owners allow
  it (the recess floors then print as clean top surfaces), and do not add a top-surface ironing offset.
- Laminate each FPC as in the FlexiTac guide, cut the Velostat to each pad's electrode area, stack bottom FPC,
  Velostat, top FPC and seal the edges with polyimide tape. Keep the wraps, spines, loops and tails un-laminated so the
  two FPCs slide over each other in the bends.
- Fit each ray from the fingertip: stick each pad into its recess with 0.05 mm transfer tape (3M 467MP; the pad then
  stands about 0.25 proud), fold the wrap over the strap edge, press the spine into the dorsal channel, let each joint
  loop form its Ω and check it with the joint at 90°. Thread the knuckle ribbon under the base bone, through its lane
  and the web-end window, and lay it into the curtain channel. Lay the ray tail in its wrist-end channel and clip it
  over the wrist segment.
- Thumb: clip the tail at the anchor on cover3's thumb-side wall, then run the corridor to the wrist. Leave the
  service loop free; it must not touch the Shell2/cover3 seam in any pose.
- Pod: press the four M2 inserts into the lugs, stick the boards on the rails, plug the tails into the ZIFs from the
  aisles, plug the five mini-USB cables into the hub, close the lid (4 × M2×5), strap the pod to the arm.

## 10. BOM and cost (estimates, USD)

| Item | Qty | Unit | Total |
|---|---|---|---|
| Custom FPC pairs, 2-layer 0.2 mm ENIG with stiffener (5 rays × top + bottom = 10 designs, 5 pcs each) | 10 designs | ≈ $15–40 per design | ≈ $150–400 per 5 hands (≈ $30–80 per hand) |
| Velostat / Linqstat sheet (Adafruit 1361, 28 × 28 cm; one sheet covers a hand) | 1 | $4.95 | $5 |
| Adhesive laminating sheets | 1 pack | $6.29 | $6 |
| Polyimide tape | 1 roll | $8–22 | $15 |
| 0.05 mm transfer tape (3M 467MP) | 1 | $10 | $10 |
| FlexiTac Reading Board 32x16 PCB | 5 | $5 | $25 |
| Arduino Nano (genuine $25; clone $5) | 5 | $5–25 | $25–125 |
| 7-port USB 2.0 hub (multi-TT preferred) | 1 | $15–25 | $20 |
| Short mini-USB cables | 5 | $2 | $10 |
| Pod + lid print (PETG, about 90 g) | 1 | | $3 |
| M2 heat-set inserts, M2×5 SHCS (hardware BOM), hook-and-loop straps | 4 + 4 + 2 | | $3 |
| **Total per hand** (prototype quantities, excluding tools) | | | **≈ $150–300** |

At volume the FlexiTac standard piece is about $2.50 per FPC pair at 100 units; the rays are roughly 10–15× its area,
so expect about $15–30 per ray pair, or $75–150 per hand, at 100 sets.

## 11. Assumptions and open questions

- **Electronics routing conflict.** The pinky J1 motor lead `W_motor_PA` crosses the ring and pinky knuckle service
  loops. The router let it leave my old, coarse knuckle claim box because the PA plug sits inside that box.
  `regions/tactile.json` now claims each knuckle ribbon as one "FREE service loop" box plus tight 15 mm segments. The
  electronics router should keep PA's lead out of those boxes, for example by dropping it proximally of world x −73. If
  that is impossible I can shorten or shift the ring/pinky loops, but they need at least 51.5 mm of free length for
  J1 90° + J0 15°.
- **Other modules in my test build:** `mod_fixes.modify(palm_pinky)` currently fails (strap-layer re-cut), and
  `mod_electronics` currently returns an empty cover3 (5 wiring tunnels cut). Tactile causes neither; I kept the input
  shapes so the verification could run.
- **Thumb tail across R1/R2/R3/R4.** Only the length budget is checked for the extreme poses (free span 87.9 mm against
  a worst reach of 48.8 mm). The loop's shape in those poses (snagging, local radius) is not simulated, so check it by
  hand on the first print. The robust upgrade is a flat clock-spring cassette around R1 plus an Ω loop at R2.
- **Thumb R4 ≥ 67.5° hits cover3** at the re-clocked rest (bone 6.2 mm³). The thumb pad adds 1.0 mm³ there. That is
  for the thumb owner.
- **Reading board size and connector layout** are not published. I assumed a 50 × 25 PCB, 15 mm tall with the Nano,
  both ZIFs on one long edge and the mini-USB at one short end. The pod bays leave 5 mm aisles and 16 mm plug gaps, so
  measure a real board before printing the pod.
- **FFC pin order** on the real board may be reversed relative to `32_16_fast.ino`. Do a press test per pad and fix the
  host map (`taxel_map.csv`).
- **P4 strap screw** is unplaced (hardware). The pinky ray tail crosses that strap top in a lowered channel.
- **Stack thickness:** 0.8 mm is a budget. A thinner laminated stack sits closer to flush; a thicker one stands further
  proud. `DEPTH` can grow by at most about 0.06 mm before the walls drop below the 1.0 mm rule (1.064 mm now).
- **Fab limits:** the rays are 315–547 mm long when flat (with the in-pod runs). Check your FPC fab's maximum length,
  or use the FFC-extender variant in §8.
- **Licence:** everything derived from FlexiTac is CC BY-NC 4.0 (non-commercial, with attribution).
