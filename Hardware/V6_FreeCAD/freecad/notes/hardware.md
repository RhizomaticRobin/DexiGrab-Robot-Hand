# Hardware (topic `hardware`): screws, nuts, washers, inserts

Files: `lib/mod_hardware.py` (new_parts, instances), `scripts/verify_hardware.py`, `regions/hardware.json`
(claimed space), `regions/hardware_report.json` (validation report; rewritten by every build and by verify).

## What the module does

- **Library** (`new_parts()`, keys `hardware_*`, all single valid solids, identity placement, no helical faces).
  Seat frame: origin = seat point (underside of the head / bearing face of nut or washer / top of insert),
  +Z from head to tip; screws occupy z -k..L, nuts z 0..h with flats normal to local X.
  - M2 SHCS ISO 4762 (dk 3.8, k 2.0, 1.5 hex): head taken from the `sw_source` STEP (real socket, 1.125 deep)
    on a plain Ø2.0 shank with the STEP's tip chamfer. Base set 5/8/10/12/14/16; any STEP length
    (3–30) is built on demand.
  - M2 hex nut ISO 4032 (s 4.0, m 1.6): STEP body, helical thread replaced by a plain Ø2.02 bore.
  - M2 nyloc DIN 985 (s 4.0, h 2.8; supplier tables give 2.8–3.0, m ≥ 1.6), parametric.
  - M2 washer ISO 7089 2.2/5.0/**0.3** (the sw_source washer STEP is 0.4 thick: not ISO 7089).
  - M1.6 SHCS ISO 4762 (dk 3.0, k 1.6, 1.5 hex) 3/5 (+ any length on demand); M1.6 nut ISO 4032 (s 3.2, m 1.3).
  - On request only: `M2 insert` (generic heat-set M2 × 3, OD 3.2, hole Ø3.0) and `M2 long insert`
    (ruthex RX-M2x4, OD 3.6 × 4, hole Ø3.2); M1.6 washer.
- **Placement** (`instances()`, group `Hardware`, links `HW_<id>_screw|nut|washer_*`):
  1. J0 pivots (4 × M2×16 + M2 nyloc) from `mod_j0`'s stack, placed with the palm-bone links.
  2. V5 sites (`data/sw_fasteners_onshape.csv`) whose holes exist in the built geometry.
  3. Every entry of every `regions/fasteners_*.json`, read at build time (unreadable/partial files are
     retried, then skipped and reported; bad entries are rejected with the reason).
  Each joint is snapped onto its modelled bore (≤ 0.35 mm, 2°), put on the real seat face, its nut face is
  inferred (first seat where the nut fits, flats aligned to the pocket) and it is validated: clearance hole
  ≥ d+0.4, head space ≥ dk+0.4, nut pocket ≥ AF+0.4, head and nut seated on material, thread engagement,
  "joins ≥ 2 parts", collisions with every placed part. Duplicates in one hole: J0 > request > V5.
  Failed joints are coloured red and listed in the report.
- **Request extras** (beyond the README format): `name`; `nut` `"M2 hex"|"M2 nyloc"|"M2 insert"|"M2 long insert"|null`;
  `nut_offset` (seat → face the nut/insert bears on; inferred when absent); `washer` `"head"|"nut"|"both"`;
  `frame: "local"` with `part` (one joint per instance of that part key) or `instance` (link label).

## J0 pivot (requirement 9)

Stack along the axis, z from the head seat (identical on all four fingers, measured on the build):

| z (mm) | item |
|---|---|
| −2.0 … 0 | M2×16 head in the Ø4.2 × 2.2 counterbore (head top 0.2 below the palm-side face) |
| 0 … 2.8 | palm-side lug (Ø2.4) |
| 2.8 … 3.0 | 0.2 gap |
| 3.0 … 9.8 | Base Bone 1 tongue (Ø2.4), the finger turns on the screw |
| 9.8 … 10.0 | 0.2 gap |
| 10.0 … 11.8 | back lug (Ø2.4) |
| 11.8 … 14.6 | M2 nyloc on the 4.4 AF × 1.0 pocket floor (1.0 in the pocket, 1.8 standing out) |
| 16.0 | screw tip |

- Length: tip **1.4 mm = 3.5 threads past the nyloc** (1.2 mm = 3.0 threads with a 3.0-tall nut); the nylon
  ring needs ≥ 2. M2×14 would end 0.6 mm *inside* the nut (fails); M2×18 would stick out 3.4 mm. **M2×16.**
- Measured clearances: shank to palm-lug holes 0.200, shank to tongue 0.200, head to counterbore wall 0.200,
  nut flats to pocket 0.200, Base Bone to palm bone 0.200 mm.
- Sweep −15…+15° in 5° steps about `mod_j0.MC_PALM` (coincides with mates.csv Revolutes 19–22: 0.000 mm,
  0.000°): 0 mm³ overlap between the hardware and the finger chain, shank-to-tongue gap 0.200 mm at every angle.

## V5 fastener sites

**Placed (3):** `V5_BRK_I1..I3`, M2×5 + M2 hex, index palm bone, world x −91.8 / −114.1 / −133.2. The bone
still has the V5 Ø2.5 bores, Ø4.3 counterbores and closed M2 nut pockets. Snap 0.03–0.07 mm onto the bores,
seat 0.25 mm lower than V5 (counterbore floor). Hole, seat, pocket and tip checks pass; tip 0.70 mm
(1.7 threads) past the nut. Two warnings:
- **They join nothing after `mod_fixes`.** The cover3 bracket straps sat inside the bone volume here (the
  V5 simulacra interference); `fixes` trims them back to the bone surface, which removes the 0.75 mm strap
  flange under each head, so the screws clamp only the palm bone. Without `fixes` the strap is under the
  head but overlaps the bone. Suggested fix: pocket the index bone for the strap (strap + 0.2 mm) instead of
  trimming the strap. Otherwise drop these three screws.
- Head stands **0.25 mm proud** of the palm face (the V5 counterbore is 1.75 deep, the head is 2.0). Deepening
  it would leave only 0.5 mm of strap under the head, so leave it or use a low-head M2.

**Not placed (35, "open items" in the report with coordinates):**

| group | V5 count | why not | owner to request |
|---|---|---|---|
| pinky brackets BRK_P1–P4 (M2×5 + nut) | 4 | bone side has Ø2.5 + nut pocket, **cover2 strap has no hole** | shells/cover2 owner: cut Ø2.4 + Ø4.2 counterbore (strap 2.5 thick → counterbore ≤ 1.5) |
| thumb-hinge bracket BRK_T1 (M2×5) | 1 | no hole in the index bone at x −151.6 | thumb |
| driver mounts DRV (7 × M2×8, 1 × M2×5) | 8 | cover2/cover_back solid; cover3 keeps 3 Ø2.5/Ø4.3 holes but the V5 nut sat on the board | electronics |
| battery lid BAT1–8 (M2×5 + nut) | 8 | Shell1 solid on the axes; case redesigned | electronics |
| fingertip anchors TIP1–5 (M1.6) | 5 | distal phalanges have no anchor bore / nut slot | actuation |
| thumb gimbal THB1–3 (M1.6) | 3 | old hinges replaced by the thumb redesign | thumb |
| cover_back / Shell1 CBK1–6 (M1.6) | 6 | no through holes in cover_back | shells |

## BOM

Placed now (verified build; the report's `bom` is regenerated on every build and includes later requests):

| qty | item | spec to buy | where |
|---|---|---|---|
| 4 | M2×16 socket-head cap screw | ISO 4762 / DIN 912, A2-70 stainless, fully threaded, 1.5 mm hex | J0 pivots |
| 4 | M2 nylon-insert lock nut | DIN 985 A2, s = 4.0, h ≤ 3.0 | J0 pivots, in the back-lug pocket |
| 3 | M2×5 socket-head cap screw | ISO 4762 A2-70 | index bracket sites (see warnings) |
| 3 | M2 hex nut | ISO 4032 A2, s = 4.0, m = 1.6 | captive in the index palm bone |

Expected once the owners send requests (V5 counts as the guide): about +13 M2×5 and +13 M2 hex nuts
(brackets, battery lid), M2×8 for the driver boards (10 boards in V6, electronics decides the count),
and about 14 M1.6 screws + nuts (anchors, shells; thumb TBD).

Order: packs of **20 × M2×16, 20 × M2 nyloc, 50 × M2×5, 50 × M2×8, 50 × M2 hex, 20 × M2 washers ISO 7089,
20 × M1.6×3, 20 × M1.6×5, 50 × M1.6 hex (ISO 4032)**, a **1.5 mm ball-end hex key** (it fits every screw here),
a 4 mm nut spinner, and heat-set inserts only if a request uses them. Nylocs lose grip after a few re-fits:
replace them after about 5 re-assemblies.

## Print and assembly notes

- J0: drop the nyloc (flat face down, collar out) into the 4.4 AF pocket on the back of the palm bone. Push
  the M2×16 from the palm side through lug, tongue and lug, then turn it with the 1.5 mm key; the pocket
  holds the nut. **Stop as soon as the head touches.** The joint must not be clamped: the two 0.2 mm gaps
  must stay open so the finger swings freely, and the nylon ring holds the screw without preload.
  The nut and 1.4 mm of thread stand 3.4 mm out of the back lug; that space is claimed in `regions/hardware.json`.
- J0 play: the tongue turns on the thread crests in a Ø2.4 hole (0.2 mm per side, as required). Its tilt
  is limited by the 0.2 mm side gaps to roughly ±1.5°, which gives a few mm of wobble at the fingertip.
- Index bracket nuts sit in **closed** pockets inside the index palm bone (world z 0.4–2.5). Pause the print
  at the layer that closes them (about bone-local y −0.9 or −3.0, depending on print orientation) and drop in
  3 M2 nuts. This is a separate pause from the TPU pause (y 0.5–1.1).

## Region claims and conflicts

`regions/hardware.json` claims each J0 fastener envelope (counterbore to screw tip + 0.2) and a 10 mm
straight hex-key approach, both in the palm-bone local frames, and the three BRK envelopes plus key access
in `palm_index` local.
`pillars.json` draft boxes (palm_index x 22.5–28.7 / 41.7–47.9 / 61.9–68.1, y −2.5…4.3 "wall zones")
overlap the BRK_I1–I3 claims. The built pillar geometry still leaves every BRK hole, head, pocket and tip
envelope clear (validation passes on the pillar-modified bone), but the pillars' wall zones and the captive
nut pockets share material. If the BRK screws stay, the pillars should keep 1 mm away from the pockets.

## Verification

`V6_MODULES=j0,hardware` build → `scripts/verify_hardware.py` (work/hardware/final/): **PASS**.
- Library: 12 types built, all valid single solids with identity placement and the catalogued extents.
- Joints: 7 placed, 4 OK (J0), 3 WARN (BRK, above), 0 FAIL; 35 V5 open items.
- Audit (exact `common()`, 14 instances against every other instance): **max overlap 0.00000 mm³**. Contacts are
  the designed ones: heads and nuts on their seats, screw inside its nut bore (0.010 gap), shank to tongue 0.200.
- J0: clearances 0.200 mm everywhere (above). The −15…+15° sweep (7 angles × 4 fingers, whole finger chain):
  0 mm³ overlap, own hardware to finger ≥ 0.1999 mm, shank to tongue 0.1999–0.200 mm, Base Bone × palm bone
  0 mm³.
- Earlier composed builds (j0 + pillars + thumb + electronics + fixes + hardware, `work/hardware/integ*`) also
  PASS, with the same numbers except BRK "joins nothing" (effect of `fixes`).
- Request intake, tested with synthetic files (`work/hardware/requests_test/`, report `work/hardware/test_report_req.json`):
  - a request 0.2 mm off an existing bore snaps onto it and supersedes the V5 twin;
  - a request with no hole is placed red with "head not seated / no nut seat";
  - a local-frame request on `palm_middle` resolves to that instance and is rejected as a duplicate of J0;
  - missing keys and unsupported sizes (M3) are rejected with reasons;
  - a truncated, still-being-written file is retried, then reported "unreadable, skipped this build";
  - nut-face inference reproduces the J0 stack exactly on all four fingers (face 11.8 mm, flats aligned).
- Build cost: about 12–18 s of validation per build; `write_manifest`'s second `instances()` call hits a cache.

Request files at the time of writing: **none** (`regions/fasteners_*.json`). Later requests are placed and
validated automatically on the next build, and the report's `requests` section lists each file's status.

## Assumptions and open questions

- DIN 985 formally starts at M3; the M2 nyloc follows supplier data: s 4.0, h 2.8 (2.7–3.0), m ≥ 1.6.
- V5 M1.6×3 screws were pan heads (k 1.3). M1.6 requests are served with ISO 4762 heads (Ø3.0 × 1.6), so
  counterbore Ø3.4 × 1.8.
- Decision needed from the orchestrator: keep the index bracket screws (and pocket the bone instead of
  trimming the straps) or drop them.
