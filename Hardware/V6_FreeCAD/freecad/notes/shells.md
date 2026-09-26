# V6 shells: Shell2 solid, Shell1–Shell2 spherical joint, shell interference fixes

Owner: the shells agent.

| File | Role |
|---|---|
| `lib/mod_shells.py` | The module |
| `scripts/verify_shells.py` | 48 checks |
| `regions/shells.json` | Claimed space |
| `regions/fasteners_shells.json` | 6 screw requests |
| `work/shells/` | Tests and logs; `bake/` holds baked inputs from other parts; `cache/` holds the Shell2 rebuild |

**Frame D** is Shell1's local frame; world = Shell1 link placement × D.
- D +y points toward the fingertips (world ≈ +X).
- D +x points toward the thumb side (world ≈ +Y).
- D +z points out of the back of the hand (world −Z).

θ is the angle about the D y-axis, measured from +z toward +x.

## 1. What the "palm shells" are and how they move (interpretation)

### Measured

**Shell1's skins are exact surfaces of revolution about the D y-axis.** Circle fits at 13 stations give 0.00 mm error. Each profile is a circular arc:

| Skin | Arc radius | Arc centre (y, r) | Radius at y = −60 |
|---|---|---|---|
| Outer | R 176.04 | (−61.40, −109.56) | 66.50 |
| Inner | R 184.39 | (−60.00, −121.92) | 62.50 |

In the hand, the D y-axis runs along the middle of the palm in its back plane, through about (X −120, Y −31, Z −3.4). That is between the middle and ring palm bones, at TPU-web level. It is the line the palm cups about.

**Shell2 shares the axis.** Its underside is a surface of revolution about the same axis, 1.3–1.6 mm inside Shell1's skin in the shared native frame. It spans θ −42.7…+57.3°, while Shell1 spans −46.6…+44.4°. At the V5 pose, its top is flush with the dorsal end of cover3's thumb wall.

**Attachments (V5 fasteners):**
- cover2 is screwed to the pinky palm bone, and cover3 to the index palm bone.
- Shell1 is screwed to cover_back with M1.6 screws. This model keeps 2 bosses with nut slots.
- The battery cover is screwed to Shell1.
- Shell2 has no fastener.
- Shell1 only rests on cover2 (2.2 mm interference) and cover3 (touching). Nothing ties Shell1 or cover_back to the palm.

**How the covers move.** Rolling cover3 about the axis by +φ (cupping on the thumb side) moves it away from Shell1 and cover_back, with no collision to +20°. Rolling it by −φ collides at once, so the CAD pose is the flat-palm limit.

**The palm's own cupping stop.** Bone-to-bone contact about the TPU webs comes at 6.5° (index/middle), 14° (middle/ring) and more than 40° (ring/pinky).

### Interpretation

**The palm shells are Shell1 (the dorsal bowl) and Shell2 (the outer logo cap).** When the palm cups, the thumb side (index bone, cover3, thumb) rolls about the palm axis relative to the pinky side (pinky bone, cover2). V6 gives each shell to one side:
- **Shell1 and cover_back ride on the pinky side.** Shell1 is rebated over cover2's wall end and fixed with 2× M2. cover_back keeps its M1.6 screws to Shell1.
- **Shell2 is driven by the thumb side.** A lip on Shell2 straddles cover3's dorsal wall end with 0.2 mm clearance.

On cupping, Shell2 rolls over Shell1 about the palm axis and keeps the Shell1/cover3 seam covered; that seam opens by up to about 17 mm at +15°. Palm twist adds tilt about the other two axes, which is the "spherical" freedom.

The joint lets both happen. It keeps the shells engaged with hard stops, and holds them lightly together with rubber (O-rings).

### Alternatives rejected

| Alternative | Why rejected |
|---|---|
| Free-floating cosmetic cap (elastic only, no link to cover3) | On cupping, cover3 pulls away from Shell2's edge. That opens a slit of up to about 17 mm into the hand: the "disengage" case |
| Elastic bias of Shell2 toward cover3 | Pre-cups the very soft TPU palm (about 0.004 N·m/deg) at rest |
| Shell2 screwed rigidly to cover3 | Needs holes in cover3 (thumb and fixes own it). The lip gives the same roll coupling without touching cover3 |
| Mirror arrangement (Shell1 thumb side, Shell2 pinky side) | Shell2 does not reach cover2, and its seam with cover3 would open on cupping |
| Nested full lemons (toroidal surfaces only) | Roll only, with no tilt for palm twist |
| Keepers or elastic anchors inside the bowl | Collide with the electronics claims (battery case in the bowl crown, Mega at the wrist). Everything here is outside Shell1 |
| O-ring riding only on the cap (earlier draft) | Exerts no net force on the cap |
| Tension bands to anchors inside Shell1 | Need anchors more than 15 mm deep. At a 17 mm roll the length changes 2–3×, so centring torque is high |

## 2. Shell2 as a valid solid

**Source.** `SRC_Shell2` is one mesh face: 8,626 vertices and 17,248 triangles. It is closed and 2-manifold with consistent winding (V − E + F = 2), but globally inverted, so its volume reads −34,681.8.

**Reversing and sewing works but can't be trusted.** It gives a valid 17,248-face solid, and booleans with primitives work. But `common` with Shell1 silently returned 0 mm³ against a true overlap of about 14,000 mm³, and distance queries took 13–17 s.

So `rebuild_shell2()` builds a clean B-rep in the native frame:
- **Outer skin, including the chamfered long edges.** The mesh's outermost radius along radial rays on a 1.25° × 2.5 mm (θ, y) grid. In the logo area it uses a degree-8 Chebyshev fit of the surrounding skin (709 points, 0.150 mm max, 0.033 mm rms). Where rays miss the cap, a harmonic fill carries the skin on smoothly.
- **Underside.** A Chebyshev fit; it is a surface of revolution to under 0.001 mm.
- **Solid.** y-sections lofted between the two, intersected with a radial prism through the underside rim loop (Douglas–Peucker 0.05).
- **Logo.** The OSHW gear-with-keyhole recess, 1.1–1.9 mm deep, cut from its own floor fit with radial walls through the mesh's logo rim loop.

**Result:** a valid single solid of 34,443.6 mm³ (−0.69%) with 212 faces. Against 80 random mesh vertices, the distance is median 0.021 mm, p90 0.25, p95 0.28, max 0.65, with 84% within 0.2 mm. The worst points are rim and logo-corner vertices, which the tessellation over-samples; the skins themselves are within 0.15 mm.

It is cached in `work/shells/cache/`, keyed by the mesh and the rebuild version. The first build takes about 25 s and about 0.8 GB of RAM; later builds load the BREP.

**Placement: the "correct relationship".** It is the V5 SolidWorks one: Shell1's orientation, shifted 5.1187 mm along D +z (world −Z). The Onshape instance is about 0.5° and 4.5 mm off. At the V5 pose the top is flush with cover3's wall end, and the underside is 2.3–3.7 mm clear of Shell1. The existing `Shell2 <1>` link is kept, and the pose correction is baked into the part's geometry.

## 3. The joint (final parameters are constants at the top of `mod_shells.py`)

**Centre C = D (0, −60, 0):** on the palm axis, at Shell1's equator.

**Bearing:**
- **Shell1:** a raised spherical land, R 70.0 about C, spanning θ −36…+38° and y −68…−52, 3.5 mm high.
- **Shell2:** a concave seat, R 70.2, across its full width over y −70…−50.
- Running clearance is 0.2 mm.

**Design range:**
- Roll −1…+15° about the axis (+ = cupping).
- Tilt ±1° about the D x- and z-axes through C.

**Retention and travel limits:**
- Two Ø5 studs on Shell1 at (y −95, θ 5°) and (y −20, θ 5°) pass through arc slots in Shell2. The slots run θ −10…+6° in Shell2, because the fixed stud travels −φ in Shell2's frame. They are 8.0 mm wide: stud 2.5 + 0.2 clearance + 1.3 tilt, per side.
- The slot ends are the hard roll stops, at −2.22° and +16.22°.
- The slot sides are the tilt stops.
- The Ø13 slider washer can't pass the 8 mm slot, so the cap can't lift off: at least 1.12 mm of overlap everywhere in the full sweep.

**Elastic.** On each stud, stacked on top of Shell2:
1. slider washer, Ø13 × 1.5 (PTFE sheet preferred; PLA works);
2. silicone O-ring 5 × 2.5 (ID 5, OD 10, 40–50 Shore A);
3. printed cap washer, Ø12 × 1.2;
4. M2×8 SHCS into a Ø1.7 × 7 pilot in the stud.

The stud length sets about 10% squeeze on the O-ring at the highest seat point. The squeezed O-rings hold Shell2 down on the land and take up rattle; the slider washers slide over Shell2 as it rolls.

Force estimate, from published O-ring compression curves scaled to soft silicone: about 1–4 N per ring at 5–10% squeeze. Friction is then about 0.01–0.05 N·m with PTFE sliders, or up to about 0.1 N·m with PLA. That compares with roughly 0.3–1 N·m of palm-cupping moment under a grasp.

- Stiffer: add a 0.2 mm shim under the cap washer (about 18%).
- Softer: use 40A silicone or file 0.2 mm off the stud top.
- **Don't use 70A NBR**, which is about 3× stiffer.

Roll is driven by cover3 through the lip, so the O-rings add no spring bias.

**Thumb coupling.** A 1.6 mm U-lip on Shell2 straddles cover3's dorsal wall end over D y −72…−30. It clears both wall faces (NT·p 53.97 / 58.47) and the wall-end top (skin + 7.6–8.6 mm) by 0.25 mm. Elsewhere, Shell2's thumb end stops 0.25 mm short of cover3's wall and block faces. The lip ends at y −72, clear of cover3's R6 fillet corner.

**Pinky side.**
- The Shell1 rim is rebated over cover2's dorsal wall end, 0.2 mm all round. The rebate is an analytic slab: NP·p 51.0…55.77, D z above 42.47, y up to −107.5, each grown by 0.2.
- 2× M2×8 go through the rim (Ø2.4 hole, Ø4.4 counterbore, 2.2 mm flange under the head) into Ø1.7 × 8 pilots in cover2's wall end, at cover2 local x −62 and −22.

**Thumb side (clearance for −1° roll and ±1° tilt).**
- Shell1's thumb edge and cover_back's thumb end are cut back 2.8 mm from cover3's wall face (NT·p 53.97) and block face (46.47).
- The split between the two faces is at D y −77, which is 3 mm past cover3's R6 notch corner (−80.3) to allow for tilt.
- This fixes the cover_back × cover3 overlap (0.44 mm³ → 2.8 mm gap).

**cover_back.** Adds 2× M1.6 holes (Ø2.0, with a Ø3.4 × 1.2 head seat) at Shell1's existing wrist bosses.

**New parts** (instanced in `Hand/Shells` with Shell1's placement): `shells_slider_1/2`, `shells_oring_1/2` and `shells_capwasher_1/2`. The O-rings are drawn 0.05 mm short of both washers, because a torus tangent to a plane upsets OCC booleans and triggered an audit false positive.

## 4. Print and assembly

1. **Print the parts.**
   - Shell1 dome-up, with the studs pointing up (no support on the studs).
   - Shell2 top-up (the lip and logo need no support).
   - Two slider washers (ideally cut from 1.5 mm PTFE) and two cap washers.
   - Pre-thread every Ø1.7 pilot with an M2 screw.
2. **Fit Shell1.** Lower it onto cover2's wall end (the rebate locates it) and against cover_back. Fit 2× M2×8 through the pinky rim into cover2, and 2× M1.6×12 plus hex nuts from cover_back into Shell1's wrist bosses.
3. **Fit Shell2.** With the palm flat, lower it along D −z (straight onto the back of the hand). The lip slides down over cover3's wall end, and the studs enter the slots.
4. **Fit the stacks.** On each stud: slider washer, O-ring, cap washer, M2×8. Tighten only until the cap washer seats on the stud top; the stud length sets the squeeze.
5. **Check by hand.** Cup the palm. Shell2 should roll with cover3 smoothly to about +15° and stop at about +16°. Flatten it: it should stop just past flat. Try twisting: there should be about ±1° of free play.
6. **Service.** Two screws release the stacks, and Shell2 lifts off straight up.

## 5. Coordination

**Electronics** (`regions/electronics.json`, draft):
- My parts put nothing inside the bowl. The land, studs, slots and stacks are all outside Shell1, and the rebate and screws are at the pinky rim. The battery-case and Mega claims are respected.
- Your wrist-end claim mentions "power switch, charge port, USB grommet through cover_back / Shell1". Those openings are in my parts, so send positions and I'll cut them.
- **Keep wires, clips and boards off the thumb-side seam.** That is the Shell1 thumb edge ↔ cover3 wall, and the cover_back thumb end ↔ cover3 block. It opens and closes by up to about 17 mm as the palm cups.

**Tactile** (`regions/tactile.json`, read 00:10): its FFC runs are on the palm side, the finger backs, the knuckles and the wrist exit (palm side). The readout pod is off-hand. Nothing enters the shells or crosses the thumb seam; there is no conflict.

**Thumb and fixes (cover3):** keep cover3's dorsal wall end at D y −72…−30 as it is: a 4.5 mm plate with faces NT·p 53.97 / 58.47 and its top at the current height. Shell2's lip straddles it. Verified against the thumb-module cover3 (51,152 mm³) and the fixes-module cover2.

**Hardware:** `regions/fasteners_shells.json` requests:
- 2× M2×8 SHCS, stud caps, thread-forming;
- 2× M2×8 SHCS, Shell1 to cover2, thread-forming;
- 2× M1.6×12 SHCS + M1.6 hex, cover_back to Shell1.

All seat points are the underside of the head.

**Build input:** the 22:06 master `DexiGrab_V6.FCStd` failed in `partkeys.resolve` with StopIteration (no wrist link points at an `SRC_Palm_bone*` object). Tests used the last good master copy, `work/thumb/base.FCStd`, copied to `work/shells/base.FCStd`.

## 6. Verification (`scripts/verify_shells.py`)

Logs are in `work/shells/`:
- **`verify_final.log`:** `V6_MODULES=j0,shells`, default 12-pose sweep: 48 checks, 0 FAIL, 337 s, 1.06 GB max RSS.
- **`verify_int.log`:** `j0,thumb,shells,fixes`, with the real thumb-module cover3 and the fixes-module cover2: 48 checks, 0 FAIL.
- **`verify_full_a.log` / `verify_full_b.log`:** the full sweep, rolls −1/0/5/10/15° × tilts 0 and all four ±1° corners (25 poses), run as two sweep-only passes: 0 FAIL.
- **Global audit on the integrated build** (`scripts/audit_clearance.py`, `work/shells/audit_int.csv`): no interference involving my parts. Remaining rows are intended contacts (Shell1 × cover_back, cap washer on stud), the 0.05 mm modelled O-ring float, and the 0.200 mm running clearances.

| Check | Result |
|---|---|
| Valid single solids | Shell1 49,880.8 mm³ (was 48,772), Shell2 32,445.1, cover_back 14,781.3 (was 15,337), cover2 21,981.2 (21,898.3 after fixes), 6 stack parts |
| Shell2 source | mesh: 1 face, 0 solids, −34,681.8 mm³. Rebuild: valid, −0.69% volume |
| Shell1 × Shell2 | 0.200 mm gap, 0 overlap |
| Shell1 × cover2 | **0.200 mm gap, 0 overlap** (was 699.6 mm³) |
| cover_back × cover3 | **2.800 mm gap, 0 overlap** (was 0.44 mm³) |
| Shell2 × cover3 (lip) | 0.250 mm |
| Shell1 × cover3 | 2.800 mm |
| Shell2 × cover_back / cover2 | 3.38 / 6.44 mm |
| Sweep, 11 pairs | Shell2 × Shell1, cover2, cover_back, sliders; cover3 × Shell1, cover_back; thumb hub and yoke × Shell1, cover_back. **0 overlap** at every pose in the full 25-pose sweep |
| Shell1 × Shell2 closest approach | ≥ 0.159 mm at the (+1,+1)/(−1,−1) tilt corners at roll −1°/+15°; 0.036 mm at the mixed (+1,−1)/(−1,+1) corners. No contact inside the range |
| Retention | slider washer overlaps the slot sides by ≥ 1.34 mm (≥ 1.12 mm in the full sweep) |
| Bearing | seat stays over the land on ≥ 63° of arc |
| Travel stops | design limits −1° / +15° are free; hard stops (stud at slot end) at −2.22° / +16.22° |

Runtime notes:
- The default run (`SHELLS_QUICK=1`) fits one 10-minute foreground call.
- For the full 25-pose sweep use `SHELLS_QUICK=0`, or split it: `SHELLS_SWEEP_ONLY=1 SHELLS_ROLLS=-1,0,5 SHELLS_TILTS=5`, then `SHELLS_ROLLS=10,15`.

## 7. Open questions for the user

1. **Design range.** Roll −1…+15° with ±1° tilt was chosen from the bone-contact limits and a realistic grasp. Is 15° of cupping enough? More is possible: about +20° is collision-free for cover3, but the slots and the land would get longer.
2. **O-ring squeeze.** It is an estimate. Tune it by shims on the first print (see §3).
3. **Shell2 top fidelity.** The tapered long edges and logo corners are reproduced to about 0.3–0.6 mm at the corner vertices. If exact 3MF fidelity matters for printing the logo, confirm that the recess depth of about 1.5 mm is fine.
4. **Arm mount.** The hand's root is undefined in this CAD. With this split, cover_back and Shell1 belong to the pinky side, so an arm mount on cover_back carries the hand through the pinky group and the TPU palm. If the arm should hold the thumb side, cover_back could be re-assigned to cover3. That needs a screw joint on cover3, which is the thumb module's part.
