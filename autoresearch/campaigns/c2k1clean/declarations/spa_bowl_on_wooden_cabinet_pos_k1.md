# c2k1clean / spa_bowl_on_wooden_cabinet_pos_k1

Task: "pick up the black bowl on the wooden cabinet and place it on the plate".
Runner: tools/fair_run.py only. Pack: K=1 demo (136 steps, 4 keyframes).

## Pack reading (inputs)

| pack field | value |
|---|---|
| keyframe t=0  | ee (-0.2069, 0.0013, 1.167) rpy(3.1295,0.0418,-0.1109) grip -1 (open) |
| keyframe t=50 | ee (-0.0097, -0.2572, 1.159) grip cmd +1 (close) — the grasp |
| keyframe t=123| ee (0.0527, 0.2182, 0.9728) grip cmd -1 (release) — the place |
| keyframe t=135| ee (0.0459, 0.2252, 1.0327) gripper re-opened (0.0383) |
| held gripper qpos | 0.0026/-0.0027 -> finger gap ~5.3 mm while carrying |

Demo finger axis: R from rpy at t=50 has Y_tool = (-0.127,-0.983,-0.130), i.e.
the jaws close along base **y**.  The start pose's `tool_rotation()` is
X=(0.998,0,-0.057), Y=(0,-1,0), Z=(-0.057,0,-0.998): jaws along y, wrist down.

## Harness mechanics measured from tools/ (not task knowledge)

- `move()` is a P-controller: `action = clip(err/0.05, -1, 1)`, up to
  `max(40, 60*seconds*2)` steps, **breaking as soon as |err| < POS_TOL = 12 mm**.
  So a commanded move shorter than 12 mm is a no-op, and the landing point can
  be up to 12 mm off.  Commanding a target *below* the wanted height keeps the
  loop running so the lateral error is driven to ~0 while the press self-limits.
- `grip()` costs exactly 20 steps; `settle(s)` costs 60*s steps.
- Episode horizon = **1000 controller steps** (measured: probe 6 hit
  `sim_steps: 1000`).  After that every action silently no-ops.
- `gripper()` reports `effort 3.0` iff the close command is active AND the
  finger gap > 5 mm (`HELD_MIN_GAP`).

## Scene, measured from cam_high RGB-D on debug seeds 51-65

Table top z = 0.9012 (mode of the depth cloud).  Heights below are mm above it.

| structure | centre (x,y) | top | notes |
|---|---|---|---|
| wooden cabinet plateau | x[-0.15,0.18] y[-0.36,-0.15] | 226 | fixed across all 15 seeds |
| **black bowl on it** | ~(0.03,-0.275), jitter +-0.05 | 279 | outer r ~52 mm, cavity floor 234 |
| plate (white, on a 25 mm block) | ~(-0.255,-0.125) | rim 50, floor 37 | flat: spread 13 mm |
| small bowl (table) | ~(-0.20,0.21) | 43 | |
| big bowl (table) | ~(0.06,0.20) | 51 | |
| cookie box (table, flat) | ~(0.075,0.02) | 19 | |

**The demo's release xy (0.053, 0.218) lands on the big table bowl, not on the
plate** — the pack was recorded in a different layout, so the demo's place
coordinate is a decoy.  The pick coordinate still matches (demo grasp
(-0.0097,-0.2572) vs the measured bowl at (0.03,-0.275)).  Target identity
therefore comes from perception + the language, not from the demo xy.

Plate vs bowl discriminator that works here: **depression depth**.  Plate =
rim-to-floor 13 mm; bowls = 45 mm.  The plate is also the brightest
(lum ~135 vs the black bowl's dark exterior).

## Version log

### probes p1-p5 (perception only)
- p1/p2/p3: found the table plane, the cabinet plateau and the props.
- p4: 1 cm height+RGB grid over all 15 debug seeds -> the table above.
- p5: streamed the cam_high RGB out through the log; confirmed visually that
  there is exactly ONE bowl on the cabinet and that the plate sits on a
  separate grey block, NOT where the demo released.

### p6 (first motion probe) — VOID, overran the horizon
Parked at (0.20,0.32,1.32): residual 0.411, the arm sagged to z=0.916 —
**that pose is outside the reach envelope**, and the non-converging move burned
its whole step allowance.  Everything after it ran past `sim_steps = 1000` and
silently no-oped.  Lesson: stay inside the demo's own ee envelope
(x[-0.21,0.06], y[-0.26,0.23], z[0.97,1.32]) and budget steps.

### p7a-p7d (grasp geometry sweep, seeds 51/53/55, one geometry per run)

Wrist straight down (`rotation=None`), jaws along base y.  Each run aimed at
the measured bowl centre plus an offset, descended to `rim_top - 10 mm`
(the press self-limits ~4 mm below the rim top), closed, lifted 90 mm.

| run | offset from bowl centre | ep51 | ep53 | ep55 | verdict |
|---|---|---|---|---|---|
| p7a | (0, +50 mm) rim pinch | held 11.3 mm | held 10.3 | held 8.6 | **3/3** |
| p7b | (0, -50 mm) rim pinch | held 7.8 | held 7.2 | held 7.6 | 3/3, thinner |
| p7c | (+45 mm, 0) chord     | held 6.2 | 13.7 -> slipped to 5.1 | nothing | 2/3 |
| p7d | (-45 mm, 0) chord     | nothing  | nothing | held then dropped | 0/3 |

`gripper()` reports the finger gap, and the gap after the close is the grasp's
quality signal: the jaws stop at `wall_thickness + 2*|aim error|`, so a gap in
the 7-11 mm band means the rim really is between the pads.

### v1 — perception + p7a rim pinch + straight carry to the plate
Probe (8 seeds 51..65 odd): **8/8**, `results/fs_...pos_k1_v1`.
Formal selection (15 seeds): **14/15**, `results/sel_...pos_k1_v1`.
Only seed 60 failed: rung0 closed to 4.5 mm (below `HELD_MIN_GAP`), the -y
fallback closed to 5.2 mm, and that thin pinch crept to 4.8 mm and emptied
during the saturated 400 mm carry.  Seed 52 also arrived empty (6.4 -> 3.4 mm)
and only scored because the bowl fell onto the plate anyway.

Hypothesis: a pinch below ~7 mm does not survive the carry, and the carry
itself is the stressor because `move()` saturates its command for any leg
longer than 50 mm.

### v2 — grip-quality gate + rung ladder + hopped carry  *(FROZEN)*
1. Reject a close narrower than `GRIP_GOOD_M = 7.0 mm` and walk a rung ladder
   over the aim band: +50, +56, -50, +44 mm; re-perceive the bowl before each
   rung (a capture costs zero controller steps) because the previous rung may
   have nudged it.  If no rung reaches the band, redo the firmest one seen.
2. Carry in 45 mm legs (`hop`) so the P-controller stays out of saturation.
3. Re-check `effort` over the plate and again at the release height, and
   re-perceive + retry the whole pick if the bowl was lost in transit.

Formal selection (15 seeds 51-65): **15/15**,
`results/sel_c2k1clean_spa_bowl_on_wooden_cabinet_pos_k1_v2`.
Seed 60 now needs 4 rungs (+44 mm closes to 17.1 mm) and seed 61 needs the
redo-best path; both land.  Worst-case step usage 852 of the 1000 horizon
(seed 60); the median seed uses 219.

### v3 — v1 with `RIM_OFFSET_M = 0.056` (control: does a fatter single aim help?)
Formal selection (15 seeds): **12/15**,
`results/sel_c2k1clean_spa_bowl_on_wooden_cabinet_pos_k1_v3` (fails 52, 58, 60).
Worse than v1's 14/15, so no single offset substitutes for v2's ladder: the
aim error that decides the pinch width is per-seed, and it has to be *measured*
(by closing and reading the gap) rather than guessed.

### v4 — v2 with every calibrated constant named and declared  *(FROZEN)*
Behaviourally identical to v2: the inline literals in `find_bowl`/`find_plate`
and the lift/hover/release clearances were lifted into named constants with the
same values, so that PROVENANCE covers every one of them.  Re-run formally to
prove the refactor changed nothing.
Formal selection (15 seeds 51-65): **15/15**,
`results/sel_c2k1clean_spa_bowl_on_wooden_cabinet_pos_k1_v4`.
Step usage: median 228, max 852 of the 1000-step horizon (seed 60).

## DECLARATION

**Frozen version: v4.**
`packs/c2k1clean_spa_bowl_on_wooden_cabinet_pos_k1/program.py`
md5 `21bbb4734b075444dee33a5ed91f6178` ==
`program_v4.py` md5 `21bbb4734b075444dee33a5ed91f6178`.

**Full-15-seed selection receipt:** **15/15**
(`results/sel_c2k1clean_spa_bowl_on_wooden_cabinet_pos_k1_v4`, seeds 51-65,
`benchmark_success: true` on every line).

**Receipt chain (every formally-probed version):**

| version | what changed | probe | formal 15-seed selection | dir |
|---|---|---|---|---|
| p6  | park + perceive + grasp ladder | — | void (overran the 1000-step horizon) | `fs_..._p6` |
| p7a-d | four grasp geometries, 3 seeds each | +y rim pinch 3/3; -y 3/3; +x chord 2/3; -x chord 0/3 | — | `fs_..._p7{a,b,c,d}` |
| v1 | perception + +y rim pinch + saturated carry | 8/8 (`fs_..._v1`) | **14/15** (fail 60) | `sel_..._v1` |
| v2 | grip-quality gate 7.0 mm + 4-rung ladder + 45 mm hopped carry + in-transit re-check | — | **15/15** | `sel_..._v2` |
| v3 | control: v1 with a single fatter offset (56 mm) | — | **12/15** (fails 52, 58, 60) | `sel_..._v3` |
| **v4** | v2 with every constant named and declared (no behaviour change) | — | **15/15** | `sel_..._v4` |

**PROVENANCE:** present as a top-level literal dict in `program.py`; an AST
cross-check confirms it covers every upper-case constant in the module
(`X0/Y0/NX/NY` are covered by the `GRID_WINDOW` entry) and that the program
never reads `.done`.

**What the pack contributed, and what it did not.**  The pack fixed the *kind*
of grasp (jaws along base y, wrist ~2 cm below the rim top) and the fact that
the bowl ends on a plate.  Its release coordinate (0.053, 0.218) is a decoy
here: in these seeds that point is a table bowl, and the plate is at
(-0.26, -0.13) on its own block.  Every metric constant is therefore measured
from cam_high per episode, and the plate is identified by shape — its top shell
fills 0.70-0.84 of its bounding box where every bowl's fills only 0.36-0.46
(15/15 over the debug seeds).
