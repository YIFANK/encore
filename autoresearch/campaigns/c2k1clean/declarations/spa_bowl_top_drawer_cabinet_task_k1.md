# c2k1clean — spa_bowl_top_drawer_cabinet_task_k1

Intent: "Pick the akita black bowl on the top of the wooden cabinet and place it on the plate"
Runner: tools/fair_run.py only. Splits sealed (debug = 51-65).

## Evidence read from the two packs

| | `..._task_k1` | `..._task_mate` |
|---|---|---|
| language | pick up the black bowl **in the top drawer** of the wooden cabinet -> plate | pick up the black bowl **on the wooden cabinet** -> plate |
| grasp keyframe (close) | t=48 ee=(0.0371,-0.1187,1.0917) | t=50 ee=(-0.0097,-0.2572,1.159) |
| release keyframe (open) | t=139 ee=(0.0757,0.2512,0.9454) | t=123 ee=(0.0527,0.2182,0.9728) |
| gripper gap while holding | 0.0053 m | 0.0053 m |

The mate pack names my OBJECT (bowl on the cabinet top); the k1 pack names my
TARGET (the plate). Both grasps are a near-straight-down wrist (rpy roll ~pi,
yaw ~-0.12, i.e. the home wrist) and both hold with a ~5 mm finger gap, which
is a pinch on the bowl's thin wall rather than a wrap of its body.

## Probe p1 — perception only (15/15 debug seeds), receipt
`results/fs_..._p1` (0/15 success by construction; it never moves).
Dumped cam_high/cam_arm_wrist RGB-D per seed to
`results/dbg_c2k1clean_spa_bowl_top_drawer_cabinet_task_k1/ep<seed>.npz`.

Measured, offline, on all 15 seeds:
- table plane z = 0.900; cabinet top plane z = 1.125-1.130.
- cabinet-top bowl: rim top z = 1.179 on every seed; Kasa circle fit on the
  rim gives r = 0.0537-0.0539 m; centre varies over seeds,
  x in [0.000, 0.046], y in [-0.301, -0.260].
- the drawer bowl (the bddl's nominal object) sits ~0.08 m lower, rim ~1.08-1.10;
  a z > 1.14 cut separates the two cleanly.
- plate: flat 0.136 m disc, rim z = 0.920, centre x in [0.048,0.072],
  y in [0.187,0.214]. The ramekin (x ~ -0.20, top 0.944) and the cookie box
  (saturated colour) are excluded by the band + a neutral-colour mask.
- the parked arm is at y = +0.04, so the y < -0.10 half-space isolates the
  cabinet without any robot pixels.

## Probe p2 — grasp ladder, receipt `results/fs_..._p2` (seeds 51,53,55,57)
Ladder over (offset direction, close height). Every seed held on the FIRST or
SECOND rung, all of them the `+y` rim straddle, i.e. the jaws do separate
along world y and the wall pinch is the right grasp:

| seed | rung that held | achieved eef z | gap after close |
|---|---|---|---|
| 51 | y+, rim_top-0.021 | 1.1650 (rim-0.0143) | 0.0124 (effort 3.0) |
| 53 | y+, rim_top-0.021 | 1.1651 (rim-0.0142) | 0.0122 (effort 3.0) |
| 55 | y+, rim_top-0.021 | 1.1639 (rim-0.0154) | 0.0129 (effort 3.0) |
| 57 | y+, rim_top-0.033 | 1.1584 (rim-0.0210) | 0.0088 (effort 3.0) |

Seed 57's first rung landed at 1.1716 (rim-0.0078) and gripped nothing
(gap 0.0041) -> **the close height is the whole game, and move() exits inside
its own 12 mm tolerance, systematically ~10 mm high on this descent.**
Also learned: the episode horizon is 1000 sim steps (seed 57 exhausted it).

## Version chain (hypothesis -> evidence -> verdict)

### v1 — rim straddle + bias-cancelled descent + plate drop
Hypothesis: grasp = tool centre at (rim_cx, rim_cy + rim_r) so the near finger
goes inside the bowl and the far finger outside, closed at rim_top-0.023 with
the commanded point re-issued to cancel the controller's tolerance shortfall;
place = same rim offset over the plate centre, release at z=0.958.
Receipt: `results/fs_..._v1`, seeds 51,53,55,57,59,61,63,65 -> **6/8**.
Evidence: the grasp itself is right — every achieved close height in
[rim-0.025, rim-0.017] carried the bowl. Both failures were budget, not belief:
seeds 61 and 63 hit the 1000-step horizon (538-791 steps on the successes),
because a blocked correction command runs its full 2x60x`seconds` cap. Seed 61
also slipped the bowl on the lift (gap 0.0128 -> 0.0021) and the 0.002 gap
floor let it carry on with an empty hand.
Verdict: keep the geometry, cut the motion budget, harden the held-check.

### v2 — same geometry, smaller step caps, 2 attempts
Changes: no redundant opening grip, one waypoint fewer, correction commands
capped at 0.5 s and clamped to 40 mm, GRASP_DZ -0.023 -> -0.026, transit caps
3.0/2.0 s -> 2.0/1.5 s, lift-gap floor 0.002 -> 0.0035.
Receipt: `results/fs_..._v2`, same 8 seeds -> **7/8**, and the step cost fell
from 408-1000 to 153-196.
Evidence: the one failure (seed 63) closed correctly (gap 0.0090, effort 3.0)
but read 0.0033 after the lift, one ten-thousandth under the new floor, and
gave up — while its own re-perception said the cabinet top was empty, i.e. the
bowl was in the hand. The finger gap simply does not separate "pinched the
wall" from "let go on the way up"; both live in 2-6 mm.
Verdict: stop arbitrating the grasp on the gap. Ask the camera.

### v3 — the empty cabinet top is the receipt (FROZEN)
Change: after every lift, park the tool at y=+0.12 (which is on the way to the
plate anyway, so it costs one command) and re-run find_bowl. The bowl is held
iff the cabinet top is now empty AND the finger gap is above 0.0025; if the
cabinet top still has the bowl, re-perceive it and retry (up to 3 attempts); if
it is empty but the hand is too, stop and say so rather than mime a placement.
Receipts: probe subset `results/fs_..._v3` -> **8/8**;
formal selection `results/sel_c2k1clean_spa_bowl_top_drawer_cabinet_task_k1_v3`,
all 15 debug seeds -> **15/15**, 163-376 sim steps of the 1000-step horizon.

## Note on the cell's re-authored intent
The bddl names the bowl *in the top drawer*; the intent sentence names the bowl
*on the top of the wooden cabinet*, and the two are different objects in the
same scene (rim tops 1.08-1.10 vs 1.179). v1-v3 only ever touch the cabinet-top
bowl the intent names, and the environment's own benchmark bit fires — so for
this task the predicate accepts the object the intent names.

## DECLARATION

- **Frozen version: v3.** `packs/c2k1clean_spa_bowl_top_drawer_cabinet_task_k1/program.py`
  md5 `20415c5dbd61cd897ec159efdcf613d1` == `program_v3.py`
  md5 `20415c5dbd61cd897ec159efdcf613d1`.
- **Selection receipt (full 15 debug seeds, one formal run):**
  `results/sel_c2k1clean_spa_bowl_top_drawer_cabinet_task_k1_v3` — **15/15**
  `"benchmark_success": true` (seeds 51-65), 163-376 sim steps per episode
  against a 1000-step horizon.
- **Receipt chain:**
  | version | receipt dir | seeds | result |
  |---|---|---|---|
  | probe p1 (perception only) | `results/fs_..._p1` | 51-65 | 15 captures dumped, 0/15 by construction |
  | probe p2 (grasp ladder) | `results/fs_..._p2` | 51,53,55,57 | held on rung 0 (3 seeds) / rung 1 (1 seed) |
  | v1 | `results/fs_..._v1` | 51,53,55,57,59,61,63,65 | 6/8 |
  | v2 | `results/fs_..._v2` | same 8 | 7/8 |
  | v3 | `results/fs_..._v3` | same 8 | 8/8 |
  | **v3 (formal selection)** | `results/sel_..._v3` | **51-65** | **15/15** |
  (all dirs prefixed `c2k1clean_spa_bowl_top_drawer_cabinet_task_k1`)
- **PROVENANCE:** present as a top-level literal dict in program.py, covering
  GREY_SAT, BOWL_Z_LO, BOWL_Z_HI, BOWL_Y_MAX, RIM_BAND, GRASP_DZ, VERIFY_GAP,
  VERIFY_PARK, SAFE_Z, PLATE_BAND, RELEASE_Z, BIAS_PASSES, STEP_BUDGET — every
  one sourced to a probe-p1/p2 or v1/v2 debug-seed measurement, or to generic
  controller mechanics. No constant comes from the packs' xy values, from any
  other cell, or from prior context.
- **Clean room:** writes confined to
  `packs/c2k1clean_spa_bowl_top_drawer_cabinet_task_k1/*` and
  `results/*c2k1clean_spa_bowl_top_drawer_cabinet_task_k1*`; the only pack
  contents read were `pack.json` + `keyframes/` of the two packs named in the
  brief; `tools/fewshot_run.py` was never invoked; `api.done` is never read.

STOP.
