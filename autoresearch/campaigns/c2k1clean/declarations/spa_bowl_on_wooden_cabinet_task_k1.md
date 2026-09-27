# c2k1clean / spa_bowl_on_wooden_cabinet_task_k1 — working notes

Intent: **"Pick the akita black bowl on the stove and place it on the plate"**
(the BDDL file the runner loads is the wooden-cabinet one; the intent sentence
is authoritative and names the *stove* bowl).

Runner: `tools/fair_run.py` only. Splits sealed: debug = 51–65.

## Pack evidence (pack.json + keyframes only, both packs)

| | k1 pack | mate pack |
|---|---|---|
| language | "…black bowl on the **wooden cabinet** → plate" | "…black bowl on the **stove** → plate" |
| K | 1 | 1 |
| grasp keyframe | t=50 ee=(-0.010,-0.257,**1.159**) gcmd→close | t=43 ee=(-0.275,-0.116,**0.949**) gcmd→close |
| hold finger state | (0.0026,-0.0027) → gap ≈ 0.005 m | (0.0027,-0.0047) → gap ≈ 0.005 m |
| release keyframe | t=123 ee=(0.053,0.218,0.973) | t=141 ee=(0.067,0.245,0.942) |

Read: both packs hold the bowl at a ~5 mm finger gap → the bowl is **pinched
across its rim wall**, not straddled. Both release over a plate at
xy ≈ (0.05,0.22)–(0.07,0.25).

## v1 — perception survey (seeds 51,53,55,57), 0/4 by construction (no motion)
`results/fs_c2k1clean_spa_bowl_on_wooden_cabinet_task_k1_v1`

Hypothesis: the scene must be re-derived from RGB-D; nothing about it may be
assumed. Evidence (cam_high, 512², K f=618, T_base_cam translation
(0.659,0,1.610), optical axis (-0.778,0,-0.628) → image **down = +x**,
image **right = +y**):

- table plane z ≈ **0.901** (dominant depth mode, 132 k px)
- **three identical akita bowls, at three different rim heights**:
  - cabinet top: xy ≈ (0.00,-0.29), rim z **1.179** (+0.278)
  - **stove: xy ≈ (-0.26,-0.13), rim z 0.979 (+0.077)**  ← the intent's bowl
  - bare table: xy ≈ (-0.20,0.21), rim z 0.943 (+0.042)
- plate: xy ≈ (0.02,0.20), top z 0.919 (+0.018), span 0.12×0.13
- cookie box: xy ≈ (0.08,0.03), z 0.920

Verdict: **the height band alone names the target bowl** — no appearance cue is
needed, the three bowls are identical. Grasp depth from the packs: EEF sits
0.020–0.030 m below the rim top at the grasp keyframe.

## v2 — first full pick-and-place (seeds 51,53,55,57) → **3/4**
`results/fs_c2k1clean_spa_bowl_on_wooden_cabinet_task_k1_v2`

Hypothesis: pinch the stove bowl's rim at its +y extreme (the jaws open along
base ±y with the wrist straight down), lift, carry at z=1.10, release over the
plate at plate_top + measured hang + 0.015.

Evidence:
- **ep51/53/55 success**; grasp held at finger gap 0.008–0.012, hang measured
  0.025 m, release at z≈0.956.
- rim offset **+0.032 m** held on all three (the +0.040 rung closed on the rim
  but lost it during the lift on ep51).
- **ep57 failed**: cluster selection picked a dark *background* blob at
  x=-0.615 (outside the table) because it beat the bowl on pixel count.
- Also measured: the stove-bowl cluster spans 0.110 m, but an isolated bowl
  spans 0.085–0.088 m → **the stove's own raised ring is inside the height
  band**, so an extents-midpoint centre is biased by up to ~0.015 m.

**Key verdict: the benchmark predicate fires for the STOVE bowl.** The intent
and the environment's own success bit agree; there is no re-authoring conflict
to resolve.

## v3 — ring-fit localisation + cheaper ladder (seeds 51…65 odd)
`results/fs_c2k1clean_spa_bowl_on_wooden_cabinet_task_k1_v3`

Changes: (a) candidate clusters gated to the table workspace and scored by a
fixed-radius (0.044 m) centre grid search that votes for the bowl's own rim
circle and penalises points inside it, so the stove ring cannot drag the
centre; (b) ladder reordered to (0.032, 0.039, 0.026) so the rung that works
runs first; (c) re-perceive before each retry; (d) two-stage lift.

Result: **6/8** on 51,53,…,65. ep57 fixed (workspace gate killed the background
blob). ep53/55 regressed: the bowl was lifted clear of the stove and then fell
out (seed 55, three times), and on seed 53 it fell out mid-transport. The
close itself was never the problem — the finger gap after the close was
0.0117–0.0132 m on **every** rung of a 13 mm offset ladder, i.e. the pinch
lands on the rim wall regardless of aim.

## v4 — glide every loaded motion → **8/8 probe, 15/15 selection**  ← FROZEN
`results/fs_…_v4` (8 probe seeds), `results/sel_…_v4` (all 15)

Hypothesis: retention, not aim, is the failure. heron's LIBERO cartesian servo
sends `clip(err / 0.05)`, so any command longer than 0.05 m runs **saturated**
from its first step: the 0.15 m lift and the 0.35 m transport both start at
full commanded velocity and jerk the bowl out of a pinch that holds ~5 mm of
rim wall. Fix: `glide()` walks every loaded motion in 0.025 m increments (half
the saturation distance).

Evidence: 8/8, then **15/15** on seeds 51–65, every episode succeeding on
**attempt 0** (226–238 sim steps, uniform — the retry ladder never fired).

Second finding, recorded because it inverts the obvious sensor: **the finger
gap is not a held/dropped signal for this pinch.** Every successful v4 carry
reported `gap=0.0041, effort=0.05` — i.e. `effort>=3` says "not holding" —
while demonstrably carrying the bowl to the plate. The lift is verified by
depth (a payload hanging under the wrist), not by the gripper.

## v5 — stricter carry check → **0/15, falsified, rejected**
`results/sel_…_v5`

Hypothesis: v4's carry test (`payload bottom > table+0.045`, depth window
floored at table+0.06) could pass on a *failed* lift, because a bowl left on
the stove reports a floor-clipped bottom of table+0.06. Lowering the window
floor to table+0.02 and demanding bottom > table+0.105 should separate them.

Evidence: **0/15**. The window under the wrist is a vertical column, so with
the floor at table+0.02 it sees the **stove top at 0.926 m** on every episode —
`bottom=0.926` for all three rungs of every seed, so every real carry was
rejected and the bowl was put back three times.

Verdict: falsified. v4's table+0.06 floor is load-bearing — it is what excludes
the ground under the payload. The latent hole it leaves is narrow and one-sided:
a false "carrying" only costs a *retry that would not have helped anyway*; it
can never spoil a grasp that worked. Rejected in favour of v4's receipt.

## DECLARATION

- **Frozen version: v4.** `packs/c2k1clean_spa_bowl_on_wooden_cabinet_task_k1/program.py`
  md5 `3d4031b5cd774d40188d3fa38430a622` == `program_v4.py` (same md5).
- **Selection receipt: 15/15** on the full debug band 51–65,
  `results/sel_c2k1clean_spa_bowl_on_wooden_cabinet_task_k1_v4`
  (51:T 52:T 53:T 54:T 55:T 56:T 57:T 58:T 59:T 60:T 61:T 62:T 63:T 64:T 65:T).
- **Receipt chain**
  | ver | seeds | result | dir |
  |---|---|---|---|
  | v1 | 51,53,55,57 | 0/4 (perception survey, no motion) | `fs_…_v1` |
  | v2 | 51,53,55,57 | 3/4 | `fs_…_v2` |
  | v3 | 51,53,…,65 | 6/8 | `fs_…_v3` |
  | v4 | 51,53,…,65 | 8/8 | `fs_…_v4` |
  | **v4** | **51–65 (all 15)** | **15/15** | **`sel_…_v4`** |
  | v5 | 51–65 (all 15) | 0/15 (falsified, rejected) | `sel_…_v5` |
- **PROVENANCE** present in program.py as a top-level literal dict, covering
  TABLE_Z, STOVE_BAND, BOWL_R_M, GRASP_DEPTH_M, RIM_OFF_LADDER_M, GLIDE_STEP_M,
  HOVER_M, CARRY_Z_M, PLACE_CLEAR_M, PLATE_BAND, WORKSPACE. Every entry sources
  to the two named packs, to debug-seed (51–65) RGB-D/EEF/gripper measurements,
  or to generic controller mechanics. No `api.done` read anywhere.
- Eval band 1–50 never run, never inspected.
