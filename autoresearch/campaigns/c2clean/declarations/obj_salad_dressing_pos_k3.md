# c2clean / obj_salad_dressing_pos_k3 — worker notes

Intent: `pick up the salad dressing and place it in the basket`.
Runner: `tools/fair_run.py` only. Debug band = seeds 51–65.

## Reading the pack

Twelve 128×128 keyframes + K=3 demo traces. Pixelwise diff of each demo's first
and last keyframe leaves one identical prop footprint missing from the table in
all three demos (pixel bbox x[44,53] y[59,80]) and a matching blob newly present
inside the basket: **the target is the dark-green tall bottle**, and the demos
agree on it.

Demo geometry (ee_path6, euler rpy, wrist straight down throughout):

| demo | close EEF (x,y,z) | release EEF | transit apex z | held finger gap |
|---|---|---|---|---|
| 0 | 0.0787, −0.1025, 0.1238 | 0.018, 0.220, 0.176 | 0.311 | 0.018 |
| 1 | 0.0492, −0.1032, 0.1356 | −0.045, 0.246, 0.199 | 0.311 | 0.037 |
| 2 | 0.0375, −0.1186, 0.1169 | 0.005, 0.245, 0.188 | 0.315 | 0.037 |

Mean close z = 0.1254; mean release z = 0.188.

## What the debug seeds actually contain

v0 (pure-sensing probe, all 15 seeds — `fs_..._v0`, `fs_..._v0b`): cam_high is at
base (0.897, 0, 0.65) looking back along −x and 32° down; floor plane at z≈0.005.
Segmenting the cloud (0.02 < z < 0.22, workspace box) gives 6 props + the basket.

Target discriminator, measured over the top 20 mm of every blob:
`g − max(r,b)` = **+19.5** for the green bottle, ≤ **−0.2** for every other prop
and for the basket. Huge margin, no height or position prior needed.

Target geometry, **identical on all 15 debug seeds**: centre (0.1535, 0.0292),
top z = 0.1475, cap/neck diameter 0.034, body 0.062 at the base tapering at a
shoulder around z = 0.085. Only the **basket** moves seed to seed
(x ∈ [−0.017, 0.016], y ∈ [0.244, 0.274], rim z ≈ 0.141). So this `_pos` cell
perturbs the destination, not the target — the debug band therefore tests the
basket search and nothing about target search, and 15/15 here is weaker evidence
than it looks.

Grasp height transfers as a *relative* number: demo close z 0.1254 vs measured
bottle top 0.1475 → **grasp at top − 0.022**. Release: demo 0.188 vs measured rim
0.141 → **rim + 0.049**.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | dump RGB-D through `api.log` (zlib+base64, 1800-char chunks) and do perception offline | 15/15 seeds decoded; green discriminator and geometry above | probe, kept as the perception source |
| v1 | perceive, then one `move` to the grasp point, close, carry, release | 0/4. Descent commanded (0.1535, 0.0292, 0.1255) stopped at (0.1699, 0.0298, 0.1372): **20 mm residual, +16 mm lateral lag**, identical on both attempts and both seeds → deterministic, not noise. Closed on air (gap 0.0018). Also 735 sim steps > 500 horizon: the retry was unaffordable | rejected |
| v2 | is the low-forward pose out of reach, or is one saturated move the problem? Three descent ladders over empty floor, 1 cm increments | At (0.10, 0.13) and (0.055, −0.30) the eef walks down to z = 0.031 unobstructed. At (0.155, 0.13) — the target's radius — the single-shot stop of v1 is reproduced as a *tolerance* effect (POS_TOL = 12 mm swallows a 7.5 mm x lag), and incremental commands then keep descending to z = 0.111, i.e. **23 mm lower than one big move**. Cost ≈ 6–10 sim steps per 1 cm step | probe; fixes the mechanism |
| v3 | descend in 1 cm increments, cancel the lateral lag from the eef actually reached (bounded integral, clip 50 mm), 2 lateral refinements at the reached height | **2/2** (51, 53), 184 sim steps, closed gap 0.0368 = the cap diameter, matching demo1/demo2's 0.037 | accepted mechanism |
| v4 | v3 + approach waypoint at z = 0.24 over the target, descent starting from the height actually reached, one re-perceive-and-retry if the close comes up empty | **8/8** on 51,53,…,65, 205 sim steps | selection candidate |
| v5 | aim-envelope probe: v4 with the grasp aim displaced **+14 mm in x and +14 mm in y** | **0/4**, and the log shows two bugs, not just a missed aim: (a) the lateral refinement commanded the height it had reached, and at the reach envelope the arm bought the lateral correction by **rising 0.1382 → 0.1641**, so it closed 17 mm *above* the bottle top; (b) attempt 1 reported `held=True` at a finger gap of **0.0800** — the fully open hand, which never closed because the 500-step horizon was already spent | probe; exposed two real defects |
| v6 | same probe displaced **−14 mm / −14 mm** | **4/4** — the aim envelope is asymmetric: ≥20 mm of diagonal margin toward the robot, marginal away from it (the cap is only 34 mm across and the finger pads are thin in x) | probe |
| v7 | v4 + (a) refinement always commands the *goal* height, never the reached one; (b) a bounded press that keeps commanding `z_goal − 0.020` while the height still improves; (c) a hold must show `0.005 < gap < 0.060`, so a fully open hand can no longer pass as a grasp | **8/8** on the probe subset, 219 sim steps. Re-probed at +14/+14 as v8: the press now drives 0.1382 → **0.1184** instead of climbing, and `held` correctly reads False — the remaining 0/4 there is the aim being 20 mm off a 34 mm cap, i.e. geometry, not mechanism | **frozen** |
| v8 | v7 re-probed at aim +14/+14 | 0/4, but for the right reason (see above) | probe |

## Aim envelope

Displacing the grasp aim on the debug seeds bounds the margin, which 15/15 on a
band that never moves the target cannot: **−14/−14 mm passes 4/4, +14/+14 fails**
because the finger pads stop covering the 34 mm cap. Perception puts the cap
centre within ~3 mm (top-face centroid, cross-checked against the full-blob
silhouette), so the working margin is several times the error.

## Honest limits

* The debug band places the salad dressing at exactly the same pose on all 15
  seeds; only the basket moves. Everything the band proves about *finding* the
  target is therefore proved once, not fifteen times. The identification rule is
  nevertheless position-free (colour of the top band, no xy prior), and the grasp
  and release heights are relative to the measured object top and basket rim, so
  a moved target should carry.
* The retry costs ~220 sim steps against a 500-step horizon, so it only helps
  when attempt 0 fails early; a failure late in attempt 0 leaves the retry
  running past termination (visible as sim_steps ≈ 1000 in the v8 probe).

## DECLARATION

* **Frozen version: v7.** `packs/c2clean_obj_salad_dressing_pos_k3/program.py`
  md5 `49a8e084144b081823a170c88f7e9923` == `program_v7.py` (verified on the
  cluster and locally).
* **Selection receipt: 15/15** on the full debug band (seeds 51–65),
  `results/sel_c2clean_obj_salad_dressing_pos_k3_v7` — every episode
  `"benchmark_success": true`, 218–219 sim steps each.
* Per-version receipt chain: v0 probe (`fs_..._v0`, `fs_..._v0b`, 15 seeds
  dumped) → v1 **0/4** (`fs_..._v1`) → v2 ladder probe (`fs_..._v2`) → v3 **2/2**
  (`fs_..._v3`) → v4 **8/8** (`fs_..._v4`) → v5 **0/4** / v6 **4/4** envelope
  probes (`fs_..._v5`, `fs_..._v6`) → v7 **8/8** (`fs_..._v7`), v8 envelope
  re-probe **0/4** (`fs_..._v8`) → selection **15/15** (`sel_..._v7`).
* `PROVENANCE` is present in program.py and covers every calibrated constant;
  each source is either a pack.json field, a debug-seed (51–65) measurement, or
  generic controller/camera mechanics.
* No `api.done` read; `tools/fewshot_run.py` was never invoked; no forbidden
  asset was opened.
