# c2k1clean / obj_orange_juice_pos_k1 — NOTES

Intent: "pick up the orange juice and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2k1clean_obj_orange_juice_pos_k1/`.
Probe subset: 51,53,55,57,59,61,63,65. Selection: all 15 (51..65).

## Pack reading (K=1 demo)

- `action_scale` [0.2095, 0.2806, 0.3088, 0.0133, 0.0195, 0.0274, 1.0]; 126 steps, stride 10.
- Keyframes: t0 ee (-0.1405, 0.0054, 0.2729) open; t42 ee (0.0779,-0.1064,0.1203) close cmd;
  t116 ee (0.0018, 0.2243, 0.1683) open cmd; t125 ee (0.0090, 0.2253, 0.1835).
- `ee_path` shape: descend to z 0.1145 at (0.0597,-0.1042) -> close -> lift to z 0.2957
  -> traverse to y +0.232 -> descend to z 0.175 -> release.
  So: **grasp z 0.1145**, **carry z ~0.29**, **release z ~0.175**.
- Keyframe PNGs (128x128 agentview) show 6 table props + a wicker basket on the right;
  at t116/t125 the orange/white carton is inside the basket => target = the juice carton.

## v0 — API surface probe
Hypothesis: `api.deproject` exists on the api object.
Evidence: `AttributeError: 'FairApi' object has no attribute 'deproject'`.
Verdict: WRONG — `deproject` lives on the **FairFrame** (`f.deproject(u,v)`).
Useful output: cam_high K = fx=fy=618.04, c=(256,256); `t_base_cam` =
camera at base (0.8966, 0, 0.65), optical axis (-0.8488, 0, -0.5288) (looks along -x,
31.9 deg down). Base +y maps to image +u. Reset eef (-0.1485, 0, 0.2613),
tool_rotation ~ diag-ish [[.998,0,-.057],[0,-1,0],[-.057,0,-.998]] (straight down,
tool y axis along base -y => **jaws close along base y**). Gripper open width 0.0778,
effort 0.05.

## v1 — 2D (x,y) grid clustering
Hypothesis: 1.2 cm xy-cell connected components segment the props.
Evidence: only 6 clusters; the juice carton and the dressing bottle were swallowed by the
arm cluster (ztop 0.436) because the arm hovers directly above them in projection.
Verdict: WRONG mechanism — **xy projection fuses anything under the arm**. Confirmed
vectorised deprojection matches `f.deproject` to 1e-3 m. Table plane at base z ~ 0.000.

## v2 — RGB-D dump
Wrote `dump_<t>_cam_high.npz` / `_cam_arm_wrist.npz` into the pack dir for seeds
51,53,...,65 and analysed locally. Scene (base frame), stable across the 8 probe seeds:

| prop | centre (x,y) | ztop | n px | orange frac |
|---|---|---|---|---|
| basket | (0.008, 0.255) | 0.142 | ~11.4k | 0.000 |
| **orange juice carton** | **(-0.140, 0.060)** | **0.141** | 1853 | **0.144** |
| tomato sauce bottle | (0.167, 0.029) | 0.148 | 2772 | 0.022 |
| salad dressing (green cap) | (-0.184,-0.080) | 0.147 | 1414 | 0.000 |
| ketchup bottle | (0.11, -0.20) | 0.113 | ~1450 | 0.000 |
| brown box | (0.062,-0.099) | 0.029 | 1015 | 0.009 |
| pudding box | (-0.116,-0.240) | 0.019 | ~430 | 0.07 |
| arm | — | 0.484 | 2656 | 0.000 |

Only the **basket, ketchup and pudding move** across seeds 51..65 (probe subset);
the carton sat at exactly (-0.140, 0.060) in all 8. Not hardcoded — perceived every run.

Key mechanism found: **3D voxel clustering (1.5 cm, 26-connectivity) instead of xy
grid** unfuses the arm from the props beneath it. Robot links render saturated
green/blue, so a colour mask removes them too.

Identifier (margin 6.5x on the probe subset): among clusters with
150 <= n <= 6000 and ztop in [0.06,0.26], take argmax orange-pixel fraction.
Carton top face: x -0.151..-0.129, y 0.034..0.085 => footprint ~2.6 x 5.1 cm,
height 0.141. Grasp z 0.1145 (pack) = ztop - 0.027.

## v3 — first pick-and-place
Hypothesis: perceive carton + basket from cam_high, hover -> descend to ztop-0.027 ->
close (jaws along base y, spanning the 5.1 cm width) -> lift to 0.29 -> traverse to
basket centre -> descend to rim+0.033 -> open.
Evidence: **8/8** on the probe subset, first attempt. Grip closed to w=0.0536 with
effort 3.00 (holds the 5.1 cm face), residuals <= 0.013 m on every move.
Verdict: CORRECT. Formal run on all 15 debug seeds: **15/15**
(`results/sel_c2k1clean_obj_orange_juice_pos_k1_v3`).

## v4 — same nominal path + failure-path recovery  [FROZEN]
Hypothesis: v3's nominal path is right, but the blind eval seeds (1-50) may perturb
positions further than the debug seeds do, so add recovery that is inert when the
nominal path works:
1. **merge guard** — if the target's top face spans > 0.075 m (wider than the 0.0778 m
   jaw opening) the voxel cluster has fused with a neighbour, so aim at the orange
   pixels' own 5..95 pct footprint instead of the cluster top-face centre;
2. **grasp retry ladder** — verify `effort >= 1.0` after close AND after lift; on
   failure re-open, re-perceive and retry (attempt 2 uses the full-cluster centre,
   a genuinely different estimate that the camera-facing front face pulls toward +x);
3. **clear-view fallback** — if no candidate cluster is found at all (arm occluding the
   carton), step the wrist to (0.05,-0.25,0.30) and re-perceive; wrapped in try/except.
Evidence: probe **8/8**; formal **15/15**
(`results/sel_c2k1clean_obj_orange_juice_pos_k1_v4`). `grep` over all 15 episode logs
shows **no fallback branch fired** — v4 is behaviourally identical to v3 on every debug
seed, so the 15/15 is not bought with any added risk on the nominal path.
Verdict: CORRECT, and strictly dominates v3 off the nominal path. FROZEN.

## Receipt chain

| version | what changed | probe (8) | formal (15) | result dir |
|---|---|---|---|---|
| v0 | api-surface probe | — | — | `fs_..._v0` (AttributeError: deproject is on FairFrame) |
| v1 | 2D xy-grid clustering | — | — | `fs_..._v1` (arm fuses the carton) |
| v2 | RGB-D dump for offline analysis | — | — | `fs_..._v2` |
| v3 | 3D voxel clustering + orange-fraction pick + pick-and-place | 8/8 | **15/15** | `sel_..._v3` |
| v4 | + merge guard, grasp retry ladder, clear-view fallback | 8/8 | **15/15** | `sel_..._v4` |

## DECLARATION

- **Frozen version: v4.** `packs/c2k1clean_obj_orange_juice_pos_k1/program.py`
  md5 `015d07858e3a918f19900be5a14c0456` == `program_v4.py`
  md5 `015d07858e3a918f19900be5a14c0456`. (v3 is `155008e47f0c3832137f767378299b1a`.)
- **Selection receipt: 15/15** `benchmark_success: true` on the full 15 debug seeds
  (51..65), dir `results/sel_c2k1clean_obj_orange_juice_pos_k1_v4`.
- Per-version receipt chain: table above. Every formally-probed version archived as
  `program_vN.py` in the pack dir.
- **PROVENANCE** present in `program.py` as a top-level literal dict covering all 21
  calibrated constants. Sources are this pack's `pack.json`/`keyframes/` and
  own debug-seed (51..65) cam_high RGB-D observations only.
- Splits respected: only `--split debug`; seeds 1-50 never touched; `tools/fewshot_run.py`
  never invoked; `api.done` never read. Cluster writes confined to
  `packs/c2k1clean_obj_orange_juice_pos_k1/*` and `results/*c2k1clean_obj_orange_juice_pos_k1*`
  (the v2 `dump_*.npz` scratch files were deleted after analysis).

STOP.
