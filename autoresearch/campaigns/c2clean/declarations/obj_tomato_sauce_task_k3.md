# c2clean / obj_tomato_sauce_task_k3

Intent: **"Pick the bbq sauce and place it in the basket"** (re-authored cell;
the bddl filename says tomato sauce — the instruction is authoritative).

## Evidence read from the two packs

- `..._k3/pack.json` language = "pick up the **tomato sauce** and place it in the
  basket" (same TARGET = basket, different object). Its scene is **my** scene.
- `..._mate/pack.json` language = "pick up the **bbq sauce** and place it in the
  basket" (same OBJECT, a different scene / different distractor set).
- Pixel difference between each demo's first and last keyframe names the object
  that was removed from the table:
  - k3 scene → the **green/red striped can** (the tomato sauce). Its demo grasp
    xy (0.05, -0.11) matches a component I measure at a fixed (0.064, -0.099).
  - mate scene → an **amber bottle with a dark red cap** (the bbq sauce).
- My scene contains that same amber-bottle asset. So **target = the amber bottle**,
  distractor = the green/red can.
- mate grasp keyframes: successful closes at ee z = 0.0732, 0.0739, 0.1043;
  empty closes at 0.0576 and 0.1142 (gripper_state 0.0019 / 0.0011).
  Holding widths |l|+|r| = 0.027-0.037.
- Both packs release over the basket at xy ~ (0.00, +0.25), z 0.13-0.21, and
  carry at z 0.24-0.32.

## Debug-seed perception (probe0 / probe1, seeds 51-65)

`api.log` used as a data pipe (zlib+base64 chunks of 1900 chars, the FairApi
log cap). cam_high: K f=618.04, c=(256,256); t_base_cam places the camera at
(0.897, 0, 0.65) looking down ~32 deg. Table top is base z = 0.000.

Components above the table (identical on all 15 seeds unless noted):

| component | ztop | centre | extent |
|---|---|---|---|
| basket | 0.142 | (0.00, +0.26) | 0.156 x 0.170 |
| two cartons (merged) | 0.141 | (0.027, -0.212) | 0.27 x 0.09 |
| green/red can (tomato sauce) | 0.080 | (0.064, -0.099) | 0.062 x 0.069 |
| dark flat box | 0.028 | (0.159, +0.030) | 0.079 x 0.047 |
| small flat box | 0.017 | (-0.148, +0.060) | 0.070 x 0.038 |
| **amber bottle (bbq sauce)** | **0.1115** | varies | 0.02 x 0.045 |

Only the bottle moves with the seed: x in [-0.202, -0.191], y in [-0.088, -0.069].
Its own profile: y-width 0.045 (z<0.06), 0.036 (z 0.06-0.08), 0.024 (z>0.08 = cap).
The oblique view cuts the near x edge of the body, so the **cap disc** (top 8 mm,
unoccluded) is used for the xy centre; body and cap y-midlines agree to 1 mm.

Selection rule (rank, not threshold): among components with footprint < 0.09 m in
both axes and >= 300 px, take the **tallest** — bottle 0.1115 beats can 0.080 and
the flat boxes. Cartons/basket are excluded by footprint.

## Version log

### v1 — perceive the tallest small-footprint prop, body grasp 38 mm below its top
Hypothesis: grasp at ztop-0.038 = 0.0735 (inside the mate pack's successful band
0.073-0.104, on the 0.036 m body), carry at z 0.30, release at (basket xy, 0.19).
Three attempts with +-0.012 m jaw-axis nudges, each verified by gripper width.
Evidence: pending.
Verdict: **8/8** probe (51,53,...,65), dir `results/fs_c2clean_obj_tomato_sauce_task_k3_v1`.
Formal selection **15/15** on all debug seeds 51-65, dir
`results/sel_c2clean_obj_tomato_sauce_task_k3_v1`. Holding width reads 0.0360-0.0366
with effort 3.00 on every seed — matching the mate pack's 0.037 body grasps —
and no seed needed the retry nudge (attempt 0 succeeded on all 15).

### Aim-envelope probe (not a version; v1 with a forced grasp offset)
The x centre is the estimate I trusted least (the oblique cam_high view cuts the
near x edge of the bottle body, so x comes from the cap disc alone). Displacing
the grasp aim along x by +-0.012 m:

| offset | result | dir |
|---|---|---|
| x +0.012 | 4/4 (51,55,59,63) | `fs_..._envxp` |
| x -0.012 | 4/4 (51,55,59,63) | `fs_..._envxm` |

Holding width stayed 0.036 in both — the grasp tolerates at least +-12 mm in x,
about 2x the full seed-to-seed spread of the bottle (11 mm in x, 19 mm in y), so
the 15/15 is not a knife-edge fit.

## DECLARATION

- **Frozen version: v1.** `packs/c2clean_obj_tomato_sauce_task_k3/program.py`
  md5 `9444ba15adf6aa3ee38f83afd8ae6f87` == `program_v1.py` (same md5).
- **Selection receipt: 15/15** on the full 15 debug seeds 51-65 —
  `results/sel_c2clean_obj_tomato_sauce_task_k3_v1/results.jsonl`.
- **Receipt chain:**
  - `probe0` — FairApi/scene reconnaissance, 4 seeds, no motion (0 grasps).
  - `probe1` — RGB-D dump for all 15 debug seeds, no motion.
  - `v1` — 8/8 probe (`fs_..._v1`), then 15/15 formal (`sel_..._v1`).
  - envelope probes `envxp` / `envxm` — 4/4 each at +-12 mm grasp offset.
- **PROVENANCE** present in `program.py` as a top-level literal dict covering all
  15 calibrated constants; every source is either the two named packs' contents,
  a debug-seed (51-65) measurement, or generic controller/camera mechanics.
- `api.done` is never read; success was never queried at runtime — the program
  verifies itself with gripper width/effort only.
