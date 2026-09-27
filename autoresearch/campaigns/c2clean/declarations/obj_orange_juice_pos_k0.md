# c2clean / obj_orange_juice_pos_k0

Intent: "pick up the orange juice and place it in the basket". Zero demos (k0).
Everything below is derived from debug seeds 51-65 only.

## Scene (debug seeds, cam_high cloud)

Table plane z = 0.0010 (modal z inside the runner's workspace crop
x(-0.45,0.45) y(-0.45,0.52), echoed by fair_run's own banner).

Seven clusters above table+0.015, once the arm is stripped with an upper
z bound (arm ztop 0.486, all props <= 0.147):

| cluster | ctr (x,y) | h | dx x dy | top-band r-b |
|---|---|---|---|---|
| basket | (+0.008,+0.256) | 0.143 | 0.157 x 0.171 | 0.6 |
| ketchup | (+0.165,+0.029) | 0.147 | 0.036 x 0.062 | 3.5 |
| **orange juice** | **(-0.144,+0.059)** | **0.142** | **0.040 x 0.053** | **80.7** |
| dressing | (-0.186,-0.080) | 0.147 | 0.033 x 0.061 | -12.0 |
| brown bottle | (+0.109,-0.206) | 0.112 | 0.027 x 0.048 | 59.7 |
| box | (+0.058,-0.099) | 0.028 | 0.080 x 0.048 | 22.3 |
| orange box | (-0.117,-0.240) | 0.018 | 0.075 x 0.039 | 50.6 |

Only the basket, the brown bottle and the orange box jitter across seeds
(~1 cm); the rest are pinned. The program still perceives everything at
runtime rather than hardcoding, since the eval band is seeds 1-50.

## Identification rule
Footprint gate (dx,dy < 0.10) drops the basket; height gate (h >= 0.12)
keeps the three tall props; then rank by **top-band redness** (mean r-b over
the top 40 mm of the cluster). Juice 80.7 vs ketchup 3.5 vs dressing -12.0 —
margin 77. Ranking, not thresholding. The height gate matters: the brown
bottle scores 59.7 and the orange box 50.6, so colour alone is only a
margin-21 cue while colour-within-the-tall-class is margin-77.

## Version chain

- **v1** — pure perception probe, no motion. `api.log` caps a message near
  2000 chars, so the first run's 3000-char base64 chunks were silently
  truncated (zlib "invalid distance too far back"). Re-chunked at 1800 and the
  RGB-D came through clean. 0/2, expected (no motion).
- **v2** — calibration probe. Close-ladder at eef z 0.26/0.23/0.20/0.17 over
  the carton: **every rung closed on air** (width 0.0010, effort 0.05), so the
  fingertips sit far closer to the eef reference than a 5 cm guess. Also
  measured move tracking: xy error ~0.5 mm, z lands a consistent **+10.6 mm
  high**. Wrist plan view was contaminated by the gripper's own body (ztop
  0.359 above the eef) — fixed by excluding points above eef_z-0.03.
- **v3** — ladder extended to 0.16/0.14/0.125/0.11/0.095. Air at eef 0.1508,
  **grip at 0.1355, 0.1206, 0.1057** (width ~0.053, effort 3.00). Carton top
  is 0.1430, so the grip band opens at roughly eef == ztop - 0.007.
  Closed width **0.053 == the carton's y span**, so with R_DOWN the jaws
  close along **base y**. That settles the aim: y is the axis the oblique cam
  measures cleanly (y bbox mid stable to ~1 mm across seeds), while x is the
  poorly-observed axis (top face reads as a 2-7 mm sliver because cam_high
  looks down at only ~32 deg, so the top face is nearly edge-on and its far
  part is a mixed-pixel ramp). Closing along the well-measured axis is the
  reason no wrist-refine step was needed.
- **v4** — full pipeline. perceive -> open -> hover 0.300 -> descend 0.120 ->
  close -> verify -> lift -> transit -> lower 0.250 over the rim centre ->
  open -> retreat. All commanded z values carry the -0.0106 bias cancel.
  Probe (51,53,...,65): **8/8**.

## Geometry used by v4
- Grasp at eef z = 0.120: mid-band of the measured grip window
  (0.1057-0.1355), 23 mm below the carton top.
- Hang = Z_GRIP - table = 0.119 (carton bottom sits on the table at grasp
  time, so after a clean lift it trails the eef by exactly that).
- Basket rim top 0.1437, rim centre from the **rim bbox mid** (points within
  12 mm of basket ztop) — the cluster *mean* is biased ~2 cm by the visible
  near wall, the bbox mid is not.
- Carry at 0.300 (carton bottom 0.181, clears the 0.147 tallest prop and the
  0.144 rim). Drop at 0.250 puts the carton bottom at 0.131, i.e. below the
  rim, inside the basket, before the jaws open.

- **v5** (FROZEN) — v4 with three robustness changes, none of which move the
  aim on debug seeds:
  1. basket chosen by **largest footprint area** (0.027 m^2 vs <=0.0038 for
     every prop, a 7x gap) instead of a 0.10 m threshold — ranking, not
     thresholding;
  2. the y aim (the jaw-closing axis) comes from the **reddest 40% of the
     carton's top band** rather than the cluster bbox mid. On debug seeds the
     two agree to 0.7 mm, but if the juice ever fuses into one footprint
     cluster with a neighbour, the red sub-blob still names the juice while
     the bbox mid would not. x is left at the bbox mid: it is the poorly
     observed axis, but it is also the forgiving one (finger length, not jaw
     gap), so it degrades gracefully;
  3. one re-perceive-and-retry if the close returns effort < 2.0 or
     width < 0.03.
  Formal 15/15. The retry never fired; all 15 grasps closed first-try at
  width 0.0536 / effort 3.00, and the y aim was identical (+0.0597) on every
  seed.

## Aim envelope (margin, not just score)
v5 re-run with the y aim deliberately displaced, seeds 51,55,59,63:

| y offset | result |
|---|---|
| +0.012 | 4/4 |
| -0.012 | 4/4 |

So the closing-axis envelope is at least +/-12 mm, against a cross-seed
measurement spread of ~1 mm and a geometric half-clearance of
(0.078-0.053)/2 = 12.5 mm. The grasp is not living near its margin.

## DECLARATION

- **Frozen version: v5.** `program.py` md5 `54051b5ff00caae6b7fb3a4dfd5a3b72`
  == `program_v5.py` md5 `54051b5ff00caae6b7fb3a4dfd5a3b72`.
- **Selection receipt: 15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_obj_orange_juice_pos_k0_v5`
  (15 x `"benchmark_success": true` in results.jsonl).
- **Receipt chain:**
  | version | run | seeds | result |
  |---|---|---|---|
  | v1 | fs_..._v1 / _v1b | 51,53 / 8 probe | 0/2, 0/8 (perception probe, no motion) |
  | v2 | fs_..._v2 | 51,53 | 0/2 (calibration ladder, all rungs closed on air) |
  | v3 | fs_..._v3 | 51,53 | 0/2 (calibration ladder, grip window found) |
  | v4 | fs_..._v4 | 8 probe | 8/8 |
  | v4 | sel_..._v4 | 51-65 | **15/15** |
  | v5 | sel_..._v5 | 51-65 | **15/15** (frozen) |
  | v5 y+12mm | fs_..._env_y0012 | 51,55,59,63 | 4/4 (envelope) |
  | v5 y-12mm | fs_..._env_ym0012 | 51,55,59,63 | 4/4 (envelope) |
- **PROVENANCE present** in program.py, covering every calibrated constant;
  all sources are debug-seed measurements, the runner's own workspace banner,
  or generic controller/camera mechanics. No pack was supplied (k0) and none
  was used. No `.done` read anywhere in the program.
- Eval seeds 1-50 were never touched.
