# c2k1clean / obj_salad_dressing_task_k1

Intent: **"Pick the tomato sauce and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65; eval 1-50 blind).

## Pack reading (pack.json + keyframes/ of the two named packs only)

| | k1 pack | mate pack |
|---|---|---|
| language | pick up the salad dressing and place it in the basket | pick up the tomato sauce and place it in the basket |
| grasp keyframe | t=51 ee=(0.0787,-0.1025,0.1238) rpy=(3.087,-0.034,-0.166) | t=54 ee=(0.0543,-0.1108,0.0460) rpy=(3.009,-0.630,-0.056) |
| release keyframe | t=128 ee=(0.0183,0.2201,0.1709) | t=112 ee=(-0.0402,0.2591,0.1808) |
| carry gripper_state | [0.0091,-0.0089] (thin bottle neck) | [0.0265,-0.0370] -> width 0.0635 |

The two packs are different scenes. The k1 pack is the scene I am evaluated
in (7 props: ketchup bottle, tomato-sauce can, blue can, green salad-dressing
bottle, tall red box, flat dark box, basket). The mate pack is a different
scene that contains the same tomato-sauce can.

Keyframe crops of the two cans (k1 t=0 vs mate t=0) are pixel-identical: dark
grey lid, red/brown body with a green mid-band. That is the object the intent
names, and the mate demo's grasp/carry numbers are therefore calibration for
it; the k1 demo's release is calibration for the basket half.

Two decoys the packs plant: the k1 demo's grasp xy (0.079,-0.103) is the
SALAD DRESSING, not my target; the mate demo's grasp xy (0.054,-0.111) is the
can but in the other scene's layout. Only the *heights* and the *hold width*
transfer.

## Version log

### v0 — perception probe, pixel-space components (no motion)
Hypothesis: 4-connected components of "above the table plane" in image space
will separate the props.
Evidence (`results/fs_..._v0`, seeds 51/53/55/57): table_z=0.0030; 7 clusters,
but one of them (n=3530, dx=0.303) fused the tomato-sauce can with the salad
dressing bottle 20 cm away in base x — a grazing camera drapes a tall prop's
body over the prop in front of it.
Verdict: geometry pipeline (cloud + table fit) is sound; pixel-space grouping
is not. 0/4, no motion attempted.

### v1 — perception probe, top-down XY occupancy clustering (no motion)
Hypothesis: clustering a 1 cm base-frame XY occupancy grid instead of pixels
un-fuses props that are separated in the world but adjacent in the image.
Evidence (`results/fs_..._v1`, seeds 51/53/55/57): exactly 7 clusters on every
seed, all identified:

| prop | xy (ep51) | ztop | h | dx,dy | body colour |
|---|---|---|---|---|---|
| tomato sauce can | (-0.177,-0.081) | 0.081 | 0.078 | 0.064,0.070 | (47,35,17) warm |
| blue can | (-0.126, 0.056) | 0.081 | 0.078 | 0.063,0.070 | (36,46,66) cool |
| ketchup bottle | (-0.103,-0.236) | 0.148 | 0.145 | 0.033,0.063 | (88,65,56) |
| basket | ( 0.076, 0.255) | 0.142 | 0.139 | 0.157,0.171 | (136,135,132) |
| salad dressing | ( 0.066,-0.099) | 0.147 | 0.145 | 0.036,0.062 | lid (21,56,35) |
| flat dark box | ( 0.108,-0.203) | 0.020 | 0.017 | 0.079,0.042 | — |
| tall red box | ( 0.175, 0.027) | 0.138 | 0.136 | 0.046,0.052 | (126,77,64) |

Cross-check: the k1 demo's grasp xy (0.0787,-0.1025) lands on the salad
dressing cluster (0.066,-0.099) — the pipeline reproduces the pack's own
geometry.
Target rule that follows: the tomato sauce is the **short round warm** prop —
h in [0.055,0.105], footprint 0.045-0.090 in both axes, body r>b. Its only
size twin is the blue can (b>r), so one colour comparison names it. The
basket is the only cluster with h>0.10 and both extents >0.115.
Across seeds 51/53/55/57 the props are fixed; only the basket (x 0.058-0.078,
y 0.251-0.261) and the two boxes jitter, so the basket must be perceived.
Verdict: adopted as the perception front end. 0/4, no motion attempted.

### v2 — first acting version
Hypothesis: straight-down grasp of the can body at eef z = can_top - 0.035,
lift to 0.30, transport, release at basket centre + 0.035 over the rim.
Constant cross-check: can_top 0.081 - 0.035 = 0.046 = the mate pack's own
grasp keyframe z, arrived at independently. Release drop 0.035 ~ the k1
pack's 0.171 release vs the 0.142 rim measured here.
Evidence (`results/fs_..._v2`, seeds 51,53,...,65): perception named the
tomato sauce on 8/8; the grasp held on 8/8 (closed width 0.0616, effort 3.00,
still 3.00 after the lift to z=0.29). The episode then died on
`api.move_path` -> `'LiberoRobot' object has no attribute 'move_path'`:
move_path is a robosuite-only primitive on this backend.
Two calibrations fell out of the logs: the extent MIDPOINT, not the median,
is the right cluster centre (the basket's median x is 0.076, dragged to the
dense far wall; its midpoint is 0.008, which is where the k1 pack released at
x=0.018), and a commanded descent under-reaches by ~9 mm (commanded 0.046,
reached 0.0549).
Verdict: 0/8, but pick confirmed; only the transport primitive was wrong.

### v3 — transport as a move() chain
Hypothesis: the only defect in v2 is move_path; a two-waypoint move() chain at
z=0.30 does the same transport.
Evidence (`results/fs_..._v3`, seeds 51,53,...,65): **8/8 benchmark_success.**
Post-release re-perception shows the tomato-sauce cluster gone from the table
on every seed while the basket cluster grows. Release eef landed at
(0.011,0.258,0.186) on ep51 against a 0.142 rim.
Verdict: the mechanism works end to end.

### v4 — v3 plus a grasp-retry ladder  (FROZEN)
Hypothesis: v3 proceeds to the basket even when the close fails. A ladder that
re-descends at -14 mm and then +14 mm, gated on `effort >= 1.5` both after the
close and after the lift, can only help — it is a no-op when the first grasp
holds.
Evidence: **15/15 benchmark_success** on the full debug band,
`results/sel_c2k1clean_obj_salad_dressing_task_k1_v4`. The ladder never fired:
all 15 episodes report `a0 closed width=0.0616 effort=3.00`, confirming it is
inert on the nominal path.
Verdict: **selected and frozen.**

## DECLARATION

- **Frozen version: v4.** `packs/c2k1clean_obj_salad_dressing_task_k1/program.py`
  md5 `c43fbd6ced8b494615fa36deb9aee754` ==
  `packs/c2k1clean_obj_salad_dressing_task_k1/program_v4.py` (same md5).
- **Selection receipt: 15/15** on the full 15-seed debug band (51-65),
  directory `results/sel_c2k1clean_obj_salad_dressing_task_k1_v4`
  (`results.jsonl`, 15 lines with `"benchmark_success": true`).
- **Per-version receipt chain**
  | ver | run dir | seeds | result |
  |---|---|---|---|
  | v0 | `results/fs_..._v0` | 51,53,55,57 | 0/4 — perception probe, no motion |
  | v1 | `results/fs_..._v1` | 51,53,55,57 | 0/4 — perception probe, no motion |
  | v2 | `results/fs_..._v2` | 51,53,...,65 | 0/8 — pick held 8/8, died on `move_path` |
  | v3 | `results/fs_..._v3` | 51,53,...,65 | **8/8** |
  | v4 | `results/sel_..._v4` | 51..65 (all 15) | **15/15** — frozen |
- **PROVENANCE**: present as a top-level literal dict in program.py, covering
  all 12 calibrated constants (WS_X, WS_Y, TABLE_BAND, CELL, ARM_H, CAN_H,
  CAN_D, GRASP_DROP, LIFT_Z, DROP_OVER_RIM, HOLD_EFFORT, GRASP_LADDER). Every
  source is one of: the two named packs' pack.json/keyframes, a debug-seed
  (51-65) measurement logged by v0/v1/v2/v3, or generic camera/controller
  mechanics. No LIBERO-specific prior knowledge was used: the table height,
  every prop height and footprint, the can's colour signature, the grasp depth
  and the basket rim were all re-measured here.
- Eval band (seeds 1-50) was never touched.

STOP.
