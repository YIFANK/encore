# c2clean / obj_tomato_sauce_pos_k3

Intent: "pick up the tomato sauce and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_obj_tomato_sauce_pos_k3/` (K=3).

## v1 -- can api.log carry a frame?
Hypothesis: dump cam_high RGB-D through `api.log` and do perception offline.
Evidence (seeds 51,53,57,61,65): every log line is cut at 2000 chars, so a
single-message blob is useless. Geometry survived: K = 618 px focal, 512x512,
`t_base_cam` translation (0.897, 0, .); bare floor deprojects to z = 0.001;
`api.gripper()` at reset reads width_m 0.0778, effort 0.05.
Verdict: chunk the blob.

## v2 -- chunked RGB-D dump, all 15 debug seeds
Hypothesis: 1800-char chunks reassemble into a usable cloud.
Evidence: yes. Top-down height map (5 mm cells) over the runner's own
workspace x(-0.45,0.45) y(-0.45,0.52) gives, on **every** debug seed, the same
tabletop:

| cluster | centre (x,y) | footprint | top |
|---|---|---|---|
| can | (-0.1225, -0.2425) | 0.070 x 0.070 | 0.081 |
| carton A | (0.098, -0.203) | 0.060 x 0.060 | 0.143 |
| carton B | (0.050, -0.105) | 0.055 x 0.055 | 0.142 |
| flat box | (0.148, 0.025) | 0.090 x 0.055 | 0.029 |
| bottle | (-0.20, -0.085) | 0.030 x 0.050 | 0.113 |
| basket | varies | 0.160 x 0.175 | 0.143 |

Verdict, and the surprise of the cell: across seeds 51-65 the **props do not
move at all** (cell counts identical to the cell), only the basket does
(y 0.238 -> 0.278, x -0.015 -> 0.013) and the arm's start pose. The `_pos`
perturbation in this cell moves the destination, not the props. The arm at its
start pose swallows the bottle and a small box into one 0.485 m cluster.

## v3 -- aim at the demo's closing xy  (0/8)
Hypothesis: all three demos close at (0.054,-0.111), (0.056,-0.116),
(0.043,-0.108); the prop sitting there is the target.
Evidence (51,53,55,57,59,61,63,65): that is carton B. Picked and placed it
cleanly on all 8 -- effort 3.0 through the whole carry, drop at (0.007,0.263),
GIF shows the carton standing upright inside the basket -- and
`benchmark_success` **0/8**.
Two measurements from the same run refuted the aim:
* the descent commanded to eef z 0.050 **stalled at z 0.096** (residual
  0.053): the hand fouls a prop top 0.0455 above the eef origin, so no demo
  could have reached z 0.050 over a prop 0.142 m tall;
* the closed gap on carton B read 0.0530, but the demos carry at
  |q0-q1| = 0.0635 / 0.0611 / 0.0644.
Verdict: the demo xy is a decoy -- the demo layout is not the debug layout.
The demo's *mechanics* name the prop, not its coordinates.

## v4 -- identity from hold width + descent clearance  (8/8, then 15/15)
Hypothesis: the graded prop is the one that is (a) ~0.063 m across, and (b)
low enough that a descent to eef z 0.050 is not fouled, i.e. top < 0.0955, yet
tall enough that z 0.050 lands on its body, i.e. top > 0.055.
Evidence: exactly one cluster qualifies on every debug seed -- the can
(top 0.081, 0.065 across). Measured closed gap on it: **0.0616**, against the
demos' 0.0630. Probe 8/8; formal 15 seeds
`results/sel_c2clean_obj_tomato_sauce_pos_k3_v4` **15/15**.
Verdict: correct target, correct grasp height.

## v5 -- band-gate before grouping  (FROZEN, 15/15)
Hypothesis (perception only, motion unchanged): v4 clusters everything above
the floor and *then* filters by top, so a prop standing under the arm is
invisible -- on the debug seeds the arm already eats two props this way. Gate
the map to the graspable band (0.055, 0.0955) **first**, then cluster.
Evidence: offline over all 15 debug clouds the gate returns the can (136 cells)
plus slivers of basket wall and bottle body, all <= 20 cells and 0.005-0.015 m
wide; the `TGT_MIN_CELLS = 25` floor plus the hold-width ranking (can 0.065 vs
sliver 0.000 against the demo's 0.063) separate them by 30x. On-cluster: every
seed logs `band=1 pool=1`, same can, closed gap 0.0616.
Formal 15 seeds `results/sel_c2clean_obj_tomato_sauce_pos_k3_v5` **15/15**.
Verdict: frozen.

## Receipt chain
| version | run | seeds | result |
|---|---|---|---|
| v1 | fs_..._v1 | 51,53,57,61,65 | dump only |
| v2 | fs_..._v2 | 51-65 | dump only |
| v3 | fs_..._v3 | 8 odd | 0/8 |
| v4 | fs_..._v4 | 8 odd | 8/8 |
| v4 | **sel_..._v4** | 51-65 | **15/15** |
| v5 | **sel_..._v5** | 51-65 | **15/15** |

# DECLARATION

* Frozen version: **v5**.
  `packs/c2clean_obj_tomato_sauce_pos_k3/program.py` md5
  `d3155385d4ec2d0e5548289f9fcd6174` == `program_v5.py` (verified on cluster).
* Selection receipt: **15/15** on the full debug band 51-65,
  `results/sel_c2clean_obj_tomato_sauce_pos_k3_v5`.
* PROVENANCE present in program.py: 14 entries, every calibrated constant
  sourced to a pack field (DEMO_HOLD_W, GRASP_Z, RELEASE_Z, CARRY_Z, HOVER_Z),
  a debug-seed measurement (HAND_FOUL_ABOVE_EEF, TGT_TOP_BAND, TABLE_Z,
  PROP_Z_MIN, ARM_Z_MIN, BASKET_AREA_MIN, TGT_MIN_CELLS, OPEN_W) or the
  runner's own banner (WS).
* No shared note file was read or written. No forbidden path was opened.
