# c2k1clean / obj_tomato_sauce_pos_k1 — NOTES

Intent: *pick up the tomato sauce and place it in the basket*.
Runner: `tools/fair_run.py` only. Pack: K=1 demo, 4 keyframes.

## Pack reading (the only task-specific input)

| t | ee xyz | grip cmd | gripper_state | reading |
|---|--------|----------|---------------|---------|
| 0 | (-0.159, -0.005, 0.239) | -1 (open) | 0.0362/-0.0362 | home |
| 54 | (0.054, -0.111, 0.046) | +1 (close) | 0.0394/-0.0396 | at the grasp, jaws still open |
| 112 | (-0.040, 0.259, 0.181) | -1 (open) | **0.0265/-0.0370** | over the basket, **carrying** |
| 131 | (-0.006, 0.247, 0.204) | -1 | 0.0389/-0.0393 | retreat, empty |

- lowest ee z in `ee_path` = **0.0489** (t=60) → the grasp height.
- carried jaw span |0.0265| + |0.0370| = **0.0635 m** → the target is ~6.4 cm wide.
- transport runs at z ≈ 0.18–0.23; release at z = 0.1802.
- The demo keyframe images show the removed object between t=0 and t=131: a
  **squat lidded can** (grey lid, red/green band). Every other prop in the
  frame is a tall box, a thin bottle, a small can or a flat slab.

## probe0 — perception dump (seeds 51, 53, 55)
`results/fs_c2k1clean_obj_tomato_sauce_pos_k1_probe0`

- cam_high: 512², K f=618.04, c=(256,256); T_base_cam puts the camera at
  (0.897, 0, 0.65) looking back and down. Image **right = +y**, image **down = +x**.
- table plane (modal cloud z) = **0.0021**, identical on all three seeds.
- Above-table clusters (1 cm grid), stable across the three seeds:

| cluster | xy | height | footprint | what it is |
|---|---|---|---|---|
| C2 | (-0.106, -0.239) | 0.079 | 0.062 × 0.065 | **lidded can → the tomato sauce** |
| C5 | (0.118, -0.197) | 0.141 | 0.053 × 0.053 | tall box |
| C4 | (0.070, -0.098) | 0.140 | 0.050 × 0.054 | milk carton |
| C0 | (-0.19, -0.074) | 0.111 | 0.026 × 0.049 | bottle |
| C6 | (0.159, 0.029) | 0.027 | 0.082 × 0.049 | flat slab |
| C3 | (0.03, 0.26) | 0.144 | 0.155 × 0.170 | **basket** |
| C1 | (-0.122, 0.004) | 0.402 | — | robot column (+ a small can merged into it) |

**C2's footprint 0.062/0.065 is the only match for the demo's 0.0635 carried
jaw span**, and it is the only squat cylinder. Verdict: identify the target by
`|width − 0.0635| + |height − 0.079|`; the runner-up is >0.06 away.

Note: the scene is *not* the demo's scene (this is the `_swap` suite and a
`_pos` cell) — the demo's grasp xy (0.054, -0.111) lands on the milk carton
here, so the demo xy is a decoy and everything must come from perception.

## Versions

### v0 (probe_v0.py) — perception dump, no manipulation
Hypothesis: the pack alone cannot name the target's position (this is a `_pos`
cell on the `_swap` suite), so the scene must be measured.
Evidence: `results/fs_c2k1clean_obj_tomato_sauce_pos_k1_probe0`, seeds 51/53/55
— the table plane and the seven clusters tabulated above; the film-strip frame
confirms the lidded can. 0/3 (no manipulation attempted).
Verdict: target = the squat lidded can; identify by footprint + height.

### v1 — perceive → rank by (width, height) → grasp → basket
Hypothesis: arm-column removal (any 1 cm xy cell carrying a point above
z0+0.22) unfuses the props; the (w, h) signature names the can; the demo's
table-relative grasp height 0.0468 and rim clearance 0.036 transfer.
Evidence: probe `fs_..._v1` **8/8** (51,53,…,65); formal
`sel_c2k1clean_obj_tomato_sauce_pos_k1_v1` **15/15**. Grasp closed on the can
on the *first* ladder rung in every episode (effort 3.0, width 0.0622);
identification margin 0.0016 vs 0.0664 for the runner-up.
Verdict: works, but the arm filter deletes whole columns — the small can at
(-0.145, 0.062) is chopped into two 0.017 m fragments (C1/C3 in the ep53 log)
because arm links pass above it. A target parked there would be lost.

### v2 — 3-D voxel connected components + a re-look fallback
Hypothesis: replacing the 2-D column filter with 26-connected 1 cm voxel
components keeps a prop that merely *hides under* an arm link (the link floats;
it does not touch), and dropping components whose top exceeds z0+0.22 still
removes the robot. If the best score is nonetheless worse than 0.030 the arm is
standing in front of the can, so park at z=0.42 and re-capture.
Evidence: probe `fs_..._v2` **8/8**; formal
`sel_c2k1clean_obj_tomato_sauce_pos_k1_v2` **15/15**. The arm-adjacent can is
now one 424-point cluster instead of two fragments (ep53 log, C5), confirming
the mechanism. Margin 0.0011 vs 0.0664. The fallback never fired on debug.
Verdict: ties v1 on the debug band and strictly dominates it off-band.

## Caveat on debug coverage (honest limit)
The `_pos` perturbation is **very weak on seeds 51–65**: the tomato sauce can
sits at (-0.118, -0.242) on *all fifteen* seeds; only the basket (±0.02 m in
xy) and the bottle / milk carton (±0.01 m) move at all. So 15/15 certifies the
pipeline but does **not** exercise large target displacement. Every stage is
nevertheless closed-loop on perception (no demo xy is ever used as a position —
the demo's own grasp xy lands on the milk carton in this scene), and the
five-rung grasp ladder plus the effort/width hold check are the guards for the
displacement the eval band may contain.

## DECLARATION

- **Frozen version: v2.** `packs/c2k1clean_obj_tomato_sauce_pos_k1/program.py`
  md5 `e299c467404c92deabb65c7153711670` == `program_v2.py` (verified on the
  cluster).
- **Selection receipt: 15/15** on the full 15 debug seeds (51–65),
  `results/sel_c2k1clean_obj_tomato_sauce_pos_k1_v2`.
- **Receipt chain**
  | version | probe (8 seeds) | formal (15 seeds) | dir |
  |---|---|---|---|
  | probe_v0 | — (perception only) | — | `fs_..._probe0` |
  | v1 | 8/8 | **15/15** | `fs_..._v1`, `sel_..._v1` |
  | v2 (frozen) | 8/8 | **15/15** | `fs_..._v2`, `sel_..._v2` |
- **Tie-break v2 over v1:** both are 15/15 on debug, so selection cannot
  separate them there. v2 is chosen on mechanism: its 3-D component filter is a
  strict superset of the layouts v1 can read (demonstrated on the arm-adjacent
  can), and its fallback only fires where v1 would already be guessing.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  DEMO_HOLD_W, DEMO_GRASP_DZ, DEMO_OBJ_H, DEMO_RELEASE_DZ, ARM_Z, CARRY_Z,
  CELL, OPEN_W, WORKSPACE, BAD_SCORE, PARK — every one sourced to a pack field
  or a debug-seed measurement. No LIBERO prior knowledge was used.
- Eval seeds 1–50 were never touched; `--split eval` was never run;
  `tools/fewshot_run.py` was never invoked; `api.done` is never read.

STOP.
