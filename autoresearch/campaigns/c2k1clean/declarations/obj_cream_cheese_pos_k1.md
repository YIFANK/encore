# c2k1clean / obj_cream_cheese_pos_k1

Task: "pick up the cream cheese and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2k1clean_obj_cream_cheese_pos_k1/`.
No shared note file; every constant re-derived from this pack + debug seeds 51-65.

---

## v1 -- perception dump (no motion)

**Hypothesis.** Nothing about the eval scene is known. Dump cam_high/cam_arm_wrist
RGB-D, intrinsics, extrinsics and a base-frame point cloud on debug seeds so the
scene can be measured instead of guessed.

**Receipts.** `results/fs_..._v1` (seeds 51,53,55) + `results/fs_..._v1b`
(seeds 52,54,56-65). 15/15 episodes ran, 0/15 success (no motion, as intended).

**Evidence.**

* Home pose is identical to the pack demo's t=0: eef `[-0.1485, 0.0, 0.2613]`
  vs demo `[-0.1442, 0.0175, 0.2624]`; tool rotation is straight down
  (`R0 = [[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]]`), so the jaw axis is
  base **y** and demo ee values are directly comparable with live `api.eef()`.
* cam_high: K = fx=fy=618.04, c=(256,256); t_base_cam places the camera at
  (0.897, 0, 0.650) looking back and down. Depth is dense (262144/262144 finite).
* Table plane: dominant z mode at **0.005**; all table objects sit above it.
* Above-table clustering on a 15 mm xy grid, z band **[0.012, 0.165]**, min 80 px,
  yields **exactly 7 clusters in all 15 debug seeds** and excludes the arm
  (the arm reaches z=0.40 and its lowest point at the home pose is above 0.165).
  A z ceiling is required: without it the arm fuses with the two objects behind it.
* The 7 clusters (top-face centroid within 5 mm of ztop):

  | object | ctr (x,y) | ztop | top-face ext | top-face RGB |
  |---|---|---|---|---|
  | basket | (+0.006, +0.265) | 0.1437 | 0.159 x 0.171 | (140,139,135) |
  | tall carton A | (+0.070, -0.098) | 0.142 | 0.053 | (96,66,59) |
  | tall carton B | (-0.180, -0.080) | 0.143 | 0.052 | (89,64,30) |
  | can A | (+0.113, -0.199) | 0.081 | 0.065 | (68,61,52) |
  | can B | (-0.105, -0.240) | 0.081 | 0.066 | (58,59,65) |
  | flat ORANGE box | (+0.154, +0.029) | 0.0188 | 0.039 x 0.074 | (105,64,45) |
  | flat BLUE box | (-0.144, +0.058) | 0.0201 | 0.042 x 0.080 | (69,75,93) |

* **Target identity.** The pack demo's keyframe at t=54 (the frame where
  `gripper_cmd` flips to +1) shows the jaws closed on a small dark-blue box, and
  at t=138 that box is the only one missing from the table. Cropping that box out
  of `keyframes/demo0_t0000.png` and cropping the eval scene's flat blue cluster
  give the same dark-blue slab with a round pale logo. So the cream cheese is the
  scene's **one flat blue box**.
* **Discriminator.** `ztop < 0.035` isolates the two flat boxes (next tallest
  object is 0.081); `B - R > 8` then separates blue (+23..+24) from orange
  (-60..-61). This gave exactly one blue flat cluster in **15/15** debug seeds.
* **Perturbation range over seeds 51-65.** cream cheese x in [-0.148,-0.142],
  y in [+0.054,+0.063]; basket x in [-0.008,+0.020], y in [+0.249,+0.279].
  The other five objects are fixed to 3 decimal places. Long axis of the box is
  always base x (yaw 0/180 deg), so its 0.042 m width lies on the base-y jaw axis
  (max jaw opening measured 0.0778) -- no wrist yaw is needed.

**Verdict.** Scene fully measured; a fixed open-loop plan driven by per-episode
perception is feasible. Proceed to v2.

## v2 -- perceive, top-down pinch, carry, release

**Hypothesis.** Identify the box and basket per episode from the v1 measurements,
pinch the box top-down at the demo's own grip height, and open above the basket rim
at the demo's own release height.

**Calibration, all from the pack demo cross-referenced with debug-seed geometry.**

* Grip height: demo closes at ee z = **0.0091**; the box top measures
  **0.0201** -> grip point is `ztop - 0.011` (mid-slab).
* Release height: demo opens at ee z = **0.1789**; the basket rim measures
  **0.1437** -> release at `rim + 0.035`.
* Carry altitude: demo's ee_path6 carries at z = 0.24-0.25 -> 0.25.

**Mechanism detail that mattered.** `move_cartesian` stops as soon as the 3-D
error is under its 12 mm tolerance, so a single descent command can halt a full
centimetre above the intended grip height. Two counters: (a) an xy refine loop
that re-commands past the target so the residual exceeds the stop tolerance, and
(b) a second descent commanded 12 mm below the grip height, whose tolerance ball
therefore lands at or below it.

**Receipts.**

* Probe, seeds 51,53,55,57,59,61,63,65: **8/8**
  (`results/fs_c2k1clean_obj_cream_cheese_pos_k1_v2`).
* Per-episode sensor receipt (ep51): descend2 reached eef z = 0.0092 against a
  target of 0.0091; finger gap after close = **0.0422**, exactly the measured
  box width across the jaw axis, with effort 3.0 sustained through the lift --
  i.e. the box was verifiably in the jaws, not inferred from the success bit.
* Selection, full 15 debug seeds: see DECLARATION below.

---

# DECLARATION

**Frozen version: v2.**

* `packs/c2k1clean_obj_cream_cheese_pos_k1/program.py`
  md5 `a796ed269e21808e4a81355920d917a1`
  == `program_v2.py` md5 `a796ed269e21808e4a81355920d917a1`. Identical.
* **Selection receipt (full 15 debug seeds, one formal run): 15/15**
  `results/sel_c2k1clean_obj_cream_cheese_pos_k1_v2`
  (seeds 51-65, every episode `"benchmark_success": true`, no program errors).
* Archived versions: `program_v1.py` (perception dump, md5
  `66b337191a6612ea55ccc04a9ff755dc`), `program_v2.py` (frozen).

**Receipt chain**

| version | role | seeds | result | dir |
|---|---|---|---|---|
| v1 | perception dump, no motion | 51,53,55 | 0/3 (by construction) | `results/fs_c2k1clean_obj_cream_cheese_pos_k1_v1` |
| v1 | perception dump, no motion | 52,54,56-65 | 0/12 (by construction) | `results/fs_c2k1clean_obj_cream_cheese_pos_k1_v1b` |
| v2 | perceive + pinch + place | 51,53,55,57,59,61,63,65 | **8/8** | `results/fs_c2k1clean_obj_cream_cheese_pos_k1_v2` |
| v2 | **formal selection** | 51-65 (all 15) | **15/15** | `results/sel_c2k1clean_obj_cream_cheese_pos_k1_v2` |

**PROVENANCE**: present as a top-level literal dict in `program.py`, 24 entries,
every one sourced to the pack (`demo0` keyframes / ee_path6) or to a debug-seed
(51-65) measurement or to generic controller/camera mechanics. No LIBERO-specific
prior knowledge was used: object identity, colours, heights, footprints, grip
height, release height, table plane and jaw axis were all measured in v1.

**Oracle-free**: the program never reads the termination flag and never branches
on success. It verifies with its own sensors only (cluster geometry, `api.eef()`
residuals, finger gap and effort after the close).

STOP.
