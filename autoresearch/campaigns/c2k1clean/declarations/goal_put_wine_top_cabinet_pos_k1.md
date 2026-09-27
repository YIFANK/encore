# c2k1clean / goal_put_wine_top_cabinet_pos_k1

Intent: "put the wine bottle on top of the cabinet". K=1 pack, `_pos` cell.
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51-65).

## probe1 — what is actually in the scene?

Hypothesis: the pack's xy anchors cannot be trusted on a `_pos` cell, so
measure everything from cam_high instead.

Method: a perception-only program that dumps cam_high + cam_arm_wrist RGB-D
(zlib+base64 through `api.log`, 1800-char chunks) plus intrinsics/extrinsics,
eef and gripper state. 6 seeds, 72 sim steps each, no motion.

Evidence (decoded offline):

* cam_high is 512x512 with real metric depth (0.75-3.07 m), K fx=fy=618,
  camera at base (0.659, 0, 1.610) looking along (-0.778, 0, -0.628).
  `api.eef()` and the depth cloud are in the SAME frame (the gripper
  deprojects to x≈-0.18, z 1.22-1.26 with the eef reported at
  (-0.208, 0, 1.173)).
* table top z = 0.901 (modal z, 134k of 214k workspace points).
* **The pack images and the debug seeds are mirror images of each other.**
  In `keyframes/demo0_t0000.png` the cabinet is the dark box at -y and the
  stove slab is at +y; on every debug seed the (grey, three-drawer) cabinet is
  at +y and the stove is at -y. The demo's release point (-0.007, -0.226) and
  its grasp point (-0.198, -0.066) are therefore both decoys — in fact the
  demo grasp xy lands exactly on the metal bowl in seed 51 (-0.199, -0.066).
* Cabinet top: a perfectly flat slab, z = 1.1276 on all 6 seeds, visible
  extent x[-0.53,-0.275] y[0.09,0.325], ~2000 cells of 25 mm². The wine rack
  (the other tall fixture) is a set of tilted slats spanning z 1.10-1.20, so
  **flatness separates the two** without any colour or prior.
* Wine bottle: the only small-footprint prop that is tall. Top z = 1.059
  (= table + 0.159), footprint ~0.0012 m². Vertical profile: body chord
  0.042 m at z 0.96-0.98, **neck chord 0.014-0.016 m at z 1.00-1.045**, gold
  cap above 1.05.

Verdict: perception is the whole cell; the pack contributes the mechanism
(where on the bottle to grip, and how far the bottle hangs below the eef).

## What the pack actually says (mechanism, not position)

* `ee_path6` flips the gripper command between t=30 (z=1.0404) and t=40
  (z=1.0223); keyframe t=32 is z=1.0291. With the bottle standing on a 0.900
  table that is **0.123-0.129 m above the support** — and with the bottle top
  at 1.059 it is **0.030-0.037 m below the bottle top**, i.e. the NECK.
* Keyframe t=85 gripper_state [0.0049, -0.0116] -> closed gap 0.0165 m, which
  matches the measured neck chord and rules out a body grasp.
* Release: eef z = 1.2515 with the gripper opening at t=85. 1.2515 - 0.1235 =
  1.128 = the cabinet top height measured on the debug seeds. So the release
  rule is **surface + hang**, and hang = (grasp z − support z) ≈ 0.1245.
* `ee_path` peaks at 1.2945, i.e. 0.043 above the release height -> carry
  clearance ≈ 0.045.

## v1 — perceive both ends, transfer only the pack's heights

Hypothesis: bottle neck centre + cabinet flat-top centroid + the pack's two
heights (grasp = top−0.033, release = slab + hang) is the whole task.

Neck xy estimator: the y chord is cross-view (the camera looks along −x), so
it is unbiased -> `y_c = (y_min+y_max)/2`, `r = (y_max−y_min)/2`; in x only the
near arc is visible, so `x_c = x_max − r`, cross-checked against `x_min`
(the two agree to 1 mm on seeds 51/57/65). Measured r = 0.0069-0.0070 on all
six probe seeds.

Sequence: open, hover at bottle_top+0.085, descend to top−0.033, close, lift to
slab+hang+0.045, translate to the slab centroid, descend to slab+hang+0.002,
open, retreat.

Receipts:
* probe `results/fs_c2k1clean_goal_put_wine_top_cabinet_pos_k1_v1`
  (51,53,55,57,59,61,63,65): **8/8**.
* selection `results/sel_c2k1clean_goal_put_wine_top_cabinet_pos_k1_v1`
  (51-65, all 15): **15/15**.
* Margins: closed finger gap 0.01496 m with effort 3.0 on every seed, and the
  gap is unchanged from close through transit to release (no slip). All move
  residuals ≤ 0.012 (= POS_TOL). 128-133 sim steps per episode.
* The place descent converges ~8 mm high (eef 1.2623 vs commanded 1.2543,
  residual 0.0093), so the bottle is released ~10 mm above the slab and drops;
  it stands anyway on all 15 seeds. This is the thinnest margin in the run and
  the obvious first thing to tighten if the cell ever regresses.

Verdict: accept. No v2 — 15/15 on the full debug band with a clean
sensor-side receipt (effort 3.0 + constant gap) for the grasp.

## DECLARATION

* Frozen version: **v1**.
  `program.py` md5 `398ab3732f8d47cf73244890ee9704e5` ==
  `program_v1.py` md5 `398ab3732f8d47cf73244890ee9704e5`
  (verified on the cluster in
  `packs/c2k1clean_goal_put_wine_top_cabinet_pos_k1/`).
* Selection receipt (full 15 debug seeds): **15/15**, dir
  `results/sel_c2k1clean_goal_put_wine_top_cabinet_pos_k1_v1`.
* Per-version receipt chain:
  * probe1 (perception dump, no motion) — 6 seeds, 0/6 by construction,
    `results/fs_c2k1clean_goal_put_wine_top_cabinet_pos_k1_probe1`.
  * v1 — probe 8/8 `…_v1`, selection **15/15** `sel_…_v1`.
* `PROVENANCE` present in `program.py`, covering GRASP_BELOW_TOP, HANG_M,
  NECK_GAP_M, TABLE_BAND, SLAB_MIN_H, SMALL_FOOTPRINT_M2, GRID_RES,
  CARRY_CLEAR_M, HOVER_M. Every constant is sourced to this pack or to
  debug-seed cam_high measurements; no LIBERO prior was used (the fixture
  swap above is exactly why none could have been).
