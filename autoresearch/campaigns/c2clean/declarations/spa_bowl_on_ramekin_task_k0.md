# c2clean / spa_bowl_on_ramekin_task_k0 (K=0, no demonstrations)

Intent: *"Pick the akita black bowl on the cookie box and place it on the plate"*.
Everything below was derived from the intent sentence plus debug seeds 51-65.
No pack, no note file, no prior-campaign constants.

## Scene, as read from debug seeds (v2 datapipe)

Table top z = 0.9014. Six clusters; identity confirmed by reprojecting each
cluster's bbox onto the cam_high RGB:

| cluster | centre | ztop | role |
|---|---|---|---|
| round rim, r=0.0553 | (~0.07, ~0.03) | 0.9712 | **target bowl**, standing on the red-checkered cookie box |
| round-ish, r=(0.022,0.053) | (-0.23, +0.20) | 1.0010 | distractor bowl on the plain ramekin |
| disc w=h=0.137 | (~0.06, ~0.19) | 0.9207 | **plate** (destination) |
| slab w=0.19 | (-0.26, -0.13) | 0.9323 | stove top (plate impostor) |
| box w=0.215 | (+0.06, -0.22) | 1.1000 | cabinet |
| r=(0.039,0.013) | (-0.41, -0.14) | 0.9606 | robot base fixture (bowl impostor) |

The instruction's target is the bowl **on the cookie box**, not the one named by
the bddl filename; the benchmark bit graded that bowl (v5 onward scored on it).

Selectors that hold on all 15 seeds with large margin: the target bowl is the
only prop with a circular rim (|rx-ry| < 0.0006; distractor 0.030, robot base
0.027) at ztop = TABLE+0.070 (distractor TABLE+0.100); the plate is the disc at
TABLE+0.019, w=h=0.135-0.140 (stove slab TABLE+0.031, w=0.19), with x > -0.12
excluding the stove and base. Both props are perturbed per seed (bowl centre
ranges 27mm in x, 30mm in y), so both are perceived, never hard-coded.

## Mechanism

Rim diameter 0.111 m exceeds the maximum jaw opening 0.0794 m, so the bowl
cannot be straddled. The wall flares outward going up (inner radius 0.0336 at
z=0.930 rising to 0.0555 at the rim), so a centred descent would have to drive a
finger through the wall: at any height above z≈0.937 the wall sits outside the
half-span 0.0397, below it inside. The only top-down grip is a **rim pinch** --
park the tool centre near the wall on the +y side (the jaws close along world y
when rotation=R_DOWN), drop the fingertips D_PINCH=0.020 below the rim so one
finger is inside the bowl and one outside, and close.

Heights: fingertips sit TIP_DZ = 0.0075 below the EEF frame. The open-loop
tracking bias is pose dependent (+7 to +11 mm in z), so every pose is closed-loop
(`goto` re-issues with the observed error folded in, bounded to +/-0.030 over 3
tries). Release height comes from the bowl's own height 0.054 (cookie-box top
0.9172 -> rim 0.9712), so the base lands 2mm above the plate top.

**The gripper effort flag is a false negative here.** After the lift it reads
effort 0.05 / width 0.0048 on every seed, yet the GIF shows the cookie box empty
and the bowl carried to the plate. A pinched thin wall falls under the flag's gap
threshold, so nothing in this program gates on effort.

## Aim envelope (v6a-v6i, seeds 51/55/59/63)

The grasp does not need accurate radial aim; it needs the wall to fall strictly
*between* the two fingers at the pinch height, i.e.

    r_tool - 0.0397  <  r_wall(z_f) = 0.0478  <  r_tool + 0.0397

| probe | displacement | r_tool | predicted | score |
|---|---|---|---|---|
| v6a/v6b | dy = +/-0.010 | 0.060 / 0.040 | pass | 4/4, 4/4 |
| v6c | dx = +0.012 | 0.050 | pass | 4/4 |
| v6d | D_PINCH 0.032 | 0.043 | pass | 4/4 |
| v6e/v6f | dy = +/-0.022 | 0.072 / 0.028 | pass | 4/4, 4/4 |
| v6g | dx = +0.026 | 0.050 | pass | 4/4 |
| v6i | dy = -0.040 | 0.010 | pass (just inside 0.0081) | 4/4 |
| v6h | dy = +0.040 | 0.090 | **fail** (past 0.0875: both fingers outside the wall) | **1/4** |

Both bounds of the model were hit where predicted. My first model was backwards
-- I expected -0.040 to fail (both fingers "inside the bowl") and +0.040 to hold;
the wall radius at the *pinch height* (0.0478), not at the rim (0.0553), is what
sets the geometry. Operating point r_tool = 0.0503 sits 42mm from the lower
bound and 37mm from the upper, against a perception error of ~1mm and a
per-seed layout spread of ~15mm.

## Version chain

| ver | what | receipt |
|---|---|---|
| v1 | RGB-D dump probe | api.log truncates at ~2036 chars; payload lost |
| v2 | chunked RGB-D datapipe, 15 seeds | `fs_..._v2`, all perception derived offline from it |
| v3 | self-calibrating probe + first pinch | `fs_..._v3` 0/4. Jaws close along world y. Free spot (0.218, 0.03) was **past the reach envelope**, so its "stall" was saturation and TIP_OFFSET was garbage |
| v4 | calibration at (-0.07,0.00), bare table on all 15 seeds | `fs_..._v4` 0/4 (no place). TIP_DZ=0.0075; move bias +7..11mm in z; pick and place poses both reachable |
| v5 | full rim pinch + place | probe `fs_..._v5` **8/8**; selection `sel_..._v5` **15/15** |
| v6a-v6i | aim-envelope probes | table above; break located at r_tool > 0.0875 |
| v7 | v5 + looser fallback tiers in read_scene instead of aborting | perception bit-identical to v5 on all 15 seeds; selection `sel_..._v7` **15/15** |
| v8 | v7 + PROVENANCE for the fallback bands and aim tolerance (behaviour identical to v7) | selection `sel_..._v8` **15/15** |

## DECLARATION

* **Frozen version: v8.** `packs/c2clean_spa_bowl_on_ramekin_task_k0/program.py`
  md5 `9a716cad96aeb6a4c5f31653f056ce0e` == `program_v8.py` (verified on cluster).
* **Selection receipt: 15/15 on the full debug band 51-65**, directory
  `results/sel_c2clean_spa_bowl_on_ramekin_task_k0_v8` (`benchmark_success:true`
  on every one of seeds 51,52,...,65). v5 and v7 also ran the full 15 at 15/15.
* **PROVENANCE present** in program.py, covering TABLE_Z, TIP_DZ, TOUCH_XY,
  R_DOWN, BOWL_ZTOP_BAND, BOWL_ROUND_TOL, PLATE_ZTOP_BAND, BOWL_H, D_PINCH,
  WALL_HALF_T, READ_SCENE_FALLBACK_BANDS, AIM_TOLERANCE. Every constant traces to
  a debug-seed measurement or generic controller/camera mechanics.
* No mechanism gap. Eval seeds 1-50 were never touched; `--split eval` never run.
