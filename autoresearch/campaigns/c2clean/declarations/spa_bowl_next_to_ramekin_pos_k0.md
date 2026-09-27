# c2clean / spa_bowl_next_to_ramekin_pos_k0 — working notes

Intent: "pick up the black bowl next to the ramekin and place it on the plate"
No demonstration pack (k0). Everything below is derived from debug seeds 51-65
under `tools/fair_run.py --split debug`.

## Scene, as measured (debug seeds 51-58, cam_high RGB-D)

Workspace plane (modal deprojected z): **TABLE_Z = 0.9027**.
Props, by height above that plane and top-down footprint:

| prop | top | footprint | notes |
|---|---|---|---|
| bowl (x2) | +0.0495 | 0.110 x 0.110 | curved wall, interior floor at +0.005 |
| ramekin | +0.0411 | 0.085 x 0.085 | straight-walled cup, bright uniform grey (rgb ~0.58) |
| plate | +0.0174 | 0.135 x 0.135 | flat disc, bright |
| cookie box | +0.018 | 0.080 x 0.060 | brown, rgb ~[0.38,0.27,0.19] |

The `_pos` perturbation moves the second bowl a lot (cy -0.06 .. -0.17,
cx 0.12 .. 0.21) and the ramekin/target-bowl pair and plate only a little.
Target bowl = the bowl nearest the ramekin.

Radial height profile of a bowl (seed 51/52/56): interior floor +0.005 for
r<0.025, wall rising through +0.030 at r~0.040, rim top +0.0495 over
r 0.045-0.058, table beyond r 0.060. So at 20 mm below the rim the wall spans
r ~ 0.040-0.050 — the straddle midline is r ~ 0.046.

## Version log

### v1 — perception dump (no motion)
Hypothesis: need the scene before anything else.
Evidence: `api.log` truncates messages at 2000 chars, so a whole frame will not
fit in one line. 4/4 episodes ran, 0 success (by design).
Verdict: superseded by v2.

### v2 — chunked RGB-D dump, seeds 51-58
Evidence: zlib+base64 in 1800-char chunks round-trips a 256x256 RGB + uint16
depth frame. Gave the table plane, the four prop classes and their heights.
Verdict: perception designed offline from this; classifier hits 8/8 on
target bowl, ramekin, plate and the distractor bowl.

### v3 — calibration probe (no grasp), seeds 51,53,55,57
Hypothesis: the fingertip-to-EEF offset and the reach at the bowl are unknown.
Evidence: pressing the OPEN gripper into bare table stalls at
eef z = TABLE_Z + 0.0058 on all four seeds -> **TIP_OFF = 0.0058**.
Hover over the intended grasp pose converged (res 0.011 < POS_TOL 0.012), but
the eef lands 5-9 mm short of the command in x/y — a systematic tracking bias.
Verdict: fold one bias-cancelling re-issue into every move.

### v4 — first full attempt, seeds 51,53,55,57 — 0/4
Hypothesis: straddle the rim on the +y side, lift, carry to the plate centre.
Evidence: the grasp itself is sound — closed gap 0.0086-0.0094 on every seed
(= the measured wall thickness) with effort 3.0, and the bowl left the table.
The placement was wrong twice over:
 1. the bowl centre trails the eef by the full grasp radius (RG) in -y, so
    putting the *eef* at the plate centre puts the *bowl* 46 mm off it —
    seed 55's final bowl sat at (0.110, 0.007) against a plate at (0.068,0.045);
 2. the place height was computed from the fingertip rather than the bowl base,
    9.5 mm too low.
Verdict: fix both; the grasp is not the problem.

### v5 — place the BOWL, not the eef — 4/4 probe
Changes: place eef xy = (plate_cx, plate_cy + RG); place eef z =
plate_top + 0.004 + (BOWL_H - GRASP_DEPTH) + TIP_OFF; retreat toward home
before the final capture.
Evidence (seeds 51,53,55,57): benchmark_success True on all four. The episode
terminates on the predicate, which is visible only as the post-place moves
returning a frozen eef.
Verdict: candidate for selection.

## Selection receipt

Formal selection run, all 15 debug seeds (51-65), one run, no re-runs:

```
results/sel_c2clean_spa_bowl_next_to_ramekin_pos_k0_v5
benchmark_success: 51 T 52 T 53 T 54 T 55 T 56 T 57 T 58 T 59 T 60 T 61 T 62 T 63 T 64 T 65 T
```
**15/15.**

Receipt chain:
| version | seeds | result | dir |
|---|---|---|---|
| v1 | 51,53,55,57 | dump only (log truncation found) | fs_..._v1 |
| v2 | 51-58 | dump only (scene measured) | fs_..._v2 |
| v3 | 51,53,55,57 | calibration only (TIP_OFF 0.0058, tracking bias 5-9 mm) | fs_..._v3 |
| v4 | 51,53,55,57 | 0/4 (placed the eef, not the bowl; place 9.5 mm low) | fs_..._v4 |
| v5 | 51,53,55,57 | 4/4 | fs_..._v5 |
| v5 | 51-65 | **15/15** | sel_..._v5 |

## DECLARATION

- Frozen version: **v5**.
  `packs/c2clean_spa_bowl_next_to_ramekin_pos_k0/program.py`
  md5 `2a308e34973debad151be71d177f25ba` == `program_v5.py`
  md5 `2a308e34973debad151be71d177f25ba`.
- Selection receipt: **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_spa_bowl_next_to_ramekin_pos_k0_v5`.
- Archived versions: `program_v1.py` .. `program_v5.py` in the pack dir.
- `PROVENANCE` present in program.py; every calibrated constant
  (TABLE_Z, BAND_CUTS, CELL, TIP_OFF, BOWL_H, RG, GRASP_DEPTH, PLACE_CLEAR)
  is sourced to a debug-seed measurement or to generic controller/camera
  mechanics. No demonstration pack was available or used; no LIBERO-specific
  prior knowledge entered any constant.
- Mechanism, in one line: segment the cam_high height map into three measured
  height classes (bowl rim +0.0495, ramekin +0.0411, plate +0.0174), take the
  bowl nearest the ramekin, straddle its rim wall 20 mm below the lip at
  r = 0.046 on the side away from the ramekin, and place with the BOWL centre —
  which trails the eef by that same radius — over the plate centre.
