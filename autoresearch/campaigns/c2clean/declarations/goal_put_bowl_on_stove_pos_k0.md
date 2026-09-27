# c2clean / goal_put_bowl_on_stove_pos_k0 — worker notes

Intent: "put the bowl on the stove". No demo pack (k0). Debug band = seeds 51-65.
Runner: `tools/fair_run.py` only.

## Runtime facts measured here (not assumed)

- **Episode horizon = 1000 sim steps.** probe_v2 froze mid-program at exactly
  `sim_steps: 1000`; every later move returned the same eef with a large
  residual. `move_cartesian` burns `max(40, 120*seconds)` steps when it cannot
  converge (POS_TOL 0.012), so contact moves must use small `seconds`.
- **Fingertip offset ≈ 9 mm below the eef.** probe_v2 descent stalled at eef
  z = 0.9096 over a table measured at z = 0.9012.
- **Reach limit in +x**: at table height the eef stalled at x ≈ 0.174 when
  commanded to x = 0.212. Everything this task needs is at x < 0.20.
- **Gripper "holding" is `grip_cmd==close AND finger gap > 0.005`** — so the
  flag reads a *fake* 3.0 if the episode terminated before the close actually
  stepped (seen in probe_v2). Real receipt: gap after a lift.

## Scene (re-derived from cam_high RGB-D, seeds 51/53/54/55)

Top-down 4 mm height map from the deprojected depth, then height-band
connected components:

| thing | band | signature |
|---|---|---|
| table | — | z = 0.9012 |
| stove slab | 0.915–0.937 | ~0.19 m square, uniform grey (rgb ≈ 87), top 0.931 |
| plate | 0.915–0.937 | ~0.13 m, light (rgb ≈ 135), top 0.920 — flat, below the slab top |
| bowl | > 0.937 | ~0.112 m round, rgb ≈ 123, rim z 0.950 |
| black pan | > 0.937 | ~0.09 × 0.02 strip, rgb ≈ 28 |
| cabinet | > 1.05 | x[-0.09, 0.16], y[-0.34, -0.13], top 1.128 — tall, on the bowl's −y side |

The `_pos` perturbation slides the whole tabletop set; on seeds 53/55 the bowl
abuts the stove and the two fuse in a single band, so the **0.937 cut (above
the slab top, below the bowl rim) is what separates them**.

Selection rules (both instance-agnostic, no fixed coordinates):
- bowl = round (aspect > 0.6), 0.07–0.17 m, light (mean rgb > 70) blob above 0.937
- stove = largest ≥ 0.14 m dark (mean rgb < 130) slab in 0.915–0.937

## Grasp mechanism

The bowl is 0.112 m across and the jaws open to 0.078, so the body is not
graspable; the rim is. At the home wrist the jaws close along **world y**
(confirmed: pinching the +y rim point closes to a gap of 0.0073 with
effort 3.0 — a jaw axis along x would have shoved the bowl and closed to ~0).
The +y rim is used rather than −y because the cabinet stands at y < -0.13 right
beside the bowl's −y side.

Descent to `rim_z - 0.020` is blocked ~10 mm high (residual ≈ 0.012) — the jaws
wedge on the flaring wall, which is what makes the pinch hold. The bowl's base
then hangs `close_z - table` below the hand; that measured hang sets the
release height over the slab.

## Version chain

| ver | change | receipt |
|---|---|---|
| probe_v1 | perception only, RGB-D shipped out through `api.log` | scene decoded offline; 72 sim steps |
| probe_v2 | fingertip/reach calibration + first pinch | horizon exhausted by 7 blocked calibration moves; gave tip offset, reach limit, and the 1000-step budget |
| v3 | perceive → pinch +y rim → carry → place | probe 4/4 (51,53,55,57); **formal 14/15** `results/sel_c2clean_goal_put_bowl_on_stove_pos_k0_v3` |
| v4 | staged lift + gripper-verified retry (3 rim aims) + release height from the measured hang | formal run: see DECLARATION |

### v3 → v4 (hypothesis → evidence → verdict)

- Hypothesis: seed 54's failure is a lift-time slip, not a perception or aim
  error. Evidence: `CLOSE gap=0.0073 effort=3.0` (identical to the seeds that
  worked) then `LIFT gap=0.0019 effort=0.05`, and the bowl was re-perceived
  afterwards still at its start position (0.030, -0.011).
- Verdict (partly wrong — see v4 → v5): retrying is right and affordable (one
  attempt costs ~210 of the 1000 steps), and the hypothesis about seed 54 held
  up. But v4 gated the retry on the harness's `effort` flag, which is not a
  slip detector, and that cost two seeds.

### v4 → v5 (hypothesis → evidence → verdict)

v4 scored **13/15** — a regression. Diagnosis from the GIF of v3 seed 53
(`ep53_ok.gif`): v3 carried and placed the bowl on seeds where the gripper
reported `effort 0.05`, i.e. **the harness's "holding" flag is wrong for this
grasp**. It is defined as `finger gap > 0.005`, and a thin bite on the rim
reads gap 0.0047. v4 treated those good grasps as failures and re-tried,
destroying them (seeds 55, 58).

- Hypothesis: hold vs slip is distinguished by whether the gap *survives the
  lift*, not by its absolute value.
- Evidence, 11 grasps across the v3/v4 logs (close gap → post-lift gap):

  | outcome | readings | ratio |
  |---|---|---|
  | held | 0.0047→0.0047, 0.0048→0.0047, 0.0049→0.0049, 0.0068→0.0066, 0.0075→0.0073, 0.0084→0.0069 | 0.82–1.00 |
  | slipped | 0.0073→0.0019, 0.0073→0.0034, 0.0074→0.0036, 0.0075→0.0046 | 0.26–0.61 |

  That is the mechanism: the jaws hold the rim wall's thickness while the bowl
  is in them and collapse toward zero once it is gone.
- Verdict: gate the retry on `gap_after_lift > 0.70 * gap_at_close` (margin
  0.61 → 0.82) and add a free second opinion — once the hand is over the slab,
  re-perceive and retry if a bowl is still sitting at its original seat.

## Envelope

The `_pos` perturbation is small and translation-only: across seeds 51-65 the
bowl centre spans x [0.031, 0.059], y [-0.038, -0.017] and the slab x
[0.091, 0.107], y [0.119, 0.138]; the table (0.9012), slab top (0.931) and rim
(0.950) are identical on every seed. Every aim is computed per-episode from
that episode's own depth, so the margin that matters is tolerance to aim error,
not to scene displacement. The logs bracket it: holds were obtained at
effective grasp radii from 0.043 (seed 53, a blob measured at rim_r 0.049 with
the 0.006 inset) to 0.056, and with a 0.020 tangential shift around the rim —
a radial envelope roughly 13 mm wide, which the three-aim ladder spans.

## DECLARATION

- **Frozen version: v5.** `packs/c2clean_goal_put_bowl_on_stove_pos_k0/program.py`
  md5 `3f8c0c8a40bed19367a3b60c49eeab43` == `program_v5.py` (same md5, verified
  on the cluster).
- **Selection receipt: 15/15 on the full 15-seed debug band**, dir
  `results/sel_c2clean_goal_put_bowl_on_stove_pos_k0_v5` (seeds 51-65, all
  `"benchmark_success": true`). 14 seeds placed on the first grasp attempt,
  seed 54 on the second. Peak cost 309 of the 1000 available sim steps.
- **Receipt chain:**
  | version | run dir | seeds | result |
  |---|---|---|---|
  | probe_v1 | `fs_..._p1` | 51,53,55 | perception only (no motion) |
  | probe_v2 | `fs_..._p2` | 51,53 | calibration; horizon exhausted |
  | v3 | `fs_..._v3` | 51,53,55,57 | 4/4 |
  | v3 | `sel_..._v3` | 51-65 | **14/15** (seed 54: lift slip) |
  | v4 | `sel_..._v4` | 51-65 | **13/15** (bad hold detector) |
  | v5 | `sel_..._v5` | 51-65 | **15/15** |
- **PROVENANCE present** and the frozen program passes `fair_run.scan_program(...,
  "eval")`: keys RES, WORKSPACE, TABLE_Z, SLAB_BAND, RIM_BAND, GRASP_DEPTH,
  HOLD_RATIO, GAP_FLOOR, MOVED_MIN, ATTEMPT_AIMS, PLACE_CLEAR — every one
  sourced to a debug-seed measurement or to generic controller/camera
  mechanics. No demonstration pack was used (k0); no `api.done` read.
