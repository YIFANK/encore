# c2clean / spa_bowl_on_ramekin_pos_k3

Task: "pick up the black bowl on the ramekin and place it on the plate".
Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_spa_bowl_on_ramekin_pos_k3/`
(K=3 demos, keyframes + ee_path/ee_path6 + actions).

## Scene, as re-derived on the debug seeds

`cam_high` is an agentview at base (0.659, 0, 1.610) looking along -x and 39 deg
down; image-right is +y, image-down is +x. Table top 0.9012.

Three small props plus a cabinet (y < -0.14, top 1.125) and the arm:

| prop | x | y | top | mean RGB |
|---|---|---|---|---|
| bowl on the ramekin (target) | ~-0.20 | ~0.21 | 1.0006 | ~121 |
| bowl on the table (twin) | ~0.06 | ~0.20 | 0.9519 | ~136 |
| plate (goal) | ~0.075 | ~0.02 | 0.9388 | ~158 |

The plate separates from the two bowls by luminance (158 vs 120-136) with a
wide margin; the two bowls separate by height (1.0006 vs 0.9519) — the target
is the one standing on the ramekin, 5 cm higher. The bowl asset is ~51 mm tall
with a rim radius ~0.054.

**The pack's demo xy is a decoy.** In the demos the plate sits near y ~ 0.17;
on the debug seeds it sits at y ~ 0.02 and the *twin bowl* occupies y ~ 0.20.
`_pos` permutes the props, so both target and goal must be perceived. The pack
is still load-bearing for the *mechanism*: its `gripper_state` shows the grasp
closing to a 7 mm finger gap, i.e. a straddle of the bowl wall, not a grasp of
the bowl body (the bowl's inner diameter, 0.09, exceeds the 0.078 jaw stroke,
so no body grasp exists).

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | dump a coarse top-down height/colour map to read the scene offline | table 0.9012; 3 props + cabinet + arm | perception baseline |
| v1 | characterise every prop (z histograms, ring extents) over all 15 debug seeds | layouts stable to 1-2 cm; target top 1.000 on every seed | ok |
| v2 | log a cam_high RGB thumbnail — the pack keyframes are a *different* render and had misled the identity call | plate is the bright prop at y~0.02, not the one at y~0.2 | identity fixed |
| v3 | straddle the wall at eef = bowl centre + rim radius, depth = top-0.012 | **1/4** (51,55,57 fail). Closed gap 0.0027-0.0034; the one success held at 0.0027 | marginal, gap is not a usable receipt |
| v4/v4b | calibration: press a closed gripper onto the table | first spot was over the cabinet (blocked at 1.069); redone at (-0.05,0.12): **fingertips sit 9 mm below `api.eef()`** | calibration |
| v5 | aim the jaws at the wall midline (rim radius - 3.5 mm), verify by re-perceiving the stand, retry on the other side | **0/8**, finger gap 0.00108 on all 6 attempts, bowl never moved: jaws closed in free air | refuted — and the retry/verify machinery proved sound (it detected every failure) |
| v6/v6b | fine 2.5 mm height maps + a full-res RGB crop of the target | the bowl is **tilted** on the ramekin: rim reads 1.0006 at its far-x edge and 0.9726 at the near-x edge, a 26 mm fall. v5 put the fingertips 3.7 mm *above* the local rim | root cause |
| v7 | measure the rim height and the wall midline **in the straddle column**, not from the bowl's global top | **8/8** on 51..65 odd; closed gap 0.0066 with effort 3.0, matching the pack's 0.0072; no retries used | selected candidate |

## Why v5 failed and v7 works

The bowl does not sit flat on the ramekin — it is tipped ~16 deg toward +x. The
component's global top (1.0006) is reached only at the far-x edge of the rim,
so a grasp depth referenced to it is ~12 mm too high at the straddle point,
which is at x = the bowl centre. v7 takes, in the straddle column:

* `ltop` = the highest reading within 13 mm of the intended straddle y,
* `y_rim` = the outermost y still within 6 mm of `ltop`,
* `y_in`  = walking inward, the first y whose surface has dropped 10 mm,

and puts the eef on `(y_rim + y_in)/2` with the fingertips 10 mm below `ltop`.
That wall measures 7.5 mm thick — which is the pack's closed gap, an
independent confirmation that this is the feature the demo grasps.

The harness `move` only guarantees a 12 mm ball, which is larger than the wall,
so every approach is re-commanded against the measured eef offset until it is
within 3 mm (`go()`), bounded to a 35 mm total correction so a blocked move
cannot run the command away.

Verification is by re-perception, never by the gripper gap: a successful hold
read 0.0027 in v3 and a failed one read 0.0034. After the lift the arm stages
between target and plate and re-captures; the bowl counts as held only if the
height at the target site has dropped by more than 25 mm (ramekin top 0.944 vs
bowl top 1.0006). On failure the program reopens, re-perceives and retries on
the opposite y side, up to 3 attempts.

## Receipts

* v3 probe `results/fs_c2clean_spa_bowl_on_ramekin_pos_k3_v3` — 1/4 (51,53,55,57)
* v5 probe `results/fs_c2clean_spa_bowl_on_ramekin_pos_k3_v5` — 0/8 (51..65 odd)
* v7 probe `results/fs_c2clean_spa_bowl_on_ramekin_pos_k3_v7` — 8/8 (51..65 odd)
* v7 selection `results/sel_c2clean_spa_bowl_on_ramekin_pos_k3_v7` — see DECLARATION

## DECLARATION

* **Frozen version: v7.** `packs/c2clean_spa_bowl_on_ramekin_pos_k3/program.py`
  md5 `d2c520817fe8593912deab0ac26b9e05` == `program_v7.py` (same md5).
* **Selection receipt: 15/15** on the full 15 debug seeds 51-65,
  `results/sel_c2clean_spa_bowl_on_ramekin_pos_k3_v7`.
* Margin: every one of the 15 grasped on attempt 0 — the retry path was never
  needed. Closed finger gap 0.00645-0.00743 with effort 3.0 on all 15 (the
  pack's demo holds at 0.0072). Max 167 sim steps per episode.
* Per-version receipt chain: v3 1/4, v5 0/8, v7 8/8 probe (51..65 odd), v7 15/15
  formal. Archived as `program_v0..v7.py` in the pack dir.
* PROVENANCE: present in `program.py` as a top-level literal dict covering
  GRID_BOX, ZCAP, N_MIN/N_MAX, FINE_C/FINE_M, BAND, GRASP_TIP_DEPTH,
  TIP_BELOW_EEF, BOWL_H, DROP_CLEAR, REFINE_TOL. Every constant is sourced to
  this pack's contents, a debug-seed measurement, or generic controller/camera
  mechanics. No `api.done` read anywhere in the program.
