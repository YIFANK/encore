# c2clean / goal_put_bowl_top_cabinet_pos_k3 — working notes

Intent: "Put the bowl on top of the cabinet." Runner: `tools/fair_run.py` only.
No shared note file (FAIR_PROTOCOL v1.1.1). Everything below is derived from
`packs/c2clean_goal_put_bowl_top_cabinet_pos_k3/` plus debug seeds 51-65.

## What the pack says

Three demos, each ~90-110 steps, all with the same shape:

| demo | grasp keyframe (close) | release keyframe (open) | closed finger gap |
|---|---|---|---|
| 0 | t=31, ee (-0.099, 0.050, 0.9147) | t=81, ee (0.006, -0.177, 1.1489) | 0.0125 |
| 1 | t=33, ee (-0.105, 0.034, 0.9139) | t=98, ee (0.049, -0.155, 1.1748) | 0.0059 |
| 2 | t=32, ee (-0.085, 0.018, 0.9162) | t=92, ee (-0.038, -0.208, 1.1473) | 0.0125 |

`ee_path6` roll ≈ π and pitch/yaw within ±0.25 rad of zero at every keyframe,
so the demos never leave the default straight-down wrist. The demos' cabinet
sits at −y; on debug seeds 51-65 it sits at +y, so the place pose has to be
perceived, not copied.

## Measured on the debug seeds (probe v0/v1)

* Table plane **z = 0.9010** (cam_high depth, all four probe seeds).
* **Fingertips ride 8 mm below `api.eef()`** — v1 pressed the open gripper onto
  a bare table patch; it stalls at eef z = 0.9090 = table + 0.0080.
* Free-air gripper: open 0.0797 m, fully closed 0.0010 m.
* Cabinet top: the only flat plateau above table+0.15, **z = 1.1276**,
  footprint x[−0.53, −0.27] × y[0.135, 0.325], ≈1600 px, centre (−0.40, 0.23).
* Bowl: circular vessel, rim top 0.9510 (table + 0.049), outer diameter
  0.109 m. It is the only prop whose rim clears table+0.035 with a circular
  footprint (kettle 0.081×0.026 aspect 3.1, bottle 0.027×0.043).
* `_pos` perturbation over seeds 51-65 is small: bowl x ∈ [0.037, 0.064],
  y ∈ [−0.036, −0.006]; the cabinet does not move.
* EGL drops textures at runtime, so runtime colours do **not** match the pack
  keyframes (pack cabinet renders near-black, runtime grey). No colour cue is
  used anywhere.

## The mechanism (hypothesis → evidence)

**Hypothesis.** The demo close at table+0.014 is not a rim pinch — with the
fingertips 8 mm below the eef that puts them ~6 mm above the table, 3.5 cm
*below* the rim. It is a deep wall straddle: one finger inside the bowl, one
outside, closing on the wall near the base. Evidence: the pack's closed gaps
(0.0125/0.0059/0.0125 m) are wall thicknesses; a rim-top pinch cannot close
that far, and a centre grasp closes on nothing.

**Where.** Back-projecting the bowl's pixel in each demo's t=0 keyframe onto
the rim plane (z = 0.950) gives bowl centres (−0.078, 0.000), (−0.087, −0.019),
(−0.078, −0.013); against the close points that is an offset of (−0.021,
+0.050), (−0.017, +0.053), (−0.007, +0.031) — dominated by **+y ≈ one rim
radius**. With the straight-down wrist the fingers separate along base ±y, so
the offset has to be radial along y for the wall to land between them. Both
facts agree: grasp at `bowl_centre + rim_r` in y.

**Release.** The demos open 0.020–0.047 m above the (measured) cabinet-top
plane; 0.025 chosen.

## Version chain

| ver | what | probe | result |
|---|---|---|---|
| v0 | perception probe only (RGB-D shipped out through `api.log` in base64/zlib chunks) | 51,53,57,61 | scene + geometry recovered; no motion |
| v1 | calibration probe: press open gripper on bare table, free-air open/close widths | 51,53 | fingertip offset = eef − 0.0080; open 0.0797 / closed 0.0010 |
| v2 | first end-to-end: perceive bowl + cabinet, straddle-grasp at `+rim_r` in y, carry, release at ctop+0.025 | 51,53,55,57 then 52,54,56,58-65 | **15/15** on the whole debug band, 157-165 sim steps/episode |
| v3 | v2 + sensor-verified retries (finger gap + re-perception of the bowl's old footprint after the lift), grasp side flips to −y when +y is occupied, cabinet plateau search hardened, self-imposed step budget | 51,53,55,57,59,61,63,65 | see below |

v2 already clears the debug band; v3 exists because the eval band is 50 unseen
seeds and a full attempt costs only ~160 of LIBERO's ~1000-step horizon, so
two spare attempts are free insurance against a grasp that misses.

## Aim-envelope probes (single attempt, seeds 51/54/58/62)

Success on the frozen program says nothing about margin, so each calibrated
constant was displaced until it broke.

| displacement | result |
|---|---|
| grasp offset rim_r − 0.030 | 4/4 |
| grasp offset rim_r − 0.015 | 4/4 |
| grasp offset rim_r + 0.015 | 4/4 |
| grasp offset rim_r + 0.030 | **3/4** (ep62: inner finger lands outside the bowl, close catches nothing) |
| grasp z, table + 0.0037 (−10 mm) | 4/4 |
| grasp z, table + 0.0237 (+10 mm) | 4/4 |
| release clearance 0.005 | 4/4 |
| release clearance 0.055 | 4/4 |

Only the lateral grasp offset has a reachable edge, and only on the outward
side. v4 therefore trims the offset 8 mm inward; re-running the +0.030
displacement against v4 (net +0.022 from rim_r) gives **4/4** where v3 gave
3/4 — the trim bought the predicted margin rather than just moving the aim.
Heights are not marginal at all: the release tolerates a 50 mm spread, which
is why the pack's own 0.020–0.047 spread across three demos was harmless.

| ver | what changed | probe | result |
|---|---|---|---|
| v4 | grasp offset trimmed 8 mm inward to centre it in the measured envelope | 51,53,55,57,59,61,63,65 | 8/8, every one on attempt 0 |

## DECLARATION

* **Frozen version: v4.** `packs/c2clean_goal_put_bowl_top_cabinet_pos_k3/program.py`
  md5 `ce3b97c25e11250b7e8520c921727c79` == `program_v4.py` (verified on the
  cluster).
* **Selection receipt: 15/15** on the full debug band (seeds 51-65), one formal
  run, dir `results/sel_c2clean_goal_put_bowl_top_cabinet_pos_k3_v4`. Every
  episode succeeded on retry attempt 0, 155-164 sim steps each.
* **Per-version receipt chain**
  * v0 — perception probe (51,53,57,61): no motion; recovered table plane,
    camera convention, bowl and cabinet geometry.
  * v1 — calibration probe (51,53): fingertip offset = eef − 0.0080 m;
    gripper open 0.0797 / closed 0.0010.
  * v2 — first end-to-end: **15/15** (4 seeds in `..._v2`, the remaining 11 in
    `..._v2b`).
  * v3 — v2 + sensor-verified retries and grasp-side flip: **8/8**
    (51,53,55,57,59,61,63,65), all on attempt 0.
  * v4 — v3 with the grasp offset centred in its envelope: **8/8** probe,
    **15/15** formal selection. **Frozen.**
  * Envelope probes `program_env_*.py` (diagnostics, not selection candidates):
    results above.
* **PROVENANCE present** — 11 entries (ASPECT_MAX, CAB_BAND, CARRY_CLEAR,
  FINGER_OFFSET, GRASP_CLEAR, HOLD_MIN_GAP, OFFSET_TRIM, OPEN_W, PLACE_CLEAR,
  RIM_BAND, STEP_BUDGET), every one sourced to a pack field or a debug-seed
  measurement. `tools/fair_run.scan_program(..., "eval")` returns clean, so the
  eval gate accepts the frozen file.
* No mechanism gap: the cell is solved, not worked around.
