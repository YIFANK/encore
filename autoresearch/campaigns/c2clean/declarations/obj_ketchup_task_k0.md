# c2clean / obj_ketchup_task_k0 — "Pick the milk and place it in the basket"

Zero-demo cell (k0). Everything below was derived from debug seeds 51-65 only.

## v1 / v1b — sensor dump (no motion)

Hypothesis: with no pack, the only way to learn the scene is to get the raw
RGB-D off the box. Dumped `cam_high` + `cam_arm_wrist` RGB and depth as
zlib+base64 chunks through `api.log` and decoded them locally.

Evidence:
- v1 chunks of 3000 chars came back truncated — `api.log` caps a message at
  2000 characters. Re-ran as v1b with 1900-char chunks; decode clean.
- `cam_high` sits at base (0.897, 0.0, 0.650) looking along -x with a 32°
  depression, K = 618 px focal, 512x512. Table top is base z = 0.000.
- Scene (identical structure on all 8 dumped seeds): 6 props + 1 basket.
  Deprojected clusters (bbox midpoints):

  | prop | xmid | ymid | ztop | xext x yext | mean rgb |
  |---|---|---|---|---|---|
  | **milk carton** | -0.200 | -0.084 | 0.142 | 0.051 x 0.053 | 93,65,58 |
  | green bottle | -0.146 | 0.059 | 0.148 | 0.034 x 0.063 | 59,66,56 |
  | dressing bottle | -0.116 | -0.240 | 0.148 | 0.033 x 0.063 | 80,61,52 |
  | basket | 0.008 | 0.256 | 0.144 | 0.157 x 0.171 | 141,140,135 |
  | ketchup bottle | 0.055 | -0.106 | 0.113 | 0.028 x 0.049 | 59,27,8 |
  | soup can | 0.101 | -0.200 | 0.081 | 0.064 x 0.070 | 64,65,70 |
  | flat blue box | 0.150 | 0.027 | 0.020 | 0.080 x 0.042 | 78,86,106 |

- Target identification: cropping and upscaling the cam_high RGB shows the
  prop at (-0.200, -0.084) is a carton printed "Milk" with a cow — this is the
  intent object. No other prop is a carton.
- Verdict: the carton is the ONLY prop that is both tall (ztop > 0.12) and
  box-shaped (min bbox extent > 0.045); the two bottles that are taller are
  0.033 across. Gate = `ztop > 0.12 and min(xext,yext) > 0.045`, basket
  excluded as the largest cluster. Holds on all 8 dumped seeds.

## v2 — tip-offset probe + open-loop pick (0/4)

Hypothesis: the fingertip-to-eef offset is unknown, so press closed jaws into
bare table at (-0.15,-0.15) and read the stall.

Evidence: stall detection was wrong — every `api.move` leaves a steady
**+0.011 m** tracking residual in z, which tripped my 0.006 stall threshold on
the first step. The program grasped at eef z = 0.228 and closed on air
(`width_m = 0.001`). 0/4, 600 sim steps.

The probe trace is still the receipt I wanted: eef_z = cmd + 0.011 at every
commanded z from 0.12 down to 0.01, then cmd -0.010 gave eef_z 0.0091
(+0.019). So **fingertips reach the table at eef_z ~ 0.009** → TIP_OFF = 0.009,
MOVE_BIAS = 0.011.

Verdict: drop the probe (it costs ~250 sim steps) and hard-code both constants;
the carton is 130 mm tall so a ±20 mm height error still lands on its body.

## v3 — perceive → grasp → drop (8/8 probe)

Changes: no probe; `goto()` commands a pose then re-commands once with the
measured error to cancel the tracking bias; grasp fingertips at
`ztop - 0.045` (mid-upper carton body); close, lift to 0.30, carry to the
basket bbox midpoint, lower to eef z 0.215 (carton bottom then sits ~40 mm
below the basket rim) and open. One re-perceive-and-retry if the closed width
says air.

Evidence (ep51): closed `width_m = 0.0530, effort 3.0` — exactly the measured
carton extent, so the jaws are on the carton and not on air or a bottle; width
unchanged through the lift and the carry.

Receipt: **8/8** on 51,53,...,65, `results/fs_c2clean_obj_ketchup_task_k0_v3`,
224 sim steps/episode.

## v4 — tightened target gate

Hypothesis: v3's gate ranks candidates by *largest* min-extent, so on an unseen
layout where the carton fuses with a neighbour the fused blob would win.

Change (perception only, motion identical to v3): candidates must satisfy
`0.12 < ztop < 0.19`, `min ext > 0.042`, `max ext < 0.090`, and are ranked by
closeness of the min extent to the measured carton width 0.051. Verified
offline on the 8 dumped debug seeds: exactly one candidate each, same xy as v3.

Selection runs: see DECLARATION.

## DECLARATION

- **Frozen version: v4.** `packs/c2clean_obj_ketchup_task_k0/program.py`
  md5 `a29a56b1d34a693b9f64d7748c3a47f0` == `program_v4.py` (same md5).
- **Selection receipt: 15/15** on the full debug split 51..65 —
  `results/sel_c2clean_obj_ketchup_task_k0_v4` (15 of 15 `benchmark_success: true`).
- Receipt chain:
  - v1/v1b — sensor dump, no motion, 0/8 by construction
    (`results/fs_c2clean_obj_ketchup_task_k0_v1b`).
  - v2 — probe + open-loop pick, **0/4**, closed on air
    (`results/fs_c2clean_obj_ketchup_task_k0_v2`); yielded TIP_OFF and MOVE_BIAS.
  - v3 — perceive/grasp/drop, **8/8** probe subset
    (`results/fs_c2clean_obj_ketchup_task_k0_v3`), **15/15** full debug split
    (`results/sel_c2clean_obj_ketchup_task_k0_v3`).
  - v4 — v3 with a bounded, ranked target gate, **15/15** full debug split
    (`results/sel_c2clean_obj_ketchup_task_k0_v4`). Frozen (ties v3 on score,
    strictly narrower target gate).
- PROVENANCE present in program.py: 12 entries, every calibrated constant
  sourced to a debug-seed measurement (cam_high deprojection on seeds 51-65,
  the v2 table press) or to generic controller mechanics. No pack, no
  foreign-campaign constants.
- Archived: program_v1.py, program_v2.py, program_v3.py, program_v4.py in the
  pack dir.
