# c2clean / obj_bbq_sauce_task_k3 — "Pick the ketchup and place it in the basket"

Runner: `tools/fair_run.py` only, split debug (seeds 51-65). No shared note file.

## What the two packs say

| pack | language | close-command ee (last close before the carry) | release ee |
|---|---|---|---|
| `..._task_k3` (target half) | "pick up the bbq sauce and place it in the basket" | d0 t54 (0.027,-0.130,0.073); d1 t96 (0.046,-0.112,0.074); d2 t90 (0.056,-0.115,0.104) | y ~ +0.24..0.26, z 0.17-0.25 |
| `..._task_mate` (object half) | "pick up the ketchup and place it in the basket" | d1 t49 (-0.114,-0.251,0.113); d2 t42 (-0.123,-0.253,0.113) | y ~ +0.24, z 0.17-0.21 |

Keyframes are emitted at gripper-command sign flips, so the keyframe ee at a
`cmd +1` flip IS the grasp pose and the width at the following `cmd -1` flip is
the held thickness: 0.017/0.017 per finger on the mate (ketchup) demos.

Both packs place over a basket at y ~ +0.24 — the basket is the common target.

## Identity: which bottle is the ketchup

Projected each pack demo's grasp ee into its own 128px keyframe using the
camera matrices read at runtime on debug seeds (K fx=fy=618.04, c=(256,256) at
512px; t_base_cam = [[0,.529,-.849,.897],[1,0,0,0],[0,-.849,-.529,.65]]),
scaled by 128/512. The crops show:

* mate (ketchup): red-orange bottle under a **grey/silver screw cap**.
* k3 (bbq sauce): plain amber bottle with a **dark brown cap**.

Debug-seed depth on all 15 seeds segments the same six props. Per-blob top-band
(top 12 mm) mean colour and chroma (max-min channel):

| prop | top z | footprint | cap RGB | chroma |
|---|---|---|---|---|
| basket | 0.143 | 0.16 x 0.17 | 167,167,167 | 1 |
| lying green bottle | 0.148 | 0.28 x 0.09 | 22,54,35 | 32 |
| **silver-cap bottle (ketchup)** | **0.148** | **0.03 x 0.06** | **97,96,95** | **2** |
| amber bottle (bbq sauce) | 0.113 | 0.03 x 0.05 | 72,23,5 | 67 |
| can | 0.081 | 0.06 x 0.07 | 81,83,92 | 11 |
| flat blue box | 0.020 | 0.08 x 0.04 | 62,68,85 | 23 |

Rule used: keep blobs with top z in [0.12,0.22], footprint <= 0.12 m, >= 100 px
(drops basket, can, box, the lying green bottle and the amber bbq sauce), then
take the minimum cap chroma. On seeds 51-65 that leaves exactly ONE candidate
every time, and it is the silver-cap bottle.

## Grasp geometry

The bottle is flat, not round: y width 0.063 at the base tapering to 0.033 on
the neck; the only band whose x extent (0.026) matches its y extent (0.030) is
the top 12 mm — the cap disc seen from above — so the cap-disc bbox centre is
the only unbiased xy estimate (the body's visible surface is the camera-facing
half and its centroid sits ~15 mm short in x).

Grasp height comes straight from the mate pack: close at ee z 0.113 with the
bottle top at 0.148 → **0.035 m below the perceived top**, i.e. the neck, where
the 0.033 m thickness matches the 2 x 0.017 held width in the pack.

## Version log

| ver | hypothesis | run | result |
|---|---|---|---|
| v0 | probe: dump cam_high RGB-D + K/T (chunked through api.log, 1800 chars/line — api.log truncates at 2000) | `fs_..._v0b` 15 eps | scene mapped; six props, only basket (±0.015) and the amber bottle (±0.01) jitter across seeds |
| v1 | silver-cap pick + cap-disc xy + top-0.035 neck grasp + carry 0.28 → basket rim bbox centre → release 0.20 | `fs_..._v1` seeds 51,53,...,65 | **8/8** |
| v1 | same, formal selection | `sel_..._v1` all 15 | **15/15** |

Receipts inside v1 (every episode): `CLOSED width 0.03344 effort 3.0` — the
closed gap equals the measured neck thickness and effort 3.0 (holding) survives
LIFT, OVER and DOWN, so the carry is verified by my own sensors, not by success.

## Aim envelope (not part of selection)

`program_m_{xp,xm,yp,ym}.py` = v1 with the grasp xy displaced ±0.012 m, seeds
51,55,60,65: **4/4 each** (`fs_..._m{xp,xm,yp,ym}`). So the grasp tolerates at
least ±12 mm in both axes — consistent with a 33 mm neck inside a 78 mm jaw.

## DECLARATION

* Frozen version: **v1**. `packs/c2clean_obj_bbq_sauce_task_k3/program.py`
  md5 `462843e260ddf5a587ab5de755940310` == `program_v1.py` (same md5, local and
  on AbakaAI).
* Selection receipt: **15/15** on the full debug band (seeds 51-65),
  `results/sel_c2clean_obj_bbq_sauce_task_k3_v1`.
* Probe chain: v0 `fs_..._v0b` (perception only), v1 `fs_..._v1` 8/8,
  margin probes 4/4 x 4.
* PROVENANCE dict present in program.py covering GRASP_BELOW_TOP, R_DOWN,
  CAP_BAND_M, TOP_MIN/TOP_MAX, EXT_MAX, Z_FLOOR/WORKSPACE, CARRY_Z, RELEASE_Z,
  GRIP_OPEN_M/GRIP_CLOSE_M — sourced only to the two named packs, debug-seed
  observations and generic camera/controller mechanics.
* No `api.done` read; no forbidden reads; writes confined to the pack dir and
  `results/*c2clean_obj_bbq_sauce_task_k3*`.
