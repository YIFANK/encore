# c2clean — goal_put_bowl_on_stove_pos_k3

Intent: **put the bowl on the stove**. Runner: `tools/fair_run.py` only.
Pack: `packs/c2clean_goal_put_bowl_on_stove_pos_k3/` (K=3 demos + 12 keyframe PNGs).

## What the pack says

Three demos, all the same shape: descend from home to EEF z 0.920–0.927 with
the gripper commanded closed, settle to a small finger gap (joint gap
0.006–0.012), carry to a second site and release at EEF z 0.945–1.023. Grasp
EEF xy ≈ (−0.11, +0.05), release EEF xy ≈ (−0.26, +0.26). Keyframe RGB shows a
kitchen table: cabinet, wine bottle, a shallow dish, a deep bowl, a slab stove
with a burner ring, a small dark prop.

The demo **xy is a decoy**: on every debug seed the eval layout is different
(bowl at x ≈ +0.04, stove at x ≈ +0.10), so the target had to be perceived. The
demos still carried the *mechanism*: grasp height, finger gap, and the fact
that the release is a simple open over the destination.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | which deprojection convention / where is the table? | vectorised pinhole reproduced `frame.deproject` to 1e-3 m on 5 probe pixels; z-histogram mode = **0.901** | convention + table height fixed |
| v2 | a 12 mm height gate segments the props | the gate fuses plate+bowl into one 0.27 m blob and swallows a dark prop into the slab | too coarse |
| v3 | a gate ladder separates them | at **0.032 m** the bowl stands alone (~0.11 m square footprint, top 0.95–0.97) on all 8 probe seeds; the slab top is 0.930; plate top < 0.923 | segmentation rule found |
| v4 | where are the fingertips relative to the EEF? | a bare-table descent stalls at EEF z **0.909**, table 0.901 → **tip offset 0.008 m** | calibrated |
| v5 | which round prop is "the bowl"? | zlib+base64 RGB dump: speckled deep bowl (yellow rim) vs a red-rimmed shallow plate; slab shows concentric burner rings | target identified |
| v6 | straddle the bowl wall from −y, place on the slab | arm never descended (cmd z 0.9235, achieved 1.106), 1000 sim steps burnt | failed |
| v7 | is the step horizon the problem? | 31 moves cost **291** steps; a converged 1.0 s move covers 0.07 m in ~9 steps; a starved move burns ~60 | not a budget problem |
| v8 | is the bowl's xy out of reach? | z ladder from −y: arm frozen at (0.087, −0.075, 1.105) from the first hover, residual grows monotonically; the GIF shows the arm folded over the table | **−y approach jams on the cabinet** (top z 1.20, y < −0.12) |
| v9 | approach from +y instead | ladder tracks cleanly to EEF 0.9399 at the bowl and 0.9395 over the stove, residual ~0.010 | side settled |
| v10 | which straddle offset grips? | offset 0.000 → stalls on the rim, closes to 0.001 m at effort 0.05 (air); offset **0.030 m** → EEF 0.939, gap 0.0071, **effort 3.0 that survives a lift** | grasp found |
| v11 | full pick-and-place | **6/8** (51,53,55,61,63,65 ok; 57,59 fail) | two bugs |
| v12 | fix both | **8/8** on the probe subset | selected |

### v11 → v12 fixes (both diagnosed from my own logs)

1. **Stove centre.** The 0.018 m slab gate fuses the bowl's skirt into the slab
   on most seeds; on ep57 the reported stove centre was (0.015, 0.068) instead
   of ≈ (0.10, 0.13) and the bowl was released off the slab. Fix: mask out the
   whole bowl **disc** (xy within `rim_r + 0.012` of the bowl centre) before
   looking for the slab — exact at every height, unlike a gate.
2. **Grasp retry.** A close that missed by ~5 mm in y left a 0.004 m gap at
   effort 0.05; the retry then could not re-find the bowl because the arm was
   parked above it, occluding the camera. Fix: retreat to a park pose clear of
   the camera line before re-perceiving, and walk a small ladder of straddle
   offsets (0, −7, +7 mm).

## Verification (no runtime success signal used)

The program never reads `api.done`. A grasp is accepted only when, after a
lift, `api.gripper()` reports `effort >= 2.0` **and** `width_m > 0.003` (a
closed-on-air gripper reads 0.001 / 0.05). Descents are closed-loop: each rung
is accepted only if the EEF actually fell 2 mm, otherwise the descent stops —
that is what finds both the grasp depth beside the bowl and the seating height
on the stove.

## DECLARATION

- **Frozen version: v12.** `program.py` md5 `254c84e4621cf6362da94a3e6f4103b8`
  == `program_v12.py` (identical on the cluster and locally).
- **Selection receipt: 15/15** on the full debug split (seeds 51–65),
  `results/sel_c2clean_goal_put_bowl_on_stove_pos_k3_v12` —
  every episode `"benchmark_success": true`.
- **Per-version receipt chain** (all `--split debug`, probe subset =
  51,53,55,57,59,61,63,65):
  - v1 `fs_..._v1` (51,53,55,57) — perception probe, deprojection + table z.
  - v2 `fs_..._v2` (51,53,55,57) — 12 mm gate clustering; fusion observed.
  - v3 `fs_..._v3` (8 seeds) — gate ladder; 0.032 m isolates the bowl.
  - v4 `fs_..._v4` (51,53) — fingertip offset 0.008 m.
  - v5 `fs_..._v5` (51) — RGB blob dump; bowl vs plate vs stove identified.
  - v6 `fs_..._v6` (51,53,55,57) — 0/4, arm never descended (−y approach).
  - v7 `fs_..._v7` (51) — step metering: 31 moves = 291 steps.
  - v8 `fs_..._v8` (51,53) — 0/2, arm jammed on the cabinet from −y.
  - v9 `fs_..._v9` (51) — +y ladder tracks to EEF 0.939 at bowl and stove.
  - v10 `fs_..._v10` (51) — offset landscape; +0.030 m holds at effort 3.0.
  - v11 `fs_..._v11` (8 seeds) — **6/8** (57, 59 fail).
  - v12 `fs_..._v12` (8 seeds) — **8/8**; `sel_..._v12` (15 seeds) — **15/15**.
- **PROVENANCE** present in `program.py` as a top-level literal dict covering
  every calibrated constant; each source is either a pack field or a
  debug-seed measurement recorded above.
