# c2clean / spa_bowl_on_wooden_cabinet_task_k0 — worker notes

Intent: "Pick the akita black bowl on the stove and place it on the plate".
No demonstration pack. Runner: tools/fair_run.py only. Debug seeds 51-65.

## Scene (derived from debug seeds 51,53,57,61,65 — cam_high RGB-D only)

Table plane (mode of the height histogram) z = 0.9025 on every debug seed.
Props, all measured from the deprojected cam_high cloud:

| prop | centre (x,y) | top z | note |
|---|---|---|---|
| target bowl | ~(-0.26, -0.14) | 0.980 | on a low dark slab whose top is 0.9262 |
| second identical bowl | ~(0.03, -0.28) | 1.180 | on the tall drawered cabinet (top 1.137) |
| small bowl | ~(-0.20, 0.21) | 0.944 | on the table, footprint 0.087 (target is 0.11) |
| plate | ~(0.06, 0.20) | 0.920 | footprint 0.135 |
| cookie box | ~(0.07, 0.03) | 0.921 | |

Two identical black bowls sit in the scene; the instruction names the one on the
*stove* (the low slab, support 0.024 m above the table), not the one on the
tall wooden cabinet. Target selection is therefore purely by support height,
expressed as a rim-top band 0.05-0.11 m above the table plane.

## Version chain

### v1 — perception dump (3 eps, seeds 51/53/57)
Hypothesis: dump cam_high/cam_arm_wrist RGB-D through api.log and do all
analysis offline at zero simulator cost.
Evidence: api.log truncates every message at 2000 chars, so the blobs came back
truncated and undecodable.
Verdict: mechanism works, needs chunking.

### v2 — chunked perception dump (5 eps, seeds 51,53,57,61,65)
Evidence: full RGB + half-res depth recovered for five seeds. Scene table above.
Layout jitter between seeds is small: target bowl centre moves ~±0.01 in x,
~±0.015 in y; plate ~±0.01; everything else fixed.
A Kasa circle fit on the rim ring gives r = 0.0497-0.0501 on all five seeds —
far more stable than the bbox midpoint, which is biased whenever the robot arm
occludes part of the ring.
Verdict: geometry settled; grasp calibration still unknown.

### v3 — blocked-descent calibration (2 eps, seeds 51,57)
Hypothesis: a descent commanded below a surface stalls when the fingertips
touch it, so the stall height measures the fingertip-to-EEF offset.
Evidence:
- open-jaw descent onto bare table (table 0.9025) stalled at eef z 0.9087
  → FINGER_DZ = 0.0062;
- descent at the bowl rim ±y stalled at eef z 0.938/0.939 (bowl floor);
- the bowl did not move under any of the three probes (re-perception identical).
Verdict: FINGER_DZ = 0.006. Note the episode horizon is 1000 sim steps and each
*blocked* move burns its whole cap (~300 steps) — this run used all 1000.

### v4 — first full pick-and-place (4 eps: 51,53,55,57)
Rim pinch: wrist left straight down (jaws then open along base ±y), jaw centre
at (cx, cy+r), fingertips 0.015 m below the rim top, close, lift, carry, drop
with the payload 0.008 m above the plate. Drop height from a re-perception of
the hanging bowl.
Evidence: 3/4 — 51 ✓, 53 ✓, 57 ✓, 55 ✗.
Diagnosis on 55: the finger gap after the close was 0.0086 (same as the three
successes) but had collapsed to 0.0017 by the lift, i.e. the wall squirted out
of the jaws immediately. Successful carries all read 0.0042-0.0048 at the lift.
So a *silent* grasp miss, not a carry or a placement problem.
(The apparent "more sim steps ⇒ failure" correlation is an artefact: a success
terminates the episode early, so successful episodes are always shorter.)
Verdict: need a hold check, not a better open-loop aim.

### v5 — verification + retry (8 eps: 51,53,55,57,59,61,63,65) → 8/8
Two own-sensor checks close the loop:
1. **hold check** — after the lift, finger gap < 0.003 means the bite failed
   (measured: 0.0042-0.0048 holding vs 0.0010-0.0020 empty). On failure the
   jaws reopen and the cycle restarts from a fresh perception with the approach
   side flipped (+y ↔ -y).
2. **placement check** — after the release, re-perceive; the cycle repeats
   until a bowl-sized rim ring is found within 0.045 m of the plate centre.
Up to 3 cycles per episode; each costs ~150-250 sim steps, so three fit inside
the 1000-step horizon.
Evidence: probe subset 8/8. Seed 55 (v4's failure) took the retry path: hold
check caught the miss at gap 0.0020, flipped to the -y side, closed at 0.0075,
carried and placed, and the placement check verified a ring at (0.058, 0.183).
Verdict: selected for the formal 15-seed run.

## Mechanism notes worth keeping

- The rim pinch is a coin-flip-ish bite on a ~7 mm wall: the aim is right
  (jaw centre at the Kasa rim radius) and the failures are not aim failures,
  they are the jaws squeezing the slanted wall out. The cheap fix is to *detect*
  it from the finger gap and retry, not to re-aim.
- `effort` is useless here: it reads 3.0 only while the gap exceeds 0.005, and a
  genuine hold on this thin wall settles at ~0.0045. The gap itself is the
  receipt.
- Excluding points above table+0.15 from the height map before taking the
  per-cell maximum is what makes the tall cabinet and the robot arm disappear
  from the rim band; without it the cabinet's vertical faces slice into every
  height band and the arm punches a hole through the target ring.

## DECLARATION

- **Frozen version:** v5.
  `packs/c2clean_spa_bowl_on_wooden_cabinet_task_k0/program.py`
  md5 `2b2d3f5e2f2148c91530fb1afe3aa8cb`
  == `program_v5.py` md5 `2b2d3f5e2f2148c91530fb1afe3aa8cb` (verified on AbakaAI).
- **Selection receipt (full 15 debug seeds, one formal run):**
  **15/15** — `results/sel_c2clean_spa_bowl_on_wooden_cabinet_task_k0_v5`
  (seeds 51-65, every episode `"benchmark_success": true`).
- **Per-version receipt chain**
  | version | seeds run | result dir | receipt |
  |---|---|---|---|
  | v1 | 51,53,57 | `results/fs_..._v1` | perception dump; api.log truncates at 2000 chars |
  | v2 | 51,53,57,61,65 | `results/fs_..._v2` | chunked dump; full scene geometry recovered |
  | v3 | 51,57 | `results/fs_..._v3` | FINGER_DZ = 0.0062 from blocked descents |
  | v4 | 51,53,55,57 | `results/fs_..._v4` | 3/4 (55 dropped the bowl at the close) |
  | v5 | 51,53,55,57,59,61,63,65 | `results/fs_..._v5` | 8/8 probe |
  | v5 | 51-65 (all 15) | `results/sel_..._v5` | **15/15 selection** |
- **PROVENANCE:** present as a top-level literal dict in program.py, covering
  the workspace crop, the raster size, the rim/plate height bands and footprint
  filters, FINGER_DZ, GRIP_DEPTH, BOWL_H, HOLD_GAP, PLACE_CLEAR and the
  hover/carry heights. Every calibrated constant is sourced either to a
  debug-seed (51-65) measurement or to generic controller/camera mechanics; no
  demonstration pack was supplied or used, and no prior-campaign constant was
  carried in.
- **Archives:** `program_v1.py` … `program_v5.py` all present in the pack dir.

STOP.
