# c2clean / spa_bowl_on_stove_task_k3 — worker notes

Intent (authoritative): "Pick the akita black bowl on the top of the cabinet and
place it on the plate".
Runner: `tools/fair_run.py` only. Debug band 51-65. No shared note file.

## Scene, as measured (not assumed)

All numbers below come from my own `cam_high` RGB-D on debug seeds 51/53/55/57
(v0 streamed the frame out through `api.log` as zlib+base64 and I deprojected it
offline) plus the two pack `pack.json` files.

Camera `cam_high` sits at base (0.659, 0, 1.610) looking along −x and down; its
image u axis maps to base +y, so image-right is +y and the robot base is at −x.

Top-down height map (5 mm cells), consistent across all four probe seeds:

| feature | footprint | top z |
|---|---|---|
| table | everywhere | 0.899–0.902 (clean gap to 0.907) |
| cabinet-top plateau (fixture) | x[−0.09, 0.17] y[−0.345, −0.14] | 1.127 |
| **target bowl** (only blob above the plateau) | 0.105–0.11 m wide, centre wanders by seed | **1.180** |
| plate (largest flat blob in [0.905, 0.928]) | 0.14 m disc, n≈630 cells | 0.920 |
| stove + stove bowl | x[−0.31, −0.17] y[−0.19, −0.05] | 0.980 |
| metal bowl on table | x[−0.24, −0.16] y[0.17, 0.26] | 0.944 |
| cookie card | ≈0.08 m, brown (94, 65, 45) | 0.920 |

Target-bowl centre over seeds 51/53/55/57: x ∈ [−0.025, 0.045],
y ∈ [−0.295, −0.267]. So the target must be perceived, but it is never
ambiguous: **exactly one blob rises above the cabinet-top plateau.**

## Mechanism, read off the packs

- `..._mate` ("pick up the black bowl on the wooden cabinet and place it on the
  plate") demonstrates my intent's object and its keyframe images show the
  cabinet-top bowl gone and a bowl on the plate. `..._k3` demonstrates the same
  mechanism on the *stove* bowl. Both are motion evidence; the instruction is
  the target.
- Finger gap at the mate close is 0.0026/0.0027 per finger (gap ≈ 0.005). The
  bowl footprint is 0.105 m and the jaws open 0.078 m, so a straddle is
  geometrically impossible: **the grasp is a rim pinch on the thin bowl wall.**
- Both packs release with eef y ≈ 0.04 past the plate centre (measured at
  y ≈ 0.195). With jaws closing along base y that means the pinched bowl trails
  the eef by one rim radius along −y, i.e. **the pinch is on the +y arc** and
  the place aim must be shifted +rim_r in y.
- Close altitude, two independent confirmations of the same relation:
  mate closes at eef z 1.146/1.152/1.159 against a measured cabinet-bowl rim of
  1.180; k3 closes at 0.948 against a measured stove-bowl rim of 0.980. Both:
  **eef_z ≈ rim_top − 0.030.**
- Carry apex 1.278–1.315; release eef z 0.954–0.980 = plate rim + 0.034…0.060.

## Version log

### v0 — perception probe (no motion)
Hypothesis: the scene can be measured from `cam_high` alone.
Evidence: seeds 51/53/55/57, table/plateau/bowl/plate all separate cleanly by
height; the band above the plateau contains exactly one blob.
Verdict: perception solved; table-surface gap 0.902→0.907 gives a 5 mm margin
for the flat band, so the plate never fuses with the table.
Receipt: `results/fs_c2clean_spa_bowl_on_stove_task_k3_v0` (0/4 — no motion).

### v1 — rim-pinch the +y arc, carry, place with the trailing offset
Hypothesis: grasp at (bowl_cx, bowl_cy + rim_r, rim_top − 0.030) with the wrist
frozen straight down (`api.move(..., rotation=None)` freezes the wrist and gives
twice the step budget of `move_pose`), lift to 1.280, traverse, descend to
(plate_cx, plate_cy + rim_r, plate_top + 0.045), open.
Evidence (probe 51,53,…,65 — `results/fs_c2clean_spa_bowl_on_stove_task_k3_v1`,
**8/8**): close gap 0.0068–0.0110 on every seed — a real wall bite, not air
(air closes to ≈0) and not a straddle; the gap survives the lift to 1.27 on
every seed; the post-place re-perception finds `cab_top_blobs=0` on every seed,
so the bowl really left the cabinet. Move residuals 0.006–0.016, i.e. every
waypoint converged inside POS_TOL. The RETREAT move no-ops in all 8 seeds
(residual 0.11, eef unchanged) — the episode has already ended by then; no logic
depends on it.
Verdict: mechanism correct and repeatable; carried to the formal 15.

Note on `effort`: the harness reports effort 3.0 only while the gap exceeds
0.005, so a thin rim bite reads "not holding" at 0.0043 during the carry. Hold
is therefore verified by **gap survival across the lift**, not by the flag.

## DECLARATION

Frozen version: **v1** — `program.py` == `program_v1.py`
(md5 `c61edaf3b54cf8e7062c0119a62fc8a4`).

Selection receipt (formal, full 15 debug seeds 51-65):
**15/15** — `results/sel_c2clean_spa_bowl_on_stove_task_k3_v1`

Per-version receipt chain:
- v0 perception probe, 4 seeds, no motion — `results/fs_c2clean_spa_bowl_on_stove_task_k3_v0`
- v1 probe 8 seeds (51,53,…,65) — 8/8 — `results/fs_c2clean_spa_bowl_on_stove_task_k3_v1`
- v1 formal 15 seeds (51-65) — **15/15** — `results/sel_c2clean_spa_bowl_on_stove_task_k3_v1`

PROVENANCE: present in `program.py` as a top-level literal dict, covering
GRID_RES, X_RANGE, Y_RANGE, TABLE_Z, FLAT_BAND, CAB_BAND, CAB_MARGIN, GRASP_DZ,
RIM_OFFSET_SIGN, CARRY_Z, PLACE_DZ, APPROACH_Z. Every entry is sourced to either
the two named packs or my own debug-seed measurements.
