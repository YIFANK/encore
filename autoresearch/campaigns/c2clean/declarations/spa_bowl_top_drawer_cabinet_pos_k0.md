# c2clean / spa_bowl_top_drawer_cabinet_pos_k0 (K=0, no demonstrations)

Intent: *pick up the black bowl in the top drawer of the wooden cabinet and
place it on the plate.* Runner: `tools/fair_run.py` only. Splits sealed
(debug 51-65; eval 1-50 never touched).

## Scene, as measured (debug seeds only)

`cam_high` depth deprojected to the base frame. Table plane = the modal z of
the workspace cloud, **0.9010 m** (std 0.1 mm over a clear patch). Three bowls
are the only bright (min RGB channel > 100) objects above it, and they sit at
three well-separated heights, identical to the millimetre across all 15 debug
seeds:

| object | rim ztop | height over table | outer rim radius |
|---|---|---|---|
| bowl on the cabinet top | 1.179-1.180 | 0.278-0.279 | 0.051 |
| **bowl in the open drawer (target)** | **1.116** | **0.215** | **0.053** |
| bowl on the table | 0.944 | 0.043 | 0.042 |

The drawer is already open at reset. Its floor is at 1.07 and its walls top out
at ~1.12, i.e. *above* the target's rim. Interior clearance around the bowl:
0.030 m in y, 0.015 m on the near (+x) side, **0.075 m on the far (-x) side**.
The plate is the only 0.134-0.137 m round flat cluster, ztop = table+0.019.

`_pos` moves the cabinet, the plate and the table bowl by 1-3 cm between seeds;
nothing is ever inferred from a fixed coordinate.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump RGB-D through `api.log` and do perception offline | works once chunks are <= 2000 chars (`api.log` truncates at 2000) | scene mapped |
| v2 | press the open gripper on the table to get the fingertip offset | stalls at eef 0.9374 over a 0.9010 plane -> "0.036" | **wrong**, see v4 |
| v3 | rim pinch at the rim radius, bite 0.032 deep | closed gap 0.0018 (full close), bowl untouched | miss |
| v4 | the wall tapers; aim at the interior radius at the bite depth | still 0.0018, bowl untouched -> the tips were never near the bowl | miss, but named the real error |
| v4b | measure the fingertips optically instead: depth-difference of `cam_high` between two hover heights isolates the arm | lowest gripper point 0.0078 / 0.0069 m below the eef; wrist cam agrees (0.0093) | **TIP_OFF = 0.008**; v2's stall was the robot's own base, not the table |
| v5 | with the true tip offset, bite the far arc at r_in(0.012) +- 0.006 | all three radii closed on the wall: gap 0.0063-0.0084, effort 3.0 | bite solved, +-6 mm tolerant |
| v6 | full pick and place, open-loop drop 0.025 m over the plate | **14/15** (seed 62: good bite, gap collapsed to 0.0017 during the lift) | near |
| v7 | closed-loop place from a re-perceived "hang" | 3/15 -- the hang mask ate the bright wood table, and the carry xy dropped the rim-radius offset | **regression, reverted** |
| v8 | v6 + lift from the eef's *actual* xy (the planned xy re-commanded during the lift shears the wall out of the jaws) + one retry | **15/15** | selected mechanism |
| v9 | v8 + park the arm away before the retry's re-perception (`perceive` rejects clusters at the eef xy, which after a failed bite is the target itself) | **15/15** | **FROZEN** |

## What was load-bearing

1. **The fingertip offset is 8 mm, not 36 mm.** The press probe measured the
   robot's own base plate at ~0.93, not the table; the two optical measurements
   (depth-difference and wrist camera) agree with each other and explain every
   earlier miss. Three versions were spent on a grasp aimed 25 mm too high.
2. **A bowl wall tapers, so the pinch radius belongs to the bite depth, not to
   the rim.** Measured interior profile: r = 0.0511 at 6 mm below the rim,
   0.0403 at 30 mm (slope 0.45 m/m). The program reads r_in(0.012) per seed.
3. **Pick the arc with room for the outer finger.** The jaws span 0.078 m, so
   the outer finger lands 0.039 m outside the pinch point: the y arcs and the
   near arc put it inside a drawer wall; only the far (-x) arc has 0.075 m.
   The wrist is yawed 90 deg (`R_YAW`) to put the jaws along x.
4. **Lift from where the arm is, not from where it was told to go.** `POS_TOL`
   is 12 mm, so a converged descent lands a few mm off; re-commanding the
   planned xy on the lift corrects that error *while holding the wall*, and the
   shear is what lost seed 62 in v6.

## DECLARATION

- **Frozen version: v9.** `packs/c2clean_spa_bowl_top_drawer_cabinet_pos_k0/program.py`
  md5 `b03ea8f67ec1fb405fa864c2e3236c6e` == `program_v9.py` (same md5).
- **Selection receipt: 15/15 on the full debug set (seeds 51-65)**, directory
  `results/sel_c2clean_spa_bowl_top_drawer_cabinet_pos_k0_v9`.
  Every seed bit the wall on the first attempt (closed gap 0.0073-0.0083,
  effort 3.0); the retry path never fired. Peak cost 156 sim steps of the
  500-step horizon.
- Receipt chain: v6 `sel_..._v6` 14/15; v7 `sel_..._v7` 3/15 (reverted);
  v8 `sel_..._v8` 15/15; v9 `sel_..._v9` 15/15. Probes: `fs_..._v1`, `_v1b`,
  `_v2`, `_v3`, `_v4`, `_v5`, `_v6`.
- Versions v1-v9 archived in the pack directory.
- `PROVENANCE` present in `program.py`, covering TABLE_BAND, BRIGHT_CUT,
  BOWL_SIZE, TARGET_BAND, PLATE_SHAPE, TIP_OFF, BITE, CARRY_Z, PARK, HOLD_GAP.
  Every constant traces to a debug-seed measurement or to generic controller /
  camera mechanics. No demonstration pack; no shared note file; eval seeds
  1-50 never run.
