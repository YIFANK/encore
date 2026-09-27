# c2k1clean / spa_bowl_next_to_plate_task_k1 — NOTES

Intent: "Pick the akita black bowl next to the ramekin and place it on the plate"
Runner: tools/fair_run.py only. bddl = pick_up_the_black_bowl_next_to_the_plate...

## Scene, re-derived from debug seeds 51/53/57/61 (v0, observation only)
cam_high 512x512 RGB-D, K=618.04/256, camera at base (0.659, 0, 1.610) looking
down the -x/-z diagonal. Deprojecting the whole frame into the base frame:

- table plane z = 0.900 (101k-pixel mode of the workspace height histogram)
- plate      : blob n~5500, top z 0.920, rim span 0.136, centre ~( 0.065, 0.20)
- cookie box : n~2150, top z 0.920, span (0.082,0.060), centre ( 0.08, 0.03)
- ramekin    : n~1850, top z 0.944, rim span 0.087, centre (-0.21, 0.20)
- akita bowl A: n~2520, top z 0.952, rim span 0.110, centre (-0.18, 0.33)
- akita bowl B: n~3300, top z 0.952, rim span 0.110, centre (-0.02, 0.34)
  (bowl B is clipped by the right image edge on 51/61 and entirely off-frame
  on 57 — bowl A and the ramekin are always fully visible)

So the scene holds TWO akita bowls. Bowl A is 0.13 from the ramekin and 0.29
from the plate; bowl B is 0.23 from the ramekin and 0.18 from the plate.
=> the intent's object ("next to the ramekin") is bowl A, which is NOT the
object the bddl filename names ("next to the plate" = bowl B). Whether the
benchmark predicate is instance-specific is unknown and is exactly what v1
measures.

Bowl radial profile (debug seed 51, distance from rim centre by height):
  z 0.907 r 0.030 | 0.917 r 0.042 | 0.922 r 0.040-0.046 | 0.937 r 0.051 |
  rim top 0.952 r 0.055.  Wall annulus at the pack's grasp height is
  0.0397-0.0476 => wall ~6 mm thick, midline r ~0.043.
Pack evidence: gripper closes at ee z 0.9225 (mate) / 0.929 (k1) = 27 mm below
the rim top, and carries at total width 0.0068-0.0115 => a thin-wall pinch,
not a body grasp (jaws open to 0.078 < the 0.110 rim).

## Versions
- v0 observation only, seeds 51,53,57,61 — 0/4 success (no motion; scene dump).
- v1 rim-pinch on the +y arc of bowl A, place centred on the plate. (running)

### v1 — rim-pinch on the +y arc of the bowl beside the ramekin
Hypothesis: the intent's object is the bowl nearest the ramekin; the bowl is a
thin-walled flare that must be pinched (jaws span 0.078 < rim 0.110), at the
pack's grasp height (rim_top - 0.027) and the wall midline radius (0.043); the
+y arc is chosen because on the -y arc the outer jaw passes within 3-10 mm of
the ramekin. Placement puts the bowl centre — which hangs at eef_y - 0.043 —
over the plate centre at eef z = plate_top + 0.0225 + 0.005.
Evidence: probe 51,53,55,57,59,61,63,65 -> 8/8
  (results/fs_c2k1clean_spa_bowl_next_to_plate_task_k1_v1); formal 15 -> 15/15
  (results/sel_c2k1clean_spa_bowl_next_to_plate_task_k1_v1). Every seed grasped
  on the FIRST attempt, gripper width 0.0067-0.0080 with effort 3.00 held from
  close through release.
Verdict: the hypothesis holds, and it settles the open question above — the
benchmark predicate FIRES for the bowl the intent names (the one next to the
ramekin), not only for the bowl the bddl filename names. Following the intent
sentence and scoring the benchmark bit are the same thing in this cell.

### v2 — harden the class test against image-edge clipping (frozen)
Hypothesis: v1 classified vessels by rim DIAMETER, which is wrong for a blob
the right image edge clips — bowl B is clipped on several seeds and measured
d=0.079, close enough to the ramekin's 0.087 to be mistaken for it (on seed 61
v1 called bowl B "not a bowl" and only the 8 mm size margin kept the true
ramekin). Height is the safe test: bowl rims 0.952, ramekin 0.944, plate 0.920.
Changes: bowls = vessels with ztop >= 0.948 (no width test); ramekin = an
UNCLIPPED vessel below 0.948 and narrower than 0.098; if no ramekin blob exists
(it fused with the bowl beside it) fall back to the bowl farthest from the
plate; a selected blob whose rim ring is not round (|sx-sy| > 0.025) is
re-centred on its top 6 mm, which belongs to the bowl alone. Debug dump off.
Evidence: formal 15 -> 15/15
  (results/sel_c2k1clean_spa_bowl_next_to_plate_task_k1_v2). Seed 61 now reads
  "2 bowls" as it should; UNFUSE never triggered (no fused blob occurs on
  51-65 — it is a guard for the eval band, where the closest observed
  ramekin-to-bowl separation, 0.102 on seed 56, leaves only 3.5 mm of rim gap).
Verdict: same 15/15 with the classification no longer resting on a width
margin that clipping can erase. FROZEN.

## DECLARATION
- Frozen version: **v2**.
  `packs/c2k1clean_spa_bowl_next_to_plate_task_k1/program.py`
  md5 `f0efa815b91d1eb33cf8d806151fe621` ==
  `program_v2.py` md5 `f0efa815b91d1eb33cf8d806151fe621`.
- Selection receipt (full 15 debug seeds 51-65): **15/15**
  `results/sel_c2k1clean_spa_bowl_next_to_plate_task_k1_v2`
- Receipt chain:
  - v0 (observation only) seeds 51,53,57,61 -> 0/4 (no motion)
    `results/fs_c2k1clean_spa_bowl_next_to_plate_task_k1_v0`
  - v1 probe 8 seeds -> 8/8
    `results/fs_c2k1clean_spa_bowl_next_to_plate_task_k1_v1`
  - v1 formal 15 -> 15/15
    `results/sel_c2k1clean_spa_bowl_next_to_plate_task_k1_v1`
  - v2 formal 15 -> 15/15 (argmax, frozen)
    `results/sel_c2k1clean_spa_bowl_next_to_plate_task_k1_v2`
- PROVENANCE: present in program.py as a top-level literal dict, 21 entries,
  every constant sourced to a named pack field or a debug-seed measurement.
- Archived versions: program_v0.py, program_v1.py, program_v2.py in the pack dir.
- No shared note file was read or written; no forbidden path was opened; splits
  respected (only --split debug, seeds 51-65); api.done never read.
