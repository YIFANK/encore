# c2k1clean / goal_put_bowl_on_stove_task_k1

Intent: **Put the plate on the stove**.
Packs: `..._k1` = "put the bowl on the stove" (right TARGET, wrong object);
`..._mate` = "push the plate to the front of the stove" (right OBJECT, wrong
target, and it never closes the gripper).

## Scene, as measured on debug seeds (cam_high depth -> base-frame height map)

- table top z = 0.9013 (median of the height map).
- **plate** (the object): a shallow dish, outer diameter 0.137 m, rim tops at
  +0.017..0.019 m, interior floor +0.006..0.007 m. Centre moves seed to seed,
  roughly (0.05, -0.01).
- **stove**: fixed across seeds. Slab 0.19 x 0.19 m at +0.025 m, spanning
  x[-0.35,-0.16], y[0.115,0.31]; raised burner disc at +0.030 m, centre
  approx (-0.256, 0.211).
- **bowl** +0.051 m at x[-0.136,-0.028], y[-0.061,0.052] -- sits directly on
  the straight-line carry path.
- **cabinet** +0.226 m, face at y = -0.124; **rack** +0.344 m at y < -0.12.
- gripper: binary open/close, jaws close along world y with the default
  downward wrist, open gap 0.079 m -- far narrower than the 0.137 m plate, so
  only a rim pinch is available.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | perception probe | table 0.9013, plate/stove/bowl geometry above | scene mapped |
| v2 | rim pinch on the -y rim | arm froze 0.16 m up, residual = full error | blocked |
| v3 | is the stall kinematic? | joint 4 pinned at its -0.070 limit | one long far+high move straightens the elbow |
| v4 | staged descending-diagonal approach (both packs' ee_paths do this) | elbow stays bent, but still frozen at the -y rim | approach fixed, aim still wrong |
| v5 | the -y rim faces the cabinet; use the rim side with headroom | descends cleanly; close gives gap 0.0108 effort 3.0; plate ratchets out on the lift | grasp exists, transport fails |
| v6 | bite vs depth (3 parallel runs) | deeper = thicker bite | go deeper |
| v7 | fingertip offset | moves settle ~10 mm above the commanded z; fingertips ~8.5 mm below the eef, so commanded dz ~= fingertip height | calibration |
| v8 | re-perceive each round | arm occludes cam_high | perceive once, up front |
| v9 | deep bite sweep | dz -0.002 -> 20.5 mm bite, the only one to survive a 0.15 m lift | GRASP_DZ = -0.002 |
| v10 | full attempt, carry at h 0.075 | held through the lift, dropped during the carry | transport is the binding constraint |
| v11 | inset 0.004..0.022 x carry in 1/5/9 hops | bite insensitive to inset (0.0203..0.0216); hops help; every drop happens near x ~ -0.11 | the **bowl** knocks the plate out |
| v12 | carry higher | LIFT_H 0.14 + 5 hops: **6/8** on the probe subset | carrying above the bowl stops the bleed entirely |
| v13 | dog-leg round the bowl at low height vs high direct | dog-leg 0/8 and 0/8 and 1/8; high direct with 0.07 m hops **7/8** (fails 57) | keep the high direct route |
| v14 | the remaining loss is all in the 0.13 m lift: hop the lift too | 0.035 m and 0.020 m lift hops both 8/8; the combined diagonal 2/8 | hop the lift |
| v15 | v14a plus one grasp retry (re-park, re-perceive, re-grab if the lift comes up empty) | 8/8 odd + 7/7 even | equal to v14a, with a fallback |
| v16 | v15 with LIFT_H / LIFT_HOP / CARRY_HOP / HELD_TEST added to PROVENANCE (code byte-identical otherwise) | selection 15/15 | **FROZEN** |

## Mechanism notes

- The rim is a wedge that is thicker at the bottom, so a squeeze drives the
  plate *down* and out. It still holds (effort 3.0) because the outer wall is
  only ~29 deg off vertical, inside the friction cone -- but the bite bleeds
  with every acceleration, and collapses abruptly once it is under ~5 mm.
- Bleed is dominated by (a) collisions along the carry and (b) the lift; a
  purely lateral move at a height that clears everything costs nothing (the
  gap even grows back slightly).
- The benchmark bit fires while the plate is still in the hand, as soon as it
  is over the stove: successful episodes terminate mid-carry at 137-222 sim
  steps. The programs still finish the place, so they do not depend on that.

## Per-version receipt chain (all `--split debug`)

| version | seeds | result | dir |
|---|---|---|---|
| v1 | 51,52 | perception probe | `results/fs_..._v1` |
| v2 | 51,52 | froze at the -y rim | `results/fs_..._v2` |
| v3 | 51,52 | joint 4 at its limit | `results/fs_..._v3` |
| v4 | 51,52 | staged approach, still frozen | `results/fs_..._v4` |
| vH | 51 | 60 moves / 372 steps, no horizon hit | `results/fs_..._vh` |
| v5 | 51,52,53 | first grasp (gap 0.0108, effort 3.0), lost on the lift | `results/fs_..._v5` |
| v6a/b/c | 51 | bite vs depth | `results/fs_..._v6{a,b,c}` |
| v7 | 51 | fingertip offset / wall profile | `results/fs_..._v7` |
| v8a/b | 51 | aborted: the arm occludes cam_high | `results/fs_..._v8{a,b}` |
| v9a/b/c | 51 | dz -0.002 -> 20.5 mm bite survives a 0.15 m lift | `results/fs_..._v9{a,b,c}` |
| v10a | 51,53,55,57 | 0/4, dropped mid-carry | `results/fs_..._v10a` |
| v10b | 51 | held-plate height map | `results/fs_..._v10b` |
| v11a-f | 51,53 | 0/12; every drop at the bowl crossing | `results/fs_..._v11{a..f}` |
| v12a/b/c/d | 51,53,55 | 1/3, 1/3, 0/3, **3/3** | `results/fs_..._v12{a,b,c,d}` |
| v12d | 51..65 odd | **6/8** | `results/fs_..._v12dp` |
| v13a/b/c/d | 51..65 odd | 0/8, 0/8, 1/8, **7/8** (fails 57) | `results/fs_..._v13{a,b,c,d}` |
| v14a/b/c/d | 51..65 odd | **8/8**, 8/8, 2/8, 8/8 | `results/fs_..._v14{a,b,c,d}` |
| v14a/b/d, v15 | 52..64 even | 7/7, 5/7, 6/7, **7/7** | `results/fs_..._e{14a,14b,14d,15}` |
| v15 | 51..65 odd | 8/8 | `results/fs_..._o15` |
| v15 | 51..65 all 15 | 15/15 | `results/sel_..._v15` |
| **v16** | **51..65 all 15** | **15/15** | `results/sel_..._v16` |

## DECLARATION

- **Frozen version: v16.**
  `packs/c2k1clean_goal_put_bowl_on_stove_task_k1/program.py`
  md5 `267c34931e51220ea0b229321827dbbf`
  == `program_v16.py` == the archived `program_archived.py` of the selection run.
- **Selection receipt: 15/15** `"benchmark_success": true` on the full debug
  band, seeds 51-65, one run:
  `results/sel_c2k1clean_goal_put_bowl_on_stove_task_k1_v16/results.jsonl`
  (51:T 52:T 53:T 54:T 55:T 56:T 57:T 58:T 59:T 60:T 61:T 62:T 63:T 64:T 65:T;
  147-231 sim steps per episode).
- **PROVENANCE present**: 12 entries, every one `allowed: True` with a source
  that is either a named pack field, a debug-seed measurement, or generic
  controller/camera mechanics. No forbidden tokens; no `.done` read.
- Runner: `tools/fair_run.py` only. `tools/fewshot_run.py` was never invoked.
  No benchmark asset was opened; no other campaign's artifacts were read.

### What the frozen program does

1. One `cam_high` capture -> base-frame height map. Table plane = the map's
   median. The plate is the disc-shaped component in the +0.008..0.024 m band
   whose extent is nearest 0.144 m; its circle is fitted to the per-row y
   silhouette extremes (the camera sits at y = 0, so those edges are unbiased,
   while the near x edge is not). The burner is the largest component above
   +0.027 m that does not reach +0.045 m.
2. Grasp side = the +/-y rim whose outboard strip is clear; the -y rim faces
   the 0.226 m cabinet and the hand cannot get down beside it.
3. Approach on a descending diagonal (x -0.03 -> gx while z falls), the shape
   both packs' ee_paths use; the direct route straightens the elbow into its
   joint-4 limit and the arm then cannot descend at all.
4. Descend to table - 0.002 (fingertips on the table beside the rim), close.
   That is the deepest bite available, ~20.5 mm of rim wall.
5. Lift to +0.14 m in 0.035 m hops, carry to the burner in 0.07 m hops. Both
   the height and the hop length matter: the carry line passes over a 0.051 m
   bowl, and a saturated long move snatches the plate out of the wedge.
6. If the lift comes up empty, park clear of the camera, re-perceive and try
   once more.
7. Lower onto the burner and open. (On every debug seed the benchmark bit
   fires earlier, while the plate is over the stove in the hand, so the
   episode terminates during the carry; the program does not read that.)
