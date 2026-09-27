# c2clean / spa_bowl_on_wooden_cabinet_pos_k3

Intent: *pick up the black bowl on the wooden cabinet and place it on the plate*.
Runner: `tools/fair_run.py` only. Splits sealed: debug 51-65, eval 1-50 (never touched).

## What the scene actually is (debug-seed observation, probe0)

One cam_high RGB-D frame at the home pose is enough. Deprojected into a top-down
max-z map (4 mm cells) the scene is:

| thing | where (seeds 51-65) | signature |
|---|---|---|
| table | z 0.903 | modal height |
| wooden cabinet | x[-0.15,0.17] y[-0.35,-0.13], plateau z 1.128-1.130 | largest blob above 1.02 that does not reach the arm's altitude |
| **target bowl** | on that plateau, rim top 1.1797-1.1798, rim radius 0.0536-0.0541 | the only thing standing >18 mm above the plateau |
| **plate** | x -0.274..-0.246, y -0.150..-0.122, rim 0.950, floor 0.9405 | low blob whose top band is a FILLED disc |
| distractor bowls | two on the table, r 0.041 / 0.052 | top band is a ring with a hole (fill 0.00) |
| cookie box | r 0.030-0.042, top 0.920 | filled but far too small |
| robot arm | x[-0.28,-0.15] y[-0.10,0.10], z to 1.371 | never below z 1.02 at the home pose |

The plate is **not** where the pack demos put it. In all three demos it stands on
the table at (0.05, 0.22); on every debug seed it stands on the grey pedestal at
(-0.26, -0.13). This is a `_pos` cell: the demo anchor is a decoy and the goal
site has to be perceived. Everything the program uses is perceived per episode;
the pack contributes only the grasp *mechanism* (two numbers, below).

Discriminators, all re-derived here:
* **bowl vs plate** = the hole. A 16 mm top band of a bowl is an annulus whose
  inner disc is empty (fill 0.00); a plate's is solid (fill 1.00). No colour used.
* **rim centre and radius** = Kasa fit of the top 6 mm band. That band is a thin
  annulus and the fit is remarkably stable: r = 0.0536-0.0541 over 15 seeds,
  median residual 1.7-2.0 mm.
* **which side to pinch** = the pack. Its close-keyframe y (-0.2572/-0.2175/
  -0.2153) sits one rim radius on the **+y** side of bowl centres like the ones
  measured here; -y would have been ~-0.34. The jaws close along base -y
  (`tool_rotation` at home), so the tool goes to the rim wall at (cx, cy+r).
* **grasp height** = the pack. Its close-keyframe z averages 1.152, i.e. 28 mm
  below the measured rim top of 1.180.

## Version chain (all runs on the 8-seed probe subset unless noted)

| v | hypothesis | receipt | verdict |
|---|---|---|---|
| probe0 | dump RGB-D through `api.log`; do perception offline | 4 seeds, 72 sim steps, no motion | gave the whole table above |
| v1 | perceive, rim-pinch at (cx, cy+r, ztop-0.028), place tool on plate centre | **2/8** | two real bugs found |
| v2 | + close the loop on every move; + place the tool one rim radius on the +y side of the plate (the pinched bowl trails the tool by a full radius) | **5/8** | both fixes real; 51/61/63 burn all 1000 steps |
| v3 | carry at 1.21 instead of 1.28 | **5/8** | height is not the cause |
| v4 | walk the carry in hops after dropping to a 1.06 corridor | **0/8** | the *retry loop* is the budget sink: a goto chasing a 5 mm tolerance on an axis that will not track spends 500+ steps |
| v5 | abandon a retry as soon as it stops converging; carry y-then-x | **6/8** | budget fixed (190-220 steps); 61/63 still frozen |
| v6 | carry x-first at cabinet-top + hang + 45 mm | **5/8** | 51 dropped the bowl mid-carry; 61/63 unchanged |
| v7 | lower hover; log joints at every stage | **5/8** | **diagnosis**: on 61/63 the descent drives joint 4 to its hard limit (-0.0695 rad, straight elbow) and the controller can never pull the tool back in. j4 at the grasp is a clean monotone function of grasp x: -1.58 at x=-0.021 ... -1.30 at x=+0.030 ... -0.07 at x=+0.038 |
| **v8** | walk every long leg in 40 mm hops so `clip(err/0.05,±1)` never saturates | **8/8 probe, 15/15 formal** | **frozen** |

The v7 -> v8 step is the whole cell. A single full-length command saturates every
axis at once; on the far-x seeds that slam threw the arm across an IK branch into
the joint-4 limit. Hops short enough to stay in the controller's linear range
never do. It also made the grasp reproducible: the closed finger gap was bimodal
(0.0055 or 0.0179) while the descent slammed, and is 0.0100-0.0116 on all 15
seeds once it is walked.

## Margin

`probe_aim_p8` / `probe_aim_m8`: the pinch aim displaced +8 mm and -8 mm across
the rim wall, on seeds 54/58/60/63 (including the extreme grasp-x seed 60,
cx=+0.0451, and the extreme cy seed 58).

* +8 mm: **4/4**   * -8 mm: **4/4**

So the grasp tolerates at least a 16 mm-wide aim window, against a rim-fit spread
of 0.2 mm. Perturbation ranges observed over the 15 debug seeds:
cx [-0.021,+0.045], cy [-0.313,-0.269], plate x [-0.274,-0.246],
y [-0.150,-0.122]; rim top, plateau and plate floor constant to <2 mm.
Step usage 210-259 of the 1000-step horizon.

## DECLARATION

* **Frozen version: v8.** `packs/c2clean_spa_bowl_on_wooden_cabinet_pos_k3/program.py`
  md5 `1f06b318b3cdb545fa3027f2670383d4` == `program_v8.py` (verified on the cluster).
* **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, directory
  `results/sel_c2clean_spa_bowl_on_wooden_cabinet_pos_k3_v8`.
  Per-seed: 51..65 all `benchmark_success: true`.
* Per-version receipt chain: v1 2/8, v2 5/8, v3 5/8, v4 0/8, v5 6/8, v6 5/8,
  v7 5/8, v8 8/8 -> 15/15 (dirs `results/fs_..._v1` ... `_v8`, `results/sel_..._v8`).
* Margin receipts: `results/fs_..._aimp8` 4/4, `results/fs_..._aimm8` 4/4.
* `PROVENANCE` present in program.py, covering GRASP_DZ, RIM_SIDE, RIM_BAND_M,
  ON_TOP_DZ, ARM_ZMAX, LOW_ZMAX, XCROP, GRID_RES, WALK_M, HOVER_CLEAR,
  CARRY_CLEAR, PLACE_CLEAR, GOTO_TOL, CARRY_OFFSET. Sources are this pack's
  keyframes, debug-seed measurements, and controller mechanics only.
* `api.done` is never read. Eval seeds 1-50 were never run.
