# c2k1clean / obj_chocolate_pudding_pos_k1

Task: "pick up the chocolate pudding and place it in the basket"
(LIBERO object-swap layout, K=1 pack, _pos perturbation, debug seeds 51-65).

## What the pack says

`pack.json` holds one demonstration, 168 steps, 4 keyframes:

| t | ee xyz | gripper_cmd | reading |
|---|--------|-------------|---------|
| 0 | (-0.1485, -0.0029, 0.2587) | -1 (open) | home |
| 54 | (-0.1103, -0.2504, 0.0114) | +1 (close) | grasp, ee z = 0.0114 |
| 150 | (-0.0027, 0.2460, 0.1635) | -1 (open) | release over the basket |
| 167 | (0.0106, 0.2398, 0.1996) | -1 | retreat |

`ee_path` (17 waypoints) is a plain top-down pick-and-place: hover at z≈0.23,
descend to z≈0.01, close, lift to z≈0.21, traverse in +y, descend to z≈0.16,
open.  Differencing keyframe images t0000 vs t0150 shows the removed prop is a
small dark-brown box that sat behind the amber bottle — the chocolate pudding
is a *box*, not a bottle.

Crucially the demo's scene is **not** the evaluated scene (object-swap): the
demo row is bottle / can+carton / red bottle / green bottle; the debug scenes
hold a maroon BBQ bottle, an orange sauce bottle, a grey can, an orange juice
carton, a green dressing bottle, and one flat brown box.  So the pack supplies
the *motion template* and the *target class* (a small brown box); everything
metric had to be re-derived from debug-seed RGB-D.

## Perception derived on debug seeds

cam_high is 512x512, K = diag(618.04, 618.04) with centre (256,256), camera at
base (0.8966, 0, 0.65) looking down ≈58°.  Pipeline: deproject every valid
pixel, take the mode of the world-z histogram as the table (+0.0025 m), keep
points 0.008–0.18 m above it, connected-component 0.012 m xy cells.

Measured layout (identical on seeds 51,52,58,64 except where noted):

| xy | footprint | height | what |
|----|-----------|--------|------|
| (-0.119, -0.241)* | 0.026 x 0.048 | 0.111 | maroon BBQ bottle (*moves ±0.015) |
| ( 0.114, -0.197) | 0.037 x 0.062 | 0.145 | orange sauce bottle |
| ( 0.069, -0.099) | 0.053 x 0.053 | 0.138 | orange juice carton |
| (-0.184, -0.082) | 0.062 x 0.068 | 0.078 | grey can |
| ( 0.164,  0.028) | 0.037 x 0.062 | 0.145 | green dressing bottle |
| **(-0.138, 0.059)** | **0.080 x 0.048** | **0.027** | **chocolate pudding box** |
| ( 0.030, 0.255)* | 0.154 x 0.170 | 0.139 | basket (*moves ±0.015) |

Two facts do the identification work and both are large margins:
* the basket is the largest component by ~4x (1.2e4 px vs <3e3);
* the pudding is the flattest prop by ~3x (0.027 m vs 0.078 m next).

The 0.18 m ceiling is what makes this work at all: at the home pose the arm
projects onto the same xy cells as the pudding and fuses with it (v0 saw only
6 components, the pudding swallowed by the arm).  The arm's lowest point is
≈0.19 m above the table, the tallest prop 0.148 m, so the ceiling separates
them cleanly.

## Version chain

| ver | hypothesis | evidence | verdict |
|-----|-----------|----------|---------|
| v0 | perception only: can cam_high RGB-D resolve the props? | `fs_..._v0`, seeds 51,53,55,57. 6 components; the pudding fused into the arm cluster (no 0.18 m ceiling). | rejected as-is; gave table_z, K, T, and the arm-occlusion diagnosis |
| p1 | dump RGB+depth (zlib+base64 through api.log) for offline study | `fs_..._p1`, seeds 51,52,58,64. Reconstructed the scene images; identified the flat brown box at (-0.138,0.059) as the pudding and confirmed the demo object class | accepted (diagnostic only) |
| v1 | full pick-and-place, flattest-component target, `move_path` transport | `fs_..._v1`, 8 seeds, 0/8 — `move_path` is not implemented on the LIBERO backend (AttributeError). **But the grasp worked**: closed width 0.0463 m, effort 3.00, held through the lift, proving the jaws close along world y and the default straight-down wrist already spans the box's 0.048 m short side. | rejected (API), mechanism confirmed |
| v2 | same, transport as three `move()` calls at z=0.22 | `fs_..._v2`, seeds 51,53,55,57,59,61,63,65 — **8/8** | accepted |
| v3 | v2 + provenance, object-relative grasp height (`top - 0.018`), rescan-if-no-flat-component fallback, grasp-verify-and-retry (effort < 2.5), wrist yaw onto the footprint minor axis if the y-extent exceeds the 0.068 m jaw span | `sel_..._v3`, all 15 debug seeds — **15/15** | **frozen** |

v3's extra branches are all no-ops on every debug seed (the box is always flat,
always found on the first scan, always gripped on the first attempt); they exist
so a perturbed eval layout that hides or rotates the box degrades gracefully
rather than failing outright.

## Observed perturbation

Under `_pos` on seeds 51-65 only the basket (±0.015 m) and the maroon BBQ bottle
(±0.015 m) move; the pudding sits at (-0.138, 0.059) on every debug seed.  The
program never uses that constant — target and basket are both re-perceived
every episode — so a wider eval perturbation is handled by the same code path.

## DECLARATION

* **Frozen version:** `program_v3.py`, copied to
  `packs/c2k1clean_obj_chocolate_pudding_pos_k1/program.py`.
  `md5(program.py) == md5(program_v3.py) == fef3d8380f3a1a4dee5220a4fa5fe5ff`.
* **Selection receipt (full 15 debug seeds 51–65):** **15/15**
  (`"benchmark_success": true` count = 15 in
  `results/sel_c2k1clean_obj_chocolate_pudding_pos_k1_v3/results.jsonl`).
* **Per-version receipt chain:**
  * `results/fs_c2k1clean_obj_chocolate_pudding_pos_k1_v0` — 0/4 (perception probe, no motion)
  * `results/fs_c2k1clean_obj_chocolate_pudding_pos_k1_p1` — 0/4 (image dump, no motion)
  * `results/fs_c2k1clean_obj_chocolate_pudding_pos_k1_v1` — 0/8 (`move_path` unavailable)
  * `results/fs_c2k1clean_obj_chocolate_pudding_pos_k1_v2` — 8/8 (probe subset 51,53,…,65)
  * `results/sel_c2k1clean_obj_chocolate_pudding_pos_k1_v3` — 15/15 (formal, all debug seeds)
* **Archived versions on the cluster:** `program_v0.py`, `program_v1.py`,
  `program_v2.py`, `program_v3.py`, `probe_v1.py` in the pack directory.
* **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  all 20 calibrated constants; every entry sourced to a pack field or a
  debug-seed measurement.
* No `.done` read; no `fewshot_run.py`; no benchmark-asset reads; writes confined
  to `packs/c2k1clean_obj_chocolate_pudding_pos_k1/*` and
  `results/*c2k1clean_obj_chocolate_pudding_pos_k1*`.

STOP.
