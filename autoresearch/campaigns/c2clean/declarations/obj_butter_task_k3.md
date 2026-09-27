# c2clean / obj_butter_task_k3 — "Pick the orange juice and place it in the basket"

Scene = the butter bddl; graded object = the orange juice. Two packs, neither
demonstrating the intent: `..._k3` (butter → basket, MY scene) and `..._mate`
(orange juice → basket, a DIFFERENT scene layout).

## What the packs actually gave

- **mate pack** (`language: pick up the orange juice and place it in the basket`):
  the grasp mechanism and, crucially, the *absolute close height* — keyframe ee z at
  `gripper_cmd=1` is 0.1203 / 0.0931 / 0.0981. Its grasp **xy** (~0.05, −0.107) is a
  decoy: different scene. Its keyframe PNGs name the target's appearance — a squat
  orange carton with a printed orange-fruit label.
- **k3 pack** (butter, my scene): the carry/release half — lift to z≈0.29, release over
  the basket at z 0.16–0.23, y≈+0.25..0.29. Its grasp (z≈0.01, y≈−0.26) is the butter,
  i.e. the anti-target.

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | *probe.* Nothing is known about the scene; log cam_high RGB-D out through `api.log` (zlib+base64) and rebuild the point cloud offline. | First cut used 3000-char chunks; `api.log` truncates at 2000 → corrupt base64. Re-run at 1900. Recovered RGB-D on 51/53/55/57. | Gave the frame; table plane at z = 0.001 in base frame, camera is plain CV convention (z forward), so `P_base = R·((u−cx)/f·d, (v−cy)/f·d, d) + t`. |
| v1 | Segment: connected components on `0.02 < z < 0.35` inside the workspace. The orange juice is the component with the most bright-label pixels (`r>110, g>60, b<70`); the basket is the big grey one. Grasp at `top − 0.040` (= the mate pack's 0.100 close height against this scene's measured carton top 0.139); carry at 0.29; release at 0.175. | Probe 51,53,55,57,59,61,63,65 → **8/8**. `lab` fraction 0.141 for the carton vs ≤0.01 for every other prop. Close: width 0.053, effort 3.0, held through the lift on every seed. | Works. Layout is deterministic across debug seeds (juice at (−0.1448, 0.0589) on all 15); only the basket jitters, ±0.02. |
| v2 | v1 plus guards for eval layouts I cannot see: basket must be grey (`|r−b|<20` on >40% of pixels) and >1200 px; if the target blob's top band spans >0.09 m it has fused with a neighbour, so fall back to the centroid of its own label pixels; grasp offset capped at `0.45·top` so a toppled carton is still gripped near mid-body; re-perceive and retry once if the lift reads not-holding; release height floored at `basket_top + 0.035`. | Formal 15/15 (see receipt). No seed triggered the retry or the fuse fallback; basket grey 0.95 vs target 0.25 — wide margin on the discriminator. | **Frozen.** Same score as v1, strictly more robust off the debug layout. |
| v3 | *Envelope probe, not a candidate.* Displace the grasp aim by +0.015 in x and +0.015 in y (21 mm diagonal) to measure the margin. | 4/4 on 51,53,55,57; close still width 0.055, effort 3.0. | The grasp is not knife-edge: ~2 cm of aim error is survivable, so perception noise on unseen layouts is not the failure mode to fear. |

## DECLARATION

- **Frozen version: v2.** `packs/c2clean_obj_butter_task_k3/program.py` md5
  `acf2536d2f3797dfb5a594b6151ba049` == `program_v2.py` (same md5).
- **Selection receipt (full 15 debug seeds 51–65): 15/15** —
  `results/sel_c2clean_obj_butter_task_k3_v2`.
- Receipt chain:
  - v1 probe (8 seeds) 8/8 — `results/fs_c2clean_obj_butter_task_k3_v1`
  - v1 formal (15 seeds) 15/15 — `results/sel_c2clean_obj_butter_task_k3_v1`
  - v2 formal (15 seeds) **15/15** — `results/sel_c2clean_obj_butter_task_k3_v2`
  - v3 aim-envelope probe (4 seeds, +21 mm aim error) 4/4 —
    `results/fs_c2clean_obj_butter_task_k3_v3env`
- **PROVENANCE** present in `program.py`: GRASP_Z, TOP_OFFSET, LIFT_Z, RELEASE_Z,
  LABEL_RGB, Z_FLOOR, ARM_Z, R_DOWN, BASKET_GREY, HOLD_CHECK, FUSE_SPAN — every
  constant sourced to a named pack field or a debug-seed measurement.
- Protocol: only `tools/fair_run.py`, only seeds 51–65, `api.done` never read, no
  forbidden file opened.
