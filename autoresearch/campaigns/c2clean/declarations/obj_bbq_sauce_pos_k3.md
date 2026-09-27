# c2clean / obj_bbq_sauce_pos_k3 — notes

Intent: "pick up the bbq sauce and place it in the basket".
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51–65).

## v1 — perception dump (no manipulation)
**Hypothesis:** nothing is known about the scene; dump cam_high/cam_arm_wrist
RGB-D through `api.log` (zlib+base64 chunks) and do perception offline.

**Evidence** (seeds 51,53,57,61):
- cam_high K = f 618.04, c (256,256) at 512×512; `t_base_cam` puts the camera
  at (0.897, 0, 0.65) looking along (−0.849, 0, −0.529). Table top deprojects
  to z ≈ 0.001, so z is height above the table.
- Reset EEF (−0.1485, 0, 0.2613); tool rotation ≈ diag(1,−1,−1);
  open gripper width 0.0778 m.
- Scene = 5 props + basket. Height-masked (z>0.015) connected components give
  per-seed: a bottle with a **dark-red cap** (ztop 0.113), a bottle with a grey
  cap (ztop 0.148), a green-capped bottle (ztop 0.148), a low can (ztop 0.081),
  a small blue box (ztop 0.020), and the basket (ztop 0.144, 0.16×0.17 m).
- Only the dark-red-cap bottle and the basket move between seeds; the other
  four props are pixel-identical across 51/53/57/61.

**Identity (the load-bearing step).** Projecting the pack's close-keyframe EEF
positions into cam_high pixels (the camera is fixed, so the runtime extrinsics
apply to the 128×128 keyframe images at u,v/4) lands on
(179,274)/(189,280)/(185,268) in demo0/1/2. Cropping the demo `t0000` images
there shows an **amber bottle with a dark-red cap** in all three demos — not
the grey-capped bottle that occupies the same *pixel slot* at runtime. The
`_pos` perturbation has swapped the two bottles relative to the demos, so the
demo xy anchor is an **anti-anchor**: taking it would grasp the wrong bottle.

Cap-band (top 12 mm) redness `R − (G+B)/2`, debug seeds:
dark-red-cap bottle **0.215**, grey-cap bottle 0.007, basket −0.002,
can −0.015, green-cap bottle −0.091. Two orders of margin → identity by cap
colour, ranked (argmax), not thresholded.

**Verdict:** target = the red-capped bottle; aim from perception, never from
the demo anchor. Pack contributes the *mechanism* (grasp height, release
height), not the position.

## v2 — first full pick-and-place
**Hypothesis:** height-mask → own 4-connected components → target by argmax
cap redness, basket = largest low cluster. Grasp xy from the **mid-point of
the cap disc's X/Y range** (the cap top is fully visible from a 32° elevation,
so its bbox mid is unbiased; the cluster centroid is biased ~6 mm toward the
camera). Grasp height = ztop − 0.040 (pack demo0/demo1 close at ee z
0.0732/0.0739 on a 0.113-tall bottle). Carry at z 0.25, release at z 0.185
over the basket rim-bbox mid.

**Evidence:** probe 51,53,55,57,59,61,63,65 → **8/8**
(`results/fs_c2clean_obj_bbq_sauce_pos_k3_v2`).
Formal full-15 selection → **15/15**
(`results/sel_c2clean_obj_bbq_sauce_pos_k3_v2`).
Every seed: `TARGET by cap redness` 0.214–0.228, closed gripper width
0.0360–0.0366 at effort 3.00 held through the lift. The descent consistently
stops ~10 mm high (residual 0.0106, eef z 0.083 vs 0.073 commanded); the grasp
still lands on the bottle body, so no correction was needed.

**Verdict:** works, but the candidate filter used a hard pixel-count cap
(`n <= 1600`) that would drop the target if its cluster ever merged with a
neighbour. Target n is 997–1021 on every debug seed — untested headroom.

## v3 — remove the absolute size cap (FROZEN)
**Hypothesis:** replacing `80 <= n <= 1600` with `n < 0.5 * max(n)` keeps the
basket out of the candidate set (it is ~11× the target's pixel count) without
an absolute cap that a cluster merge could trip. Nothing else changed.

**Evidence:** formal full-15 selection → **15/15**
(`results/sel_c2clean_obj_bbq_sauce_pos_k3_v3`).

**Verdict:** frozen.

## v3e — aim-envelope probe (diagnostic, not a candidate)
v3 with the grasp aim displaced +12 mm in y. Seeds 51,53,55,57,59,61 → **6/6**
(`results/fs_c2clean_obj_bbq_sauce_pos_k3_v3e`). The perception-to-grasp
margin is therefore ≫ the 7 mm of target motion the `_pos` perturbation
produces across the debug seeds; the 15/15 is not a knife-edge result.

---

# DECLARATION

- **Frozen version:** `program_v3.py`, copied to
  `packs/c2clean_obj_bbq_sauce_pos_k3/program.py`.
  md5 `5b3feb995b0741ef925a93c0ec44fdfa` for **both** files (verified on the
  cluster).
- **Selection receipt (full 15 debug seeds, one formal run):**
  **15/15** — `results/sel_c2clean_obj_bbq_sauce_pos_k3_v3`
  (episodes 51–65, `benchmark_success` counted from `results.jsonl`).
- **Receipt chain:**
  | version | run | seeds | result |
  |---|---|---|---|
  | v1 | `fs_…_v1` | 51,53,57,61 | perception dump only, 0/4 (no manipulation) |
  | v2 | `fs_…_v2` | 8 probe seeds | 8/8 |
  | v2 | `sel_…_v2` | 51–65 | 15/15 |
  | v3 | `sel_…_v3` | 51–65 | **15/15** (frozen) |
  | v3e | `fs_…_v3e` | 6 probe seeds | 6/6 (+12 mm aim offset, diagnostic) |
- **PROVENANCE:** present as a top-level literal dict in `program_v3.py` /
  `program.py`, covering Z_TABLE, WORKSPACE, CAP_BAND, RED_MIN, TARGET_H,
  GRASP_DROP, GRIP_OPEN/GRIP_CLOSE, CARRY_Z, RELEASE_Z, R_DOWN. Every constant
  is sourced from `packs/c2clean_obj_bbq_sauce_pos_k3/pack.json`, from
  debug-seed (51–65) RGB-D / proprio measurements, or from generic
  camera/controller mechanics. No foreign-campaign or prior-knowledge
  constants were used.
- Eval split (seeds 1–50) was never run or read from this worker.
