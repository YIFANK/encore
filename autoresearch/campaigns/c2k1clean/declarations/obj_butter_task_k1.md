# c2k1clean / obj_butter_task_k1 — NOTES

Intent: **"Pick the orange juice and place it in the basket"** (scene = the
butter/basket bddl; the k1 pack demos butter→basket in THIS scene, the mate
pack demos orange-juice→basket in a DIFFERENT scene).

## Pack reading (pack.json + keyframes only)

- k1 pack language `pick up the butter and place it in the basket`, K=1, 169
  steps. Grasp keyframe t=63: eef `(-0.121,-0.254,0.0095)`, gripper_cmd +1.
  Release keyframe t=150: eef `(0.023,0.250,0.191)`, cmd -1.
- mate pack language `pick up the orange juice and place it in the basket`,
  K=1, 126 steps. Grasp keyframe t=42: eef `(0.078,-0.106,0.120)`.
  Release t=116: eef `(0.002,0.224,0.168)`.
- **Object identity**: differencing the mate pack's t0000 and t0116 keyframes
  shows the object that left the table is a tall carton with a saturated
  orange disc on a yellow-green label. The same carton is visible in the k1
  pack's own t0000 frame (128-px pixel ~(71,52)) and is still there at t0150,
  i.e. it is present in our scene and is *not* what the k1 demo picks (the k1
  demo picks a flat brown box — the butter).
- Grasp-height cue: mate closes at z=0.120; k1 closes at z=0.0095 on a box
  whose measured top is 0.019 ⇒ the eef z sits at the mid-span of what it
  grips (not a fingertip offset). So the carton wants z ≈ top − 0.023.

## Version log

### v1 — perception probe (no motion), seeds 51,53,55,57
Hypothesis: find the table plane and the props. Evidence: table z = 0.0012;
a pure-xy clustering fuses the robot arm (points up to z=0.486 at x≈−0.15)
into the props. Verdict: need a height band + connected components.

### v2 — perception probe #2, seeds 51,53,55,57
Height band (0.015, 0.25) above table + 12 mm xy occupancy-grid flood fill
gives exactly **7 components** on every seed:

| k | xy | ztop | span | px | orange-frac | reading |
|---|----|------|------|----|-------------|---------|
| 0 | (+0.03,+0.26) | 0.144 | .16×.17 | (414,264) | 0.00 | **basket** |
| 1 | (+0.063,−0.101) | 0.081 | .064×.070 | (194,293) | 0.03 | green can |
| 2 | (−0.130,+0.059) | 0.143 | .040×.053 | (286,222) | **0.44** | **orange juice** |
| 3 | (−0.185,−0.079) | 0.148 | .033×.063 | (215,210) | 0.14 | brown bottle |
| 4 | (+0.16,+0.03) | 0.113 | .027×.049 | (275,327) | 0.11 | brown bottle |
| 5 | (+0.108,−0.199) | 0.029 | .080×.048 | (133,325) | 0.11 | flat box |
| 6 | (−0.118,−0.239) | 0.019 | .073×.039 | (131,256) | 0.53 | flat box = butter (k1 pack grasps it) |

Pixel columns match the k1 keyframe one-for-one. Verdict: among props taller
than 0.07 the orange fraction is 0.44 vs ≤0.14 — a clean identity cut. Note
k1/k2/k3/k5 are byte-identical across these four seeds; only the basket, k4
and k6 move, so the per-seed variation is small but real.

### v3 — first pick-and-place, seeds 51..65 odd
Hypothesis: pick the orange carton at `top − 0.023`, carry at 0.26, release at
0.19 over the basket point-cloud centroid.
Evidence (`results/fs_c2k1clean_obj_butter_task_k1_v3`, seeds 51..65 odd):
**0/8**. The grasp itself was perfect on every seed — gripper closed to
width 0.0536 with effort 3.0 (the mate pack's own hold width was 0.0546) and
the carton stayed held through the transport. The gif shows it released
*beside* the basket. Verdict: transport/release geometry, not grasping.

### v4 — release on the rim-bbox midpoint, seeds 51..65 odd
Hypothesis: the basket centroid is pulled onto the camera-facing walls.
Evidence: it is — centroid (+0.028,+0.264) vs rim-bbox midpoint
(+0.005,+0.264), 23 mm of x bias. Result
(`results/fs_c2k1clean_obj_butter_task_k1_v4`): still **0/8**, and the gif
shows the carton standing upright hard against the basket's near wall.
Verdict: xy was no longer the problem.

### v5 — clear the rim in transit  ← FROZEN
Hypothesis (from the numbers, not the gif): the payload **hangs 0.126 m below
the eef** (grasp eef z = 0.127 with the carton bottom on the 0.0013 table).
At the v4 transport height of 0.26 the carton base rode at 0.134 — *below* the
0.144 basket rim — so the lateral approach drove the carton into the near wall
instead of over it. Fix: carry at 0.30 (base 0.174, clear) and descend
vertically to 0.20 inside the basket before opening.
Evidence: probe `results/fs_c2k1clean_obj_butter_task_k1_v5` = **8/8**.

## DECLARATION

- **Frozen version: v5.**
  `packs/c2k1clean_obj_butter_task_k1/program.py` md5
  `082b7614c5ecf1e1941679a59920e15f` ==
  `packs/c2k1clean_obj_butter_task_k1/program_v5.py` (verified on AbakaAI).
- **Selection receipt (full 15 debug seeds 51–65):** **15/15**,
  `results/sel_c2k1clean_obj_butter_task_k1_v5`.
- **Receipt chain:**
  | version | run dir | seeds | result |
  |---|---|---|---|
  | v1 | `fs_..._v1` | 51,53,55,57 | perception probe, no motion |
  | v2 | `fs_..._v2` | 51,53,55,57 | perception probe, no motion |
  | v3 | `fs_..._v3` | 51..65 odd | 0/8 |
  | v4 | `fs_..._v4` | 51..65 odd | 0/8 |
  | v5 | `fs_..._v5` | 51..65 odd | 8/8 |
  | v5 | `sel_..._v5` | 51..65 all | **15/15** |
  (all dirs prefixed `results/…c2k1clean_obj_butter_task_k1…`)
- **PROVENANCE present** in program.py, 14 entries covering every calibrated
  constant: `TABLE_Z_FALLBACK, BAND_LO, BAND_HI, CELL, ORANGE_HSV, MIN_TALL,
  GRASP_DROP, HOVER, CARRY_Z, HANG, RELEASE_Z, RIM_BAND, OPEN_W, HOLD_EFFORT`
  — sourced only from the two named packs and debug-seed measurements.
- **Clean room:** only `tools/fair_run.py` was ever invoked; writes confined to
  `packs/c2k1clean_obj_butter_task_k1/*` and
  `results/*c2k1clean_obj_butter_task_k1*`; no `.done` read; no
  `ground/vqa/sam3/pick_at/place_at` calls; no benchmark asset was opened.

STOP.
