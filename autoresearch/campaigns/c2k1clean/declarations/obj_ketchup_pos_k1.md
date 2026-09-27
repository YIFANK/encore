# c2k1clean / obj_ketchup_pos_k1 — worker notes

Task: "Pick up the ketchup and place it in the basket."  K=1 pack,
`_pos` perturbation (object placements move per seed), debug seeds 51-65.

## Pack reading (the only task-specific input)

`pack.json` demo0, 248 steps, stride 10.  Gripper command channel:

| t | cmd | ee (x,y,z) | note |
|---|-----|------------|------|
| 0 | open | (-0.156, 0.004, 0.243) | start |
| 53 | CLOSE | (-0.087, -0.242, 0.093) | **failed grasp** — at t=69 the finger state is 0.0013/-0.0014, i.e. fully closed on air |
| 69 | open | (-0.105, -0.250, 0.103) | release, retreat |
| 106 | open | (-0.164, -0.259, 0.035) | re-approach, traverses at z≈0.037 |
| 136 | CLOSE | (-0.142, -0.220, 0.013) | **grasp that held** |
| 229 | open | (0.092, 0.223, 0.177) | release over the basket |
| 247 | — | (0.083, 0.244, 0.248) | retreat |

Two facts the demo hands me for free:
* the grasp that works is at **eef z = 0.0133**; a close at z = 0.0666 catches
  nothing, so the useful grasp height is very low (fingertips sit well below
  the eef site).
* the release over the basket is at **eef z = 0.1768**, carried at z ≈ 0.29.

Demo xy is a decoy under `_pos` — the object placements are re-randomised per
seed — so the target must be found by perception every episode.

## Which object is the ketchup

Differencing keyframe `demo0_t0000.png` against `demo0_t0229.png` (the frame
after the drop) leaves exactly one table blob changed: pixels x∈[28,40],
y∈[48,67] in the 128×128 agentview.  That blob is the object the demo removed,
so it is the ketchup.

Appearance of that blob (from the pack keyframe):
* a **grey / achromatic cap** on top (RGB ≈ 45-95 with R≈G≈B),
* a **strongly red-orange body** below it (e.g. 84,31,9 / 90,34,6 / 70,20,11).

The scene holds six table objects plus the basket.  The confusable one is a
second, darker **brown** bottle (mean 62,35,20) whose cap is dark red-brown,
not grey.  So the discriminator to test is *grey cap over a red body*.

## Version log

### v1 — perception probe + first-cut pick  (seeds 51,53,55)
Hypothesis: a depth-fitted table plane plus connected-component blobs gives
per-object height / footprint / colour bands good enough to name the ketchup;
the pack's grasp and release heights transfer verbatim.
Provisional target rule for v1 only: most red-dominant body among bottle-sized
blobs (expected to be ambiguous — v1 exists to produce measurements).
Evidence (results/fs_c2k1clean_obj_ketchup_pos_k1_v1, seeds 51/53/55): the run
crashed at `api.move_path` — that primitive does not exist on the LIBERO
backend — but the perception ran first and returned the numbers I needed:
table plane z = 0.0011 on all three seeds; camera at (0.897, 0, 0.65) looking
32° below horizontal (so a blob's y-extent is its true width while its
x-extent is only the front sliver); six table blobs + the robot (h = 0.475).
The provisional "reddest body" rule picked the *dark brown* bottle, and the
descent to the pack's z = 0.0133 stalled at z = 0.0422/0.0422/0.0419 over a
blob whose top is 0.113 — a clean, thrice-repeated measurement that the
colliding part of the gripper sits **0.0709 m above the eef site**.
That also explains the pack: the demo's own first close (z = 0.093) grabbed
nothing, then it swept through the bottle at z = 0.037 — which knocks a
0.147-tall bottle over — and grasped the fallen bottle at z = 0.0133.  So the
pack's grasp height is NOT transferable to a standing bottle; the reachable
band is only the top 0.071 m.
Verdict: harness fix needed (chain `move`), rule wrong, geometry measured.

### v2 — measured geometry, cap/body colour rule (seeds 51…65 odd)
Hypothesis: rank blobs by `body_redness − cap_achromaticity` to name the
ketchup; grasp 0.050 m below the measured top; aim x at `x98 − radius`.
Evidence (fs_…_v2): **0/8**, but the pick itself worked on every seed —
close w = 0.0335, effort 3.00, still 3.00 after the lift and the whole carry.
The identification was right too (the grey-cap blob won by a margin of 47).
The failure is entirely at the drop: the descent over the basket stalled with
residual 0.078 at z = 0.2533, which puts the carried bottle's base at 0.1418 —
the basket rim (0.142) to the millimetre.  The blob's median x (0.075) sits on
the *near wall*, because that wall contributes most of the visible pixels.
Verdict: grasp solved, drop point biased onto the rim.

### v3 — basket height map (seeds 51,57)
Hypothesis: a 1 cm max-z grid over the basket will show the rim ring and the
hole, and the ring's centroid is the opening.
Evidence (fs_…_v3): **2/2**.  The grid shows the rim as a U of 0.14 cells with
the interior returning no depth (occluded by the near wall), the near wall at
x ≈ 0.076–0.086 and the side walls at y ≈ 0.177–0.187 / 0.327–0.347.  Aiming
at the ring centroid put the drop at (0.042, 0.259), residual 0.0117.
Also switched the target's x from `x98 − radius` to the top-band mean, which
is the cap's own centre.
Verdict: right mechanism, but the grid window clipped the far wall, so the
centroid still leaned ~0.034 m toward the near wall — about one bottle radius
of margin.  Replace the grid with the rim ring's own bounding box.

### v4 — rim bounding box + sensor-verified grasp ladder (seeds 51…65 odd, then all 15)
Hypothesis: the whole rim ring is visible from a 32° elevation view, so the
midpoint of the rim points' bounding box is the opening centre; and a grasp
ladder retried until `effort == 3.0` removes the single-attempt risk.
Evidence: fs_…_v4 **8/8**; sel_c2k1clean_obj_ketchup_pos_k1_v4 **15/15**.
The rim bbox gives (0.008, 0.255) on seed 51 versus v3's (0.042, 0.259) — the
far wall is now included and the drop lands in the middle of the opening.
Every seed took exactly one grasp attempt (`tries=1`, `held=True`).
Verdict: solved.

### v5 — frozen version (all 15)
Hypothesis: v4's one remaining hard failure mode is structural, not
statistical — if the ketchup ends up touching a neighbour the two merge into
one blob, no blob passes the size gate, and v4 returns without acting.  Rank
the whole scene in that case instead, and exclude the chosen basket from the
target ranking so the fallback cannot select it.
Evidence: sel_c2k1clean_obj_ketchup_pos_k1_v5 **15/15**.  No regression; the
fallback never fired on the debug split (it is a guard for eval layouts the
debug seeds do not produce).
Verdict: frozen.

## Note on what the debug split does and does not exercise

Across all 15 debug seeds the ketchup itself lands at the *same* place
(tx = −0.144, ty = 0.059 every time); the `_pos` jitter that is visible moves
the basket (x ∈ [−0.012, 0.018], y ∈ [0.244, 0.272]), the red box and the
brown bottle by 1–2 cm.  So the debug split tests the basket estimator across
a ~3 cm spread but tests the target estimator at a single placement.  Nothing
in the program is tied to that placement — target xy, grasp height, drop point
and the held-check all come from the current frame — but the honest statement
is that its tolerance to a *large* ketchup displacement is argued from the
mechanism, not measured.  Two things bound the risk: the jaws open to 0.080 m
against a 0.033 m cap, so the grasp tolerates ≈ ±0.023 m of aim error (v2 and
v4 aimed 0.013 m apart and both held), and a failed close is detected by
`effort`/`width` and retried after re-perceiving.

---

# DECLARATION

* **Frozen version:** `packs/c2k1clean_obj_ketchup_pos_k1/program.py`,
  md5 `6f61787fa93ba9d2fc944edc5ceb54e4` == `program_v5.py` (same md5, verified
  on the cluster).
* **Selection receipt (full 15 debug seeds, 51–65):** **15/15**
  (`results/sel_c2k1clean_obj_ketchup_pos_k1_v5`, every episode
  `"benchmark_success": true`).
* **Receipt chain:**
  | version | seeds | result | dir |
  |---|---|---|---|
  | v1 | 51,53,55 | 0/3 (crashed on `api.move_path`; perception logged) | `results/fs_c2k1clean_obj_ketchup_pos_k1_v1` |
  | v2 | 51…65 odd | 0/8 (grasp held every seed; drop landed on the rim) | `results/fs_c2k1clean_obj_ketchup_pos_k1_v2` |
  | v3 | 51,57 | 2/2 | `results/fs_c2k1clean_obj_ketchup_pos_k1_v3` |
  | v4 | 51…65 odd | 8/8 | `results/fs_c2k1clean_obj_ketchup_pos_k1_v4` |
  | v4 | all 15 | 15/15 | `results/sel_c2k1clean_obj_ketchup_pos_k1_v4` |
  | **v5** | **all 15** | **15/15** | `results/sel_c2k1clean_obj_ketchup_pos_k1_v5` |
* **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  every calibrated constant (`CARRY_Z`, `DROP_CLEARANCE`, `PALM_OFFSET`,
  `GRASP_BELOW_TOP`, `GRASP_LADDER`, `HELD_MIN_W`, `RIM_BAND`, `CAP_BAND`,
  `TABLE_MIN_H`, `ARM_MAX_H`, `TARGET_H_RANGE`, `WORKSPACE_BOX`).  Sources are
  the pack's own keyframes/`ee_path6` and debug-seed depth/gripper
  measurements only.
* **Clean room:** the only cluster writes were
  `packs/c2k1clean_obj_ketchup_pos_k1/program*.py` and
  `results/*c2k1clean_obj_ketchup_pos_k1*`; `api.done` is never read; seeds
  1–50 were never touched.

STOP.
