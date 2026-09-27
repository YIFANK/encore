# c2k1clean / spa_bowl_next_to_plate_pos_k1

Intent: "Pick up the black bowl next to the plate and place it on the plate."
Runner: `tools/fair_run.py` only (FAIR_PROTOCOL v1.1.1). No shared note file.

## What the scene is (derived, not assumed)

From `cam_high` RGB-D on debug seeds 51/53/55/57 (all four layouts agree to
within ~2 cm of jitter):

| thing | rim-fit centre (seed 51) | rim-fit r | top above table | mean lum |
|---|---|---|---|---|
| dark cabinet | (0.09, -0.20) | - | 0.215 | 58 |
| small vessel | (-0.206, 0.192) | 0.041 | 0.042 | 117 |
| large vessel A | (-0.171, 0.314) | 0.053 | 0.050 | 100 |
| large vessel B | (0.008, 0.309) | 0.053 | 0.050 | 107 |
| bright flat disc | (0.072, 0.040) | 0.057 | 0.019 | 141-154 |
| flat brown box | (0.068, 0.200) | 0.031 | 0.019 | 68 |

Table top z = 0.9010 m (modal world z of the dominant horizontal surface).

## How the pack was read

The pack's keyframes are a 128x128 render of the same fixed `cam_high`. Using
the intrinsics/extrinsics measured on a debug seed, the pack's bright disc
deprojects to (0.070, 0.189) and its brown box to (0.071, 0.028) — **exactly
the seed positions with the two swapped.** That is the whole `_pos`
perturbation here: the disc and the box trade places, the three vessels do not
move. So "next to the plate" in the pack's scene and in the eval scene are
different geometric facts, and the pack's demo xy is not directly transferable.

Against the rim fits, the pack's grasp keyframe (t=53, ee=(0.0084, 0.254,
0.9226)) is `large vessel B centre + (0.000, -0.055)` at 0.0284 below its rim
top: a **rim pinch at the -y extreme, jaws along y, wrist straight down**.
Adding the same +y rim offset back to the release keyframe (t=109,
ee=(0.0584, 0.1525, 0.9355)) lands the vessel at (0.058, 0.208) — the pack
scene's disc centre to within 2 cm. The model closes.

## Version log

**v1 — diagnostic + naive centroid grasp.** Hypothesis: a top-down grasp at the
cluster centroid. Evidence: probe 51/53/55/57 = 0/4. The candidate filter
(dark + raised) selected the *brown box*, and the close returned gap 0.0024
(nothing held). Verdict: refuted; but it produced the cluster inventory above.
Receipt: `results/fs_c2k1clean_spa_bowl_next_to_plate_pos_k1_v1`.

**v2 — rim pinch, aimed off the cluster MEDIAN.** Hypothesis: pinch the rim at
the pack's measured offset from the vessel's median xy. Evidence: 51/53/55/57
= 0/4, but every episode grasped on the first try (gap 0.0077-0.0100, effort
3.00), carried, and released; the post-hoc re-capture showed a merged
vessel-on-disc blob 0.067 m tall, and the last gif frame shows the bowl
visibly sitting inside the plate rim. Verdict: the mechanism worked and the
benchmark still said false — so either the aim or the target instance was
wrong. Receipt: `results/fs_c2k1clean_spa_bowl_next_to_plate_pos_k1_v2`.

Diagnosis: the cluster median is biased ~0.03 m by the oblique view (median
(-0.017, 0.321) vs Kasa rim fit (0.008, 0.309)). Aiming the *release* off the
median put the vessel 0.024 m off the disc centre. Replacing every centre with
a least-squares circle through the cluster's top band removes the bias.

**v3 — Kasa rim fits everywhere + explicit target hypothesis.** Three variants
run in parallel on seeds 51,53 to settle which vessel the predicate names:

| variant | target | result |
|---|---|---|
| `program_v3.py` (`near`) | large vessel nearest the disc | **2/2** |
| `program_v3far.py` | the other large vessel | 0/2 |
| `program_v3small.py` | the small vessel | 0/2 |

Verdict: the target is the large vessel nearest the disc — the literal reading
of the instruction — and v2 failed on **aim**, not on instance. Receipts:
`results/fs_c2k1clean_spa_bowl_next_to_plate_pos_k1_v3near` /`_v3far` /
`_v3small`.

Probe (8 seeds 51,53,...,65): **8/8** —
`results/fs_c2k1clean_spa_bowl_next_to_plate_pos_k1_v3probe8`. Every episode
grasps on ladder rung 0 (gap ~0.009, effort 3.00) and finishes in ~130 sim
steps.

## DECLARATION

- **Frozen version: v3.** `packs/c2k1clean_spa_bowl_next_to_plate_pos_k1/program.py`
  md5 `06e4056d09fc228a02a6b906d888b823` == `program_v3.py` (same md5).
- **Selection receipt (full 15 debug seeds 51-65): 15/15**, dir
  `results/sel_c2k1clean_spa_bowl_next_to_plate_pos_k1_v3` (episodes 51-65 all
  `"benchmark_success": true`, 127-134 sim steps each).
- **Per-version receipt chain:** v1 0/4 (`fs_..._v1`) -> v2 0/4 (`fs_..._v2`)
  -> v3 2/2 near, 0/2 far, 0/2 small (`fs_..._v3near|v3far|v3small`) -> v3 8/8
  (`fs_..._v3probe8`) -> v3 15/15 (`sel_..._v3`).
- **PROVENANCE present** in program.py: `RIM_DROP`, `RIM_EXCESS`, `PINCH_DIR`,
  `PLACE_RISE`, `CARRY_Z`, `HELD_GAP`, `LARGE_R_CUT`, `VESSEL_H_BAND`,
  `DISC_LUM_MIN`, `TABLE_Z`, `WORKSPACE_BOX` — every one sourced to a pack
  field or a debug-seed depth/RGB measurement.
- Archived: `program_v1.py`, `program_v2.py`, `program_v3.py` (frozen),
  plus the two refuted target-hypothesis variants `program_v3far.py`,
  `program_v3small.py`.
- No `--split eval` run; seeds 1-50 never touched.
