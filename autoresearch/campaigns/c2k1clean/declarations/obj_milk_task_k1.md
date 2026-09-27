# c2k1clean / obj_milk_task_k1 — notes

Intent: **"Pick the butter and place it in the basket"** (FAIR_PROTOCOL v1.1.1,
no shared note file). Runner: `tools/fair_run.py` only.

## The problem

Neither pack demonstrates the intent. The scene (perceived on debug seeds,
never read from any asset file) holds 6 objects + a basket:

| object | top z | y-extent | r−b |
|---|---|---|---|
| milk carton | 0.142 | 0.0534 | +30.7 |
| **orange box** | **0.019** | **0.0387** | **+52.9** |
| blue box | 0.020 | 0.0415 | −24.1 |
| brown box | 0.030 | 0.0479 | +20.8 |
| orange-juice carton | 0.143 | 0.0530 | +60.8 |
| can | 0.081 | 0.0655 | +18.6 |
| basket | 0.144 | 0.170 | +5.8 |

So the whole cell turns on one question: **which of these is the butter?**

## Version chain (hypothesis → evidence → verdict)

**v1 — perception dump, no motion.** `results/fs_…_v1` (8 eps, 0/8 by
construction). Logged cam_high RGB+depth (zlib+base64 through `api.log`),
intrinsics, extrinsics, EEF, gripper. Verdict: cam_high is a *fixed* camera
(base-frame pose `t=(0.897, 0, 0.650)`, 32° down-look), table plane at
z≈0.0017, and the object layout is near-deterministic across seeds 51–65.
This dump made all later analysis offline and cheap.

*Validation of the projection:* reprojecting the k1 pack's grasp keyframe
(`ee=(-0.1379,-0.2552,0.1016)`) through that camera lands exactly on the tall
milk carton in the k1 pack's own keyframe image. The camera model is sound,
so pack keyframes and my frames live in one comparable frame.

**v2 — pick the brown box.** Hypothesis: butter = flat **brown** box, from
(a) a subpixel silhouette-width match against the mate pack's butter
(0.0574 m vs brown 0.0582, blue 0.0530, orange 0.0509) and (b) warm-brown hue.
Result: `RuntimeError: move_path` — **`api.move_path` is not implemented on the
LIBERO backend** (robosuite-only op; the cell brief's API list omits it and I
should have stuck to that list). Grasp itself succeeded on all 8 seeds
(effort 3.0, held width 0.0458). `results/fs_…_v2`, 0/8.

**v3 — same hypothesis, transport as a chain of `api.move()` waypoints.**
`results/fs_…_v3`, **0/8**. Crucially the logs show the *mechanics were
perfect*: grasped first try, effort 3.0 held continuously through the carry,
released over the perceived basket centre, and the post-hoc re-perception
found `still_at_start: 0, n_objs: 6` — the brown box really was deposited in
the basket. **Verdict: the brown box is not the butter; silhouette width was
the wrong cue.**

**v4 — three one-variable probes (4 seeds each).** A sharper cue: in *both*
packs the keyframe where `gripper_cmd` flips back to −1 is the release over
the basket, and its `gripper_state` is still spanning the object.

- k1 pack (milk) t=142 → finger gap **0.0540**; the milk carton measures
  **0.0534** here → the cue is calibrated to **0.6 mm** against this very scene.
- mate pack (butter) t=150 → finger gap **0.0395** → orange (0.0387) or
  blue (0.0415), *not* brown (0.0479).

| probe | target | result |
|---|---|---|
| `fs_…_v4orange` | orange box | **4/4** |
| `fs_…_v4blue` | blue box | 0/4 |
| `fs_…_v4milk` | milk carton | 0/4 |

**Verdict: the butter is the flat orange box.** Two independent confirmations:
grasping it reports a held width of **0.0389** (0.6 mm from the demo's 0.0395,
same residual as the milk calibration), and the milk probe failing 0/4 shows
the benchmark predicate really is the re-authored *butter* predicate, not the
milk one the bddl filename advertises.

**v5 — final.** Selection rule generalised from the winning probe: among
non-basket clusters (preferring the flat ones, since the mate pack closed at
z=0.0090, a height only a low object reaches) pick the argmin of
`((ye − 0.0395)/0.006)² + ((r−b − 52.9)/20)²`. Offline over all 8 dumped
seeds the winner scores 0.01–0.05 with the runner-up at 4.55 — a ~150×
margin. Motion: open, hover 0.145 → 0.045 → grasp at z=0.0090 (mate pack),
close, verify effort/width, lift to 0.270, two straight-line carry waypoints,
descend to 0.1909 (mate pack), release, retreat, re-perceive.

## DECLARATION

- **Frozen version: `program_v5.py`.**
  `md5(packs/c2k1clean_obj_milk_task_k1/program.py) ==
   md5(packs/c2k1clean_obj_milk_task_k1/program_v5.py) ==
   048d81a56edf6d72fb23d62b2fdd0596`
- **Selection receipt: 15/15** on the full 15-seed debug band
  (51–65, every seed, no exceptions), dir
  `results/sel_c2k1clean_obj_milk_task_k1_v5`. No program errors; 190–194
  sim steps per episode.
- **Per-version receipt chain:**
  | version | dir | seeds | result |
  |---|---|---|---|
  | v1 perception dump | `results/fs_c2k1clean_obj_milk_task_k1_v1` | 8 | 0/8 (no motion by design) |
  | v2 brown box | `results/fs_c2k1clean_obj_milk_task_k1_v2` | 8 | 0/8 (`move_path` unsupported) |
  | v3 brown box | `results/fs_c2k1clean_obj_milk_task_k1_v3` | 8 | 0/8 (placed, wrong object) |
  | v4 orange box | `results/fs_c2k1clean_obj_milk_task_k1_v4orange` | 4 | **4/4** |
  | v4 blue box | `results/fs_c2k1clean_obj_milk_task_k1_v4blue` | 4 | 0/4 |
  | v4 milk carton | `results/fs_c2k1clean_obj_milk_task_k1_v4milk` | 4 | 0/4 |
  | **v5 final** | `results/sel_c2k1clean_obj_milk_task_k1_v5` | **15** | **15/15** |
- **PROVENANCE present**: 21 entries, every one `allowed: True` with a source
  that is either a named pack field, a debug-seed measurement, or generic
  controller/camera mechanics. No `.done` attribute read anywhere (AST-checked);
  no forbidden tokens (regex-checked).

## Clean-room statement

Every constant was re-derived from the two named packs' `pack.json` +
`keyframes/`, and from my own debug-seed RGB-D / EEF / gripper observations.
No `.bddl`/`.xml`/`.hdf5`/init-state file was opened; no other campaign's
artifacts were read; `tools/fewshot_run.py` was never invoked; seeds 1–50 were
never touched. No LIBERO prior knowledge was used — object heights, widths,
colours, the table plane, the grasp/release heights and the camera model were
all measured in-cell.
