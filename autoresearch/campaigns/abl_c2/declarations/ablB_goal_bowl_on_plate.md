# abl_c2 / ablB_goal_bowl_on_plate — worker ledger (variant B, no-LAWS)

Intent: "put the bowl on the plate". Runner: `tools/fair_run.py` only, GPU 5,
split `debug` (seeds 51-65) only. No law file was read, written or sought; no
cross-episode law was distilled. All dates UTC.

## 2026-08-20 — pack study (no episodes)

Read `packs/ablB_goal_bowl_on_plate/pack.json` (K=3) and looked at 6 keyframe
PNGs (128x128 agentview). What the demos say:

- Gripper closes at `ee_path6` row t=40 in all three demos, at
  (-0.1027,0.0301,0.9252) / (-0.1087,0.0439,0.9179) / (-0.1040,0.0424,0.9199)
  — mean (-0.1051,0.0388,0.9205), spread <= 8 mm.
- Closure gap is 0.008-0.017 m (`gripper_state`), i.e. the demos pinch
  something thin — a rim, not the whole vessel.
- Release keyframes (gripper_cmd flips to -1): (0.0448,0.0121,0.9359) /
  (0.0499,0.0313,0.9280) / (0.0500,0.0396,0.9309) — mean
  (0.0482,0.0277,0.9316). Carry apex ~1.02-1.03.
- Keyframe images: a small achromatic bowl in front of a wine bottle, a wider
  white/pink-rimmed plate nearer the camera, plus a stove board, a blue box and
  a dark cabinet as distractors on a tan wood table.

## v1 — hypothesis: the demo waypoints are directly replayable

Program: open, hover at the demo grasp xy at z=1.02, descend to z=0.9205,
close, lift, transit to the demo release xy, descend to z=0.9316, open, settle,
retreat. Plus a cam_high measurement (vectorised deprojection -> base frame,
table plane from the z-histogram mode, achromatic + raised-above-table mask,
4-connected blobs) logged before and after, purely to instrument the scene.

Evidence — `results/fs_ablB_goal_bowl_on_plate_v1`, seeds 51,53,...,65:
**8/8 benchmark_success**, ~123 sim steps/episode, every close reported
gap 0.008-0.013 m with effort 3.0, and ep51_ok.gif shows the bowl genuinely
resting on the plate at the end.

The instrumentation also gave the numbers v2 needed:
- bowl blob: n 570-593 px, 0.048 m above the table plane (0.9018), 0.105 m
  across, centre (-0.115, +0.003) +- ~0.010 m over the 8 seeds;
- plate blob: n 1205-1295 px, 0.016 m high, 0.135 m across, centre
  (+0.048, -0.014) +- ~0.010 m;
- demo grasp point minus bowl centre = (+0.0095, +0.0359) — a rim pinch;
- demo release point minus plate centre = (+0.0000, +0.0414) — the same
  carried-rim offset to within 9 mm, which cross-checks the first number.

Verdict: works, but the waypoints are fixed constants; the observed layout
jitter (~10 mm here) is unmodelled and an unseen layout could put the fixed
point off the rim.

## v2 — hypothesis: re-anchoring the same waypoints on the perceived object
centres is strictly safer

Change: grasp xy = perceived bowl centre + RIM_OFFSET, release xy = perceived
plate centre + PLACE_OFFSET, with each blob chosen by a size/height/extent gate
and the resulting correction **clamped to 0.05 m of the demo waypoint**, so a
mis-segmentation can do no worse than v1. Added a pinch check: a finger gap
below 0.0035 m after closing (every successful close was 0.008-0.014 m) means
the fingers met each other, so reopen, re-measure and descend 12 mm lower once.

Evidence — probe `results/fs_ablB_goal_bowl_on_plate_v2`, seeds 51,53,...,65:
**8/8**. Anchoring was live on every episode: exactly 1 gate-passing bowl
candidate and 2 plate candidates (the second ~0.30 m away, the stove board),
corrections -0.013..+0.012 m, no clamp hit, no fallback, no retry fired, all
pinches held.

Verdict: keep. Same score, less dependent on the layout being the demo layout.

## DECLARATION

- Frozen program: `packs/ablB_goal_bowl_on_plate/program.py`
  md5 `897961e580a2fc63895526f2fb244fe2` == `program_v2.py` (verified on the
  cluster, identical).
- Selection (ONE formal run, all 15 debug seeds 51..65):
  **15/15** — `results/sel_ablB_goal_bowl_on_plate_v2`.
- Receipt chain:
  - v1 `results/fs_ablB_goal_bowl_on_plate_v1` — 8/8 (seeds 51,53,...,65)
  - v2 `results/fs_ablB_goal_bowl_on_plate_v2` — 8/8 (seeds 51,53,...,65)
  - v2 `results/sel_ablB_goal_bowl_on_plate_v2` — 15/15 (seeds 51..65) [SELECTION]
- Versions used: 2 of the 5 allowed. Debug episodes run in total: **31**
  (8 + 8 + 15). No episode outside seeds 51-65 was ever run.
- `PROVENANCE` present in the frozen program: 18 entries, every one
  `allowed: True` with a pack field, a debug-seed measurement of my own, or
  generic controller/camera mechanics as its source. No `.done` attribute
  appears anywhere in the source (AST-checked locally).

## Clean-room self-audit

Only `packs/ablB_goal_bowl_on_plate/*` and `results/*ablB_goal_bowl_on_plate*`
were written on the cluster. No .bddl/.xml/.hdf5/init-state file was opened
(the bddl path was passed as an opaque `--bddl` argument); nothing under
campaigns/c1, c2 or c2fix, no `packs/c1_*`/`packs/c2_*`, no `results/*c1_*`/
`*c2_*`, no `tools/probe_*`, and no other abl_c2 cell's pack, results or
workspace was read. `tools/fewshot_run.py` was never invoked. Only
`tools/fair_run.py --split debug` was run, always on GPU 5. Prior LIBERO
knowledge in session context was not used: every constant above comes from this
cell's pack or from this cell's own debug-seed logs. No law file was read,
created or written, and no cross-episode law was formalised.
