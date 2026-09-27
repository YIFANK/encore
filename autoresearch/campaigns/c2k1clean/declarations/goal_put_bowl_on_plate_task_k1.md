# c2k1clean / goal_put_bowl_on_plate_task_k1

Intent: **"Put the wine bottle on the plate"** (bddl `put_the_bowl_on_the_plate`,
so the graded predicate is re-authored; the instruction is authoritative).

## Evidence read from the two packs

`packs/c2k1clean_goal_put_bowl_on_plate_task_k1` — language *"put the bowl on
the plate"*. Acts on the TARGET (plate) with the wrong object (bowl).
- close (gripper_cmd -1 -> +1) at t=32, ee = (-0.0943, 0.0335, 0.9237)
- carry, release at t=76, ee = (0.0448, 0.0121, 0.9359), then retreat to z 0.9893
- => in that scene the plate sat at xy ~ (0.045, 0.012) and the bowl was
  released with the eef ~0.936.

`packs/c2k1clean_goal_put_bowl_on_plate_task_mate` — language *"put the wine
bottle on top of the cabinet"*. Handles the OBJECT (wine bottle), wrong target.
- close at t=32, ee = (-0.1823, -0.0652, 1.0291); the arm keeps descending to
  (-0.1982, -0.0658, 1.0223) while closed.
- => the wine bottle is grasped at eef z ~ 1.023-1.029, i.e. on the neck, and
  the demo yaws the wrist (rpy yaw ~ -0.65 rad) to get there.
- release at t=85 on the cabinet top, ee z 1.2515.

Mechanism transferred: **grasp height on the neck (from the mate pack), target
identity and release-xy semantics (from the k1 pack).** Neither xy transfers —
both packs are different scenes.

## Version log

### v1 — perception dump, no motion (`fs_..._v1`, `_v1b`)
Hypothesis: I can do all scene analysis offline by streaming cam_high RGB-D
through `api.log`.
Evidence: works, but `api.log` truncates each message at 2000 chars
(`tools/fair_client.py:280`), so chunks must be <=1900. v1b re-ran with 15...8
seeds and decoded cleanly.
Measured from the 8 dumped debug frames (seeds 51-58):
- table top z = **0.901** (median of the deprojected tabletop).
- scene is near-identical across debug seeds; props jitter a couple of cm.
- props: dark cabinet (top 1.128) at y < -0.15; wooden rack behind it; grey
  stove slab (top 0.932) at (-0.23, +0.21); silver bowl (top 0.952) near
  (-0.09, 0.00); white plate (top ~0.910) near (0.04, -0.01); small blue box
  near (-0.06, +0.135).
- **wine bottle**: dark green/black, at (-0.20, -0.05) on seed 51, top (cork)
  z = 1.059 => 0.158 m tall. Body radius ~0.019 below z 0.99, neck radius
  ~0.011 from z ~1.00 up. This is consistent with the mate pack grasping at
  z 1.023-1.029 (neck).
Verdict: **blocked on one thing** — at reset the arm sits at (-0.208, 0, 1.173),
directly on the camera's line to the bottle, and the two fuse into one
height-band cluster (arm+bottle, z up to 1.371). Perception must run from a
parked pose.

### v2 — park then dump (`fs_..._v2`)
Hypothesis: moving the eef to (-0.10, 0.30, 1.25) before capturing clears the
arm off the bottle in cam_high, so a height+colour rule can isolate the bottle.
Evidence: `park residual 0.007`, eef (-0.093, 0.3047, 1.2595) on every seed;
the prop is fully unoccluded in all 15 dumps. Two further things fell out of
the parked frames:
- the parked gripper's own lowest deprojected point is z 1.253 against an eef
  of 1.2595, so **the eef reference sits ~6 mm above the fingertips** (which is
  why the k1 pack can grasp a bowl rim with the eef *below* the rim top).
- the target disc's floor is z 0.9079 at its centre and its rim rises to
  ~0.920 at radius 0.06-0.075; diameter 0.131-0.143 m on every seed.
Verdict: **good.** Perception rules that survive all 15 debug seeds:
- table = modal deprojected height over x,y in (-0.30, 0.30) -> 0.9006 on 15/15.
- ROI x(-0.32, 0.30) y(-0.14, 0.17).
- tall dark prop = points darker than (35,45,35) more than 0.09 above the
  table, trimmed 4x to the compact blob around the median. The cabinet's right
  face also enters the band but renders at ~(50,50,50), so the darkness cut
  alone separates them; the trim is the belt to that braces.
  Centre: y from the silhouette midpoint (unbiased), x from
  (xmin + xmax - r)/2 (the visible arc's far edge). Top = max z of the blob.
  -> obj (-0.21..-0.19, -0.065..-0.037, 1.047..1.050), r 0.010-0.012 on 15/15.
- flat grey disc = R-B<16, R>110, within 0.026 of the table; the centre is the
  5 mm cell whose 0.060 m disc collects the most such points (the bowl is the
  only other grey thing that low, and it is 14 cm away and half the area, so it
  never out-votes). -> dst (0.02..0.06, -0.042..-0.003, floor 0.908) on 15/15,
  diameter 0.131-0.143 in both axes.

### v3 — park, perceive, pick by the neck, place (`fs_..._v3`, `sel_..._v3`)
Hypothesis: the mate pack's grasp height is the whole mechanism. It grasps the
wine bottle at eef z 1.0223-1.0291; the same prop's dark top measures
1.047-1.050 here, so **grasp = top - 0.025** puts the fingertips on the neck
(r 0.011) rather than on the shoulder (r 0.019). Since the prop's base starts
on the table, `hang = grasp_z - table` is the eef's height above the held base
with no fingertip calibration needed, and `release = disc_floor + hang` lands
the base on the disc. Carry at eef 1.20 (held base ~1.08) clears the bowl rim
at 0.952, the only obstacle on the line.
Evidence:
- probe (odd seeds 51-65): **8/8**, `results/fs_..._v3`.
- formal selection, all 15 debug seeds: **15/15**, `results/sel_..._v3`.
  Grip gap 0.015 and effort 3.0 from close through release on every seed;
  ~200 sim steps per episode.
- seed 52 is the one ragged episode: the place descent stalled 40 mm high
  (residual 0.0403) with the disc offset to y -0.035, so the prop was dropped
  rather than set down — and still scored.
Verdict: **selected.**

### margin probes (exploratory, not candidates)
Because 15/15 says nothing about how much room there is, two displaced copies
were run on seeds 51-58:
- `program_x_aim.py`, grasp xy displaced +14 mm in both axes (20 mm aim
  error): **7/8** (`fs_..._xaim`).
- `program_x_drop.py`, release raised 30 mm (pure drop): **8/8**
  (`fs_..._xdrop`).
The two independent estimates of the prop's centre (neck-arc vs body-arc)
agreed to ~1 mm, so v3 is aiming well inside a margin that only starts to
break at ~20 mm, and the place is insensitive to a 30 mm height error.

## DECLARATION

- Frozen version: **v3**.
  `packs/c2k1clean_goal_put_bowl_on_plate_task_k1/program.py`
  md5 `f76be29c71be262342787613e20b3322` ==
  `program_v3.py` md5 `f76be29c71be262342787613e20b3322`.
- Selection receipt (full 15 debug seeds, one formal run):
  **15/15** — `results/sel_c2k1clean_goal_put_bowl_on_plate_task_k1_v3`
  (seeds 51-65, every `benchmark_success: true`).
- Receipt chain:
  | version | what | seeds | result |
  |---|---|---|---|
  | v1 / v1b | perception dump, no motion | 51-57 / 51-58 | 0/4, 0/8 (by construction) |
  | v2 | park + dump | 51-65 | 0/15 (by construction); perception rules fixed |
  | v3 | park, perceive, pick by the neck, place | 51,53,..,65 | **8/8** |
  | v3 (formal) | same program | 51-65 | **15/15** |
  | x_aim | v3 + 20 mm grasp aim error | 51-58 | 7/8 (margin probe) |
  | x_drop | v3 + 30 mm high release | 51-58 | 8/8 (margin probe) |
- PROVENANCE: present in `program.py` as a top-level literal dict, 10 entries,
  all sourced to either `packs/c2k1clean_goal_put_bowl_on_plate_task_mate`
  (the grasp height) or debug-seed 51-65 observations. `scan_program(..., "eval")`
  accepts the file; no `.done` read anywhere in the program.
- Clean room: the only cluster writes were
  `packs/c2k1clean_goal_put_bowl_on_plate_task_k1/*` and
  `results/*c2k1clean_goal_put_bowl_on_plate_task_k1*`. Seeds 1-50 were never
  touched; `--split eval` was never run. No benchmark asset was opened.

STOP.
