# abl_c2 / ablB_goal_turn_on_stove — worker ledger (variant B, no-LAWS)

Intent: "turn on the stove". Runner: tools/fair_run.py only. GPU 4.
All dates UTC, 2026-08-20.

## 2026-08-20 — pack study (the only task-specific input)

`packs/ablB_goal_turn_on_stove/pack.json`: K=3 demos, lengths 80/96/89, stride 10,
plus 9 keyframe PNGs (128x128 agentview). What the pack says:

- Every demo starts at the same home pose, tool straight down (rotvec magnitude
  ~pi, tool z-axis down), gripper OPEN (gripper_cmd -1, finger gap 0.072).
- The path is a single arc: out in +y at height (-0.21,0.07,1.15), then
  (-0.27,0.16,1.12), (-0.31,0.19,1.06)/(-0.37,0.21,0.98), arriving at
  (-0.4328,0.2209,0.9279) / (-0.4258,0.2194,0.9306) / (-0.3777,0.2093,0.9633).
- At exactly that arrival pose `gripper_cmd` flips to +1 (close) in all three
  demos. Final finger gaps 0.031 / 0.029 / 0.035 — a real closure on something
  ~3 cm across, not a full close on air.
- After the close the xyz stays put (z settles to 0.9308/0.9294/0.9292) and only
  the ORIENTATION changes: converting the final keyframe rotvecs to matrices,
  each is Rz(theta) applied to the straight-down tool basis, with
  theta = 57.7 / 45.3 / 70.8 degrees about world +z.
- Keyframe images: a white flat stove plate carrying a dark burner disc, with a
  black knob post standing on its far (more -x) side. In the t=0 frames the disc
  is grey; in every final keyframe the disc is RED while the gripper is closed on
  that post and yawed. So the demonstrated mechanism is: straddle the knob post
  with the open gripper, close, and yaw about world +z in place.

Derived plan: approach along the demo arc to the mean close-pose
xy = (-0.4121, 0.2165) at z = 0.9295, grip(0.0), then command
move(hold, rotation=Rz(theta) @ R_current) for theta = 25, 50, 70, 90 deg.
Yaw is applied on top of the measured current tool rotation rather than on an
idealised Rx(pi), because the demo tool basis carries a few degrees of tilt.
Step budget: 5 short position moves + a close + 4 yaw moves, all with small
`seconds`, well inside the 1000-step horizon.

## v1 — hypothesis / evidence / verdict

**Hypothesis.** The knob pose barely moves across episodes (the three demo
close-poses span 5.5 cm in x, 1.2 cm in y, and the three t=0 keyframes show the
stove in visually the same spot), so the pack-mean knob xy plus the demo grasp z
is a good enough target, and the whole task reduces to close-then-yaw. v1 also
carries a read-only cam_high perception pass (whole-image deprojection, table-z
mode, bright-plate stats, dark-blob connected components) that only LOGS what it
sees, so that if the prior missed I would have the measured knob geometry per
seed for v2 without spending a version on a pure diagnostic.

**Evidence.** Probe `results/fs_ablB_goal_turn_on_stove_v1` on seeds
51,53,55,57,59,61,63,65 -> **8/8** benchmark_success, no errors.
From program_ep51.log: the final approach lands at eef (-0.4041,0.2168,0.9356)
(residual 0.0101), the close reports width 0.0338 with effort 3.0 (fingers
stopped on the post, not shut on air), and the yaw reaches 24.5 deg on the first
command and 44.5 deg on the second — at which point the episode ends (sim_steps
136-138 of a 1000 horizon; the third and fourth yaw commands are no-ops because
the environment has already stopped stepping). Two seeds ended even earlier
(96/97 steps). The logged perception pass was not needed and, as written, its
dark-blob heuristic keys onto the arm rather than the knob — it is diagnostic
only and does not influence any command.

**Verdict.** Accept v1 as the argmax; no v2 needed. Selection run
`results/sel_ablB_goal_turn_on_stove_v1` on all 15 debug seeds 51..65 ->
**15/15**, no failures, no errors.

## DECLARATION

- Frozen program: `packs/ablB_goal_turn_on_stove/program.py`
  md5 `147b7d9ddef86a27e46e17a1a67f5665`, identical to
  `packs/ablB_goal_turn_on_stove/program_v1.py` (same md5).
- Selection receipt: **15/15** on debug seeds 51..65,
  dir `results/sel_ablB_goal_turn_on_stove_v1`.
- Per-version receipt chain:
  - v1 probe (51,53,55,57,59,61,63,65): 8/8 — `results/fs_ablB_goal_turn_on_stove_v1`
  - v1 selection (51..65): 15/15 — `results/sel_ablB_goal_turn_on_stove_v1`
  - v2..v5: not used (1 version of the 5-version cap consumed).
- Total debug episodes executed in this cell: **23** (8 probe + 15 selection).
- PROVENANCE: present as a top-level literal dict in program.py, covering
  KNOB_XY, GRASP_Z, APPROACH_WAYPOINTS, YAW_TARGETS_DEG, TOOL_BASIS,
  CLOSE_WIDTH_M, PERCEPTION_ROI — every one sourced to a pack.json field or to
  generic controller/camera mechanics.
- Splits: seeds 1-50 never touched; `--split debug` only; every run via
  tools/fair_run.py.

STOP.
