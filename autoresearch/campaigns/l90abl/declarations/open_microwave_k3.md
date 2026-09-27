# l90abl / open_microwave_k3 — NOTES

Intent: "open the microwave". FAIR_PROTOCOL v1.1.1. Runner: `tools/fair_run.py` only.
Debug/learn seeds 51-65; eval seeds 1-50 never touched.

## Pack facts (packs/l90abl_open_microwave_k3/pack.json)
- K=3 demos, 131/150/143 steps, stride 10, 2 keyframes each (t0 + final).
- `gripper_cmd = -1.0` at both keyframes and column 6 of every logged action in all
  three demos: **the demonstrator never closes the gripper.**
- `ee_path6[:,3:6]` is a **rotation vector (Rodrigues)**, not euler rpy. Derived here:
  rvec(3.1429,-0.0149,-0.1526) -> [[.995,-.010,-.097],[-.009,-1,.005],[-.097,-.005,-.995]],
  which matches the measured `api.tool_rotation()` at home
  ([[.998,0,-.057],[0,-1,0],[-.057,0,-.998]]); read as rpy it produces a -0.152
  off-diagonal that the measurement contradicts.
- Trajectory shape shared by all three demos: descend z 1.17 -> ~1.02; a sustained
  action[0] ~ +0.7..0.9 (+x) push whose realised displacement is far smaller than
  commanded (contact-resisted); the contact point slides +y; a retract; a second +x
  push ending at (0.171,0.071) / (0.155,0.029) / (0.125,0.098), z ~ 1.03.
- Those three endpoints are the load-bearing pack signal: they lie on the 90-degree
  arc of radius |handle - hinge| about the box's near-x front corner. That is what
  fixes which side the door is hinged on and how far it must be swept.

## Scene geometry (my own cam_high deprojections, debug seeds 51-65)
- Camera: f=618.04, c=(256,256); t_base_cam translation (0.659, 0, 1.610);
  **image-right = base +y, image-down = base +x**, optical axis along -x and down.
- Table plane z0 = 0.9012 on every debug seed.
- Microwave lid slab z ~ 1.121-1.144, x in [-0.180,+0.162], y in [-0.372,-0.146].
  Placement varies only ~ +-0.012 m across seeds 51-65.
- Closed door plane y_face = -0.146: vertical slices at x=-0.16,-0.10,0.00,0.08,0.15
  all return that same plane. The face is FLUSH — there is nothing to push on.
  (x in [-0.03,0.11], z 0.95..1.05 returns no depth at all: that is the glass window.)
- Handle: a vertical bar standing 0.042 proud of the door plane, x ~ -0.06,
  x-thickness 0.0118, spanning z 0.951..1.071, with standoffs top and bottom.
- Fingertip offset: pressing straight down on bare table stalls at
  api.eef()[2] = z0 + 0.008..0.012.
- Colours are randomised (debug microwave is blue, pack keyframes black): never a cue.

## Version log (hypothesis -> evidence -> verdict)
- **v0** capture-only probe, seeds 51,55. 0/2 (expected). Gave the camera convention,
  the table plane and the first microwave bbox. Confirmed writes under
  `packs/l90abl_open_microwave_k3/*` work, so clouds can be dumped for offline work.
- **v1** literal replay of demo0's 14 `ee_path6` waypoints, seeds 51..65 odd. **0/8.**
  Every episode identical, every residual ~0.01 (nothing ever resisted), and the GIF
  shows the hand waving in free air to the +y side of the microwave. The demo path
  never comes within 0.10 m of the door plane. Verdict: the pack's absolute ee_path is
  not directly executable here; only its endpoint is usable.
- **v2** cloud dump on all 15 debug seeds (no motion). Used offline to build a
  perception front end that is robust to the robot arm: scan slab heights 0.10..0.34 m
  above the table, keep the largest 4-connected top-down component by area*fill.
  Stable lid + handle on 15/15 seeds. (v1's naive z-threshold had been measuring the
  arm, not the microwave.)
- **v3** fingertip-offset + grip-the-bar probe. Perception aborted (the z-band still
  admitted the arm) and the table-press point (0.30,0.25) proved unreachable.
  Salvaged: fingertip offset ~0.012.
- **v4** one open finger into the slot behind the bar, then pull +y. **0/2.** The
  staging move commanded (-0.183,-0.094,1.021) but the tool arrived at
  (0.087,-0.100,1.116): x and z never moved. First sign that something stops the tool
  well short of the door.
- **v5** same hook, with a segmented `goto` to defeat starved moves. **0/3.** Now the
  stall is explicit: commanded y=-0.132, stalled at y=-0.044, and the arc that followed
  swept cleanly through empty air 0.09 in front of the door. Also established the
  **1000 sim-step episode cap** (all three episodes ran to exactly 1000).
- **v6** reach-envelope probe, 1 seed, no manipulation. The decisive measurement:
  | config | wrist | stalls at y |
  |---|---|---|
  | A x=-0.16 | straight down | -0.044 |
  | B x=-0.06 | straight down | -0.012 |
  | C x=-0.06 | yaw 90 | **-0.082** |
  | D x=+0.08 | straight down | -0.044 |
  Verdict: not a reach limit and not the door — the *hand body* is long in its
  jaw-opening axis and fouls the box. Yawing the wrist 90 deg turns the hand edge-on
  and buys ~0.04 m; and because the jaws then open along base x, they can straddle a
  bar that is only 0.0118 thick in x.
- **v7** descend from directly above with the jaws astride the bar, close, then carry
  the bar round the hinge arc yawing the wrist with it. Seeds 51,53,55: **3/3.**
  Clamped width reads 0.0227 with effort 3.0 (the bar) and holds through the whole arc.
  Full 15 debug seeds: **15/15** (`results/sel_l90abl_open_microwave_k3_v7`).
- **v8 = FROZEN.** v7 with the required `PROVENANCE` filled in and the debug-dump
  scaffolding removed; behaviour otherwise unchanged. Full 15 debug seeds: **15/15.**

## Mechanism (what actually opens this door)
The door is flush, so it cannot be pushed; it must be taken by the handle. A
straight-down wrist cannot get to the handle at all — the hand fouls the box 0.07 m
short. Yawed 90 degrees the hand is narrow in y *and* its jaws open along x, which is
the one axis in which the handle bar is thin and has free space either side. So the
tool descends vertically astride the bar (fingers clear it by 0.039 in x, and clear the
standoffs too), clamps it, and then the opening is a rigid-body arc of the gripped bar
about the hinge at the box's near-x front corner, with the wrist yawing by the same
angle. Swept in 10-degree steps to 100 degrees so the last commands stay loaded against
the stop.

## Candidate laws (offered for LAWS.md)
- **A flush panel has no push affordance; the handle is the only interface.** When the
  perceived door plane is the same plane as the box front, do not look for a place to
  press — find what protrudes past it.
- **Wrist yaw is a reach tool, not just a grasp tool.** When the tool stalls short of a
  fixture with no depth return in the way, re-probe the same target with the wrist
  yawed 90 deg before concluding "unreachable": the hand body is markedly longer along
  its jaw-opening axis than across it (measured here: 0.04 m of extra reach).
- **Straddle from above beats inserting from the side.** Reaching a slot behind a
  handle sideways costs the whole hand's y-envelope; descending vertically with the
  jaws straddling the bar costs only the bar's own thickness in the jaw axis.
- **A demo whose absolute path is unexecutable can still be sound at its endpoint.**
  v1's replay was a 0/8, yet the same demos' final eef positions are what identified
  the hinge side and the sweep angle.

## DECLARATION
- **Frozen version: v8.** `packs/l90abl_open_microwave_k3/program.py` md5
  `2fb06d49571dfb8bf2f1b419ef604dd2` == `program_v8.py` md5
  `2fb06d49571dfb8bf2f1b419ef604dd2` (verified on the cluster).
- **Selection receipt: 15/15 on the full 15 debug seeds 51-65**, dir
  `results/sel_l90abl_open_microwave_k3_v8` (episodes 51-65 all
  `"benchmark_success": true`; 730-796 sim steps each, under the 1000 cap).
  Prior formal run of the identical policy: 15/15 in
  `results/sel_l90abl_open_microwave_k3_v7`.
- **Per-version receipt chain:** v0 0/2 (probe) · v1 0/8 · v2 probe (15 seeds, no
  motion) · v3 0/2 (probe, aborted) · v4 0/2 · v5 0/3 · v6 probe (1 seed, reach map) ·
  v7 3/3 then 15/15 · v8 15/15.
- **PROVENANCE present** in `program.py` as a top-level literal dict covering
  TABLE_Z, LID_SLAB_SCAN, Y_FACE, HANDLE, BAR_HALF_THICKNESS, TIP_OFFSET, Z_INSERT,
  HINGE, ARC_SWEEP_100DEG, WRIST_YAW90, GRIP_WIDTHS, MOVE_SECONDS. Every constant is
  sourced from this pack, debug-seed observation, or generic controller/camera
  mechanics; the geometric constants are computed at run time from `cam_high`, not
  hard coded.
- No `.done` read (AST-checked). No forbidden file was opened; the bddl path was only
  ever passed to `--bddl`.
