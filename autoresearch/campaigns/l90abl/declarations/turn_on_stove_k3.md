# l90abl / turn_on_stove_k3 — NOTES

## Pack reading (pre-run, no cluster execution)
- 3 demos, lengths 90/89/94. All three share one shape:
  1. from home (~[-0.21, 0.00, 1.19], wrist straight down) travel to
     (x~-0.20, y~+0.21, z~0.95);
  2. close the gripper there (gripper_cmd flips -1 -> +1 at t=43/41/47);
  3. press down to z~0.9285 AND hold a large sustained yaw command.
- Raw actions confirm phase 3 is contact-limited, not a pose move: after the
  close, `actions[:,5]` (daz) pins at +0.375 for ~40 consecutive steps while
  dz stays ~-0.39. action_scale[5]=0.164 rad/unit -> commanded 0.06 rad/step,
  ~2.8 rad over the hold, but the realised yaw change is only 53-67 deg. The
  knob resists; the command is force, not a setpoint.
- ROTATION CONVENTION: ee_path6[3:6] is a ROTATION VECTOR (axis-angle), not
  euler rpy. Evidence: under rvec the tool z-axis stays ~world-down
  ([-0.08,0.03,-0.996] -> [-0.114,-0.088,-0.990]) across every demo and the
  start rvec has norm ~3.175 ~ pi about +x (= straight down); the relative
  rotation R0^T R1 is a pure spin about the tool z axis (axis
  [-0.11,-0.09,-0.99], 53-73 deg). Under euler-rpy the same data would swing
  the tool z to horizontal, which the keyframe images refute (the hand still
  points down in the final frame). To be re-checked against
  api.tool_rotation() on a debug seed.
- Realised yaw of the tool x-axis: demo0 +67deg, demo1 +67deg, demo2 +53deg
  (all positive = counter-clockwise about world +z). 53 deg is the smallest
  turn that sufficed in the pack.
- Keyframe images (128x128 cam_high): table with a black pan (left), a moka
  pot (centre), and a stove slab (right) carrying a burner disc and a small
  BLACK KNOB standing up at its back-right. Final frame shows the burner
  glowing red. So the manipulandum is the knob, grasped top-down and spun.
- Grasp xy across demos: (-0.1954,0.2096) (-0.1810,0.2092) (-0.2114,0.2027).
  y is tight (spread 0.007) but x spreads 0.030 -> worth perceiving rather
  than hard-coding, if the debug seeds move the stove.

## v1 -- pack-mean open-loop plan (measurement + first candidate)
Hypothesis: the knob is a fixture, so the pack-mean grasp xy plus a
contact-limited spin is enough; nothing needs perceiving.
Program: aim (-0.1959, 0.2072) = pack mean; hover z 1.05 -> descend to 0.9493
with the wrist straight down (R_DOWN = diag(1,-1,-1)); `api.grip(0.0)`; then
10 staged yaw targets 10 deg apart about world +z, each `api.move` carrying a
position target 15 mm BELOW the pack's final z so a downward error stands the
whole time (the pack's press).
Also dumped cam_high/cam_arm_wrist RGB-D + K + t_base_cam per episode to
results/l90abl_turn_on_stove_k3_dbg/ for offline analysis.

Evidence:
- probe results/fs_l90abl_turn_on_stove_k3_v1 (51,53,...,65): **8/8**
- selection results/sel_l90abl_turn_on_stove_k3_v1 (all 15): **15/15**
- `api.act` is NOT available on this backend: only robosuite implements
  step_raw, so the raw pack actions cannot be replayed. move_pose is the
  substitute and it happens to be the right one -- it runs max(60, 20*seconds)
  steps and breaks only when position AND orientation are both converged, so
  an unconvergeable rotation target keeps commanding torque for the whole
  budget. That is exactly the pack's sustained command.
- ROTATION CONVENTION CONFIRMED. api.tool_rotation() at t=0 is
  [[0.998,0.001,-0.057],[0.001,-1,0],[-0.057,0,-0.998]]; the pack's start
  ee_path6[3:6] read as a ROTATION VECTOR gives
  [[0.999,-0.015,-0.044],[-0.013,-0.999,0.038],[-0.045,-0.037,-0.998]]
  -- agreement to 0.04. Read as euler rpy the tool z would leave vertical by
  the end of the demo, which the keyframe images refute.
- Success fires at only +37.9 deg of realised yaw (all 15 seeds; the wrist
  then freezes because the episode terminates), i.e. well short of the 53-67
  deg the demos ran to. The pack over-turns.
- Descent stalls on contact: target 0.9493, eef 0.9592 (= knob top - 0.001),
  residual 0.010. Gripper closes to a 0.025-0.028 m gap at effort 3.0.
Verdict: mechanism solved, 15/15. Kept as a candidate.

## Scene measurement from the v1 dumps (offline, no extra episodes)
Detector: crop cam_high to +-0.13 m around the pack-mean prior; take the flat
support as the 90th percentile of z within [0.905,0.950) (this band excludes
the knob standing on it); the knob is the dark (lum<80) blob more than 8 mm
above that support.
- Fires on 15/15 seeds, 488-512 px every time.
- Support (stove slab top) = 0.9315, sd 0.00000 across seeds.
- Knob top = 0.9601, sd 0.00000 across seeds. Knob height 0.0286.
- Knob centroid: x -0.2011 +- 0.0042 (range 15 mm), y 0.1983 +- 0.0056
  (range 18 mm). Knob luminance 47-49 vs slab ~110.
- The stove slab centroid moves by the same 15 mm -> the fixture jitters as a
  rigid body; the loose pan moves ~48 mm but sits at y = -0.24, nowhere near
  the knob or the approach corridor.
- Two runs of the SAME seed produce bit-identical scenes: seeding is
  deterministic per episode.

## Aim-margin diagnostics (16 runs x 4 seeds 51,55,58,62; diagnostics, not
## candidates -- archived as diag_off_*.py)
v1 re-run with the aim point deliberately displaced:
| displacement | -x | +x | -y | +y |
| 25 mm | 4/4 | 4/4 | 4/4 | 4/4 |
| 40 mm | 4/4 | 4/4 | 4/4 | 4/4 |
| 60 mm | 0/4 | 1/4 | 4/4 | 3/4 |
| 80 mm |  -  | 0/4 | 1/4 | 0/4 |
So the aim envelope is about +-50 mm in x and +-70 mm in y. The whole
observed placement spread is 15-18 mm, i.e. the frozen constant sits inside a
region roughly 3x larger than anything the seed distribution produces. The
margin comes from the open jaw gap (0.078 m) being far wider than the knob.

## v2 -- perceived knob, verified grasp, one retry (FROZEN)
Hypothesis: v1's 15/15 rests on a constant with a measured +-50 mm envelope
and only 15 mm of observed scene jitter, so it will hold on the eval band --
but a perceived target should be immune to placement drift altogether, and
that difference is testable without touching the eval seeds.
Program changes over v1:
  - aim xy AND the hover/close/press heights are derived from the knob found
    in cam_high by the detector above (support = 90th pct of z in
    [0.905,0.950) under the prior; knob = dark blob >8 mm above it), with the
    z offsets carried over from the pack relative to the measured knob top;
  - the detection is gated on blob size (200-1400 px) and knob height
    (0.015-0.055 m) and falls back to the pack-mean prior if a gate fails;
  - the grasp is verified (effort 3.0 AND finger gap in [0.010,0.045]);
  - a turn that yields <20 deg of realised yaw releases, re-perceives from the
    hover pose and retries the grasp-and-turn once.
No success signal is read anywhere; every branch is driven by the program's
own sensors (`api.done` is never touched).

Evidence:
- probe results/fs_l90abl_turn_on_stove_k3_v2 (51,53,...,65): **8/8**
- selection results/sel_l90abl_turn_on_stove_k3_v2 (all 15): **15/15**
- logs: the gate fired on all 15 seeds (px 488-512, slab 0.9315, ztop 0.9601,
  knob height 0.0286, dprior 3-20 mm), held=True on all 15, turned +38.0 deg
  on all 15, one attempt -- the fallback and the retry never fired.

## The experiment that separated v1 from v2 (8 runs x 4 seeds, diagnostics)
v2 re-run with its PRIOR (the search-window centre) deliberately displaced,
against the same displacement applied to v1's fixed aim point:
| prior/aim displacement | v1 (fixed aim) | v2 (perceived) |
|  60 mm -x |  0/4 | 4/4 |
|  60 mm +x |  1/4 | 4/4 |
|  60 mm -y |  4/4 | 4/4 |
|  60 mm +y |  3/4 | 4/4 |
| 100 mm -x |   -  | 4/4 |
| 100 mm +x |  0/4 (at 80 mm) | 4/4 |
| 100 mm -y |  1/4 (at 80 mm) | 4/4 |
| 100 mm +y |  0/4 (at 80 mm) | 4/4 |
v2 is 32/32 across displacements that break v1. Perception replaces v1's
+-50 mm aim envelope with one bounded only by the 130 mm search half-window,
at no cost on the debug band. That is the reason v2 is the frozen version
rather than the equally-scoring v1.

## DECLARATION
- Frozen version: **program_v2.py**, copied to
  `packs/l90abl_turn_on_stove_k3/program.py`.
  md5 `dd27b996791862343fd628a4fe2827c4` == md5(program_v2.py). Verified on
  the cluster and locally.
- Selection receipt (full 15 debug seeds 51-65, run on the exact frozen
  bytes): **15/15** in `results/sel_l90abl_turn_on_stove_k3_v2_final`
  ("turned +38.0 deg held=True" on all 15; no gate failure, no retry).
  An earlier selection of the same program before its PROVENANCE dict was
  completed (`FALLBACK_ZTOP` was undeclared) also scored 15/15 in
  `results/sel_l90abl_turn_on_stove_k3_v2`; the run above supersedes it and
  receipts the frozen file byte-for-byte.
- Per-version receipt chain:
  - v1 (pack-mean open loop): probe `fs_..._v1` 8/8; selection `sel_..._v1`
    **15/15**; aim envelope measured at +-50 mm (x) / +-70 mm (y) over 16
    diagnostic runs.
  - v2 (perceived knob + verified grasp + retry): probe `fs_..._v2` 8/8;
    selection `sel_..._v2_final` **15/15** (and `sel_..._v2` 15/15 before the
    PROVENANCE completion); robust to 100 mm prior displacement in all four
    directions (32/32) where v1 fails.
- PROVENANCE: present in program.py, 20 entries (every module-level
  constant is covered), every calibrated constant
  sourced to pack.json fields or to debug-seed (51-65) measurements.
- Splits: eval seeds 1-50 were never run, listed, or read. Every episode in
  this cell used `--split debug` with episode lists drawn from 51-65.
- Clean room: reads were limited to `packs/l90abl_turn_on_stove_k3/pack.json`
  + `keyframes/`, this cell's own `results/*l90abl_turn_on_stove_k3*`, and
  the harness/controller sources `tools/fair_run.py`, `tools/fair_client.py`,
  `heron/robot/libero.py` (runner mechanics, not benchmark assets). No .bddl,
  .xml, .hdf5, init_states or gt_trace file was opened; no other cell's pack,
  results, program or NOTES was opened.
- GIFs: `gifs/v2_ep51_ok.gif` (frozen version), 
  `gifs/v2_prior_offset_-100mm_ep51_ok.gif` (perception following a 100 mm
  displaced prior), `gifs/v1_aim_offset_-60mm_ep51_fail.gif` (the fixed-aim
  failure mode that motivated v2).
STOP.
