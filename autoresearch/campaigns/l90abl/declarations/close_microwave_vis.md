# l90abl / close_microwave_vis — worker notes

Cell: LIBERO-90 KITCHEN_SCENE6, intent "close the microwave", vision-only pack
(K=3 demos, keyframe images + language only; no EEF/action data).
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65).

## What the pack shows (only task-specific input used)

14 keyframes, 3 demos. Every demo does the same two things: pick the near mug
up, set it down next to the other mug on the left, then move right and the
black-framed panel that is visible left of the microwave body in frame 0 is
gone by the last frame. Read: that panel is the OPEN microwave door and the
last motion closes it. The mug transport is in all three demos but the door
sweep is what changes the scene state; I treated the mug as optional and
tested that reading (it held — see v2).

## Scene geometry, re-derived from debug seeds (probe v0/v1)

cam_high is at base (0.659, 0, 1.610) looking -x and down; image u -> base +y.
Renders under EGL are untextured (microwave blue, arm green) but depth is exact.

- table top = modal workspace height, 0.901-0.902 m on 51/52/53.
- In the height band [table+0.14, table+0.22] the scene contains exactly two
  things: the microwave body (a filled slab, x -0.18..+0.18, y 0.24..0.40,
  top 1.108) and the open door (a thin 1-3 cell line sticking out toward -y).
  Mugs top out ~table+0.10 and are excluded by the band — that is what makes
  the segmentation trivial.
- The door is hinged at the body's front-left corner (min-x, min-y of the
  slab), is 0.230-0.235 m long, spans z table+0.03..table+0.20, and starts at
  -80 deg .. -105 deg from the closed direction (+x along the body front
  face). Closing = rotating it back to 0 about that corner.
- Closing sweep direction at door angle th: v = (-sin th, cos th); the pushing
  side is the -v side (-x when the door is ~-90 deg).
- probe v1: closed fingers bottom out on the table with eef z = 0.908 (table
  0.902), i.e. the tips sit ~6 mm below the eef point.
- probe v2: the episode horizon is exactly 1000 sim steps. A blocked
  `api.move` burns max(40, 120*seconds) steps, so seconds is the budget knob.
- `api.act` is refused on this backend (LiberoRobot has no step_raw), so the
  program is built from move/grip/settle only.

## Versions

### v1 — arc sweep, staged on the door line  (1/8, receipt fs_..._v1)
Hypothesis: place the closed gripper behind the door's free half at
r = 0.17 m from the hinge, then walk the tool along the hinge arc in 12 deg
steps to theta = 0.
Evidence: 51 F, 53 T, 55 F, 57 F, 59 F, 61 F, 63 F, 65 F. The staging descent
to table+0.056 stopped at eef z = 1.075 with a 0.125 m residual on 7/8 seeds —
the gripper landed on the door's TOP EDGE (door top 1.108) because the staging
point, one 10 deg step short of the door, was still inside the panel's
thickness/quantisation. Those runs then pushed at the door's top edge, dragged
it only to -14..-21 deg, and jammed the hand on the microwave's front-top edge.
The one seed whose descent did reach 0.955 (53) closed the door and terminated
at 361 steps.
Verdict: the arc is right; the approach height/offset is the failure.

### v2 — stage 55 mm BEHIND the door plane, 15 deg steps  (8/8 probe, 15/15 formal)
Change: staging point = hinge + 0.17*u(th0) - 0.055*v(th0) (clear table, no
panel underneath), descend there to table+0.056, then sweep in 15 deg steps at
seconds=0.7 so 8 pushes fit inside the 1000-step horizon.
Evidence: probe 51,53,...,65 = 8/8 (`results/fs_l90abl_close_microwave_vis_v2`).
Every seed's descent converged (residual 0.006-0.008 at z ~0.958) and the
episode terminated after 3-5 pushes, at 78-146 sim steps — the predicate fires
once the door is dragged back to roughly -20 deg; the remaining scripted
pushes then cost nothing. The mugs are never touched, which settles the
question the demos raised: the mug transport is not required for the intent.
Verdict: SELECTED.

## Selection receipt (formal, full 15 debug seeds)

`results/sel_l90abl_close_microwave_vis_v2` — 15/15
(51 T, 52 T, 53 T, 54 T, 55 T, 56 T, 57 T, 58 T, 59 T, 60 T, 61 T, 62 T,
63 T, 64 T, 65 T; 78-146 sim steps each, no program errors).

## DECLARATION

- Frozen version: **v2**. `packs/l90abl_close_microwave_vis/program.py`
  md5 `5693414d302964cca877d4d02c4a995b` == `program_v2.py` (same md5).
- Selection receipt: **15/15** on the full debug band,
  `results/sel_l90abl_close_microwave_vis_v2`.
- Receipt chain: v1 1/8 (`results/fs_l90abl_close_microwave_vis_v1`),
  v2 8/8 probe (`results/fs_l90abl_close_microwave_vis_v2`), v2 15/15 formal.
- PROVENANCE present in program.py: TABLE_BIN, BAND_LO, BAND_HI, CELL,
  BODY_MIN_CELLS, DOOR_L, CONTACT_R, PUSH_Z, TIP_OFFSET, STEP_DEG, STAGE_BACK —
  every constant sourced to a debug-seed measurement or generic controller /
  camera mechanics. No pack field other than the keyframe images and the
  language was used; no forbidden file was read; `api.done` is never read.

## Candidate law (for LAWS.md)

Approach a hinged panel from BEHIND its swing plane, not from above its edge:
a top-down descent onto the panel line stalls on the panel's top edge and the
subsequent push acts at the top of the panel, where the hand fouls the
cabinet the panel belongs to. Staging one panel-thickness plus tool-radius
behind the plane (55 mm here) turns a 1/8 cell into 15/15.
