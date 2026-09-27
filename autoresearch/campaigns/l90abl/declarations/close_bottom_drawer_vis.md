# l90abl / close_bottom_drawer_vis — worker notes

Intent: "close the bottom drawer of the cabinet". Pack = K=3 vision-only demos
(keyframe images + language, no EEF/gripper/action channel).

## Pack reading

8 keyframes, 3 demos. Every demo t=0 frame shows the same layout: a cabinet at
the right of the table with a white top slab, its **bottom drawer pulled out
toward the image left**, a black bottle just in front-left of the drawer, a
plate and a wooden rack further left. Every demo's final keyframe has the arm
folded down over the cabinet and no protruding drawer box. So the task is a
straight push of an already-open drawer; no grasp is implied by the pack.

## Scene frame (my own debug-seed measurements, cam_high depth)

- cam_high extrinsics: camera at base (0.659, 0, 1.610), image **u -> +y**,
  image v (down) -> +x and -z. So the table's near edge is at the bottom of the
  frame and +y is to the right.
- table top z = **0.901** (modal deprojected height, seeds 51/55/60).
- cabinet top slab z ~ **1.11-1.13**, spans x -0.13..0.13, front face y ~0.19-0.22.
- open bottom drawer: interior floor a flat plane at z = **0.925**, front-wall
  top rim at z = **0.985**, handle bar at z ~0.954 and y ~0.05.
- drawer therefore opens toward **-y**; closing it is a **+y push**.
- open extent (seed 51): drawer front ~y 0.07 vs cabinet front ~y 0.21, i.e.
  ~0.14 m of travel.

## Runtime mechanics (measured, v4)

- Episode horizon = **1000 sim steps**; `api.act` is unsupported on the LIBERO
  backend (no `step_raw`), so `api.move` is the only actuator.
- `api.move(seconds=s)` runs at most `max(40, 60*s*2)` steps and **breaks early
  only when the residual is under 0.012**. A converged 0.12 m descent used far
  fewer than its cap; an *unreachable* target burns the whole cap for nothing.
- **-y is kinematically blocked**: from x=-0.21, z=1.05 the arm stalls at
  y=-0.124 and six further 60-step moves make zero progress. A single move to
  y=+0.29 covered 0.42 m and converged. Approaches must live at y >= ~-0.10.

## Version log

- **v1** (probe, ep51): pure observation. Logged intrinsics/extrinsics and a
  32x32 deprojected grid. Established the frame above. Verdict: scene understood.
- **v2** (probe, ep51/55/60): fine ASCII height map + height bands. Verdict:
  drawer geometry above, stable across the three seeds.
- **v3** (probe, ep51/55): fingertip calibration by descent + first push.
  FAILED for a mechanism reason, not a hypothesis reason: the calibration point
  was at y=-0.22, which is past the -y reach wall, so every 96-step move burned
  its full cap making ~1 mm of progress and the horizon (1000 steps) expired
  before the push ever started. Verdict: budget lesson, not evidence about the push.
- **v4** (probe, ep51): motion-rate measurement. Produced the runtime mechanics
  above. Verdict: `api.move` is fast when the target is reachable; the v3 stall
  was a reach wall.
- **v5**: first real attempt. Perceive cabinet slab (-> push x, cabinet front y)
  and drawer floor (-> drawer front y); transit above the rim; press the closed
  fingers onto bare table in front of the drawer to calibrate the fingertip
  offset from the stall height; lift 0.035 m; push +y to (cabinet front + 0.12)
  so the OSC keeps pressing after the drawer bottoms out.
- **v6** (probe, ep51/53/55/57): identical to v5 but the push column shifted
  -0.12 in x. **2/4 success** (51, 53). Both failures had `tip_off=0.091`, i.e.
  the table-press calibration landed on a prop and set the push height 7 cm too
  high, so the fingers rode over the drawer rim. Verdict: the v5 stall was the
  **+y reach envelope**, not drawer resistance -- moving the push column
  0.11-0.12 m toward the base raised the stall from y=0.087 to y=0.115, which
  is enough to shut the drawer. The in-episode height calibration is the
  liability, not the asset.
- **v7** (ep51..65 odd, 0/8): replaced the floor mask with a "front wall" band
  mask restricted to the push column, and dropped nothing else. The mask
  returned yf ~= -0.10 on all eight seeds (a prop, not the drawer), and a
  `np.clip(lo>hi)` ordering bug then let the approach be placed at y=-0.14 --
  past the -y reach wall. Every episode stalled at y~=-0.115 without touching
  the drawer (`moved 0.000` on all eight). Verdict: two independent bugs; the
  useful residue is a clean re-confirmation of the -y wall at ~-0.12.
- **v8** (FROZEN): perception reduced to the one surface that is unambiguous --
  the cabinet's top slab (>12000 points on every seed) -- giving the push
  column x and the cabinet front y. Push height is the declared constant
  `TABLE_Z + TIP_OFF + PUSH_TIP_H` = 0.944 instead of an in-episode press; the
  descent column is instead *checked* clear against the depth map, with three
  fallback standoffs. Push column = slab centre minus 0.11. Over-command to
  `yc + 0.12` so the OSC keeps pressing. **8/8 on the probe subset, 15/15 on
  the full debug band.** Episodes terminate at ~72 sim steps of a 1000 budget,
  i.e. the drawer shuts almost immediately once the geometry is right.

## Candidate laws (for LAWS.md)

1. **A push stall can be the reach envelope, not the object.** The same +y push
   stalled at y=0.087 from the object's x-centre and at y=0.115 from 0.11 m
   nearer the base, at two different heights and on every seed. A stall that is
   identical across heights and seeds is kinematic; move the contact column
   toward the base before concluding the object is jammed.
2. **In-episode calibration by pressing the table is a liability when the
   press point is chosen by the same perception that may be wrong.** The
   fingertip offset is fixed geometry (0.008 m here); measuring it once on
   debug seeds and *checking the descent column clear* is strictly safer than
   re-measuring it per episode at a point a prop may occupy.
3. **`np.clip(v, lo, hi)` with `lo > hi` silently returns `hi`.** A guard band
   written as `clip(yf - gap, REACH_MIN, yf - 0.03)` inverts whenever the
   perceived feature is far enough away, and hands back the value the guard
   existed to forbid.

## DECLARATION

- **Frozen version: v8.** `packs/l90abl_close_bottom_drawer_vis/program.py`
  md5 `d93d30d74d57c7245a3017d8c604a6ba` == `program_v8.py` (same md5).
- **Selection receipt: 15/15** on the full debug band (seeds 51-65),
  `results/sel_l90abl_close_bottom_drawer_vis_v8` on AbakaAI.
- **Receipt chain:** v1 probe (ep51, scene frame) -> v2 probe (ep51/55/60,
  drawer geometry) -> v3 probe (ep51/55, budget lesson) -> v4 probe (ep51,
  motion mechanics + -y wall) -> v5 0/8 `fs_..._v5` -> v6 2/4 `fs_..._v6` ->
  v7 0/8 `fs_..._v7` -> v8 8/8 `fs_..._v8` -> v8 15/15 `sel_..._v8`.
- **PROVENANCE:** present in program.py, 14 entries, every calibrated constant
  sourced to this pack's keyframes or to a debug-seed measurement recorded above.
- Eval (seeds 1-50) is coordinator-run; this worker never touched it.
