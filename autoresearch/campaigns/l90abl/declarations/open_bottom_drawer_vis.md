# l90abl / open_bottom_drawer_vis — worker notes

Intent: "open the bottom drawer of the cabinet". Pack is vision-only (K=3,
keyframe PNGs + language, no EEF/gripper/action data). Runner: tools/fair_run.py
only. Debug seeds 51-65; eval seeds 1-50 never touched.

## Scene, re-derived from the pack images + debug-seed RGB-D (no priors used)

- Table plane z = 0.900 (modal depth bin, cam_high, seeds 51/55/60).
- Cabinet: block at x -0.133..0.124, y -0.381..-0.190, top slab z = 1.128. Its
  front plane is at y = -0.2318 and its face normal is **+y**, so a drawer opens
  by pulling +y. cam_high's x axis in base is exactly (0,1,0), so image-right IS
  +y, and the pack's final keyframes agree (the drawer body extends right).
- **Three** drawers, each with a horizontal handle bar running along base x:
  bar centres z = 0.948 / 1.017 / 1.090 (spacing 0.069-0.073). Measured profile
  of the bottom bar (ep51, uniform in x, no end posts): front y = -0.1997,
  back y = -0.218, top z = 0.9558, lowest visible z = 0.9426. So the bar is
  ~0.013 tall, ~0.018 deep, and stands **0.0321 proud of the face**, leaving a
  0.0138 slot between its back and the face. It spans x -0.042..+0.044.
  "Bottom drawer" = the z = 0.948 bar.
- Two seed-randomised tabletop props: a bowl (diameter ~0.11, top z = 0.952,
  centre near x -0.02, y 0.00..0.03) and a flat plate (top z = 0.920, y ~ 0.24).
- cam_high: camera at (0.659, 0, 1.610) looking along (-0.778, 0, -0.628);
  image +u = base +y, +v = base +x.
- Episode horizon = 1000 sim steps. `move_pose(seconds=s)` caps at
  max(60, 60*s) steps (STEPS_PER_SECOND = 60) and breaks early on convergence.
- Fingertips sit 0.0087 m ahead of the eef reference along tool z. Measured
  twice against two known planes: a straight-down descent stalls at eef
  z = 0.9087 over the 0.900 table, and at 1.1366 over the 1.128 cabinet slab.
- Open gripper half-width 0.0389 (robot0_gripper_qpos = [0.0389, -0.0389]);
  finger half-thickness ~0.0065, measured from where a descending rear finger
  catches the cabinet's front lip.
- The demos pull the drawer face from y = -0.200 to y = -0.054, a **0.146 m**
  pull, shoving the bowl aside (deprojected from demo0/demo2 keyframes with the
  measured intrinsics/extrinsics; the bottom handle is the one that leaves the
  cabinet, so the demos do open this drawer).

## Handle detector (works, all seeds, sub-mm agreement)

Brightness thresholding does NOT work — the cabinet's lit front edge at
x ~ -0.10 is as bright as the bars and bridges the z clusters (v2 collapsed 3
bars into 2). Geometry does: the handles are the only thing that **protrudes**.

1. box = depth points with z in (0.915, 1.140), y in (-0.50, -0.10), |x| < 0.30
2. y_face = 99.5th percentile of y over the box
3. handles = box points with y > y_face - 0.022 and z < 1.112 (drop the top slab)
4. cluster by z with an 0.018 gap; lowest cluster = the bottom drawer

Gives exactly 3 clusters of n = 597-689 px on every seed tried.

## THE REACH MAP — the load-bearing measurement of this cell

Lowest reachable eef z in front of the cabinet, by wrist tilt (0 deg = straight
down, 90 deg = tool z horizontal along -y), measured at y = bar_front + 0.055,
identical on seeds 51 and 55, and unchanged by gripper open/closed:

| tilt | 0      | 30     | 45     | 60     | 90     |
|------|--------|--------|--------|--------|--------|
| z_min| 1.1366 | 1.0256 | 0.9411 | 0.9646 | 1.0003 |

It is **not monotonic**: 45 deg drops the arm onto a different joint branch
(joint5 ~ -0.90 rather than ~ -0.60/-1.12) and gains 6 cm. At 0 deg the limit is
not the arm at all — the rear finger lands on the cabinet's front lip at 1.128.
No joint is anywhere near a limit at any of these stalls.

## Version log

- **v0** perception probe (heightmap/darkmap), seeds 51,53. Table z = 0.900,
  cabinet top 1.13, two props. No motion.
- **v1** RGB-D dump probe (zlib+base64 through api.log), seeds 51,55,60. Made
  the geometry above measurable offline.
- **v2** fingertip calibration + rotated-wrist reach, seeds 51,55. tip offset
  0.0087; horizontal wrist reaches (0, -0.19, 1.02). Detector still
  brightness-based, merged the bottom two bars.
- **v3** first grasp+pull, seeds 51,53,55,57 → **0/4**. Geometric detector found
  all 3 bars. eef never got below z ~ 1.00; closed on air (width 0.0020).
  Also burned the whole 1000-step horizon (seconds=3 caps at 180, not 60).
- **v4** descend in the drawer-face/bowl corridor, seeds 51,53,55,57 → **0/4**.
  Two seeds gripped (effort 3.0, width 0.023/0.032) and pulled 0.102/0.121 m —
  but at z ~ 1.01, i.e. the **middle** bar. Wrong drawer, mechanism sound.
- **v5** wrist-tilt sweep, seeds 51,55. dz above the bar fell 0.064 → 0.015 as
  tilt went 90 → 55; the arm slid to x ~ 0.085 to clear the bowl.
- **v6** descend hard against the face + x-detour, seeds 51,53,55,57 → **0/4**.
  Stalled at z ~ 1.00 even at x = 0.10, far from the bowl: the blocker is the
  hand, not the scene.
- **v7** tilt/gripper-state limit probe, seeds 51,55. Open vs closed changes
  z_min by 0.5 mm — the open fingers are not the blocker. Also measured the
  face panel at y = -0.2310, fixing the slot width at 0.0138.
- **v8** top-down rear-finger-into-slot hook, seeds 51,53,55,57 → **0/4**.
  The finger is ~0.013 thick and the slot is 0.0138 — it jams on the cabinet's
  front lip. Top-down hook is geometrically impossible.
- **v9** reach map with joint logging, seeds 51,55. Straight down at y = -0.19
  lands on the cabinet lip (z = 1.1366); at y = -0.10 and +0.02 it reaches
  0.9165/0.9098. rotation=None vs explicit R0 makes no difference.
- **v10** fine tilt sweep — produced the reach map above. **45 deg reaches
  0.9411, below the 0.948 bar.**
- **v11** 45 deg grasp + pull, seeds 51,53,55,57 → **0/4**. Approach stalled
  0.027 m short of the bar along tool z; closed on air; pulled 0.119 m of empty
  space.
- **v12** same at the bar's base-side end (0.042 m closer to the base), seeds
  51,53,55,57 → **0/4**. Stall is **x-independent** (x = -0.011 / -0.030 /
  +0.018 all stop at y ~ -0.179), so it is not a distance-from-base limit.
- **v13** stall diagnosis, seed 51. From the stall, -y and -z are hard-blocked
  (48 steps moved 0.0002 m) while ±x moves freely 0.02 m. No joint near a limit.
  A contact whose normal lies in the y-z plane, fixed in the tool frame.
- **v14** insert closed, open around the bar, seeds 51,53,55,57 → **0/4**.
  Closing first buys nothing (depth -0.0264 vs -0.0271). A -22 deg yaw did reach
  the bar's depth but 0.068 m off in x and then destabilised the arm entirely.
- **v15** get low in the clear first, then creep in along -y at constant height,
  seeds 51,53,55,57 → **0/4**; **formal 15-seed run 0/15**. Cleanest run of the
  family: holds z = 0.945 and walks in to y = -0.1677, where creeps 4 and 5 make
  literally zero progress.
- **v16t55 / v16t50** same creep at 55 and 50 deg, seeds 51,53,55,57 → **0/4**
  each. Wedge at y = -0.180 / -0.182, bar still ~0.027 m ahead.

## MECHANISM-GAP STOP

**Candidate law (falsifiable).** *A parallel-jaw gripper whose open half-width
is 0.039 m cannot enclose a handle bar that stands only 0.032 m proud of a solid
face, unless the jaw axis is within ~25 deg of parallel to that face. On this
cabinet the jaw axis can only be made that vertical by tilting the wrist toward
horizontal, and the arm's own ground clearance forbids a horizontal wrist below
z = 1.000 over a 0.900 table — 0.052 m above the bottom bar. The two windows do
not overlap, so no pose in this controller's repertoire grasps the bottom bar.*

The arithmetic, from the measured numbers. At tilt θ the far ("upper") fingertip
sits Δy = 0.0087·sinθ + 0.0389·cosθ behind the eef in y. To clear the drawer
face when the eef is at the bar, that must not exceed the protrusion less the
finger half-thickness, 0.0321 - 0.0065 = 0.0256 m:

| tilt | Δy (needs ≤ 0.0256) | z_min (needs ≤ 0.948) |
|------|---------------------|-----------------------|
| 45   | 0.0337  ✗ (+8 mm)   | 0.9411  ✓             |
| 50   | 0.0317  ✗ (+6 mm)   | ~0.949  ~             |
| 55   | 0.0294  ✗ (+4 mm)   | ~0.952  ✗ (+4 mm)     |
| 60   | 0.0270  ✗ (+1 mm)   | 0.9646  ✗ (+17 mm)    |
| 65   | 0.0243  ✓           | ~0.975  ✗ (+27 mm)    |
| 90   | 0.0087  ✓           | 1.0003  ✗ (+52 mm)    |

The crossover is near 62 deg and both constraints are violated there. 55 deg is
the closest approach, missing on both by ~4 mm — and v16t55 confirms it wedges.

**Receipt on debug seeds.** The predicted failure is that the far finger fouls
the bar/face and the eef parks with the bar ~0.027 m beyond the fingertip plane.
Observed, every seed, every tilt: v11 β = 0.0269, v13 β = 0.0271 (with -y and -z
hard-blocked, ±x free — a contact, not an envelope), v14 β = 0.0264, v15
β = 0.027, v16t55 β = 0.027. The close then reads width 0.0018 (the empty-close
value) and effort 0.05 on 100% of attempts at the bottom bar.

**Control that the rest of the stack is sound.** The identical machinery on the
*middle* bar, which sits 0.069 m higher and so admits a horizontal wrist, grips
(effort 3.0, width 0.023-0.032) and pulls the drawer 0.102-0.121 m — v4, seeds
51 and 53. So the detector, the pull direction, and the grasp-and-drag mechanism
are all verified; only the bottom bar's reach/geometry window is missing.

**What would falsify it.** Any pose that puts the eef within 0.009 m of
(x_bar, y_bar, 0.948) with the jaw axis within 25 deg of vertical. Two untried
leads: a yawed wrist (v14's -22 deg yaw did reach the bar's depth before the arm
destabilised — a smaller yaw with the target recomputed in the yawed frame is
the obvious next probe), and api.act's raw single-step control, which could push
through the wedge with force instead of relying on move_pose's convergence test.

## DECLARATION

- **Frozen version: v15.** `packs/l90abl_open_bottom_drawer_vis/program.py`
  md5 `b39314787aeb24f97ef8beff40bc5b3b` == `program_v15.py` (verified on the
  cluster and locally).
- **Selection receipt (full 15 debug seeds, 51-65):**
  **0/15** — `results/sel_l90abl_open_bottom_drawer_vis_v15`.
  Formally-probed alternative: v6, **0/15** —
  `results/sel_l90abl_open_bottom_drawer_vis_v6`. v15 is the argmax by
  tie-break: it is the only candidate that targets the bottom bar (v6 grasps the
  middle one), and it reaches the bottom bar's neighbourhood at the correct
  height.
- **Per-version receipt chain (probe subset 51,53,55,57 unless noted):**
  v0 probe · v1 probe · v2 probe · v3 0/4 · v4 0/4 · v5 probe · v6 0/4
  (formal 0/15) · v7 probe · v8 0/4 · v9 probe · v10 probe · v11 0/4 · v12 0/4 ·
  v13 probe · v14 0/4 · v15 0/4 (formal 0/15) · v16t55 0/4 · v16t50 0/4.
- **PROVENANCE:** present in program.py, 11 entries, every constant sourced to
  this pack's keyframes or to a named debug-seed measurement.
- Versions v0-v16t50 archived as `program_vN.py` in the pack directory.

STOP — mechanism-blocked after 16 versions.
