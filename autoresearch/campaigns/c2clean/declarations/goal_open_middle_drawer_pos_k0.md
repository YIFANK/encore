# c2clean goal_open_middle_drawer_pos_k0 — NOTES

Intent: "open the middle drawer of the cabinet". No demo pack (k0). Runner: tools/fair_run.py only.

## Scene, as measured from debug seeds 51/53/55/57 (cam_high + wrist RGB-D)

- Support plane (table) z = 0.901; cam_high f = 618 px (2.1 mm/px at the cabinet),
  wrist cam f = 334 px (0.7 mm/px at 0.22 m).
- The drawer cabinet is the dark neutral box at y > 0; the wooden slatted object at
  upper-left is a dish rack, not a cabinet. Robot links are saturated in colour, the
  cabinet and its handles are neutral — a saturation mask (S < 0.18) removes the arm
  cleanly and is what makes connected-component segmentation work at all (without it
  the whole scene merges into one component through the arm).
- Cabinet: top z = 1.128, front face plane y ≈ 0.124–0.137 (varies by seed under the
  _pos perturbation), x span ≈ −0.52 … −0.27.
- Three horizontal bar handles, axis along base **x**, length ≈ 0.085 m, centred at
  x ≈ −0.41. Silhouette tops (cam_high, direct measurement): 0.9550 / 1.0250 / 1.0984.
- Bar diameter **0.0174 m** — receipt: the closed-gripper gap when the grip lands on
  the bar (v1 ep53/55/59/61, all four read 0.0173–0.0174). Bar front y ≈ 0.105, face
  y ≈ 0.136, so the bar stands ≈ 32 mm proud with only ≈ 15 mm behind it: nothing can
  be hooked between the bar and the face.
- Free vertical channel around the middle bar (top of the bar below → bottom of the bar
  above) = 0.955 … 1.081, i.e. **126 mm**. The fully open Panda hand measures ≈ 123 mm
  across its finger backs (inner faces ±0.0395, blade ≈ 22 mm). So the straddle window
  is only a few mm wide in z.
- api.eef sits **11.5 mm behind the fingertips** along the approach axis (p5 preclose:
  front-most gripper point 0.1057 vs eef y 0.0942).

## Controller mechanics that dominate this cell

- Episode horizon **1000 sim steps**. move_pose costs 60·max(s,0.5) steps and does NOT
  exit early unless BOTH position (<12 mm) and orientation (<1°) converge — the wrist
  never gets under 1°, so every move_pose chunk costs its full 60 steps but also keeps
  driving position far tighter than move_cartesian's 12 mm break. move_cartesian costs
  60 steps minimum. api.grip costs 20.
- api.grip gives only 20 steps of closing — not enough to travel the 60 mm from fully
  open down onto a 17 mm bar. It has to be called repeatedly (v4 receipt).
- Rolling the wrist and translating in the same move_pose drags the eef sideways into
  the cabinet (p3). Rotating in place first, then translating, works.

## Version chain

| ver | change | receipt (probe seeds 51–65 odd unless noted) |
|-----|--------|----------------------------------------------|
| p0/p1 | perception dump (chunked around the 2000-char api.log cap) | scene geometry above |
| p2 | first straddle attempt | horizon exhausted by move_pose seconds= budget |
| p3 | budget-instrumented, rotate+travel together | arm driven into the cabinet, jammed |
| p4 | rotate in place, wrist rolled 180° about the approach axis | arm frozen at start pose — that roll is unreachable |
| p5 | wrist rolled +90° about world x (minimal roll) | reaches the bar; upper finger collides with the bar ABOVE |
| v1 | channel-centre aim + z-retry scan | 5/8 gripped the bar (gap 0.0174), 0/8 success |
| v2 | roll the wrist at the start pose, then travel | 0/8 — the rolled arm cannot reach down to z = 1.02 |
| v3 | v1 order + bar radius from the closed-gap receipt + saturated far pull | 3/8 gripped; pull stalls at eef y ≈ 0.06 |
| p7 | release after the stall and retreat | arm still frozen at y ≈ 0.06 → **the 90° wrist walls the arm out**, not the drawer |
| p8 | y-sweep at two x values, pose and lin | same wall at y ≈ 0.070 everywhere |
| v4 | tilted grasp (wrist 60° from home) | arm reaches y = −0.074 once the bar slips → tilt buys reach, but the bar sits near the fingertips and squirts out |
| v5 | v4 + repeated grip + pure −y pull | pull runs free to y ≈ −0.17, grip lost in 8/8 |
| v6 | deep 90° grasp, then roll the wrist 30° back about the bar axis mid-pull | **2/8** (ep53, ep61) — first non-zero |
| v7 | same, roll ramped 90→82→74→67→61→56 | 1/8 |
| p10 | y-reach map vs wrist roll, no object | wrist straight down reaches y = −0.18…−0.27; any roll ≥45° walls out at y ≈ 0.035 |
| v8 | bite 1 (bar) + bite 2 (push the open drawer from inside, wrist straight down) | 2/8 (ep51, ep53); bite 2 fouled the bar of the drawer above |
| v9 | bite 1 with an extra 72° roll chunk; side insertion in x | 0/8 — the extra roll slips the grip, so eef travel over-reports the opening and bite 2 aims at nothing |
| v10 | re-measure the drawer with the camera before bite 2 | 0/8 — the re-measure locked onto the dish rack |
| v11 | pull with x,z pinned to the grip pose, ramped roll + re-grip | 1/8 — pinning x,z levers the bar out as the wrist rolls |
| v12 | = v6 plus a free end-of-episode capture | **2/8** (ep53, ep61) — reproduces v6 exactly |
| v13 | v12 + ramped roll + re-grip after every pull chunk | 0/8 — the re-grip spends the steps the pull needs |
| v14 | v12 but the pull drops the orientation command entirely | 0/8 — letting the wrist relax does not buy reach |

## Selection

**v12 formal run, all 15 debug seeds: 6/15** — `results/sel_c2clean_goal_open_middle_drawer_pos_k0_v12`
(51 F, 52 T, 53 T, 54 F, 55 F, 56 F, 57 F, 58 T, 59 F, 60 T, 61 T, 62 F, 63 F, 64 T, 65 F).

Note the odd-seed probe subset read 2/8 for exactly this program while the even seeds
ran 4/7 — the probe subset was pessimistic by a factor of two, so every 0/8 verdict in
the table above is a probe-subset verdict, not a full-split one.

v8 (the two-bite variant) was also run formally on all 15: **6/15** as well
(`results/sel_c2clean_goal_open_middle_drawer_pos_k0_v8`, seeds 51,52,53,58,60,64).
A tie, so the frozen program is v12 — one mechanism rather than two, and it needs no
estimate of how far the first bite opened the drawer.

## Mechanism gap (why this cell is not solved, stated falsifiably)

The grasp is solved: 7/8 probe seeds seat the hand around the middle bar and 6/8 close on
it with a gap of 0.0174 m, the bar's measured diameter. What is not solved is dragging the
drawer far enough. Two hard constraints collide:

1. **The grasp wrist walls the arm out.** With the wrist rolled 90° (the only roll that
   seats the bar deep enough in the jaws to survive a pull) the arm cannot retract past
   y ≈ 0.06 no matter the x, the height, or whether the orientation is commanded —
   p7 shows it still frozen there after the gripper is opened, and p8 reproduces the wall
   at two x positions with both move_pose and move_cartesian. The grip is taken at
   y ≈ 0.128, so a fixed-wrist pull can only ever open the drawer ≈ 70 mm.
2. **Buying reach costs the grip.** Rolling the wrist back mid-pull does unlock the arm
   (straight down it reaches y = −0.27, p10), and when the bar survives the roll the
   drawer comes out 150 mm and the predicate fires. But the roll swings one finger toward
   the drawer face and levers the bar out of the jaws about half the time. Ramping the
   roll (v7, v13), re-gripping between roll steps (v11, v13) and pinning x/z to the grip
   pose (v11) all made it worse, not better.

Falsifiable statement of the missing mechanism: **the cell needs a way to apply a −y drag
whose wrist pose is free to change without unloading the grip** — either a partially-closed
gripper (the binary open/close of `api.grip` cannot straddle the 126 mm inter-bar channel
with anything narrower than the 123 mm fully-open hand, which is what forces the 90° roll),
or a purchase that is not the bar at all. The most promising untried version of the latter
is v8's second bite — dropping the closed gripper into the opened drawer's mouth and
pushing from inside with the wrist straight down, which needs no grip and has the full
y = −0.27 reach. It ties v12 at 6/15 today only because the first bite rarely opens the
drawer the ≈ 55 mm that the insertion needs, and because the gripper's 36 mm closed width
in the finger-separation axis does not fit the gap the first bite leaves. Predicted and
testable: yaw the wrist 90° so the closed pair presents its ≈ 22 mm edge to that gap, and
bite 2 should carry most of the seeds that bite 1 opens past ≈ 45 mm.

## DECLARATION

- **Frozen version: v12.** `packs/c2clean_goal_open_middle_drawer_pos_k0/program.py`
  md5 `085d141ad9ec64841d6f0606e8a81060` == `program_v12.py` (verified on the cluster).
- **Selection receipt: 6/15 on the full debug split (seeds 51–65)** —
  `results/sel_c2clean_goal_open_middle_drawer_pos_k0_v12`, successes on seeds
  52, 53, 58, 60, 61, 64.
- **Per-version receipt chain:** the table above; every entry is a run under
  `tools/fair_run.py --split debug` with its `results/` directory named in the run log.
  Probe subset = seeds 51,53,…,65; formal runs use all 15.
- **PROVENANCE present:** 15 entries in `program.py`, every one `allowed: True` with a
  debug-seed or generic-mechanics source. No `api.done` read anywhere in the program.
- Status: **mechanism-blocked after 14 versions**, declared at the argmax. Gap documented
  above.
