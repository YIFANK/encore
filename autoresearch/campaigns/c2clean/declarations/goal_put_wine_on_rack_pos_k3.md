# c2clean · goal_put_wine_on_rack_pos_k3

Intent: `put the wine bottle on the rack`. Runner: `tools/fair_run.py` only.
Pack: `packs/c2clean_goal_put_wine_on_rack_pos_k3/` (K=3 demos, 18 keyframes).

## What the pack says (mechanism, not coordinates)

`ee_path6[3:6]` is an **axis-angle rotvec**, not rpy: at every demo t=0 its norm
is 3.13 ≈ π and the implied tool z-axis is (≈0, ≈0, −1), i.e. straight down.
That check is what makes the rest of the pack readable.

Gripper channel `actions[:,6]`: +1 = close, −1 = open (demo1 closes at t=76 and
opens at t=157; the gripper_state pair goes 0.079 → 0.031 over that window).

Per-demo successful grasp → release:

| demo | close t | grasp eef | release t | release eef | hold width |
|---|---|---|---|---|---|
| 0 | 81  | (−0.210, −0.082, 1.018) | 150 | (−0.156, −0.269, 1.203) | 0.018 |
| 1 | 76  | (−0.214, −0.068, 0.983) | 157 | (−0.192, −0.263, 1.191) | 0.031 |
| 2 | 117 | (−0.198, −0.100, 0.947) | 169 | (−0.189, −0.251, 1.193) | 0.045 |

(demo0 and demo2 each contain a failed close before the good one.)

**The load-bearing quantity is the rotation change, not the xy.** Applying
`R_place @ R_grasp^T` to the initially-vertical bottle axis gives, per demo,
(0.043,−0.730,0.682) / (0.133,−0.837,0.531) / (0.148,−0.532,0.834): the bottle
is released **tilted 33–58° from vertical, leaning −y, cork up-slope**. Demo1's
(0.133,−0.837,0.531) is within 2° of the rack plane's own slope direction
measured on the debug seeds. The demos grasp with a tilted wrist and rotate to
near-vertical; an equivalent and far easier-to-aim route is to grasp top-down
and rotate the wrist by the same net angle at the rack. That is what the
program does.

**Demo absolute xy is a decoy here.** The demo scene sits ~(+0.27, +0.05) away
from every debug-seed scene — the bottle is at x≈−0.21 in the pack and at
x≈+0.06 on seeds 51–65, and the rack moves with it by the same vector, so the
bottle↔rack offset (0.20 m) is preserved. Everything positional is therefore
re-derived from cam_high each episode.

## Scene, measured from debug seeds (cam_high RGB-D)

- Table top deprojects to z = 0.901. Walls at x = −1.99; floor at z = 0.003.
- **Bottle**: dark (mean RGB 6–15 vs >80 for table/rack/stove), stands on the
  table, cork tops out at z ≈ 1.053 (table+0.151). Body radius 0.020, neck
  radius 0.0077 over z ∈ [1.01, 1.05], cork band (grey ≈ 110) above that.
  Across seeds it jitters by ~2 cm.
- **Rack**: a slatted ramp on the −y side with two parallel shelves. The upper
  shelf fits a plane `z = 1.04 − 0.575 y` (residual σ = 2.5 mm, no x term),
  i.e. 29.9° from horizontal, spanning x ∈ [−0.10, 0.17], y ∈ [−0.30, −0.16].
  Intercept and y-extent shift by ~1.5 cm between seeds, so the rack is
  perceived per episode, not hard-coded.
- Decoy: a dark stove knob at (−0.39, +0.20), top z = 0.960 — it is the
  highest dark cluster in the scene whenever the bottle is missed.

## The step budget is a first-class resource

`--horizon` defaults to 1000 sim steps, and `results.jsonl` reports `sim_steps`
per episode. A converged `api.move` costs what it costs; a **starved** one
spends its whole cap (`max(60, 20*seconds)`) and still misses. Two of my
versions were wrecked by this and I initially misread both as geometry:

- v5 hit **exactly 1000 steps on 13 of 15 seeds**. On ep51 v5 and v6 put the eef
  within 0.3 mm of the same release pose; v5 spent 1000 steps and scored 0, v6
  spent 783 and scored 1. v5's release and settle were simply never simulated.
- v3's own two losses (ep60, ep64) also read 1000.

Corollary learned the hard way in v7: trimming `seconds` is *inert* — it lowers
a cap, not a cost. The 14 standing seeds reported byte-identical `sim_steps`
under v6 and v7. The only way to buy budget is to stop issuing moves that
starve.

## Version chain

### v1 — perception probe (4 seeds), no motion
Dumped cam_high/cam_arm_wrist RGB + depth through `api.log` (zlib+base64,
1900-char chunks) so all the scene geometry above could be derived offline at
zero sim cost. v1b repeated it from the parked pose on seeds 54/60/62/64.
Verdict: scene mapped; the demo-frame offset identified.

### v2 — open-loop top-down grasp + tilt-and-release → **3/8 probe**
`results/fs_…_v2` (seeds 51,53,…,65).
Evidence: the descent **never reached the commanded pose** — every episode
logged residual 0.069, landing +0.025 in x and +0.065 in z (the arm cannot
reach that low this close to its base). 4 of 8 closes were empty (width 0.001,
effort 0.05 vs 0.008/3.0 holding); ep59 lost the bottle mid-carry. The three
that scored did so by accident: their lower move starved completely (residual
0.11, eef unchanged) and dropped the bottle from 0.15 m up.
Verdict: refuted as written; the defect is aim and verification, not the plan.

### v3 — closed-loop aim + hold check + geometric lay-down → 8/8 probe, **13/15 formal**
`results/fs_…_v3` (8/8) and `results/sel_…_v3` (13/15).
(a) grasp the **neck** at `top − 0.025`, under the cork collar, a height the arm
can reach; (b) `aim()` re-commands from the measured eef error until under 4 mm,
cancelling the repeatable tracking bias; (c) verify the hold by gripper width
(>0.003), retry up to 3×; (d) derive the release from the bottle's own geometry
— `hg = z_close − 0.901`, `L = top − 0.901`, `dmid = hg − L/2`, so the eef target
puts the bottle's **centre** over the rack plane + 0.035; (e) descend in 0.030
steps because one long pos+rot `move_pose` starves.
Evidence: all 8 probe seeds grasped on attempt 0 with aim error < 2.5 mm and
held effort 3.0 through the carry.
Losses: ep64 (bottle inside the 0.26 m eef box → perception fell through to the
stove knob → grasped the bowl) and ep60 (1000 sim steps — budget, not physics).
Verdict: mechanism confirmed.

### v4 — park + height test + finer descent + up-slope bias → **9/15**
Four changes at once; `results/sel_…_v4`. Regression. The logs isolate it: the
0.9 cm up-slope shift (UPSLOPE_FRAC 0.50→0.40) flipped 62/63/65 with every other
number unchanged, and the 0.022-step/two-stall descent pushed several episodes
toward the horizon. Verdict: both motion changes refuted; the shelf midpoint is
right.

### v5 — v4 with the bias reverted + lying branch → **2/15**
`results/sel_…_v5`. Looked inexplicable until `sim_steps`: 1000 on 13 of 15.
The finer descent's extra starved moves exhausted the horizon before release.
Verdict: refuted, and the true constraint identified.

### v6 — v3's motion verbatim + only the perception fixes → **14/15**
`results/sel_c2clean_goal_put_wine_on_rack_pos_k3_v6`. Descent block diffed
byte-for-byte against v3. Imported: park the hand at (−0.05, 0.33, 1.30) before
each capture and delete the eef box; refuse candidates below table+0.11; the
lying-bottle branch. Seeds 51–63, 65 all pass; ep60 recovered (970 steps).
Remaining loss ep64: the lying **grasp worked** (width 0.044, effort 3.0) but the
single large lying→slope `move_pose` starved, leaving the eef 0.05 m high and
0.17 m above the release target.

### v7 — budget trim + stepped lying reorientation → **14/15**
`results/sel_…_v7`. The 14 standing seeds are byte-identical in `sim_steps` to
v6, proving the `seconds` trims are inert. ep64 got worse in kind: removing the
intermediate carry waypoint left one 0.30 m transit that starved at z=1.086 and
stranded every later move. Verdict: reorientation stepping is right, the transit
shortcut is wrong.

### v8 — starvation-proof lying transit → **14/15**
`results/sel_…_v8`. Standing path bit-identical to v6/v7. Lying path: lift 0.32
clear before translating, re-command the transit up to 3× until within 0.030 of
the carry pose, loosen the lying aim (tol 0.012, 2 tries) since that grasp sits
on the reach boundary and never converges to 4 mm. The transit was fixed, but the
lying→slope rotation then dragged the eef **up** 0.12 m across its sub-steps and
jammed the wrist at z=1.474, so the bottle was let go 0.27 m above the shelf.

### v9 — lying lay-down by yaw only → **14/15**
`results/sel_…_v9`. Rotating a horizontal bottle onto the slope needs ~70° of
yaw *and* 32° of pitch and the wrist cannot hold position through it. Keeping
only the yaw — a rotation about the tool's own approach axis — lines the bottle
up with the rack's slope line while leaving it horizontal; gravity does the rest
on a 30° ramp. Receipt: the yaw held the eef to within 1 mm, the descent reached
1.2419 against a 1.2283 target, the hand still held at effort 3.0 over the rack
centre. ep64 lost **only on budget** — 1000 sim steps, release never simulated.

### v10 / v11 — lower lying carry → **14/15 each**
`results/sel_…_v10`, `…_v11`. Tried to buy budget by carrying the lying bottle at
1.278 (rack top 1.213) instead of 1.34. Worse in kind: the eef froze
bit-identical at z=1.2837 from the transit through rot1–3 and six descent moves.
That is the hand **jammed against the rack** — its observed top is 1.213 but the
posts run higher — not starvation. v11 confirmed it: restoring 3 rot sub-steps
changed nothing while the carry stayed low. Verdict: the high carry is required.

### v12 — coarse lying descent, high carry → **14/15** (FROZEN)
`results/sel_…_v12` and `results/sel_c2clean_goal_put_wine_on_rack_pos_k3_final`.
Keeps v9's high carry and buys the budget where it is free: the lying descent
covers 0.11 m, so 0.055 steps instead of 0.030 removes three moves. The descent
now completes in 4 moves to 1.2413 (+13 mm over target) and the release
executes. The descent's contact test is also tightened from ez+0.06 to ez+0.035,
since every scoring seed stops between ez+0.012 and ez+0.028 — a stall 0.055
high is starvation, not contact.

## Mechanism-gap stop: the pre-tipped bottle (ep64)

Falsifiable statement of what is missing: **there is no lay-down for a bottle
that starts on its side that both fits the 1000-step horizon and leaves the
bottle inside the rack's success region.** Every component is individually
demonstrated on ep64 and the receipts are in
`results/sel_c2clean_goal_put_wine_on_rack_pos_k3_final/program_ep64.log`:

- detection — dark cluster 0.147 × 0.040, elongation 3.65, top table+0.042;
- grasp — closed across the axis at width 0.0443, effort 3.0 (a body grasp);
- transit — reaches (0.043, −0.226, 1.343), within 0.030 of the carry pose;
- yaw — −70.1° applied in three sub-steps, eef moves under 1 mm;
- descent — 1.2413 against a 1.2283 target, still holding at effort 3.0 over
  the rack centre;
- release — executes (the eef moves after `grip(0.08)`).

The episode still reads 1000 sim steps, so the settle is truncated, and the film
shows the bottle ending up teetering against the rack's back post rather than
bedded in a shelf. The two remaining costs are structural, not tunable: the
lying grasp sits on the arm's reach boundary (its `aim` never converges below
0.010, burning two full-cap iterations), and the high carry that is mandatory
for clearing the rack forces a long descent. Closing this would need a mechanism
I could not derive from this pack — either standing the bottle up first (an
extra grasp the budget cannot afford) or a lay-down that approaches the shelf
from downhill instead of from directly above.

This is one debug seed with a qualitatively different initial condition: the
bottle is upright at reset on 14 of 15 seeds and tips during the env's own
settle on this one. The standing path is unaffected — it is byte-identical
across v6–v12, reporting the same `sim_steps` on all 14 seeds in six independent
formal runs.

## Why v12 and not v6

v6 through v12 all score 14/15, and their standing paths are provably identical
(same `sim_steps` per seed, descent block diffed against v3). They differ only
inside `if lying is not None`, a branch that fires only when no standing
candidate exists — exactly the case where v6 scores 0 regardless. v12 therefore
weakly dominates: it cannot do worse on a standing bottle and gets much further
on a tipped one, which matters on 50 unseen eval seeds.

## DECLARATION

- **Frozen version:** `program_v12.py` → `packs/c2clean_goal_put_wine_on_rack_pos_k3/program.py`
  md5 `3d8b52dccad79a44293f9817c9ae8c36` (both files, verified on the cluster).
- **Selection receipt (full 15 debug seeds, frozen bytes):** **14/15** —
  `results/sel_c2clean_goal_put_wine_on_rack_pos_k3_final`.
  Passing: 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 65. Failing: 64.
- **Receipt chain:** v2 3/8 probe (`fs_…_v2`) → v3 8/8 probe (`fs_…_v3`) and
  13/15 (`sel_…_v3`) → v4 9/15 → v5 2/15 → v6 14/15 → v7 14/15 → v8 14/15 →
  v9 14/15 → v10 14/15 → v11 14/15 → v12 14/15 → final 14/15. All version files
  archived as `program_v1.py` … `program_v12.py` (plus the `program_v1b.py`
  perception dump) in the pack directory.
- **PROVENANCE:** present and complete — every module-level constant
  (TABLE_Z, DARK_GREY, BOTTLE_FOOT_MAX, NECK_DROP, TILT_DEG, CARRY_Z, LAY_CLEAR,
  HOLD_WIDTH, AIM_TOL, PARK, MIN_TOP, LYING_LEN, LYING_HOLD, LYING_GRIP_DROP,
  ROT_STEPS, LYING_LIFT, LYING_REST, LYING_DOWN_STEP, TRANSIT_TOL, R_DOWN, plus
  RACK_WOOD and LYING_YAW_ONLY) is declared with an allowed source drawn from
  the pack or from debug-seed measurement. Verified by replicating
  `fair_run.scan_program`'s checks: no forbidden tokens, no `.done` attribute
  read, every entry `allowed: True` with a source.
- **Splits:** only seeds 51–65 were ever run, always under `--split debug`.
  Seeds 1–50 were never touched; `--split eval` was never invoked.
