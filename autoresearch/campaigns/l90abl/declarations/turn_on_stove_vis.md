# l90abl / turn_on_stove_vis — NOTES

Intent: "turn on the stove". Pack = K=3 vision-only demos (keyframe PNGs + language),
no EEF/gripper/action data.

## What the pack shows
128x128 agentview keyframes. Scene: black frying pan (image left), moka pot (centre),
flat stove slab (right) with a small black knob standing behind/above the slab's back
edge. Across all three demos the arm descends onto that knob (t~43/47 of ~90) and by the
final frame the burner disc renders red. Nothing else in the scene is touched. So the
pack tells me: the target is the knob, the approach is top-down, and the whole task is
one knob actuation.

## Versions

### v1 — perception probe (seeds 51,53) — 0/2, no motion
Purpose: map the scene. Vectorised the deprojection locally from
frame.depth/intrinsics/t_base_cam (per-pixel api.deproject would be one socket
round-trip each).
Evidence: table plane z=0.900. cam_high K=618.04, centre (256,256); T puts image-down
along base +x and image-right along base +y. Coarse 5 cm heightmap:
frying pan y in [-0.35,-0.15] h=0.04; moka pot y~[-0.05,0.00] h=0.13-0.15; stove slab
y in [0.10,0.30] h=0.03. Verdict: props separate cleanly in y; the knob is below the
5 cm grid resolution.

### v2 — fine heightmap + first grasp (seeds 51,53,55) — 0/3
Hypothesis: knob = tallest thing on the stove side.
Evidence: WRONG — "tallest, y>0.05" selected the ROBOT ARM (z up to 1.342, mean rgb
[28,29,92], the blue link). The gripper closed on air at z=1.36.
But the 2 cm heightmap did resolve the knob: a 0.059-0.060 m plateau at x in
[-0.25,-0.15], y in [0.17,0.23] — behind the slab's back edge (slab x in [-0.15,0.05],
top 0.032). Verdict: identify the knob by a HEIGHT BAND, not by argmax.

### v3 — height band [0.042,0.115] + straddle + yaw sweep (51,53,55) — **3/3**
Band n=545, x [-0.245,-0.163], y [0.185,0.211], z top 0.960 (h=0.060), mean rgb
[48,48,48] (black), pixel bbox u [360,377] v [222,258] — matches the knob in the
keyframes. Descent ladder to eef = knob_top-0.025 then grip(0.012) gave
width 0.0253 / effort 3.0 => the jaws are closed ON the knob (25 mm across).
Then yaw -30/-60/-90, then -45/0/+45/+90. Succeeded, but which motion did it?
741 sim steps.

### v4-v7 — ablation of the actuating motion (51,53,55 each)
| variant | motion | result |
|---|---|---|
| v4 | descend only, gripper open | **0/3** |
| v5 | descend + close | **0/3** |
| v6 | descend + close + yaw -30/-60/-90 | **0/3** |
| v7 | descend + close + yaw +30/+60/+90 | **3/3** |

Verdict (the load-bearing finding): the knob is a revolute joint whose ON direction is a
POSITIVE yaw of the tool about the world z axis. Pressing on it does nothing, closing on
it does nothing, and the opposite sense does nothing. v7 costs only 138 sim steps.

Note: the burner never renders red in the recorded gif even on a successful episode
(zero pixels pass a red test on all 120 frames), so the film strip is NOT a usable
proxy for the predicate here — only results.jsonl is.

Also observed: api.move with a rotation starves — residual is identical at yaw 60 and
yaw 90 (0.0186), i.e. the last request did not converge. v8 therefore walks the yaw in
20 deg increments at 2.5 s so each request can converge.

### v8 — hardened v7
Changes over v7: band widened progressively if the first band is thin (fallback bands
[0.038,0.14] then [0.032,0.16]); knob centre taken from the top 12 mm slice of the band
rather than the whole band; yaw walked +20..+120 in 20 deg steps; and a second complete
attempt seated 7 mm deeper. The second attempt is free of risk because LIBERO sets
terminated=True the moment check_success() fires and heron.robot.libero._step_env then
returns immediately, so every later command is a no-op and task_success is sticky.
The scene is deliberately NOT re-perceived for attempt B: after the retreat the arm
hovers over the knob and would pollute the height band (the v2 failure mode).

### v9 — column-max height map replaces the band (51,52,53,54,55,57) — **6/6**
Diagnosis of the v8 failures (seeds 52 and 54): a height BAND keeps every pixel whose
point lies in [table+0.042, table+0.115], and a tall prop contributes flank pixels at
every height on its way up. On seeds 52/54 the moka pot sits at y≈0.06, inside the
stove-side window, so the band contained the pot's flank and its 0.111 m top; the "top
12 mm slice" then centred on the pot. The logs show it plainly: `knob kx=0.046
ky=0.061 ktop=1.0115 h=0.111`, and the close returned width 0.001 / effort 0.05 —
the jaws met nothing, and every yaw was a no-op in mid-air.
Fix: build a top-down COLUMN-MAX raster (1 cm cells, each cell keeps only its tallest
point) and accept cells whose column tops out in [0.048, 0.085]. A prop taller than
that never contributes a cell, so the pot and the arm drop out by construction rather
than by a y-window guess. Then 8-connected components, filtered on footprint.
Evidence: 6/6, including both former failures. The knob component is identical on every
seed: n=26-30 cells, extent 0.09 x 0.03 m, top exactly 0.060 m, mean rgb ~[46,46,46].

### v10 — tighten the component tiebreak — **15/15 (selection)**
v9 ranked shape-passing components by size alone, and on every seed there is a second
passing component (the moka pot's low flank ring) with n=13-21. Margin 26 vs 21 is thin.
Two measured separations exist and v10 uses both: the decoy's y-extent is 0.06-0.11 m
against the knob's 0.03 m (MAX_YEXT 0.16 -> 0.05), and the decoy tops at 0.055 m against
the knob's 0.060 m (MIN_TOP_H 0.057). Selection ranks tier1 (shape ok AND tall) first,
falling back to tier2 (shape ok) then tier3 (largest component) so a novel scene still
gets an answer.
Receipt: results/sel_l90abl_turn_on_stove_vis_v10 = **15/15** on debug seeds 51-65.

## Candidate law (for LAWS.md)
A knob-style revolute fixture is actuated by a wrist YAW with the jaws closed on it, and
the ON sense is one specific direction: pressing on the knob, and closing on it, and
yawing the wrong way all score 0/3 while the correct yaw scores 3/3. Ablate the
direction before hardening anything else — it is one 3-seed run and it is the whole task.

A height BAND is the wrong primitive for isolating a short prop when a tall prop shares
the window: the tall prop's flank passes through the band. Raster the scene into a
top-down COLUMN-MAX height map first and select on the column top; then a prop is
represented by its own height and nothing else's.

## DECLARATION
- Frozen version: **v10**. `packs/l90abl_turn_on_stove_vis/program.py` md5
  `d0cb0aa158c0060291e422061e8e988d` == `program_v10.py` (verified on the cluster and
  locally).
- Selection receipt: **15/15** on the full 15 debug seeds 51-65,
  `results/sel_l90abl_turn_on_stove_vis_v10`.
- Receipt chain: v1 0/2 (perception only, no motion) - v2 0/3 (band argmax grabbed the
  robot arm) - v3 3/3 (first working straddle+yaw, 741 steps) - v4 0/3 descend only -
  v5 0/3 descend+close - v6 0/3 yaw negative - v7 3/3 yaw positive (the ablation that
  identified the mechanism) - v8 8/8 probe but 13/15 selection (moka pot hijacked the
  band on seeds 52,54) - v9 6/6 column-max fix - v10 15/15 selection.
- PROVENANCE: present in program.py, 10 entries, every calibrated constant sourced to
  this pack or to a debug-seed measurement.
- Cost: ~135 sim steps per episode.
