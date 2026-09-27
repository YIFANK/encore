# l90abl / open_top_drawer_s2_vis — NOTES

Intent: "open the top drawer of the cabinet". Pack = fair-pack-v1-vision,
K=3 demos, keyframe IMAGES + language only (no eef / gripper / action data).

## Pack reading (images only)
9 keyframes, 128x128, agentview. Each demo: t0 (home), mid (arm down at the
cabinet, left of frame), final (a drawer box protruding out of the cabinet
toward image-right). The cabinet sits at the image-left edge of the table with
three stacked handles on the face that points image-right. So the demo
sequence is: reach the cabinet face, engage the top handle, pull toward
image-right. The pack carries no metric information; every number below comes
from my own debug-seed sensing.

## v0 — perception dump (probe, no motion). results/fs_..._v0 (0/8, no motion attempted)
cam_high extrinsics: camera at base (0.659, 0, 1.610), optical axis toward
-x/-z; image-right = base +y, image-down = base +x, so the drawer face points
+y and the drawer must be pulled in +y.

Measured from the cam_high point cloud (seed 65 dump):
- table top z = 0.901
- cabinet drawer panel plane: y = -0.229, x in [-0.119, 0.122] (width 0.241),
  top surface z = 1.127, bottom at the table.
- three handle bars protruding to y = -0.198 (0.031 m in front of the panel),
  each spanning x in [-0.036, 0.052] (bar length 0.088, centre x = 0.008):
    bottom bar z 0.940-0.955, middle bar z 1.005-1.025, TOP bar z 1.082-1.098.
- zoomed RGB: each handle is a cylindrical bar (dia ~0.016) on two end
  standoffs, so the slot between the bar's middle and the panel is open from
  above. Slot depth ~ 0.031 - 0.016 = 0.015 m.

Target = the top bar, centre approx (x 0.008, y -0.198, z 1.090).

Open question that decides the strategy: api.grip is binary (open ~0.078 /
closed), so a two-finger pinch around a bar only 0.031 m proud of the panel is
geometrically impossible unless the finger blade is thin. Hence v1.

## v1 — measurement probe (in flight)
Measures (a) the panel/handle geometry per seed, (b) the fingertip z-offset
below the reported eef (press a closed gripper onto bare table at (-0.28,
0.34), which the v0 cloud confirms is clear), (c) a descent sweep at
y = y_face + dy for dy in {4,7,10,13,16,20} mm with the tip at bar height —
the dy values that reach the commanded z are the ones where the closed blade
fits the slot.

Result of v1: the sweep is BLOCKED at every dy. All six descents stall at
eef z = 1.1375 (fingertip z = 1.129, i.e. exactly the cabinet-top plane at
1.127), so the closed gripper rests on the cabinet top instead of entering the
slot. Also learned:
- tip offset: pressing a closed gripper onto bare table stalls with the eef
  0.0085 m above the table, so the eef reference sits essentially at the
  fingertips.
- horizon = 1000 sim steps (results.jsonl sim_steps), and STEPS_PER_SECOND=60,
  so api.move(seconds=2.5) can burn 300 steps when it is blocked. v1 exhausted
  the horizon after the second sweep entry -- the later rows are stale repeats.
- per-seed variation: the bar z is fixed (top bar top face z = 1.098 on every
  seed) but the cabinet translates in y by ~13 mm and in x by ~15 mm across
  seeds, so the geometry must be perceived every episode, not hard-coded.
Verdict: a top-down blade cannot reach the slot. The slot is ~0.015 m and the
closed finger pair is wider than that; worse, the approach column is roofed by
the cabinet top.

## v2 — finger-geometry dump (probe, superseded)
Parked the gripper over bare table and dumped cam_high open/closed. Ran, but
v3 answered the question first, so this was never analysed.

## v3 — horizontal bar grasp. results/fs_..._v3 = 4/4 (seeds 51,53,55,57)
Hypothesis: the bar does not have to be hooked, it can be GRASPED, if the tool
is turned so the approach axis is -y and the finger axis is vertical. The open
jaws span 0.078 m; there is 0.029 m of free air above the top bar (up to the
cabinet top) and 0.058 m below it (down to the middle bar), so a fully open
gripper centred on the bar straddles it without touching anything, and closing
captures it.
  R_HORIZ = [[1,0,0],[0,0,-1],[0,1,0]]  (tool z -> base -y, tool y -> base +z)
Evidence: the wrist reached the pose with residual 0.011 and the achieved
rotation matched R_HORIZ to 0.013; the close left a finger gap of 0.0174 m
with effort 3.0 (i.e. the jaws are holding the 17 mm bar, not air); the pull
carried the eef from y=-0.187 to y=-0.049 (0.139 m) with the grip intact, and
the drawer stopped it there. 4/4 benchmark_success.
Verdict: this is the mechanism. The pull direction and the "grasp, don't hook"
insight are the whole task.

## v4 — v3 + proprioceptive verification and retries. FROZEN
Same mechanism, plus: (a) no debug dumps, (b) after the close, the grip is
accepted only if 0.005 < gap < 0.040 and effort == 3.0, (c) after the pull,
success is asserted only if the held eef travelled >= 0.08 m in +y, (d) up to
three engage attempts, each re-perceiving the cabinet and nudging the target
by (dz,dy) in {(0,0), (0,-0.006), (0.006,0)}. Three attempts fit inside the
1000-step horizon.
Formal selection run (all 15 debug seeds): **15/15**, dir
`results/sel_l90abl_open_top_drawer_s2_vis_v4`. Every seed succeeded on
attempt 0, travel 0.1377-0.1397 m, closed gap 0.01740 +/- 0.00001, effort 3.0.
The retry ladder never fired on the debug band; it is insurance for the blind
band. Success GIF: `ep51_ok_v4.gif` (drawer visibly out, jaws still on the bar).

# DECLARATION

- Frozen version: **program_v4.py**, copied to
  `packs/l90abl_open_top_drawer_s2_vis/program.py`.
  md5 (both files) = `ce9e6f841a44051e788f548ddffdfdae`.
- Selection receipt: **15/15** on the full 15 debug seeds (51-65),
  `results/sel_l90abl_open_top_drawer_s2_vis_v4/results.jsonl`,
  counting `"benchmark_success": true`.
- Receipt chain:
  - v0 perception dump  — `results/fs_l90abl_open_top_drawer_s2_vis_v0` (0/8, no motion)
  - v1 measurement probe — `results/fs_l90abl_open_top_drawer_s2_vis_v1` (0/4, no attempt)
  - v2 finger dump       — `results/fs_l90abl_open_top_drawer_s2_vis_v2` (0/2, no attempt)
  - v3 first real attempt — `results/fs_l90abl_open_top_drawer_s2_vis_v3` **4/4**
  - v4 selection         — `results/sel_l90abl_open_top_drawer_s2_vis_v4` **15/15**
- PROVENANCE: present in program.py, 11 entries, every calibrated constant
  sourced to this pack's keyframes, debug-seed (51-65) sensing, or generic
  tool-frame/controller mechanics. `tools/fair_run.py:scan_program(..., "eval")`
  run against the frozen file: PASS.
- No forbidden reads: only `packs/l90abl_open_top_drawer_s2_vis/pack.json` +
  `keyframes/`, this cell's own `results/*l90abl_open_top_drawer_s2_vis*`, and
  the harness sources `tools/fair_run.py`, `tools/fair_client.py`,
  `heron/robot/libero.py`. `--split eval` was never invoked; seeds 1-50 were
  never touched.
