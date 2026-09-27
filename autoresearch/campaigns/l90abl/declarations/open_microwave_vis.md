# l90abl / open_microwave_vis — worker notes

Pack modality: **vis** (K=3 demos, first + last keyframe RGB only, 128x128; no EEF /
gripper / action data). Intent: "open the microwave".

## Scene facts re-derived on debug seeds (v0 probe, seeds 51/53/57/61)

cam_high: K = f 618.04, c (256,256), 512x512. t_base_cam puts the camera at
(0.659, 0, 1.610) with image +u = base +y exactly and optical axis
(-0.778, 0, -0.628). Table plane z = 0.901.

Microwave: a box on the table, x in [-0.17, +0.18], y in [-0.36, -0.14],
z in [0.90, 1.118]. Its **door faces +y** and the door plane sits at y ~ -0.14.
The **handle is a C-shaped bar**: two stubs off the door face and a vertical bar
spanning z 0.951-1.092 standing ~30 mm proud of the door in +y, bar axis at
(x ~ -0.06, y ~ -0.11). Colours are randomised per episode (blue microwave in the
eval env vs black in the pack keyframes), so all cues are geometric.

Handle detector used everywhere below: take the on-table structure
(0.93 < z < 1.12, y < 0, -0.5 < x < 0.4); the y-histogram has a hard edge at the
door plane (92nd percentile); everything beyond that edge + 5 mm is the handle.
Bar centre = median of the middle 40% (in z) of those points, stepped back along the
camera view direction by BAR_R = 0.009.

Pack keyframes agree: t0 shows the closed microwave with the bar handle on its
right (= +y) face; the final frame shows the door swung ~90 deg out over the table.

## Version log

### v0 — pure probe (no motion). 0/4, by construction.
Recorded RGB-D + extrinsics. Gave every geometric constant above.

### v1 — top-down pinch, fingers along base x (R_X), z-sweep from above, then pull +y.
Hypothesis: the vertical bar can be pinched top-down and the door dragged out.
Evidence: the z-sweep broke on its FIRST success at eef z = 1.09, which is the
**top stub** of the C (a y-axis cylinder), not the vertical bar. Pulling +y then slid
the gripper straight off along the stub axis: width 0.0093 -> 0.001, effort 3.0 -> 0.05
on the very first 25 mm step. 0/4.
Verdict: grasp the MIDDLE of the vertical bar, not the first thing the sweep hits.
Also bracketed the fingertip offset: eef z 1.13 grabs nothing (bar top 1.092),
eef z 1.09 grabs -> offset < 0.038 m.

### v2 — same pinch but at bar mid-height (zmid+0.02), then 20 x 10 mm pulls in +y.
Evidence: grasp 4/4 (w = 0.0227, effort 3.0). The door dragged 7.8 cm
(door plane y -0.146 -> -0.068) and then the bar levered the fingers apart
(w 0.0227 -> 0.0403 over steps 7-12) and popped out at step 13, identically on all
four seeds. 0/4.
Verdict: the pinch is strong enough; a straight +y pull is not the door's arc, so the
residual grows radially and pries the jaws.
This run also fixed the **hinge side**: while gripping, the handle drifted +x as y
increased. A -x hinge predicts -x drift, so the hinge is the **+x edge** of the door
plane, i.e. (x = box.xhi, y = box.yface); the door rotates clockwise about +z.

### v3 — hook: closed fingers inserted into the 28 mm slot between bar and door, pull +y.
Evidence: the insertion never happened — residual 0.040, eef stopped 20 mm short in x
and 29 mm high; the door did not move (face -0.146 -> -0.142). 0/4.
Verdict: the blade does not fit / the hand fouls. Hook abandoned.

### v4 — v2 grasp + circular arc about the (xhi, yface) hinge with the wrist yawing
with the door (R = rz(-a) @ R_X), 40 x 3 deg.
Evidence: **2/4 (seeds 51, 53 succeed; 57, 61 fail).** The grip now survives the whole
sweep (w pinned at 0.0227 out to 90 deg) — the yaw fix works. All four doors stall at
the same place, handle ~8.5 mm short of the hinge x, i.e. ~79 deg of door rotation;
51/53 fire the predicate there, 57/61 do not. On the failures the commanded point ran
away from the stalled eef (target x 0.207 vs eef 0.166) so the lead became a radial
pull and the bar popped out, after which the arm sank (z 1.061 -> 0.970).
Verdict: keep the arc + yaw; bound the lead so the command stays tangential.

### v5 — arc servo: each step commands a bounded angular lead (<= 10 deg) from the
CURRENT eef angle, radius clamped to [R0-0.03, R0], 60 x 3 deg, stop after 3
consecutive steps without grip. (running)
Evidence: **8/8** on the probe subset (51,53,55,57,59,61,63,65,
`results/fs_l90abl_open_microwave_vis_v5`), then **15/15** on the full debug split
(`results/sel_l90abl_open_microwave_vis_v5`). Grip width stays pinned at 0.0227 with
effort 3.0 for the whole sweep on every seed; the predicate fires at ~78 deg of door
rotation, before the door stalls. 365-544 sim steps per episode.
Verdict: the bounded tangential lead is the fix — it is the difference between
v4 (2/4) and v5 (8/8).

### v6 — FROZEN. Code identical to v5 (only the debug-dump path differs); the
PROVENANCE dict was completed to cover DPHI, MAXLEAD, the radius clamp, NSTEP, the
stop rule and the hinge choice, which v5 had left undeclared.
Evidence: **15/15** on the full debug split, `results/sel_l90abl_open_microwave_vis_v6`.

## Mechanism, in one paragraph

A microwave door is a one-DoF revolute joint whose axis this pack never states. Two
things had to be recovered from the robot's own sensors: **which edge is the hinge**
(the sign of the handle's x-drift under a straight pull settles it — the drift is
toward the hinge side), and **how hard the command may lead the eef**. A top-down
pinch on the middle of the vertical handle bar is plenty strong (effort 3.0 the whole
way), but it is a friction pinch on a cylinder, so it only survives if the commanded
force stays tangential to the door's arc. Two things break that: not yawing the wrist
with the door (v2/v4 fix), and letting the commanded point run away from a slow or
stalled eef, which turns the lead into a radial pull that pries the jaws open (v4 ->
v5 fix). With both in place the grasp never slips and the door opens on every seed.

## DECLARATION

- Frozen version: **v6**. `packs/l90abl_open_microwave_vis/program.py`
  md5 `dfd479c23a0088413a8c91d0e8412ce9` == `program_v6.py` (verified on cluster).
- Selection receipt (full 15 debug seeds 51-65):
  **15/15** — `results/sel_l90abl_open_microwave_vis_v6`.
  Prior full-15 receipt for the identical mechanics: 15/15 —
  `results/sel_l90abl_open_microwave_vis_v5`.
- Receipt chain: v0 0/4 (probe, no motion) · v1 0/4 · v2 0/4 · v3 0/4 · v4 2/4 ·
  v5 8/8 probe then 15/15 full · v6 15/15 full.
  Probe dirs `results/fs_l90abl_open_microwave_vis_v{0,1,2,3,4,5}`.
- Every formally probed version is archived as `program_vN.py` in the pack dir.
- PROVENANCE: present in program.py, a top-level literal dict covering every
  calibrated constant (band limits, BAR_R, finger-axis choice, the yface percentile,
  the handle mid-band rule, the grasp-z candidates, the hinge definition, DPHI,
  MAXLEAD, the radius clamp, NSTEP and the stop rule). All sources are this pack's
  keyframes/language or debug-seed (51-65) observations.
- Clean room: no eval seed (1-50) was ever run; no .bddl/.xml/.hdf5/init_state was
  read; no other cell's pack, program or notes was read; only
  `packs/l90abl_open_microwave_vis/*` and `results/*l90abl_open_microwave_vis*` were
  written.
