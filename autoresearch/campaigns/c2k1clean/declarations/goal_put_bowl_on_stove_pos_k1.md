# c2k1clean / goal_put_bowl_on_stove_pos_k1

Intent: "put the bowl on the stove". Runner: tools/fair_run.py only. K=1 pack
(one demo, 4 keyframes, 94 actions). No shared note file.

## Scene, as read from the pack + debug seeds 51/53/55 (v1 probe)

cam_high is a fixed 512x512 RGB-D view, K = diag(618.04) c=(256,256),
t_base_cam translation (0.6586, 0, 1.6104): the camera sits at +x and looks
back toward -x and down. Image-right is +y. Table plane z = 0.9014 (the median
of the cropped cloud; ~50% of all points).

Props (debug band): black wooden cabinet at y in [-0.35,-0.12] rising to
z=1.244; a wine bottle (top 1.059); a black kettle (top 0.960, solid);
a flat plate (top ~0.918); the target bowl (footprint 0.110x0.111, rim top
0.952 when flat); the stove = a 0.19x0.19 m slab whose top plane sits
24.4 mm above the table (zmed 0.9258, sigma 2.3 mm) with a burner ring
rising to 0.932.

Discriminators measured on 51/53/55:
  * bowl vs kettle/bottle: hollowness of the top-down height map above
    table+40 mm. Bowl interior fill 0/96 cells (hollow 0.95-1.00); bottle
    13/13 (0.00); kettle 34/39 (0.10-0.13). Colour is NOT used: the pack
    keyframes and the debug seeds render the same assets with different
    textures (pack: white stove/dark bowl; debug: dark stove/silver bowl).
  * stove: the only >2000-point component in the table+[18,40] mm band with a
    0.13-0.30 m square footprint.

_pos perturbation is small here (bowl centre moves ~2 cm across 51/53/55) but
it changes the bowl's POSE: on 53/55 the bowl leans on the stove's near edge,
tilted ~20 deg (rim 0.937 on the -y arc, 0.975 on the +y arc); on 51 it sits
flat.

Demo geometry (pack.json): grasp at t=39, EEF (-0.0973, 0.0563, 0.9325),
release at t=87, EEF (-0.2729, 0.2605, 0.9451), closed finger gap 0.0123 m.
Deprojecting the bowl in keyframe demo0_t0000 through the debug-seed camera
model puts its rim centre at (-0.090, 0.013, 0.952): so the demo grasp is
12 mm INSIDE the rim line on the +y arc and 19.5 mm BELOW the rim top, with
the jaws closing radially (the default straight-down wrist closes along y).

## Versions

### v1 - perception probe (no motion), seeds 51,53,55
Dumped RGB + per-pixel base-frame xyz through api.log (zlib+base64). Gave
every number above. No success bit expected (0/3).

### v2 - first full attempt, seeds 51,53,55,57 -> 3/4
Rim pinch on the -y arc, carry at z=1.05, release with the bowl base 10 mm
over the slab top (hang measured from the held bowl's rim top). 53/55/57 ok
(136-138 sim steps each).
ep51 failed as a COLLISION, not a perception miss: the -y arc put the hand
beside the 0.34 m tall cabinet; the hover stalled 41 mm short, the descent
moved 1.4 mm with a 137 mm residual, and the gripper closed on air
(width 0.001 vs 0.0078 when holding). Verdict: the approach arc must be
chosen by clearance, and a grasp must be verified before the carry.

### v3 - clearance-chosen arc + verified grasp + retry, seeds 51..65 odd -> 2/8
Ranks the four rim arcs (+y,-y with the default wrist; +x,-x with a 90 deg
yaw) by the number of points taller than grasp_z+30 mm within 90 mm of the
arc point, prefers +y (the demo's arc) on ties, retries on a thin grip.
The collision is gone (+y is always chosen, -y scores 4400-6400 obstacle
points from the cabinet at htall=1.127) but the pinch itself fails: it
closes on the bowl (effort 3.0) and ratchets out during the lift,
0.0083 -> 0.0048 (ep51), 0.0089 -> 0.0048 (ep53). Verdict: the 12 mm inset
pinches the rim CREST, which is a wedge.

### v4probe - depth sweep (diagnostic), seeds 51,53,61
Two facts, both load-bearing:
  * the descent is CONTACT-limited, not tolerance-limited. Commanding
    rim-35 mm lands at rim-13 mm and re-issuing 7 mm lower changes nothing
    (0.9396 -> 0.9392 -> 0.9389). The fingertips sit ~38 mm below the EEF
    reference, so they are standing on the table (0.9389 - 0.038 = 0.901).
  * the flat bowl never leaves the table (ep51 gif: it sits beside the
    gripper through the entire lift).
Also: the table estimate must be the MODAL depth bin, not the median -- once
the arm is in view the median jumps 0.9014 -> 0.9258 and the "stove" detector
then locks onto the cabinet.

### v5probe - radial aim sweep (diagnostic), seeds 51,61,53
Gap at close -> gap after a 45 mm lift, on the +y arc:
  inset -8 mm: 0.00819->0.00813 | 0.00839->0.00832 | 0.00825->0.00680
  inset  0 mm: 0.00909->0.00902 | 0.00727->0.00691 | 0.00889->0.00860
  inset +12 mm: 0.00815->0.00807 | 0.00838->0.00386 | 0.00640->0.00631
Verdict: aim AT the measured rim crest (inset 0). (The 4th trial of each
episode is void -- the episode hit its horizon and the gripper froze.)

### v6 - inset 0 + contact-limited descent, seeds 51..65 odd -> 4/8
Flat bowls now work (51, 59, 61, 63); every tilted bowl fails (53, 55, 57)
and so does one flat one (65). The receipt separates cleanly on BITE DEPTH:
  holds closed 11.6, 15.3, 12.0, 11.2 mm below the rim crest
  losses closed 17.7, 20.1, 21.6, 27.0, 29.4, 30.1, 31.3, 31.8, 33.0 mm below
On a tilted bowl the outer finger falls past the bowl all the way to the
table, so a contact-limited descent overshoots the crest by ~32 mm and the
jaws close on the shell's flat under-belly, which slides out.

### v7 - tolerance-limited depth + re-perceived hold check  [FROZEN]
Two changes:
  * descend to rim-20 mm instead of rim-35 mm. The move is no longer
    contact-limited: POS_TOL (12 mm) stops it ~8 mm high, i.e. ~11-17 mm
    under the crest, whatever the tilt.
  * stop trusting the finger gap as the hold test. A tilted bowl is held by
    its shell at a gap of 0.0037-0.0045 m, BELOW the harness's own 5 mm
    "effort" flag -- v6 discarded exactly those grips as "closed on
    nothing". Instead the arm carries first, over the stove, and then
    re-perceives: the bowl's old footprint must be bare. Receipt across the
    8 probe seeds: 0, 6, 8, 3, 0, 0, 0, 0 points left against baselines of
    831-1182. No episode needed a retry arc.
Receipts: probe seeds 51,53,55,57,59,61,63,65 -> 8/8
          (results/fs_c2k1clean_goal_put_bowl_on_stove_pos_k1_v7)
          formal, all 15 debug seeds 51-65 -> 15/15
          (results/sel_c2k1clean_goal_put_bowl_on_stove_pos_k1_v7)

## DECLARATION

Frozen version: **v7**.
  packs/c2k1clean_goal_put_bowl_on_stove_pos_k1/program.py
  md5 a3a4f64e82b192fd36349479dc609b90 == program_v7.py (same md5, local and
  on the cluster).
Selection receipt: **15/15** on the full 15-seed debug band (51-65),
  results/sel_c2k1clean_goal_put_bowl_on_stove_pos_k1_v7 (15 lines in
  results.jsonl, all "benchmark_success": true).
Per-version chain: v1 probe (0/3, perception only) -> v2 3/4 -> v3 2/8 ->
  v4probe, v5probe (diagnostics) -> v6 4/8 -> v7 8/8 probe, 15/15 formal.
PROVENANCE: present, 13 entries, every one sourced to pack.json, a debug-seed
  measurement, or generic controller/camera mechanics; no api.done read
  (AST-checked).
No shared note file was read or written; no forbidden path was opened; every
constant above was re-derived in this cell.
