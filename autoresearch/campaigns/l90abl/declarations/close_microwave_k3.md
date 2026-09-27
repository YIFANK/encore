# l90abl close_microwave_k3 — NOTES

Intent: "close the microwave" (KITCHEN_SCENE6). Pack = K=3 demos.

## Pack reading (2026-09-09)
Keyframe images (128x128 agentview, upscaled offline) show a kitchen counter:
grey mug at image-left, YELLOW mug centre, an OPEN microwave door standing as a
black-framed panel with tan inner face across the centre-right, microwave body =
dark box at image right.

All three demos do the SAME two things:
 1. close the gripper on the yellow mug (~x 0.00, y -0.02, z 0.99-1.02) and carry
    it to ~(-0.20,-0.15,1.01), release. In the final frames the yellow mug sits
    next to the grey mug at image-left.
 2. with the gripper open, sweep the arm from about (-0.32,-0.05,1.04) through
    (-0.21,0.00,0.98) to (-0.10,+0.13,0.98) — i.e. a (+x,+y) push at z~0.98 —
    after which the door panel is gone from the image (closed).
Demo2 fumbles the first grasp (closes at t=68, reopens t=88, regrasps) but the
shape is identical.

Hypothesis H1: the yellow mug blocks the door's swing arc and must be cleared.
Hypothesis H2: only the push matters; the mug move is incidental.

## v0 — perception dump + naive replay (seeds 51,53,55)
Dumps cam_high intrinsics/extrinsics + 128x128 RGB & depth as hex to the log so
the scene can be reconstructed offline; then replays demo1's ee_path6 xyz with a
straight-down wrist (rotation=None) for a GIF baseline.

## v0 -- perception dump + naive replay of demo1's xyz (seeds 51,53,55)
0/3. Replay with a straight-down wrist jams at the door's free edge (waypoint 14
targets a point ON the door plane; residual grows 0.02 -> 0.21). Value was the
dump: cam_high K = f 618.04, c (256,256); t_base_cam puts image-right = base +y
and image-down = base +x; table top z = 0.9010.

Scene (offline reconstruction from the dumps):
  microwave box top = table+0.211, footprint x[-0.16,0.16] y[0.25,0.37], fixed
  across seeds; the open door is a thin panel of length 0.24-0.28 hinged at the
  box's -x front corner, open angle varies per seed (-80 to -110 deg); two mugs,
  tops at table+0.105; the arm at home occupies z > table+0.28.

## v1 -- geometry-derived mug clear + hinge arc.  0/8, "no door"
The band segmenter was validated on 4x-subsampled dumps where the door and body
are separate components; at full camera resolution they are ONE component. Bug
in method, not in the model.

## v1d -- diagnostic dump at full res
Confirmed the merge, and gave the fix: erode the top-slab footprint (band
[table+0.195,+0.245], which excludes the arm) -- the 1-2 cell door edge vanishes,
the box slab survives -- then the door is whatever the [table+0.14,+0.19] band
holds outside a skirt round that slab, and the hinge is where the door line
re-enters the slab. Stable on every seed tried (H ~ (-0.17,0.25), L 0.24-0.28).

## v2 -- robust finder + mug push + arc.  7/8 (fails 61) -- BUT SPURIOUS
GIF + logs: the transits ran at table+0.20 and the fingertips clipped the door's
top edge on the way past, knocking it shut before the intended push ever ran.
Two real measurements fell out:
  * the blocked-descent probe stops the eef 0.0078 m above the table -> the eef
    reference sits AT the fingertips and the hand rises ~0.10 m ABOVE it.
  * with the eef at table+0.165 the hand fouls the door's top edge: 0.08 m of
    standing residual right through the sweep.

## v3 -- tip offset 0.008 hardcoded, transits at table+0.26, push at table+0.085
4/8. Where the arc actually ran (ep51) it tracked to 0.01 m and closed the door
cleanly. The four failures all stall in the MUG-PUSH step: the radial push runs
+x and leaves the arm at x ~ +0.09, z ~ 1.01, after which it can move in y but
neither in -x nor up (residual 0.25-0.31 for the rest of the episode).

## v4 -- v3 with the mug step DELETED.  8/8 on 51..65 odd, 114-337 sim steps
## v5 -- v3 with the mug pushed away from the box centre (~-y).  8/8, 300-522 steps
Verdict on H1/H2: H2. The mug is NOT a blocker -- the door's swing disc reaches
it only in my first (wrong) hinge estimate. The pack's demos move the mug for
their own reasons; the benchmark predicate does not care. v4 is the argmax:
simplest, fastest, and it removes the only step that ever stalled the arm.

## v6 -- v4 with the dead mug code removed and a finder fallback added
Behaviourally identical to v4 (same sim_steps on all 15 seeds). Adds only: if
find_door returns None, fall back to the median debug-seed hinge/free-edge/box
footprint so a perception miss still attempts the push. Never triggered on the
debug band.

## Formal selection (full 15 debug seeds, 51-65)
  v4  15/15  results/sel_l90abl_close_microwave_k3_v4
  v5  15/15  results/sel_l90abl_close_microwave_k3_v5
  v6  15/15  results/sel_l90abl_close_microwave_k3_v6   <-- FROZEN
v6 is the argmax: it ties on success and is the leanest (108-389 sim steps of a
1000-step horizon, vs 282-574 for v5), so it carries the most headroom on unseen
seeds, and it has no step that ever stalled the arm.

## Candidate law (for LAWS.md)
An articulated panel is closed by sweeping a closed gripper along an ARC about
its measured hinge, standing off ~0.045 m behind the panel and entering past its
free edge -- never by a waypoint that lands ON the panel plane (v0 jammed there).
Two heights decide it, and both follow from one measurement: press the closed
gripper into the table and read where the eef stops. Here that is 0.008 m, i.e.
the eef reference IS the fingertips and the hand rises ~0.10 m above them, so
(a) transits must clear the tallest top by the fingertip height (v2 transited
0.01 m too low and knocked the door shut by accident, faking 7/8), and (b) the
push height must keep the HAND below the panel's top edge or it rides the edge
instead of pushing the face (v2/v3 at table+0.165: 0.08 m of standing residual
for the whole sweep; table+0.085: 0.01 m).

Second law: a K=3 pack can demonstrate a whole sub-task that the benchmark
predicate does not require. All three demos here pick up and relocate a mug
before touching the door; measured, the door's swing disc never reaches it, and
deleting the step took the debug band from 4/8 to 8/8 because the extra reach
stalled the arm. Test the pack's sub-goals against geometry before copying them.

## DECLARATION
Cell: l90abl / close_microwave_k3 ("close the microwave", KITCHEN_SCENE6, K=3).
Frozen version: program.py, md5 0507f94e90c587cb10469ba0f40b5b1d, byte-identical
to program_v6.py (verified on the cluster).
Selection receipt: 15/15 on the full debug band (seeds 51-65),
  results/sel_l90abl_close_microwave_k3_v6/results.jsonl
Per-version receipt chain (all on debug seeds only; eval band 1-50 never touched):
  v0  0/3   fs_l90abl_close_microwave_k3_v0    demo-xyz replay, jams on the door edge
  v1  0/8   fs_l90abl_close_microwave_k3_v1    finder returns "no door" at full res
  v1d  --   fs_l90abl_close_microwave_k3_v1d   diagnostic dump, no motion
  v2  7/8   fs_l90abl_close_microwave_k3_v2    spurious: transit clipped the door
  v3  4/8   fs_l90abl_close_microwave_k3_v3    correct heights; mug push stalls the arm
  v4  8/8   fs_l90abl_close_microwave_k3_v4    mug step deleted
  v5  8/8   fs_l90abl_close_microwave_k3_v5    mug pushed away from the box
  v4 15/15  sel_l90abl_close_microwave_k3_v4
  v5 15/15  sel_l90abl_close_microwave_k3_v5
  v6 15/15  sel_l90abl_close_microwave_k3_v6   FROZEN
PROVENANCE: present in program.py, 13 entries, every calibrated constant sourced
to this pack's demos or to a debug-seed measurement.
