# c2clean / obj_alphabet_soup_task_k0 — working notes

Intent: "Pick the cream cheese and place it in the basket".
No demo pack. Everything below is derived from debug seeds 51-65 only.

## v1 — perception datapipe (seeds 51,53,57,61)
Hypothesis: cam RGB-D can be exfiltrated through `api.log` so perception is
done offline at zero sim cost.
Evidence: works, but the logger truncates each `msg` at ~2000 chars; 3000-char
base64 chunks lost ~12 chars each and the stream would not inflate. Re-ran
(`v1b`) with 1800-char chunks -> clean zlib/base64 round-trip of 512x512x3 RGB
and 512x512 float16 depth.
Verdict: datapipe adopted. Chunk size 1800.

## v2 — geometry calibration (seeds 51..65 odd)
Hypothesis: pixel->base mapping is plain OpenCV pinhole composed with
`t_base_cam`, so the whole cloud can be built vectorised instead of calling
`api.deproject` per pixel.
Evidence: dumped a 7x7 grid of ground-truth `api.deproject` samples alongside
the frame. Fit over sign/flip conventions: `cam = ((u-cx)/fx*z, (v-cy)/fy*z, z)`
then `T_base_cam @ cam` reproduces the API to max error 7.6e-4 m (float16 depth
quantisation). No flips, no sign changes.
Verdict: adopted. Table plane sits at z = 0.001 m in base frame.

Scene (seeds 51/53, band 0.010 < z < 0.16, 8 mm grouping) — 7 clusters, and
the layout is nearly identical across seeds:
  n=11.9k  c=(+0.03,+0.26) ztop=0.144  grey    -> basket
  n= 3.0k  c=(+0.12,-0.19) ztop=0.142  red     -> milk carton
  n= 2.4k  c=(+0.16,+0.03) ztop=0.081  dark    -> can (bottom of view)
  n= 2.3k  c=(+0.06,-0.10) ztop=0.148  green   -> dressing bottle
  n= 1.5k  c=(-0.11,-0.24) ztop=0.082  blue    -> can with "S" label
  n= 0.5k  c=(-0.14,+0.06) ztop=0.020  blue    -> flat box, blue/white
  n= 0.4k  c=(-0.19,-0.08) ztop=0.019  orange  -> flat box, orange
Without the height band the robot arm fuses with the three far props.

## v3 — grasp mechanism, first end-to-end (seeds 51,53,57,61)
Hypothesis: perceive -> descend -> close -> carry -> release is enough.
Evidence: grasped the blue can (width 0.0556, effort 3.0 held through the
carry), released over the basket, benchmark_success False on 4/4.  The can left
the table, so the mechanism works and the *object* is wrong.
Verdict: mechanism sound; identity is the open question.

## v4 — identity test #1 (blue box / orange box), 4 seeds each
Evidence: both closed to width 0.001 (empty jaws) and post-drop re-perception
showed both boxes still in place.  0/4 and 0/4, but uninformative: the grasp
never touched anything.  Root cause found in v5.
Verdict: void as an identity test; motivated the fingertip calibration.

## v5 — fingertip calibration (seeds 51,53)
Hypothesis: the unknown fingertip-to-eef offset can be measured by descending
in 5 mm steps onto surfaces of known height (bare table, a prop top) and
watching where eef z stops tracking the command.
Evidence (seed 51): jaws closed and jaws open both stall descending on bare
table (z=0.001) at eef z=0.0089; closed jaws stall on the can top (z=0.081) at
eef z=0.0824.  So the fingertips sit ~0.005 m below the eef, and in free space
`api.move` settles 0.011 m above the commanded z.
Verdict: TIP_OFF = 0.005, TRACK_LAG = 0.011.  v4's 0.044 was wrong and is what
made both box grasps close on air.  (The v3 stall at eef 0.045 over the can was
the gripper body fouling the can top, not the tips reaching the table.)

## v6 — identity test #2 at the calibrated height, 4 seeds each
Hypothesis: the graded prop is one of the two flat boxes.
Evidence:  blue/white flat box  **4/4**  (grip width 0.042 = its short span)
           orange flat box       0/4     (grip width 0.039, carried and dropped)
           blue can              0/4     (v3 receipt)
Verdict: the graded "cream cheese" is the flat blue-and-white box at
c=(-0.14,+0.06).  Note the bddl filename names the alphabet soup; the graded
predicate follows the *instruction*, not the filename.

## v7 — hardened, frozen
Changes over v6: the target is selected among the flat props (ztop < 0.05) by
chromaticity B-R rather than by position (blue box +23.6, orange box -48.8, a
72-count gap); the basket is the largest footprint and the drop uses its bbox
centre; wrist yaw switches to closing across base x if the y span exceeds the
0.060 m the jaws can straddle; the close is verified by effort/width with a
re-perceive-and-bite-lower retry.
Evidence: probe 8/8 on seeds 51,53,...,65 (no retry fired, every close read
width 0.0422 / effort 3.00).  Formal full-15: **15/15**.
Aim envelope: displacing the aim +12 mm and -12 mm along the closing axis both
score 4/4 on seeds 51,55,59,63, against a seed-to-seed layout jitter of ~10 mm.
Verdict: FROZEN.

# DECLARATION

- Frozen version: **v7**.  `packs/c2clean_obj_alphabet_soup_task_k0/program.py`
  md5 `83b19ca2e7ce76a73f1a31c6f861cf86` == `program_v7.py` (same md5).
- Selection receipt: **15/15** on the full debug split (seeds 51-65),
  `results/sel_c2clean_obj_alphabet_soup_task_k0_v7`.
- Receipt chain:
  | version | what | seeds | result |
  |---|---|---|---|
  | v1/v1b | RGB-D exfil through api.log | 51,53,57,61 | datapipe works at 1800-char chunks |
  | v2 | deprojection fit vs api.deproject | 51..65 odd | max error 7.6e-4 m |
  | v3 | first pick-place, blue can | 51,53,57,61 | 0/4, can carried and dropped |
  | v4 / v4b | blue box / orange box, bad TIP_OFF | 51,53,57,61 | 0/4, 0/4 (empty jaws) |
  | v5 | fingertip calibration | 51,53 | tips 0.005 m below eef |
  | v6 blue_box | identity | 51,53,57,61 | **4/4** |
  | v6 orange_box | identity | 51,53,57,61 | 0/4 |
  | v7 | hardened | 51,53,...,65 | 8/8 |
  | v7 env +12 mm y | aim envelope | 51,55,59,63 | 4/4 |
  | v7 env -12 mm y | aim envelope | 51,55,59,63 | 4/4 |
  | **v7 formal** | **selection** | **51-65** | **15/15** |
- PROVENANCE: present in program.py, 11 constants, every one sourced to a
  debug-seed measurement or generic controller/camera mechanics.  No pack was
  issued for this cell and none was read.
