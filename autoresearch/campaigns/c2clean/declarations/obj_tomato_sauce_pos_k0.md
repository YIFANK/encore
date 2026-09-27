# c2clean / obj_tomato_sauce_pos_k0

Intent: "pick up the tomato sauce and place it in the basket".
No demonstration pack (k0). All constants re-derived from debug seeds 51-65.

Runner: `tools/fair_run.py` only. Probe subset 51,53,55,57,59,61,63,65.

## v1 -- log-as-datapipe, attempt 1
Hypothesis: RGB-D can be exported through `api.log` for offline perception at
zero sim cost.
Evidence: `results/fs_..._v1` (seeds 51,53,55). 72 sim steps, no motion. The
dump was truncated: `fair_client.log` clips each message at 2000 chars, so the
60000-char chunks arrived cut. Metadata lines survived.
Verdict: mechanism works, chunk size wrong. Measured anyway from the run:
start eef = (-0.1485, 0.0, 0.2613); start finger gap 0.0778 m (= full open);
cam_high K f=618.04, c=(256,256), t_base_cam origin (0.8966, 0.0, 0.65).

## v2 -- datapipe at 1900-char chunks
Evidence: `results/fs_..._v2`, all 8 probe seeds, 72 sim steps each, full
512x512 RGB + uint16(0.1 mm) depth recovered and decoded offline.
Verdict: the scene is now fully measurable. Findings:

* table plane z = 0.0015 m in the base frame (histogram mode of the cropped
  point cloud) -- identical on all 8 seeds.
* the back wall deprojects to x = -1.99, so the workspace crop
  x[-0.45,0.45] y[-0.60,0.60] is needed before fitting the table.
* the parked arm is a single cluster topping at 0.484 m; excluding z > 0.20
  above the table removes it. Without that exclusion it fuses with the bbq
  bottle on some seeds (ep51) and not others (ep53).
* seven props, verified by reprojecting each cluster's pixel bbox onto the RGB:

  | prop            | n     | mid (x,y)        | footprint     | top   |
  |-----------------|-------|------------------|---------------|-------|
  | basket          | 11500 | (~0.00, +0.26)   | 0.159 x 0.170 | 0.142 |
  | orange juice    |  3021 | (+0.100, -0.201) | 0.053 x 0.053 | 0.141 |
  | milk            |  2550 | (+0.051, -0.100) | 0.053 x 0.054 | 0.141 |
  | **tomato sauce**|  1443 | (-0.119, -0.241) | 0.064 x 0.069 | 0.080 |
  | cookie box      |  1326 | (+0.150, +0.029) | 0.082 x 0.049 | 0.028 |
  | bbq bottle      |   980 | (-0.198, -0.083) | 0.026 x 0.048 | 0.112 |
  | butter box      |   445 | (-0.151, +0.059) | 0.074 x 0.038 | 0.018 |

* **identity rule**: the can is the only prop whose top lies in 0.055-0.100 m.
  The nearest neighbours are the bbq bottle at 0.112 and the cookie box at
  0.028 -- a gap of 32 mm on both sides. Height is invariant to a planar
  `_pos` perturbation, so this rule does not depend on any xy coordinate.
* only the basket, the bbq bottle and the milk carton move across the probe
  seeds; the can sat at the identical xy on all 8. The rule is used anyway
  because the eval seeds (1-50) are unseen.
* **the can is not a plain cylinder**: height slices give a body width of
  0.0620-0.0632 m over z=0.010-0.058 but 0.0689 m at z>=0.074. The top rim
  flares ~3 mm per side over a 0.0625 m body.

## v3 -- descent-block calibration
Hypothesis: driving the open gripper down into the bare table reveals the
fingertip-to-eef offset; repeating it at the can centre shows whether the can
obstructs a top-down descent.
Evidence: `results/fs_..._v3` (seeds 51,53,55), 458 sim steps each, identical
on all three.
* bare table: commanded z = tz-0.08, stalled at eef z = **0.00915** with the
  table at 0.0015 -> **fingertips sit 0.0077 m below the eef**.
* can centre: stalled at eef z = 0.0679, i.e. fingertips 0.0602, which is
  21 mm below the can top -- the jaws entered, then jammed.
* both probes drifted badly sideways while pressing (table probe overshot
  50 mm in y; can probe slid 38 mm in +x). The gif shows the can toppled and
  lying beside the orange juice carton.
Verdict: fingertip offset measured and trusted. The failure mode is the
*saturated* descent into an unreachable z target, which wanders laterally and
knocks the can over. A grasp must (a) stop at a legitimate z, and (b) get its
xy right *above* the rim, because the open jaws (0.0778 m) clear the 0.0689 m
rim by only 4.4 mm per side.
Also measured: LIBERO's default horizon here is 1000 sim steps, so a ~10-move
plan fits.

## v4 -- perceive, body-grasp, drop in the basket
Hypothesis: with the centre taken from the parallel-sided body band and the xy
bias cancelled above the rim, a pure-z descent onto the body grasps the can.
Centre estimator: cam_high looks down -x, so y extent is the true diameter and
x is the near tangent pulled back by one radius.
Evidence: `results/fs_..._v4` 8/8 on the probe subset in ~202 sim steps (of a
1000-step horizon); `results/sel_..._v4` **15/15** on all 15 debug seeds.
Closed finger gap 0.0622 m against a body measured at 0.0625 m, effort 3.0 held
through the lift and the carry on every seed.
Verdict: works, but the descent lands 5.8 mm short in -y. The bias-cancel loop
converges at the *hover* height and the standing error then changes over the
0.166 m descent, so the correction is spent by the time the jaws reach the can.
5.8 mm of error against 7.6 mm of body clearance is not a margin worth shipping
when the eval seeds put the can somewhere else, with a different standing bias.

## v5 -- re-centre above the rim, and verify the grasp
Hypothesis: cancelling the bias a second time at eef z = 0.11 (fingertips
0.102, clear of the 0.080 rim) removes the residual, and checking the finger
gap after the lift lets a failed close be retried instead of carried.
Evidence: `results/fs_..._v5` 8/8 probe; `results/sel_..._v5` **15/15** on all
15 debug seeds. Grasp aim error fell from (0.0014, -0.0058) to
(0.0024, -0.0016) -- 3.6x better in y.
Verdict: adopted.

## Aim-envelope sweep (why v5 and not v4)
Both versions score 15/15, so the debug split does not separate them. On every
debug seed the can sat at the *identical* xy (-0.119, -0.242), so that score
says nothing about a `_pos` perturbation moving it. The versions were separated
by injecting a deliberate aim offset into the perceived can centre and
re-running 4 seeds (51,55,59,63):

| injected y offset | -26 mm | -20 mm | -10 mm | +10 mm | +16 mm | +24 mm |
|-------------------|--------|--------|--------|--------|--------|--------|
| v4                |   --   |   --   | **0/4**|  4/4   |  4/4   |   --   |
| v5                |  4/4   |  4/4   |  4/4   |  4/4   |  4/4   |  4/4   |

v4's collapse is one-sided and that is the point: its own uncorrected -5.8 mm
bias adds to a -10 mm injection to make ~16 mm, past the body clearance, while
a +10 mm injection partly cancels it. v4 is passing on debug by a bias that
happens to point the right way at this one can position.

Honest reading of v5's wide envelope: the logs show that past about +/-10 mm it
is not the first grasp that succeeds. At -26 mm attempt 0 closes on air
(gap 0.0010, effort 0.05); the gap check fires, the scene is re-perceived, and
attempt 1 lands 4.6 mm off and closes cleanly (gap 0.0621, effort 3.0). The
envelope is bought by the *verification and retry*, not by the aim.

## DECLARATION
* Frozen version: **v5**.
  `packs/c2clean_obj_tomato_sauce_pos_k0/program.py`
  md5 `4bfeff5d697d04209d87656479efd70a` == `program_v5.py` (same md5).
* Selection receipt: **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_obj_tomato_sauce_pos_k0_v5`.
* Receipt chain:
  | version | run dir | seeds | result |
  |---------|---------|-------|--------|
  | v1 | `fs_..._v1` | 51,53,55 | datapipe truncated at 2000 chars; metadata only |
  | v2 | `fs_..._v2` | 8 probe | full RGB-D recovered; scene + identity rule |
  | v3 | `fs_..._v3` | 51,53,55 | fingertip offset 0.0077 m; saturated descent topples the can |
  | v4 | `fs_..._v4` / `sel_..._v4` | 8 probe / 15 | 8/8 / **15/15** |
  | v4e | `fs_..._v4e_{m10,p10,p16}` | 51,55,59,63 | 0/4, 4/4, 4/4 |
  | **v5** | `fs_..._v5` / `sel_..._v5` | 8 probe / 15 | 8/8 / **15/15** |
  | v5e | `fs_..._v5e_{m26,m20,m10,p10,p16,p24}` | 51,55,59,63 | 4/4 on all six |
* PROVENANCE: present in program.py as a top-level literal dict, covering
  XLO/XHI/YLO/YHI, CELL, ZARM, CAN_BAND, BODY_LO/BODY_HI, FINGER_OFF,
  GRASP_TIP_Z, RIM_CLEAR_Z, SAFE_Z, HELD_GAP_MIN, DROP_CLEAR. Every constant
  is sourced to a debug-seed measurement (v2/v3/v4 logs) or to generic
  controller/camera mechanics. No pack was supplied (k0) and none was read.
* No mechanism gap: the cell is solved, not blocked.
