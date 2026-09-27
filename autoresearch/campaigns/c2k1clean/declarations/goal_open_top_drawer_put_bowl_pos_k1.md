# c2k1clean / goal_open_top_drawer_put_bowl_pos_k1

Intent: *"Open the top layer of the drawer and put the bowl inside"*.
Runner: `tools/fair_run.py` only. Pack: K=1 (`demo0`, 170 steps, 6 keyframes).

## What the pack says (read before any debug run)

`ee_path6` + `actions` decompose the demo into three mechanisms:

| steps | eef | gripper | what it is |
|---|---|---|---|
| 0-30 | (-0.185,-0.013,1.181) -> (0.021,-0.093,1.104) | open | descend in front of the drawer-handle bar; z **stalls at 1.104** while the z command stays negative -> contact |
| 35-60 | y -0.093 -> +0.065 at z 1.105 | open | `actions[1]` ~ +0.7 **and** `actions[2]` ~ -0.4 together: press *and* pull. 0.158 m of drag = the drawer coming out |
| 70-100 | -> (-0.107, 0.0715, 0.9386) | closes at t=100 | descend on the bowl and pinch |
| 100-163 | -> (0.018,-0.046,1.1354) | closed, gap 0.0047 | carry and release over the open drawer |

The closed gap of 4.7 mm says the grasp is a **rim pinch** on a thin wall,
not a body grasp. Keyframe `t0169` shows the bowl sitting upright in the
open drawer, so the release lands it inside.

## Scene, measured on debug seeds (v0, pure perception dump)

`cam_high` K = 618.04, t_base_cam puts the camera at (0.659, 0, 1.610);
image-right = base +y, image-down = base +x.

* table top **z = 0.9014**
* cabinet top plateau **z = 1.1276**, x[-0.098,0.158], face at y = -0.158
* top-drawer handle bar: **top z = 1.098**, ~0.028 m deep in y, protrudes
  ~0.030 m in +y from the face; centre (0.034, -0.140) +/- 0.008 across seeds
* bowl: footprint 0.108 x 0.112 (outer radius 0.055), **rim top 0.952**
  (= table + 0.0507), mean RGB ~ 120; centre wanders over
  x[-0.060,-0.044], y[0.121,0.141]
* nearest impostor is the stove slab (top 0.932, RGB 68) -> a top cut at
  table+0.040 plus a brightness rank separates them on every debug seed

## Version log

### v0 -- perception dump (seeds 51,53,55,57)
Hypothesis: everything needed is in one `cam_high` frame.
Evidence: zlib+base64 RGB-D through `api.log`; all the numbers above.
Verdict: kept; perception is deterministic and costs 0 sim steps.

### v1 -- full pipeline, demo-relative constants. 0/4
Hypothesis: press-drag at handle+0.047 in y, rim-pinch at (cx, cy-0.0525),
release at the demo's handle-relative point.
Evidence: the drawer opened on **4/4** (handle cy -0.140 -> +0.021, 0.159 m).
The pinch held on 3/4 (gap 0.0056-0.0061, effort 3.0). Every episode failed
at the **release**: the demo-relative drop y = -0.044 sits on the drawer's
front lip. Post-open profile: cavity floor 1.064, wall tops 1.123, cavity
y from -0.168 to -0.028 -> **centre y = -0.098**, 54 mm behind where v1 let go.
The GIF shows the bowl tipping off the lip onto the table.
Verdict: open = solved; release point wrong; bite too shallow (the descent
converged 4 mm below the rim because POS_TOL is 12 mm).

### v2 -- cavity-centred release + 30 mm commanded bite. 3/8
Hypothesis: measure the opened drawer and drop into its centroid, descending
until the hanging bowl contacts the floor.
Evidence: the cavity detector found floor 1.0642 and a 1200-cell plateau on
**8/8**. Every episode that actually held the bowl (53, 55, 57) **succeeded**.
The other 5 never got the bowl: the descent onto the bowl froze at
**z = 1.0902** (identical to 4 decimals) with the eef pushed ~+0.017 in x.
GIF: the gripper is sitting on the **opened drawer's front lip/handle**.
The -y pinch arc puts the near finger at cy-0.0525-0.039 ~ +0.03, which is
exactly where the handle bar ends up after a 0.16 m pull.
Verdict: the open drawer is an obstacle the closed-drawer demo never had.

### v3 -- pinch the +y arc instead
Hypothesis: mirroring the pinch to (cx, cy+0.0525) moves the whole hand
0.105 m away from the opened drawer while grasping the same rim.
Evidence: **8/8** on the probe subset (51,53,...,65); every episode grasped on
the first try (closed gap 0.0082-0.0092, effort 3.0), held through a 0.34 m
lift (0.0052-0.0081) and stalled on the drawer floor with the gap *widening*
to 0.013-0.020 as the bowl seated. Formal 15-seed run: **15/15**
(`results/sel_c2k1clean_goal_open_top_drawer_put_bowl_pos_k1_v3`).
Verdict: the drawer, not the demo, dictates the approach side.

### v4 -- v3 plus fallbacks and a failure-triggered re-place. 15/15
Hypothesis: the pack's own demo has the bowl at cx = -0.107, 47 mm outside
the x band the 15 debug seeds span, so the eval split probably perturbs
wider than anything I have measured. Harden without changing behaviour.
Changes, all inert on a nominal seed:
* `find_handle` / `find_bowl` returning nothing no longer raises -- they fall
  back to cabinet-relative and pack-relative geometry;
* pick-and-place factored into one function, so after the release a
  `find_stray` pass (bowl-sized bright blob still standing on the table,
  35-135 mm tall, off the cabinet footprint) can run it a second time;
* the whole verify/retry block is inside `try/except` -- a perception miss
  can never cost an episode that already worked.
Evidence: **15/15** on the full debug split
(`results/sel_c2k1clean_goal_open_top_drawer_put_bowl_pos_k1_v4`),
392-528 sim steps. `RETRY place`, `FALLBACK` and `VERIFY-SKIP` fired on
**zero** seeds -- the new code is dormant, as intended.
Verdict: **FROZEN**.

## What actually carried this cell

1. **The handle is pressed, not hooked.** The pack's open phase commands +y
   and -z *simultaneously* with the fingers wide; the descent stalls 7.8 mm
   above the bar top and the drag then takes the drawer out 0.16 m. 15/15
   on the very first version that tried it.
2. **The release point is a measurement, not a demo constant.** Transcribing
   the pack's release y put the bowl on the drawer's front lip (v1, 0/4).
   Deprojecting the opened drawer and dropping into the *cavity centroid*
   (floor 1.0642 on 8/8, a 1200-cell plateau) fixed it outright.
3. **Opening the drawer changes the reachable set.** The pack grasps the
   bowl's -y rim arc; after a 0.16 m pull the handle bar sits exactly where
   that hand has to be, and the descent froze at z = 1.0902 on 5/8 seeds.
   The same rim, pinched on the +y arc, is 0.105 m clear. A demo recorded
   with the drawer shut cannot tell you this.
4. **Seat, don't drop.** Lowering to `floor + 0.04` is contact-limited: the
   dangling bowl grounds at eef z 1.129-1.149 and the gap widens as it takes
   its own weight. Releasing from there beats releasing from a fixed height.

## DECLARATION

* **Frozen version: v4.** `program.py` md5 `93dff6a4fa177458bc049f7962aa8709`
  == `program_v4.py` (verified on the Mac and on AbakaAI).
* **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, directory
  `results/sel_c2k1clean_goal_open_top_drawer_put_bowl_pos_k1_v4`.
* **Receipt chain**
  | ver | seeds | score | dir |
  |---|---|---|---|
  | v0 | 51,53,55,57 | perception dump, 0 sim steps | `fs_..._v0` |
  | v1 | 51,53,55,57 | 0/4 | `fs_..._v1` |
  | v2 | 51..65 odd | 3/8 | `fs_..._v2` |
  | v3 | 51..65 odd | 8/8 | `fs_..._v3` |
  | v3 | 51-65 | **15/15** | `sel_..._v3` |
  | v4 | 51-65 | **15/15** | `sel_..._v4` |
* **PROVENANCE** present and accepted: `DRAG_DY, DROP_DY, DROP_DZ, EMPTY_W,
  GRASP_DZ, PRESS_DOWN, PRESS_DY, R_DOWN, R_PINCH`. Every one is sourced to
  a `pack.json` field or a debug-seed measurement; `fair_run.scan_program(...,
  'eval')` run against the frozen file returns clean.
* No `api.done` read anywhere in the program (AST-checked).
* Splits respected: seeds 51-65 only, always `--split debug`; seeds 1-50
  never touched.
