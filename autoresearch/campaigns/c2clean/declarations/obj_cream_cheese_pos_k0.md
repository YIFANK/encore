# c2clean / obj_cream_cheese_pos_k0

Intent: "pick up the cream cheese and place it in the basket". No demo pack (k0).
Debug seeds 51-65 only. Every constant below is measured from my own cam_high
captures and my own arm probes on those seeds.

## v1 / v1b -- can I see the scene at all?
Hypothesis: `api.log` can carry a zlib+base64 RGB-D dump so perception can be
done offline.
Evidence: yes, but `api.log` TRUNCATES a message at ~1980 chars -- 3000-char
chunks came back clipped and base64 failed to decode. 1400-char chunks decode
cleanly.
Scene (seed 51, cam_high): six props on a plain table plus a wicker basket.
Two cylindrical cans (left), an orange-juice carton, a milk carton, a small
DARK BLUE flat brick with a pale-blue oval logo, and a small flat brick with a
sky/red/orange label. Camera extrinsic maps image u -> base +y, image v ->
base +x; table plane at z = 0.002.
Verdict: perception pipeline works; two flat bricks are the only candidates for
a "cream cheese" package.

## v2 -- footprint clustering
Hypothesis: a 1 cm XY-footprint clustering of the above-table point cloud
segments the props.
Evidence: 6 clusters, but CL0 (h=0.400, u197-317, v46-259) swallowed BOTH the
orange-juice carton and the blue brick -- the parked arm hovers over them
(eef = [-0.1485, 0.0, 0.2613]) and footprint-only clustering fuses a floating
structure with whatever is underneath it.
Verdict: need a height band, not a full column.

## v3 -- band-masked clustering
Hypothesis: clustering ONLY z in (zt+0.012, zt+0.055) excludes the arm.
Evidence: 7 clean clusters on every one of seeds 51-65, one per prop. But the
per-cluster height, re-measured over the full column, still read 0.471/0.481 for
the two props under the arm.
Verdict: segmentation fixed, height measurement still contaminated.

## v4 -- height by vertical continuity + fingertip calibration
Hypothesis A: walking up the sorted z values from the table and stopping at the
first gap > 15 mm gives the prop's own top, because the arm floats far above it.
Evidence: heights became 0.017-0.018 (the two bricks), 0.079 (both cans),
0.140 (both cartons), 0.141 (basket) -- stable across seeds 51,53,55,57.
Hypothesis B: pressing the OPEN gripper straight down onto empty table stalls
when the fingertips touch, so eef_z_at_stall - table_z is the fingertip offset.
Evidence: commanded z = -0.058 at a free cell, stalled at eef z = 0.0095 on both
repeats and on all four seeds; table z = 0.0020.
  => TIP_OFF = 0.0075 m (fingertips sit 7.5 mm below the eef reference).
Verdict: both confirmed.

Geometry established (seeds 51-65, cam_high):
  blue brick  : 0.079 (x) x 0.042 (y) x 0.018 (z), mean rgb (69,75,92), blue=+20
  other brick : 0.073 (x) x 0.038 (y) x 0.017 (z), mean rgb (105,64,44), blue=-40
  cans        : h 0.079;  cartons h 0.140;  basket ext 0.156, rim h 0.141
Both bricks are long in x, short in y, on every debug seed -- and the gripper
closes along world y with rotation=None, so the short axis is already the
closing axis.

## v5 -- first full pick-and-place (blue brick = cream cheese)
Hypothesis: the blue brick is the cream cheese; grasp it top-down at mid height
(fingertips at ztop - 0.008), carry at zt+0.30, release at zt+0.21 over the
basket bbox centre. The debug-seed `benchmark_success` bit is the test of the
identification, since a clean grasp+place of the wrong brick must score 0.
Evidence: pending.
Verdict (v5): FAILED 0/8 on seeds 51,53,55,57,59,61,63,65 -- but the failure was
mechanical, not identification. The 2 s descent to a computed mid-height target
never reached it (eef 0.0241 vs target 0.0196, residual 0.0104: OSC steady-state
droop), so the fingertips sat at 0.0166 with the brick top at 0.020, and the
close ended at width 0.0015 = empty jaws. The post-episode re-perception showed
the brick had not moved at all (cen (-0.150,0.060) vs (-0.150,0.059)).
Two candidate causes: (a) descent too shallow, (b) wrong closing axis.

## v6 / v7 -- A/B on the closing axis, with a press-to-table descent
Hypothesis: a flat brick only needs the OPEN fingers driven PAST it to the
table (command z = zt - 0.030 and let the arm stall) -- the tips then straddle
its full 18 mm height instead of grazing its lid. And the closing axis is
settled by running the same program at wrist yaw 0 (v6) and yaw 90 (v7).
Evidence, seeds 51,53,55,57:
  v6 (yaw 0):  4/4 benchmark_success. Press stalled at eef z 0.0095, i.e.
               tips exactly on the table at 0.0020 -- the same stall value the
               v4 calibration measured. Close -> width 0.0422, effort 3.0, held
               all the way to release.
  v7 (yaw 90): 0/4. Press stalled 12 mm high (eef 0.0213, tips 0.0138) because
               the fingers now came down ON the brick's 79 mm long axis, which
               equals the gripper's full 79 mm opening. The close read width
               0.0411 / effort 3.0 for one instant, then the lift emptied the
               jaws (width 0.0010, effort 0.05) -- a lid pinch, not a grasp.
Verdict: rotation=None / yaw 0 closes along world +-y, the brick's SHORT axis;
the closing axis is confirmed, the press-to-table descent is confirmed, and the
BLUE brick is confirmed to be the cream cheese (a clean, verified grasp-and-
place of it scores the benchmark bit; the identification could not have been
wrong and still scored 4/4).

Note on the effort flag: v7 shows effort 3.0 is NOT proof of a grasp -- it read
3.0 on a pinch that the very next lift dropped. The receipt that survives is
the closed width matching the measured object dimension AND still reading 3.0
after the lift.

## Selection
v6 on all 15 debug seeds: 15/15.
Per-seed receipt: every episode selected cluster c=1 (blue = +20.4..+21.0
against -40 for the other brick, the only other prop under h=0.07), pressed to
eef z 0.0095-0.0096, closed to width 0.04221-0.04222 (= the brick's measured
0.042 m y extent) and still read effort 3.0 after the lift to zt+0.30.

## DECLARATION
- Frozen version: **v6**.
  `packs/c2clean_obj_cream_cheese_pos_k0/program.py`
  md5 `290a9198f00351bb04442c35ff14fc84` == `program_v6.py` (verified on the
  cluster with md5sum).
- Full-15-seed selection receipt: **15/15**
  `results/sel_c2clean_obj_cream_cheese_pos_k0_v6final` (frozen program.py,
  episodes 51-65), reproducing `results/sel_c2clean_obj_cream_cheese_pos_k0_v6`
  (15/15, byte-identical program).
- Per-version receipt chain:
  v1/v1b perception dump, no episode outcome claimed (0/2, 0/4 -- no motion)
      `results/fs_..._v1`, `results/fs_..._v1b`
  v2  footprint clustering, no motion, 0/8   `results/fs_..._v2`
  v3  band-masked clustering, no motion, 0/15 `results/fs_..._v3`
  v4  calibration + wrist look, no grasp, 0/4 `results/fs_..._v4`
  v5  first pick-and-place, 0/8              `results/fs_..._v5`
  v6  press-to-table, yaw 0, **4/4**         `results/fs_..._v6`
  v7  press-to-table, yaw 90, 0/4            `results/fs_..._v7`
  v6  full 15                 **15/15**      `results/sel_..._v6`, `results/sel_..._v6final`
- PROVENANCE: present as a top-level literal dict in program.py, covering
  WS_X, WS_Y, CELL, BAND_LO, BAND_HI, TIP_OFF, SMALL_H, SMALL_EXT, YAW_DEG,
  GRASP_BITE, CARRY_Z, REL_Z. Every one is sourced to a debug-seed (51-65)
  measurement of my own or to generic controller/camera mechanics. This cell
  had no demonstration pack and no note file; no LIBERO-specific prior was used.
