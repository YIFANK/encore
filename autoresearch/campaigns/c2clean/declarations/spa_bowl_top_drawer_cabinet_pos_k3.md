# c2clean / spa_bowl_top_drawer_cabinet_pos_k3

Intent: "pick up the black bowl in the top drawer of the wooden cabinet and
place it on the plate". Runner: tools/fair_run.py only. Debug seeds 51-65.

## Pack read (K=3)

All three demos are the same shape: home (-0.20,0.00,1.17) -> descend to
(0.035..0.077, -0.106..-0.119, 1.092..1.098) with the wrist tilted (rpy pitch
0.58/0.73/1.09, i.e. the tool approach axis leaning ~33-62 deg toward -x)
-> close (gripper state goes 0.072 open -> ~0.010 gap) -> lift to z~1.21-1.25
-> traverse to y~+0.25 -> descend to z 0.94-0.96 with the wrist back near
straight down -> open. So: rim-style pinch at ~0.02 below the bowl rim, carry
high, release ~0.04 above the plate.

Demo keyframe images show the plate at LOWER-right and a third bowl at
mid-right; on every debug seed those two are swapped. Position of the goal
must be perceived, never replayed.

## v0 -- perception probe (15/15 debug seeds, no motion)

RGB+depth dumped through api.log (zlib+base64, 1800-char chunks).
Camera cam_high: intrinsics f=618.04, c=(256,256); t_base_cam puts the camera
at (0.659, 0.000, 1.610) with image +u -> base +y.

Scene model (identical structure on all 15 seeds):
- table plane z = 0.901/0.902
- wooden cabinet: top slab plateau z = 1.127; open TOP drawer, floor z = 1.064,
  side rails / handle z = 1.098..1.124
- TARGET bowl: sits in the drawer. Rim top z = 1.116 on every seed, outer
  diameter 0.110-0.115, interior floor 1.071
- a second, identical bowl stands ON the cabinet top slab, rim z = 1.176-1.180
- on the table: the plate (flat disc, bbox 0.125-0.140, ztop 0.920, inner
  surface 0.908), a cookie box (0.080 x 0.060, ztop 0.920) and a third bowl
  (0.085-0.095 disc, ztop 0.944)

Detector that works on all 15 seeds:
- slab = modal height above table+0.15
- band = slab-0.040 < H < slab-0.006; largest near-square component is the
  drawer bowl (198-225 cells, bbox 0.110x0.110 every seed, ar 1.00).
  The only other component in that band is the drawer handle (86-92 cells,
  bbox 0.085x0.060, ar 1.4) -> aspect-ratio filter separates them.
  This rule encodes the instruction: the target bowl is the one whose rim is
  BELOW the cabinet top, i.e. recessed in the drawer; the twin on the slab is
  above it and the table bowl is 0.18 below it.
- plate = largest near-square component of table+0.006 < H < table+0.08 with
  x > -0.25 (552-626 cells vs <=235 for the box and the table bowl).

The cabinet, the drawer and the bowl inside it translate as one rigid body
across seeds (handle centre is always bowl centre + (0.054, 0.099)); only
their common offset changes (bowl centre x 0.072..0.097, y -0.158..-0.129).

## v1 -- straight-down rim pinch, three offsets (seeds 51,55,59,63)

0/4, and all three attempts closed on air (gap 0.0014 everywhere). The
descents to rim-0.020 stalled at z 1.1195 / 1.1308 / 1.1368 with residuals
0.026 / 0.036 / 0.042. sim_steps 636-642.

## v2 -- calibration probe (seeds 51,55)

- **Episode horizon is 1000 sim steps.** The run froze from MV13 onwards with
  a constant eef and gap; results.jsonl reported sim_steps 1000 in both.
- move() with rotation= goes through move_pose, whose step cap is
  max(60, 60*seconds); with seconds=1.0-1.5 a 0.13 m descent does NOT finish.
  Measured 1.0 mm/step on a saturated descent near the drawer vs 6.9 mm/step
  on the first long free-space move, so the "stalls" in v1/v2 are a mixture of
  starved moves and real contact and cannot be told apart from one call.
- The bare-table calibration at (0.20, 0.10) was itself starved/limited (the
  arm only reached x=0.144), so the 0.0069 "tip offset" it produced is not
  trustworthy.

Open questions for v3: the true fingertip-to-EEF offset (read optically off
the depth map with the gripper parked over bare table), and whether the
descent into the drawer is blocked or merely starved (repeated bounded move()
calls to the same target, logging EEF after each).

## v3 -- optical calibration (seeds 51,55)

Repeated bounded move() calls to the same target plateau after the first, so
the drawer descents in v1/v2 were genuinely BLOCKED, not starved:
centre pinch stops at z 1.1174 (= the rim), the +y rim pinch at 1.1059.
Parking the gripper over bare table and reading the depth cloud in the column
around the EEF gives the real geometry: **the fingertips sit 0.008 m below the
reported EEF z**, and the open fingers occupy |dy| ~ 0.039..0.055 (gap 0.0778,
so 0.039 per side plus finger thickness).

That explains v1. With the jaws open the hand is 78 mm across, and the drawer
is barely wider than the bowl, so on nearly every azimuth the OUTER finger
lands on a drawer rail (1.122-1.124) or the cabinet slab (1.127) and the hand
grounds out above the rim. The v3 rim pinch at +y did close on the wall
(gap 0.0276, effort 3.0) but lost it on the lift.

## v4 -- runtime clearance scan + wrist yaw (seeds 51,53,...,65) -> 7/8

Offline over all 15 debug clouds, the outer-finger ring (radius rp+0.043) is
free on exactly two arcs, present on every seed: ~5-10 deg (hanging off the
drawer's open front, over bare table) and **108-160 deg, 32-48 deg wide**
(the drawer's open corner). v4 finds those arcs at run time, takes the one
centred in 90..180 deg, yaws the wrist about base z so the jaws close along
that azimuth, and pinches the rim wall at radius rout-0.006.

Derived plan, identical on all 15 seeds offline: th 125-136 deg, rp 0.050-0.053,
grasp EEF z 1.094, support (drawer floor) 1.064, place z 0.934 -- and the
resulting grasp points (0.037..0.068, -0.121..-0.088) land inside the range
the three pack demos grasped at (0.035..0.077, -0.119..-0.106). Independent
confirmation that the demos used this same slot.

Result 7/8. Seed 57 closed on the wall (gap 0.0080, effort 3.0) and lost it
during the lift (gap 0.0019).

## v5 -- deeper bite + verified grasp with retries (seeds 51,53,...,65) -> 8/8

Two changes: (a) descend to the deepest z the bowl interior allows,
ceil+0.008+0.006 (~1.088) instead of rim-0.022 (1.094), so more wall sits
between the pads; (b) after closing, settle and lift to rim+0.040, then read
the gripper -- if effort has dropped the bowl is gone, so reopen and retry
with a 6 mm shallower bite, then +-10 deg of azimuth. 8/8, all first-try.

## v6 -- v5 with the depth dumps switched off (selection run, all 15) -> 15/15

sim_steps 165-175 on thirteen seeds; 54 and 60 took 237/240, i.e. the
verify-and-retry loop fired once and recovered. Budget is 1000 steps/episode.

## v7bias -- aim-envelope probe (NOT a candidate)

v6 with a deliberate +0.008,+0.008 (11.3 mm) error injected into the perceived
bowl centre, seeds 51,53,...,65: **8/8**. The run-time clearance scan absorbed
it by re-centring the approach azimuth on the free slot (th moved 125-136 ->
139-143). The margin does not come from luck in the aim; it comes from
choosing the azimuth from the measured free arc rather than from a constant.

## DECLARATION

- Frozen version: **program_v6.py**, md5 `e14c35159a6a079a55e1372f7946fbd0`;
  `packs/c2clean_spa_bowl_top_drawer_cabinet_pos_k3/program.py` has the same md5.
- Selection receipt (full 15 debug seeds, one formal run):
  **15/15**, `results/sel_c2clean_spa_bowl_top_drawer_cabinet_pos_k3_v6`
  (seeds 51-65 all benchmark_success=true).
- Receipt chain:
  - v0 perception probe, 15 seeds, no motion -- scene model
  - v1 straight-down rim pinch, `fs_..._v1`, 0/4
  - v2 calibration, `fs_..._v2`, horizon = 1000 steps
  - v3 optical calibration, `fs_..._v3`, tip offset 0.008
  - v4 clearance scan + wrist yaw, `fs_..._v4`, **7/8**
  - v5 deeper bite + verified retry, `fs_..._v5`, **8/8**
  - v6 = v5 without depth dumps, `sel_..._v6`, **15/15**  <- FROZEN
  - v7bias aim-envelope probe (11.3 mm injected error), `fs_..._v7bias`, 8/8
- PROVENANCE: present in program.py as a top-level literal dict covering
  GRID_CELL, RIM_BAND, BAND_AR_MAX, PLATE_MIN_CELLS, TIP_OFFSET, JAW_HALF_OPEN,
  FINGER_PATCH, GRASP_DEPTH, ARC_WINDOW, PROBE_RADIUS. Every constant is
  sourced to this pack's own fields or to a debug-seed (51-65) measurement.
- api.done is never read.
