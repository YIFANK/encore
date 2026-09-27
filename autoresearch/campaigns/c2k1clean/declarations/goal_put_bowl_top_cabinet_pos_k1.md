# c2k1clean / goal_put_bowl_top_cabinet_pos_k1

Intent: "Put the bowl on top of the cabinet."  Runner: `tools/fair_run.py` only.
Pack: `packs/c2k1clean_goal_put_bowl_top_cabinet_pos_k1/` (K=1, 4 keyframes).
Debug seeds 51-65 only; eval seeds 1-50 never touched.

## What the pack gave (and what it did NOT)

`pack.json` demo0, length 91, keyframes at t=0/31/81/90 with `gripper_cmd`
sign flips:

| t | ee xyz | cmd | gripper_state |
|---|--------|-----|---------------|
| 0 | (-0.217, 0.027, 1.167) | -1 (open) | 0.036 |
| 31 | (-0.099, 0.050, 0.915) | **+1 (close)** | 0.039 |
| 81 | (0.006, -0.177, 1.149) | **-1 (open)** | 0.004 (holding) |
| 90 | (0.023, -0.165, 1.193) | -1 | 0.036 (released) |

So the mechanism is: descend beside the bowl, close **deep** (ee z 0.915,
i.e. ~13 mm above the table), lift, carry, release at ee z ~1.15, retreat.
`ee_path6` rotations are all near (pi, 0, 0) -> straight-down wrist
throughout.

The demo **xy are decoys**.  Keyframe images show the cabinet on the -y side
of the demo scene; on every debug seed the cabinet is a grey box at
**+y** (centre ~(-0.40, +0.22)) and the demo release point (0.006, -0.177)
lands on the wooden rack instead.  Every xy in the program is perceived.

## Perception (probe1/probe2, no robot action)

Shipped cam_high RGB + depth (uint16 mm) out through `api.log`
(zlib+base64, 1900-char chunks -- `api.log` truncates at 2000) and measured
offline.  `probe1` was wasted: 60000-char chunks were silently truncated.

Measured, all 15 debug seeds, from a 5 mm top-down height map:

- table plane **z = 0.902** (modal deprojected z).
- exactly one large flat plateau above it: **z = 1.128**, x[-0.53,-0.27],
  y[0.10,0.33], ~1400-2000 cells, z spread < 2 mm -> the **cabinet top**.
- in the band table+22..110 mm exactly two blobs:
  * **bowl** -- n~210 cells, footprint 0.110 x 0.110-0.116 m (round),
    rim top z = 0.953, colour (115,115,101);
  * black pot on the stove -- n~92, footprint 0.090 x **0.028** m, z 0.961.
  A "round in both axes" window (0.08-0.15 m each, min/max > 0.7) separates
  them on every seed.
- bowl outer-wall radius profile (seed 51): 0.044 @ z0.920, 0.049 @ 0.930,
  0.052 @ 0.940, 0.056 @ 0.950; interior floor a flat disc r~0.026 at
  z~0.905.  Rim radius is uniform to ~1.7 mm around the ring, so the ring
  bbox midpoint is a good centre.
- layout jitter across seeds 51-65 is small: bowl centre x in [0.033,0.062],
  y in [-0.036,-0.005]; cabinet centre moves ~1 cm.
- the wine bottle (top z 1.04 at (-0.195,-0.045)) is the only thing between
  bowl and cabinet and sits far below the carry height.

## v1 (frozen)

Perceive table_z / cabinet plateau / bowl ring every episode, then:

1. open jaws, hover at (cx, cy + r_wall, rim_z + 0.09) with R_DOWN;
2. descend to z = rim_z - 0.026 (the pack's deep-close height, expressed
   relative to this bowl's rim);
3. close -- the jaws straddle the bowl **wall** (one finger inside, one
   outside); closed width ~0.0075 m, effort 3.0;
4. lift to z = 1.26, carry to the cabinet-top centre clipped 55 mm inside
   its measured bbox;
5. descend to cab_z + (z_close - table_z) + 0.006 = 1.1575 and open.

Step 5's height is the pack's release relation re-derived: the bowl bottom
rides (z_close - table_z) = 25 mm below the eef, so the eef must sit that
much above the cabinet top.  Lands at 1.167 (a consistent +10 mm tracking
overshoot), which is still inside the bowl's own clearance.

### Receipts

| version | seeds | result | dir |
|---|---|---|---|
| probe1 | 51,53,55,57 | perception only (chunks truncated, redone) | `results/fs_c2k1clean_goal_put_bowl_top_cabinet_pos_k1_probe1` |
| probe2 | 51-65 | perception only, all 15 scenes decoded | `results/fs_c2k1clean_goal_put_bowl_top_cabinet_pos_k1_probe2` |
| v1 probe | 51,53,55,57 | **4/4** | `results/fs_c2k1clean_goal_put_bowl_top_cabinet_pos_k1_v1` |
| **v1 formal** | **51-65 (all 15)** | **15/15** | `results/sel_c2k1clean_goal_put_bowl_top_cabinet_pos_k1_v1` |

### Aim envelope (margin, not just score)

The grasp offset along the closing axis was displaced from its perceived
value and re-run on seeds 51,53,55,57,59,61:

| displacement | result | dir |
|---|---|---|
| -20 mm | 6/6 | `..._aimm020` |
| -10 mm | 6/6 | `..._aimm010` |
| nominal | 6/6 (subset of the 15/15) | `..._v1` |
| +10 mm | 6/6 | `..._aimp010` |
| +20 mm | 6/6 | `..._aimp020` |

At +20 mm the jaws still closed to 0.0085 m on the wall: the straddle close
drags the bowl and self-centres, so the capture basin is at least +-20 mm,
against ~3 mm of perception noise.  No further version was warranted.

## DECLARATION

- **Frozen version: v1.**  `packs/c2k1clean_goal_put_bowl_top_cabinet_pos_k1/program.py`
  md5 `ba7ad0aacbd236779b417eb763f20c3d` == `program_v1.py` (same md5,
  verified on the cluster).
- **Selection receipt: 15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2k1clean_goal_put_bowl_top_cabinet_pos_k1_v1`,
  `[fair] ...: 15/15`.
- **PROVENANCE** present as a top-level literal dict in `program.py`,
  11 entries, every one sourced to this pack or to a debug-seed measurement
  recorded above.
- No shared note file used; no `fewshot_run.py`; no forbidden reads; eval
  seeds 1-50 never run.
