# c2clean / spa_bowl_top_drawer_cabinet_task_k3

Intent: "Pick the akita black bowl on the top of the wooden cabinet and place
it on the plate."  Runner: tools/fair_run.py only.  Debug seeds 51-65.

## Evidence read from the two packs

- k3 pack language: "pick up the black bowl **in the top drawer** of the wooden
  cabinet and place it on the plate" (my target, a different object).
- mate pack language: "pick up the black bowl **on the wooden cabinet** and
  place it on the plate" (my object).
- Keyframes sit on gripper sign flips.  Close poses:
  mate (-0.010,-0.257,1.159) / (-0.015,-0.218,1.147) / (-0.017,-0.215,1.152);
  k3 (0.037,-0.119,1.092) / (0.035,-0.106,1.097) / (0.077,-0.118,1.098).
  Release poses (both packs) cluster at x~0.07, y 0.214..0.272, z 0.945..0.973.
- Closed gripper_state at the holding keyframes: 0.0022..0.0057 per finger
  (~5 mm gap) => the demos PINCH THE RIM WALL, they do not straddle the bowl.
- ee_path6 rpy at the close pose: roll ~pi, yaw ~-0.12 rad => the wrist is in
  its reset (straight-down) attitude, so the jaws close along base y.

## Scene, measured on debug seeds 51/53 (v0 = RGB-D dumped through api.log)

- table top z = 0.900; cabinet slab top z = 1.127 over x[-0.09,0.21],
  y[-0.345,-0.015]; the open top drawer and the bowl inside it sit at
  1.06..1.10; the plate is a disc of diameter 0.135 at (0.067,0.205),
  top 0.920; a third bowl sits on the table at (-0.20,0.21).
- The target bowl is the only object standing on the slab: a rim ring at
  z = 1.180 (= slab + 0.053), Kasa fit r = 0.054, fit residual 0.002 m.  Its
  centre jitters per seed (51: (0.020,-0.267); 53: (-0.002,-0.287);
  63: (0.043,-0.298)).
- Derived: grasp eef z = rim_top - 0.028 (mate close-pose mean 1.153 vs rim
  1.180); the demos' release y = plate_y + r, i.e. a rim-pinched bowl's centre
  trails the eef by r; release eef z = plate_top + 0.026.
- The predicate grades the bowl the INSTRUCTION names: v1 never touched the
  drawer bowl and still scored 6/8, so the re-authored cell is graded on the
  cabinet-top bowl.

## Controller mechanics measured on debug seeds (this is what the versions turned on)

- `move` runs until the eef is within the controller's own 0.012 m tolerance
  or for 60*seconds sim steps, whichever comes first; the episode horizon is
  1000 steps.  A converged move costs ~10 steps, a starving one the full cap
  (180 at seconds=3.0).  Past the horizon the eef freezes silently and even
  the gripper stops responding -- v2/v3/v5/v6 losses are all this.
- `effort` is only a gap flag (3.0 iff the finger gap exceeds 5 mm); the hold
  receipt is the WIDTH: an empty close reads 0.0010, a real rim pinch
  0.0045..0.0107.
- The descent onto the cabinet-top bowl is the one badly-tracked hop: straight
  from a 1.28 hover it swings up to 3 cm in +x.  Staging it (hover -> a hover
  0.05 above the grasp, converged to <=0.006 -> the last 0.05) lands it within
  0.0007..0.0013.  On some seeds (63) the descent is additionally contact- or
  reach-limited and stalls ~0.01 m high; closing there still catches the wall.

## Version chain (probe = seeds 51,53,55,57,59,61,63,65)

| ver | change | probe |
|-----|--------|-------|
| v0 | RGB-D dump, no motion (seeds 51/53) | - |
| v1 | perceive -> rim pinch at (cx, cy+r, rim-0.028) -> release at (plate_x, plate_y+r, plate_top+0.026) | 6/8 (fs_..._v1) |
| v2 | converge every waypoint, retry the grasp on an empty close | 5/8 (fs_..._v2) -- three episodes hit the 1000-step horizon |
| v3 | move budget, hold judged by width alone | 6/8 (fs_..._v3) |
| v4 | staged descent (hover -> approach -> 0.05 descent) + lateral correction | 7/8 (fs_..._v4), 224 steps on the wins |
| v5 | lateral correction replaced by lift-and-redescend + jam detection | 7/8 (fs_..._v5) |
| v6 | no correction at all: close where the descent stalls | 7/8 (fs_..._v6) -- ep63's pinch caught (width 0.0054) but the horizon was already spent |
| v7 | seconds sized per hop (1.0 near the bowl, 2.0 transit, 1.5 place descent) | **8/8** (fs_..._v7) |

## DECLARATION

- **Frozen version: v7.**  `packs/c2clean_spa_bowl_top_drawer_cabinet_task_k3/program.py`
  md5 `f6cc609eea0aa37bdde96a256dc5791a` == `program_v7.py` (same md5).
  Every formally probed version is archived as `program_vN.py` in the pack dir.
- **Selection receipt: 15/15** on the full debug split 51-65,
  `results/sel_c2clean_spa_bowl_top_drawer_cabinet_task_k3_v7`
  (every episode `benchmark_success: true`, held=True, 220-425 sim steps of
  the 1000 available).
- Per-version receipt chain: the table above; result dirs
  `results/fs_c2clean_spa_bowl_top_drawer_cabinet_task_k3_v{0..7}`.
- PROVENANCE: present as a top-level literal dict in program.py, 15 entries,
  every one sourced to a named pack field or to a debug-seed measurement /
  generic controller mechanics.
- No shared note file was read or written; no forbidden asset was opened; the
  bddl path was only ever passed to --bddl.
