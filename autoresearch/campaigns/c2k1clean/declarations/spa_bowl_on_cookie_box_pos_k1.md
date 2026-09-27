# c2k1clean / spa_bowl_on_cookie_box_pos_k1

Intent: *pick up the black bowl on the cookie box and place it on the plate*.
Runner: `tools/fair_run.py` only. Splits sealed: debug = 51-65, eval = 1-50 (blind).

## What the pack says

`packs/c2k1clean_spa_bowl_on_cookie_box_pos_k1/pack.json`, K=1, 99 steps, 5 keyframes:

| t | ee xyz | ee rot (axis-angle) | grip cmd | finger gap |
|---|--------|---------------------|----------|-----------|
| 0 | (-0.1935, -0.0074, 1.1881) | (3.125, 0.030, -0.013) | open | 0.0724 |
| 46 | (0.0784, 0.0638, 0.9701) | (2.985, 0.092, 0.435) | close | 0.0788 |
| 55 | (0.0687, 0.0642, 0.9530) | (2.952, 0.167, 0.412) | close | 0.0085 |
| 86 | (0.0687, 0.2235, 0.9476) | (2.812, 0.197, 0.341) | open | 0.0054 |
| 98 | (0.0536, 0.2257, 0.9766) | (2.770, 0.108, 0.316) | open | 0.0769 |

Read off it: the demo **pinches**, it does not grab a body — 0.0085 m of gap is a
wall between the jaws. The tool y-axis at the grasp is (0.088, -0.982, 0.170),
i.e. essentially world -y: the jaws separate along y and the demo's yaw is the
same straight-down wrist that `api.move(rotation=None)` holds. The demo's own
scene is **not** the debug scene — its plate stands on the table and a bowl
stands on the cabinet; in seeds 51-65 those two are swapped. So the demo gives
mechanism (pinch, depth), never positions.

## What the debug seeds say (v1 recon, 15/15 seeds)

v1 = recon only: capture `cam_high` + `cam_arm_wrist`, dump rgb/depth/K/T/eef to
`results/recon_c2k1clean_spa_bowl_on_cookie_box_pos_k1/`; analysed off-cluster.
Deprojected cloud, height-continuity segmentation (0.008 m link) above the modal
surface height. Every seed decomposes identically:

| object | centre | z range | footprint | grey |
|--------|--------|---------|-----------|------|
| cabinet | (+0.07, -0.23) | 0.911..1.128 | 0.31 x 0.21 | 64 |
| arm (parked) | (-0.18, +0.00) | 1.165..1.320 | 0.07 x 0.20 | 38 |
| dark slab | (-0.26, -0.12) | 0.907..0.960 | 0.19-0.26 x 0.16 | 62 |
| **goal disc (plate)** | (+0.02, -0.28) | 1.135..1.148 | 0.12-0.14 | 131 |
| **target bowl** | (+0.06, +0.03) | **0.926**..0.971 | 0.110 x 0.111 | 117 |
| other bowl | (+0.05, +0.20) | 0.907..0.952 | 0.110 x 0.111 | 117 |
| ramekin | (-0.20, +0.21) | 0.907..0.944 | 0.087 | 121 |
| cookie box (front face) | (+0.11, +0.03) | 0.907..0.920 | 0.026 x 0.061 | 69 |

- surface z = 0.9025 in 15/15.
- The two bowls are geometrically identical; the **only** separator is the base
  height — the target's base is 0.0235 m up on the cookie box, the other's is on
  the table. Rule `argmax zmin over deep discs`: correct 15/15.
- Goal disc rule (shallow + bright + 0.095-0.175 m): correct 15/15 once the
  brightness term was added — without it the dark slab wins in ep51, where it
  happens to shrink to 0.187 x 0.161 m.
- Rim ring (top 6 mm, bbox midpoint) is the reliable centre: half-extent
  0.0551 +/- 0.0004 m, rim z = 0.9707 in 15/15. The *blob centroid* is biased
  ~0.014 m in x by the grazing view — do not use it.
- Bowl outer wall radius profile (ep51): 0.0202 @ 0.928, 0.0398 @ 0.940,
  0.0485 @ 0.952, 0.0534 @ 0.968, 0.0549 @ 0.971.
- Projecting ep51's rim circle into `keyframes/demo0_t0000.png` lands on the
  demo bowl's rim, so the demo's grasp sat ~0.039 m from its bowl's centre and
  0.018 m below its rim — one wall radius at that height, as expected.
- `robot0_eef_pos` is the grip site, not the flange: a demo pinch commanded to
  z = 0.953 on a bowl whose wall spans 0.926..0.971 only makes sense that way.
- Runner mechanics (`heron/robot/libero.py`): `POS_TOL = 0.012`, so every move
  may stop up to 12 mm short; `get_gripper` reports `effort 3.0` only while the
  jaws are commanded shut *and* the gap exceeds 0.005 m — that is the hold
  sensor this cell verifies with.

## Version chain

### v1 — recon (no motion)
Receipt: `results/fs_..._v1` (51,53,57,61) + `_v1b` (the other 11). 0/15 success
by construction. Produced the table above.

### v2 — perceive, rim-pinch at a fixed 0.018 m depth, carry, release
Receipt: `results/fs_..._v2`, seeds 51,53,55,57,59,61,63,65 -> **5/8**
(51,53,59,61,63 ok; 55,57,65 fail).

Two mechanisms, both visible in the logs:
1. **Depth decides the carry.** The descent stops 6-11 mm *above* the commanded
   z every time (POS_TOL). Where that left the pinch 11 mm below the rim
   (55: achieved 0.9596, 57: 0.9592) the gap collapsed 0.0083 -> 0.0048 in
   transit — the bowl slipped out mid-carry. Where a retry reached 16 mm
   (51: 0.9547) it held to the release.
2. **The carried bowl cannot be re-perceived.** `_held_bowl` locked onto the
   arm blob (z[1.244..1.320], centred on the tool, `off=(+0.0015,-0.0002)`),
   reporting a 0 m hold offset. The release then aimed the *tool* at the goal
   centre, putting the bowl 0.048 m off it — ep55 ends with the disc at
   (-0.019, -0.318) against a goal at (-0.019, -0.279).

### v3 — depth + bias compensation, computed hold geometry
Changes: grasp depth 0.026 m commanded with an explicit +0.008 m tolerance bias;
pinch radius taken from the bowl's *own* measured wall profile at the grasp
height (inset 4 mm); hold offset computed as `wall_radius(z_grasp) - gap/2`
instead of re-perceived; carry height lowered to goal/rim + 0.09; hold verified
at every waypoint with a full re-acquire-and-retry cycle (target re-identified
by proximity to its last known centre, not by height, since a dropped bowl is no
longer elevated).

Receipt: `results/fs_..._v3`, same 8 seeds -> **1/8**. A clear regression, with
two named causes:

1. **The tolerance bias was applied twice.** `z_want` already carried the
   0.026 m depth; subtracting `MOVE_BIAS_M` again commanded 0.034 m and the
   jaws landed 27-28 mm below the rim (ep51: cmd 0.9367, achieved 0.9435).
   At that depth the close *reports* a gap (0.0078-0.0094) but the bowl falls
   straight back out on the lift — 8 of 9 attempts in ep51 went
   `close gap=0.0094 -> lift gap=0.0046`. Too deep is as bad as too shallow;
   the window is 16-20 mm.
2. **The retry ladder exhausted the episode.** LIBERO's horizon here is 500
   control steps (`tools/fair_run.py` default) and `api.grip` alone costs 20.
   Three cycles x three attempts = nine grips, nine settles: ep51 ran out
   mid-place. The receipt is unmistakable — `over goal`, `descend` and the
   final capture all carry the same eef and the same timestamp to 3 ms, and
   the descent reports `res=0.3077`. A terminated environment accepts calls
   and silently does nothing.

### v4 — one ladder, three attempts, correct depth, gentle break-out
Changes from v3: the depth ladder commands 0.026/0.021/0.031 m directly (the
tolerance bias is what turns those into the 16-20 mm that holds); at most three
attempts and no outer cycle; a 0.02 m unsaturated first lift before the full
one; release z no longer over-corrected.

Receipt: `results/fs_..._v4`, seeds 51,53,55,57,59,61,63,65 -> **8/8**, 192-321
sim steps against a 500-step horizon (63 and 65 needed a second attempt).

Selection receipt (formal, all 15 debug seeds):
`results/sel_c2k1clean_spa_bowl_on_cookie_box_pos_k1_v4` -> **15/15**.

| seed | success | note | sim_steps |
|------|---------|------|-----------|
| 51 | true | placed | 194 |
| 52 | true | placed | 321 |
| 53 | true | placed | 207 |
| 54 | true | placed | 192 |
| 55 | true | placed | 199 |
| 56 | true | placed | 200 |
| 57 | true | placed | 198 |
| 58 | true | placed | 198 |
| 59 | true | placed | 203 |
| 60 | true | placed | 198 |
| 61 | true | placed | 192 |
| 62 | true | placed | 198 |
| 63 | true | placed | 321 |
| 64 | true | placed | 193 |
| 65 | true | placed | 320 |

12/15 seated the bowl on the first attempt; 52, 63 and 65 took a second rung of
the depth/inset ladder. Nothing reached rung three and nothing came within
170 steps of the horizon.

---

# DECLARATION

- **Frozen version:** v4. `packs/c2k1clean_spa_bowl_on_cookie_box_pos_k1/program.py`
  md5 `8919d673684970da703ca07e177b09b3` == `program_v4.py` md5
  `8919d673684970da703ca07e177b09b3` (verified on the cluster).
- **Selection receipt:** **15/15** on the full 15-seed debug band,
  `results/sel_c2k1clean_spa_bowl_on_cookie_box_pos_k1_v4`.
- **Per-version receipt chain:**
  - v1 (recon, no motion) — `results/fs_..._v1` (51,53,57,61) + `results/fs_..._v1b`
    (52,54,55,56,58,59,60,62,63,64,65). 0/15 by construction.
  - v2 — `results/fs_..._v2`, 8 probe seeds, **5/8**.
  - v3 — `results/fs_..._v3`, same 8 probe seeds, **1/8**.
  - v4 — `results/fs_..._v4`, same 8 probe seeds, **8/8**;
    `results/sel_..._v4`, all 15 debug seeds, **15/15**.
- **Archived versions:** `program_v1.py`, `program_v2.py`, `program_v3.py`,
  `program_v4.py` in the pack directory.
- **PROVENANCE:** present in `program.py` as a top-level literal dict, one entry
  per calibrated constant (RIM_BAND_M, SEG_DZ_M, DEEP_DISC_RANGE_M,
  FLAT_DISC_RANGE_M, GRASP_DEPTH_LADDER_M, MOVE_BIAS_M, GENTLE_LIFT_M,
  PINCH_INSET_LADDER_M, HELD_GAP_MIN_M, CARRY_CLEAR_M, PLACE_CLEAR_M,
  REACQUIRE_R_M). Every source is this pack's own contents or a debug-seed
  measurement made in this cell; no constant is carried in from outside it.
- **Splits:** seeds 1-50 were never run, listed or read from this cell.

STOP.
