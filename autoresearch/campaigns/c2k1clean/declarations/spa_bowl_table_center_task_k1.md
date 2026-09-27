# c2k1clean / spa_bowl_table_center_task_k1

Intent: "Pick the akita black bowl next to the plate and place it on the plate"

## Evidence base

Allowed inputs only: `packs/c2k1clean_spa_bowl_table_center_task_k1/{pack.json,keyframes}`,
`packs/c2k1clean_spa_bowl_table_center_task_mate/{pack.json,keyframes}`, and my own
debug-seed (51-65) observations.

### Packs

| | k1 pack | mate pack |
|---|---|---|
| language | "pick up the black bowl from table center and place it on the plate" | "pick up the black bowl next to the plate and place it on the plate" |
| grasp ee (close) | (-0.0908, 0.0409, 0.9194) | (0.0084, 0.2540, 0.9226) |
| release ee | (0.0634, 0.2268, 0.9342) | (0.0584, 0.1525, 0.9355) |
| transport apex z | 1.069 | 1.180 |

### Debug-seed scene (v0 / v0b probes, seeds 51-58)

- table top z = 0.902 (depth-histogram mode, identical in all 8 seeds)
- two wide bowls, footprint 0.111 x 0.111, top z = 0.951 (table + 0.049)
  - one is **pinned** at x = -0.088, y in [-0.015, +0.012] in every seed
    -> the "table center" bowl
  - the other roams: (0.00, 0.33) in 7 seeds, (-0.123, 0.378) in seed 51
- plate: bright flat disc, footprint 0.136, top z = 0.919 (table + 0.017),
  y ~ 0.190, x in [0.04, 0.10]
- distractors: small round object (footprint 0.087, top 0.944) parked at
  (-0.21, 0.20); dark brown flat box (0.082 x 0.061, rgb 92/65/46).

### Reading that fixes the target

The k1 pack grasps at x = -0.0908 -- exactly the pinned table-centre bowl,
matching its own language. So the intent's "bowl next to the plate" is the
**other** bowl. Cross-check with the mate pack, whose language *is* the
intent: it releases at y = 0.1525, and the plate sits at y ~ 0.19 in every
debug seed, so the tool carries a -0.041 y offset; applying that same offset
to its grasp (y = 0.254) puts its bowl centre at y ~ 0.295 -- the roaming
bowl, not the pinned one. Both packs therefore agree, and they agree with
the intent sentence.

### Motion constants both packs agree on

- **rim straddle**: the bowl (0.111) is wider than the open jaws (0.0362*2
  = 0.072), so the tool sits 0.041 m off the bowl centre along world y and
  pinches the rim wall. k1 uses +y (bowl y~0, plate y~0.186); mate uses -y
  (bowl y~0.295, plate y~0.194). Rule: **the offset points toward the plate.**
- jaw axis is world y (both offsets are purely in y).
- close at z = table + 0.017 / + 0.021.
- release at z = plate_top + ~0.016 (0.9337 / 0.9369 vs plate top 0.919),
  i.e. the same 0.017 tool-above-support the grasp used.
- release xy = plate centre + the same +-0.041 y offset.

## Version log

### v0, v0b -- perception probes (no manipulation)
Hypothesis: the scene must be re-derived from scratch. Evidence: logs above.
Verdict: scene model established; both bowls, plate, table height and the
target-identity rule are measurable from cam_high depth alone.
Receipt: `results/fs_..._v0` (4 eps), `results/fs_..._v0b` (8 eps).

### v1 -- first full pick-and-place
Hypothesis: grasp the roaming bowl by a y-axis rim straddle at table+0.018,
carry at z=1.03, release over the plate centre with the same y offset at
plate_top+0.016.
Evidence: probe `results/fs_c2k1clean_spa_bowl_table_center_task_k1_v1`
(seeds 51,53,55,57,59,61,63,65) = **8/8 benchmark_success**. Per-episode
sensor receipts confirm the mechanism rather than a lucky predicate: gripper
effort 3.00 and width 0.007-0.009 after every close (a real rim pinch), all
move residuals <= 0.017, and the descent onto the plate stalls ~0.016 above
the commanded z because the carried bowl bottoms out on the plate.
Verdict: accepted, no second mechanism needed.

Selection: `results/sel_c2k1clean_spa_bowl_table_center_task_k1_v1`, all 15
debug seeds 51-65 = **15/15 benchmark_success** (15 `ep*_ok.gif`).

## DECLARATION

- **Frozen version**: v1.
  `packs/c2k1clean_spa_bowl_table_center_task_k1/program.py`
  md5 `119eeeb9621d545797df4e9f6fc5a1b4`
  == `program_v1.py` md5 `119eeeb9621d545797df4e9f6fc5a1b4`.
- **Selection receipt**: 15/15 on the full debug split (seeds 51-65),
  `results/sel_c2k1clean_spa_bowl_table_center_task_k1_v1`.
- **Receipt chain**:
  | version | run dir | seeds | result |
  |---|---|---|---|
  | v0 (perception probe) | `results/fs_..._v0` | 51,53,55,57 | scene model: table z=0.902, cabinet/plate/bowl clusters |
  | v0b (perception probe) | `results/fs_..._v0b` | 51-58 | two 0.111 bowls (one pinned at x=-0.088), plate 0.136 @ top 0.919 |
  | v1 | `results/fs_..._v1` | 51,53,55,57,59,61,63,65 | **8/8** |
  | v1 (selection) | `results/sel_..._v1` | 51-65 | **15/15** |
- **PROVENANCE**: present as a top-level literal dict in `program.py`,
  covering ZTAB_FALLBACK, BOWL_BAND, BOWL_FOOTPRINT, PLATE_FOOTPRINT,
  TABLE_CENTRE_SITE, RIM_OFFSET_Y, Z_GRASP_OVER_TABLE, Z_RELEASE_OVER_PLATE,
  CARRY_Z, JAW_AXIS, GRIP_CLOSED_WIDTH. Every source is one of the two named
  packs or a debug-seed (51-65) measurement; no LIBERO-specific prior
  knowledge was used.
- **Clean room**: only `tools/fair_run.py` was ever invoked; writes were
  confined to `packs/c2k1clean_spa_bowl_table_center_task_k1/*` and
  `results/*c2k1clean_spa_bowl_table_center_task_k1*`; no benchmark asset,
  no other cell's artifacts, and no pack `program*.py`/`NOTES.md` were read.

STOP.
