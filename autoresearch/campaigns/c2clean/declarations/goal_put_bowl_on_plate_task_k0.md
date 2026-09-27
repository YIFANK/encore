# c2clean / goal_put_bowl_on_plate_task_k0 — "Put the wine bottle on the plate"

Zero-demo cell. Everything below comes from debug seeds 51-65 through `tools/fair_run.py`
only. Eval seeds 1-50 were never touched.

## Scene (debug-seed perception, cam_high RGB-D deprojected to base frame)

Table plane z = 0.9010-0.9015. Free props, with height above the table:

| prop | h | footprint | mean RGB |
|---|---|---|---|
| wine bottle | 0.158 | body 0.043 wide, neck 0.0134-0.0151 over 0.105-0.150 | 18 |
| bowl | 0.051 | ~0.11 across, sits between bottle and plate | 104 |
| plate | 0.019 | 0.136 across | 138-150 |
| stove slab | 0.059 | y > 0.115 | 64 |
| blue box | 0.019 | small | 74 |

Cabinet + rack + rear wall fuse into one large cluster at y < -0.119; every free prop
sits at y > -0.104.

## Version chain

| ver | hypothesis | run | evidence | verdict |
|---|---|---|---|---|
| v0 | perception only: can the scene be read through `api.log`? | fs_..._v0 (51,53), fs_..._v0b (51,53,55,57) | zlib+base64 RGB-D through api.log works, but api.log truncates a message at ~2000 chars — chunk at 1900. Deprojection uses the OpenCV convention (table lands at z=0.9015; the OpenGL sign flip puts it at 2.32). | perception pipeline established |
| v1 | tall dark cluster = bottle; grasp the neck top-down at 0.125 above the table, carry, release over the plate | fs_..._v1 51,53,55,57 = **4/4**; sel_..._v1 all 15 = **9/15** | closed width 0.01496 on the 0.0134-0.0151 neck, effort 3.0 every time. All 6 failures (54,56,61,63,64,65) logged no BOTTLE line at all: the bowl abuts the bottle in those layouts and the table-level mask fuses them into one cluster of mean RGB 72, which the `rgb<60` darkness test rejects. | motion sound, perception brittle |
| v2 | v1 + clear-spot selection for the calibration descent, tip-offset fallback, bounded grasp retry, post-release re-perception | fs_..._v2 51,53,...,65 = **5/8** | same three fusion failures (61,63,65); the safety additions cost nothing | fusion is the whole gap |
| v3 | height-gate BEFORE grouping: find the bottle in a 0.105-0.145 band where nothing else in the scene reaches, then measure its axis in a 0.056-0.080 band that clears the 0.051-tall bowl | fs_..._v3 51,54,56,57,61,63,64,65 = **8/8**; sel_..._v3 all 15 = **15/15** | every previously fused seed now yields a BOTTLE line; post-release re-perception finds the bottle standing at the plate's xy (e.g. ep54 bottle (0.051,-0.039) vs plate (0.043,-0.036)) | **FROZEN** |
| v3probe_aimdy | margin: displace the grasp aim by +8 mm in y | fs_..._v3aimdy 51,54,58,63 = **4/4**, effort 3.0, closed width 0.01496 | the closing jaws self-centre on the neck; the grasp is not living on a knife edge | diagnostic only, not a candidate |

## Mechanism

1. **Perceive.** cam_high depth -> base-frame cloud (OpenCV convention). Table z = median
   of the lower 60% of the workspace crop (the crop excludes the rear wall, which
   deprojects past x = -0.45).
2. **Bottle.** Label a 0.105-0.145 height band, not the table-level mask. Nothing else in
   this scene reaches 0.10, so the bowl cannot fuse with the bottle there. Pick the darkest
   blob with y-width 0.006-0.030 at y > -0.11. Refine the axis in the 0.056-0.080 band
   (above the bowl's 0.051 rim): y-centre and radius from the y-extent, x-centre from the
   near-surface x minus the radius — an oblique camera only sees the near half of a
   cylinder, so `xmax - r` is the axis.
3. **Fingertip offset.** Blocked descent onto bare table at a spot verified clear of every
   detected cluster: `tip = eef_z_at_contact - z_table` = 0.0080, identical on every debug
   seed. Fallback constant if the descent is not blocked.
4. **Grasp.** Top-down at mid-neck. A body grasp is impossible: the palm sits directly over
   the grasp point and the neck rises 0.078 above the body's top, so the fingertips can
   never reach the body.
5. **Carry** with the bottle base 0.09 above the table (the bowl, 0.051, is the tallest
   thing on the path), then lower until the base is 0.012 over the plate top and release.

The episode terminates the moment the predicate fires — typically at the release, before
the retreat moves run — so the tail of the log is often absent on a success. No `api.done`
read anywhere.

## DECLARATION

- **Frozen version: v3.** `packs/c2clean_goal_put_bowl_on_plate_task_k0/program.py`
  md5 `d074c594592285e4dd4e89e0ce9b1722` == `program_v3.py` (same md5).
- **Selection receipt: 15/15** on the full debug split (seeds 51-65),
  `results/sel_c2clean_goal_put_bowl_on_plate_task_k0_v3`.
- Receipt chain: v1 4/4 probe -> 9/15 formal; v2 5/8 probe; v3 8/8 hard-seed probe ->
  15/15 formal; aim-displacement diagnostic 4/4 at +8 mm.
- PROVENANCE present in program.py, covering every calibrated constant; all sources are
  debug-seed measurements or generic camera/controller mechanics. No pack (k0 cell), no
  shared note file, no eval-seed contact.
