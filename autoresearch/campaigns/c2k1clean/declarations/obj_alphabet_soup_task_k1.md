# c2k1clean / obj_alphabet_soup_task_k1

Intent: **"Pick the cream cheese and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51-65).

## What the two packs say

| pack | language | grasp keyframe (eef) | release keyframe |
|---|---|---|---|
| `..._task_k1` | "pick up the **alphabet soup** and place it in the basket" | t=42, z=**0.0622**, wrist yaw ~0 deg | t=128, (-0.035, 0.225, 0.153) |
| `..._task_mate` | "pick up the **cream cheese** and place it in the basket" | t=54, z=**0.0108**, wrist yaw **-19.4 deg** | t=120, (0.030, 0.266, 0.179) |

Two things the packs settle that nothing else could:

1. **Which item is the cream cheese.** The mate pack's t=0054 keyframe shows the
   fingers closing on the small **blue** box (its t=0000 crop and the blue box in
   the debug scenes are the same asset). Nothing else in the scene is blue.
2. **How high to close.** The mate demo closes at eef z = 0.0108 -- a couple of
   centimetres above the table -- against the k1 demo's 0.0622 on a can. The
   named object is FLAT.
3. **How wide the hold is.** Mate keyframe t=0120 `gripper_state`
   `[0.0199, -0.0239]` -> held width **0.0438 m**, i.e. the jaws span the box's
   short axis. That is the program's own hold check.

The rotation vectors in `ee_path6` have |r| = pi throughout, i.e. they are
axis-angle "straight down with a yaw": R = Rz(theta) @ diag(1,-1,-1), with
theta = -19.4 deg at the mate grasp. That fixes the wrist family to use.

## Debug-seed measurements (seeds 51-65, cam_high RGB-D, v1 probe)

Captured once with a motion-free probe (`program_v1.py`) that dumps `rgb/depth/
K/T` into the run's own results dir, so perception was developed offline against
the exact frames.

- table plane z = **0.0012** on every seed.
- cream cheese: blueness `b-(r+g)/2` = **+20.6**; every other table item <= 0.
  Top face **0.0200** m, footprint **0.079 x 0.041 m**, long axis along base x
  (yaw = 180 deg mod pi == 0). Centre varies over
  x in [-0.155, -0.146], y in [0.055, 0.064] across the 15 seeds.
- confounders: the milk carton has blue lid/lettering, but its blue fragments
  top out at **0.047-0.078 m**; the robot arm renders blue in LIBERO-PRO but
  lives above 0.25 m. A flatness cut at **0.035 m** separates cleanly.
- basket: largest structure below 0.25 m; rim bbox **0.159 x 0.171 m**, rim top
  **0.142** m, centre near (0.00, 0.26).
- tallest table item = bottle at 0.147 m -> carry at 0.26 m.

## Mechanism (program_v2.py)

1. cam_high -> base-frame cloud -> table plane = median z below 0.02.
2. object = largest 4-connected cluster of `blue > 10` pixels with z in
   (table+0.008, table+0.035); regrow a +-0.07 m window around its centroid;
   grasp point = **bbox centre of the top face**, not the cloud mean (the mean
   is pulled by the visible side wall).
3. basket = largest cluster with z in (table+0.015, 0.25); release point =
   **bbox centre of the rim band**, again not the mean (biased to the near wall
   by ~0.02 m).
4. open -> hover 0.14 -> descend to table+0.010 (straight-down wrist, jaws
   already along base y = the box's short axis, so no yaw needed) -> close ->
   lift to 0.26 -> traverse -> descend to rim+0.040 -> open.
5. Verification is by the program's own sensors: gripper width in (0.030,0.060)
   and effort > 1 after the close AND again after the lift. A 3-rung retry
   ladder (re-perceive from home, small z/x offsets) fires only if that check
   fails.

No success signal is read; `api.done` never appears in the source.

## Receipt chain

| version | what changed | run | result |
|---|---|---|---|
| v1 | motion-free perception probe (dumps obs npz) | `results/fs_..._v1`, seeds 51-65 | n/a (probe; 0/15 by construction) |
| v2 | full pick-and-place as above | `results/fs_..._v2`, seeds 51,53,...,65 | **8/8** |
| v2 | (unchanged) formal selection | `results/sel_..._v2`, seeds 51-65 | **15/15** |

v2 needed no retry rung on any probe seed: `att0 after_lift ... width=0.0422
effort=3.00 held=True` on all 8, and the held width 0.0422 m sits right on the
mate pack's demonstrated 0.0438 m.

## DECLARATION

- **Frozen version: v2.**
  `packs/c2k1clean_obj_alphabet_soup_task_k1/program.py` md5
  `54ac12d1f322eaaf76c6e4bc3309d64e` ==
  `packs/c2k1clean_obj_alphabet_soup_task_k1/program_v2.py` md5
  `54ac12d1f322eaaf76c6e4bc3309d64e`.
- **Selection receipt: 15/15** on the full debug band (seeds 51-65),
  `results/sel_c2k1clean_obj_alphabet_soup_task_k1_v2` (15 lines in
  `results.jsonl`, all `"benchmark_success": true`; 15 `ep*_ok.gif`).
- **Per-version receipt chain**
  - v1 `results/fs_c2k1clean_obj_alphabet_soup_task_k1_v1` (seeds 51-65):
    motion-free perception probe, no behaviour.
  - v2 `results/fs_c2k1clean_obj_alphabet_soup_task_k1_v2` (seeds
    51,53,55,57,59,61,63,65): **8/8**.
  - v2 `results/sel_c2k1clean_obj_alphabet_soup_task_k1_v2` (seeds 51-65):
    **15/15**.
- **Margin**: the retry ladder fired on **0 of 15** selection episodes -- every
  episode grasped on rung 0, `held=True` with width 0.0422 m after both the
  close and the lift.
- **PROVENANCE**: present as a top-level literal dict in `program.py`, 8 entries
  (`BLUE_MIN`, `FLAT_TOP_MAX`, `GRASP_Z_ABOVE_TABLE`, `HOLD_WIDTH_RANGE`,
  `CARRY_Z`, `RELEASE_ABOVE_RIM`, `WORKSPACE_BOX`, `HOME_XYZ`), each sourced to
  either a named pack field or a debug-seed measurement, all `allowed: True`.
  No LIBERO-specific prior knowledge was used; every constant is re-derived
  above.
- Clean room respected: writes confined to
  `packs/c2k1clean_obj_alphabet_soup_task_k1/*` and
  `results/*c2k1clean_obj_alphabet_soup_task_k1*`; only `pack.json` +
  `keyframes/` were read from the two named packs; `tools/fewshot_run.py` was
  never invoked; seeds 1-50 were never touched.

STOP.
