# c2k1clean / obj_salad_dressing_pos_k1

Intent: "pick up the salad dressing and place it in the basket".
Runner: `tools/fair_run.py` only. Pack: `packs/c2k1clean_obj_salad_dressing_pos_k1/`
(K=1, 4 keyframes, ee_path6, 137 raw actions).

## Pack reading

| t | ee xyz | gripper_cmd | gripper_state |
|---|--------|-------------|---------------|
| 0   | (-0.1477, 0.0084, 0.2565) | -1 (open) | 0.0362/-0.0362 |
| 51  | ( 0.0787,-0.1025, 0.1238) | +1 (close) | 0.0393/-0.0395 |
| 128 | ( 0.0183, 0.2201, 0.1709) | -1 (open) | 0.0091/-0.0089 |
| 136 | ( 0.0131, 0.2253, 0.2092) | -1 | 0.0337/-0.0334 |

So the demo: grasp at ee z 0.1238, transport, release over the basket at
(0.018, 0.220, 0.171), retreat to 0.209. Held width at release ~0.018 m.
Keyframe t=136 shows a dark bottle sitting in the basket that was standing on
the table at t=0 -> the target is a bottle, grasped near its neck.

## v0 -- perception probe (seeds 51,53,55,57)

`results/fs_c2k1clean_obj_salad_dressing_pos_k1_v0`. No motion; dumped the
cam_high cloud.

- Camera: K = 618.04 / centre 256; `t_base_cam` translation (0.897, 0, 0.65),
  looking back along -x and down. In the image, right = +base_y, down = +base_x.
- Table plane: modal z = 0.0009 m. Cloud z range 0.001 .. 0.486.
- Clusters (0.02 m grid, 8-neighbour union-find, > table+0.012):
  parked arm h=0.396 at (-0.124, 0.002); basket n~1.17e4 ext 0.157x0.171
  h=0.141 at (0.076, 0.255); four props, all h <= 0.147, ext <= 0.081.

**Hypothesis falsified here:** the demo's grasp xy (0.0787, -0.1025) coincides
almost exactly with a debug-seed prop at (0.077, -0.098) -- but the gif frame of
seed 51 shows that prop is a *milk carton*, and the salad-dressing bottle
(white body, green cap) is a different cluster at (0.166, 0.028). The demo scene
carries a different prop set from the debug scenes, so **the demo anchor is a
decoy**; the target must be found by appearance.

Verdict: identify by colour + shape, not by the demo xy.

## v1 -- appearance-selected pick and place (seeds 51,53,55,57,59,61,63,65)

`results/fs_c2k1clean_obj_salad_dressing_pos_k1_v1` -- **8/8**.

Mechanism:
1. Cluster the above-table cloud. Tag `ARM` if h > 0.25, `BASKET` if either
   extent > 0.11, else `prop`.
2. Target = the prop with h > 0.06 maximising greenness `g - (r+b)/2` on the
   cluster's mean RGB. On every probe seed: target +9.8, runner-up -4.9
   (the ketchup bottle), then -5.8 and -11.6. Clean margin.
3. Grasp point from the band `[ztop-0.045, ztop-0.008]`: y-centre from the
   2nd/98th percentiles (the whole width is visible), x from the *near*
   surface minus the half-width, because the camera sees only the near arc.
   Grasp z = ztop - 0.025 (the neck; matches the demo's 0.1238 under a
   0.146 m top).
4. Open, hover 0.28, descend, close, lift 0.30, translate over the basket,
   descend to 0.19, open, retreat.

Receipt (ep51): profile shows the bottle tapering 0.063 m wide at z=0.03 to
0.034 m at z=0.10-0.14 -- a neck. Close gave `effort 3.0, width 0.037` and the
effort held through the lift and the transport.

Fragility seen: the basket release xy came from the whole-cluster
xmin/xmax midpoint, (0.008, 0.256), while the cluster *median* was (0.076,
0.255). If the median were the truth the release sat near the far wall.

## v2 -- rim-band basket centre (seeds 51,53,55,57,59,61,63,65)

`results/fs_c2k1clean_obj_salad_dressing_pos_k1_v2` -- **8/8**.

Only change: the basket centre is read from the rim band (points within
0.020 m of the basket top), which is a closed rectangle seen from above and so
has an unbiased midpoint. Result: rim centre (0.009, 0.254) vs whole-cluster
midpoint (0.008, 0.256) -- they agree, so the earlier median (0.076) was just
point-density bias from the near wall and the interior floor. The release point
was already the true centre; v2 makes that a measurement rather than a
coincidence. Behaviour identical, 8/8.

## Scene variation across debug seeds

The `_pos` perturbation here moves the **basket** (rim centre ranged x
-0.009..0.011, y 0.244..0.259 over the eight probe seeds) while the target
cluster was invariant (0.166, 0.028), n=2601 every seed. Perception is done
per-episode regardless, so this costs nothing.

# DECLARATION

- **Frozen version: v2.** `packs/c2k1clean_obj_salad_dressing_pos_k1/program.py`
  md5 `b2fb8b913c22def39538a300afc454ef` == `program_v2.py` (same md5, verified
  on the cluster).
- **Selection receipt: 15/15** on the full debug band 51-65,
  `results/sel_c2k1clean_obj_salad_dressing_pos_k1_v2` (15 lines in
  results.jsonl, all `"benchmark_success": true`).
- Version receipt chain:
  - v0 `fs_..._v0` -- perception probe, 0/4 by construction (no motion).
  - v1 `fs_..._v1` -- 8/8 on 51,53,55,57,59,61,63,65.
  - v2 `fs_..._v2` -- 8/8 on the same eight.
  - v2 `sel_..._v2` -- 15/15 on 51-65.
- PROVENANCE dict present in program.py, covering TABLE_Z, ABOVE_M, CELL_M,
  ARM_H_MAX, PROP_EXT_MAX, NECK_BELOW_TOP, GRASP_WIDTH, HOVER_Z,
  BASKET_RIM_BAND, RELEASE_Z, CARRY_Z. Every constant sourced to the pack or to
  a debug-seed measurement.
- Archived: `program_v0.py`, `program_v1.py`, `program_v2.py` in the pack dir.
- Splits respected: only seeds 51-65 were ever run; `--split eval` was never
  invoked; `api.done` is never referenced.

STOP.
