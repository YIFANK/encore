# c2k1clean / spa_bowl_on_cookie_box_task_k1

Intent: **Pick the akita black bowl on the top of the cabinet and place it on
the plate.** Success = the environment's own benchmark bit.

## Evidence read from the two packs (pack.json + keyframes/ only)

- `..._task_k1` language: "pick up the black bowl on the cookie box and place
  it on the plate" — MY scene, but the demo grasps the **cookie-box** bowl:
  grasp ee (0.0687, 0.0642, **0.953**) with gripper_state [0.004, -0.0045];
  release ee (0.0687, **0.2235**, 0.9476).
- `..._task_mate` language: "pick up the black bowl on the wooden cabinet and
  place it on the plate" — a different scene, but it handles **my object**:
  grasp ee (-0.0097, -0.2572, **1.159**) with gripper_state [0.0026,-0.0027];
  release ee (0.0527, 0.2182, 0.9728).
- Both packs close the jaws to ~5-9 mm total: the grasp is a **rim pinch**,
  not an enclosing grasp (the bowl is ~0.11 m across, the jaws open ~0.079 m).
- Both packs release over a plate at ~(0.06, 0.22) — so the plate sits in
  roughly the same place in both scenes.

## Version log

### v1 — perception dump + guessed attempt (seeds 51,53,55,57) → 0/4
Hypothesis: the cabinet-top bowl is the highest bright cluster in cam_high.
Evidence: it is not — the **robot arm** itself is the highest bright cluster
(clusters at z=1.26..1.37 around x≈-0.16, the parked arm), so the ring fit
landed on the arm and all four rim-pinch attempts closed on air (w=0.0010).
Useful measurements from the dump (debug seed 51): table plane z=**0.9025**;
cabinet-top bowl rim top z=**1.179** at y≈-0.29; cookie-box bowl rim top
z=**0.970**; plate rim z=**0.920** at (0.055, 0.239).
Verdict: perception must remove the robot; the demo grasp depths then follow —
1.179-1.159 = 0.020 and 0.970-0.953 = 0.017, i.e. **grasp 0.018 m below the
rim top**.
Receipt: `results/fs_c2k1clean_spa_bowl_on_cookie_box_task_k1_v1` 0/4.

### v2 — robot-free perception + verified rim-pinch ladder (51,53,55,57) → 1/4
Hypothesis: two cam_high captures with the arm displaced between them isolate
the static scene (pixels whose depth is unchanged); the target is then the
highest compact bowl cluster; a rim-pinch ladder over {jaw axis} x {radial
sign}, verified by gripper effort/width, gets the bowl.
Evidence: perception is now stable and repeatable across seeds — target rim
top z = **1.1792** on every seed, rim radius r = **0.054**, plate rim z =
**0.9196**, plate width 0.136 m. The **grasp succeeded on all four seeds**
(effort 3.00, width 0.0066-0.0120, still held after a 0.06 m lift). But only
ep51 scored.
Verdict: the failure is in the **place**, not the pick. The jaws hold the
*rim*, so the bowl centre trails the gripper by r=0.054 m; driving the gripper
to the plate centre puts the bowl centre 5.4 cm off-centre, half off a plate
of radius 0.068. ep53's GIF shows the bowl resting on the plate's edge.
Receipt: `results/..._v2` 1/4 (51 ok; 53,55,57 fail).

### v3 — compensate the rim offset at the drop (8 seeds)
Hypothesis: aiming the gripper at `plate_centre + grasp_offset` lands the bowl
centre on the plate centre. Also release at plate_z+0.046 (v2 seeds 53/57
stalled their descent at eef z=0.955 with plate_z=0.9196, so the bowl bottom
hangs ~0.036 m below the jaws; v2's 0.040 jammed the bowl into the plate on
51/55 and popped the rim out of the jaws).
Evidence: 7/8 on the probe subset (51,53,55,57,59,61,63,65) — every seed but
51. Perception and grasp are now uniform across seeds (rim top 1.1792,
r≈0.054, plate rim 0.9196). On 51 the first rung grips (eff 3.00) but the bowl
slips out during the 0.06 m lift check; the second rung then aims at the
**stale** ring, so the true grip offset differs from the assumed r and the
bowl lands on the plate's edge.
Verdict: right fix, one gap left — a failed attempt moves the bowl.
Receipt: `results/..._v3` 7/8.

### v4 — re-perceive after a failed rung (FROZEN)
Hypothesis: a failed rung nudges the bowl, so before the next rung park the
arm at home, rebuild the static scene, and re-fit the ring of the bowl nearest
the previous target.
Evidence: on seed 51 the re-fit measured the bowl at (0.0204,-0.2845) instead
of (0.0313,-0.2746) — it had been dragged ~1.5 cm by the slip — and the
corrected rim pinch and drop then scored. Probe 8/8, selection **15/15**.
Verdict: accepted, frozen.

## DECLARATION

- **Frozen version: v4.** `packs/c2k1clean_spa_bowl_on_cookie_box_task_k1/program.py`
  md5 `b55aaa1a160ba2b0e1a7cd46e7cf2b7d` == `program_v4.py` (same md5).
- **Selection receipt (full 15 debug seeds 51-65): 15/15**, directory
  `results/sel_c2k1clean_spa_bowl_on_cookie_box_task_k1_v4`
  (`benchmark_success: true` on 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65).
- **Per-version receipt chain**
  | version | probe | receipt dir |
  |---|---|---|
  | v1 | 0/4 (51,53,55,57) | `results/fs_..._v1` |
  | v2 | 1/4 (51,53,55,57) | `results/fs_..._v2` |
  | v3 | 7/8 (51,53,...,65) | `results/fs_..._v3` |
  | v4 | 8/8 (51,53,...,65) | `results/fs_..._v4` |
  | v4 | **15/15** (51-65, formal selection) | `results/sel_..._v4` |
- **PROVENANCE**: present as a top-level literal dict in `program.py`, covering
  `GRASP_DEPTH`, `CLOSED_RIM_W`, `OPEN_W`, `RELEASE_ABOVE_PLATE`,
  `PLACE_XY_FALLBACK`, `PROBE_SHIFT`, `TABLE_BIN`. Every constant is sourced
  either from the two named packs' `pack.json` keyframes or from my own
  debug-seed (51-65) RGB-D / gripper measurements. No seed 1-50 was touched;
  `tools/fewshot_run.py` was never invoked; `api.done` is never read.

STOP.
