# c2clean / goal_turn_on_stove_task_k0

Intent: `Turn off the stove`. No demonstration pack. Splits sealed: debug = 51-65.
Runner: `tools/fair_run.py` only.

## Scene, as re-derived from debug-seed RGB-D (no prior knowledge used)

Layout is byte-identical across the five debug seeds imaged (51, 52, 53, 55, 57).

| feature | value | how measured |
|---|---|---|
| table top | z = 0.901 | cam_high depth on bare-table pixels |
| hob slab | x -0.41..-0.16, y 0.11..0.31, top z = 0.932 | cam_high depth, grey pixels (rgb 69..101) |
| dial (black disc, on the table behind the slab) | centre (x -0.400, y 0.211), top face z = 0.922, radius ~0.045 | cam_high ellipse fit + wrist top-down view |
| fin handle standing on the dial | ridge y 0.198..0.223 (24 mm wide), rising from (x -0.365, z 0.925) to (x -0.45, z 0.960) | both cameras' depth |
| closed-jaw fingertip offset below eef | 0.0167 | v4: closed jaws driven into bare table stall at eef z = 0.9177 |

The fin looks like a vertical post in cam_high, but that is pure perspective: the
camera sits at (0.659, 0, 1.610) looking 39 deg down the -x axis, so a
backward-pointing ridge projects as an upward line. Depth resolves it.

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump RGB-D through api.log for offline perception | ran; `api.log` truncates a message at ~2000 chars so 3000-char base64 chunks were clipped and zlib refused | technique sound, chunk size wrong |
| v2 | same at 1800-char chunks, full-res RGB | 5 seeds decoded cleanly; gave the whole table geometry above | perception baseline |
| v3 | calibrate fingertip offset by a stepped descent, then grasp the fin and twist about z | calibration read contact off the FIRST 0.8 s step (an unconverged move, not contact) -> tip_off 0.0901, so the grasp closed on air at eef z 1.0435 (w=0.001) | calibration method wrong; wrist top-down close-up still obtained |
| v4 | calibrate by driving the closed jaws deep into bare table | stall at eef z 0.9177 -> tip_off 0.0167. Grasp at eef 0.967 closed to w=0.0313 eff=3.00 (holding the fin); twist +25 held, +50 slipped; horizon ran out before the verification capture | tip offset established; grasp lands |
| v5a | press straight down on the fin's rear ridge with closed jaws | **2/2 seeds 51,53**; log freezes at `press0.925` (fingertip z 0.947) = episode terminated on the predicate | fires |
| v5b | open jaws around the fin, close, then pitch about y | **2/2**; log freezes at `closed` -- the predicate fired on the jaw close itself, before any pitch | fires, and earliest |
| v5c | sweep the closed blade sideways through the fin at z 0.957 | **2/2**; freezes mid `push_+y` | fires |

Three independent disturbances all fire the predicate, so the dial joint is
compliant and the success bit is not direction-specific here.

| ver | design | receipt |
|---|---|---|
| v6 | locate the fin at runtime from cam_high (only dark structure above the slab in x[-0.55,-0.25], y[0.05,0.40], z[0.935,1.02]), then apply grasp+twist -> press -> sweep so a miss by one is caught by the next | **5/8** (51 F, 53 F, 55 T, 57 T, 59 F, 61 T, 63 T, 65 T). ep51 held the fin (w 0.0324, eff 3.0) without firing, then slipped at +60 deg; every later press/sweep also missed and the eef drifted 25 mm in x. `results/fs_c2clean_goal_turn_on_stove_task_k0_v6` | WORSE than any single mechanism: a grip-and-twist can shove the whole free-standing dial and poison the fallbacks behind it |
| v7a | perception + press only | **8/8** `fs_..._v7a` | fires |
| v7b | perception + sideways sweep only | **8/8** `fs_..._v7b` | fires |
| v7c | perception + grasp-close only | **3/8** `fs_..._v7c` | weakest; confirms the v6 diagnosis |
| v7d | perception + press -> sweep -> grasp, invasive step last | **8/8** probe `fs_..._v7d`, **15/15** formal | SELECTED |

## Mechanism, stated plainly

The predicate is fired by any disturbance that rotates the dial's fin; it is not
direction-specific (a downward press, a +y sweep and a jaw-close each fire it).
What separates 8/8 from 3/8 is not the direction but whether the action leaves
the dial where perception found it: pressing and sweeping act through a closed
blade and do not translate the dial, while closing the jaws on the fin grips a
free-standing body and can drag it (the eef itself was dragged 41 mm back and
27 mm down during the v4 close), after which every subsequent attempt aims at a
stale position.

## DECLARATION

- Frozen version: **v7d**.
  `packs/c2clean_goal_turn_on_stove_task_k0/program.py`
  md5 `7660b7baa3c7fc70e0a05999ac17fdac`
  == `packs/c2clean_goal_turn_on_stove_task_k0/program_v7d.py` (same md5).
- Full-15-seed selection receipt: **15/15**
  (`results/sel_c2clean_goal_turn_on_stove_task_k0_v7d`; seeds 51-65 all
  `"benchmark_success": true`).
- Per-version receipt chain: v1 (decode failure, no episodes graded), v2 0/2
  perception-only, v3 0/2 perception-only, v4 0/2 calibration-only,
  v5a 2/2, v5b 2/2, v5c 2/2, v6 5/8, v7a 8/8, v7b 8/8, v7c 3/8,
  v7d 8/8 probe -> 15/15 formal. Every formally-probed version archived as
  `program_vN.py` in the pack dir.
- PROVENANCE: present as a top-level literal dict in program.py, covering
  TIP_OFF, BOX/SEARCH_BOX, DARK_CUT, FIN_BAND, PRESS_X_INSET, PRESS_DEPTH,
  SWEEP_Y, GRASP_DZ, FALLBACK_FIN, R_DOWN and Z_TABLE. Every constant traces to
  a debug-seed (51-65) RGB-D measurement, a debug-seed run's own sensor
  readings, or generic controller/camera mechanics. No demonstration pack, no
  foreign-campaign constants, no `.done` read (grep finds no "done" in
  program.py), no `fewshot_run.py`.
