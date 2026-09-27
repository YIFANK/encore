# c2k1clean / spa_bowl_next_to_ramekin_task_k1

Intent: **Pick the akita black bowl next to the cookie box and place it on the plate.**
Scene bddl is the *ramekin* task; the intent has been re-authored to name the
*other* black bowl. No shared note file; everything below is derived from the two
packs named in TASK.md plus my own debug-seed (51-65) observations.

## Pack evidence (packs/..._task_k1 and ..._task_mate, pack.json + keyframes only)

| | k1 pack | mate pack |
|---|---|---|
| language | "pick up the black bowl next to the **ramekin** and place it on the plate" | "pick up the black bowl next to the **cookie box** and place it on the plate" |
| grasp frame (gripper_cmd -> close) | t=60, ee xyz (-0.163, 0.351, **0.922**) | t=50, ee xyz (0.146, -0.034, **0.919**) |
| transport apex | t=90 z=1.092 | t=90 z=1.062 |
| release frame | t=131, ee (0.0895, 0.228, **0.945**) | t=122, ee (0.084, 0.233, **0.933**) |

Both packs release at essentially the same spot (~0.085, 0.23) = the plate, so the
plate is the shared target. The two packs therefore bracket my intent: the k1 pack
shows the *plate* half on the wrong bowl, the mate pack shows the *bowl* half in a
different scene. Depths (grasp z, place z, carry z) are the transferable part.

Wrist: `ee_path6` roll is ~pi throughout (tool straight down), yaw small
(0.10-0.34 rad). `api.move(rotation=None)` already holds straight-down, so no
rotation is commanded.

## v0 - perception probe (seeds 51,53,55) - results/fs_..._v0, 0/3 (no motion)

Hypothesis: a cam_high RGB-D height map is enough to name every table prop.
Evidence (seed 51 cluster dump, heights above table, table z = **0.9008**):

| cells | xy | top | ext | mean RGB | reading |
|---|---|---|---|---|---|
| 439 | (0.016,-0.254) | 0.227 | 0.32x0.22 | 67,66,64 | wooden cabinet |
| 358 | (-0.246,-0.088) | 0.470 | 0.20x0.32 | 35,49,56 | robot arm (+ stove under it) |
| 84 | (0.136,-0.074) | 0.051 | 0.12x0.13 | 113,115,111 | **black bowl A** |
| 79 | (-0.183,0.320) | 0.051 | 0.12x0.12 | 99,99,97 | black bowl B |
| 79 | (0.065,0.216) | 0.019 | 0.15x0.15 | 153,142,139 | **plate** |
| 63 | (0.075,0.035) | 0.020 | 0.09x0.07 | 93,66,46 | **cookie box** (brown) |
| 43 | (-0.206,0.192) | 0.043 | 0.09x0.10 | 116,117,118 | ramekin |

Cross-check against the packs: the k1 pack grasps at (-0.163,0.351) = bowl B, which
sits beside the ramekin (-0.206,0.192) -> matches its language. The mate pack grasps
at (0.146,-0.034) = bowl A's location, beside the cookie box (0.075,0.035). So
**my target is bowl A, the one nearest the cookie box**, and my goal is the plate.

Verdict: identification is solved. Two traps recorded: on seed 53 the bowl and the
cookie box *fuse* into one 0.17x0.19 cluster when a single height threshold is used,
and the cabinet/arm would fuse with a neighbouring bowl. Both are fixed by keying
each 1 cm cell on its **max** height and then selecting height *windows*
(0.030-0.075 = rims, 0.010-0.030 = flats), which removes tall things cell-wise
instead of cluster-wise.

## v1 - first full pick-and-place (running)

Hypothesis: height-window blobs + "rim blob nearest the brown flat blob" names the
target bowl; replaying the packs' depths (grasp table+0.0197, carry 1.065, place
table+0.045) executes it.

Evidence (seed 51, full 8 debug-seed probe `results/fs_..._v1`): identification worked
on every seed - cookie (0.075,0.035), plate (0.065,0.215), bowl (0.140,-0.075) - but
**0/8**. The descent stalled at ee z 0.949 with a 29 mm residual and the close produced a
1 mm gap: the jaws were shut on air.

Verdict: perception right, grasp wrong. The jaws span 78 mm and the bowl is 120 mm wide,
so aiming at the bowl centre simply rests both fingers on the rim annulus. Both packs
grasp ~36 mm OFF the bowl centre and hold an 11.5 mm gap - the bowl wall. It is a rim
pinch: one finger inside the bowl, one outside.

## v2 - grasp-direction diagnostic (seeds 51,55,61, `--horizon 4000`)

Hypothesis: offsetting the aim by 38 mm along the jaw axis brackets the wall; a ladder
of four base-frame directions will say which axis the jaws close along.

Evidence: the very first rung, **+y**, gripped on all three seeds - effort 3.00, gap
0.0074-0.0120, descent reaching ee z 0.917-0.925 (the packs' 0.919/0.922). `tool_rotation`
at home is `[[.998,0,-.057],[0,-1,0],[-.057,0,-.998]]`, so tool y is base -y: the jaw axis
is base y, as the +y rung implies.

Second finding, a trap: the run crashed on rung 2 (`bowl -> None`). **Re-perceiving
mid-episode does not work** - with the arm standing over the table the cookie-box blob is
lost. Perception must happen once, before the arm enters the frame, and the rest of the
episode must run on sensors.

Verdict: rim pinch along +y, single up-front perception.

## v3 - first working program - `results/fs_..._v3` = **7/8**

(51,53,55,59,61,63,65 true; 57 false). Perceive once, aim `bowl + (0, +0.038)`, press to
table+0.018, close, lift, carry to `plate + (0, +0.038)` so the bowl - which hangs one
offset behind the tool - lands on the plate centre, release at table+0.045.

Seed 57 evidence: close gap 0.0085 at ee z 0.9286, then the lift dropped it to 0.0047
(below the harness's 5 mm held-gap floor). Seed 51 reached 0.9235 and held at 0.0122.

Verdict: the miss is depth. `move()` stops anywhere inside a 12 mm tolerance, and 5 mm of
that is the difference between pinching the thin upper lip and the thicker wall below.

## v4 - press through the tolerance - `results/fs_..._v4` = **8/8**

Commanded z = table+0.000 instead of table+0.018, so the descent is ended by contact (the
inner finger bottoms on the bowl's interior floor, measured at table+0.027) rather than by
the tolerance; three retry rungs at offsets 0.038/0.044/0.032.

Evidence: every seed reached ee z 0.919-0.923 (variance gone). 8/8, but the first rung
still lost the bowl on the lift on 4 of 8 seeds (gap 0.012 at the close -> 0.002-0.0045
after) and seeds 55/63 cost 598/600 sim steps.

## v5 - 0.044 as the primary rung - `results/fs_..._v5` = **8/8**

Hypothesis: 0.044 held every seed that 0.038 dropped in v4, so it should be rung 0.
Evidence: 8/8, but rung 0 still hit only 4/8 (55,57,61,63 needed rung 1). Verdict: the two
offsets are interchangeable - **the close is fine on every rung (gap ~0.012); the bowl is
lost on the lift.** Aim is not the lever.

## v6 - two-stage lift - `results/fs_..._v6` = **8/8**, `results/sel_..._v6` = **14/15**

Hypothesis: a single 16 cm climb saturates the position command and shakes the bowl out.
Split it (table+0.055, then table+0.160) and settle 0.3 s after the close.

Evidence: 8/8 with rung 0 hitting 5/8, and the peak cost fell from 600 to 412 sim steps.
Formal 15-seed run: **14/15**, seed 54 the only miss.

Seed 54 evidence: all three axial rungs stalled 5 mm high (z 0.9246-0.9272 vs 0.921
elsewhere), closed on a thin 0.0082-0.0096 gap and slipped to 0.0049 every time. Its
bowl's interior floor measures table+0.030 against table+0.027 on the other seeds - the
inner finger bottoms out sooner, so the pinch never reaches thick wall. Sliding the
bracket along the jaw axis cannot fix that; all three rungs failed identically.

## v7 - chord rungs - `results/fs_..._v7` = **6/6**, `results/sel_..._v7` = **15/15**

Hypothesis: if the pinch depth is capped by the interior floor, move the bracket to a
different **chord** of the rim instead of sliding it along the jaw axis. Two extra rungs
offset 22 mm across the jaw axis (with 4 mm less axial offset, since the wall sits at
sqrt(0.058^2 - 0.022^2) = 0.0537 there rather than 0.058).

Evidence: probe on 54,55,57,63,65,51 = 6/6. Seed 54's rungs 0-2 reproduced v6 exactly and
**rung 3 (the +x chord) held it** - gap 0.0052, benchmark true. Formal 15-seed run:
**15/15**, no seed regressed, peak cost 736 sim steps on seed 54.

Verdict: frozen.

---

# DECLARATION

- **Frozen version: v7.** `packs/c2k1clean_spa_bowl_next_to_ramekin_task_k1/program.py`
  md5 `95cb66221cf0fe1744dbbb0f0e5e9cc3` == `program_v7.py` md5
  `95cb66221cf0fe1744dbbb0f0e5e9cc3` (verified on AbakaAI after the selection run).
- **Selection receipt (full 15 debug seeds, one formal run): 15/15** in
  `results/sel_c2k1clean_spa_bowl_next_to_ramekin_task_k1_v7` - seeds 51-65 all
  `"benchmark_success": true`.
- **Receipt chain**

  | version | run dir | seeds | result |
  |---|---|---|---|
  | v0 (perception probe, no motion) | `results/fs_..._v0` | 51,53,55 | 0/3 by design |
  | v1 (centre grasp) | `results/fs_..._v1` | 8 probe seeds | 0/8 |
  | v2 (grasp-direction diagnostic) | `results/fs_..._v2` | 51,55,61 | diagnostic |
  | v3 | `results/fs_..._v3` | 8 probe seeds | 7/8 |
  | v4 | `results/fs_..._v4` | 8 probe seeds | 8/8 |
  | v5 | `results/fs_..._v5` | 8 probe seeds | 8/8 |
  | v6 | `results/fs_..._v6` / `results/sel_..._v6` | 8 probe / all 15 | 8/8 / **14/15** |
  | v7 | `results/fs_..._v7` / `results/sel_..._v7` | 54,55,57,63,65,51 / all 15 | 6/6 / **15/15** |

  (probe seed list = 51,53,55,57,59,61,63,65; every formally-probed version is archived as
  `program_vN.py` in the pack dir.)
- **PROVENANCE present**: a top-level literal dict with 16 entries, one per calibrated
  constant, every entry `allowed: True` with a source that is either a named pack's
  contents, a debug-seed measurement, or generic controller/camera mechanics. Checked
  against `fair_run.scan_program`'s own rules: no forbidden tokens, no `.done` attribute
  read anywhere in the AST, no uncovered upper-case constant.
- **Protocol**: seeds 1-50 were never touched; `--split eval` was never run; the bddl path
  was only ever passed as an opaque `--bddl` string; the only files written on the cluster
  are `packs/c2k1clean_spa_bowl_next_to_ramekin_task_k1/*` and
  `results/*c2k1clean_spa_bowl_next_to_ramekin_task_k1*`. `tools/fewshot_run.py` was never
  invoked. No shared note file was read or written; no other cell's or campaign's
  artifacts were opened.
