# c2clean / goal_put_bowl_on_plate_pos_k3 — working notes

Intent: "put the bowl on the plate". Runner: tools/fair_run.py only.
Pack: packs/c2clean_goal_put_bowl_on_plate_pos_k3 (K=3 demos, keyframes).

## Pack reading (what the demos say)

- 3 demos, ~90 steps each. Straight-down wrist throughout (rpy roll ~3.09-3.13).
- Close (gripper_cmd -1 -> +1) at t=40 in all three: eef
  (-0.1027,0.0301,0.9252) / (-0.1087,0.0439,0.9179) / (-0.1040,0.0424,0.9199).
- Release (cmd +1 -> -1) at t~76-87: eef (0.0448,0.0121,0.9359) /
  (0.0499,0.0313,0.9280) / (0.0500,0.0396,0.9309).
- Transport z ~1.018-1.032.
- Closed finger qpos ~0.005 per finger => a thin-wall pinch, not a body grasp.
- The demo scene layout is NOT my seeds' layout (this is a _pos cell), so demo
  xy is a decoy; only the demo's *relative* geometry transfers.

## Calibration derived from pack + debug seeds

- Ray-cast the demo0 keyframe bowl pixel (250,288 at 512-scale) onto the rim
  plane z=0.952 (rim height measured on my own debug seeds) -> demo bowl centre
  (-0.0755,-0.0096). Demo grasp minus that = (-0.027,+0.040), |0.048| m,
  dominant +y => a RIM PINCH at ~ one rim radius, along the jaw axis. The home
  tool rotation maps tool y -> base -y, so base y IS the jaw-closing axis.
  Cross-check: the same ray-cast of the demo plate pixel gives (0.047,-0.016)
  vs the demo release xy (0.045,0.012) — the release is over the plate centre.
- Grasp height: demo close z 0.918-0.925 vs measured rim top 0.952 =>
  eef sits 0.032 below the rim top at the close (GRASP_DZ).

## Versions

- v1 (probe, no motion): streamed cam_high + cam_arm_wrist RGB-D out through
  api.log (zlib+base64 chunks) on seeds 51/53/57/61 and reconstructed them
  offline. Table z = 0.900 (dominant mode). Props: bowl rim ring r=0.0528,
  ztop 0.952; plate rim ring r=0.0648, ztop 0.920; stove slab 0.19x0.19 top
  0.928-0.932; bottle top 1.059; cabinet; small box. Verdict: geometry
  recoverable from depth alone on every seed.
- v2 (first pipeline, 51/53/57/61): 0/4. Grasp itself worked on the first try
  (close -> effort 3.0, width 0.0085). Failure was perception: the plate was
  selected as "largest blob in the flat band", which is the BOWL'S OWN OUTER
  WALL ring — the arm lifted the bowl and set it back down in place.
- v3 (plate band tightened + bowl-xy exclusion, 8 probe seeds): 7/8
  (fail: 61). In 61 the plate's band blob FUSED with the wine bottle standing
  beside it (dx 0.216, asp 0.63 -> rejected) and the selector fell through to a
  0.10 m blob on the stove top; the bowl was dropped on the stove.
  Side finding from the logs: on 5 of the 8 seeds the gripper reads width
  0.0036 / effort 0.05 during the carry (no squeeze) yet the bowl still rides
  to the plate — the closed fingers HOOK the rim rather than pinch it. Grip
  effort is therefore NOT a usable held/not-held receipt on this task.
- v4 (ring perception): both vessels found as a HOLLOW RING of known radius —
  annulus vote on a 4 mm grid, interior must be empty (<3% of the ring count)
  and the ring must cover >=30 of 36 angular bins. Validated offline on the
  seed 51/53/57/61 clouds: exactly one hit per band per seed, plate centre
  (-0.197,-0.039)/(-0.196,-0.056)/(-0.199,-0.040)/(-0.207,-0.040), bowl centre
  matching the v3 cluster fit to <1 mm. Immune to the bottle fusion that broke
  seed 61, and to the filled stove slab (fails the hollow test).
- v5 (swept plate band, min_bins=24): 14/15 on the full debug band
  (results/sel_c2clean_goal_put_bowl_on_plate_pos_k3_v5, fail: 54). The plate
  is raised/occluded by the cabinet on 54/64, so its rim clears only ~24 of 36
  angular bins; my offline replay of the seed-54 cloud passed at exactly 24,
  the live run (float32 depth vs my float16 dump, ~0.7 mm) fell below. A
  threshold with no margin is not a criterion.
- v6 (FROZEN): candidates are ranked by angular completeness instead of being
  cut at a threshold, and a plate candidate must own a FLOOR — points inside
  the rim, above the table, below the rim. On seeds 51/54/64 the true plate
  holds 830-1000 floor points and every phantom arc (stove edge, cabinet
  corner, table seam) holds exactly 0, so the ranking cannot be stolen by a
  16-20 bin arc. 15/15 on the full debug band. Offline replay of all 11 dumped
  clouds also survives 0.8 mm of injected depth noise with centres stable to
  ~3 mm.
- v7/v8 (aim envelope, seeds 51/54/58/62): RIM_OFFSET 0.040 -> 4/4,
  0.048 (frozen) -> 15/15, 0.056 -> 3/4 (fail 54). The frozen offset sits
  inside the safe side of the envelope; the failing edge is ~+8 mm away.

## DECLARATION

- Frozen version: **v6**. `packs/c2clean_goal_put_bowl_on_plate_pos_k3/program.py`
  md5 `acdbfcd7df20cf5eaf42eb59e790e2a3` == `program_v6.py` (same md5).
- Selection receipt (full 15 debug seeds 51-65, one formal run):
  **15/15**, `results/sel_c2clean_goal_put_bowl_on_plate_pos_k3_v6`.
- Receipt chain:
  - v1 perception dump, seeds 51/53/57/61 —
    `results/fs_c2clean_goal_put_bowl_on_plate_pos_k3_v1` (no motion);
    even seeds 52-64 — `..._v1even`.
  - v2 — `..._v2` 0/4 (51/53/57/61).
  - v3 — `..._v3` 7/8 (51,53,...,65).
  - v4 — `..._v4` 8/8 probe; selection `sel_..._v4` **13/15** (54, 64).
  - v5 — selection `sel_..._v5` **14/15** (54).
  - v6 — probe-free selection `sel_..._v6` **15/15**.
  - v7 (offset 0.040) `fs_..._v7` 4/4; v8 (offset 0.056) `fs_..._v8` 3/4.
- PROVENANCE: present in program.py, every calibrated constant sourced to the
  pack (demo grasp/release eef, ee_path6 transport z, keyframe ray-casts) or to
  my own debug-seed RGB-D measurements. No foreign constants.
- Clean room: only `packs/c2clean_goal_put_bowl_on_plate_pos_k3/*` and
  `results/*c2clean_goal_put_bowl_on_plate_pos_k3*` were written; no forbidden
  file was read; `tools/fewshot_run.py` was never invoked; `api.done` is never
  read.
