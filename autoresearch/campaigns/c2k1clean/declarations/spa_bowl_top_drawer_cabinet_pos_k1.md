# c2k1clean / spa_bowl_top_drawer_cabinet_pos_k1

Task: "pick up the black bowl in the top drawer of the wooden cabinet and place
it on the plate".  K=1 pack, `_pos` perturbation cell, debug seeds 51-65.

## Pack reading (the only task-specific input)

`pack.json`, demo0, length 153, stride 10.  Keyframes (ee = xyz + euler rpy):

| t | ee xyz | euler rpy | grip cmd | finger gap |
|---|--------|-----------|----------|------------|
| 0 | (-0.204, 0.002, 1.167) | (3.132, 0.028, -0.087) | open | 0.0724 |
| 48 | (0.037, -0.119, 1.092) | (3.091, 0.728, -0.206) | CLOSE | 0.0784 |
| 125 | (0.048, 0.274, 0.941) | (3.001, 0.141, -0.067) | close | 0.0094 |
| 139 | (0.076, 0.251, 0.945) | (3.034, -0.182, -0.092) | OPEN | 0.0094 |
| 152 | (0.059, 0.284, 0.963) | (3.002, -0.386, -0.161) | open | 0.0774 |

Read-outs:
- The grasp is a **rim pinch**: the held finger gap is 9.4 mm, far narrower
  than any bowl body, so the jaws are on the bowl wall.
- The demo descends with a growing wrist **pitch** (0.45 rad at t=40, 0.73 at
  t=48, 0.78 at t=50) and unwinds it during the transport (0.68 at t=100,
  0.28 at t=120, 0.14 at the release).  So the grasp is tilted ~42 deg.
- Release is ~0.04 m above the demo's table, at the plate.
- No drawer-opening phase: the top drawer is already open at t=0.

`_pos` cell, so the demo xy is a decoy; only the offsets/heights transfer.

## Version log

### v1 — perception probe (height/luminance map), seeds 51,53
Hypothesis: cam_high RGB-D alone can localise the scene.
Evidence: table modal z = 0.8987/0.9025 (bin resolution), camera at
(0.659, 0, 1.610) looking down the -x axis; **image right = +y, image up = -x**.
Coarse 2.5 cm height map shows one big elevated structure at
x in [-0.15, 0.2], y in [-0.375, -0.025] (the cabinet) and small low blobs on
the +y half of the table.
Verdict: geometry legible; resolution too coarse to separate bowls from the
cabinet. Receipt: `results/fs_..._v1` (2 eps, perception only).

### v2/v2b — blob table by prominence, seeds 51,53,55,57
Hypothesis: prominence against a 9-15 cm annulus isolates each prop.
Evidence (v2 had a `np.maximum.at` bug against a NaN-initialised grid and
produced an empty map; v2b fixed it).  v2b fuses the whole cabinet + drawer +
both bowls into one 0.30 x 0.35 m blob (top z 1.180 = table+0.277), so the
annulus baseline is inside the same structure.  Only one of the two table props
is recovered.
Verdict: prominence is the wrong operator on a piece of furniture.  Receipt:
`results/fs_..._v2b` (4 eps, perception only).

### v3 — RGB-D dump, all 15 debug seeds
Perception-only run that writes the initial cam_high + cam_arm_wrist frames
(rgb, depth, K, T_base_cam), eef, tool rotation and gripper state to
`results/c2k1clean_spa_bowl_top_drawer_cabinet_pos_k1_dumps/*.npz`, so the
detector can be designed offline instead of through 10-minute cluster rounds.
Debug-seed observations only; no benchmark asset is read.

### Scene, as measured from the debug seeds (cam_high RGB-D only)

All 15 debug seeds share one layout; `_pos` moves the props a few cm but never
changes the heights.  Camera at (0.659, 0, 1.610) looking down the -x axis, so
**image right = +y, image up = -x**.  Modal table plane z = 0.9005.

| feature | measurement |
|---|---|
| cabinet-top bowl | rim table+0.278, centre near (0.02, -0.27) |
| **drawer bowl (target)** | rim table+0.215, rim radius 0.053 +/- 0.001, centre x 0.073-0.100, y -0.161 to -0.129 |
| drawer floor (its support) | table+0.165 -> the bowl is 0.050 m tall |
| drawer / cabinet-top wall tops | table+0.2275 |
| free run outside the rim | +x 0.054-0.064, -x 0.044-0.059, **+y 0.019-0.034, -y 0.014-0.019** |
| plate | top table+0.020, 0.14-0.15 m across, centre near (-0.20, 0.20) |
| ramekin (rival) | top table+0.043, 0.09-0.10 m across |
| cookie box (rival) | top table+0.020, 0.09x0.07 m |

Two consequences drive the whole program:
1. **The target is the lower of the two cabinet bowls.**  The drawer floor sits
   below the cabinet top, so "in the top drawer" is a height relation, not an
   xy one.  Detector: ring-plus-hole rims above table+0.12; among those with
   rim angular coverage >= 28/36 (the drawer bowl scores 29-36, every
   distractor 22-26), take the LOWEST rim.  Correct on 15/15 debug seeds.
2. **The straddle must be along x.**  The jaws separate along the tool y axis,
   which is world y at the home wrist, but the drawer's side walls leave only
   ~2 cm beside the rim in y against ~5.5 cm in x, and the open jaws need
   0.039 m.  So the wrist is yawed 90 deg about world z and the rim is
   straddled at its +x extreme.

### v4 — first full pick-and-place: 4/8
`results/fs_..._v4`, seeds 51,53,55,57,59,61,63,65 -> 51 F, 53 S, 55 F, 57 F,
59 S, 61 S, 63 S, 65 F.  Two independent failure modes, both visible in the
journal:
- **Horizon (51, 55, 57).**  `sim_steps` hit the 1000 cap; the place move ended
  0.20-0.21 m short at z~0.98.  Cause: `move(rotation=R)` never satisfies its
  exit test — measured rot_err settles at 0.03-0.06 rad, which is above the
  controller's rotation tolerance, so **every rotation-carrying move spends its
  entire step budget**.  v4 used rotation=R on all nine moves.
- **Grip slip (55, 65).**  Finger gap after the lift collapsed to 0.0010 m.
  Across the 8 episodes the gap was 0.0033-0.0073 m whenever the bowl came
  along and 0.0010 m when it did not, so the gap — not the `effort` flag, which
  thresholds at 0.005 and called four carried bowls "not holding" — is the
  usable hold sensor.
Verdict: mechanism sound, budget and hold-check wrong.

### v5 — stage the yaw once, then rotation=None: 8/8 probe, **15/15 selection**
Changes: one rotation-carrying move to stage the yaw, every later move
rotation=None (measured: the wrist then drifts only 0.007-0.10 rad, so the yaw
holds); shorter descend/lift; a depth ladder (0.024, 0.032, 0.017) gated on
finger gap >= 0.0025 m with a re-perception between rungs.
- probe `results/fs_..._v5` seeds 51,53,55,57,59,61,63,65 -> **8/8**, 360-761 steps.
- selection `results/sel_..._v5` seeds 51-65 -> **15/15**, 360-901 steps.
The retry ladder never fired: all 15 grasps held on rung 0 (depth 0.024).
Remaining concern: ep56 (901) and ep58 (895) leave only ~100 steps of horizon.

### v6 — same mechanism, trimmed budget + a receipt on every move
No change to perception or to the grasp; only `seconds` trimmed, the redundant
pre-descend hover removed, and every move routed through `_go()` so its
residual and stop point are logged.  Probed on 51,53,55,56,57,58,61,64 (the two
slowest seeds included).
- probe `results/fs_..._v6` seeds 51,53,55,56,57,58,61,64 -> **8/8**, 333-715
  steps (the two slowest v5 seeds fell 901->715 and 895->705).
- selection `results/sel_..._v6` seeds 51-65 -> **14/15**, ep52 F at the 1000
  cap.  The receipts name the cause exactly: `mv over ... res=0.1198` — the
  transport starved at `seconds=2.0` and stalled at x=-0.041 instead of
  -0.156, then `lower` starved too and the wrist drifted to rot_err 0.285.
Verdict: the trim was applied to the wrong move.  **A move that converges
exits early, so a generous `seconds` on it is free; only a move that never
converges spends its whole budget.**  v6's own receipts say which is which:
`stage` (rot_err never clears tolerance) and `retreat` (blocked, res 0.115)
always run to the cap; everything else converges.

### v7 — budget by convergence class: 8/8 probe, **15/15 selection** (FROZEN)
Identical perception and grasp to v5/v6.  Only the step budget changed:
transport 2.0 -> 3.0 s (free, it converges), descend 1.4 -> 1.6, lift 0.9 ->
1.0, lower 1.6 -> 1.8, carry 1.4 -> 1.2, and the always-blocked retreat
0.9 -> 0.4 s.
- probe `results/fs_..._v7` seeds 52,54,56,58,60,62,64,51 -> **8/8**, 313-730
  steps; ep52 now finishes in 427 (v6: 1000 and a fail).
- selection `results/sel_..._v7` seeds 51-65 -> **15/15**, 313-730 steps.

## Receipt chain

| version | what changed | run | result |
|---|---|---|---|
| v1 | height/luminance map probe | `results/fs_..._v1` (51,53) | perception only |
| v2/v2b | prominence blob table | `results/fs_..._v2b` (51,53,55,57) | perception only; cabinet fuses |
| v3 | RGB-D dump, 15 seeds | `results/fs_..._v3` (51-65) | dumps for offline design |
| v4 | first pick-and-place | `results/fs_..._v4` (8 eps) | **4/8** |
| v5 | stage yaw once, rotation=None, hold gate on finger gap | `results/fs_..._v5` (8) / `results/sel_..._v5` (51-65) | 8/8 / **15/15**, max 901 steps |
| v6 | budget trim + per-move receipts | `results/fs_..._v6` (8) / `results/sel_..._v6` (51-65) | 8/8 / **14/15**, ep52 starved |
| v7 | budget by convergence class | `results/fs_..._v7` (8) / `results/sel_..._v7` (51-65) | 8/8 / **15/15**, max 730 steps |

## DECLARATION

- **Frozen version: v7.**  `packs/c2k1clean_spa_bowl_top_drawer_cabinet_pos_k1/program.py`
  md5 `0b87530858e4d2078cd7b51967ead3a2` == `program_v7.py` md5
  `0b87530858e4d2078cd7b51967ead3a2`.
- **Selection receipt (full 15 debug seeds, one formal run):**
  `results/sel_c2k1clean_spa_bowl_top_drawer_cabinet_pos_k1_v7`,
  episodes 51-65, **15/15 `benchmark_success: true`**, 313-730 sim_steps
  (horizon 1000, so >= 270 steps of margin on every seed).
  v5 also scored 15/15 (`results/sel_..._v5`) but with only 99 steps of margin
  on its worst seed; v7 is the argmax on the tie-break that matters for blind
  eval seeds whose transport may be longer.
- **Receipt chain:** the table above; every formally probed version is
  archived as `program_vN.py` in the pack directory (v1-v7).
- **PROVENANCE:** present as a top-level literal dict in program.py, 11
  entries, every one `allowed: True` with a source that is either this pack's
  demo0 or a debug-seed (51-65) RGB-D measurement.  No `.done` read anywhere
  in the program.
- Splits respected: seeds 51-65 only, always via `tools/fair_run.py --split
  debug`; `tools/fewshot_run.py` was never invoked; no benchmark asset was
  read.  Cluster writes confined to
  `packs/c2k1clean_spa_bowl_top_drawer_cabinet_pos_k1/*` and
  `results/*c2k1clean_spa_bowl_top_drawer_cabinet_pos_k1*`.
