# c2k1clean / spa_bowl_next_to_ramekin_pos_k1 — worker notes

Intent: "pick up the black bowl next to the ramekin and place it on the plate"
Runner: tools/fair_run.py ONLY. Pack: packs/c2k1clean_spa_bowl_next_to_ramekin_pos_k1/.
Debug seeds 51-65; probe subset 51,53,...,65.

## Pack reading (demo0, 152 steps, stride 10)

Keyframes / ee_path6 (xyz + rpy + grip):
- t=0   home  (-0.2073, 0.0011, 1.1730) rpy (3.1325, 0.016, -0.0772), open
- t=55  close commanded, eef (-0.1575, 0.3512, 0.9318)
- t=60  eef (-0.1628, 0.3508, 0.9225) rpy (3.0541, -0.182, 0.3433)  <- grasp
- t=113 carrying, gripper_state sum 0.0115 (thin feature -> rim pinch)
- t=131 open commanded, eef (0.0895, 0.2280, 0.9450)  <- release
- t=151 home-ish, gripper open

Demo scene reconstruction (keyframes/demo0_t0000.png, grey-blob connected
components in the 128px frame, deprojected with the cam_high K/T measured on
debug seeds — the camera is bit-identical on every seed):
- target bowl rim centre  (-0.2109, 0.3234, 0.9513)
- ramekin rim centre      (-0.2096, 0.1975, 0.9434)
- plate centre            ( 0.0576, 0.2101, 0.9195)
=> GRASP_OFFSET = eef(t60).xy - bowl.xy = (+0.0481, +0.0274), |.| = 0.0553
   (equal to the measured bowl outer rim radius 0.0555 -> an outer rim pinch)
=> release eef - plate = (+0.032, +0.018), same direction; consistent with the
   bowl riding at eef - GRASP_OFFSET, i.e. it was dropped ~centred on the plate.
Identifying the target: the bowl present at t=0 and absent at t=113 is the one
at u=110 (128-frame); the ramekin is the shorter, narrower vessel at u=92.

## v1 — perception probe (seeds 51,53,55,57)

Receipt: results/fs_c2k1clean_spa_bowl_next_to_ramekin_pos_k1_v1b (no motion).
cam_high is fixed on every seed: K = fx=fy=618.04, c=(256,256), 512x512;
t_base_cam = [[0,.6283,-.778,.6586],[1,0,0,0],[0,-.778,-.6283,1.6104]].
Base frame: +y = image right, +x = toward the viewer. table_z = 0.9025 on all
four seeds (z-histogram mode).

Measured object signatures (height above table / near-table footprint / span):
| object     | h      | fp          | span  | mean rgb      |
|------------|--------|-------------|-------|---------------|
| black bowl | 0.0488 | 0.083       | 0.111 | (100,101,98)  |
| ramekin    | 0.0409 | 0.076       | 0.086 | (115,116,117) |
| plate      | 0.0170 | 0.137       | 0.137 | (153,142,139) |
| cookie box | 0.0178 | 0.082x0.061 | 0.082 | (92,65,46)    |
| cabinet    | 0.225  | 0.21        | 0.31  | (67,65,64)    |
| stove      | 0.058  | 0.08        | 0.09  | (26,26,26) at x=-0.39 |

Verdict: the three vessel classes separate cleanly by TOP-Z band, which also
unfuses props that abut in plan view (seeds 51 and 57 fuse clusters).

## v2 — band perception + demo rim-pinch grasp (seeds 51..65 odd)

Receipt: results/fs_c2k1clean_spa_bowl_next_to_ramekin_pos_k1_v2 — **2/8**
(53, 55 succeeded).
Evidence:
- The grasp works: all 8 closed with effort 3.0 and width 0.0060-0.0070.
- FAILURE 1 (plate): the cabinet side wall and the robot base both cross the
  plate height band, so 6/8 seeds "found" the plate at the cabinet (0.14,-0.27)
  or at the arm (-0.17,-0.11) and released there. Exactly the 2 seeds that
  found the real plate are the 2 successes.
- FAILURE 2 (ramekin): seeds 57 and 65 fused the ramekin with a bowl, the
  cluster span exceeded the filter, ram=None, and the WRONG bowl was picked.
Verdict: perception, not manipulation, is the bottleneck.

## v3 — shadow-masked perception, achieved-offset release

Hypothesis: (a) masking the plan-view shadow of everything taller than
table_z+0.07 removes the cabinet/arm/stove from the flat bands; (b) masking a
disc of radius R_BOWL+eps around each detected bowl before clustering the
ramekin band unfuses the ramekin; (c) releasing at plate + (achieved eef - bowl
centre) instead of the commanded offset absorbs the ~0.010 move residual.
Evidence: pending.
Receipt: results/fs_c2k1clean_spa_bowl_next_to_ramekin_pos_k1_v3 — **8/8** on the
probe subset (51,53,55,57,59,61,63,65). Every seed now reads the real plate at
(0.05-0.07, 0.045-0.06) and the ramekin at (-0.21, 0.20); the target is always
the bowl at y≈0.32, the distractor the one at y≈-0.06..-0.12.
Verdict: hypothesis confirmed on all three counts — promoted to the formal run.

Residual margin note (for anyone extending this): the release move leaves a
~0.02 m xy residual, so the bowl centre lands ~2 cm short of the plate centre.
The plate footprint radius is 0.068 and the bowl's 0.042, so the containment
budget is ~0.027 — thin but sufficient on all 15 debug seeds. A repeated
(converging) move at the carry and release poses is the obvious hardening if a
future variant needs more margin; it was NOT applied here because selection is
by the formal run and v3 already scored 15/15.

---

# DECLARATION

- **Frozen version: v3.**
  `packs/c2k1clean_spa_bowl_next_to_ramekin_pos_k1/program.py`
  md5 `9d3bb50719c80cf20b067b34cc2f8344`
  == `program_v3.py` md5 `9d3bb50719c80cf20b067b34cc2f8344`
  == `results/sel_c2k1clean_spa_bowl_next_to_ramekin_pos_k1_v3/program_archived.py`.
- **Selection receipt (full 15 debug seeds 51-65):** **15/15**
  `results/sel_c2k1clean_spa_bowl_next_to_ramekin_pos_k1_v3` —
  seeds 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 all `benchmark_success: true`.
- **Per-version receipt chain:**
  | ver | receipt dir | seeds | score |
  |-----|-------------|-------|-------|
  | v1  | `results/fs_..._v1b` | 51,53,55,57 | perception probe, no motion (0/4 by construction) |
  | v2  | `results/fs_..._v2`  | 51,53,55,57,59,61,63,65 | 2/8 |
  | v3  | `results/fs_..._v3`  | 51,53,55,57,59,61,63,65 | 8/8 |
  | v3  | `results/sel_..._v3` | 51-65 (all 15)          | **15/15** |
- **PROVENANCE:** present as a top-level literal dict in program.py; 16 entries,
  each sourced to a pack field, a debug-seed (51-65) depth/RGB measurement
  logged by the v1/v2 probes, or generic camera/controller mechanics.
- **Clean room:** the only cluster writes were
  `packs/c2k1clean_spa_bowl_next_to_ramekin_pos_k1/*` and
  `results/*c2k1clean_spa_bowl_next_to_ramekin_pos_k1*`; every run went through
  `tools/fair_run.py`; `tools/fewshot_run.py` was never invoked; `api.done` is
  never read; no benchmark asset, no other cell's artifacts, and no
  LIBERO-specific prior knowledge were used.

STOP.
