# c2clean / obj_chocolate_pudding_pos_k0

Intent: "pick up the chocolate pudding and place it in the basket". No demo pack
(k0). All constants re-derived from debug seeds 51-65 under `tools/fair_run.py`.

## Scene, as measured (debug seeds 51-65, cam_high RGB-D)

Six props plus the basket. Deprojected into the base frame (table surface
deprojects to z = 0.001 +- 0.003, so z = 0 is the table):

| prop (visual) | x | y | z_top |
|---|---|---|---|
| two sauce bottles (fused cluster) | 0.051..0.060 | -0.210 | 0.148 |
| orange-juice carton + blue can (fused) | -0.018 | -0.093 | 0.141 |
| green-cap dressing bottle | 0.164 | 0.028 | 0.148 |
| **small brown box = chocolate pudding** | **-0.149** | **0.059** | **0.030** |
| basket | 0.010..0.059 | 0.245..0.273 | 0.1435 |

Target identification: the pudding is the only brown box in the scene and,
structurally, the only prop whose top is below 0.14 m — a 0.030 vs 0.141 gap, so
"lowest-topped cluster" is a rank with an enormous margin, not a tuned cut. Its
top face is flat to 3 mm and measures 0.080 (x) x 0.048 (y) x 0.030 tall, so the
jaws (0.078 open, closing along base y under R_DOWN) straddle the short axis.

**What `_pos` actually perturbs here:** across all 15 debug seeds the pudding is
bit-identical (x=-0.14904582, y=0.05916089 every seed). Only the **basket**
(x -0.012..0.018, y 0.245..0.273 rim-bbox mid) and one sauce bottle move. The
destination is the perturbed thing, so the basket must be perceived per episode;
the target position was nevertheless left perception-driven rather than
hard-coded, since eval seeds need not share that property.

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | stream RGB-D out through `api.log` (zlib+base64) and do all perception offline, zero sim steps | 15 seeds x 2 cams in ~7 s/ep; gave the table above and the finding that only the basket moves | method kept |
| v2 | perceive + calibrate the fingertip offset on "clear" table at (-0.05, 0.12), then grasp and drop | 0/4. GIF: the arm **carried the basket around**. (-0.05, 0.12) is clear under the eef but the far jaw sits at y = 0.16, which is exactly the basket's near wall; the open jaw hooked it. Grasp descent then stalled at z = 0.029 with a 5 cm lateral deflection (dragging the basket) | refuted: a calibration spot must clear the **jaw span** (~4 cm each side), not just the eef point |
| v3 | diagnostic: which forward spots are reachable, and what is the fingertip offset | x=0.20 presses to eef z=0.009; x=0.26 stalls at x=0.219; x=0.30 never leaves the table. Reach limit x ~ 0.22. Horizon is 1000 steps ~ 48 steps per commanded second, i.e. ~21 s of motion total; v3 exhausted it | reach + budget bounds fixed; TIP_Z0 = 0.009 (agrees with v2's independent 0.0088 at a different xy) |
| v4 | drop the runtime calibration (TIP_Z0 is robot geometry, not scene), straddle-grasp at the top-face bbox mid with tips at 0.012, carry at tips 0.22, release at tips 0.180 over the rim-bbox mid | probe 8/8; **formal 15/15**; 147-150 sim steps/ep; grip width 0.0463 with effort 3.0 held from close through release | **FROZEN** |
| v5 | envelope probe (not a candidate): v4 with the grasp aim displaced (+20, +15) mm | 4/4, width 0.0461 | grasp margin >= 20 x 15 mm |
| v6 | envelope probe (not a candidate): v4 with the release point displaced (+30, +30) mm | 4/4 | drop margin >= 30 x 30 mm |

(The first v5/v6 launch returned 0/4 on a `SyntaxError` from a mangled
docstring, not on behaviour; both were repaired and rerun.)

## DECLARATION

- **Frozen version: v4.** `packs/c2clean_obj_chocolate_pudding_pos_k0/program.py`
  md5 `8eb0e46e8710bef6b66399232a7dd699` == `program_v4.py` (same md5).
- **Selection receipt: 15/15** on the full debug band 51-65,
  `results/sel_c2clean_obj_chocolate_pudding_pos_k0_v4` (every episode
  `"benchmark_success": true`, 147-150 sim steps).
- Receipt chain: v1 perception `results/fs_..._v1`, `..._v1all`; v2 0/4
  `results/fs_..._v2`; v3 diagnostic `results/fs_..._v3`; v4 probe 8/8
  `results/fs_..._v4`; envelope v5 4/4, v6 4/4 `results/fs_..._v5`, `..._v6`.
- PROVENANCE present in `program.py` (13 entries; every constant sourced to a
  debug-seed measurement or to generic controller/gripper mechanics).
- No demonstrations were available or used; no `api.done` read; the only runner
  used was `tools/fair_run.py --split debug`.
