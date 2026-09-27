# c2clean obj_tomato_sauce_task_k0 — worker notes

Intent: "Pick the bbq sauce and place it in the basket". No demo pack (k0).
Runner: tools/fair_run.py only. Debug seeds 51-65; eval seeds 1-50 never touched.

## v1 / v1b — perception dump (0/8, by design)
Hypothesis: all perception can be done offline by shipping RGB-D through `api.log`.
Evidence: works, but `api.log` truncates a message at ~2000 chars — 3000-char
base64 chunks came back clipped and un-decodable. At n=1400 all 8 seeds decoded.
Verdict: log-as-datapipe confirmed; keep chunks <= 1400 chars.

## v2 — deprojected grid (0/8, by design)
Hypothesis: `.depth` + `.t_base_cam` can be turned into base-frame points offline.
Evidence: every convention tried (fwd/inv extrinsics, y-flip) put the table at
z ~ -0.74, not 0. Logging `f.deproject(u,v)` on a stride-4 grid instead gives
table z = 0.001 and a sane workspace. Verdict: use api.deproject, never a
hand-rolled reprojection.

### Scene (cam_high, stride-4 deprojected grid)
| n | ztop | centre (x,y) | red-ratio | identity |
|---|---|---|---|---|
| ~750 | 0.478 | (-0.12, 0.01) | 0.20 | robot arm (parked) |
| ~700 | 0.141 | (0.02, 0.26) | 0.34 | **basket** (goal) |
| ~303 | 0.139 | (0.03, -0.21) | 0.46 | milk + orange-juice cartons, fused |
| ~57 | 0.113 | (-0.19, -0.08) | **0.63** | **bbq sauce bottle** (target) |
| 116 | 0.080 | (0.06, -0.10) | 0.36 | tomato sauce can |
| 72 | 0.029 | (0.16, 0.03) | 0.40 | small box |

**Target identification (load-bearing).** The bddl filename says *tomato sauce*;
the instruction says *bbq sauce*. Across all 8 probed seeds the can, cartons and
boxes are byte-identical (same n, same bbox to 3 dp); only the bottle
(x -0.184..-0.196, y -0.070..-0.091) and the basket (x 0.008..0.036,
y 0.247..0.267) move with the seed. The sampler randomises exactly the target and
the goal, so the graded object is the **bbq sauce bottle named in the
instruction**, not the can named in the bddl path. Confirmed by the 15/15 receipt.
Selector: among clusters with 0.04 < ztop < 0.25 and n < 200, take max red-ratio
(bottle 0.626 vs box 0.400 vs can 0.359 — a 0.22 margin).

## v3 — table-press tip calibration + grasp (0/4)
Hypothesis: pressing the gripper onto empty table stalls at fingertip contact, so
eef z at stall is the fingertip offset.
Evidence: z floored at 0.1006 over a 0.10 m command range, but x drifted
-0.052 -> -0.014 with residual growing to 0.145. The grasp using the implied
offset closed at eef z=0.1904 reading **width 0.001** (fully closed, empty); the
GIF shows the bottle never moving.
Verdict: a repeatable stop is not proof of contact. Offset still unknown.

## v4 — gripper-sensed height sweep (0/4)
Hypothesis: bracket the offset by descending in 0.01 m steps, closing at each,
reading width_m.
Evidence: every close from eef 0.2597 down to 0.1183 read 0.00108 (empty). The
descent floored at 0.1183 and then the arm **jammed** — frozen at that pose for
every later move (res 0.44). At the floor, `grip(0)` left width at 0.078 (fully
open) yet reported **effort 3.0**.
Verdict: (a) `effort` is a *gap* threshold, not a holding signal — it reads 3.0
on an open gripper, so never use it to confirm a grasp; use width_m. (b) Something
stops the descent near z~0.10-0.12.

## v5 — wrist view + press matrix (0/2)
Evidence: closed jaws at (-0.05,0.10) floored at 0.1008; open jaws at the same xy
floored at 0.1005; closed jaws at (0.10,-0.20) floored at 0.1367. Different floors
at different xy -> read at the time as a reach envelope.
The wrist-cam grid at a hover gave the unbiased top-down target: bottle centre
(-0.194,-0.074), ztop 0.112 (agrees with cam_high, so the aim was never wrong),
and resolved the **two finger clusters** at z~0.30 with a 0.078 gap running along
base **y** — so the jaws close along y and the bottle's 0.042 y-width fits.

## v6 — reach-envelope map (0/2) — THE KEY RESULT
Hypothesis: the z-floor is the arm's reach envelope.
Evidence: **refuted.** With a single committed push (hover 0.28 -> 0.16 -> one
move to z=0.02, seconds=4.0) the closed jaws reached z = 0.0270-0.0276 at *every*
x on the free strip (-0.24,-0.20,-0.16,-0.12,-0.08), and z=0.0103 at the bottle's
own xy. The floor is ~0.075 m lower than the "envelope" v3/v5 measured.
Verdict: the 0.10 floor was an **artifact of stepped descent**. Each 0.02 m step
converged inside the controller's own ~12 mm tolerance and returned (res 0.008),
so the arm stopped while reporting success; repeated across many steps this
fabricates a stable, repeatable floor that mimics a kinematic limit. A far target
with a long budget drives through it.
Bonus: the 0.0270-0.0276 floor, identical across five x positions with the table
at z=0.001, is real table contact -> **fingertip offset TIP_DZ = 0.027**.

## v7 — committed-move pick and place — FROZEN
Hypothesis: with TIP_DZ=0.027 and committed (re-issued) moves, a top-down grasp on
the bottle mid-body at h=0.055 (eef z = 0.082) will hold and carry.
Evidence: probe 51,53,...,65 **8/8** (results/fs_c2clean_obj_tomato_sauce_task_k0_v7).
Close read width 0.0362 — the true bottle diameter, not 0.001 and not 0.078.
Selection, all 15 debug seeds: **15/15**
(results/sel_c2clean_obj_tomato_sauce_task_k0_v7), every episode with a real hold
(width 0.0362 on 14 seeds, 0.0275 on seed 58 — an off-centre bite that still
carried). Verdict: accepted and frozen.

---

# DECLARATION

- **Frozen version:** `packs/c2clean_obj_tomato_sauce_task_k0/program.py`
  md5 `aad2a9a9b5bd140fdb32dd5a3ed29181` == `program_v7.py` (verified on cluster).
- **Selection receipt:** **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_obj_tomato_sauce_task_k0_v7` — 15 lines in results.jsonl,
  15 with `"benchmark_success": true`.
- **Per-version receipt chain:**
  | version | episodes | result | what it bought |
  |---|---|---|---|
  | v1/v1b | 51-65 odd | 0/8 | log chunk limit ~2000 chars |
  | v2 | 51-65 odd | 0/8 | api.deproject grid; scene table; target identified |
  | v3 | 51,53,55,57 | 0/4 | stall != contact; empty close receipt |
  | v4 | 51,53,55,57 | 0/4 | effort is a gap threshold, not a hold signal |
  | v5 | 51,53 | 0/2 | wrist-cam bottle centre; jaws close along base y |
  | v6 | 51,53 | 0/2 | stepped descent fabricates a floor; TIP_DZ=0.027 |
  | v7 | 51-65 odd | **8/8** | committed moves + TIP_DZ -> real grasp |
  | v7 | **51-65 all** | **15/15** | selection |
- **PROVENANCE:** present as a top-level literal dict in program.py, covering
  R_DOWN, Z_MIN_OBJ, WS, N_MAX, RED_RANK, TIP_DZ, GRASP_H, CLEAR_Z, CATCH_LO/HI.
  Every constant traces to a debug-seed measurement or generic controller
  mechanics; no pack (k0) and no foreign-campaign source.
- **Clean room:** writes confined to `packs/c2clean_obj_tomato_sauce_task_k0/*`
  and `results/*c2clean_obj_tomato_sauce_task_k0*`. No bddl/xml/hdf5/init_state
  read; no other pack's program.py or NOTES.md read; `--split eval` never invoked;
  `api.done` never read.

STOP.
