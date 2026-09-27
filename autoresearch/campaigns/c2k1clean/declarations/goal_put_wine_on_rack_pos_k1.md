# c2k1clean / goal_put_wine_on_rack_pos_k1

Intent: **put the wine bottle on the rack**. Success = the environment's own
benchmark bit. Runner: `tools/fair_run.py` only. Splits sealed: learn on 51-65,
never touch 1-50.

---

## What the pack says, and what it does not

`packs/c2k1clean_goal_put_wine_on_rack_pos_k1/pack.json`, K=1, 347 steps, 8 keyframes.

* `ee_path6` rotations are **extrinsic XYZ euler**, verified: at t=0 the
  recovered tool z-axis is (0, 0, -1), i.e. straight down.
* The demo approaches the bottle with a **horizontal** wrist (tool z ≈
  (-0.74, -0.67, 0)), closes at t≈85 with EEF z 1.0207 and a finger gap of
  0.0177 m, lifts to z 1.289, and **opens at t=150** at EEF
  (-0.156, -0.269, 1.203) with the bottle tilted ~34° off vertical. Everything
  after t=150 is the operator fumbling with an empty gripper.
* 0.0177 m is the **neck**, not the body: on my own seeds the bottle is 0.041 m
  across below the shoulder and 0.0179 m across over z_top-0.049..z_top-0.019.
  So the transferable mechanism is *pinch the neck, carry, release over the rack*.
* The pack's **absolute xy is a decoy**: its keyframe images show a large dark
  cabinet, a plate and a rack in the far top-left corner — a different scene
  from any debug seed (side-by-side diff of demo t=0 against seed 51 at 128 px:
  mean |Δ| = 27.6). Its release pose (-0.156, -0.269) is meaningless here.
  Only the *relative* grasp height (0.029 below the top) was carried over.

## Scene, re-derived from my own debug-seed RGB-D (nothing imported)

| fact | value | how |
|---|---|---|
| table top | z = 0.901 | mode of cam_high depth over the central crop, all 15 seeds |
| camera | K f=618, c=(256,256); base pos (0.659, 0, 1.610), looking -x and down | `capture()` |
| bottle | 0.1485 m long, body 0.041, neck 0.0179, cap band 0.0140 | dark-green connected component, z-banded |
| bottle top | 1.0494 ± 0.0003 on the 14 standing seeds | same |
| rack | **two planar slat beds**, each dz/dy = -0.575 (rms 2.2 mm), intercepts 0.103 apart | least-squares plane on the wood-coloured points |
| upper bed | x ≈ -0.10..+0.17, y ≈ -0.30..-0.16, top edge z ≈ 1.21 | 2/98 percentiles of the bed points |

The rack reads as a single steep ramp if you only take a top-down max-z map —
that is the *two* beds blending. Collapsing on `z + 0.575*y` splits them into
two clean spikes; the upper spike is the target.

**14/15 debug seeds spawn the bottle standing; seed 64 spawns it lying down**
(top only 0.042 above the table, footprint 0.141 long). That is the one
qualitative split the cell contains, and it needs its own grasp.

## Runtime mechanics (probed, not assumed)

* **Horizon = 1000 controller steps** (probe3b: burn moves at an unreachable
  target, `sim_steps` saturates at 1000). One pick-and-place costs ~150.
* `STEPS_PER_SECOND = 60`, so `move(seconds=s)` costs at most `max(40, 120s)`
  and exits early when inside the controller's own `POS_TOL = 0.012`.
* **`move(rotation=...)` is the trap.** Its convergence test needs the rotation
  *command* under 0.06, i.e. under 1° of error, which the OSC does not hold, so
  a posed move burns its whole cap and tracks ~50 mm wide (probe2: asked for
  x = 0.047, landed at 0.099 — the grasp then closed on air). With
  `rotation=None` the same moves land within 0.011. The program therefore sets
  the wrist **at most once, high and empty-handed**, and passes `rotation=None`
  everywhere else. probe7 confirms a posed move at a comfortable pose is exact
  (0.5° error for three different yaws) and that `rotation=None` tracking
  survives it.
* **`goto(tol=...)` must stay above `POS_TOL`.** v2 used 0.008, below what the
  controller can report, so every refine burned its full 60 steps for nothing
  and seed 64 ran out of horizon mid-carry.
* Fingertips sit **0.008 below the EEF** (probe2: an open gripper pressed onto
  bare table stops with the EEF at 0.9092).
* `gripper()` reports `effort 3.0` iff the commanded close left a gap > 5 mm; a
  neck pinch reads 0.0148, an empty close 0.0018.

## The predicate is not a region test

probe5 held the bottle at z = 1.34 over a 4x3 raster spanning the whole rack
(12 waypoints, all reached) — **no success anywhere**. probe6 pressed the
hanging bottle down onto the rack at four points: seed 57 fired the instant the
descent stalled with the bottle's base at 1.159, on the upper bed; seeds 51 and
63 made contact at other spots and did not. So merely being above the rack is
not enough and a poke is not reliable either — the bottle has to actually come
to rest on the upper bed. Once it does, LIBERO terminates the episode
immediately (every successful run's `sim_steps` sits at the release).

---

## Version log

| ver | change | receipt |
|---|---|---|
| probe1 | dump cam_high RGB-D through `api.log` (zlib+base64, chunked at 1800 chars) for offline perception | frames for all 15 debug seeds |
| probe2 | tip calibration + top-down neck grasp with `rotation=R_DOWN` | grasp missed 4/4; exposed the posed-move tracking error and the 0.008 tip offset |
| probe3/3b | step accounting | horizon = 1000 |
| probe4 | same grasp with `rotation=None` | grasp holds (effort 3.0) 2/2; seed 51 fired en route to the rack |
| probe5 | 12-point raster at z=1.34 over the rack, bottle held | 0/3 — not a region predicate |
| probe6 | press the hanging bottle onto the rack at 4 points | 1/3 — contact alone is not enough either |
| **v1** | perceive bottle + upper bed, neck pinch, release 0.004 above the bed plane | probe 8/8; **selection 14/15** (`sel_..._v1`), seed 64 failed: the lying bottle is not a standing one |
| **v2** | lying-bottle branch (principal axis, grasp across the widest band, optional wrist yaw), grasp/placement verification, 3 attempts | probe 3/4 — seed 64 grasped on attempt 1 but the horizon was gone (the sub-`POS_TOL` refine bug) |
| **v3** | `tol=0.014`, explicit step budget, keep hold of the bottle across a failed placement | probe 4/4; **selection 15/15** (`sel_..._v3`), seed 64 at 901/1000 steps |
| **v4** | strict green dominance (`G>R and G>B`: the grey fingers passed the `>=` test), transit height computed from the perceived bed, loose tolerances on transit moves | **selection 15/15** (`sel_..._v4`), seed 64 down to 718 steps |
| **v5** | cancel the descent's +9 mm x tracking bias with one bounded re-command | **selection 15/15** (`sel_..._v5`); envelope becomes symmetric |
| **v6** | fallback neck bands + finite guards when the neck band is occluded (never fires on debug seeds) | **selection 15/15** (`sel_..._v6`) — FROZEN |

### Aim envelope (the thing 15/15 does not tell you)

Single-attempt variants, seeds 51/55/59/63.

*Placement* (v3, target displaced): dy +40 mm 4/4, dy -40 mm 4/4, dx +70 mm
4/4, dx -70 mm 4/4. The placement is not the fragile part.

*Grasp* (perceived bottle pose displaced):

| offset | v4 (no bias cancel) | v5/v6 |
|---|---|---|
| x +15 mm | **0/4** | 4/4 |
| x -15 mm | 4/4 | — |
| x ±22 mm | — | 0/4 both signs |
| y +15 / +22 mm | 4/4 | 4/4 |
| z -15 mm | **0/4** | 4/4 |
| z -22 mm | — | 4/4 |

v4's envelope was off-centre by exactly the +9 mm the descent overshoots: the
finger plates are only ~20 mm wide, so that bias ate the whole margin on one
side. v5's re-command re-centres it (seed 51: 0.0556 → 0.0505 against a target
of 0.0471) and the envelope becomes ±15 mm symmetric in x, ≥22 mm in y and z.
Perception repeatability across the 15 seeds is ~1 mm, so the margin is ~15x.

---

## DECLARATION

* **Frozen version: `program_v6.py`.**
  `md5(program.py) == md5(program_v6.py) == ef4da42d0d8bf94040fc8763c6f36ee0`
  (verified on the cluster copy in
  `packs/c2k1clean_goal_put_wine_on_rack_pos_k1/`).
* **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, directory
  `results/sel_c2k1clean_goal_put_wine_on_rack_pos_k1_v6`, per-seed
  `benchmark_success` true for 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65
  (`sim_steps` 146-251 for the standing seeds, 729 for the lying seed 64,
  against a 1000-step horizon).
* Per-version receipt chain: v1 14/15 (`sel_..._v1`), v3 15/15 (`sel_..._v3`),
  v4 15/15 (`sel_..._v4`), v5 15/15 (`sel_..._v5`), v6 15/15 (`sel_..._v6`);
  probe receipts in the version log above.
* `PROVENANCE` is a top-level literal dict in `program.py` covering every
  calibrated constant, each sourced to this pack or to a debug-seed measurement.
* Seeds 1-50 were never run, and `tools/fewshot_run.py` was never invoked.
