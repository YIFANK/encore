# c2clean — spa_bowl_cookie_box_pos_k0

Intent: *pick up the black bowl next to the cookie box and place it on the plate*
Zero demonstrations. Every constant re-derived from debug seeds 51-65 only.

## Scene, as measured (v2 probe, cam_high RGB-D dumped through `api.log`)

`api.log` truncates a message at 2000 chars (v1 → v2 fix: chunk base64 at 1900),
so a whole 512×512 RGB-D frame can be shipped off-box and analysed offline at
zero sim-step cost.

Deprojection: pixel → camera → base with `f.intrinsics` / `f.t_base_cam`, no
vertical flip (that convention gives a 120k-pixel modal plane at **z = 0.901**,
the table; the flipped one gives noise).

| object | how it is found | where (seed 51) |
|---|---|---|
| table | modal plane of the cloud | z 0.901 |
| cookie box | warm mask `r-b>0.15, r>0.25`, z 0.910–0.935 | (0.085, 0.042), top 0.920 |
| plate | flat component (thr 0.912, top < 0.935), non-warm | (-0.193, 0.199), top 0.920 |
| bowls | components above z 0.930 | big (0.125,-0.070) top 0.952; small (0.062,0.201) top 0.944 |

**Target selection.** Two bowls flank the cookie box. Centre-to-centre distance
nearly ties on seed 57 (0.144 vs 0.145); **bbox edge-gap** separates them cleanly
on all eight probed seeds (0.004–0.041 vs 0.074–0.121, worst margin 1.8×),
because the two bowls are different sizes. Target = smallest edge gap to the box.

**Rim fit.** Kasa fit on the target component's own top 7 mm band, clipped to the
component bbox **and** to `ztop + 0.004`. Seeds 51-58: centre varies with the
_pos perturbation, r = 0.0533 ± 0.0001, rstd 0.0015, ztop 0.9519 every seed.
(v5 seed 55 failed because the bbox was expanded by 20 mm and swallowed the
cabinet top at z 1.128 — the cabinet sits ~4 mm from the bowl bbox on some seeds.)

## Mechanism facts measured on debug seeds

* **Jaw closing axis = base y** under the straight-down wrist (v3: diff of
  `cam_arm_wrist` RGB open vs closed moves along image *u*, and that camera's
  x-axis is base −y).
* **Fingertip is 0.0077 m below the eef reference** (v4: closed gripper pressed
  onto the table stalls at eef z 0.9087; table 0.901). Identical on 3 seeds.
* **`api.move` has a ~1 cm steady-state bias** that repeating the same command
  does not remove (v3/v4: commanded z 0.930 → 0.9405, 0.910 → 0.9204). Feeding
  the residual back into the command (`cmd += target − eef`) converges to
  ≤ 0.003 in two corrections. This is the single biggest accuracy win.
* **Horizon = 1000 sim steps and `api.move(seconds=s)` costs 20·s of them**
  (v5a: 5 moves @2.0 s + 2 grips + settle(1.0) = 214 steps). v4 spent the entire
  horizon on a calibration phase and every sensor reading after it was garbage.
  All later versions carry an explicit step budget.
* **The bite decays with every post-close move** (v7 sweep: 0.00704 → 0.00628
  over three lift stages). Deeper pinch ⇒ thicker bite (seed 57 landed 2 mm
  lower and bit 0.0093 — the only v6 carry that held). Hence PINCH_DEPTH 0.024
  and as few moves as possible after closing.

## The blocker, and what it actually was

v6/v8/v10 all grasped 8/8 and then lost the bowl in the *carry*, the eef parking
at x ≈ 0.05 with a 0.25 residual. Three probes to pin it down:

* **v9p (empty gripper)** — four repeats of the same long command leave the eef
  bit-identical at (0.035, 0.273, 1.137). Not time starvation: saturated.
* **v9p2 (waypoints)** — stepping along the path in quarters still stalls at
  x ≈ 0.06. Not command runaway either.
* **v10p / v10p2 (escape menu from the post-pick pose)** — with the straight-down
  wrist the arm **cannot retract past x ≈ 0.05–0.10 at any height** (1.17, 1.05,
  0.99 all fail) and in any y, while re-posing *forward* to the pick pose
  succeeds every time. The forward reach to the bowl leaves the arm in a
  configuration it cannot come back from.

v9p3 measured the envelope with the wrist **yawed 90°** (jaw axis along base x)
and found the same workspace wide open: the −x rim pose (0.075,−0.070,0.936)
res 0.0024 and the −x plate pose (−0.246,0.199,1.00) res 0.0047.

So the fix is not a taller carry or a smarter path — it is the wrist. v11 runs
the whole task under `R_YAW`, grips the **−x** side of the rim (bowl then trails
at +x, keeping the plate pose at x −0.246 rather than y +0.26), and the carry
becomes unremarkable.

## Version chain

| v | change | receipt |
|---|---|---|
| v1 | RGB-D dump probe | log truncates at 2000 chars — no usable frame |
| v2 | chunk at 1900 | 8 seeds of cam_high RGB-D decoded; scene + target rule validated offline |
| v3 | jaw-axis + move-accuracy probe | jaw axis = base y; moves leave ~1 cm bias |
| v4 | calibration + pinch + lift | tip offset 0.0077; bite 0.0067; held 1/3 — and burned all 1000 steps |
| v5a | step-cost probe | 5 moves @2.0 s + grips + settle = 214 steps |
| v5 | closed-loop positioner, budgeted | 1/4 (57). Aim res 0.002. Rim fit broke on 55 (cabinet leak) |
| v6 | robust rim fit, carry 1.10 | 1/8. Carry stalls at x≈0.05 |
| v7 | pinch-depth sweep | bite decays per move; deeper is thicker |
| v8 | depth 0.024, fewer post-close moves | 1/8. Grasp now survives the lift 8/8; carry still stalls |
| v9p/v9p2/v9p3 | carry diagnostics + reach envelope | stall is a configuration lock, not collision; yawed wrist is free |
| v10 | neutral waypoint | 1/8 — the arm cannot even reach the neutral waypoint |
| v10p/v10p2 | escape menu | no retraction at any height with the straight-down wrist |
| **v11** | **yawed wrist end-to-end, −x rim grip** | **8/8 on 51-58** |

## DECLARATION

**Frozen version: v11.**
`packs/c2clean_spa_bowl_cookie_box_pos_k0/program.py`
md5 `b24a33e144b8b19d304dd0b0474a3b12` == `program_v11.py` (verified on the cluster).

**Selection receipt: 15 / 15** on the full debug split (seeds 51-65), one formal run:
`results/sel_c2clean_spa_bowl_cookie_box_pos_k0_v11` — every episode
`"benchmark_success": true`.

Per-version receipt chain (all on seeds drawn from 51-65 only; evaluation seeds
1-50 were never run and `--split eval` was never invoked):

| v | result dir | receipt |
|---|---|---|
| v1 | fs_..._v1 | probe, log truncation found |
| v2 | fs_..._v2 | probe, 8 seeds of RGB-D; offline target rule validated |
| v3 | fs_..._v3 | probe, jaw axis + move bias |
| v4 | fs_..._v4 | 0/3, horizon exhausted by calibration |
| v5a | fs_..._v5a | probe, 214 sim_steps → 20 steps per move-second |
| v5 | fs_..._v5 | 1/4 (57) |
| v6 | fs_..._v6 | 1/8 (57) |
| v7 | fs_..._v7 | probe, pinch-depth sweep |
| v8 | fs_..._v8 | 1/8 (54); grasp survives the lift 8/8 |
| v9p, v9p2, v9p3 | fs_..._v9p* | probes, carry diagnosis + reach envelope |
| v10 | fs_..._v10 | 1/8 (54) |
| v10p, v10p2 | fs_..._v10p* | probes, escape menu |
| **v11** | fs_..._v11 / **sel_..._v11** | **8/8 probe, 15/15 formal** |

PROVENANCE present in the frozen program: 11 calibrated constants, every source a
debug-seed measurement or generic controller/camera mechanics.

No `api.done` read anywhere in the program (verified by string check on the
frozen file). No demonstration pack was available or used; no forbidden path was
opened; all cluster writes went to
`packs/c2clean_spa_bowl_cookie_box_pos_k0/*` and `results/*c2clean_spa_bowl_cookie_box_pos_k0*`.

STOP.
