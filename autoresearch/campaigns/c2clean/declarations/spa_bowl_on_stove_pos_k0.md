# c2clean / spa_bowl_on_stove_pos_k0 — worker notes

Intent: **"pick up the black bowl on the stove and place it on the plate"**
Runner: `tools/fair_run.py` only. No demonstration pack (k0). No shared note file.
Debug split 51–65; eval seeds 1–50 never touched.

---

## Scene, as re-derived from debug-seed RGB-D (nothing imported)

`cam_high` at base `(0.659, 0, 1.610)`, looking down the −x axis; `cam_arm_wrist`
on the tool. Deprojecting `cam_high` and taking a 5 mm top-down max-z map gives:

| thing | where (ep51) | top z |
|---|---|---|
| table plane (modal z) | — | **0.901** |
| plate (light grey, 0.134 m across) | (0.065, 0.028) | 0.920 |
| cookie box (brown, 0.080×0.065) | (0.065, 0.192) | 0.921 |
| stove slab (+ black back panel, burner disc) | x∈[-0.35,-0.16] | 0.927 |
| **bowl on the stove** (target) | (-0.251, -0.137) | **0.980** |
| bowl on the table | (-0.197, 0.210) | 0.944 |
| bowl on the cabinet top | (0.03, -0.24) | 1.180 |

Radial profile of the target bowl about its own centre (ep51):

```
r 0.010 z 0.935   <- interior floor, only 8 mm above the slab
r 0.030 z 0.939
r 0.040 z 0.952
r 0.045 z 0.964
r 0.050 z 0.973
r 0.055 z 0.980   <- rim top;  beyond this the slab (0.930)
```

So: outer rim diameter **0.107 m**, bowl height **0.053 m**, interior 0.045 m deep,
rim wall ~5 mm thick. The gripper's full opening is 0.0778 m, i.e. **the bowl cannot
be straddled** — it has to be pinched across the rim wall.

Jaw axis: the wrist camera's extrinsics put wrist-image +x along base −y, and the two
finger blobs in the wrist image are separated along wrist-image x. **The jaws close
along base ±y** with the tool straight down, so the pinch point is `(cx, cy + r)`.

Three-bowl disambiguation: inside the band `[table+0.033, table+0.130]` and the window
`x ∈ [-0.35, 0.22]` (which masks the robot base) only two bowls survive, and **the one
on the stove has the higher top** (0.980 vs 0.944) because the stove slab lifts it. The
cabinet bowl is above the band. So "max z_top in the band" names the target, with no
colour cue needed. Plate vs cookie box: the band `[table+0.010, table+0.030]` holds
both; the plate is the one whose footprint is ≥ 0.095 m across.

---

## Version chain

### v1 — perception dump (probe, seeds 51,53,55 then 51–57)
Hypothesis: nothing is known about the scene; dump it.
Mechanism: zlib+base64 RGB-D through `api.log`, perception done offline, zero sim steps.
Evidence: `api.log` truncates a message at ~2000 chars — the first run's base64 chunks
(3000 chars) came back clipped and undecodable. Re-run at 1500 chars decoded cleanly.
Verdict: gave the whole table above. No motion, no score.

### v2 — first full pick-and-place — **4/8** (`results/fs_…_v2`)
Hypothesis: rim-pinch at `cy + r_fit − 0.003`, fingertips 0.014 below the rim top;
measure the fingertip-to-eef offset in-episode by pressing the open gripper into bare
table.
Evidence:
* fingertip offset came back **0.0071 on all 8 seeds** — a robot constant, not a
  per-episode unknown.
* ring circle-fit gave **r = 0.0527 ± 0.0003** on every seed. A first version of the
  fit inflated the ring to 0.135 m because it gathered the *robot arm* (z≈1.1) within
  the search radius; gating the gather to `[z_top−0.010, z_top+0.006]` fixed it. The
  fit (rather than a bbox midpoint) matters on seeds where the cabinet corner splits
  the ring into two components (ep57: bbox centre off by 8 mm, fit correct).
* **The failure was mechanical and perfectly separable.** Post-lift gripper width is
  bimodal: `0.0017` (bowl gone) on 51/53/61/63, `0.0033–0.0037` (bowl held) on
  55/57/59/65 — exactly the success split. The close gap split the same way
  (fail ≤ 0.0120, ok ≥ 0.0126).
* Root cause: the 0.135 m descent leg is time-budgeted and **stopped ~8 mm high**, so
  the fingertips actually bit 0.007 below the rim top instead of 0.014 — the very
  thinnest part of a flaring wall, which then ratcheted out of the closing jaws.
* Also learned: `effort` reads 0.05 after the lift **even while holding**. The honest
  hold receipt on this cell is the **closed gap surviving the lift**, not effort.
Verdict: mechanism right, depth wrong.

### v3 — deeper, converged bite — **8/8 probe, 15/15 formal** (`results/sel_…_v3`)
Changes, each tied to a v2 receipt:
* hover only `z_top + 0.05` instead of `+0.10`, so the descent leg is short enough to
  land where commanded;
* `GRASP_DEPTH` 0.014 → **0.025**, and the aim radius `r_fit − 0.008` (= 0.045) to
  follow the flare — the wall mid-radius at 25 mm down, not at the rim top;
* `FINGER_OFFSET` frozen at 0.007 (measured 8/8 in v2), freeing three moves of horizon;
* every critical leg re-issued until the pose is held;
* a regrasp branch if the post-lift width falls under `HOLD_W = 0.0025`.
Evidence: **15/15**, and the margin is no longer marginal —
post-lift hold width **0.0047–0.0048 on all 15 seeds** (v2's successes were 0.0033 and
its failures 0.0017), and the bowl centre lands **≤ 0.004 m** from the plate centre
against a `(0.0675 − 0.0535) = 0.014 m` allowance. The regrasp branch never fired.
Verdict: **selected.**

### v4 — carry-bias cancellation — rejected
Hypothesis: the carry leg shows a steady-state x offset of −0.007…−0.012 that
re-issuing the same target never removes, so add the residual back into the command.
Evidence: **the premise was wrong.** The offset is on the *carry* leg, but the
subsequent short *drop* leg converges it out on its own — v3's measured
`drop_x − plate_x` is +0.000…+0.004 m on all 15 seeds. Adding a +0.010 correction
would push the release off-centre in the opposite direction.
Formal receipt: v4 also scored **15/15** (`results/sel_…_v4`), so the score ties — but
the margin does not. Measured `drop_x − plate_x` over the 15 seeds:

| version | mean | max abs | range |
|---|---|---|---|
| **v3** | +0.0017 | **0.0040** | [−0.001, +0.004] |
| v4 | +0.0093 | 0.0170 | [−0.003, +0.017] |

v4's worst seed releases the bowl 17 mm off-centre, past the 14 mm allowance — it
passed anyway, which says the predicate is more forgiving than the pure geometry, but
it is strictly the less robust of the two.
Verdict: **not adopted**; v3 kept on the margin receipt.

---

## Things that would have cost a rerun

* `api.log` truncates near 2000 characters — chunk a base64 payload at ≤1500.
* A ring gather for a circle fit must be gated **above and below**, or the robot arm
  hovering over the scene joins the ring.
* `gripper()["effort"]` is 3.0 only while the jaws are actively squeezing; once the
  motion settles it reads 0.05 whether or not anything is held. Use the surviving gap.
* A single `api.move` is time-budgeted: a 0.135 m leg lands ~8 mm short and re-issuing
  the *same* target does not fix it (the +0.009 z offset is steady-state, not
  starvation). Shorten the leg instead.

---

## DECLARATION

* **Frozen version: v3.** `packs/c2clean_spa_bowl_on_stove_pos_k0/program.py`
  == `program_v3.py`, md5 **245909be906433b785bf0176e191619d** (verified equal on the
  cluster).
* **Selection receipt: 15/15** on the full debug split 51–65,
  `results/sel_c2clean_spa_bowl_on_stove_pos_k0_v3` — episodes
  51,52,53,54,55,56,57,58,59,60,61,62,63,64,65 all `"benchmark_success": true`.
* Per-version receipt chain:
  * v1 `results/fs_…_v1`, `_v1b` — perception dump, no motion, no score.
  * v2 `results/fs_…_v2` — **4/8** on 51,53,55,57,59,61,63,65.
  * v3 `results/fs_…_v3` — **8/8** on the same probe; `results/sel_…_v3` — **15/15**.
  * v4 `results/sel_…_v4` — **15/15**, tied on score, rejected on the margin receipt
    (drop offset up to 0.017 m vs v3's 0.004 m).
* `PROVENANCE` is present in program.py and covers every calibrated constant; every
  source is a debug-seed (51–65) measurement or generic controller/camera mechanics.
  No pack was supplied (k0) and none was read.
