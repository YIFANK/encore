# c2clean / spa_bowl_next_to_ramekin_task_k3

Intent: **"Pick the akita black bowl next to the cookie box and place it on the plate"**
Runner: `tools/fair_run.py` only. Debug seeds 51-65; probe subset 51,53,...,65.

## Pack reading

The two packs are named against the usual convention for this cell:

| pack | language | relation to intent |
|---|---|---|
| `..._task_k3` | "pick up the black bowl next to the **ramekin** and place it on the plate" | same *target* (plate), **different object** |
| `..._task_mate` | "pick up the black bowl next to the **cookie box** and place it on the plate" | names my object, but its demos are in a scene without the second bowl |

The graded scene is the `..._next_to_the_ramekin...` bddl, which contains **four**
vessels: ramekin, bowl-next-to-ramekin, bowl-next-to-cookie-box, plate. The
instruction sentence is authoritative, so the target is the bowl beside the
cookie box; the k3 pack supplies the *mechanism* (grasp height, release height)
on the other bowl. Open question going in: does the environment's predicate
accept that bowl? **v1 answered yes** (ep59 `benchmark_success: true`).

Useful pack numbers (k3 pack, 3 demos): gripper-close EEF z =
0.9318 / 0.9169 / 0.9215; release EEF z = 0.9450 / 0.9446 / 0.9322, i.e. release
sits ~0.017-0.020 m above the close, matching a 0.020 m plate-top rise.
Closed finger gap at hold = 0.008-0.012 m.

## v0 -- perception probe (results/fs_..._v0, 8 seeds, no motion)

Streamed cam_high RGB + depth out through `api.log` (zlib+base64) and did the
perception offline. Measured, on all 8 probe seeds:

- table top z = **0.900 m** (modal depth over bare tabletop)
- a **z > 0.946 m band is exactly the two bowl rim annuli** — radial histogram
  about the bbox mid peaks at 0.045-0.055 m, interior empty. bbox span
  0.107-0.111 m on every seed, so the bbox mid is an unbiased rim centre and the
  **rim wall midline is 0.050 m**, wall thickness ~0.0095 m.
- rim top z = 0.952 m; bowl height = 0.052 m.
- cookie box: warm-colour (R-B > 0.12) cluster in the 0.908-0.940 m band,
  mid ~(0.075, 0.028), span 0.081 x 0.059, top 0.920 m. Unique on all 8 seeds.
- plate: low-band cluster whose *footprint* max height <= 0.928 m and span
  >= 0.110 m. mid ~(0.058, 0.207), span 0.134 x 0.136, top 0.920 m. Unique on all 8.
- target bowl (nearest the cookie box) mid ~(0.132, -0.068); decoy bowl
  (next to ramekin) mid ~(-0.176, +0.318). Separation 0.4 m -- identification
  is never close.
- (0.18, 0.10) is bare table on every seed.

## v1 -- first acting version (results/fs_..._v1) — **1/8**

Rim straddle with the wrist left at reset (finger axis = world y, verified from
`api.tool_rotation()` col 1 = (0.0005, -1, 0)), EEF at bowl centre + (0, +RIM_R),
descend to rim_top - 0.029 (pack close z), close, carry to the *plate centre*,
descend, release. Fallback attempt B offsets along -x instead.

| | |
|---|---|
| receipt | `results/fs_c2clean_spa_bowl_next_to_ramekin_task_k3_v1` |
| score | **1/8** (ep59) |

Verdict: **grasp solved, place is the failure.**

- `held=True` on all 8 seeds; attempt A (+y straddle) succeeded on 7/8, the
  fallback B was needed only on ep53.
- Episode horizon is **1000 sim steps** (7 seeds hit it).
- Fingertip calibration (valid only on the 2 seeds with steps to spare):
  closed jaws pressed into bare table stop with EEF z = 0.909, so the
  **fingertips sit 0.009 m below the EEF site**.
- Re-perceiving the post-place frame shows the released bowl landing at
  **plate centre + (+0.0149, -0.0282) m** (mean over the 7 A-grasp seeds,
  sd ~0.004/0.007). |d| = 0.032-0.040 failed; the one seed at |d| = 0.017
  scored. So the predicate wants the bowl much closer to centred.
- The place descent was **blocked** on 7/8 (residual 0.015-0.024 vs POS_TOL
  0.012): the bowl bottoms out on the plate with the EEF at z 0.958-0.962 while
  the plate top is 0.920, i.e. the **bowl base hangs 0.040 m below the EEF**.
  The arm then pressed for the rest of the move budget, and the finger gap
  decayed from 0.008-0.017 down to 0.0043-0.0063 (the harness counts gap > 0.005
  as holding). Pressing wastes ~90 steps and squeezes the bite away.

## v2 -- cancel the landing bias, stop short of the plate

Changes from v1:
1. Place command = plate centre **minus** the measured (+0.0149, -0.0282)
   landing bias (sign-swapped if the B grasp fires).
2. Release z = plate_top + 0.040 (hang) + 0.004 (clearance) so the move
   converges instead of pressing; the release becomes a 4 mm drop.
3. Grasp retry: if the closed gap < 0.006 m the straddle caught the rim lip, so
   re-seat 0.008 m deeper before falling back to the -x chord grasp.
4. Dropped the fingertip calibration (answer already obtained) to free ~200 steps.

| | |
|---|---|
| receipt | `results/fs_c2clean_spa_bowl_next_to_ramekin_task_k3_v2` |
| score | **5/8** (51, 53, 57 failed) |

Verdict: right direction, wrong diagnosis of the remaining failures. Re-reading
the post frames, all three failures had an **empty gripper at `pre-release`**
(gap 0.0010-0.0048): the bowl was squeezed out *during the carry*, not misaimed.
ep53 never got a bite at all (0.0013 after all three rungs).

## v3 -- instrument every move, carry lower, compensate the tolerance stop

Changes: log target/achieved/residual for **every** move; carry at 0.985 m
(bowl base 0.945 m, clears the 0.920 m plate and cookie box) instead of 1.02 m;
lower the descent command by 0.011 m to cancel the controller's habit of
breaking as soon as it is inside its own 0.012 m position tolerance; grasp
rung ladder (offset 0.050 / 0.050 deeper 0.008 / 0.045 deeper 0.004); a
post-release reach probe around the plate.

| | |
|---|---|
| probe receipt | `results/fs_c2clean_spa_bowl_next_to_ramekin_task_k3_v3` — **8/8** |
| **selection receipt** | `results/sel_c2clean_spa_bowl_next_to_ramekin_task_k3_v3` — **15/15** (seeds 51-65) |

What the instrumentation showed:

- **The carry move stalls against the arm's reach envelope.** The bias-cancelled
  place target has x = plate_x - 0.015, which lands at 0.033-0.047 on most
  seeds; the EEF cannot be commanded below x ~= 0.056 while stretched to
  y ~= 0.23 at z ~= 0.98, and the stalled move drifts *up* in z (0.985 -> 1.000)
  rather than converging. Re-issuing reproduces the residual to four decimals.
- That stall used to cost ~300 steps, and **every step is another increment of
  gripper closure**: on the thin-bite seeds the bowl is squeezed out over the
  plate and drops onto it. ep53's bite went 0.0048 (v2, lift to 1.02) ->
  0.0144 (v3, lift to 0.985) from the shorter lift alone -- direct evidence for
  the squeeze-lifetime reading.
- Carrying lower + compensating the tolerance stop turned all three v2 failures
  into successes.

## v4 -- bound moves by progress, not by a step cap

Only change of substance: `_reach()` re-issues a short (0.8 s) move while the
residual is still shrinking by >= 0.002 m and gives up otherwise, so a move that
cannot converge costs ~50 steps instead of ~300. The now-answered reach probe
is removed.

| | |
|---|---|
| probe receipt | `results/fs_c2clean_spa_bowl_next_to_ramekin_task_k3_v4` — **8/8** |
| **selection receipt** | `results/sel_c2clean_spa_bowl_next_to_ramekin_task_k3_v4` — **15/15** (seeds 51-65) |

v4 matches v3's 15/15 while spending far fewer sim steps per episode
(median 484 vs 640; max 544 vs 699), i.e. it reaches the same result with more
of the 1000-step horizon still in hand. Frozen on that margin.

## Residual mechanism note (honest, not a blocker)

On the seeds whose bite comes out thin (0.0069-0.0079 m), the bowl is still
squeezed out of the jaws during the carry and *drops* the last ~25-40 mm onto
the plate rather than being set down on it. It lands on the plate on every one
of the 15 debug seeds, but the set-down is a drop, not a placement. The missing
mechanism is a bite that does not decay: the gripper is binary (robosuite takes
only -1/+1), so a hold cannot be commanded at constant force, and every control
step after contact closes the jaws further. A version that wanted to remove this
would have to shorten the carry further still (the reach envelope forbids the
ideal place target, which is what makes the carry move stall in the first place)
or find a grasp whose carry offset points in a reachable direction -- the -y rim
is inside the cabinet footprint and the +x rim is past the arm's x ~= 0.15 limit,
so neither is available with a straight-down wrist.

---

# DECLARATION

- **Frozen version: v4.** `packs/c2clean_spa_bowl_next_to_ramekin_task_k3/program.py`
  md5 `1422b60e0a2f3be1ceb8a2a5107192f3` == `program_v4.py` (verified on the cluster).
- **Selection receipt (full 15 debug seeds, one formal run):**
  `results/sel_c2clean_spa_bowl_next_to_ramekin_task_k3_v4` — **15/15**
  (`benchmark_success: true` on seeds 51-65).
- **Receipt chain:**

  | version | run | seeds | score |
  |---|---|---|---|
  | v0 | `fs_..._v0` | 51,53,…,65 | perception probe, no motion |
  | v1 | `fs_..._v1` | 51,53,…,65 | 1/8 |
  | v2 | `fs_..._v2` | 51,53,…,65 | 5/8 |
  | v3 | `fs_..._v3` | 51,53,…,65 | 8/8 |
  | v3 | `sel_..._v3` | 51-65 | 15/15 |
  | v4 | `fs_..._v4` | 51,53,…,65 | 8/8 |
  | **v4** | **`sel_..._v4`** | **51-65** | **15/15** |

- **PROVENANCE present:** 17 entries, all `allowed: True` with a pack-field or
  debug-seed source. `tools/fair_run.py scan_program(..., "eval")` accepts the
  frozen file (no forbidden tokens, no `api.done` read, PROVENANCE complete).
- Programs were run only under `tools/fair_run.py --split debug` on seeds 51-65.
  Seeds 1-50 were never touched. No `.bddl`/`.xml`/`.hdf5`/init_states file was
  opened; no other cell's artifacts were read.
