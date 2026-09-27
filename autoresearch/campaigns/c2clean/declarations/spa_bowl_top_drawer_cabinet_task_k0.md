# c2clean — spa_bowl_top_drawer_cabinet_task_k0

Intent served by `api.instruction()`:
> Pick the akita black bowl on the top of the wooden cabinet and place it on the plate

No demonstration pack (k0). Everything below is measured from debug seeds
51–65 under `tools/fair_run.py --split debug`. No shared note file;
FAIR_PROTOCOL v1.1.1 governs.

## Scene (measured, v1 perception probe, seeds 51/53/57/61)

| quantity | value | how |
|---|---|---|
| table top z | 0.9012 | modal height of the cam_high height map, 0.86–0.96 |
| cabinet top slab z | 1.1272 | modal height, table+0.17 … table+0.30 |
| target bowl: rim top | 1.1799 | max of the rim ring |
| target bowl: interior floor | 1.1356–1.1361 | median height inside r/2 |
| target bowl: rim ring radius (Kasa) | 0.0513–0.0521 | circle fit, residual σ 0.0034–0.0038 |
| target bowl centre | (+0.003,−0.262) … (+0.042,−0.297) | Kasa, per seed (≈3 cm of jitter) |
| second bowl, down in the open drawer | rim top 1.116 | present in every seed; NOT the named target |
| plate | top 0.908, r 0.057, centre (0.046…0.069, 0.191…0.205) | light flat disc, table+0.008…+0.038 band |
| fingertip offset below the eef | 0.0103 | open gripper pressed into the bare slab, seeds 57/61 |
| episode horizon | 1000 sim steps | v4f clipped there exactly |

Cameras: `cam_high` at (0.659, 0, 1.610), looking toward −x and down. The
gripper's closing axis at the resting wrist is base **y**.

Two bowls are in the scene and they are the same object. The intent names the
one standing *on the cabinet top*; that is the one the program takes, and the
benchmark bit agrees (v2 onward score on it).

## Mechanism

The bowl's inner rim diameter (~0.104 m) is wider than the gripper's full
opening (0.0778 m), so a grasp centred on the bowl puts *both* fingers inside
it and closes on nothing. The grasp is a **one-wall straddle** instead: put the
eef at `bowl_centre + (0, −(r+0.002))` — one rim radius along the closing axis
— so the near finger descends *inside* the bowl and the far finger *outside*
it, then close. The fingertips stop 4 mm above the interior floor, deep enough
that the clamped cross-section of the shell is ~8 mm rather than the ~7 mm rim
lip; the closed gap is the receipt.

Carry geometry follows from the grasp: the bowl base rested on the slab at
closing time, so it rides `grasp_eef_z − slab_z` (≈0.023 m) below the eef for
the whole transfer, and the bowl centre trails the eef by the straddle offset.

## Version log (hypothesis → evidence → verdict)

### v1 — perception probe, no motion
*H*: the scene must be measured before anything is designed.
*E*: `fs_…_v1`, seeds 51/53/57/61, 0/4 (no motion attempted). RGB-D dumped
through `api.log` (zlib+base64, chunked under the 2000-char cap) and the point
cloud rebuilt offline.
*V*: a `_task` cell — layout fixed to ~3 cm of per-seed jitter. Exactly one
bowl candidate and one plate candidate survive the band+shape filters on every
seed, so no disambiguation logic is needed.

### v2 — straddle grasp + place, in-episode fingertip calibration
*H*: the one-wall straddle grips the bowl and a straight carry satisfies the
predicate.
*E*: `fs_…_v2`, seeds 51/53/57/61, **4/4**.
*V*: mechanism confirmed on the first motion attempt. Two weaknesses in the
receipts: the in-episode slab calibration returned `PROBE_XY None` on 2 of 4
seeds (its clear-patch search was too strict), so those ran 10 mm shallow —
seed 51's gap decayed 0.0082 → 0.0043 and the harness's `effort` flag read
*not holding*, yet the GIF shows the bowl carried and the episode scored (that
flag is a gap threshold, not a force reading). And the single-stage place
descent was dragged laterally by contact: seed 61 landed 0.028 m off in x.

### v3 — hardcoded tip offset, staged place, held-bowl re-perception (logged)
*H*: the measured constant 0.0103 removes the shallow-grip mode.
*E*: `fs_…_v3`, 8 probe seeds, **8/8**.
*V*: grip still marginal — closed gap 0.0046–0.0096, decaying to 0.0033 on 5
of 8 during the lift (the bite creeping up to the thin rim lip). The logged
re-perception of the *held* bowl was unreliable (the ring is clipped by the
gripper) and was dropped; the geometric carry model is used instead.

### v4 — 4 mm grasp depth, repeat-to-converge aiming, thin-gap retry
*H*: closing 6 mm deeper clamps a thicker cross-section.
*E*: `fs_…_v4` 8/8; **`sel_…_v4` 15/15** on the full debug band, ≤875 steps.
*V*: closed gap 0.0071–0.0088 on every seed and the bowl held through the lift
on all 8 — the grip risk is gone. Place aim unchanged (|dx| mean 0.016, max
0.028): re-issuing the command does not recover it.

### v4e / v4f — aim-envelope probes (not candidates)
*H*: how much straddle-aim error does the mechanism tolerate?
*E*: `fs_…_v4e` (offset **+8 mm**) **6/8**, fails 61/63; `fs_…_v4f` (offset
**−8 mm**) **6/8**, fails 53/55.
*V*: the window is ≈ ±8 mm and v4's operating point sits centred in it, so
there is nothing to gain by re-centring. Both failures are at the *place*, not
the grasp: the grip was sound (gap 0.0092/0.0099, held through the carry) but
the bowl caught the plate rim and the release sprang the arm 0.03 m. The
place aim, not the grasp, is the binding constraint. v4f also clipped at
exactly 1000 sim steps, which is how the horizon is known.

### v5 — cheaper place staging
*E*: `fs_…_v5` 8/8, ≤804 steps. *V*: no measured advantage over v4; dropped.

### v6 — place descent split into six short legs
*H*: the place-x error is drift accumulated under a saturated command.
*E*: `fs_…_v6` 8/8, ≤826 steps; |dx| mean 0.0164 → 0.0153, **max unchanged at
0.028**, per-seed values nearly identical to v4's.
*V*: hypothesis refuted. The offset is a *standing* tracking bias, not
accumulation — splitting the descent buys nothing.

### v7 — cancel the bias in the command
*H*: if it is a standing bias, measuring it at a hover and shifting the command
by it will halve it.
*E*: `fs_…_v7` 8/8; |dx| mean 0.0164 → 0.0077, max 0.028 → 0.013.
*V*: confirmed, and the residual halves exactly as a one-shot proportional
correction predicts. But peak cost rose to 944 steps against a 1000 horizon —
too little headroom to freeze.

### v8 — v7 trimmed to fit the horizon  ← **FROZEN**
*H*: the bias cancel can be kept if the move budget is trimmed where v6 showed
the extra legs earn nothing.
*E*: `fs_…_v8` 8/8 (≤854 steps); **`sel_…_v8` 15/15**, peak 854 steps.
*V*: dominates v4 — same score, better centred (|dx| mean 0.0083, max 0.0134
on a plate of radius 0.057), and 21 steps cheaper. Frozen.

## v8 full-band selection receipts (`sel_…_v8`, seeds 51–65, 15/15)

| seed | closed gap | place dx | place dy | release eef z |
|---|---|---|---|---|
| 51 | 0.0084 | +0.0085 | +0.0010 | 0.9668 |
| 52 | 0.0071 | +0.0126 | +0.0024 | 0.9708 |
| 53 | 0.0073 | +0.0084 | +0.0014 | 0.9723 |
| 54 | 0.0087 | +0.0041 | +0.0006 | 0.9537 |
| 55 | 0.0071 | +0.0106 | +0.0019 | 0.9715 |
| 56 | 0.0098 | +0.0031 | +0.0007 | 0.9587 |
| 57 | 0.0088 | +0.0019 | +0.0004 | 0.9580 |
| 58 | 0.0067 | +0.0130 | +0.0019 | 0.9720 |
| 59 | 0.0074 | +0.0088 | +0.0011 | 0.9674 |
| 60 | 0.0086 | +0.0061 | +0.0009 | 0.9522 |
| 61 | 0.0086 | +0.0134 | +0.0026 | 0.9679 |
| 62 | 0.0074 | +0.0113 | +0.0018 | 0.9684 |
| 63 | 0.0087 | +0.0046 | +0.0008 | 0.9596 |
| 64 | 0.0064 | +0.0127 | +0.0018 | 0.9710 |
| 65 | 0.0079 | +0.0049 | +0.0011 | 0.9702 |

Every seed closed on the bowl wall (gap 0.0064–0.0098; an empty close reads
<0.004) and released within 0.0134 m of the plate centre.

## Known limitation (documented, not a blocker)

The arm cannot reach the commanded release height: the release happens with the
eef at 0.952–0.972 against a 0.937 command, so the bowl base is let go
1.5–3.5 cm above the plate and drops. It lands and scores on all 15 seeds, and
the bias cancel keeps it centred, but a plate sitting further forward than any
debug seed would both worsen the drop and push the aim toward the ±8 mm window
edge measured in v4e/v4f.

---

# DECLARATION

- **Frozen version**: `program.py` == `program_v8.py`,
  md5 `510c11981502023f3456faae72caf6fa` (verified identical in
  `packs/c2clean_spa_bowl_top_drawer_cabinet_task_k0/`).
- **Selection receipt**: **15/15** on the full 15 debug seeds (51–65),
  `results/sel_c2clean_spa_bowl_top_drawer_cabinet_task_k0_v8`.
- **Per-version receipt chain**:
  | version | run dir | seeds | score |
  |---|---|---|---|
  | v1 | `fs_…_v1` | 51,53,57,61 | 0/4 (perception only, no motion) |
  | v2 | `fs_…_v2` | 51,53,57,61 | 4/4 |
  | v3 | `fs_…_v3` | 51,53,…,65 | 8/8 |
  | v4 | `fs_…_v4` | 51,53,…,65 | 8/8 |
  | v4 | `sel_…_v4` | 51–65 | 15/15 |
  | v4e | `fs_…_v4e` | 51,53,…,65 | 6/8 (envelope probe, +8 mm aim) |
  | v4f | `fs_…_v4f` | 51,53,…,65 | 6/8 (envelope probe, −8 mm aim) |
  | v5 | `fs_…_v5` | 51,53,…,65 | 8/8 |
  | v6 | `fs_…_v6` | 51,53,…,65 | 8/8 |
  | v7 | `fs_…_v7` | 51,53,…,65 | 8/8 |
  | v8 | `fs_…_v8` | 51,53,…,65 | 8/8 |
  | **v8** | **`sel_…_v8`** | **51–65** | **15/15** |
- **PROVENANCE**: present in `program.py` as a top-level literal dict, every
  entry `allowed: True`, covering CELL/bounds, BAND_BOWL, BAND_PLATE,
  GRIP_DEPTH, RIM_BIAS, TIP_OFFSET, CARRY_Z, THIN_GAP, PLACE_LEGS, BIAS_CLIP
  and PLACE_CLEAR. Every source is either a debug-seed (51–65) observation of
  this cell or generic camera/controller mechanics. No demo pack exists for
  this cell and none was used; no constant came from any other campaign.
- **Clean room**: no `.bddl`/`.xml`/`.urdf`/`.hdf5`/init-state/gt-trace file
  was opened, no `program*.py`/`NOTES.md`/`_stock_copies/` under any pack was
  read, no other campaign's artifacts were touched, and no seed outside 51–65
  was run or inspected. All cluster writes are under
  `packs/c2clean_spa_bowl_top_drawer_cabinet_task_k0/*` and
  `results/*c2clean_spa_bowl_top_drawer_cabinet_task_k0*`.
- **Eval**: not run by this worker (coordinator-run, blind, seeds 1–50).

STOP.
