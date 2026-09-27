# c2clean / spa_bowl_between_pos_k0 — working notes

Intent: *pick up the black bowl between the plate and the ramekin and place it on the plate*
Zero demonstrations (k0). Debug band = seeds 51–65. Runner = `tools/fair_run.py` only.

## Scene, as measured from debug seeds (v1 probe, seeds 51/53/55/57)

`cam_high` is at base (0.659, 0, 1.610) looking along (-0.778, 0, -0.628) — a 51°
oblique, purely in the x–z plane, so **y is the unbiased cross-view axis**.
Table top z = 0.900 (mode of the cropped cloud, identical on all four seeds).

Top-down 5 mm height map, components above table+0.008:

| component | n cells | centre (x, y) | footprint | z_top |
|---|---|---|---|---|
| stove + cabinet | ~750 | (-0.22, -0.07) | 0.18 × 0.26 | 1.371 |
| **plate** | ~598 | (0.07, 0.03–0.04) | 0.14 × 0.14 | 0.920 |
| **bowl A** | ~300 | (-0.05, 0.20) | 0.115 × 0.115 | 0.952 |
| **bowl B** | ~272 | (-0.18, 0.33) | 0.115 × 0.115 | 0.952 |
| cookie box | ~216 | (0.07, 0.19–0.20) | 0.085 × 0.065 | 0.921 |
| **ramekin** | ~138 | (-0.21, 0.20) | 0.09 × 0.09 | 0.944 |

Layout topology is stable across the four probed seeds; `_pos` moves each prop
by ≲2 cm. The two bowls are geometrically identical, so **"black bowl" alone
does not disambiguate** — the *between* relation does.

Segment-distance test (plate centre → ramekin centre): bowl A lands 0.081 m
from the segment at parameter t = 0.59 (i.e. genuinely *between*); bowl B
projects to t = 1.13, past the ramekin, at ≥ 0.14 m. Margin ≈ 1.7×.
**Target = bowl A**, the bowl nearest the plate–ramekin segment.

### Bowl geometry (seed 51, y cross-section at |x−cx| < 0.008)

- rim top z = 0.952 (52 mm above the table), interior floor z = 0.908
- table reappears at |dy| = 0.055–0.056 → **outer radius ≈ 0.0555**
- rim top edge at |dy| = 0.052–0.053; interior surface falls smoothly to the
  floor by |dy| = 0.020
- outer diameter 0.111–0.115 m vs. a gripper that opens to 0.0778 m
  → **a full straddle is impossible; the only grasp is a rim pinch**
- shell thickness at fingertip depth: ~0.014 at z 0.926, ~0.019 at z 0.920

## Version log

### v1 — perception probe (no motion)
*Hypothesis:* the scene can be read well enough from `cam_high` RGB-D alone to
name the target without demos.
*Evidence:* `results/fs_..._v1`, 0/4 (no motion attempted, as designed). All the
measurements above. The zlib+base64 dump through `api.log` costs ~102 sim steps.
*Verdict:* perception is solid and seed-stable; proceed to a grasp.

### v2 — rim pinch on the −y arc, contact-limited descent
*Hypothesis:* aiming the eef at radius `AIM_R = 0.048` on the bowl's −y arc and
descending until the servo stalls puts the jaws across the shell; closing then
grips it, and `z_stall` doubles as the calibration that fixes the place height
(bowl bottom sat on the table at the stall pose, so it must rise by
plate_top − table_z + clearance to sit on the plate).
*Evidence:* `results/fs_..._v2`, 0/4. Naming was already perfect (2 bowls,
correct plate/ramekin, between-score 0.077 vs 0.126). The motion was not: a
7 mm / 0.8 s descent step lags the command by 8–11 mm **in free air**, so
"lag > 10 mm" declared contact 35 mm above the rim and the jaws closed on
nothing (width 0.001, effort 0.05).
*Verdict:* the stall heuristic is a fake stop. Replace the guess with a
measurement.

### v3 — felt fingertip offset, then rim pinch
*Hypothesis:* pressing the open jaws onto **bare table** (a patch found in the
height map, ≥ 0.075 m clear of every prop) and reading where the eef stops gives
`TIP = z_touch − TABLE_Z`; that one number converts any desired fingertip height
into an eef command and also fixes the place height, without ever needing the
hand geometry. Every move closed-loop (command → measure → cancel the error).
*Evidence:* probe `results/fs_..._v3` **4/4**. TIP = 0.0082–0.0091 on every
seed. Closed gap 0.0075–0.0083 with effort 3.0 — a real bite on the shell.
Formal `results/sel_..._v3` **11/15**; failures 56, 61, 64, 65.
*Verdict:* the grasp mechanism is right. Both failure modes are upstream:

- **56, 64** — the ramekin's footprint touches bowl B's, so height-gating at
  table+0.008 fused them into one 0.205 m blob. With no ramekin left there was
  no *between* test at all, and the program grasped the blob.
- **61, 65** — the cookie box fused with the target bowl and dragged its
  centroid 50 mm toward the box. Reaching the bad aim point the arm jammed
  (eef frozen at x = 0.078 while the bias-cancel loop drove the command to
  x = −0.176) and burned the whole 1000-step horizon.

### v4 — deeper bite (BITE_Z 0.024 → 0.017, AIM_R 0.047 → 0.045)
*Hypothesis:* a lower fingertip height crosses a thicker part of the shell.
*Evidence:* `results/fs_..._v4` 6/8; 61 and 65 failed identically to v3
(hold_w 0.0800 = jaws still open, 1000 sim steps = horizon exhausted).
Closed gaps were unchanged (0.0035–0.0087).
*Verdict:* depth is not the lever — the shell thickness the jaws actually see
is set by the closing squeeze, not by the bite height. Abandoned.

### v5 — ring-vote perception + jam-aware motion  **(FROZEN)**
*Hypothesis:* four changes, each aimed at one diagnosed cause.

1. **Find the bowls by voting for their rim circle**, in a band above the
   ramekin top (0.9465 m), instead of clustering footprints. A fused blob
   cannot fool a circle of fixed radius, and a broken annulus still votes for
   its own centre. Radius swept offline: votes peak at **0.054 m** (55–61)
   versus 48 at 0.0555 and 38 at 0.050.
2. **Find the ramekin as the remainder** of the 0.930 m vessel band once both
   rim circles are masked out.
3. **Drop the per-episode calibration.** It measured 0.0082–0.0091 m on all 15
   seeds, and the excursion to a far-flung bare patch is what put the arm in
   the configuration that jammed. `TIP = 0.0086` is that measurement.
4. **Bounded, jam-aware servoing** plus a transit altitude: if the eef stops
   responding, stop correcting instead of chasing the error.

*Offline validation on all 15 debug frames* (t0 dumps from the v3 formal run):
exactly two rim peaks every time, 37–59 votes against a third peak of 9–16
(2.3× margin); ramekin found 15/15; plate found 15/15 (n = 594–606). The
*between* test and the weaker *nearest-the-plate* test agree on 15/15, worst
segment-distance margin 0.056 vs 0.107.

*Aim-envelope probe* (seeds 51,53,…,65), three offsets run in parallel:

| AIM_R | probe (8) | formal (15) | closed gap |
|---|---|---|---|
| 0.044 (`v5b`) | 8/8 | **15/15** | 0.0071–0.0082 |
| **0.047 (`v5`)** | 8/8 | **15/15** | 0.0072–0.0082 |
| 0.050 (`v5a`) | 8/8 | **15/15** | 0.0072–0.0085 |

The closed gap is the same to ±0.001 across a ±3 mm aim sweep — the straddle
close self-centres, so the aim is not the fragile part. Sim steps fell from
~480 (v3) to ~220 of the 1000-step horizon.
*Verdict:* selected. `AIM_R = 0.047` is the centre of an envelope whose two
edges also score 15/15, so it carries ≥ 3 mm of measured margin on each side.

---

## DECLARATION

- **Frozen version:** `packs/c2clean_spa_bowl_between_pos_k0/program.py`
  == `program_v5.py`, md5 `5ea92704d7f049e485b4228d5a455a55` (verified on the
  cluster).
- **Selection receipt:** **15/15** on the full 15 debug seeds (51–65),
  `results/sel_c2clean_spa_bowl_between_pos_k0_v5`.
- **Receipt chain:**

  | version | run | seeds | score |
  |---|---|---|---|
  | v1 perception probe | `fs_..._v1` | 51,53,55,57 | 0/4 (no motion, by design) |
  | v2 fake-stall descent | `fs_..._v2` | 51,53,55,57 | 0/4 |
  | v3 felt tip offset | `fs_..._v3` | 51,53,55,57 | 4/4 |
  | v3 formal | `sel_..._v3` | 51–65 | 11/15 |
  | v4 deeper bite | `fs_..._v4` | 51,53,…,65 | 6/8 |
  | v5b aim 0.044 | `fs_..._v5b` / `sel_..._v5b` | 51,53,…,65 / 51–65 | 8/8 / 15/15 |
  | **v5 aim 0.047 (frozen)** | `fs_..._v5` / `sel_..._v5` | 51,53,…,65 / 51–65 | 8/8 / **15/15** |
  | v5a aim 0.050 | `fs_..._v5a` / `sel_..._v5a` | 51,53,…,65 / 51–65 | 8/8 / 15/15 |

- **PROVENANCE:** present, 17 entries, all `allowed: True` with a stated source;
  `tools/fair_run.py scan_program(..., "eval")` passes on the frozen file.
  Every constant traces to a debug-seed (51–65) measurement or to generic
  controller/camera mechanics. No demonstration pack exists for this cell and
  none was used; no benchmark asset was read; no success signal is consulted at
  runtime (the program has no `.done` attribute read, and its only verification
  sensors are gripper width/effort, servo residuals and its own re-perception).
