# c2clean / obj_milk_pos_k0 — worker notes

Intent: **pick up the milk and place it in the basket**. No demo pack (k0).
Everything below is derived from debug seeds 51-65 only.

## Tooling

`api.log` truncates each message at **2000 characters**. A chunked
zlib+base64 RGB-D dump therefore has to use ~1900-char chunks; 4000-char
chunks silently lose half of every chunk (base64 padding error on decode).
With 1900-char chunks a full `cam_high` RGB-D reconstructs exactly, so
perception can be developed offline against real frames at zero sim cost.

## v1 — perception datapipe (seeds 51,53,57,61)

Hypothesis: the scene can be read from `cam_high` alone.
Evidence:

- Deprojection with the OpenCV convention (`p_cam = ((u-cx)/fx*d,
  (v-cy)/fy*d, d)`, `p_base = R p_cam + t`) is correct: the empty surface
  comes out flat at **z = 0.0012 m** across the whole frame, so the base
  frame's z origin sits on the table top.
- `cam_high` sits at (0.897, 0, 0.65) looking along (-0.849, 0, -0.529),
  i.e. 32 deg below horizontal toward -x. Image top = -x, image right = +y.
- Walls/floor deproject out to x = -1.99, y = +-1.15, so the workspace must
  be cropped before anything is fitted. Props live in
  x in [-0.21, 0.18], y in [-0.27, 0.35].

Verdict: datapipe good.

### Clustering trap (cost: one wasted analysis pass)

Clustering everything above the table **fuses the milk and the can into the
robot-arm column** — the arm hangs over them in the top-down projection, so
a single connected component spans z 0.006 to 0.482 and the two tall props
vanish from the table. Band-masking to **z in (0.006, 0.19)** before
grouping drops the arm entirely and recovers 8 clean clusters.

### Scene (seed 51)

| prop | top z | xy | footprint | mean rgb |
|---|---|---|---|---|
| basket | 0.142 | (+0.03, +0.26) | 0.153 x 0.170 | 0.55 0.55 0.53 |
| **milk carton** | 0.140 | (-0.180, -0.080) | (occluded) x 0.053 | 0.345 0.244 0.215 |
| juice carton | 0.140 | (+0.171, +0.029) | 0.045 x 0.053 | 0.367 0.264 0.128 |
| can (2 pieces) | 0.081 | (-0.13, +0.057) | — | — |
| flat box | 0.030 | (-0.105, -0.237) | 0.074 x 0.048 | — |
| flat box | 0.020 | (+0.055, -0.097) | 0.080 x 0.041 | — |
| flat box | 0.019 | (+0.107, -0.203) | 0.074 x 0.039 | — |

Two tall cartons, one milk one juice. They separate on **blue fraction**
(blue / (r+g+b)): milk **0.267**, juice **0.169** — the milk carton's white
panel lifts blue, the juice carton's orange graphic suppresses it. The gap
is 0.10, far wider than the ~0.001 seed-to-seed jitter, so rank on it.

### The cartons are gable-topped

Reading the raw (x, z) profile down the milk's pixels:

    x   -0.200 -0.190 -0.180 -0.175 -0.175 ... -0.175
    z    0.132  0.124  0.116  0.106  0.093 ...  0.011

The top is not flat: a roof slopes from a **ridge at x = -0.200, z = 0.141**
down to the shoulder at x = -0.175, z = 0.106, below which the body face is
vertical at x = -0.175. Body depth is therefore 2 x (0.200 - 0.175) = 0.050,
and the **ridge x equals the body centre x** — the single most useful
number, because it is the only x measurement the grazing view gives
honestly. (Same structure on the juice carton: ridge 0.149, face 0.176.)

So the milk grasp target is `(ridge_x, mid of the y range)` — y is measured
cleanly (5.1 cm) because the carton is not occluded across y.

## v2 — fingertip offset + reach calibration (all 15 debug seeds)

Ports the clustering into the program (full 512x512 frames), logs the whole
cluster table per seed, then:
1. presses a **closed** gripper into an empty table spot (0.02, 0.12) with a
   z = -0.06 command — where the eef freezes gives the fingertip-to-eef
   offset;
2. steps down over the milk at z = 0.26/0.22/0.18/0.15 logging residuals, to
   find whether x = -0.20 is inside the reach envelope.

Evidence (all 15 debug seeds, `fs_..._v2`):

- **Fingertip probe was a lie.** A closed gripper commanded to z = -0.06 at
  (0.02, 0.12) froze at eef z = 0.1013 +- 0.0005, dead consistent across 15
  seeds — which reads exactly like table contact and is not. See v4.
- **Reach:** the milk at x ~ -0.20 is comfortably inside the envelope;
  residual sits at a 0.008-0.011 tracking floor at every height 0.26 -> 0.15,
  with no stall.
- **Perception is stable on 15/15:** `TALL=2` on every seed, milk blue
  fraction 0.268-0.270 vs juice 0.164. Layout spread is small — milk
  x in [-0.203, -0.196], y in [-0.086, -0.076]; basket x in [0.011, 0.037],
  y in [0.244, 0.272].

Verdict: calibration good except TIP_DZ, which was wrong.

## v3 — first pick-and-place: 0/8

Grasped at eef z = 0.150 assuming fingertips 0.100 m lower. Gripper closed to
**width 0.001 (air)** on every seed, and the post-episode cluster table was
byte-identical to the pre-episode one: nothing was ever touched. The GIF shows
the two finger prongs hanging just above the carton's roof.

## v4 — measure the fingertips instead of inferring them

Held the hand over empty table and looked at it with `cam_high`, taking the
lowest gripper point within 10 cm of the eef xy:

| eef z | lowest gripper point | dz |
|---|---|---|
| 0.3638 | 0.3558 | 0.005 |
| 0.3164 | 0.3083 | 0.004 |
| 0.2646 | 0.2563 | 0.002 |

**The eef frame sits at the fingertips** (TIP_DZ ~ 0.006), not 0.100 above
them. So in v3 the fingers closed at z ~ 0.151, just above the 0.142 ridge —
which is exactly what the GIF showed.

Two more facts from the same run:

- The fingers close along **base y**; `width_m` is the inner-face gap (open
  0.080 -> outer faces 0.093 apart, shut 0.001 -> 0.0185 apart, so each finger
  is ~6.5 mm thick).
- **The z-floor is a function of xy, and it is not the table.** At
  (0.08, -0.28) the arm cannot get below z ~ 0.25 at all: commanding z = 0.02
  left the eef at 0.2519 with residual 0.2324. The v2 "contact" at z = 0.1013
  was this same envelope wall at a different xy. A repeatable stall is not
  evidence of contact.

## The roof is a prism, not a gable end

Slicing the milk cluster by height, the y-width is **0.050 at every z from
0.00 to 0.15** — including 0.13-0.15, right at the ridge. The roof slopes only
in x; the gable ends are flat vertical triangles running the full width.

That matters because a top-down grasp on the *body* is impossible: the fingers
are only ~0.075 long, so putting the fingertips at mid-body (z ~ 0.05) would
drive the hand into the 0.141 ridge. But since the cross-section is constant,
gripping **just below the ridge** works just as well — the fingers close along
y onto two flat vertical faces, and the hand stays clear above.

## v5 — grip the roof below the ridge: **15/15**

`z_tip = ridge - 0.028`, `z_cmd = z_tip + TIP_DZ`, closing along y at
`(ridge_x, mid of the y range)`; carry at 0.300, release at 0.200 over the
perceived basket centre. `goto()` commands, reads `api.eef()` back and cancels
the standing bias once, because the open-loop tracking error runs ~10 mm.

Verification is by my own sensors, not by any success signal: the close is
accepted only when `W_LO < width_m < W_HI` (0.030-0.062, against the measured
0.050 cross-section), with one retry 30 mm lower if it misses. On every seed
the close reported width 0.0537 and effort 3.0, and the post-episode scene
showed a new cluster topping out at 0.189 inside the basket's footprint — the
carton standing in the basket, gone from x = -0.20.

| receipt | seeds | result |
|---|---|---|
| `results/fs_c2clean_obj_milk_pos_k0_v5` | 51,53,55,57,59,61,63,65 | **8/8** |
| `results/sel_c2clean_obj_milk_pos_k0_v5` | 51-65 (all 15) | **15/15** |

### Aim envelope (score alone says nothing about margin)

| probe | change | seeds 51,55,59,63 |
|---|---|---|
| `probe_aim12` | grasp xy displaced +12 mm in **both** x and y | 4/4 |
| `probe_drop12` | grip at ridge - 0.012 instead of ridge - 0.028 | 4/4 |

So the grasp tolerates at least 12 mm of aim error and a 16 mm shift in grip
height. The margin is not thin.

### Residual risk

The debug band perturbs the layout only slightly (milk within 7 mm in x, 10 mm
in y). The program perceives rather than hardcodes, so a shifted layout is
handled, but two failure modes are untested because no debug seed exhibits
them: (1) the milk placed close enough to another prop to fuse with it in the
4 mm footprint clustering, and (2) the milk placed at an xy whose z-floor is
above the grasp height — the envelope wall found at (0.08, -0.28) is real and
would defeat the descent.

---

# DECLARATION

- **Frozen version:** `program_v5.py`, copied to
  `packs/c2clean_obj_milk_pos_k0/program.py`.
  md5 `c0b16be1e1ce747f9ecba83a86b749ac` — identical for both files, verified
  on the cluster.
- **Selection receipt:** **15/15** on the full 15 debug seeds (51-65),
  `results/sel_c2clean_obj_milk_pos_k0_v5`.
- **Receipt chain:**
  - v1 — perception datapipe, seeds 51/53/57/61, no motion (0/4, by design).
  - v2 — calibration + perception on all 15 seeds (0/15, by design).
  - v3 — first pick-and-place, `fs_..._v3`, **0/8** (closed on air).
  - v4 — fingertip/envelope measurement, seeds 51/57 (0/2, by design).
  - v5 — `fs_..._v5` **8/8**, `sel_..._v5` **15/15**.
  - envelope probes: `probe_aim12` 4/4, `probe_drop12` 4/4.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  WS_X, WS_Y, Z_TABLE, Z_BAND, GRID_RES, TALL_Z, PROBE_XY, TIP_DZ, GRIP_DROP,
  W_LO/W_HI, HANG, OPEN_W, Z_CARRY, Z_DROP, PARK_XY. Every constant traces to a
  debug-seed measurement or generic camera/controller mechanics; no pack, no
  foreign source.
