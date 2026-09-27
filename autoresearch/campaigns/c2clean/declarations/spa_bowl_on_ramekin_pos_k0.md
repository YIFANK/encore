# c2clean / spa_bowl_on_ramekin_pos_k0 — working notes

Cell: "pick up the black bowl on the ramekin and place it on the plate".
No demo pack (k0). Every constant re-derived from debug seeds 51-65 only.

## Scene (re-derived, v1 probe, all 15 debug seeds)

Runner captures cam_high RGB-D; zlib+base64 through `api.log` ships the frames
off-box so perception is designed offline at zero sim cost (`api.log` truncates
a message at ~2000 chars, so chunks are 1800).

Table plane z = 0.9006 (modal z of the workspace cloud, identical all seeds).
Four props, consistent in all 15 debug seeds:

| prop | top above table | footprint | mean RGB |
|---|---|---|---|
| stove slab | 0.032 | 0.135 x 0.155 | 69 (dark) |
| plate | 0.039 (floor +0.026) | 0.135 x 0.140 | 154 (bright) |
| bowl on ramekin | 0.100 (crest) | 0.110 x 0.110 | ~110 |
| bowl on table | 0.051 | 0.110 x 0.110 | ~118 |

Identification is structural, no position constants:
* bowls = square-ish footprint 0.085-0.135, top > table+0.035, dark (rgb < 120)
* **target = the bowl with the higher rim** (the ramekin lifts it ~0.048)
* plate = flat (< table+0.055), bright (rgb >= 120), large
15/15 debug seeds classify with exactly 2 bowls + 1 plate, target unambiguous.
`_pos` here is ~+/-20 mm jitter, not a permutation, but nothing is hardcoded.

## Calibrations (all from debug-seed observation)

* **TIP_BELOW_EEF = 0.0093** — cam_arm_wrist depth at episode start resolves
  both finger pads; lowest pad point z = 1.1640 against api.eef() z = 1.17328.
* **width_m is the pad inner-face gap** — wrist depth puts the pad inner faces
  at +/-0.0389 in tool y while api.gripper() reports width_m = 0.0778.
* Bowl outer dia 0.110 > JAW_MAX 0.0778, and the rim is the bowl's widest
  point (the body tapers to a small foot under an overhanging rim), so a
  vertical body straddle is geometrically impossible. **Rim pinch is forced.**

## Version log

### v1 — perception probe (no motion)
Hypothesis: ship RGB-D off-box and design perception offline.
Evidence: 15/15 captures decoded; scene table above; wrist camera resolves the
fingertips. Verdict: perception basis established, 0 sim cost beyond 72 steps.

### v2 — first full attempt, aim at the global crest — **0/8**
Hypothesis: pinch the rim 12 mm below the cluster's max height, on the -y arc.
Evidence (results/fs_..._v2, ep51 log):
* `MOVE descend want z=0.998 got z=1.009` — **api.move stops ~10 mm short**;
  residuals 0.0076-0.0108 on every leg, i.e. the controller's stop band is
  ~0.010. Open-loop aiming cannot place a 5 mm bite.
* `GRIP closed width=0.0010 effort=0.05` — jaws closed on air on all 8 seeds;
  effort never reached 3.0 at any point in any episode.
Verdict: two independent aiming errors, both fatal. Fixed in v3.

### The tilt (diagnosed from the v1 captures, after v2 failed)
The bowl standing on the ramekin is **tilted ~16 deg toward the camera**, in
every one of the 15 debug seeds identically:

| meridian | rim height above table |
|---|---|
| far (-x) | 0.100 |
| -y | 0.0835 |
| +y | 0.0875 |
| near (+x) | 0.070 |

far-minus-near = 30 mm in 15/15 seeds; the +/-y meridians sit at the mean — a
clean azimuthal sinusoid, i.e. a genuine tilt, not a view artefact. Control:
the table-standing bowl reads a level 0.050 rim on all four meridians, so the
oblique view is not responsible.

Consequence: the cluster's `zmax` (0.100) is the height of the *far arc only*.
v2 aimed the grip depth off it and so aimed ~15 mm above the rim where the
jaws actually close. **A tilt about y leaves the wall vertical in the jaw
plane at the +/-y meridians**, which is therefore where a straight-down pinch
belongs — and the rim height must be read on that same meridian.

Wall at the -y meridian: exterior radius 0.0548 at the rim, interior reaching
0.050 — a ~5 mm lip, tapering inward ~0.45 mm per mm of depth.

### v3 — meridian-local aim + closed-loop moves
Changes: (a) `go()` re-issues each command shifted by the observed error, up
to 3 tries, clamped to +/-0.06 of the target; (b) rim height and wall radius
read on the -y meridian, not from the global crest; (c) grip 8 mm below the
local rim at mid-wall (r_ext - 0.0025).
Offline check on all 15 seeds: rim_z 0.9830-0.9849 (+/-1 mm), r_ext@grip
0.047-0.051. Result: pending.

### v4 — carry the rim-pinch offset — **8/8 probe, 15/15 formal**
Hypothesis: the pinch already works; what remains is that the bowl's centre
sits a full grip radius (0.046) from the eef, so driving the eef to the plate
centre parks the bowl on the plate's rim.  Aim the eef at
`plate_cy - (-GRIP_SIDE * r_grip)` instead.
Evidence: probe 8/8 (results/fs_..._v4); formal 15/15
(results/sel_..._v4).  Independent confirmation: the post-lift re-perception
measures the held bowl's centre at +0.039..+0.050 in y from the eef, matching
the geometric r_grip 0.045-0.048 it was derived from.
Verdict: carry offset was the whole remaining gap.

Side finding — **`effort` is a gap threshold, not a hold signal.**  It reads
3.00 iff width_m > ~0.005.  In 3 of the 8 v4 probe episodes the grip decayed
to 0.0034-0.0037 and effort fell to 0.05, yet all three still scored: the bowl
was never dropped.  Grip width decays ~0.0005-0.0010 per move under load
(0.0063 -> 0.0058 -> 0.0048 -> 0.0038 in v3 ep51), so loaded moves are
budgeted, not distances — but the decay alone does not predict failure.

### v5 — frozen version. Declared. **15/15**
No behaviour change from v4; adds `GRIP_SIDE` and `WORKSPACE_CROP` to
PROVENANCE, which v4 had left undeclared, and was re-run formally so the
frozen md5 owns its own receipt.

### Aim envelope (diagnostic, not a candidate version)
`probe_envp4.py` / `probe_envm4.py` = v4 with the radial grip aim biased
+4 mm and -4 mm.  Both scored **5/5** on seeds 51,54,57,60,63, with closed
bites of 0.0063-0.0071 — indistinguishable from the unbiased version.  The
jaws self-centre on the ~6 mm rim lip, so the grasp tolerates at least
+/-4 mm of radial aim error.  15/15 is therefore not a knife-edge result.

## DECLARATION

* **Frozen version: v5.**
  `packs/c2clean_spa_bowl_on_ramekin_pos_k0/program.py`
  md5 `bbb80f5a591d17c880f9f3911e1fe093` == `program_v5.py` (verified on the
  cluster).
* **Selection receipt: 15/15** on the full debug split (seeds 51-65),
  `results/sel_c2clean_spa_bowl_on_ramekin_pos_k0_v5`.
* **Receipt chain**
  | version | change | probe | formal |
  |---|---|---|---|
  | v1 | perception probe, no motion | — | — |
  | v2 | rim pinch aimed at the global crest | 0/8 | — |
  | v3 | meridian-local aim + closed-loop moves | 0/8 (8/8 grasped) | — |
  | v4 | + rim-pinch carry offset | 8/8 | 15/15 (sel_..._v4) |
  | v5 | PROVENANCE completion, no behaviour change | — | **15/15** (sel_..._v5) |
  | env+4/-4 | aim-bias diagnostics | 5/5 and 5/5 | — |
* **PROVENANCE present** in program.py, covering TIP_BELOW_EEF, JAW_MAX,
  BOWL_H, BOWL_DIA, GRIP_DEPTH, WALL_HALF, MOVE_TOL, PLATE_RGB_MIN,
  GRIP_SIDE, WORKSPACE_CROP, R_DOWN.  Every constant is sourced to a
  debug-seed (51-65) observation or to generic controller/camera mechanics.
  No pack (k0), no foreign constants, no seeds 1-50 touched.

Three findings carried the cell: the target bowl is **tilted ~16 deg** so its
cluster zmax is the far arc only and the grip must be aimed on a +/-y
meridian; `api.move` **stops ~10 mm short** so every aim must be closed-loop;
and a rim pinch **carries the object a full grip radius off the eef**.
