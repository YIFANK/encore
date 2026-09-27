# c2clean / spa_bowl_on_cookie_box_pos_k0 — worker notes

Intent: *"pick up the black bowl on the cookie box and place it on the plate"*
Zero demonstrations (k0). Runner: `tools/fair_run.py` only. Debug band 51–65.

---

## Scene, as measured (no pack; all from debug-seed RGB-D)

`cam_high` deprojected into the base frame and rasterised to a 5 mm top-down
max-z height map, then single-link clustered.

| object | centre (x, y) | size | z top | note |
|---|---|---|---|---|
| table | — | — | **0.9010** | modal deprojected height |
| tall fixture (cabinet) | (+0.01, −0.26) | 0.31 × 0.21 | 1.1273 | its top carries the plate |
| **plate** (goal) | ≈ (+0.02, −0.28), moves with `_pos` | r 0.067–0.069 | **1.1480** | interior floor 1.1360 → depth **0.0117** |
| stove slab | (−0.41, −0.14) | 0.09 × 0.09 | 0.9603 | r 0.046 — below the plate radius cut |
| **target bowl** | ≈ (+0.07, +0.03), moves with `_pos` | r_out **0.059** | **0.9707** | interior floor 0.9263 → depth **0.0438** |
| distractor bowl | (+0.05, +0.20) | r 0.059 | 0.9520 | on the table, 19 mm lower |
| cookie box | under the target bowl | — | 0.9204 | 19 mm tall; only a front sliver is visible |
| ramekin | (−0.20, +0.21) | 0.085 | 0.9438 | — |

Target selection needs no colour: the bowl **on** the cookie box is the only
bowl-sized annulus standing 19 mm proud of the others, and `ztop` ranking picks
it on every debug seed. Goal selection uses **rim-minus-floor depth**, which
separates the three disc-like objects with a 4× gap: plate 0.0117, bowl 0.0438,
bare slab ≈ 0.001.

## Controller mechanics, measured (v2 calibration probe)

- **`Z_BIAS` = +0.0112 m** — a free `api.move` settles eef z that far *above*
  every commanded z, identically on every seed and at every height.
- **`FINGER_DZ` = 0.008 m** — closed jaws stall at eef z 0.9088 on the table at
  0.9010, so the fingertips sit 8 mm below the eef origin.
- Jaws open along base **y** at the home rotation (the v1 wrist frame maps image
  u to base −y and the two fingers separate along u), so `Rz(90°)` turns them
  onto base x.
- Bowl (0.118 m) is far wider than the jaws (0.078 m): straddling is impossible,
  a **rim pinch** is the only grasp.
- Held-bowl geometry, from re-perceiving the lifted bowl: its centre sits at
  eef + `GRASP_R` along the outward grasp radial, its rim top at eef z + 0.007,
  its base at **eef z − 0.044**. The bite does not slide; the geometry transfers
  exactly.

---

## Version chain

| ver | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | perception first: dump RGB-D through `api.log` (zlib+base64) and do all perception offline | 51/53/55, 0/3 (no motion by design). Gave the whole table above. | scene solved |
| v2 | measure the controller instead of guessing it: closed-jaw descent onto the bare table + one rim pinch + lift + re-perception | 51/53, 0/2 (no place attempted). `Z_BIAS`, `FINGER_DZ`, closed gap 0.0058–0.0060 at effort 3.0 surviving a 0.20 m lift, held-bowl offset. | mechanics solved |
| v3 | full task: rim pinch at r 0.052 / rim−0.015, carry above the plate rim, lower, release | probe 51..65 odd **8/8**; formal full-15 **14/15** (`sel_..._v3`) | ep58 crashed — see below |
| v4 | ep58's plate test used `median(ring z) − floor`, and the ring band also catches cabinet-top cells, so the "rim lift" read 0.0021 and the plate list came up empty. Replace it with `ztop − floor`, which is a 4× separation, and add fallbacks so perception can never raise. | formal full-15 **15/15** (`sel_..._v4`) | correct |
| v5 | v4's only soft spot is grasp *depth* (see envelope below). Add an own-sensor hold check after the lift (`effort > 1.0 and width > 0.0050`) and re-grasp at a different rim depth on a slip. | formal full-15 **15/15** (`sel_..._v5`), all 15 held on the first try | **FROZEN** |

### Envelope sweep (each a 6–8-seed debug run; "measure the margin, not just the score")

| perturbation | result |
|---|---|
| place aim +15 mm x / −15 mm x / +15 mm y / −15 mm y | 6/6, 6/6, 6/6, 6/6 |
| grasp radius 0.060 (+8 mm) / 0.044 (−8 mm) | 6/6, 6/6 — the close self-centres on the rim |
| grasp depth rim−0.008 | **4/6**, closed gap 0.0047 |
| grasp depth rim−0.022 / −0.028 / −0.035 | 6/6, **7/8**, 8/8 |

So xy is wide open (±15 mm both axes, against a ~9 mm residual aim error) and
depth is the single sensitive axis, good over roughly rim−0.015…−0.035 and bad
at rim−0.008. v5's first try sits at **rim−0.018**, mid-band.

### Retry path, deliberately exercised

An untested recovery path is a liability, so `envprobe_retry` set the first
depth to rim−0.004 to force it. All 8 first tries slipped; 6 recovered on try 1
and 1 on try 2 (1 never held), 5/8 finished successfully, no crash, worst
episode 763 sim steps — the horizon accommodates all three cycles.

### Failure mode the hold check catches

At rim−0.028 on seed 59 the jaws closed to 0.0079 at effort 3.0 but the bowl
slid out during the lift: at the top the width had collapsed to 0.0039 at
effort 0.05. Closed gap alone is therefore *not* a hold receipt — the gap must
still be there **after** the lift.

---

## DECLARATION

- **Frozen version: v5.** `packs/c2clean_spa_bowl_on_cookie_box_pos_k0/program.py`
  md5 `27b9c2c6d8043f07b8b802ea965bce35` == `program_v5.py` (verified on AbakaAI).
- **Selection receipt: 15/15 on the full 15 debug seeds 51–65**, dir
  `results/sel_c2clean_spa_bowl_on_cookie_box_pos_k0_v5`.
- Per-version receipt chain: v1 0/3 (`fs_..._v1`, perception only) → v2 0/2
  (`fs_..._v2`, calibration only) → v3 8/8 probe (`fs_..._v3`) and 14/15 formal
  (`sel_..._v3`) → v4 15/15 formal (`sel_..._v4`) → v5 15/15 formal
  (`sel_..._v5`). Envelope + retry probes in `fs_..._env{px,mx,py,my,gr_p,gr_m,
  gz_p,gz_m,gz_28,gz_35,retry}`.
- **PROVENANCE present** in program.py: 15 entries, every calibrated constant
  sourced to a debug-seed measurement or to generic controller/camera mechanics.
  No pack was issued for this cell and none was read; no prior-context LIBERO
  facts were used — table height, reach behaviour, object sizes, grasp offsets
  and predicate behaviour were all re-derived from seeds 51–65.
- Clean room: writes confined to `packs/c2clean_spa_bowl_on_cookie_box_pos_k0/*`
  and `results/*c2clean_spa_bowl_on_cookie_box_pos_k0*`; no `.bddl`/`.hdf5`/
  `init_states` read; `api.done` never referenced; eval seeds 1–50 never touched.
