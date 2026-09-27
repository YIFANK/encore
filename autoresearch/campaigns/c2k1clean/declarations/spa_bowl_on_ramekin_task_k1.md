# c2k1clean / spa_bowl_on_ramekin_task_k1 — working notes

Intent: **"Pick the akita black bowl on the cookie box and place it on the plate"**
(bddl passed to the runner is the *ramekin* task; the intent sentence is
authoritative, so the target is the cookie-box bowl, not the ramekin one).

## Reading the two packs

| pack | language | what its demo does |
|---|---|---|
| `..._task_k1` | "pick up the black bowl on the ramekin and place it on the plate" | acts on my TARGET (the plate) with a different object |
| `..._task_mate` | "pick up the black bowl on the cookie box and place it on the plate" | handles my OBJECT (the cookie-box bowl) in a different scene |

Camera model recovered from a probe (`cam_high`): `K = [[618.04,0,256],[0,618.04,256]]`,
`T_base_cam` puts the camera at `(0.659, 0, 1.610)` looking down the −x axis at
≈39° from vertical. Image column → base **+y**, image row → base **+x**. That
mapping lets the packs' 128×128 keyframes be reprojected onto a known-height
plane, which is how the demos' grasp offsets were recovered below.

## Probes (perception only, no motion unless stated)

- **probe0** (seeds 51,53): dumped a 128×128 RGB+depth of `cam_high` back through
  `api.log` as base64 so the scene could actually be looked at. Scene: wooden
  cabinet + stove (far −y), **two** akita black bowls — one on a ramekin, one on
  a red-banded cookie box — and a plate.
- **probe1** (all 15 debug seeds): world-frame blob census. Result is remarkably
  stable — *every* seed shows exactly three blobs above table+2 cm:

  | blob | top z | note |
  |---|---|---|
  | cabinet+stove | 1.1276 | far −y, not in play |
  | bowl on the **ramekin** | 1.0005–1.0007 | rim clipped in x (the bowl sits tilted in the ramekin) |
  | bowl on the **cookie box** | **0.9707 exactly** | full circular rim, dx=0.110, dy=0.111 |

  Table plane `z = 0.9011` on every seed. Target centre wanders only over
  x∈[0.057,0.084], y∈[0.014,0.044]; the plate sits near (0.05, 0.20).

  ⇒ **The two bowls are separated by the height of their support**, which is a
  property of the support, not of where the seed dropped things. That is the
  identity cue, and it is worth 28 mm of margin.

- **probe2** (seeds 51,53,55): two questions at once.
  1. *Which way do the jaws open?* Closed the gripper in place and diffed the
     wrist camera; the changed pixels lie along an image axis that maps to base
     `[0,-1,0]`. **The jaws open along base y** with the default (straight-down)
     wrist. So a rim pinch must sit on the ±y arc — which is exactly the arc the
     mate demo used.
  2. *Does a rim pinch hold?* The bowl is 0.110 m across and the jaws span
     0.080 m, so the body cannot be grasped; the demos pinch the **wall**, one
     finger inside the bowl and one outside. Aiming at `centre + (0, +0.042)`
     and descending to `rim_top − 0.018`:
     - the descent stalls on contact at z ≈ 0.959 (6 mm short — this is fine),
     - close → `width 0.0076–0.0082, effort 3.0` (holding) on **3/3**,
     - after a straight 10 cm lift: held on 53 and 55, **dropped on 51**.

  So the grasp mechanism is right and marginal; the lift is where it is lost.

### Constants recovered from the mate pack
Its demo grasps at ee `(0.069, 0.064, 0.953)`; reprojecting its own
`demo0_t0000.png` puts that bowl's centre at ≈`(0.075, 0.023)`. Hence a rim
offset of **+0.042 in y** and a grasp **0.018 m below the rim top**. It releases
at ee z `0.9476` — only 5 mm below its grasp height, i.e. the bowl simply steps
down from the cookie box to the plate.

## Versions

### v1 — rim pinch on the +y arc, ladder of 5 arcs, contact-guided set-down
Hypothesis: perceive the low-rim bowl, pinch its +y rim arc, verify the hold by
`gripper()['effort']`, retry on another arc if it fails, carry at z=1.06, set
down at `table + 0.040` over the plate centre (offset by the grasp arc so the
*bowl* centre lands on the plate), release, and re-perceive to verify.

**Evidence:** `results/fs_c2k1clean_spa_bowl_on_ramekin_task_k1_v1`, seeds
51,53,55,57,59,61,63,65 → **0/8**, yet every episode returned `"placed"` and
held the bowl the whole way (`effort 3.0` at the set-down on 8/8).

So the pick was sound and the *place* was wrong. Two receipts pinned it:

- ep53 grasped on the **+y** arc, released at ee y `0.1629`, and the end-of-run
  perception found the bowl at y≈`0.137` with the plate at y≈`0.206` — short.
- ep59 grasped on the **−y** arc, released at ee y `0.2440`, and the bowl landed
  at y≈`0.263` — long, by about the same amount.

Undershoot on one arc and overshoot on the other, symmetric about the plate: a
**sign error**. The jaws clamp the wall, so the carried bowl's centre sits at
`eef − grasp_off`; to land it on the plate the eef must go to
`plate + grasp_off`. v1 computed `plate − grasp_off`, i.e. it aimed one whole
rim-diameter (2×0.042 = 84 mm) off. The gif of ep53 shows exactly that — the
bowl set down neatly on the table *beside* the plate, rims touching.

**Verdict:** rejected; the defect is one line of place arithmetic, not the
mechanism.

### v2 — v1 with the place sign corrected (+ two small hardenings)
1. `px, py = plate + grasp_off` (was `plate − grasp_off`). The whole gap.
2. The descend move is re-issued once before closing: one OSC pass leaves ~9 mm
   of x error at the rim, which shortens the wall the jaws engage.
3. The arm parks at (−0.20,−0.18,1.15) before the verification capture, so it
   does not occlude the plate it is trying to measure.

**Evidence:**
- probe subset `results/fs_c2k1clean_spa_bowl_on_ramekin_task_k1_v2`,
  seeds 51,53,55,57,59,61,63,65 → **8/8**.
- formal selection `results/sel_c2k1clean_spa_bowl_on_ramekin_task_k1_v2`,
  all 15 debug seeds → **15/15**.
- re-run against the frozen file after a PROVENANCE-only edit:
  `results/sel_c2k1clean_spa_bowl_on_ramekin_task_k1_v2frozen`,
  all 15 debug seeds → **15/15**.

**Verdict:** accepted; frozen.

A note on the re-authored intent: the bddl handed to the runner is the *ramekin*
task, but the intent names the *cookie-box* bowl. Placing the cookie-box bowl on
the plate fires the benchmark predicate (15/15), so the predicate is not tied to
the ramekin-borne instance, and following the intent and satisfying the
benchmark are the same thing here. No control run on the other bowl was needed
or performed.

## DECLARATION

- **Frozen version:** `packs/c2k1clean_spa_bowl_on_ramekin_task_k1/program.py`,
  md5 `996e29dac4c18928907f260e1b2b9552` == `program_v2.py`
  (`c75c64a9954ff9ca12a624bb27dd9938` is the archived `program_v1.py`).
- **Selection receipt (full 15 debug seeds, 51-65):** **15/15**,
  `results/sel_c2k1clean_spa_bowl_on_ramekin_task_k1_v2frozen`
  (and 15/15 in `..._v2` before the provenance-only edit).
- **Per-version receipt chain:**
  | version | run dir | seeds | result |
  |---|---|---|---|
  | probe0 | `fs_..._probe0` | 51,53 | perception dump (no motion) |
  | probe1 | `fs_..._probe1` | 51-65 | blob census; layout stable on all 15 |
  | probe2 | `fs_..._probe2b` | 51,53,55 | jaw axis = base y; rim pinch grips 3/3, holds 2/3 |
  | v1 | `fs_..._v1` | 8 probe seeds | 0/8 — place-offset sign error |
  | v2 | `fs_..._v2` | 8 probe seeds | 8/8 |
  | v2 | `sel_..._v2` | 51-65 | 15/15 |
  | v2 (frozen) | `sel_..._v2frozen` | 51-65 | **15/15** |
- **PROVENANCE:** present, 12 entries, every top-level constant declared, all
  sourced to the named packs or to debug-seed measurement; checked against the
  eval gate's own rules (no forbidden tokens, no `.done` read).
- No `tools/fewshot_run.py` was invoked; no forbidden asset was read; writes
  were confined to the pack dir and `results/*c2k1clean_spa_bowl_on_ramekin_task_k1*`.

STOP.
