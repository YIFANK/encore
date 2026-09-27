# c2clean / spa_bowl_between_pos_k3 — worker notes

Intent: *pick up the black bowl between the plate and the ramekin and place it on the plate.*
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51-65). No shared note file.

---

## Scene, as re-derived from the pack + debug seeds 51-65

`cam_high` is at base (0.659, 0, 1.610), `fx=fy=618.04`, principal point (256,256) on a
512×512 frame; deprojection was validated against `api.deproject` (centre pixel agreed to
1e-4 m). Table top `tz = 0.9010` in every debug seed.

Five props, positions essentially fixed across the 15 debug seeds (jitter ≲ 2 cm):

| prop | centre (x,y) | footprint | ztop − tz |
|---|---|---|---|
| plate | (0.06, 0.04) | 0.134 × 0.127 | 0.0188 |
| cookie box | (0.07, 0.20) | 0.080 × 0.061 | 0.0194 |
| ramekin | (−0.21, 0.20) | 0.086 × 0.086 | 0.0426 |
| bowl **B** (near) | (−0.05, 0.20) | 0.109 × 0.110 | 0.0506 |
| bowl **A** (far) | (−0.18, 0.32) | 0.109 × 0.110 | 0.0508 |

The bowl is an open shallow vessel: Kasa rim radius 0.055, interior floor at tz+0.007.

**The `_pos` perturbation moved the flat props, not the bowls.** Deprojecting the demo
keyframe blobs (pixel centroid → ray → object's own top plane) gives the demo layout
plate (0.098, 0.200), ramekin (−0.173, 0.200), bowl B (−0.040, 0.201), bowl A
(−0.169, 0.329) — plate, ramekin and bowl B collinear at y≈0.20, bowl B literally
between them. In the debug seeds the two bowls and the ramekin sit at the *same* places
but the plate and the cookie box have swapped. So the demo xy anchors are decoys for the
plate, and the geometric "between" test has to be recomputed per seed.

## Perception (frozen)

1. deproject `cam_high` depth at ½ resolution; `tz` = median z in the workspace box.
2. **tall band** `z > tz+0.033` — the bowls and ramekin fuse below this (they abut in
   5/15 seeds at the full object height) but separate cleanly above it.
3. bowls = tall blobs with `ztop > tz+0.047` **and** footprint > 0.090 m;
   ramekin = tall blob with `ztop ∈ (tz+0.036, tz+0.047]` and footprint 0.055–0.095 m.
   The ramekin/bowl ztop gap is 8 mm and never inverted over 15 seeds.
4. **flat band** `z ∈ (tz+0.012, tz+0.030)` minus the bowl discs; plate = grey
   (mean R−B = 13–14) with footprint > 0.10 m; the cookie box is warm (R−B = 43–48) and
   small, so colour *and* size both separate it.
5. target = the bowl minimising perpendicular distance to the plate→ramekin segment,
   with a penalty for projecting outside it.

Offline over all 15 debug seeds: 2 bowls + 1 ramekin + 1 plate every time, and the
"between" score separates the twins by ≳2× (target 0.057–0.099, other 0.161–0.251).

## Grasp / place

Jaws close along world **y** (tool y = world −y with `rotation=None`; demo keyframe rpy
yaw ≈ 0, so the default wrist is the demo wrist). Pinch the **−y** rim: one finger drops
inside the bowl, one outside, and the close traps the near wall. Demo-derived offsets
(grasp ee minus the deprojected demo bowl centre): dx = −0.001/−0.031/−0.023,
dy = −0.025/−0.039/−0.046, dz = −0.007/−0.027/−0.033 relative to the rim. Frozen as
(−0.010, −0.037, −0.025). Closed width comes out 0.0075–0.0078 m, matching the demo hold
(gripper_state sum 0.0122–0.0130 against 0.0724 open).

Place: the bowl hangs at (+0.010, +0.037) from the tool, so the release point is the
plate centre shifted by the *grasp* offset. Release height = `plate.ztop + (eef_z at the
grasp − tz) + 0.004`, with up to two bias-cancelling re-issues because `api.move`
overshoots z by a steady ~7 mm.

## Version chain

| ver | change | probe 51,53,…,65 | full 15 |
|---|---|---|---|
| v0 | perception probe only (dumps RGB-D through `api.log`) | 0/8 (no manipulation) | — |
| v1 | perceive → rim pinch → release **at the plate centre** | **0/8** | — |
| v2 | release shifted by the grasp offset so the *bowl* lands centred; release height from the measured base offset; z-bias cancel | **8/8** | **15/15** (`results/sel_c2clean_spa_bowl_between_pos_k3_v2`) |
| v2b | same but targets the runner-up twin bowl | **0/8** | — |
| v3 | v2 + fail-open fallbacks when the strict classifier finds nothing + one empty-jaw re-grasp | — | **15/15** (`results/sel_c2clean_spa_bowl_between_pos_k3_v3`) |

### v1 → v2: what the 0/8 actually was

v1 lifted and deposited the bowl on the plate in 8/8 episodes — post-episode re-perception
showed the bowl at `ztop = plate.ztop + 0.050` and 0.029–0.035 m from the plate centre —
and the benchmark bit was still false in all 8. The offset was exactly the grasp offset
(0.038 m) fed through to the release: I had aimed the *tool* at the plate centre, so the
*bowl* landed a rim-radius off and overhung the plate. Aiming the tool at
`plate + (GRASP_DX, GRASP_DY)` brought the landed bowl to 0.008 m of the plate centre and
flipped every episode to true. **The failure was never the grasp; it was that the held
object is not where the tool is.**

### v2b: the twins are not permuted

`_pos` cells can swap identical props, which would make the geometric "between" answer the
wrong *instance*. v2b (identical except it takes the runner-up bowl) scored 0/8 against
v2's 8/8, so the graded instance is the geometrically-between bowl and the perturbation
left the bowls alone.

### v3 additions (both dormant)

Neither the fallbacks nor the re-grasp fired in any of the 15 selection episodes
(`grep FALLBACK`/`RETRY` over the logs is empty). They only cover states where v2 would
have returned without acting, so they cannot cost anything that v2 already earns.

---

## DECLARATION

* **Frozen version:** `program_v3.py`, copied to
  `packs/c2clean_spa_bowl_between_pos_k3/program.py`.
  `md5 = 855418684a8b74c9e99a2b5770872b7d` for **both** files.
* **Selection receipt (full 15 debug seeds 51–65):** **15/15**
  `results/sel_c2clean_spa_bowl_between_pos_k3_v3` (`[fair] …_v3: 15/15`).
  v2, the same policy without the dormant branches, independently scored **15/15** in
  `results/sel_c2clean_spa_bowl_between_pos_k3_v2`.
* **Receipt chain:** v0 perception probe (15 seeds, data only) → v1 0/8
  (`fs_…_v1`) → v2 8/8 (`fs_…_v2`) → v2b 0/8 (`fs_…_v2b`) → v2 15/15 (`sel_…_v2`) →
  v3 15/15 (`sel_…_v3`).
* **PROVENANCE:** present as a top-level literal dict in `program.py`, covering every
  calibrated constant; each source is either a pack field (demo keyframe ee / gripper_state
  / keyframe pixels) or a debug-seed 51–65 measurement.
* **Clean room:** only `packs/c2clean_spa_bowl_between_pos_k3/*` and
  `results/*c2clean_spa_bowl_between_pos_k3*` were written; no benchmark asset, no other
  campaign's artifact, and no other pack's program was read; `--split eval` was never run
  and seeds 1–50 were never touched; `api.done` is never read.

STOP.
