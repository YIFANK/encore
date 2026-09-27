# abl_c2 / ablC_goal_put_bowl_on_stove — worker ledger (variant C, "no-verify")

Intent: **put the bowl on the stove**. Variant C contract: exactly ONE program
version, straight to the blind sealed eval; ZERO episodes; `tools/fair_run.py`
never invoked on any split or seed.

---

## 2026-08-21 — inputs consumed

Only `packs/ablC_goal_put_bowl_on_stove/` (scp'd down):
`pack.json` (K=3, `fair-pack-v1`) and the 12 `keyframes/*.png` (128×128).
No other cluster path was read. No `.bddl`/`.xml`/`.hdf5`/init-state file was
opened. No c1/c2/c2fix material, no other abl_c2 cell's pack/results/workspace.

### Measurements taken from the pack (all offline, source text + images)

| quantity | value | evidence |
|---|---|---|
| grasp height (ee z at gripper-close cmd) | 0.9325 / 0.9312 / 0.9235 → **0.925** | pack keyframes t39/t37/t43 |
| deepest descent (ee_path minima) | 0.9269 / 0.9204 / 0.9214 | pack ee_path |
| release height (ee z at gripper-open cmd) | 0.9451 / 0.9721 / 0.9485 → **0.955** | pack keyframes t87/t91/t95 |
| carry height (ee_path maxima) | 1.0526 / 1.0838 / 1.0403 → **1.06** | pack ee_path |
| grasp xy | (-0.0973,0.0563) (-0.1086,0.0460) (-0.1319,0.0390) | pack keyframes |
| release xy | (-0.2729,0.2605) (-0.2516,0.2599) (-0.2453,0.2464) | pack keyframes |
| hold width (finger separation while carrying) | 0.0123 / 0.0063 / 0.0123 m | pack `gripper_state` |
| open width | 0.0786 m | pack `gripper_state` |
| stove blob in image (t0000) | centroid u,v = (92.2,58.9)/(92.5,58.5)/(91.2,59.1) | keyframe pixel analysis |
| bowl blob in image (t0000) | centroid (65.24,69.52)/(63.93,69.89)/(62.91,70.46), 16×13 px | keyframe pixel analysis |
| image scale | 125.8 / 131.5 / 126.5 px·m⁻¹ at 128 px, from Δu(stove−bowl) vs Δy(release ee − grasp ee) | derived within the pack |
| ⇒ bowl outer diameter | 16 px / 128 px·m⁻¹ ≈ **0.125 m**, rim radius 0.0625 | derived |

---

## Version 1 (the only version) — hypothesis → evidence → verdict

**Hypothesis.** The demos do *not* cage the bowl: 0.0786 m of open aperture
cannot span a 0.125 m bowl, and the carried finger separation is 6–12 mm.
That is a **rim-wall pinch** — one finger inside the bowl, one outside — so
the grasp point is `bowl_centre + r·d` with `d` the gripper's finger-closing
axis and `r` the rim radius. Because a bowl rim is rotationally symmetric,
**the sign of `d` is irrelevant**; only the *axis* matters, which collapses
the unknown from four candidates to two.

**Evidence available without running.**
- Pack hold widths and open width (above) refute the caging reading.
- Across the three demos, `u(stove blob) − u(bowl blob)` scales with
  `Δy(release ee − grasp ee)` at a consistent 126–132 px/m, which shows the
  grasp offset is a *constant* vector across layouts (as a rigid rim offset
  must be) and pins the image scale used for the diameter.
- The stove blob centroid is invariant across the three demo layouts
  (u 92.2/92.5/91.2, v 58.9/58.5/59.1, ≈1 px ≈ 1 cm) while the release ee
  spread is 2.4 cm — i.e. the stove is (near-)fixed and the burner tolerates
  ≳±1.5 cm of placement error. The program still perceives the stove and only
  falls back to the pack prior.
- The open hand's silhouette in the grasp keyframes is elongated along image
  `u`, and `u` tracks base `+y`, so `+y` is the default closing axis.

**Program mechanism (frozen).**
1. One `cam_high` RGB-D capture; base-frame cloud built vectorised from
   `intrinsics`/`t_base_cam`, with the axis convention **validated against
   `api.deproject`** at 24 sample pixels (median error < 0.02 m) and a coarse
   `api.deproject` grid as fallback.
2. Table z = modal plane inside [0.75, 0.95] (band implied by the pack's grasp
   height). Objects = points above it.
3. Bowl = the elevated cluster in the pack-implied region scoring best on
   diameter ≈ 0.125 m, height ≈ 0.055 m, annulus occupancy, low saturation,
   and proximity to the pack prior; centre = silhouette bbox centre,
   `r` = half-diameter clamped to [0.048, 0.075].
4. Stove = the large bright low-saturation elevated slab at y > 0.10; burner
   target = its dark disc, blended with the slab bbox centre.
5. **Closing axis measured in-episode, fairly**: the wrist camera is diffed
   open-vs-closed, the two finger lobes are deprojected, and their base-frame
   separation gives the axis (PCA sign ambiguity is harmless). Falls back to
   `+y` whenever any confidence check fails.
6. Up to 3 grasp attempts — (axis, z=0.925), (perpendicular, z=0.925),
   (axis, z=0.916) — each gated **only on the gripper's own sensors**: width
   inside the pack's hold band [0.003, 0.030], effort ≥ 1, and survival of a
   10 cm lift. Re-perceives between attempts. No task/termination feedback is
   read anywhere; `api.done` is never touched.
7. The measured held offset `grasp_xy − bowl_centre_xy` is carried to the
   release so the **bowl**, not the wrist, is centred over the burner.
   Release z = 0.925 + (stove_top − table_z) + 0.008 (clipped to
   [0.945, 0.995]), i.e. set down rather than dropped; falls back to the pack
   release height 0.955.
8. Whole thing wrapped in try/except with prior-based fallbacks at every
   perception step, so a perception failure degrades to the pack's own
   open-loop demo pose rather than aborting.

**Verdict.** UNVERIFIED BY CONSTRUCTION — this is the ablation. No episode
was run on any split or seed, so there is no empirical evidence for or
against this version. It is frozen and submitted as written.

### Static self-checks (source-text only; no policy executed)
- `python -m py_compile program.py` → OK
- forbidden-token scan (`gt_pose`, `gt_trace`, `gt_joints`, `_sim`, `env.sim`,
  `sim.data`, `body_xpos`, `site_xpos`, `geom_xpos`, `.bddl`, `bddl_files`,
  `init_states`, `pruned_init`, `h5py`, `.hdf5`, `api.robot`, `self.robot`,
  `check_success`, `OffScreenRenderEnv`) → **NONE**
- AST scan for any `.done` attribute read → **0**
- top-level literal `PROVENANCE` dict → present, 20 entries, every entry has
  `{"source": ..., "allowed": True}`; every module-level constant is covered
  (`uncovered: NONE`)
- `run(api)` defined at module level → yes

### What I would have probed first, had probing been allowed
Ranked by how much of the residual risk each would remove:
1. **The finger-closing axis** — a single debug episode would settle base-`y`
   vs base-`x` outright and delete the whole retry ladder (and its step cost).
2. **Rim pinch vs. some other demonstrated grasp mode** — one close-up and one
   `api.gripper()` reading after closing would confirm or destroy the entire
   grasp hypothesis, which everything downstream rests on.
3. **The EEF ↔ fingertip offset**, i.e. whether z = 0.925 really seats the
   fingers across the rim; I transferred the demo's absolute height because I
   could not measure the offset, and 1 cm of error either jams the table or
   misses the rim.
4. **Step/time budget of the eval horizon** — unknown, so the retry ladder is
   capped at 3 attempts on a guess; a probe would let me spend the real budget.
5. **The perception pipeline against ground truth in one frame** — bowl centre,
   rim radius, stove slab and burner disc; the camera-convention check is
   self-validating but the clustering thresholds are not.
6. **Whether the stove is genuinely fixed** across seeds (3 demo layouts hint
   yes) and how much placement error the burner tolerates.

### Candidate law (falsifiable, receipt = this pack)
> **Aperture-vs-silhouette test for grasp mode.** When a demo pack's carried
> `gripper_state` separation is an order of magnitude below the open aperture
> *and* the object's silhouette exceeds the open aperture, the demonstrated
> grasp is a rim/wall pinch, not a cage — so the transferable grasp target is
> `object_centre ± r·(closing axis)`, and because the rim is symmetric the
> sign is free. Receipt: hold widths 0.0123/0.0063/0.0123 m vs open 0.0786 m
> against a 0.125 m silhouette in `packs/ablC_goal_put_bowl_on_stove`.
> (Falsifiable: a cage grasp of this bowl would have to report a carried
> separation near the silhouette width.)

---

## DECLARATION

- **Frozen version: `program.py` == `program_v1.py`, md5 `911241ce3e36149a9762ce415ad5aa15`**
  (verified identical on the cluster at
  `/mnt/data/YifanKang/Heron/packs/ablC_goal_put_bowl_on_stove/`).
- **Episodes run: 0.** `tools/fair_run.py` was never invoked — not on
  `--split eval`, not on any debug/selection split, not on any seed. Seeds
  1–50 and 51–65 alike were never touched. `tools/fewshot_run.py` was never
  invoked.
- **Program versions written: 1.**
- **PROVENANCE: present**, top-level literal dict, 20 entries, all
  `allowed: True`, covering every calibrated constant. Every source is a
  pack field or generic controller/camera mechanics (no debug observations
  exist for this variant).
- Clean room respected: cluster reads limited to
  `packs/ablC_goal_put_bowl_on_stove/`; cluster writes limited to
  `packs/ablC_goal_put_bowl_on_stove/program.py` and `program_v1.py`.
  No `.bddl`/`.xml`/`.hdf5`/init-state read; no c1/c2/c2fix material; no
  other abl_c2 cell touched; no LIBERO priors from memory or system context
  were used — every constant re-derived from this pack.

STOP.
