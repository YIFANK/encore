# l90abl / close_microwave_k0 — zero-demo cell notes

Intent: "close the microwave". No demonstration pack; every constant is
re-derived from debug seeds 51-65 and declared in `PROVENANCE`.

## v1 — pure perception probe (seeds 51,53) — `results/fs_l90abl_close_microwave_k0_v1`

**Hypothesis:** nothing is known; look first.

**Evidence.** `cam_high` is at `(0.659, 0, 1.610)` looking toward -x and down,
so image-right is +y and image-up is -x. Table plane z = 0.900 (dominant depth
mode). The microwave is the only strongly blue-dominant body; its top face is a
flat plane at z = 1.108, footprint x ∈ [-0.16, 0.18], y ∈ [0.24, 0.40+]
(clipped by the image edge). At mid height (z 0.95-1.09) the y ≈ 0.24 plane is
solid only for x ≳ 0.10 and x ≲ -0.14 and hollow between — i.e. the recessed
cavity. **The microwave front face therefore has normal -y.** The door is a
thin (~13 mm) vertical panel whose top face reads as a 6 mm-wide diagonal strip
running from the body corner out to (x=-0.245, y=0.00).

**Verdict:** geometry legible from `cam_high` alone. `benchmark_success` false
(no motion), as intended.

## v2 — 15-seed perception dump — `results/fs_l90abl_close_microwave_k0_v2`

**Hypothesis:** is the microwave pose fixed, so constants can be hardcoded?

**Evidence.** Body extents are identical on all 15 debug seeds (top z = 1.108
on every seed; x_max 0.162-0.180, y_max 0.392-0.399). **Only the door angle
varies.** Fitting the top-band strip (z > 1.09, y < 0.18) per seed by PCA and
least-squares intersecting the 15 lines gives a common hinge at
**(-0.178, 0.265)**, per-line residual ≤ 11 mm. Door tip radius 0.254-0.273 m.
Open angle about that hinge (0° = +x = shut against the -y-facing front face)
spans **-75.7° to -117.0°** across the 15 seeds.

**Verdict:** per-episode perception of the door angle is mandatory; the hinge
is a legitimate fixed constant. First attempt (`z < 1.30` blue cut) was wrong —
it caught the robot's own blue links at z ≈ 1.285 and reported an identical
"microwave" on every seed. Fixed with `Z_ROBOT_CUT = 1.15`.

## v3 — perceive-then-arc-sweep — FROZEN

**Hypothesis.** A door rotating about a hinge moves every point of itself along
its own face normal. To shut it, put the closed gripper on the face the door
travels *away from* and walk the tool along a circular arc about the hinge from
the measured open angle round to a small positive overshoot.

Mechanism: outward-side unit vector at panel angle `a` is `(sin a, -cos a)`;
tool xy = `hinge + R·(cos a, sin a) + δ·(sin a, -cos a)`. Descend at
`δ = 0.05` (clear of the panel) then sweep at `δ = 0.008` (on the panel
centreline, so contact is guaranteed and the tool drives the door). Contact
radius `R = 0.20` (inside the 0.266 m tip radius), push height `z = 1.05` (on
the panel, 37 mm above the tallest mug at z = 1.013), 8 arc steps to +8°.

**Evidence.**
- probe, seeds 51,53,55,57,59,61,63,65 — **8/8**
  (`results/fs_l90abl_close_microwave_k0_v3`), 106-280 sim steps of the 1000
  budget. ep51 log: perceived `ang=-98.3`, hinge `(-0.180, 0.265)`, R=0.266;
  the predicate fired partway through the arc (around a = -32°), after which
  the eef stops tracking — the remaining commanded waypoints are no-ops.
- **formal selection, all 15 debug seeds 51-65 — 15/15**
  (`results/sel_l90abl_close_microwave_k0_v3`).

**Verdict:** accepted and frozen. No mechanism gap; the cell is solved on the
first executing version.

## Candidate law (offered to LAWS.md)

*A hinged panel travels along its own face normal.* Fit the panel's thin top
strip, recover the hinge (least-squares intersection of the strip lines over
several seeds — the hinge is fixture-fixed even when the opening angle is not),
then command the tool along a circular arc about that hinge at a radius inside
the panel, offset onto the panel centreline so contact is guaranteed. Descend
with a stand-off, sweep with none. This needs no contact sensing and no success
feedback.

## DECLARATION

- **Frozen version:** `program_v3.py` == `packs/l90abl_close_microwave_k0/program.py`,
  md5 `34bfd3bd7695b8d825f6c59ee0e7f484` (verified identical on the cluster).
- **Selection receipt:** **15/15** on the full debug split (seeds 51-65),
  `results/sel_l90abl_close_microwave_k0_v3`.
- **Receipt chain:** v1 perception probe (0/2, no motion, by design) →
  v2 15-seed perception dump (0/15, no motion, by design) →
  v3 probe 8/8 (`fs_..._v3`) → v3 formal 15/15 (`sel_..._v3`).
- **PROVENANCE:** present in `program.py`, covering all 13 calibrated
  constants; every source is a debug-seed (51-65) measurement or generic
  controller/camera mechanics. No pack (zero-demo cell), no foreign constants.
- Eval seeds 1-50 were never touched.
