# c2clean / goal_put_wine_top_cabinet_task_k3

Intent: **put the wine bottle in the bowl**  (bddl file name is a decoy — the
instruction sentence is authoritative; the graded object/target pair is
bottle → bowl).

## Packs (roles are swapped relative to the brief's wording — content wins)
- `..._task_k3`  language "put the wine bottle on top of the cabinet" — demos the
  OBJECT my intent names (the wine bottle), toward a different target.
  K=3 grasp keyframes: eef (-0.182,-0.065,1.029) / (-0.246,-0.047,0.992) /
  (-0.233,-0.061,1.024); closed gaps 0.0165 / 0.0246 / 0.0157.
- `..._task_mate` language "put the cream cheese in the bowl" — demos the TARGET
  my intent names (the bowl), with a different object. Release keyframes
  (-0.096,-0.025,0.959) / (-0.045,-0.010,0.973) / (-0.084,-0.002,0.979).
  → the bowl sits near (-0.08, 0.00) and a release just under z 0.96 works.

## v0 — perception dump (no motion)
Hypothesis: the scene must be perceived per-seed; the pack grasp x spans 6 cm.
Method: `api.log` used as a data pipe (zlib+base64 chunks) to stream the
cam_high RGB (512x512x3) and depth (uint16, 0.1 mm) off all 15 debug seeds.
Receipt: `results/fs_..._v0` — 15/15 episodes dumped, 0 successes (no motion).

Offline findings (every number below re-derived from those dumps only):
- table plane z = 0.9010 on all 15 seeds (histogram mode).
- wine bottle: dark **green-tinted** pixels ([1,6,0]); every other dark prop in
  the scene is neutral grey. Vertical cylinder, radius by height band —
  body 20.2 mm (table+30..65), shoulder 15.5 mm (+80..95), neck 6.9 mm
  (+115..142), apex table+147 mm. Centre estimator (x_max − r, y_mid) agrees
  across the three bands to **< 0.8 mm on 15/15 seeds**.
  (x_min is biased: at this camera's 39° grazing view the silhouette edge
  deprojects short, so x_max − r is the estimator that is self-consistent.)
- bowl: the single connected component in the [table+43, table+62] mm band with
  a 90–140 mm square footprint. Found uniquely on 15/15 seeds; rim table+51 mm,
  outer diameter 108 mm, interior floor ≈ table+6 mm.
- bottle xy spans (-0.209..-0.187, -0.065..-0.038); bowl xy spans
  (-0.104..-0.077, -0.015..+0.014) — so both really do move seed to seed.

## v1 — neck grasp + measured hang
Hypothesis: grasp the bottle **by the neck** so that the fingers stay well above
the bowl rim while the base goes inside; measure the bottle's hang below the
tool by re-perceiving after the lift, so the unknown fingertip-to-eef offset
never enters the drop height.
Why a neck grasp: the bottle is 147 mm tall and the bowl rim is only 51 mm above
the table, so a body grasp would put the fingers at rim height at release. Held
by the neck the fingers stay ~120 mm above the rim throughout the insertion, and
the neck (13.8 mm) is the one band whose width is unambiguous in the closed
finger gap — so the gap itself reports whether the right part was caught, and a
mis-aimed close can be re-tried without any success signal.

Why re-perceiving after the lift: the fingertip-to-eef offset is unknown (the k3
pack's three closes imply anything from 0 to −24 mm). Measuring the bottle base
directly (lowest green point within 7 cm of the tool, ceiling at eef−20 mm to cut
the arm's own green links) gives the hang, and the drop height is then
`table + 18 mm + hang` with no offset in it at all.

Receipts:
- probe `results/fs_c2clean_goal_put_wine_top_cabinet_task_k3_v1`
  (51,53,55,57,59,61,63,65) — **8/8**.
- formal `results/sel_c2clean_goal_put_wine_top_cabinet_task_k3_v1`
  (51..65) — **15/15**.

Margin, read off the 15 selection logs (no success signal used):
- closed gap **0.0150 on 15/15**, first attempt, retry loop never entered;
  effort 3.0 held from the close through the drop.
- measured hang 0.131–0.138 m (spread 7 mm), consistent with a neck grasp.
- final bottle base table+14..21 mm, apex table+152 mm — the bottle stands
  upright, inside the bowl, 2–8 mm from the perceived bowl centre. Interior
  radius ≈ 45 mm against a 20 mm base radius, so ~25 mm of unused slack.
- perception spread between the three cylinder bands ≤ 0.8 mm on 15/15; the bowl
  was the unique candidate component on 15/15.

## DECLARATION
- Frozen version: **program_v1.py**, copied to
  `packs/c2clean_goal_put_wine_top_cabinet_task_k3/program.py`;
  `md5 = 554a853a44303f0a176c32f2deee6d2c` for both files.
- Selection receipt: **15/15** on the full debug band 51–65,
  `results/sel_c2clean_goal_put_wine_top_cabinet_task_k3_v1`.
- Version chain: v0 (perception dump, no motion, 0/15 by construction) →
  v1 (8/8 probe, 15/15 formal). No further versions were needed.
- PROVENANCE: present in program.py; every constant sourced to the v0 debug-seed
  RGB-D dumps or to generic controller mechanics. No pack constant is used as a
  coordinate — the two packs contributed only the roles (bottle = object,
  bowl = target) and the confirmation that a release just under the rim works.
- `api.done` is never read; success was never queried at runtime.
