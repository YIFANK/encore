# c2clean / goal_put_cream_cheese_in_bowl_task_k3

Intent: **"put the wine bottle in the bowl"** (re-authored; the bddl file name is
the original cream-cheese task and is not the intent).
Runner: `tools/fair_run.py` only. No shared note file (FAIR_PROTOCOL v1.1.1).

## Reading the two packs

| pack | language | what it gives me |
|---|---|---|
| `..._k3`   | "put the cream cheese in the bowl"       | my TARGET, wrong object — where the bowl is and how a release over it looks |
| `..._mate` | "put the wine bottle on top of the cabinet" | my OBJECT, wrong target — how the wine bottle is grasped |

The intent is the join: take the mate pack's grasp, take the k3 pack's release
site. Neither pack's motion is replayable end to end.

Numbers lifted from the packs (keyframes at the gripper_cmd sign changes):

* mate grasp z = 1.029 / 0.992 / 1.024, held finger-state sums 0.0157 / 0.0246 /
  0.0165 -> the demo closes on the bottle's **neck**, not its body.
* k3 release xy = (-0.096,-0.025) / (-0.045,-0.010) / (-0.084,-0.002),
  release z = 0.959 / 0.973 / 0.979 -> the bowl sits near the origin and the
  release is ~0.06 above the table. Used only as a cross-check on my own
  perception, never as a hard-coded target (the bowl moves seed to seed).

## Scene, measured on debug seeds (probe1, perception only, 0 sim steps)

RGB-D was dumped through `api.log` (zlib+base64, 1700-char chunks) and analysed
offline; `probe1` = 3 seeds, `probe2` = the other 12, so all 15 debug seeds have
a stored point cloud.

* table plane z = **0.901** on all 15 seeds.
* wine bottle: **0.158 m** tall (top 1.0588), body diameter 0.043, shoulder at
  ~0.98, **neck diameter 0.0135 from z=1.007 to the cap**.
* bowl: rim 0.952 (**0.051 proud**), outer diameter 0.110, **interior floor
  0.9075** (6.5 mm above the table), flat out to r=0.03.
* other props: stove slab 0.058 tall, plate 0.019, cream-cheese box 0.019,
  cabinet 0.344, arm 0.450.

## Version chain

### v1 — first mechanism. **5/8** (probe 51,53,55,57,59,61,63,65)
`results/fs_..._v1`

Hypothesis: grasp the neck at `bottle_top - 0.035` (the mate pack's demonstrated
close height), lift, translate, and lower until the bottle **base** — a fixed
`grasp_z - table` below the hand — sits 6 mm above the measured bowl floor.
Holding the neck is what makes this possible: the hand stays 8 cm clear of the
rim while the base goes deep into the bowl.

Evidence: 5/5 on every seed where perception resolved; **0/3 on 61, 63, 65**,
all with note `no bottle candidate`.

Verdict: the *motion* is right, the *segmentation* is not. On those three seeds
the bottle and the bowl merge into ONE 4-connected component (n≈3640 vs 1300+2350
separately) because the bottle occludes part of the rim in the image. v1 keyed
the bottle on footprint (<0.10 m) and the fused blob measures 0.16, so it was
rejected and the bowl vanished with it.

### v2 — order the segmentation. **8/8** probe, **15/15** selection
`results/fs_..._v2`, `results/sel_..._v2`

Hypothesis: find the bottle *first* on the one cue the fusion cannot corrupt —
height, 0.158 vs 0.058 for the next-tallest prop — then re-mask the band with a
0.032 m cylinder around the bottle axis removed and re-cluster. The bottle's
axis comes from the **top 5 mm** of its cluster (the cap's horizontal face,
which the oblique camera sees whole); the body's bbox is biased +x because its
far side is hidden (seed 51: cap centre -0.2028 vs near-edge+radius -0.2027,
vs whole-cluster bbox -0.1957, a 7 mm error).

Evidence: offline, the cylinder cut restores the bowl on all 15 seeds
(n = 2286-2377, indistinguishable from the unfused seeds) and a Kasa rim fit
returns **r = 0.0536-0.0537 on every one of the 15** — the receipt that it
found a rim and not a silhouette. On the cluster: 8/8 probe, then **15/15**
full selection, `held=True` on every episode.

Verdict: works, but one fragility survives (below).

### v3 — bowl by circularity, not by size. **15/15** selection — FROZEN
`results/sel_..._v3`

Hypothesis (falsifiable, and the reason not to stop at v2): v2 picked the bowl
as the *tallest* cluster inside a 0.07-0.16 m footprint window. The **stove slab
stands 0.058 m — taller than the 0.051 m bowl and inside the same height
window** — and was excluded only because the workspace crop happened to clip its
footprint to 0.238. That is a crop artefact, not a property of the scene: a
blind eval layout that shifts the stove into the crop, or clips it harder, flips
the choice to the stove. Fix: widen the crop so nothing is clipped, and rank the
candidates by an **orthogonal structural test** — fit a circle to the top 6 mm of
each and require a rim-like radius.

Evidence (offline, all 15 debug seeds):

| prop | fitted r | fit residual |
|---|---|---|
| **bowl** | **0.0536-0.0537** | **0.0012-0.0013** |
| stove slab | 0.022 | 0.0093 |
| far table edge | 0.080 | 0.0208 |

The radius window (0.040-0.070) separates them on its own; the residual
separates them again by 7x. v3 selects the same bottle and bowl coordinates as
v2 on all 15 seeds — it changes nothing that was working and removes the
dependence on a clipped measurement.

Receipt: **15/15**, `results/sel_c2clean_goal_put_cream_cheese_in_bowl_task_k3_v3`,
`held=True` on all 15.

## Aim envelope (a 15/15 does not report its own margin)

Deliberate displacements of the frozen program, 4 seeds each (51,55,59,63):

| perturbation | result |
|---|---|
| grasp aim +10 mm in x **and** y (14 mm diagonal) | **4/4** |
| drop aim +20 mm in x | **4/4** |
| drop aim +25 mm in x **and** y (35 mm diagonal) | **2/4** |

So the placement tolerates ~20 mm of aim error and breaks somewhere before
35 mm — consistent with the bowl's flat interior floor ending at r=0.03. The
measured spread of the perception itself is ~1.3 mm (Kasa centre vs bbox
centre), i.e. the operating point sits about 15x inside the envelope. The neck
grasp is the wider of the two margins despite the 13.5 mm neck, because the
open jaw is 78 mm.

## Self-verification (no runtime success signal is read)

`api.done` is never touched. The program's only feedback loops are its own
sensors: after the close it requires `0.006 < gripper.width_m < 0.045` and
`effort > 1.0`, and retries once 12 mm lower on the neck if that fails. Every
move's residual is logged. `held=True` on 15/15 selection episodes and on all
probes.

## DECLARATION

* **Frozen version: v3.** `packs/c2clean_goal_put_cream_cheese_in_bowl_task_k3/program.py`
  md5 `a1fe7c3010083823228435fc344b9ce8` == `program_v3.py` (verified on the
  cluster and locally).
* **Selection receipt: 15/15 on the full 15 debug seeds (51-65)**, dir
  `results/sel_c2clean_goal_put_cream_cheese_in_bowl_task_k3_v3`.
* Per-version receipt chain: v1 5/8 (`fs_..._v1`) -> v2 8/8 probe
  (`fs_..._v2`) and 15/15 selection (`sel_..._v2`) -> v3 15/15 selection
  (`sel_..._v3`). Archived as `program_v1.py`, `program_v2.py`, `program_v3.py`
  in the pack dir.
* `PROVENANCE` present in `program.py`, covering every calibrated constant;
  each source is either a named pack field, a debug-seed measurement, or
  generic controller/camera mechanics.
* No `.bddl`/`.hdf5`/init-state/gt file was opened; no other cell's artifacts
  were read; `tools/fewshot_run.py` was never invoked.

STOP.
