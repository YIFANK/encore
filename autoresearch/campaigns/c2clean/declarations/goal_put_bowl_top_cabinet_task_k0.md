# c2clean / goal_put_bowl_top_cabinet_task_k0 — dev log

Intent: **"Put the plate on the top of the drawer"**. K=0: no demonstrations, no
note file. Everything below was measured on debug seeds 51–65 only.

## Scene, as measured (v1, cam_high RGB-D dumped through api.log)

Deprojection convention checked against `f.deproject` in-episode (agreement to
1e-4 on well-conditioned pixels), so the whole point cloud can be built once per
capture instead of pixel by pixel.

| thing | measurement |
|---|---|
| table plane | z = 0.901 |
| plate (target) | circular component, bbox 0.136 × 0.136, crest z = 0.920, floor 0.9079, centre x∈[0.039,0.063] y∈[-0.036,-0.007] over 51–65 |
| plate radial profile | surface radius 0.0524@0.9075 → 0.0687@0.9175, i.e. a ~31° interior cone; nothing outboard of 0.0687 |
| metal bowl | centre ≈(-0.082,-0.005), rim r 0.055, top 0.951 |
| cabinet top | dominant flat plane z = **1.127**, x[-0.12,0.16], y[-0.36,-0.157] (clipped at the image edge) |
| cabinet front | vertical wall at y = -0.1574 over the plate's height band; drawer fronts/handles stand out to y = -0.126 above z = 0.925 |
| fingertips | 0.0095 below the eef reference (v7: closed jaws stall on the bare table at eef_z = 0.9105) |
| jaws | open width 0.0775; `api.grip` is binary (no intermediate width) |

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1/v1b | dump RGB-D for all 15 debug seeds | `api.log` truncates a message at ~2000 chars — chunk at 1900 | scene mapped |
| v2 | straight rim pinch, `rotation=None` | eef froze at (0.083,-0.059,1.064), horizon burnt | **`rotation=None` does NOT hold the wrist down** — the hand flops and the arm jams; every move must pass an explicit R |
| v3 | budget probe | 22 moves + 1 capture + 2 grips = 327 sim steps ⇒ ~12 steps/move, ~35/capture, horizon 1000 | budget model |
| v4/v5 | re-issued moves | same freeze | confirms v2 diagnosis, not starvation |
| v6 | explicit R_DOWN on every move, approach from +y | full trajectory ran, residuals ~0.01 | **fix**; grasp closed on air (tip offset wrong) |
| v7 | fingertip probe | first goto stalled at x=0.137 and sank to eef_z 0.9105 | TIP_OFFSET = 0.0095; also: +x reach limit near 0.14 at table height |
| v8 | rim pinch at tips 0.9095 | closed w = 0.0190, eff 3.0 — then w = 0.0014 after the lift | brim is bitten but lost |
| v9 | **control: carry the BOWL to the cabinet top** | bowl grasped, carried, placed (image confirms), `benchmark_success` **false** on 51 & 54 | the graded object is the plate, not the bowl; also validates the drop point |
| v10 | staged lift | w decays 0.0190→0.0181→0.0141→0.0091→0.0051→0.0037 over a 0.315 m lift; plate still on the table in the wrist view | the brim ratchets out of the jaws |
| v11 | tilted (30°) undercut grasp | closed w = 0.0010 on all three variants | nothing to hook |
| v12/v13 | slide the closed jaws in at 5 mm, then 1 mm above the table | jaws never stall; the plate is pushed 29–33 mm | **no undercut**: the plate cannot be hooked from below |
| v14 | bite sweep (inset 0.0687/0.0655/0.0630) | closed w 0.0010 / 0.0043 / 0.0112, all lost by +0.06 m | the widest bite is the deepest one |
| v15/v16 | pin the plate against the cabinet face and push / wedge at table level | the plate jams, the eef stalls, ztop stays 0.9197 | the plate will not tip up by pushing |
| v17 | **arc lift**: the gripped rim point can only travel on a circle about the far edge while that edge is still on the table | held to h = 0.113 (w = 0.0053, eff 3.0), lost on the last 28 mm step | mechanism found |
| v18 | finer arc + staged hoist | held through the arc and a 0.09 m hoist, lost at move 13 after closing | the plate hangs vertical, clamped on the 5 mm shell slab |
| v19 | 2 cm creep everywhere | lost at move 15 after closing, same z | the loss is **per sim step, not per metre**: the jaws keep squeezing and the wedge-shaped rim creeps out |
| v20 | do the job in 9 moves | ep51 **success**, ep54 lost it in the arc | speed is the lever |
| v21 | 9 moves, inset 0.060 | 6/6 probe; formal **13/15** (56, 62 = PLATE NOT FOUND) | mechanics solid, perception brittle |
| v22 | as v21, inset 0.055 / tips 0.9090 | 6/6 probe; formal **13/15**, same two seeds | bite depth is not the limiter |
| **v23** | footprint height test over the inner 0.75 r disc instead of the bbox (on 56/62 the bowl fell inside the plate's bbox and vetoed it) | formal **15/15** | **frozen** |

## The mechanism, in one paragraph

The plate is a 0.137 m dish, 0.019 m tall, with a ~31° interior cone and no
undercut. It is wider than the jaws open (0.0775), so it cannot be straddled;
a radial rim pinch bites the 0.019 m brim but the inner jaw rides a 31° cone,
which pushes the plate down and out, so a vertical lift ratchets it free. The
plate can, however, be *tipped*: while its far edge is on the table it can only
pivot about that edge, so the gripped rim point must travel on a circle of
radius D = inset + r about it. Following that circle lifts the near edge until
the plate hangs vertical, at which point the jaws are clamping the 5 mm shell
slab instead of the brim wedge — a stable hold. The jaws never stop squeezing,
so that hold has a fixed lifetime (~13–15 moves after closing); the program
therefore spends nine: four arc steps, one hoist, one carry, three to run the
arc backwards and lay the plate flat on the cabinet top, then release.

## DECLARATION

* **Frozen version: v23.** `packs/c2clean_goal_put_bowl_top_cabinet_task_k0/program.py`
  md5 `504f499d41fdfacc02b884f5414d45bb` == `program_v23.py` == the
  `program_archived.py` of the selection run.
* **Selection receipt: 15/15** on the full debug band 51–65,
  `results/sel_c2clean_goal_put_bowl_top_cabinet_task_k0_v23`.
* Runner-up formals: v21 13/15 (`sel_..._v21`), v22 13/15 (`sel_..._v22`);
  both failed only on the perception veto fixed in v23.
* Every formally probed version is archived as `program_vN.py` in the pack dir.
* `PROVENANCE` is present as a top-level literal dict covering TABLE_Z,
  PLATE_TOP, PLATE_DIAM, PLATE_DISC_TEST, GRASP_INSET, GRASP_TIPS, TIP_OFFSET,
  ARC_D (the arc schedule ARC_H/HANG_H are sample points on that circle),
  CAB_TOP_Z, DROP_XY, PLATE_SIT_OFF, R_DOWN. Every source is a debug-seed
  measurement or generic controller mechanics.
* `api.done` is never read; success was judged only from `results.jsonl` after
  each run.
