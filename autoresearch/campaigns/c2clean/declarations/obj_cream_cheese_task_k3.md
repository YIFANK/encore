# c2clean / obj_cream_cheese_task_k3 — notes

Intent: **"Pick the alphabet soup and place it in the basket"** (re-authored cell: the
bddl path names cream cheese, the instruction names alphabet soup; the instruction is
authoritative and the benchmark bit confirmed it — see v1).

## Evidence read from the two packs

- `..._task_k3/pack.json` (language "pick up the cream cheese and place it in the
  basket"): scene = 2 cans (left column), 2 tall cartons (centre, far), 1 small flat blue
  box, 1 small orange box, basket at right. Demos close on the **small flat blue box** at
  EEF z = 0.009/0.010/0.010 → that is the cream cheese, i.e. the anti-target here.
  Releases over the basket at (x −0.02…0.06, y 0.26…0.27, z 0.15…0.23).
- `..._task_mate/pack.json` (language "pick up the alphabet soup and place it in the
  basket"): a *different* scene. Demos close at EEF z = 0.0447 / 0.0450 / 0.0454 on the
  **blue-bodied can with a grey lid** (keyframes demo0_t0042, demo2_t0046 show the jaws
  on it). That object is also present in my scene → identity + grasp height both come
  from this pack.

## Version chain

| version | what | receipt |
|---|---|---|
| probe_v0 | capture sanity: deprojection, camera matrices, table plane | `fs_..._p0` — table z ≈ 0.000–0.005; my vectorised cloud matches `f.deproject` to <4 mm. **api.log truncates a message at ~2000 chars**, so images must be chunked. |
| probe_v1 | 5 mm top-down height map + connected-component clusters + chunked RGB dump | `fs_..._p1` — 7 clusters: basket (0.00,0.26) top 0.144; arm (2 pieces, top 0.47–0.49); two cylinders top 0.081 at (−0.122,−0.243) and (0.099,−0.202); flat box top 0.020 at (0.05,−0.10); flat box top 0.019 at (0.15,0.03). Decoded image = the k3 pack scene. |
| probe_v2 | per-cluster colour split (top face vs body) on **all 15 debug seeds** | `fs_..._p2` — the two cans are at *identical* xy on every debug seed; only the basket and the two flat boxes jitter (≈±2 cm). Top-face colour does NOT separate the cans (both grey lids: [84,82,84] vs [95,97,100]) because an oblique view of a vertical cylinder collapses its body onto the near rim arc in a top-down height map. |
| — | offline identity check: project (cx,cy,0.45·top) into the image and sample | can A (−0.122,−0.243) body **[29,43,81] = blue** → alphabet soup; can B (0.099,−0.202) body [71,17,0] = red → not the target. Matches the mate pack's grasped can. |
| **v1** | perceive → blue-cue can → top-down grasp at top−0.036 → carry 0.24 → release over the perceived basket rim centre at 0.20 | probe `fs_..._v1` **8/8** (51,53,…,65); formal `sel_..._v1` **15/15**. Blue separation 0.501 vs 0.001. Grip after close 0.0625 m at effort 3.0 on every seed. |
| off15 / off25 | v1 with the grasp aim displaced +15 mm / +25 mm in **both** x and y (21 mm / 35 mm diagonal) | `fs_..._off15` **4/4**, `fs_..._off25` **0/4** → the aim envelope is ≈2 cm, against a perception repeatability of ~2 mm. Not a marginal cell. |

## Mechanism (v1)

1. `cam_high` depth → base-frame cloud → 5 mm top-down max-z height map over
   x[−0.40,0.30] × y[−0.42,0.45]; cells with z > 0.012 8-connected into clusters.
2. Candidate cans: top ∈ (0.040, 0.200) and footprint ∈ (0.030, 0.120) in both axes —
   this drops the flat boxes (top 0.019/0.020), the basket (0.160×0.175) and the arm
   (top 0.47+).
3. Identity: project the point (cx, cy, 0.45·top) — which lies on the visible near-side
   body of a vertical cylinder — into the RGB image, take a 7×7 mean, rank by
   B/(R+G+B). The blue can wins by 0.50 vs 0.00.
4. Grasp xy = the centroid of the cells within 10 mm of the cluster top (the lid disc;
   unbiased, unlike the whole-cluster centroid which leans toward the camera).
   Grasp z = top − 0.036, the mate pack's closing height against a measured top of 0.081.
5. Verify with the gripper: width 0.0625 m at effort 3.0 after the lift; a width below
   0.030 triggers one re-perceive-and-retry (never fired on any debug seed).
6. Basket = the cluster with top > 0.08 and footprint > 0.12 in both axes; release at
   its rim-ring centroid, carrying at z 0.24 (held can's base ≈0.195, clear of the
   0.144 rim) and opening at 0.20.

## DECLARATION

- **Frozen version: v1.** `packs/c2clean_obj_cream_cheese_task_k3/program.py`
  md5 `68f92e94eb121402324f355ebea5bf1e` == `program_v1.py` (same md5).
- **Selection receipt: 15/15** on the full debug band 51–65,
  `results/sel_c2clean_obj_cream_cheese_task_k3_v1` (15 of 15 records carry
  `"benchmark_success": true`).
- Probe receipts: `fs_..._v1` 8/8; envelope `fs_..._off15` 4/4, `fs_..._off25` 0/4.
- `PROVENANCE` is present in program.py and covers every calibrated constant
  (sources: the two named packs' keyframes/pack.json, debug-seed 51–65 measurements,
  generic camera/controller mechanics).
- No `api.done` read; no `fewshot_run.py`; writes confined to the pack dir and
  `results/*c2clean_obj_cream_cheese_task_k3*`.
