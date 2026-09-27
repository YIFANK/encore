# c2clean / obj_milk_task_k3 — "Pick the butter and place it in the basket"

Runner: `tools/fair_run.py` only. bddl = pick_up_the_milk...bddl (opaque string),
language re-authored to the butter.

## Evidence from the two packs

- `..._k3` (language "pick up the milk and place it in the basket") — demos in
  THIS scene. Grasp at eef z ≈ 0.102/0.105/0.106, i.e. a TALL prop; place at
  (x≈0.0, y≈+0.25, z 0.14-0.19).
- `..._mate` (language "pick up the butter and place it in the basket") — demos
  in a DIFFERENT scene. Grasp at eef z = 0.0095 / 0.0100 / 0.0111 → the butter
  is a FLAT box. Carry altitude 0.26-0.33, release z 0.19-0.23 at y≈+0.25.

Identity chain for "butter" in my scene (no bddl, no asset names):
1. mate keyframe diff (t0 vs t150): the prop removed from the table is the
   small warm orange/red box; the second same-size box (dark brown) stays →
   anti-target.
2. that same dark-brown box also appears in my scene, so it is excluded.
3. my scene's only other flat box is cool/blue (top-band r−b ≈ −25) vs the
   warm one (r−b ≈ +56). Target = the warm flat box.

## Debug-seed scene (v0 perception probe, seeds 51/53/55/57)

cam_high: K f=618, principal (256,256); camera at base (0.897, 0, 0.65),
pitch such that base_y = cam_x. Table plane z ≈ 0.000-0.005.
Clusters (1.2 cm grid, z>0.012):

| cluster | centre (x,y) | top z | span | colour |
|---|---|---|---|---|
| basket | (+0.02..0.04, +0.25..0.27) | 0.144 | 0.16 × 0.17 | grey |
| arm (+2 fused props) | (−0.134, +0.006) | 0.350 | — | — |
| tall prop (OJ) | (+0.171, +0.029) | 0.143 | 0.045 × 0.053 | warm |
| tall prop (milk, k3 pack target) | (−0.10, −0.24) | 0.142 | 0.053 × 0.053 | warm |
| flat box (cool) | (+0.056, −0.10) | 0.020 | 0.080 × 0.041 | [72,79,97] |
| **flat box (warm) = butter** | (+0.105, −0.20) | 0.019 | 0.075 × 0.039 | [96,58,40] |

Layout is near-fixed across seeds (jitter ≤ 6 mm on props, ≤ 20 mm on the basket).

## Version log

- **v0** — perception probe, no motion. 0/4 (expected; no action). Gave the
  table plane, the camera model and the cluster table above.
- **v1** — perceive (flat + warm → target; largest footprint → basket), descend
  to z=0.010, close, lift to 0.285, traverse, release at 0.195. Probe 51/53/55/57.
- **v1** (formal) — probe 51/53/55/57 = **4/4** (`results/fs_c2clean_obj_milk_task_k3_v1`);
  full 15 debug seeds = **15/15** (`results/sel_c2clean_obj_milk_task_k3_v1`).
  Receipts per episode: gripper effort 3.0 + width ≈ 0.0388 m held from close
  through the carry; post-release re-perception shows the warm flat box absent
  from the table. The descent stalls at z ≈ 0.020 (commanded 0.010, residual
  0.0106) — contact with the box top, which is the grasp.
- **v2** (frozen) — v1 + a dormant occlusion-retry: if the first capture yields
  no basket or no warm flat cluster, retreat the arm to (-0.10, 0.30, 0.33) and
  re-perceive before giving up. The arm's start pose fuses with two tall props
  in every debug seed, so a target that landed under it would be invisible.
  Full 15 debug seeds = **15/15** (`results/sel_c2clean_obj_milk_task_k3_v2`);
  the retry never fired on the debug band (no PERCEPTION THIN lines), so the
  passing path is byte-identical in behaviour to v1.

## Predicate check

The instruction and the benchmark bit agree: v1/v2 move ONLY the warm flat box
(the milk carton and every other prop are untouched, confirmed by the post-run
cluster table) and every episode scores `benchmark_success: true`. This cell's
bddl grades the butter, not its filename's milk.

## DECLARATION

- **Frozen version: v2.** `packs/c2clean_obj_milk_task_k3/program.py`
  md5 `9b10b26450ea260302481a1a484689c2` == `program_v2.py`.
- **Selection receipt: 15/15 on the full debug band (seeds 51-65)**,
  `results/sel_c2clean_obj_milk_task_k3_v2` (no program errors).
- Receipt chain: v0 perception probe (0/4, no motion, by construction) →
  v1 4/4 probe → v1 15/15 formal → v2 15/15 formal (frozen).
- PROVENANCE present in program.py: R_DOWN, GRASP_Z, CARRY_Z, RELEASE_Z,
  TABLE_Z, WS_BOUNDS, FLAT_TOP_MAX, WARM_MIN, BASKET_MIN_SPAN, RETREAT, GRID_M —
  every one sourced to a named pack field or a seed-51..65 observation.
- Archived versions: program_v0.py, program_v1.py, program_v2.py in the pack dir.
