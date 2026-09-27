# c2clean / obj_milk_pos_k3 — "pick up the milk and place it in the basket"

Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_obj_milk_pos_k3/` (K=3).
Debug seeds 51–65 only; eval seeds 1–50 never touched.

## The problem the pack sets

All three demos close the gripper at essentially one point —
(-0.133,-0.252,0.100), (-0.114,-0.238,0.105), (-0.122,-0.248,0.098) — and
release over (-0.006,0.245) / (-0.028,0.271) / (0.011,0.266). Tight cluster,
4.5x margin: a textbook demo anchor.

It is a decoy. Projecting the demo grasp keyframe into the pack's *own*
agentview keyframe with the pack's camera pose (fx 618·128/512, t_base_cam from
a debug capture) lands on pixel (31,54) of `demo0_t0047.png` — the tall
red/white carton at the far-left of the demo scene. On every debug seed that
same base-frame xy holds a **flat brown box**: the `_pos` perturbation permutes
which prop sits where. Aiming at the demo xy would have grasped the wrong
object with a clean-looking receipt.

What does transfer from the pack: the target's **appearance** (tall red/white
carton) and the **heights** (grasp eef z ≈ 0.10, release eef z ≈ 0.14–0.17).

## Version chain

| ver | what | receipt |
|---|---|---|
| v1 | log-as-sensor probe: RGB/depth/K/T through `api.log` | api.log truncates at ~2000 chars — image dumps arrive clipped. Confirmed OpenCV camera convention (cv z-range 0–0.49; OpenGL gave z 0.81–1.30), table plane z = 0.000 |
| v2 | chunked base64 dump, 15 debug seeds, no motion | All 15 seeds share **one** prop layout; only the basket jitters (x 0.009–0.036, y 0.244–0.273). At the home pose the arm is a tall structure covering x∈(-0.19,-0.07), y∈(-0.12,0.12) — exactly the far prop row — and fuses two props into a 3798-pt blob |
| v3 | park the arm at two retreat poses, then capture | Residuals 0.010 / 0.024. Clean scene: 6 props + basket resolve as 7 clusters. Measured the two cartons: footprint 0.028 (x) × 0.051 (y), top 0.137–0.142; tomato can top 0.081; flat boxes 0.019–0.029; basket rim 0.143, footprint 0.15×0.18 |
| v4 | full pipeline, cluster band z∈(0.008,0.30) | probe 8/8, **formal 15/15** (`results/sel_c2clean_obj_milk_pos_k3_v4`). Closed gap 0.053 on every seed = the carton's own measured y-width; effort 3.0 survived the lift and the carry. Two fragilities visible in the logs: the parked arm fused into the basket cluster (ztop 0.278 instead of 0.143) and the 0.081-tall can sat only 0.024 below the band's prop cut |
| v5 | band-mask **before** grouping (z∈0.090–0.200) + verified-hold retry | probe 8/8, **formal 15/15** (`results/sel_c2clean_obj_milk_pos_k3_v5`). Exactly 3 clusters survive the band — milk, juice, basket — with no arm and no can. **FROZEN** |

Aim-envelope probes (not in the chain, `probe_envp.py` / `probe_envm.py` =
v5 with the grasp y displaced ±0.012): **4/4 and 4/4** on seeds 51,55,59,63.
So the grasp tolerates at least ±12 mm of lateral aim error, matching the
geometric bound (0.078 opening − 0.051 object)/2 = ±13.5 mm.

## Mechanism

1. **Park.** Two moves to (-0.10,0.40,0.38) then (0.10,0.42,0.30). At the home
   pose the arm occludes the far prop row; parked, it is also above the
   perception band and cannot contaminate a cluster.
2. **Band-mask, then group.** Keep only 0.090 < z < 0.200. That single cut
   deletes the flat boxes (≤0.029), the tomato can (0.081) and the parked arm,
   so connected-component grouping in xy cannot fuse a short prop into a
   carton. Three clusters survive.
3. **Split by footprint.** max(dx,dy) < 0.10 → the two cartons;
   min(dx,dy) ≥ 0.10 → the basket.
4. **Split the carton tie by colour.** The two cartons are geometric twins
   (same footprint to 1 mm, same height to 2 mm), so shape cannot separate
   them. The juice carton's body is yellow/orange; the milk's is red on white.
   Rank by B−G: milk −10.8…−11.7, juice −38.8 on all 15 seeds — a 27-unit gap
   against ~1 unit of within-seed spread.
5. **Grasp** at the cluster's bbox mid, z = ztop − 0.040 (the demos' 0.098–0.105
   against a 0.139 top). Default straight-down wrist: the fingers close along
   world y, across the carton's 0.051 width, inside a 0.078 opening.
6. **Verify** the hold: gap > 0.020 *and* effort 3.0 after the lift, not just
   after the close. On all 15 seeds the gap came out 0.0530–0.0533 — the
   carton's own width — so zero retries fired.
7. **Place** at the perceived basket bbox mid, descend to eef z = 0.17 (demo
   release 0.142–0.171), open. A post-episode capture shows the milk cluster
   gone from the table and the basket cluster's top risen 0.142 → ~0.195: the
   carton standing inside the basket.

## What the cell is evidence for

Verification is what makes the demo anchor safe to discard. The anchor here is
tight enough (three demos within 2 cm) to look authoritative, and following it
would have produced a confident grasp on a brown box. The thing that made the
difference was a receipt the anchor cannot fake: the **closed gap equals the
target's independently measured width**. That test is what licensed replacing
the pack's xy with perception while still using the pack for what it does carry
— which object, and at what height.

## DECLARATION

- **Frozen version: v5.** `packs/c2clean_obj_milk_pos_k3/program.py`
  md5 `94ada1b3ad3e42ff3cc8eca5c16ae676` == `program_v5.py`
  md5 `94ada1b3ad3e42ff3cc8eca5c16ae676`.
- **Selection receipt: 15/15 on the full 15 debug seeds (51–65)**, dir
  `results/sel_c2clean_obj_milk_pos_k3_v5`.
- Per-version receipt chain: v1–v3 perception probes (no acting version);
  v4 probe 8/8, formal 15/15 (`results/sel_c2clean_obj_milk_pos_k3_v4`);
  v5 probe 8/8 (`results/fs_c2clean_obj_milk_pos_k3_v5`), formal 15/15.
  Envelope probes ±12 mm: 4/4 and 4/4.
- v5 selected over v4 (both 15/15) on robustness, not score: band-before-group
  removes the arm/can fusion risk that v4's logs showed, and v5 adds the
  verified-hold retry.
- `PROVENANCE` present in program.py, covering every calibrated constant;
  every constant sourced to this pack's keyframes/ee_path6 or to debug-seed
  (51–65) measurements.
- No `tools/fewshot_run.py`. No forbidden reads. No `api.done`. Writes confined
  to `packs/c2clean_obj_milk_pos_k3/*` and `results/*c2clean_obj_milk_pos_k3*`.

STOP.
