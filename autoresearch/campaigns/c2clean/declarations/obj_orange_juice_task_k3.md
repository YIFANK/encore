# c2clean / obj_orange_juice_task_k3 — working notes

Intent: **"Pick the chocolate pudding and place it in the basket"**.
Runner: `tools/fair_run.py` only. Debug split 51–65.

## Evidence read (packs, before any sim step)

| pack | language | grasp EEF (keyframes) | held gripper gap | release EEF |
|---|---|---|---|---|
| `..._task_k3` | *pick up the orange **juice** and place it in the basket* | (0.078,−0.106,0.120), (0.046,−0.109,0.093), (0.048,−0.109,0.098) | 0.0546 / 0.0533 / 0.0533 | z 0.147–0.184 over (≈0, +0.25) |
| `..._task_mate` | *pick up the **chocolate pudding** and place it in the basket* | (−0.110,−0.250,0.011), (−0.123,−0.263,0.009), (−0.111,−0.249,0.009) | 0.0465 / 0.0465 / 0.0466 | z 0.164–0.234 over (≈0, +0.25) |

Neither pack is my task: the k3 pack is my *bddl's* stock task (orange juice),
the mate pack performs my intent **in another scene**. The intent sentence is
authoritative, so the mate pack is my object evidence and the k3 pack is my
motion/scene evidence.

Projecting the mate grasp EEF into its own keyframe image
(`K` and `t_base_cam` recovered from my own debug capture, scaled to the pack's
128 px render) lands on pixel (31, 63) — a small **dark brown box**, mean RGB
`[0.245, 0.206, 0.194]`, chroma r = 0.380.

## Scene, from debug seeds only (v1, pure perception probe, 0 motion)

`api.log` used as a data pipe (zlib+base64 RGB-D), perception done offline.
`t_base_cam` = `[[0,.529,−.849,.897],[1,0,0,0],[0,−.849,−.529,.65]]`,
`K = diag(618,618)`, `c=(256,256)`. Table surface deprojects to **z = 0.0011**.

Seven props, identical on all four probed seeds except the basket, the amber
bottle and the left box:

| component | x | y | z_top | chroma r |
|---|---|---|---|---|
| basket | +0.030 | +0.255 | 0.141 | 0.339 |
| tan bottle | +0.165 | +0.029 | 0.148 | 0.406 |
| orange-juice carton | +0.069 | −0.099 | 0.138 | 0.489 |
| green-cap bottle | −0.186 | −0.080 | 0.147 | 0.324 |
| amber bottle | +0.109 | −0.207 | 0.113 | 0.631 |
| **dark box** | **−0.147** | **+0.059** | **0.0295** | **0.399** |
| red box | −0.114 | −0.242 | 0.0191 | 0.485 |

Only two props are short (0.019 / 0.0295); everything else tops out ≥ 0.113.
**Three independent cues agree that the dark box is the chocolate pudding:**
1. chroma r 0.399 vs the mate box's 0.380 (the red box is 0.485; mean RGB
   `[0.276,0.220,0.196]` vs the mate box's `[0.245,0.206,0.194]`);
2. its y footprint, 0.0475, matches the mate pack's held gap 0.0465; the red
   box is only 0.0388 across;
3. its top, 0.0295, puts the mate pack's grasp z (0.010) at mid-height; on the
   red box (top 0.0191) the same z would be 9 mm under the top.

## Versions

| v | hypothesis | run | result |
|---|---|---|---|
| v1 | perception only — what is on this table? | `fs_..._v1` (51,53,55,57) | 0/4 by construction; produced the table above |
| v2 | full component seg, target = least-red short component; grasp z = top − 0.019; release over the basket rim-bbox centre | `fs_..._v2` (51,53,55,57) → **4/4**; `fs_..._v2b` (other 11) → **11/11** | closes to 0.0458 m at effort 3.0 — the mate pack's 0.0465 — and the box is gone from the post-place capture |
| v3 | harden by segmenting the low band first and rejecting candidates with tall pixels over their footprint | offline replay of the v1 frames | **rejected, never run**: the robot column shares the pudding's xy footprint, so the real target was filtered out and the basket's own base won on chroma. Kept as a negative result. |
| v4 | v2's rule kept as primary; wider workspace crop; low-band pass demoted to a fallback used only when no short component exists; one re-grasp if the hand comes up empty | offline replay reproduces v2's target/basket to ≤2 mm; formal 15 → see DECLARATION | frozen |

### Aim envelope (v4 with a deliberate bias on the closing axis, seeds 51,55,59,63)

| y-bias | −0.020 | −0.012 | +0.012 | +0.020 |
|---|---|---|---|---|
| success | 4/4 | 4/4 | 4/4 | 4/4 |

The straddle self-centres: the box is 76 mm long across the jaws' open axis and
the jaws open to 78 mm, so ±20 mm of aim error is still absorbed. Perception
error on unseen eval layouts has a wide margin to work in.

### Caveat carried into eval

Every debug seed places the pudding at exactly (−0.1467, +0.0592) — the debug
split does not perturb it at all. The program is fully perceptual (no pose is
hard-coded), and the ±20 mm envelope above is the only margin evidence I could
get; the identity rule itself has been exercised against one layout only.

---

# DECLARATION

**Frozen version: v4.**
`packs/c2clean_obj_orange_juice_task_k3/program.py`
md5 `a0dcaa714847557807d186e234b5abdc`
== `packs/c2clean_obj_orange_juice_task_k3/program_v4.py` (same md5, verified on
the cluster).

**Selection receipt (full 15 debug seeds, one formal run):**
`results/sel_c2clean_obj_orange_juice_task_k3_v4` — **15/15**
(seeds 51–65, `benchmark_success: true` on every episode;
runner line: `[fair] results/sel_c2clean_obj_orange_juice_task_k3_v4: 15/15`).

**Receipt chain**

| run dir | version | seeds | result |
|---|---|---|---|
| `fs_c2clean_obj_orange_juice_task_k3_v1` | v1 (perception probe, no motion) | 51,53,55,57 | 0/4 (by construction) |
| `fs_c2clean_obj_orange_juice_task_k3_v2` | v2 | 51,53,55,57 | 4/4 |
| `fs_c2clean_obj_orange_juice_task_k3_v2b` | v2 | 52,54,56,58–65 | 11/11 |
| — | v3 | — | rejected in offline replay, never run |
| `fs_c2clean_obj_orange_juice_task_k3_aim_ym20` | v4, y−0.020 | 51,55,59,63 | 4/4 |
| `fs_c2clean_obj_orange_juice_task_k3_aim_ym12` | v4, y−0.012 | 51,55,59,63 | 4/4 |
| `fs_c2clean_obj_orange_juice_task_k3_aim_yp12` | v4, y+0.012 | 51,55,59,63 | 4/4 |
| `fs_c2clean_obj_orange_juice_task_k3_aim_yp20` | v4, y+0.020 | 51,55,59,63 | 4/4 |
| **`sel_c2clean_obj_orange_juice_task_k3_v4`** | **v4 (frozen)** | **51–65** | **15/15** |

**PROVENANCE**: present as a top-level literal dict in `program.py`, covering
`R_DOWN`, `Z_FLOOR`, `Z_LOW_BAND`, `Z_TALL`, `GRASP_BELOW_TOP`, `MIN_PIX`,
`BASKET_MIN_PIX`, `CARRY_Z`, `RELEASE_Z`, `OPEN_W`, `WS`. Every source is either
a named pack's keyframe field or a debug-seed (51–65) measurement.

No mechanism gap. STOP.
