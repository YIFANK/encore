# c2clean / obj_ketchup_pos_k3 — working notes

Task: "Pick up the ketchup and place it in the basket." (libero_object_swap)
Runner: `tools/fair_run.py` only. Debug band = seeds 51–65.

## Pack reading (v0 input, no sim)

`pack.json`: K=3 demos, keyframes at every gripper transition, `ee_path6`
(stride 10, xyz + euler), raw actions with `action_scale`.

Gripper transitions (action[-1] sign flips):
- demo0: close t=53, **open t=69** (failed grasp), close t=136, open t=229
- demo1: close t=49, open t=148
- demo2: close t=42, open t=120

`ee_path6` at those instants:
- demo1 close ≈ (-0.11, -0.25, **0.113**); demo2 close ≈ (-0.125, -0.25, **0.112**)
- demo0's first (failed) close was at z=0.109 and its recovery close at
  z≈0.013 — i.e. after it had knocked the bottle over. The two clean demos
  agree on a **0.112 m grasp altitude**.
- releases: (0.092, 0.223, 0.176), (0.020, 0.241, 0.176), (-0.027, 0.246, 0.212)

Keyframe images (128×128) show one fixed layout in all three demos: five
upright props + a flat box on the left/centre and a wicker basket on the right.
The grasped one is the **left-slot bottle: grey/silver cap over a red-orange
labelled body**.

## v0 — perception probe (all 15 debug seeds)

Program: dump cam_high/cam_arm_wrist RGB-D through `api.log`
(zlib+base64, 256×256 subsample); no motion. 72 sim steps/episode.

Receipts (`results/fs_..._v0`, `..._v0b`):
- cam_high: K = fx=fy=618.04, c=(256,256); `t_base_cam` puts the camera at
  (0.897, 0, 0.65) looking back and down. **Table plane z ≈ 0.0005**.
- **The demo xy anchor is a decoy.** The runtime scenes are the *same slots*
  with a *different assignment*: the pack's left slot (-0.11,-0.25) holds a
  GREEN-capped bottle at runtime, and the grey-cap/red-orange bottle sits at
  (-0.145, +0.061). Identity has to come from appearance, not position.
- z-profile of the target (ep51): base 0.063 m wide and warm-dark, shoulder
  tapering 0.050→0.042 at z=0.07–0.10 with the saturated red (88,42,21), then
  a **grey neck/cap 0.030–0.034 m wide from z≈0.105 to the top at 0.147**.
  The demos' 0.112 close is therefore a *cap* grasp, 0.035 below the top.
- Scene inventory, stable across all 15 debug seeds (cap = top-20 mm mean RGB,
  body = 35–60 mm below top, grey = cap max−min, warm = body R−B):

  | component | top | cap colour | grey | warm | score = warm−3·grey |
  |---|---|---|---|---|---|
  | basket (n≈11k) | 0.142 | (167,167,168) | 0.7 | 6 | — (rim span 0.17 m) |
  | can | 0.081 | (76,78,88) | 11.9 | −3.5 | −39 |
  | **ketchup** | **0.148** | (96,94,94) | **1.5** | **55.1** | **+50.6** |
  | green bottle | 0.148 | (22,52,34) | 29.8 | −2.2 | −92 |
  | red box | 0.140 | (134,100,91) | 42.5 | 53.5 | −74 |
  | dark bottle | 0.113 | (73,22,3) | 70.4 | 44.2 | −167 |

  Every distractor either has a coloured cap or a cold body; the margin from
  the runner-up is 124 score points, so this is a rank, not a threshold.
- Caveat honestly recorded: the **debug band does not exercise the
  permutation** — the ketchup sits at (-0.145,+0.061) on all 15 seeds and only
  the basket, the red box and the dark bottle jitter by 1–2 cm. The identity
  rule is validated against the *pack* (which shows a different assignment),
  not against seed-to-seed motion inside the band.

## v1 — perceive, cap-grasp, place

Mechanism: cam_high cloud → z>0.02 mask → 4-connected labelling (half-res,
membership expanded back to full res) → per-component top/cap/body features →
rank by warm−3·grey among prop-sized components → grasp xy from the top-band
bbox midpoint, z = top − 0.035 → lift to 0.32 → basket rim-band bbox centre →
descend to 0.21 → open.

Receipt: `results/fs_c2clean_obj_ketchup_pos_k3_v1` — **8/8** on seeds
51,53,55,57,59,61,63,65; 140–142 sim steps. Closed finger gap 0.0336 on every
seed = the cap diameter, so the grasp is on the cap as designed, not a lucky
body pinch.

## Reach probe (v2probe, `--horizon 8000`, seed 51)

10×11 xy raster at z=0.112 (the grasp altitude), residual + achieved eef
logged. Residual is 0.006–0.012 (the controller's own POS_TOL) **everywhere**
in x∈[-0.26,0.10], y∈[-0.30,0.24] — the whole region any slot occupies. Only
x=0.14 overshoots (0.018–0.024), and no slot is that far forward. Verdict: no
reach hole anywhere the target could be permuted to, so a wrong-slot eval seed
is a perception risk, not a kinematics risk.

## v2 — v1 + sensor-verified retries

Added, on top of v1's mechanism (no change to perception or geometry):
- `goto()` re-issues a move once when the reported residual exceeds 0.016 (a
  blocked descent stops short and returns silently).
- after the close *and* again after the lift, the finger gap is checked
  (`effort>1` and `gap>0.010`); a failure clears, **parks at the pose
  perception was validated at**, re-perceives, and picks again, up to 3 picks.
- no runtime success signal is consulted: the only evidence is the finger gap,
  the move residual, and re-perception.

Retry path exercised deliberately (`program_v2t`, first close forced 0.10 m
off in y): `results/fs_..._v2t` seeds 51,53 — the air-close is detected
(gap 0.0010), the arm clears/parks/re-perceives, the second pick holds
(gap 0.0336), both episodes succeed. 271 sim steps.

Selection receipt: see DECLARATION below.

## Version receipt chain

| version | what changed | run | receipt |
|---|---|---|---|
| v0 | perception probe only (RGB-D dump, no motion) | `fs_..._v0` (51,53,55,57), `fs_..._v0b` (52,54,56,58–65) | 0/15 by construction; produced the scene inventory + identity rule above |
| v2probe | reach raster at z=0.112, `--horizon 8000` | `fs_..._v2probe` (51) | residual ≤ 0.012 across the whole slot region |
| v1 | perceive → cap-grasp → place | `fs_..._v1` | **8/8** on 51,53,55,57,59,61,63,65 |
| v2t | v2 with the first close forced 0.10 m off (retry-path test) | `fs_..._v2t` | 2/2; air-close detected and recovered |
| **v2** | v1 + residual re-issue + finger-gap-verified re-pick | `sel_..._v2` | **15/15** on 51–65 |

## DECLARATION

- **Frozen version: v2.** `packs/c2clean_obj_ketchup_pos_k3/program.py`
  md5 `3a5ca73841662591044dae9c76ea621d` == `program_v2.py` (same md5).
- **Selection receipt: 15/15** on the full debug band (seeds 51–65),
  directory `results/sel_c2clean_obj_ketchup_pos_k3_v2`. Every episode
  reported a closed finger gap of 0.0336 m (the cap diameter) and 140–144
  sim steps; no episode needed a retry.
- **PROVENANCE present** in program.py: 17 entries, every calibrated constant
  sourced either to a `pack.json` field (`CAP_GRASP_DROP`, `HOVER_Z`,
  `CARRY_Z`, `DROP_Z`) or to a debug-seed measurement (table plane, workspace
  bounds, component pixel counts, cap widths, colour bands, move tolerance).
  No constant comes from prior context; the identity rule, the grasp altitude
  and the basket centre were all re-derived in this cell.
- **Known residual risk, stated honestly:** the debug band holds the target at
  one slot on all 15 seeds, so the band cannot demonstrate the appearance rule
  surviving the permutation. The evidence that it does is (a) the pack's own
  layout puts the target at a *different* slot than every debug seed and the
  rule picks it correctly there, and (b) the 124-point score margin over the
  runner-up on every debug seed. The reach raster rules out kinematics as a
  failure mode at any slot.

STOP.
