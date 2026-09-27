# c2clean — obj_chocolate_pudding_pos_k3

Intent: `pick up the chocolate pudding and place it in the basket`
Runner: `tools/fair_run.py` only. Pack: `packs/c2clean_obj_chocolate_pudding_pos_k3/` (K=3).

---

## v0 — perception probe (no motion)

**Hypothesis.** Nothing about the scene is known; dump cam_high RGB-D out through
`api.log` (zlib+base64, 1800-char chunks) and do the perception offline.

**Evidence.** 8 probe seeds (51,53,…,65), 72 sim steps each.
- `t_base_cam` = [[0,.529,-.849,.897],[1,0,0,0],[0,-.849,-.529,.65]] → image *u*
  maps to base **+y**, image *v* to base **+x**. Table plane at **z ≈ 0.001**.
- Six props + a basket. Height map (6 mm cells, max-z) gives exactly one **short**
  object: a small brown box, top **z = 0.030 m**, footprint x[−0.187,−0.109] ×
  y[0.033,0.081]. Every other prop (2 bottles, a can, a juice carton, a dressing
  bottle) tops out at 0.14–0.15 m; the basket rim at 0.143 m.
- **Per-seed RGB diff:** only the *basket* changes across the 8 seeds (centre
  x −0.012…+0.010, y 0.246…0.261, ≈±1.5 cm). The six props do not move at all —
  their short-band components are bit-identical on all 8 seeds.

**Verdict.** Layout is deterministic except the basket. Perceive both anyway.

## v0b — full-resolution crop

**Hypothesis.** 128-px pack keyframes cannot resolve labels; dump the native
512-px crop instead.

**Evidence.** The rear-left bottle carries a legible **"BBQ"** label. The brown
box at y≈+0.06 shows a dessert photo on its top face.

**Verdict.** The pack demos' grasp keyframes (t=54/48/53, ee xy ≈ (−0.115,
−0.254)) land on the **BBQ bottle**, not on anything pudding-shaped.

## The target conflict

Comparing `demo0_t0000` against a debug frame downsampled to 128 px: the two
scenes are the same layout, same six slots, **except the upper-right slot** —
demo holds a small brown *bottle*, the eval scene holds the brown *box*. So one
slot was swapped, and the pack's grasp (y = −0.25) is in the left cluster.
The pack demo difference (t0000 vs t0150) shows the BBQ bottle removed.

Two readings, both defensible from the pack alone: either the demo is valid and
the swap touched a bystander, or the demo names the wrong target and the
instruction ("chocolate pudding") means the brown box. Resolved by experiment,
not argument.

## v1 / v1b — A/B on the target identity

Identical mechanism (lifted from the demos: approach z 0.21 → descend to
z 0.010 → close → lift → traverse to the basket at z 0.26 → descend to
z 0.170 → open). Only the grasp xy differs.

| version | target | seeds 51,53,55,57 | closed gap |
|---|---|---|---|
| v1  | perceived short box (x −0.148, y +0.057) | **4/4** | 0.0458 m |
| v1b | pack demo grasp xy (−0.115, −0.254)      | **0/4** | 0.0010 m |

**Verdict.** Decisive. v1b closes on **empty air** (1 mm gap = nothing between the
jaws), so the demo's xy is not even a valid grasp in the eval scene. v1's held
gap 0.0458 m reproduces the pack's own carry-keyframe width (0.0465 m) — the
demo's *hold width* transfers even though its *xy* does not.
**The chocolate pudding is the short brown box; the pack demos name the wrong
object.** Instruction = target, pack = mechanism.

## v2p / v2m — aim margin (not a candidate, a measurement)

15/15 says nothing about how close to the edge the aim sits, so displace it.

| version | grasp aim | seeds 51,53,55,57 | closed gap |
|---|---|---|---|
| v2p | perceived y **+12 mm** | 4/4 | 0.0458 m |
| v2m | perceived y **−12 mm** | 4/4 | 0.0461 m |

**Verdict.** The close self-centres: a ±12 mm lateral error leaves the held gap
unchanged to 0.3 mm and does not cost a single episode. The aim is not near a
cliff; the 48 mm box in 78 mm jaws absorbs the error.

---

# DECLARATION

- **Frozen version:** `program_v1.py` → `program.py`,
  md5 `9c8b34f80afb7953159c0a406060e436` (both files, verified equal on cluster).
- **Selection receipt (full 15 debug seeds):** **15 / 15**
  `results/sel_c2clean_obj_chocolate_pudding_pos_k3_v1` (seeds 51–65, no failures).
- **Receipt chain:**
  - v0  `results/fs_…_k3_v0`  — perception probe, 8 seeds, no motion.
  - v0b `results/fs_…_k3_v0b` — full-res probe, 3 seeds, no motion.
  - v1  `results/fs_…_k3_v1`  — 4/4 (51,53,55,57).
  - v1b `results/fs_…_k3_v1b` — 0/4 (51,53,55,57), target-identity control.
  - v2p `results/fs_…_k3_v2p` — 4/4, aim +12 mm margin probe.
  - v2m `results/fs_…_k3_v2m` — 4/4, aim −12 mm margin probe.
  - sel `results/sel_…_k3_v1` — 15/15 formal selection.
- **PROVENANCE:** present in `program.py` as a top-level literal dict; every
  calibrated constant sourced either to a `pack.json` field (GRASP_Z, LIFT_Z,
  CARRY_Z, RELEASE_Z, HOLD_W) or to a debug-seed measurement (Z_TABLE,
  SHORT_BAND, OPEN_W, CAM).
- No LIBERO-specific prior knowledge was used; the short-object band, the table
  plane and the camera axes were all re-derived from the v0 probe.

STOP.
