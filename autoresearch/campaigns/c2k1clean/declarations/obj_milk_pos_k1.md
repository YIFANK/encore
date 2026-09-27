# c2k1clean / obj_milk_pos_k1 — NOTES

Task: `pick up the milk and place it in the basket` (LIBERO-object-swap, `_pos`
perturbation, K=1 pack). Runner: `tools/fair_run.py` only. No shared note file;
every constant below is sourced from the pack or from debug-seed (51–65)
measurements and is declared in `PROVENANCE`.

## Pack reading (packs/c2k1clean_obj_milk_pos_k1)

`pack.json` — 1 demo, 150 steps, stride 10, 4 keyframes.

| t | ee xyz | gripper_cmd | reading |
|---|--------|-------------|---------|
| 0 | (-0.158, -0.009, 0.256) | -1 (open) | home |
| 40 | (-0.099, -0.249, 0.149) | -1 | pre-grasp hover |
| 47/50 | (-0.138, -0.255, 0.102) | -1→+1 | **close on the carton at z=0.102** |
| 80–110 | z = 0.297…0.313 | +1 | transport plateau → **CARRY_Z = 0.31** |
| 142 | (-0.006, 0.245, 0.142) | +1→-1 | **release at z=0.142** |

Keyframe images (128×128) show 6 props + a wicker basket. The prop the gripper
covers at t=47 is a tall carton with a red-ish top; its patch mean is
(81, 45, 37) → **G−B = 8**. The other tall carton (bottom-centre) reads
(70, 49, 25) → **G−B = 24**. That gap is the identity cue.

## v0 — perception probe (no manipulation)

`results/fs_c2k1clean_obj_milk_pos_k1_v0`, seeds 51,53,…,65.

Measured from cam_high RGB-D (K = 618.04, principal point 256,256; camera sits
at base (0.897, 0, 0.65) looking down ~32°, so it is **axis-blind in x**):

- table plane z-mode = **0.0062 m** (identical on all 8 seeds)
- 8 above-table components: the arm (h=0.300) + 7 scene items
- two **tall cartons**, h = 0.135–0.137, footprint dx=0.027 dy=0.048:
  - G−B = 6, mean rgb (95,65,59)  ← matches the pack's grasped carton
  - G−B = 36, mean rgb (94,68,32) ← the other carton
- the **basket**: n≈11.2k px, dx=0.145 dy=0.153, rim ztop = 0.144
- every other prop: h ≤ 0.081, footprint ≤ 0.077

Cross-check on the non-tall props: the blue box (75,83,103), the brown prop
(103,63,43) and the can (62,57,48) sit at the *same pixels* as in the demo
image, and their patch colours match. Only the milk and the small dark box have
exchanged slots relative to the demo (this is the `_swap` bddl), so the demo's
grasp xy cannot be reused verbatim — the grasp must be re-perceived.

**Seed-to-seed variation on 51–65 is almost entirely the basket** (x 0.056→0.082,
y 0.247→0.267, ~3 cm); the milk jitters ±3 mm and the remaining props are
pixel-identical. Eval seeds 1–50 are unseen, so the program is written to be
fully position-agnostic rather than tuned to this spread.

Verdict: identity is decidable from (height, footprint, top-band G−B); geometry
is decidable from the cloud. Proceed to manipulation.

## v1 — perception-driven pick-and-place

Hypothesis: grasp the tall low-G−B carton at its top-slab xy centroid,
`z = ztop − 0.040` (the demo's closing height below the carton top), carry at
z = 0.31, release at the measured basket rim height over the basket's rim
centroid.

Probe receipt: `results/fs_c2k1clean_obj_milk_pos_k1_v1` — **8/8** on
51,53,…,65. ep51 detail: grasp residual 0.0104, gripper width 0.0534 with
**effort 3.00** (holding) through the lift and the whole transport; the
post-release re-capture shows the milk cluster gone from the table and the
basket cluster grown from 11.5k to 19.3k px (the carton now inside it).

Selection receipt: see DECLARATION below.

## Version receipt chain

| version | run dir | seeds | successes |
|---|---|---|---|
| v0 (perception probe, no motion) | `results/fs_c2k1clean_obj_milk_pos_k1_v0` | 51,53,…,65 (8) | 0/8 (probe only — measures the scene, never moves) |
| v1 (pick-and-place) | `results/fs_c2k1clean_obj_milk_pos_k1_v1` | 51,53,…,65 (8) | **8/8** |
| v1 (formal selection) | `results/sel_c2k1clean_obj_milk_pos_k1_v1` | 51–65 (all 15) | **15/15** |

Per-seed selection bits: 51✓ 52✓ 53✓ 54✓ 55✓ 56✓ 57✓ 58✓ 59✓ 60✓ 61✓ 62✓ 63✓
64✓ 65✓.

No further versions were needed: v1 is the first manipulation version and it
saturates the debug split, so there is nothing to argmax over and no
mechanism gap to report.

## DECLARATION

- **Frozen version: v1.**
  `packs/c2k1clean_obj_milk_pos_k1/program.py` md5 `fc5fc2f3fe20a70ec997eeab3a3734de`
  == `program_v1.py` md5 `fc5fc2f3fe20a70ec997eeab3a3734de`.
- **Selection receipt (full 15 debug seeds): 15/15**, dir
  `results/sel_c2k1clean_obj_milk_pos_k1_v1`.
- **Per-version receipt chain**: table above; every formally-probed version is
  archived as `program_vN.py` in the pack dir (v0 probe, v1 frozen).
- **PROVENANCE**: present as a top-level literal dict in `program.py`, 12
  entries, every one sourced from `pack.json` / `keyframes/demo0_t0000.png` /
  the v0 debug-seed measurements / the documented FairApi contract. No
  benchmark asset was read, no LIBERO-specific prior knowledge was used, and
  `api.done` is never referenced.
- Splits respected: only seeds 51–65 were ever run, always via
  `tools/fair_run.py --split debug`. `tools/fewshot_run.py` was never invoked.

STOP.
