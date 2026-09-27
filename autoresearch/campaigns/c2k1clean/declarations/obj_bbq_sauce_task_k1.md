# c2k1clean / obj_bbq_sauce_task_k1 — working notes

Intent: **"Pick the ketchup and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed (debug = 51–65; eval 1–50 never touched).

---

## Reading the two packs

Neither pack's language is the intent, but between them they name both halves:

| pack | language | what it gives me |
|---|---|---|
| `c2k1clean_obj_bbq_sauce_task_k1` | *pick up the bbq sauce and place it in the basket* | the **target** (basket) — its scene is MY scene, and its release keyframe is the drop geometry |
| `c2k1clean_obj_bbq_sauce_task_mate` | *pick up the ketchup and place it in the basket* | the **object** (ketchup) — a different scene, but it shows me what the ketchup looks like |

**Naming the ketchup from the packs alone.** Diffing each pack's first and last
keyframe says which prop each demo removed:

* k1 pack (my scene): the prop that disappears is the small dark bottle at
  128-px (48,73) — body RGB **(57.0, 33.6, 12.1)**. That is the bbq sauce, i.e.
  the **anti-target**.
* mate pack: the prop that disappears is an amber squeeze bottle with a pale
  grey cap at 128-px (32,57) — body RGB **(73.6, 48.0, 32.6)**, cap
  **(93.3, 88.6, 85.3)**. That is the ketchup.

The mate pack's ketchup has a twin in my own scene: k1 128-px (72,55), body
**(71.4, 48.9, 34.3)**, cap **(74.3, 70.7, 68.4)**. Same object. Cross-pack
agreement to ~2 RGB counts is what licenses the constant.

The bbq sauce measures (57,34,11) in *both* packs, so the two warm props are
separable: the ketchup is the brighter, less-saturated one (B/R 0.46 vs 0.19).

---

## Version log

### v0 — perception probe (no motion). seeds 51,53,55,57
**Hypothesis:** the scene can be read from `cam_high` RGB-D alone.
**Evidence** (`results/fs_c2k1clean_obj_bbq_sauce_task_k1_v0`, 4 eps, 72 sim steps each):
* `K = diag(618.04, 618.04)`, principal point (256,256); camera at base
  (0.897, 0, 0.65) looking along (−0.849, 0, −0.529) — an oblique down-forward
  view, so **image u ↔ base y** (crisp) and image v mixes x and z (depth-smeared).
* Table plane **z = 0.0011** (33.6 k inliers), identical on every seed.
* 7 clusters; one is the arm (h = 0.4455 — every prop is ≤ 0.147).
* **Reprojection check:** every cluster centroid lands on the pack keyframe's
  prop at 4× scale — ketchup 128(72,55)→512(288,220) vs measured (287,223);
  bbq 128(48,73)→(192,292) vs (196,290); basket (420,280) vs (415,264). So the
  cluster table and the pack images are the same scene, and the identification
  above transfers.
* Debug-seed randomisation is small jitter and moves only the basket
  (±2 cm), the bbq sauce (±1 cm) and the blue box; the ketchup, green bottle
  and blue can are bit-identical across 51/53/55/57.

**Verdict:** perception is solid; go to a full pipeline.
*(A sampling bug: v0's "upper 60 %" colour window read the ketchup's grey cap,
not its body — fixed in v1 with a 20–70 % body band plus a separate cap band.)*

### v1 — identify + pick + place. seeds 51,53,55,57 → **4/4**
**Hypothesis:** rank props by RGB distance to the mate pack's ketchup
(`body + 0.5·cap`), grasp the neck, release at the k1 demo's drop height.
**Evidence** (`results/fs_c2k1clean_obj_bbq_sauce_task_k1_v1`, 164 sim steps):
* Ranking margin is wide — ketchup score 25.3, runner-up (blue can) 43.0,
  bbq sauce 80.5, basket 197.0.
* Height profile of the ketchup: y-width falls 0.063 (base) → 0.042 (z≈0.10)
  → 0.030 (cap). Grasping 0.039 below the top (the k1 demo's own grasp offset,
  read off keyframe t=54) lands on the neck; the jaws closed to **0.0334 m**
  with **effort 3.0** on the first try, every seed — no ladder retry needed.
* Descending to a *neck* grasp is collision-free because the neck is above the
  widest part of the bottle.
**Verdict:** works; one weakness found — the basket aim point.

### v2 — rim-centre basket aim. seeds 51…65 odd → **8/8**
**Hypothesis:** v1 aimed with the *median* of the basket's top-1.5 cm band,
which is biased to the far rim (x = 0.018 against a 0.074 body centroid on
seed 51) because the camera looks down-forward and exposes more of the far
rim's top face. The **midpoint of that band's x/y range** is the opening centre.
**Evidence** (`results/fs_c2k1clean_obj_bbq_sauce_task_k1_v2`): rim-midrange and
whole-cluster midrange agree exactly on all 8 seeds (e.g. seed 51 both
(0.005, 0.264)), confirming the estimate is the geometric centre of a
0.159 × 0.170 footprint rather than a visibility artefact. 8/8.
**Verdict:** selected.

---

## DECLARATION

* **Frozen version: v2.**
  `packs/c2k1clean_obj_bbq_sauce_task_k1/program.py`
  md5 `2519c8a1374a0374c1527960da9750fa` == `program_v2.py`
  md5 `2519c8a1374a0374c1527960da9750fa`. ✔

* **Selection receipt (full 15 debug seeds, one formal run):**
  **15/15** — `results/sel_c2k1clean_obj_bbq_sauce_task_k1_v2`
  succeeded on 51,52,53,54,55,56,57,58,59,60,61,62,63,64,65; no failures.
  Peak cost 163 sim steps of the 500-step horizon.

* **Receipt chain:**

  | version | seeds | result | dir |
  |---|---|---|---|
  | v0 | 51,53,55,57 | perception probe, no motion (0/4 by construction) | `results/fs_c2k1clean_obj_bbq_sauce_task_k1_v0` |
  | v1 | 51,53,55,57 | **4/4** | `results/fs_c2k1clean_obj_bbq_sauce_task_k1_v1` |
  | v2 | 51,53,55,57,59,61,63,65 | **8/8** | `results/fs_c2k1clean_obj_bbq_sauce_task_k1_v2` |
  | **v2 (formal)** | 51–65 (all 15) | **15/15** | `results/sel_c2k1clean_obj_bbq_sauce_task_k1_v2` |

* **PROVENANCE:** present as a top-level literal dict in `program.py`, 12 keys —
  `KETCHUP_BODY_RGB`, `KETCHUP_CAP_RGB` (both packs' keyframes),
  `REL_DZ`, `GRASP_FRAC` (k1 pack ee_path6 / keyframe t=54),
  `TABLE_BAND`, `ARM_H`, `WS_X`, `WS_Y`, `SAFE_Z`, `OPEN_W`, `BASKET_AIM`
  (debug-seed measurements), `HOLD_EFFORT` (FairApi contract).
  No LIBERO-specific prior knowledge was used; every constant is re-derived
  from the two named packs plus debug seeds 51–65.

* **Clean room:** writes confined to `packs/c2k1clean_obj_bbq_sauce_task_k1/*`
  and `results/*c2k1clean_obj_bbq_sauce_task_k1*`; no `.bddl`/`.xml`/`.hdf5`/
  `init_states` read; no other cell's or campaign's artifacts read; no
  `tools/fewshot_run.py`; no `api.done` attribute anywhere (AST-checked).

**STOP.**
