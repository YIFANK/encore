# c2k1clean / obj_chocolate_pudding_task_k1

Intent: **"Pick the salad dressing and place it in the basket"**
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65; eval 1-50 never touched).

---

## What the packs actually said

| pack | its own language | grasp keyframe (EEF, gripper closes) | release |
|---|---|---|---|
| `..._task_k1` | "pick up the chocolate pudding and place it in the basket" | t=54, (-0.1127, -0.2513, **0.0101**) | t=150, (-0.0027, 0.246, 0.164) |
| `..._task_mate` | "pick up the salad dressing and place it in the basket" | t=51, (0.0712, -0.1069, **0.1282**) | t=128, (0.017, 0.220, **0.176**) |

The two packs live in different scenes, so **no xy transfers**. What does transfer is
(a) which object the intent names and (b) the *heights* at which the demo closes and opens.

**Naming the object (the whole cell).** Projecting the mate pack's grasp EEF xy through the
runtime `cam_high` intrinsics/extrinsics (K = 618.04, c = 256; `t_base_cam` from a debug
capture) onto the mate keyframe `demo0_t0000.png` lands at uv512 = (191, 295) — a bottle
with a **dark-green cap** over a pale flared body. The identical 128 px patch appears in the
k1 pack's own t=0 frame at uv512 = (277, 320), i.e. the same asset is present in *this*
scene. In the runtime clouds that asset reads "CREAMY Ranch Dressing" and its cap is the
only green top band on the table (cap RGB (24,58,38); every other prop is >= +12 on R or B).
**Target = the green-capped bottle**, not the chocolate pudding the bddl filename mentions.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v0 | perception probe: dump cam_high RGB-D + K + `t_base_cam` for all 15 debug seeds | table plane Z=0.001; props Z<=0.148; arm cloud Z>0.24; 5 clusters; dressing cap axis (0.1503, 0.0291), top 0.1475, cap dia 0.035, body dia 0.063; basket rim top 0.1437, centre X in [-0.014,0.015], Y in [0.245,0.273]. Across seeds 51-65 only the bbq-sauce bottle and the basket jitter; the dressing is pixel-identical | perception settled | 
| v1 | aim at cap axis, descend to top-0.021 (the mate demo's height), 6-offset ladder, `api.move(seconds=2.0)` legs | **0/8**. `sim_steps` = 1000 on every episode — the *step budget*, not aim, was the binding constraint. `move_cartesian` costs `2*60*seconds` steps and only breaks inside POS_TOL=0.012, so one 0.12 m leg burned its full 240-step allowance and still missed by 0.018. try2's "held=True" was an artifact: after the horizon fires `_step_env` is a no-op, so the gripper never actuates and the open gap reads as a hold | refuted; budget is the mechanism |
| v2 | diagnostic: same aim, descent chopped into 0.02-0.03 m legs at `seconds=0.5` | first try grips the cap: gap 0.0343, effort 3.0, still held after lifting to 0.27. **218 sim steps total** | staged legs converge; aim was right all along |
| v3 | v2's staging generalised: `_hop()` walks any leg in <=0.055 m hops; full pick-carry-place | **8/8** on 51,53,...,65 at 215 steps/episode; grasp succeeds on try0 every seed | works |
| v4 | harden perception for unseen eval layouts (cap found from green *pixels* inside a cluster rather than from a whole cluster, so a fused prop still exposes its cap; workspace sanity boxes with debug-measured fallbacks; second full pick attempt with re-perception if the carry loses the hold) | **15/15** on the full debug band | **FROZEN** |

### The one thing that mattered
The failure was never perception or grasp geometry — v1 and v3 aim at the same point. It
was that `api.move` charges `2 * 60 * seconds` steps against a 1000-step episode and only
exits early once it is inside 12 mm. Long legs overshoot, never break early, and eat the
episode. Chopping every leg to <= 55 mm cut a full run from >1000 steps to 215.

---

## DECLARATION

- **Frozen version:** `packs/c2k1clean_obj_chocolate_pudding_task_k1/program.py`
  md5 `97ed0e4d0b307cbc0211219f158399f9` == `program_v4.py` (same md5). Local copy:
  `program_frozen_v4.py` in this worker dir, same md5.
- **Selection receipt (full 15 debug seeds):** **15/15**
  `results/sel_c2k1clean_obj_chocolate_pudding_task_k1_v4/` — `grep -c '"benchmark_success": true' results.jsonl` = 15, episodes 51-65.
- **Receipt chain:**
  - v0 `results/fs_c2k1clean_obj_chocolate_pudding_task_k1_v0/` — 15 eps, perception probe (0/15 by construction, no motion)
  - v1 `results/fs_c2k1clean_obj_chocolate_pudding_task_k1_v1/` — 0/8 (51,53,...,65)
  - v2 `results/fs_c2k1clean_obj_chocolate_pudding_task_k1_v2/` — 0/2 diagnostic (51,53), grasp+lift verified
  - v3 `results/fs_c2k1clean_obj_chocolate_pudding_task_k1_v3/` — **8/8** (51,53,...,65)
  - v4 `results/sel_c2k1clean_obj_chocolate_pudding_task_k1_v4/` — **15/15** (51-65)
- **PROVENANCE:** present as a top-level literal dict in `program.py`, 21 entries, every one
  sourced to the named packs' `pack.json`/`keyframes/`, a debug-seed (51-65) measurement, or
  generic controller/camera mechanics. No LIBERO-specific prior knowledge was used: table
  height, object heights, cap diameter, reach and grip widths were all re-measured from the
  debug captures.
- **Clean room:** writes confined to `packs/c2k1clean_obj_chocolate_pudding_task_k1/*` and
  `results/*c2k1clean_obj_chocolate_pudding_task_k1*` (incl. the v0 dump dir
  `results/dbg_c2k1clean_obj_chocolate_pudding_task_k1/`). Only `pack.json` + `keyframes/`
  were read from the two named packs. `api.done` never read; no `fewshot_run.py`; no eval
  seed touched.

STOP.
