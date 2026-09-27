# c2clean — goal_open_top_drawer_put_bowl_pos_k0

Intent: *"Open the top layer of the drawer and put the bowl inside"*.
No demonstration pack (K=0). Every constant below was re-derived from debug
seeds 51–65 via `tools/fair_run.py --split debug`.

## What the scene turned out to be (debug-seed RGB-D, seeds 51/53/55/57)

* Table plane z = 0.9009–0.9010.
* A three-drawer cabinet on the −y side. Its top slab is at z = 1.1270 and its
  **front face is a plane of constant y** (y ≈ −0.157 … −0.169 depending on the
  seed), so the drawers pull along **+y**, not +x.
* Three handles, detected as y-protrusions in front of that face, at
  z = 1.090 / 1.017 / 0.949. "Top layer" = the z = 1.090 one.
* Free props on the table: a plate (h ≈ 0.019), a small box (h ≈ 0.019) and the
  **bowl (h ≈ 0.051, dx ≈ dy ≈ 0.110)**. Height separates the bowl cleanly —
  0.051 vs 0.019 — so `max(h)` names it with a 2.7× margin.
* A stove slab + kettle at x ∈ [−0.30, −0.16], y ∈ [0.11, 0.30]. This matters:
  it is what the forearm collides with (see v6).

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | stream RGB-D out through `api.log` and do perception offline | `api.log` truncates messages at ~2000 chars; 3000-char chunks were cut | fix chunk size to 1400 (v1b), then clean |
| v1b | deprojection can be reproduced from `.intrinsics`/`.t_base_cam` | matches `api.deproject` to < 0.5 mm on 5 probe pixels | ✅ whole-frame clouds usable |
| v2 | press the bare table at (0.15, 0.30) to measure the fingertip offset | arm never reached the target (resid 0.05 on the *hover*), then froze for the rest of the episode; 1000 sim steps burnt | ❌ that spot is at a reach limit — a repeatable stop is not proof of contact |
| v3 | press the **cabinet top** instead (near the work area, high, easy pose) | two presses at different xy both stopped at eef_z − z_top = 0.0111 / 0.0124 | ✅ **TIP = 0.0117**; agreement across two spots is what makes it contact and not a limit |
| v4 | straddle the handle bar top-down, inboard finger in the standoff gap | descent hard-stopped at fingertip z = 1.1249 ≡ the cabinet top (1.1270), on both seeds | ❌ `api.grip` is **binary** (0.079 open / closed), so the open jaw is 0.079 wide and the inboard finger always lands on the cabinet top |
| v4 (wrist close-up) | resolve the handle from straight above | top surface is *continuous* from yface+0.003 to yface+0.030 at z ≈ 1.089–1.097 | the handle is a **solid ledge**, not a bar with a graspable window |
| v5 | turn the wrist so the approach axis is −y and the jaws open along **z**, then clamp the ledge from above and below | seed 57: clamp width 0.0175 @ effort 3.0, drawer travelled 0.160 m. Seed 51: width 0.0015 @ effort 0.05 — closed on air | ⚠️ mechanism right, engage depth too shallow (fingertips only 10 mm past the ledge face, and the arm drifts +y ~0.01 during the close) |
| v6 | engage 16 mm past the ledge face (what seed 57 reached by accident); add bowl pinch + place | drawer: **2/2** clamp 0.0174 @ 3.0, travel 0.1598/0.1597. Bowl: descent hard-stopped 52 mm high, correction move moved 1 mm | ❌ approaching the bowl's **+y** rim puts the wrist/forearm down on the stove (visible in the gif) |
| v7 | take the bowl from its **+x (front)** rim with the jaws turned along x, keeping the forearm clear of the stove at −x,+y | probe 51/53/55/57 4/4; **formal 15-seed selection 13/15**, failures 56 and 59 | ⚠️ drawer stage 15/15, bowl pinch loses two seeds |
| v8 | aim at the *measured* mid-wall radius (near half of the bowl shows the outer surface, far half the inner) + a second correction move + retry | wall mid = 0.0498–0.0500 on every seed, so perception was never the problem. Probe 6/8; 56 and 59 still miss, and both stop **1.3–1.5 mm short in +x** while the six successes converge to ±0.4 mm | ❌ the 2 mm wall needs ±1 mm aim, and jaw_x ≳ 0.010 is outside the arm's low-z reach |
| v9 | rotate the grasp bearing to −60° to keep eef_x small | **0/8**, every episode burning the full 1000-step horizon; moves never converge | ❌ that wrist orientation is unreachable, so `api.move` never converges |
| v10 | keep v7's +x bearing, add a verified −y-bearing fallback on a failed pinch | the fallback fires but the retry path exhausts the 1000-step horizon; the "held" reading afterwards is a frozen-state artefact | ❌ no step budget for a retry |
| v11 | −y bearing as the primary | 3/8; on the losses the descent is blocked at the *hover* height (z 1.090 / 1.065), i.e. it never descends at all | ❌ the low-z reach window in x is narrow; eef_x ≈ −0.04 is outside it |
| v12 | pinch **6 mm deeper** (0.016 below the rim instead of 0.010): mid-wall radius drops 0.0499 → 0.0474, which pulls jaw_x back inside the reach window, and the wall is *thicker* there | probe **8/8** incl. 56 and 59; closed width 0.0064–0.0068 @ effort 3.0 on all eight (was 0.0051); **formal 15-seed selection 15/15** | ✅ |
| v13 | v12 with PROVENANCE rewritten to the constants the program actually uses (v12's block was still v2's); `run()` and all helpers byte-identical | **formal 15-seed selection 15/15** | ✅ **frozen** |

## The four mechanisms that carry the cell

1. **The drawer face is a y-plane and the handle is a solid ledge.** A top-down
   straddle is impossible because the binary 0.079 jaw cannot fit in front of
   the face without one finger landing on the cabinet top. Turning the wrist so
   the jaws open along **z** and clamping the ledge from above/below uses the
   whole 0.079 opening on a 0.016 ledge. Receipt: closed width ≈ ledge
   thickness (0.0174 vs measured 0.0148–0.0162) with effort 3.0, on 15/15.
2. **Engage depth, not aim, decides the clamp.** The arm drifts ≈ +0.010 in y
   while the gripper closes, so the fingertips must start ~16 mm past the
   ledge's outer face or the jaws close in front of it (v5 seed 51).
3. **The jaw centre must land inside the bowl wall.** Each finger can travel
   only as far as the jaw centre, so a rim pinch holds iff
   r_inner ≤ jaw_centre ≤ r_outer. That window is the *wall thickness* — 2 mm
   at 10 mm below the rim. Both v7/v8 misses sat 0.1–0.5 mm inboard of
   r_inner; the six hits sat inside it.
4. **Depth buys both margins at once.** Pinching 16 mm below the rim instead
   of 10 mm widens the window (2.0 → 3.4 mm mean) *and* shrinks the commanded
   jaw x by 2.5 mm, which is what brings the two lost seeds back inside the
   arm's low-z reach. The reach ceiling is real and sharp: every debug pose
   with jaw_x ≳ 0.010 stopped 1.3–1.5 mm short and two further correction
   moves could not close the gap.

## Two things that cost versions

* **A repeatable stop is not proof of contact.** v2 read a "fingertip offset"
  off a press at (0.15, 0.30) that was really a reach limit, and the arm then
  froze for the rest of the episode. v3 only trusted the number because two
  presses at different xy agreed.
* **Step budget.** `api.move` is cheap when it converges (≈45 sim steps) and
  expensive when it cannot. v2/v3/v6/v10 hit the 1000-step horizon purely
  because they contained moves that could never converge; once the horizon is
  gone every later `api.eef`/`api.gripper` read is a frozen-state artefact and
  looks like a result. v13 finishes in 353–477 steps.

## DECLARATION

* **Frozen version: v13.**
  `packs/c2clean_goal_open_top_drawer_put_bowl_pos_k0/program.py`
  md5 `87f8751ff7fbd271dc144bc7e195da4c` == `program_v13.py` (verified on the
  cluster and locally).
* **Selection receipt: 15/15** on the full debug band 51–65, one formal run,
  `results/sel_c2clean_goal_open_top_drawer_put_bowl_pos_k0_v13`
  (no failures; 353–477 sim steps per episode).
* **Per-version receipt chain** (all under `results/`):
  `fs_..._v1`, `fs_..._v1b` (perception), `fs_..._v2`, `fs_..._v3`,
  `fs_..._v4`, `fs_..._v5`, `fs_..._v6` (mechanism probes),
  `fs_..._v7` 4/4 probe → `sel_..._v7` **13/15**,
  `fs_..._v8` 6/8, `fs_..._v9` 0/8, `fs_..._v10` 6/8, `fs_..._v11` 3/8,
  `fs_..._v12` 8/8 → `sel_..._v12` **15/15**,
  `sel_..._v13` **15/15** (frozen file).
* **PROVENANCE**: present as a top-level literal dict in `program.py`, ten
  entries, every one sourced to a debug-seed measurement or to generic
  camera/controller mechanics. No pack was supplied for this cell (K=0) and
  none was read.
* All versions archived as `program_v1.py` … `program_v13.py` in the pack dir.
* Eval seeds 1–50 were never touched.
