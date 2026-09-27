# abl_c2 / ablC_goal_open_middle_drawer — worker ledger

Variant C ("no-verify"): ONE program version, ZERO episodes, straight to the
coordinator's blind sealed eval on seeds 1-50.

## 2026-08-20 (UTC) — session

### Inputs consulted
- `TASK.md` (this workspace), `LAWS.md` (empty by design).
- `packs/ablC_goal_open_middle_drawer/pack.json` (K=3 demos: keyframes with
  EEF 6-DoF + gripper_cmd + gripper_state, ee_path, ee_path6, 138/138/151 raw
  actions, action_scale, language).
- `packs/ablC_goal_open_middle_drawer/keyframes/*.png` (10 cam_high frames,
  128x128), inspected visually at 5x and measured numerically.

Nothing else. In particular no .bddl/.xml/.hdf5/init-state file, nothing under
campaigns/c1, c2 or c2fix, no `packs/c1_*` / `packs/c2_*`, no `results/*c1_*` /
`results/*c2_*`, no `tools/probe_*.py`, no other abl_c2 cell's material.

**Disclosure (no violation, recorded for the audit trail):** the initial `scp
-r` of the pack directory also pulled the sibling directory
`packs/ablC_goal_bowl_on_plate/` onto the local scratchpad. It was deleted
immediately and unread — the only thing that ever entered context was the `ls`
listing of its file NAMES (`pack.json`, `keyframes/demo*_t*.png`). No byte of
its pack.json, no keyframe image, no program, no result was opened. That cell
is a different task (bowl on plate) and none of its content informed this
program.

### Hypothesis (the only one; no evidence can be gathered in this variant)

**H1 — the drawer is opened by an OPEN-gripper hook, not a grasp, and the
cabinet is a fixed fixture, so a K=3-averaged EEF replay with a heavily-shrunk
perception correction and an image-based re-hook loop is the best available
policy.**

Evidence available *from the pack alone*:

1. `gripper_cmd == -1.0` and `gripper_state ~ (+0.036..+0.040,
   -0.036..-0.040)` at **every** keyframe of **all three** demos. The gripper
   is never closed. The drawer is dragged by hooking the open fingers on the
   handle.
2. Converting `ee_path6` axis-angles to rotation matrices: the finger-
   separation axis (tool y) rolls to world +x and the palm axis (tool z) tilts
   from straight-down to **57.4-60.6 deg** off vertical, pointing mostly along
   -y and ~30 deg down. Reproduced exactly by `tool_rot(58.5)` →
   toolZ (0, -0.853, -0.522), toolY (1, 0, 0), det +1.
3. Phase-resampling the three `ee_path6`s to 14 samples and averaging gives a
   clean, low-variance approach. The insertion sample is
   **(0.0208, -0.1435, 1.0388)** with cross-demo sd **(18.0, 0.5, 5.2) mm**.
   The 0.5 mm sd in y is the strongest constraint in the whole pack: it is the
   cabinet's drawer-face plane, i.e. a hard geometric constant. The 18 mm sd
   in x says the contact is laterally forgiving (a bar handle).
4. Then z drops 6.6 mm at fixed y (the "seat" that puts the handle in the
   finger crook), and the path translates along +y to y = +0.0237 (per-demo
   +0.025 / +0.030 / +0.016) at z ~ 1.0325. Total drawer travel ~ 0.17 m.
5. **Cabinet is fixed.** Dark-pixel mask (max(R,G,B) < 70) in the window
   u<45, 40<=v<115 of the three t=0 keyframes: n = 2281 / 2279 / 2228,
   centroid = (19.59, 74.88) / (19.40, 74.67) / (19.09, 73.91). Agreement
   <= 1 px in both axes across three different init layouts, while the small
   objects (bottle, bowl, plate, phone) visibly move. So layout variation is
   in the clutter, not in the cabinet.
6. **A legitimate, pack-calibrated open/closed sensor exists.** Dark-pixel
   count in the band 45<=u<75, 40<=v<125 — the strip the drawer sweeps into —
   is **222 / 238 / 233** with the drawer closed and **869 / 866 / 825** in
   each demo's final (opened) keyframe, *even with the arm in shot*. A 3.6x
   separation. Threshold 520.

### Verdict (static only — this is the ablation)

Frozen as **program_v1.py == program.py**, md5 `1eb225228ef8091809e43987c18c2dbc`.
No episode was run, so H1 carries **no execution evidence**. Its support is
entirely the six pack observations above. The re-hook loop is what verification
would otherwise have bought: it converts "which z is right?" from a question I
cannot answer offline into an in-episode search over
`RETRY_DZ = (0, -0.014, +0.014)`, adjudicated by observation 6.

Design choices made to bound the harm of untestable code:
- The perception shift is *shrunk* by 0.70 and hard-clipped to +/-30 mm in x
  and **+/-8 mm in z (below the 5 mm pack sd)**; it is never applied to y (the
  0.5 mm-sd face plane). It is discarded entirely if the mask size leaves
  [1500, 3400] px-128, if the centroid moves > 22 px, or if the deprojection
  Jacobian shows a depth discontinuity. Worst case it degrades to pure replay.
- Every FairApi call is wrapped; a perception failure cannot kill the episode.
- A self-imposed motion-time counter (34 s) bounds the retries; no episode
  termination flag is consulted anywhere.

### Static self-checks actually run (permitted; none execute a policy)
| check | outcome |
|---|---|
| `python -m py_compile program.py` | OK |
| forbidden-token scan (all 18 tokens from TASK.md) | NONE present |
| AST scan for any `.done` attribute read | 0 |
| top-level literal `PROVENANCE` dict | present, 21 entries, every one `{"source": ..., "allowed": True}` |
| every module-level UPPER_CASE constant covered by PROVENANCE | uncovered: NONE |
| `run(api)` defined and callable | yes |
| offline replay of the two image predicates over the pack's own keyframes (pure image maths, no env) | `drawer_looks_open`: False on all 3 closed t=0 frames, True on all 3 opened final frames; cabinet centroid recovered to <= 0.5 px of the reference |

### DECLARATION
- Frozen version: **program.py**, byte-identical to **program_v1.py**,
  md5 **`1eb225228ef8091809e43987c18c2dbc`** (verified identical on AbakaAI at
  `/mnt/data/YifanKang/Heron/packs/ablC_goal_open_middle_drawer/`).
- **Episodes run: 0.** `tools/fair_run.py` was never invoked, on any split, on
  any seed. `tools/fewshot_run.py` was never invoked. No seed in 1-50 or 51-65
  was touched in any way.
- **Program versions written: 1.**
- PROVENANCE present: yes (21 entries, all `allowed: True`, all sourced to
  pack.json fields, pack keyframes, or generic controller/camera mechanics).
- Cluster writes: only `packs/ablC_goal_open_middle_drawer/program.py` and
  `program_v1.py`.

STOP.
