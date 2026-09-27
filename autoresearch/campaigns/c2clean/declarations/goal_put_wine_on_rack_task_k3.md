# c2clean / goal_put_wine_on_rack_task_k3 — "Put the cream cheese on the rack"

Runner: `tools/fair_run.py` only. No shared note file (FAIR_PROTOCOL v1.1.1).

## Evidence read from the two packs

| pack | language | what it gives |
|---|---|---|
| `..._task_k3` | "put the wine bottle on the rack" | the TARGET half: where the rack is and where a released object must go |
| `..._task_mate` | "put the cream cheese in the bowl" | the OBJECT half: how the cream cheese is taken |

- **mate pack** — wrist stays straight down the whole demo (rpy ≈ (3.14, 0, 0),
  i.e. the home orientation, never re-oriented). The closing keyframe is
  demo0 t40 z=0.9104, demo1 t45 z=0.9205, demo2 t42 z=0.9106 → the cream
  cheese is taken at eef z ≈ 0.911. Held gripper width 0.0432 m.
- **k3 pack** — the bottle is taken with a heavily rotated wrist (rpy ≈
  (1.5, 1.68, −0.83)) — a bottle-specific side grasp, not transferable — and
  released over the rack at eef y = −0.281 / −0.273 / −0.234, z = 1.223 /
  1.198 / 1.194. That release band is the transferable part.

## v1 — perception probe (seeds 51,53,55,57)

Moves nothing; streams cam_high + cam_arm_wrist RGB-D out through `api.log`
(zlib+base64, 1900-char chunks) for offline analysis. Receipt: 4/4 episodes,
72 sim steps each, `benchmark_success: false` (expected — it never acts).

Measurements, all four seeds:

- table height z = **0.9009**
- **cream cheese** = the only blue-dominant object on the table
  (B/sum > 0.42, B > R+18, B > G+12): 253–260 top-face pixels/seed, a flat
  box, top face **0.065 × 0.0335 m**, top at z = **0.9198** → only 0.019 m
  tall. Centroid moves ~0.02 m between seeds (e.g. (−0.041, 0.133) on 51,
  (−0.035, 0.116) on 57), so it must be perceived per episode.
- **rack** = warm-coloured points (R−B > 12) above z = 1.00 at y < −0.02.
  Cross-sections along x are identical, so the rack is **two parallel planks,
  each a plane tilted ≈ 33° about the x axis**, dropping toward +y:
  upper plank (y −0.31, z 1.21) → (y −0.185, z 1.13); lower plank the same
  0.13 m below. No lip, no horizontal shelf anywhere. The rack also shifts
  ~0.02 m between seeds (top-edge centroid y −0.301 on 51, −0.318 on 57), so
  it too must be perceived per episode.
- the k3 release band (y ≈ −0.26, z ≈ 1.19) lands on the **upper** plank a few
  cm down-slope of its high back edge. That is the goal site.

## v2 — pick with the mate grasp, place on the k3 release site

Hypothesis: the two halves compose directly. Take the cheese the way the mate
pack takes it (straight-down wrist, eef z = table + 0.010), carry at z = 1.29
(clears the cabinet top 1.134 and the rack top 1.243), and lower it onto the
upper plank at the *measured* plank height, 0.05 m down-slope of the measured
back edge — no absolute coordinate anywhere, both object and target are
re-perceived each episode.

Evidence (probe, seeds 51,53,55,57): **4/4 benchmark_success**, dir
`results/fs_c2clean_goal_put_wine_on_rack_task_k3_v2`.
Sensor receipts on ep51: closed to width 0.0422 at effort 3.0, still 0.0422
at effort 3.0 after the lift to z = 1.2825 (so the grasp survived the carry),
released at eef (−0.225, −0.257, 1.215) over a measured plank surface of
1.1815. The post-episode capture (after retreat + 0.8 s settle) shows the box
resting on the plank, so the predicate is satisfied by a real placement and
not by a transient pass-through.

Verdict: mechanism works. Promoted to the full-15 selection run.

Note the grasp width receipt: the gripper closes to 0.0422 m on the cheese,
which matches the mate pack's held width of 0.0432 m — independent
confirmation that the object being grasped is the one the mate pack demos.

## Selection (full 15 debug seeds)

See DECLARATION below.

Formal selection run, all 15 debug seeds (51–65), one run, no re-selection:

**15/15 benchmark_success** — `results/sel_c2clean_goal_put_wine_on_rack_task_k3_v2`

Per-episode receipts confirm every success is a real pick-and-place, not a
brush-past: in all 15 the gripper closed to 0.0422 m at effort 3.0, was still
0.0421–0.0423 at effort 3.0 after the lift to z ≈ 1.28, and was still holding
at effort 3.0 in the frame before the release command. The perceived cheese
spanned x ∈ [−0.068, −0.030], y ∈ [0.118, 0.153] and the perceived drop
target tracked the rack over y ∈ [−0.275, −0.255], so the program is
perceiving both ends per episode rather than replaying fixed coordinates.

---

# DECLARATION

- **Frozen version:** `program.py` == `program_v2.py`,
  md5 `0943c97f432407c339faad8ec1f352c0` (both files, verified on the cluster).
- **Selection receipt:** **15/15** on the full 15 debug seeds 51–65,
  dir `results/sel_c2clean_goal_put_wine_on_rack_task_k3_v2`.
- **Receipt chain:**
  - v1 (`program_v1.py`, md5 `dfc1c26b753cbb765ce62656774245ca`) — perception
    probe, 4/4 episodes ran, 0/4 success by construction (it never acts);
    dir `results/fs_c2clean_goal_put_wine_on_rack_task_k3_v1`.
  - v2 probe — 4/4 on seeds 51,53,55,57;
    dir `results/fs_c2clean_goal_put_wine_on_rack_task_k3_v2`.
  - v2 selection — 15/15 on seeds 51–65 (above).
- **PROVENANCE:** present, 8 entries, every one `allowed: True` with a source
  that is either a named pack field or a debug-seed measurement. No `.done`
  read anywhere in the program (AST-checked).
- **Clean room:** the only cluster writes were
  `packs/c2clean_goal_put_wine_on_rack_task_k3/*` and
  `results/*c2clean_goal_put_wine_on_rack_task_k3*`. Only `pack.json` and
  `keyframes/` were read from the two named packs. No `fewshot_run.py`, no
  benchmark asset, no other campaign's artifacts, no shared note file.
