# l90abl / open_top_drawer_s1_k0 — NOTES

Zero-demo cell. Everything below is derived from my own debug-seed observations
(seeds 51-65) under `tools/fair_run.py --split debug`.

## Scene reading (probe1-3, seed ~54, cam_high)
- `cam_high` extrinsics: camera at base (0.659, 0.0, 1.610), image-right = base **+y**,
  image-down = base **+x** (view direction ~(-0.778, 0, -0.628)).
- Table top plane z ~= 0.900 (dominant depth-histogram bin 0.890-0.915, n=9915/16384).
- A dark cabinet sits on the **-y** side of the table with its drawer fronts facing **+y**
  (three light-grey horizontal handle bars visible on that face).
- Three handle bars found as bright-achromatic clusters, evenly spaced in z by 0.071 m:
  z ~= 0.950, 1.021, 1.092. Top handle: y in [-0.211, -0.182], x in [-0.036, 0.049].
- Cabinet top slab: z ~= 1.128, y in [-0.379, -0.213], x in [-0.128, 0.123].
  So the drawer-front plane is y ~= -0.213 and the top handle protrudes to y ~= -0.182.
- Two distractor props on the table (a metal bowl near y~0 and a white plate near y~+0.25);
  both are bright/achromatic, so the handle detector must be restricted to y < -0.05.

Consequence: **opening a drawer = pulling in +y**, with the wrist's default straight-down
orientation the jaws already open along base y (tool_rot at reset ~ diag(1,-1,-1)).

## Probe log
- probe1: geometry/scale sanity. Table z=0.90. Relative writes land in a temp cwd; use the
  absolute pack path to dump artefacts. Log lines are truncated, so base64-in-log is out.
- probe3: dumped cam_high rgb+cloud to the pack dir (works).
- probe4: per-seed detector + closed-gripper press on the cabinet top slab to measure the
  eef->fingertip offset.

## Mechanism discovery (probes 4-17, debug seeds 51/53)

Rail geometry (cam_high + a top-down wrist capture at a known pose):
- Top rail: front face spans z 1.087-1.098, y-depth ~13 mm, x from about -0.045 to
  +0.043 with two mounting brackets near x = -0.032 and +0.030.
- Behind the rail is a ~12 mm slot, then the cabinet's top-slab edge (z=1.127).
- The cabinet TRANSLATES in y between seeds (slab edge -0.211 .. -0.234 over probe4
  seeds) while its z is fixed to <1 mm, so every y must be re-perceived per episode.

Negative results (each a measured block, not a guess):
- probe5: top-down straddle with jaws fully open -> descent stalls with the fingertips
  on the cabinet top slab. `api.grip(w)` has no intermediate width: anything >= 0.025
  opens fully (0.0795), so the jaw span cannot be narrowed to fit.
- probe7/8/9: apparent "reach floor" at eef z=1.1056 was an artefact -- `api.move` is
  step-budgeted and a `goto` that retries 6 x 2.5 s exhausts the episode, after which
  every move returns instantly and the arm is frozen. probe10: 80 moves of seconds=1.0
  that converge quickly all run fine, so the budget is in STEPS, and a non-converging
  long move is what burns it. probe11: descent at (x=-0.04, y=-0.10) reaches z=0.954 --
  there is no reach floor.
- probe12/13/14: with the jaws open, the descent is blocked once the rear finger's back
  face meets the rail's front face; free descent only resumes at tool y >= rail_front +
  0.065, i.e. the finger body extends ~0.025 m behind its gripping face. A top-down
  pinch of this rail is therefore geometrically impossible.
- probe14/15: yawing the wrist 90 deg (jaws along x) lets the closed gripper reach into
  the slot's y-range above the rail, but it still cannot descend past the rail top: the
  closed pair is wider in y than the 12 mm slot.

Positive result (probe17, 2/2 benchmark_success on seeds 51 and 53):
- Pitch the wrist 90 deg so the fingers point along -y and the jaws open along z
  (R_PITCH = [[1,0,0],[0,0,-1],[0,1,0]]). There is open air both above and below the
  rail (the top slab only overhangs behind it), so the open jaws swallow the rail from
  the front and close on it: width 0.0173, effort 3.0.
- Then pull +y. A 0.26 m command moved the rail 0.146 m and the benchmark bit fired.

Motion primitive that makes this work: `hold(target)` re-issues the SAME target up to n
times, returning early on convergence or on two successive commands that move nothing.
That distinguishes a starved move from a real block without burning the budget.

## Version log

| version | change | probe receipt | selection receipt |
|---|---|---|---|
| v1 | pitched-wrist front clamp on the top rail + `hold` motion primitive + one lower re-aim retry if the close reads no effort | 8/8 on 51,53,...,65 (`results/fs_l90abl_open_top_drawer_s1_k0_v1`) | **15/15** (`results/sel_l90abl_open_top_drawer_s1_k0_v1`) |

v1 was the first version formally probed; probes 1-17 were perception/mechanism probes
that ran no candidate program (their receipts and reasoning are in the section above).

## DECLARATION

- Frozen version: **v1**. `packs/l90abl_open_top_drawer_s1_k0/program.py` md5
  `7a49a3bf0e3015e260e7424c44cbf660` == `program_v1.py` md5
  `7a49a3bf0e3015e260e7424c44cbf660`.
- Full-15-seed selection receipt: **15/15 benchmark_success**, seeds 51-65, dir
  `results/sel_l90abl_open_top_drawer_s1_k0_v1` (15 episodes in results.jsonl, 15 with
  `"benchmark_success": true`). 0 retries fired; the gripper read effort 3.0 on the rail
  in every episode.
- Per-version receipt chain: v1 probe 8/8 (`fs_..._v1`) -> v1 selection 15/15
  (`sel_..._v1`).
- PROVENANCE present in program.py as a top-level literal dict covering R_PITCH,
  EEF_TIP_OFFSET, BAR_HALF_LEN, BAR_MID_DROP, TIP_INSET, STANDOFF, PULL_DIST,
  GRAY_MIN/SAT_MAX, Z_LINK and HOLD_EFFORT. Every constant traces to a debug-seed
  (51-65) measurement or to generic controller/camera mechanics. No pack was used
  (this is a k0 cell) and no LIBERO-specific prior knowledge was carried in.

STOP.
