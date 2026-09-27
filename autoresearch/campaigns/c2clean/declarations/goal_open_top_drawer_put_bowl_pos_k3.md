# c2clean / goal_open_top_drawer_put_bowl_pos_k3 — worker notes

Intent: "Open the top layer of the drawer and put the bowl inside".
Runner: `tools/fair_run.py` only. Splits sealed (debug 51-65).

## Scene facts re-derived in this cell

All from the pack (`pack.json` keyframes / ee_path6 / actions, `keyframes/*.png`)
and my own debug-seed (51-65) RGB-D captures. No foreign constants.

| fact | value | source |
|---|---|---|
| table top z | 0.9012 | debug seeds 51/53/55/57 cam_high deprojection of bare table |
| fingertip offset below eef ref | 0.009 m | v1 probe: press straight down onto bare table, eef stalls at z=0.9102 |
| cabinet top plane z | 1.127 | debug-seed heightmap mode, y<-0.12 band |
| top-drawer handle slab | top z 1.098, protrudes to y≈-0.126, x-span ≈0.15 | debug-seed heightmap |
| bowl | round, dia 0.108, top z 0.9518 (=table+0.051) | debug-seed heightmap cluster |
| plate (distractor) | dia 0.136, top 0.920 | same |
| episode horizon | **1000 sim steps**; each `api.move` costs a fixed minimum (~60) | v2 receipt: 12 moves+settles → `sim_steps: 1000`, arm frozen mid-run |

## Mechanism read off the pack

All three demos share the same three-phase structure:

1. **Drawer open (t≈25-70)** — eef at x≈+0.02..0.05, **z pinned at 1.105-1.108**
   while the raw action commands a strong sustained `-z` (−0.3..−0.9) and
   `+y` (≈0.7). So it is a *contact-limited press-and-drag*: the −y finger is
   pressed onto the drawer's handle slab (measured top 1.098 — the eef sits
   7 mm above it, fingertip 2 mm below its top) and the wrist drags **+y** by
   0.163 m (demo0 eef y −0.093 → +0.070). The cabinet is on the −y side and the
   drawer slides out toward +y.
2. **Bowl grasp** — descend to eef z 0.9149/0.918/0.939 (fingertip ≈ table+0.01)
   at an offset of 0.055-0.067 m from the bowl centre, mostly +y, and close.
   Closed gripper_state ≈ 0.005 m → the fingers **straddle the bowl wall**
   (rim radius measured at 0.055 in the debug seeds), not the whole bowl.
3. **Release** — lift to z≈1.22-1.26, translate to (x≈+0.01, y≈−0.048), descend
   to z≈1.13-1.17 and open. Relative to the handle: Δx ≈ −0.017 from the handle
   x-centre, Δy ≈ −0.083 from the *post-drag* handle tip.

## Version log

### v0 — perception probe (no motion)
Hypothesis: the scene can be recovered from cam_high alone.
Evidence: seeds 51/53/55/57. Built a base-frame point cloud from
depth+intrinsics+extrinsics and a 4 mm top-down heightmap. Recovered, on every
seed: cabinet top plane (1.1271-1.1272), cabinet face y, the top-drawer handle
slab, the bowl, the plate and the blue box.
Gotcha: `api.log` truncates each line at 2000 chars — base64 payloads must be
chunked.
Verdict: perception is solid and seed-stable. The cabinet shifts ~15 mm in y and
the bowl ~20 mm in x/y between seeds, so everything must be perceived per-seed.

### v1 — fingertip-offset probe
Hypothesis: the eef reference is well above the fingertips (so demo z values
could not be used literally).
Evidence: pressing down on the bare table stalls the eef at z=0.9102, table
z=0.9012 → **fingertip offset is only 0.009 m**. Refuted the hypothesis; the
demos' absolute z values transfer directly.

### v2 — first full attempt
Hypothesis: perceive → press-drag the handle → straddle-grasp the bowl → drop.
Evidence (seeds 51,53,55,57): 0/4. Perception + phase 1 both worked — the GIF
shows the drawer pulled open, eef y −0.095 → +0.050 (drag 0.145 of 0.163) —
but every api.move after the drag was a no-op and `sim_steps` was exactly
**1000** on all four episodes.
Verdict: **step-budget exhaustion, not a mechanism failure.** `goto()` retry
loops (4 moves each) plus an 8-step drag plus a re-perception detour burned the
whole horizon. Fix: a fixed, short sequence of ~11 moves and no retries.

### v3 — budgeted fixed sequence (11 moves)
Hypothesis: with the retries and the re-perception detour removed, the same
mechanism completes inside 1000 steps. The post-drag drawer position is computed
from the *achieved* drag (eef y at the end of the drag) instead of a second
capture, and the release point from the demo's handle-relative offsets.
Evidence: **8/8** on the probe subset (51,53,55,57,59,61,63,65), dir
`results/fs_c2clean_goal_open_top_drawer_put_bowl_pos_k3_v3`.
Verdict: accepted; taken to the formal full-15 selection run.

### v3 aim-envelope check (not a candidate version)
Hypothesis: 15/15 could still be a knife-edge aim.
Evidence: the grasp offset `GRASP_DY` displaced by −8 mm and +8 mm, 6 seeds each
(51,53,55,57,59,61): **6/6 and 6/6**
(`results/fs_..._env_m008`, `results/fs_..._env_p008`).
Verdict: the straddle grasp self-centres over at least a ±8 mm band, so the
15/15 is not a lucky aim. (Mechanism note: the closing jaws drag the bowl wall
to the jaw midline, which is why the offset is forgiving.)

## DECLARATION

- **Frozen version: v3.** `packs/c2clean_goal_open_top_drawer_put_bowl_pos_k3/program.py`
  md5 `bd940ce1510111b10e12e6c77e1a2df2` == `program_v3.py` md5
  `bd940ce1510111b10e12e6c77e1a2df2` (verified on the cluster).
- **Selection receipt: 15/15** on the full debug split (seeds 51-65), one formal
  run, dir `results/sel_c2clean_goal_open_top_drawer_put_bowl_pos_k3_v3`.
  Per-seed: 51-65 all `benchmark_success: true`; `sim_steps` 914-922 of the
  1000-step horizon (~80 steps of headroom; the sequence is fixed, so this count
  is deterministic).
- **Receipt chain:**
  - v0 perception probe — 51,53,55,57 — no motion; scene recovered on every seed.
  - v1 fingertip probe — 51,53 — fingertip offset 0.009 m below the eef ref.
  - v2 first full attempt — 51,53,55,57 — **0/4**, cause identified as step-budget
    exhaustion (`sim_steps: 1000`, arm frozen after the drag), not mechanism.
    Dir `results/fs_..._v2`.
  - v3 budgeted fixed sequence — probe **8/8** (`results/fs_..._v3`), formal
    **15/15** (`results/sel_..._v3`), envelope 6/6 at ±8 mm grasp displacement.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  TABLE_Z, TIP_OFFSET, DRAG_LEN, DRAG_Z_ABOVE_HANDLE, HOOK_FINGER_DY, GRASP_Z,
  GRASP_DX, GRASP_DY, RELEASE_Z, RELEASE_DY_FROM_TIP, RELEASE_DX_FROM_HANDLE,
  CARRY_Z, OPEN_W, CAM_MODEL. Every source is this cell's pack or a debug-seed
  (51-65) measurement.
- Clean room respected: only `tools/fair_run.py` was ever invoked; writes confined
  to the pack dir and `results/*c2clean_goal_open_top_drawer_put_bowl_pos_k3*`;
  no benchmark asset, other campaign's artifact, or any `program*.py`/`NOTES.md`
  under any pack directory was read.

STOP.
