# l90abl / open_top_drawer_s2_k3 — notes

Intent: "open the top drawer of the cabinet" (KITCHEN_SCENE2, LIBERO-90).
Runner: `tools/fair_run.py` only. Pack: K=3 demos + 9 keyframe PNGs.

## Pack reading (before any run)

Three demos, all 64–74 steps, stride 10, agreeing closely:

| | t000 (home) | t020 (arc) | t030 | t040 (engage) | end |
|---|---|---|---|---|---|
| d0 | (-0.195,-0.013,1.162) | (-0.027,-0.133,1.168) | (0.019,-0.172,1.109) | (0.020,-0.141,1.109) | (0.013,0.019,1.107) |
| d1 | (-0.193,-0.003,1.167) | (-0.032,-0.130,1.157) | (0.008,-0.155,1.112) | (0.009,-0.179,1.104) | (0.010,0.004,1.106) |
| d2 | (-0.193,-0.008,1.175) | (-0.051,-0.106,1.183) | (0.012,-0.154,1.144) | (0.027,-0.168,1.109) | (0.017,0.033,1.101) |

Facts extracted:

1. **`gripper_cmd = -1` (open) at every keyframe of every demo.** There is no
   close anywhere in the pack. The handle is engaged with open fingers, not
   pinched. `gripper_state` at the engage keyframe is asymmetric
   ([0.033,-0.040], [0.031,-0.040], [0.030,-0.040] vs [0.036,-0.036] at home):
   one finger is pushed in, the other stays fully open — a one-sided contact,
   i.e. a hook, not a straddle.
2. **The drawer opens along +y.** All three demos travel ~0.19 m in +y at
   constant z after reaching y ≈ -0.173, and the episode ends there.
3. **The pull is a press-and-drag.** Raw actions over the pull phase are a
   near-constant (dx,dy,dz) ≈ (-0.30, +0.90, -0.57) while x and z do not move
   at all. Saturated command + zero motion = contact constraint: the
   demonstrator presses the open fingers down (and slightly -x) into the handle
   for the whole pull. Reproducing that press is what `PULL_BIAS` is for.

Keyframe PNGs (128 px, upscaled locally) corroborate: the cabinet is a dark box
at the left edge of the table with two stacked handles on its camera-facing
front; the upper one is the top drawer. Final frames show that drawer extended.

## Design

`api.move` drives `action = clip((target - eef)/0.05, -1, 1)` (generic
controller mechanics, stated in the harness docstring). To reproduce the demo's
action ratio without saturating all three axes, the pull target is placed
0.017 m in -x and 0.032 m in -z of the measured engage point — those errors
alone give -0.34 and -0.64 — and 0.28 m in +y so the y term stays saturated for
the whole travel. `|err| ≥ 0.032 > POS_TOL`, so the move can never converge and
spends its whole step budget pressing and dragging, exactly as the demo does.

Waypoints are componentwise means of the demos' t020 / t030 / t040 samples; the
engage y is the mean of each demo's *deepest* logged y (-0.173) rather than the
t040 mean, because d0 had already begun retracting by t040.

## Version log

### v1 — demo-faithful replication + demo-derived press vector
- **Hypothesis:** the drawer is a fixed fixture and the demos' waypoints
  transfer verbatim; the only non-obvious ingredient is the downward/-x press
  that keeps the open fingers hooked during the +y drag.
- **Evidence:**
  - probe (8 seeds 51,53,…,65): **8/8**, `results/fs_l90abl_open_top_drawer_s2_k3_v1`
  - selection (all 15 debug seeds 51–65): **15/15**,
    `results/sel_l90abl_open_top_drawer_s2_k3_v1`
  - `sim_steps` 83–143 of a ~500 horizon; every episode ended inside the first
    pull.
  - Per-episode logs are **bit-identical across seeds** through the engage
    (`eef0=[-0.2085,0,1.1733]`, `arc res=0.0084`, `low res=0.0102`,
    `engage res≈0.012 eef≈[0.023,-0.165,1.115]`). The cabinet is seed-invariant
    in this scene; only the plates on the table randomize, and they are nowhere
    near the approach corridor. That is why a fixed sequence suffices.
  - The engage move stops ~9 mm short of `W_ENGAGE` at residual 0.012 — blocked
    by the handle. That block is the engagement; the pull then carries
    `travel_y` = 0.159–0.170 m.
- **Verdict:** accepted and frozen. No further version was needed.

### Diagnostic caveat (does not affect control)
`_look()` logs a base→pixel projection that assumes a +z-forward camera. The
deprojection it reads back at t0 (`[-0.243,-0.243,0.901]`) is not the engage
point, so the sign convention is wrong for this camera (MuJoCo looks down -z).
It is pure logging — no control path reads it — and it did still register the
scene change caused by the pull (`[-0.036,-0.187,1.064]` afterwards). Left
as-is rather than re-running a 15/15 program to fix a log line.

## DECLARATION

- **Frozen version:** `program_v1.py`, copied verbatim to
  `packs/l90abl_open_top_drawer_s2_k3/program.py`.
  `md5 == 57bef0dacc4515334bc6cfd5bcc61806` for both, verified on the cluster
  and locally.
- **Selection receipt (full 15 debug seeds 51–65):** **15/15**
  `benchmark_success: true` in
  `results/sel_l90abl_open_top_drawer_s2_k3_v1/results.jsonl`.
- **Per-version receipt chain:** v1 probe 8/8
  (`results/fs_l90abl_open_top_drawer_s2_k3_v1`) → v1 selection 15/15
  (`results/sel_l90abl_open_top_drawer_s2_k3_v1`). Single version; no earlier
  candidates were formally probed.
- **PROVENANCE:** present as a top-level literal dict in `program.py`, covering
  all six calibrated constants (`W_ARC`, `W_LOW`, `W_ENGAGE`, `PULL_BIAS`,
  `PULL_SECONDS`, `OPEN_WIDTH_M`). Every source is a pack field (ee_path6
  samples, raw actions, gripper_cmd) or generic controller mechanics. No
  debug-seed measurement was needed and none was used as a constant.
- **Clean room:** reads were limited to this cell's pack, `tools/fair_run.py`,
  `tools/fair_client.py`, `heron/robot/libero.py` (harness/controller mechanics)
  and this cell's own results. No `.bddl`/`.xml`/`.hdf5`/init_states, no other
  campaign's packs or results, no `program*.py`/`NOTES.md` under any pack dir,
  no `tools/probe_*.py`. `--split eval` was never invoked; seeds 1–50 untouched.

STOP.
