# Campaign abl_c2 — ENCORE component ablations (FAIR_PROTOCOL v1.0 governs)

3 variants x 3 tasks = 9 cells, namespace `abl_c2_`. Every cell inherits c2's
isolation regime verbatim: programs run ONLY under `tools/fair_run.py`
(fair-v1.1.1 gate: no `api.done`, PROVENANCE required, sealed splits),
debug/learn seeds 51-65, blind coordinator-run eval on seeds 1-50.

## The full-ENCORE reference column (NOT rerun)

c2's stock cells for the same three tasks, same bddl, same protocol:

| task | c2 cell | sealed |
|---|---|---|
| turn on the stove | `c2_goal_turn_on_stove_stock` | 50/50 |
| put the bowl on the plate | `c2_goal_bowl_on_plate_stock` | 50/50 |
| open the middle drawer of the cabinet | `c2_goal_open_middle_drawer_stock` | 48/50 |

All three were c2 **wave-1** cells, run with an **EMPTY** LAWS.md. That fixes
the abl_c2 baseline: the reference condition is *mined pack + empty law
library + law-writing discipline + probe/verify loop*. Each variant removes
exactly one of those relative to that reference (see CONFOUNDS in NOTES.md
for what this does and does not let variant B claim).

## Variants

- **A `naive-demos`** — the pack-mining stage is removed. Instead of
  `fair_pack.py`'s distillation (keyframe segmentation, per-keyframe EEF /
  gripper table, strided `ee_path`/`ee_path6`, `action_scale`), the worker
  gets `raw_demos.json`: the K=3 demos' DENSE per-timestep robot proprio and
  DENSE raw action arrays, plus uniform-stride RGB frames. Nothing is
  selected, summarised or derived coordinator-side. Law discipline and the
  probe/verify loop stay. Empty LAWS.md, as in the reference.
  (Note: this is a *presentation* ablation, not an information-removal one —
  the raw dump is a superset of the pack's numeric content.)
- **B `no-LAWS`** — mined pack stays, probe/verify loop stays. No LAWS.md is
  created in the workspace; the brief forbids reading or writing any law file
  and forbids the law-formalisation step (no candidate laws, no banking).
- **C `no-verify`** — mined pack stays, empty LAWS.md stays. The worker writes
  ONE program version from the pack and it goes straight to the sealed eval.
  Zero probe episodes, zero debug seeds, zero iterations. A local static
  self-check (py_compile + the fair gate's own token/AST/PROVENANCE rules) is
  permitted because it inspects source text, not behaviour; anything that
  executes an episode is forbidden and VOIDS the cell.

## Cells

    ablA_goal_turn_on_stove      ablB_goal_turn_on_stove      ablC_goal_turn_on_stove
    ablA_goal_bowl_on_plate      ablB_goal_bowl_on_plate      ablC_goal_bowl_on_plate
    ablA_goal_open_middle_drawer ablB_goal_open_middle_drawer ablC_goal_open_middle_drawer

bddl paths and intent sentences are taken VERBATIM from c2's
`stock_manifest.txt` / `languages.txt`.

## Budget

5 program versions for A and B; 1 for C. Nine sealed evals, one per cell,
run once at the end by the coordinator.
