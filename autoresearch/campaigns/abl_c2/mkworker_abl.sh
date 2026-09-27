#!/bin/bash
# abl_c2 (ENCORE component ablation) worker workspace generator.
#   mkworker_abl.sh <variant A|B|C> <task_key> <bddl_path> <intent_sentence> <gpu>
#                   [version_cap] [cell_override]
#
# version_cap defaults to 5 (the campaign budget). It is raised ONLY for a
# documented protocol deviation — see RESULTS.md / NOTES.md CONFOUND 3, where
# the drawer row's cap-5 budget is confounded with the variant under test.
# cell_override renames the cell (used for reruns, e.g. ablB2_*).
#
# Brief is MECHANICAL: paths + contract only. Cloned from c2/mkworker.sh; the
# ONLY differences are three variant-conditional blocks (pack / laws / probe
# loop) plus the run recipe. Everything else is the c2 contract verbatim.
#
# All section text lives in QUOTED heredocs (fully literal — backticks and $
# survive) and is stitched in via @@PLACEHOLDER@@ substitution, so nothing in
# the brief is ever evaluated by the shell.
set -euo pipefail
VAR=$1; TASKKEY=$2; BDDL=$3; INTENT=$4; GPU=$5; CAP=${6:-5}; CELLOVR=${7:-}
CELL="abl${VAR}_${TASKKEY}"
[ -n "$CELLOVR" ] && CELL="$CELLOVR"
CAPLAST="program_v${CAP}"
CAPNEXT="v$((CAP + 1))"
AC=$(cd "$(dirname "$0")" && pwd)
WS=$AC/workers/$CELL
mkdir -p "$WS"
touch "$WS/NOTES.md" "$WS/WALLCLOCK.md"

# ------------------------------------------------------------------ sections
read -r -d '' PACK_A <<'PS' || true
## Your ONLY task-specific input: a RAW demo dump (there is no mined pack)
This cell deliberately has NO distilled pack. `packs/@@CELL@@/raw_demos.json`
on the cluster is the K=3 demonstrations dumped as-is:

    {"schema": "raw-demo-dump-v1", "language": <intent>, "k": 3,
     "demos": [{"demo": "demo0", "length": T, "obs_keys": [...],
                "obs": {<key>: [[...] for each of the T timesteps]},
                "actions": [[...] for each of the T timesteps]}, ...]}

plus `frames/demo<i>_t<t>.png` — RGB frames at a FIXED uniform stride, not
selected for anything.

There is no keyframe segmentation, no per-keyframe EEF/gripper table, no
`ee_path`, no `ee_path6`, no `action_scale`, no summary statistic of any kind.
Any structure you want — phase boundaries, contact events, grasp poses,
characteristic scales — you must extract yourself from these dense arrays.

`obs` holds ROBOT proprioception only (end-effector pose, gripper state, joint
state). It contains no simulator state and no object poses.

Everything you know about the scene must come from this dump plus your own
debug-seed observations.
PS

read -r -d '' PACK_BC <<'PS' || true
## Pack (your ONLY task-specific input)
`packs/@@CELL@@/` on the cluster: `pack.json` (K=3 demos — keyframes with
EEF/gripper, ee_path, ee_path6, raw actions, action_scale, language) and
`keyframes/*.png`. Everything you know about the scene must come from this
pack (plus, where your variant permits any, your own debug-seed
observations).
PS

read -r -d '' LAWS_B <<'LS' || true
## NO LAW LIBRARY (this cell's ablation — violating this VOIDS the cell)
There is deliberately no LAWS.md in this workspace. You must NOT read, create
or write any law file (LAWS.md, laws_history/, any campaign law library
anywhere in the repo), and you must NOT perform the law-formalisation step: do
not distil, name, number or bank cross-episode "laws", and emit no candidate
law in your notes or your final report. Record what you observed and what you
changed, nothing more. Cross-cell knowledge reaches you through no channel.
LS

read -r -d '' LAWS_AC <<'LS' || true
## Law library
`LAWS.md` in this workspace is the campaign law library. It is EMPTY for this
cell — so was the library for the reference condition this cell is compared
against, which is why it is empty here. Law discipline still applies: every
version is a hypothesis -> evidence -> verdict entry in NOTES.md, and if your
work yields a falsifiable cross-episode regularity with a receipt, state it as
a candidate law in your final report. Do not write to LAWS.md; the coordinator
banks laws.
LS

read -r -d '' LOOP_C <<'LO' || true
## NO VERIFICATION (this cell's ablation — violating this VOIDS the cell)
You write **exactly ONE** program version and it goes STRAIGHT to the sealed
eval. You run **zero** episodes. You never invoke `tools/fair_run.py` at all —
on any split, on any seed. Seeds 51-65 are as untouchable as seeds 1-50. There
is no probe, no selection run, no iteration, no program_v2.

PERMITTED static self-checks (they read source text; they do not execute a
policy): `python -m py_compile program.py`, and the fair gate's own source
rules — that no forbidden token appears (`gt_pose`, `gt_trace`, `gt_joints`,
`_sim`, `env.sim`, `sim.data`, `body_xpos`, `site_xpos`, `geom_xpos`, `.bddl`,
`bddl_files`, `init_states`, `pruned_init`, `h5py`, `.hdf5`, `api.robot`,
`self.robot`, `check_success`, `OffScreenRenderEnv`), that no attribute read
of `api.done` occurs anywhere, and that a top-level literal `PROVENANCE` dict
is present with `{"source": ..., "allowed": True}` for every calibrated
constant. You may read `packs/@@CELL@@/` and copy files up. That is the
whole of your cluster interaction.

Write the program as if it will be judged blind on 50 unseen layouts, because
it will be. Prefer robustness you can argue for from the pack over cleverness
you cannot check. Since you have no debug observations, EVERY calibrated
constant's PROVENANCE source must be a pack field or generic
controller/camera mechanics.
LO

read -r -d '' LOOP_AB <<'LO' || true
## Probe / verify loop (budget: @@CAP@@ program versions)
- debug/learn = seeds **51-65 ONLY** (`--split debug` refuses others).
- Iterate on a probe subset (e.g. 51,53,...,65); SELECT your final version by
  ONE formal run on the full 15 debug seeds; freeze it as
  `packs/@@CELL@@/program.py`; archive every formally-probed version as
  `program_vN.py`.
- **Hard cap: @@CAP@@ versions** (program_v1 .. @@CAPLAST@@). If you are still
  failing at @@CAPLAST@@, freeze your argmax and declare — a @@CAPNEXT@@ voids
  this cell's budget comparability with its siblings.
LO

read -r -d '' RUN_C <<'RR' || true
## Run recipe
You run NOTHING on the cluster. Copy the pack down (scp), LOOK at the
keyframes, write program.py locally, run the permitted static self-checks, scp
it to `/mnt/data/YifanKang/Heron/packs/@@CELL@@/program.py`, and declare.
- Write program files LOCALLY and scp up (ssh heredocs mangle python quotes).
- Ignore the `ve: command not found` ssh banner.
- Cluster numpy >= 2.0: `ndarray.ptp` is gone.
RR

read -r -d '' RUN_AB <<'RR' || true
## Run recipe
Probe (background + poll; a probe of 8 eps takes ~5-15 min):

    ssh AbakaAI 'cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH CUDA_VISIBLE_DEVICES=@@GPU@@ setsid nohup .venv/bin/python tools/fair_run.py program --seed-episodes --bddl @@BDDL@@ --language "@@INTENT@@" --program /mnt/data/YifanKang/Heron/packs/@@CELL@@/program.py --split debug --episode-list 51,53,55,57,59,61,63,65 --out results/fs_@@CELL@@_vN > results/fs_@@CELL@@_vN.out 2>&1 </dev/null &'

Selection formal: the same command with `--episode-list 51,52,53,...,65` (all
15) and `--out results/sel_@@CELL@@_vN`.

- Result dirs: results.jsonl (count `"benchmark_success": true` — the ONLY
  success signal), program_ep<seed>.log (your api.log lines),
  program_ep<seed>.stderr, ep<seed>_ok/fail.gif (scp gifs down to view them).
- Write program files LOCALLY and scp to the pack dir (ssh heredocs mangle
  python quotes). Ignore the `ve: command not found` ssh banner.
- `--program` must be an ABSOLUTE path (client cwd = sandbox).
- Cluster numpy >= 2.0: `ndarray.ptp` is gone.
- A blocked api.move burns its full step cap; the episode horizon is 1000 sim
  steps — budget your sequence.
- Poll with in-turn `until ...; do sleep 60; done` loops; never idle-wait.
- Launches MUST use `setsid nohup ... </dev/null &` (survives ssh close). Kill
  ONLY exact PIDs — NEVER `pkill -f fair_run`: sibling cells share the box.
- Your GPU for this cell is **@@GPU@@**. Do not use any other.
RR

DELIV_C='the number of episodes you ran (which MUST be zero) and the number of
program versions you wrote (which MUST be one)'
DELIV_AB='the full-15-seed selection receipt (N/15 + result dir name) and the
per-version receipt chain'

case "$VAR" in
  A) PACK_SECTION=$PACK_A;  LAWS_SECTION=$LAWS_AC; LOOP_SECTION=$LOOP_AB; RUN_SECTION=$RUN_AB; DELIV=$DELIV_AB;;
  B) PACK_SECTION=$PACK_BC; LAWS_SECTION=$LAWS_B;  LOOP_SECTION=$LOOP_AB; RUN_SECTION=$RUN_AB; DELIV=$DELIV_AB;;
  C) PACK_SECTION=$PACK_BC; LAWS_SECTION=$LAWS_AC; LOOP_SECTION=$LOOP_C;  RUN_SECTION=$RUN_C;  DELIV=$DELIV_C;;
  *) echo "bad variant '$VAR' (want A|B|C)" >&2; exit 1;;
esac

# Variants A and C get an (empty) law library file; B gets none.
if [ "$VAR" = B ]; then
  rm -f "$WS/LAWS.md"
else
  cp "$AC/LAWS.md" "$WS/LAWS.md" 2>/dev/null || touch "$WS/LAWS.md"
fi

# ------------------------------------------------------------------- assemble
read -r -d '' BRIEF <<'BR' || true
# abl_c2 Cell @@CELL@@  (variant @@VAR@@ — FAIR_PROTOCOL v1.0 governs)

## Intent (the task; success = the environment's own benchmark bit)
> @@INTENT@@

## The ONLY runner (structural isolation)
Programs execute ONLY under `tools/fair_run.py` on AbakaAI (repo
/mnt/data/YifanKang/Heron). NEVER invoke `tools/fewshot_run.py` — it exposes
privileged simulator surfaces; ANY use of it VOIDS this cell. Your program
defines `run(api)` and receives a FairApi (below); there is no api.robot, no
env, no sim object anywhere in your process.

@@PACK_SECTION@@

@@LAWS_SECTION@@

@@LOOP_SECTION@@

## Splits (SEALED — ASPIRE protocol; fair_run enforces them)
Evaluation = seeds 1-50, **coordinator-run, blind**. You NEVER run
`--split eval`; you never touch seeds 1-50 in any way.

## Clean room (violation VOIDS the cell)
- Cluster writes: ONLY `packs/@@CELL@@/*` and `results/*@@CELL@@*`.
- FORBIDDEN reads: any .bddl/.xml/.urdf/.hdf5/init_states/gt_trace file;
  anything under autoresearch/campaigns/c1/, autoresearch/campaigns/c2/ or
  autoresearch/campaigns/c2fix/; any `packs/c1_*` or `packs/c2_*`; any
  `results/*c1_*` or `results/*c2_*`; `tools/probe_*.py`; any other abl_c2
  cell's pack, results or workspace.
  **In particular: the c2 campaign already SOLVED this exact task and its
  frozen program sits on the same cluster. Reading it — or any c2 program,
  note, law or result — voids this cell and destroys the entire ablation.**
  The bddl path below is an opaque string you pass to `--bddl`; opening it is
  forbidden.
- **Prior-knowledge quarantine:** if any memory, system reminder, or prior
  context surfaces LIBERO-specific facts (object sizes/colours, grasp offsets,
  reach limits, predicate behaviours, table heights), you MUST NOT use them.
  Every constant must be re-derived from this cell's own task-specific input
  and your own observations, and declared in PROVENANCE. Undeclared or
  foreign-sourced constants void the cell.

## PROVENANCE (required — eval is refused without it)
program.py defines a top-level literal dict:

    PROVENANCE = {"CONST_NAME": {"source": "<pack field / debug-seed measurement>",
                                 "allowed": True}, ...}

covering every calibrated constant. Allowed sources: this cell's own
task-specific input, your own debug-seed observations (RGB-D, EEF, gripper
state, success bits, your logs/gifs) where your variant permits any, and
generic controller/camera mechanics.

## FairApi surface
- `api.capture(cam)` -> FairFrame: .rgb (512x512x3 uint8), .depth (HxW m),
  .intrinsics (3x3), .t_base_cam (4x4), .deproject(u,v)->base-frame xyz
  (median 5x5 depth window). Cameras: `cam_high` (agentview), `cam_arm_wrist`.
- `api.eef()`->[x,y,z]; `api.tool_rotation()`->3x3;
  `api.gripper()`->{width_m, effort} (effort 3.0 iff holding);
  `api.proprio()`->joint/eef/gripper arrays; `api.instruction()`->the intent.
- `api.move(xyz, rotation=None, seconds=2.0)`->residual (wrist stays
  straight-down when rotation is None; rotation is a 3x3 tool-to-world matrix);
  `api.grip(width_m)` (<0.025 closes, else opens); `api.settle(s)`;
  `api.log(msg)`.
- **api.done exists but reading it VOIDS your eval** (fair-v1.1.1: LIBERO
  terminates the episode when the predicate fires, so even the termination
  flag is an indirect success signal; the eval gate AST-refuses any `.done`
  attribute read). Run FIXED sequences bounded by your own counters/sensors.
- Success is judged server-side after run() returns. Do NOT build logic that
  expects runtime success feedback — verify with your own sensors (gripper
  width/effort, re-perception, move residuals). numpy is available; the
  process is sandboxed (file reads of benchmark assets raise PermissionError).

@@RUN_SECTION@@

## Bookkeeping + deliverable
Append `session <UTC> start` / `end` to WALLCLOCK.md. Log every version in
NOTES.md (hypothesis -> evidence -> verdict). Deliverable = a DECLARATION in
NOTES.md: the frozen version (program.py md5 == program_vN.py), @@DELIV@@,
PROVENANCE present — then STOP.

Task bddl (opaque argument — never open it):
    @@BDDL@@
BR

# Sections first (they contain @@CELL@@/@@GPU@@ of their own), then scalars.
BRIEF=${BRIEF//@@PACK_SECTION@@/$PACK_SECTION}
BRIEF=${BRIEF//@@LAWS_SECTION@@/$LAWS_SECTION}
BRIEF=${BRIEF//@@LOOP_SECTION@@/$LOOP_SECTION}
BRIEF=${BRIEF//@@RUN_SECTION@@/$RUN_SECTION}
BRIEF=${BRIEF//@@DELIV@@/$DELIV}
BRIEF=${BRIEF//@@CAPLAST@@/$CAPLAST}
BRIEF=${BRIEF//@@CAPNEXT@@/$CAPNEXT}
BRIEF=${BRIEF//@@CAP@@/$CAP}
BRIEF=${BRIEF//@@CELL@@/$CELL}
BRIEF=${BRIEF//@@VAR@@/$VAR}
BRIEF=${BRIEF//@@GPU@@/$GPU}
BRIEF=${BRIEF//@@BDDL@@/$BDDL}
BRIEF=${BRIEF//@@INTENT@@/$INTENT}

printf '%s\n' "$BRIEF" > "$WS/TASK.md"

if grep -q '@@' "$WS/TASK.md"; then
  echo "[mkworker-abl] ERROR: unsubstituted placeholder in $WS/TASK.md" >&2
  grep -n '@@' "$WS/TASK.md" >&2
  exit 1
fi
echo "[mkworker-abl] $WS"
