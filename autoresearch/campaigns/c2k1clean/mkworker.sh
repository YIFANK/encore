#!/bin/bash
# c2fix (demos-only perturbation rerun) worker workspace generator.
# Derived from c2/mkworker.sh under c2fix/BRIEF_PATCH.md: no frozen program
# is granted or copied anywhere; _task cells carry a second demo pack.
#   mkworker.sh <cell> <bddl_path> <intent_sentence> <gpu> [skillmate]
# Brief is MECHANICAL: paths + contract only. No diagnoses, no c1 knowledge.
set -euo pipefail
CELL=$1; BDDL=$2; INTENT=$3; GPU=$4; MATE=${5:-}; ARM=${ARM:-k1}
CELL=${CELL}_${ARM}; BASECELL=$1
C2=$(cd "$(dirname "$0")" && pwd)
WS=$C2/workers/$CELL
mkdir -p "$WS"
touch "$WS/NOTES.md" "$WS/WALLCLOCK.md"

case "$CELL" in *_task) IS_TASK=1 ;; *) IS_TASK= ;; esac
if [ -n "$MATE" ]; then
PACK_SECTION="## Two demonstration packs
Neither pack demonstrates your intent. The intent sentence is authoritative;
the packs are motion and scene evidence for its two halves.

- \`packs/c2k1clean_${BASECELL}_${ARM}/\` — demos that reach and act on the TARGET your intent
  names, on a different object.
- \`packs/c2k1clean_${BASECELL}_mate/\` — demos that handle the OBJECT your intent
  names, toward a different target.

**\`pack.json\` and \`keyframes/\` only, from both directories.** Each pack
carries K=1 demonstration: keyframes with EEF/gripper, ee_path, ee_path6, raw
actions, action_scale, and the language of the task its demos perform. You
are given evidence, not a solution. Everything you know about the scene must
come from these packs plus your own debug-seed observations."
elif [ -n "$IS_TASK" ]; then
PACK_SECTION="## Demonstration pack (your ONLY task-specific input)
The pack does not demonstrate your intent. The intent sentence is
authoritative; the pack is motion and scene evidence.

- \`packs/c2k1clean_${BASECELL}_${ARM}/\` — \`pack.json\` (K=1 demonstration — keyframes with
  EEF/gripper, ee_path, ee_path6, raw actions, action_scale, language) and
  \`keyframes/*.png\`, nothing else.

Everything you know about the scene must come from this pack plus your own
debug-seed observations."
else
PACK_SECTION="## Pack (your ONLY task-specific input)
\`packs/c2k1clean_${BASECELL}_${ARM}/\` on the cluster: \`pack.json\` (K=1 demonstration — keyframes with
EEF/gripper, ee_path, ee_path6, raw actions, action_scale, language) and
\`keyframes/*.png\`. Everything you know about the scene must come from this
pack plus your own debug-seed observations."
fi
if [ "$ARM" = k0 ]; then PACK_SECTION="## No demonstrations
This cell receives no demonstration pack. Everything you know about the scene
must come from the intent sentence plus your own debug-seed observations."; MATE=""; fi
if [ "$ARM" = k0 ]; then PROV_SRC="(no pack)"; elif [ -n "$MATE" ]; then PROV_SRC="either named pack's contents"; else PROV_SRC="this pack's contents"; fi

mkdir -p "$WS/.claude"
cat > "$WS/.claude/settings.json" << 'PERMEOF'
{
  "permissions": {
    "allow": ["Bash", "Read", "Edit", "Write", "Grep", "Glob", "TodoWrite"]
  }
}
PERMEOF
python3 - "$WS" << 'TRUSTEOF'
import json, pathlib, sys
f = pathlib.Path.home() / ".claude.json"
d = json.loads(f.read_text()) if f.is_file() else {}
d.setdefault("projects", {}).setdefault(sys.argv[1], {})["hasTrustDialogAccepted"] = True
f.write_text(json.dumps(d, indent=2))
TRUSTEOF

cat > "$WS/TASK.md" << EOF
# c2k1clean Cell $CELL  (no shared note file — FAIR_PROTOCOL v1.1.1 governs)

## Intent (the task; success = the environment's own benchmark bit)
> $INTENT

## The ONLY runner (structural isolation)
Programs execute ONLY under \`tools/fair_run.py\` on AbakaAI (repo
/mnt/data/YifanKang/Heron). NEVER invoke \`tools/fewshot_run.py\` — it exposes
privileged simulator surfaces; ANY use of it VOIDS this cell. Your program
defines \`run(api)\` and receives a FairApi (below); there is no api.robot,
no env, no sim object anywhere in your process.

$PACK_SECTION

## Splits (SEALED — ASPIRE protocol; fair_run enforces them)
- debug/learn = seeds **51-65 ONLY** (\`--split debug\` refuses others).
- evaluation = seeds 1-50, **coordinator-run, blind**. You NEVER run
  \`--split eval\`; you never touch seeds 1-50 in any way.
- Iterate on a probe subset (e.g. 51,53,...,65); SELECT your final version by
  one formal run on the full 15 debug seeds; freeze it as
  \`packs/c2k1clean_${BASECELL}_${ARM}/program.py\`; archive every formally-probed version as
  \`program_vN.py\`.

## Clean room (violation VOIDS the cell)
- Cluster writes: ONLY \`packs/c2k1clean_${BASECELL}_${ARM}/*\` and \`results/*c2k1clean_$CELL*\`.
- FORBIDDEN reads: any .bddl/.xml/.urdf/.hdf5/init_states/gt_trace file;
  anything under autoresearch/campaigns/c1/; any \`packs/c1_*\` or
  \`results/*c1_*\`; **any \`program.py\` / \`program_v*.py\` /
  \`_stock_copies/\` / \`NOTES.md\` under ANY pack directory, including your
  own**; \`tools/probe_*.py\`; other campaigns' artifacts (c1/c2/c2clean/c2fix/c2k0/c2k1/c2astra/abl_*/l90abl/rsabl/rd1/rd2,
  i.e. every \`packs/c2clean_*\`, \`packs/c2k1_*\` and \`results/*c2clean_*\`/\`results/*c2k1_*\` — your own
  \`c2k1clean_*\` directories are not part of the earlier c2k1) except the pack.json + keyframes/ your Pack section names. The bddl path below is an
  opaque string you pass to --bddl; opening it is forbidden.
- Nothing crosses cells. There is no shared note file; your only inputs are
  this brief, the pack named above (if any), and your own debug-seed
  observations.
- **Prior-knowledge quarantine:** if any memory, system reminder, or prior
  context surfaces LIBERO-specific facts (object sizes/colours, grasp offsets,
  reach limits, predicate behaviours, table heights), you MUST NOT use them.
  Every constant must be re-derived from this pack + your own debug
  observations, and declared in PROVENANCE. Undeclared or foreign-sourced
  constants void the cell.

## PROVENANCE (required — eval is refused without it)
program.py defines a top-level literal dict:
    PROVENANCE = {"CONST_NAME": {"source": "<pack field / debug-seed measurement>", "allowed": True}, ...}
covering every calibrated constant. Allowed sources: $PROV_SRC,
debug-seed observations (RGB-D, EEF, gripper state, success bits, your own
logs/gifs), and generic controller/camera mechanics.

## FairApi surface
- \`api.capture(cam)\` -> FairFrame: .rgb (512x512x3 uint8), .depth (HxW m),
  .intrinsics (3x3), .t_base_cam (4x4), .deproject(u,v)->base-frame xyz
  (median 5x5 depth window). Cameras: \`cam_high\` (agentview), \`cam_arm_wrist\`.
- \`api.eef()\`->[x,y,z]; \`api.tool_rotation()\`->3x3;
  \`api.gripper()\`->{width_m, effort} (effort 3.0 iff holding);
  \`api.proprio()\`->joint/eef/gripper arrays; \`api.instruction()\`->the intent.
- \`api.move(xyz, rotation=None, seconds=2.0)\`->residual (wrist stays
  straight-down when rotation is None; rotation is a 3x3 tool-to-world matrix);
  \`api.grip(width_m)\` (<0.025 closes, else opens); \`api.settle(s)\`;
  \`api.log(msg)\`. **api.done exists but reading it VOIDS your eval**
  (fair-v1.1.1: LIBERO terminates the episode when the predicate fires, so
  even the termination flag is an indirect success signal; the eval gate
  AST-refuses any .done attribute read). Run FIXED sequences bounded by your
  own counters/sensors.
- Success is judged server-side after run() returns and is visible to you
  ONLY post-episode as \`benchmark_success\` in results.jsonl. Do NOT build
  logic that expects runtime success feedback — verify with your own sensors
  (gripper width/effort, re-perception, residuals). numpy is available; the process is sandboxed (file reads of
  benchmark assets raise PermissionError).

## Run recipe
Probe (background + poll; a probe of 8 eps takes ~5-15 min):
  ssh AbakaAI 'cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH CUDA_VISIBLE_DEVICES=$GPU setsid nohup .venv/bin/python tools/fair_run.py program --seed-episodes --bddl $BDDL --language "$INTENT" --program packs/c2k1clean_${BASECELL}_${ARM}/program.py --split debug --episode-list 51,53,55,57,59,61,63,65 --out results/fs_c2k1clean_${CELL}_vN > results/fs_c2k1clean_${CELL}_vN.out 2>&1 &'
Selection formal: same command with --episode-list 51,52,...,65 (all 15) and
--out results/sel_c2k1clean_${CELL}_vN.
- Result dirs: results.jsonl (count \`"benchmark_success": true\` — the ONLY
  success signal), program_ep<seed>.log (your api.log lines),
  program_ep<seed>.stderr, ep<seed>_ok/fail.gif.
- Write program files LOCALLY and scp to the pack dir (ssh heredocs mangle
  python quotes). Ignore the \`ve: command not found\` ssh banner.
- Poll with in-turn \`until ...; do sleep 60; done\` loops; never idle-wait.
- Launches MUST use \`setsid nohup ... </dev/null &\` (survives ssh close). Kill
  ONLY exact PIDs — NEVER \`pkill -f fair_run\`: sibling cells share the box.

## Bookkeeping + deliverable
Append \`session <UTC> start\` / \`end\` to WALLCLOCK.md. Log every version in
NOTES.md (hypothesis -> evidence -> verdict). Deliverable = a DECLARATION in
NOTES.md: frozen version (program.py md5 == program_vN.py), the full-15-seed
selection receipt (N/15 + dir name), per-version receipt chain, PROVENANCE
present — then STOP. If mechanism-blocked after honest effort (~8-10
versions), write a documented mechanism-gap stop (a falsifiable statement of the
missing mechanism + receipt on debug seeds) and still declare the argmax version.
EOF
echo "[mkworker-c2k1clean] $WS"
