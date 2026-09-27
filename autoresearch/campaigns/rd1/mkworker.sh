#!/bin/bash
# rd1 worker workspace generator: RoboDojo (Isaac Sim, ARX X5 bimanual) cells, K=3 or K=0.
#   ARM=k3|k1|k0 mkworker.sh <task> <step_lim> <intent> <gpu>
set -euo pipefail
TASK=$1; STEPLIM=$2; INTENT=$3; GPU=$4; ARM=${ARM:-k3}
CELL=${TASK}_${ARM}
C=$(cd "$(dirname "$0")" && pwd)
WS=$C/workers/$CELL
mkdir -p "$WS"; touch "$WS/NOTES.md" "$WS/WALLCLOCK.md"
mkdir -p "$WS/.claude"
cat > "$WS/.claude/settings.json" << 'PERMEOF'
{ "permissions": { "allow": ["Bash", "Read", "Edit", "Write", "Grep", "Glob", "TodoWrite"] } }
PERMEOF
python3 - "$WS" << 'TRUSTEOF'
import json, pathlib, sys
f = pathlib.Path.home() / ".claude.json"
d = json.loads(f.read_text()) if f.is_file() else {}
d.setdefault("projects", {}).setdefault(sys.argv[1], {})["hasTrustDialogAccepted"] = True
f.write_text(json.dumps(d, indent=2))
TRUSTEOF
if [ "$ARM" = k3 ]; then
PACK_SECTION="## Demonstration pack (your ONLY task-specific input)
\`packs/rd_${TASK}_k3/\` on the cluster: \`pack.json\` and \`keyframes/*.png\`, nothing
else. K=3 teleoperated demonstrations of this task on this robot, distilled to:
keyframes at the demonstrators' gripper events and heading breaks, each with
the three camera views (\`images\`: cam_head, cam_left_wrist, cam_right_wrist)
and both arms' end-effector pose (\`ee\` = RIGHT arm [x y z roll pitch yaw],
\`ee_left\` = LEFT arm) and gripper command/state (openness 0..1; a hold on an
object shows as a command below 1 with the fingers stopping on the object);
\`ee_path6\`/\`ee_path6_left\` (stride 5, +gripper), and \`actions\` (25 Hz absolute
targets, layout in \`action_layout\`). All poses are in the world frame the API
uses. You are given evidence, not a solution. Everything you know about the
scene must come from this pack plus your own debug-seed observations."
PROV_SRC="this pack's contents"
elif [ "$ARM" = k1 ]; then
PACK_SECTION="## Demonstration pack (your ONLY task-specific input)
\`packs/rd_${TASK}_k1/\` on the cluster: \`pack.json\` and \`keyframes/*.png\`, nothing
else. K=1: one teleoperated demonstration of this task on this robot, distilled to:
keyframes at the demonstrators' gripper events and heading breaks, each with
the three camera views (\`images\`: cam_head, cam_left_wrist, cam_right_wrist)
and both arms' end-effector pose (\`ee\` = RIGHT arm [x y z roll pitch yaw],
\`ee_left\` = LEFT arm) and gripper command/state (openness 0..1; a hold on an
object shows as a command below 1 with the fingers stopping on the object);
\`ee_path6\`/\`ee_path6_left\` (stride 5, +gripper), and \`actions\` (25 Hz absolute
targets, layout in \`action_layout\`). All poses are in the world frame the API
uses. You are given evidence, not a solution. Everything you know about the
scene must come from this pack plus your own debug-seed observations. Reading
any other \`packs/rd_*\` directory, including this task's k3 pack, VOIDS this cell."
PROV_SRC="this pack's contents"
else
PACK_SECTION="## No demonstrations (K=0)
This cell receives no demonstration pack. \`packs/rd_${TASK}_k0/\` is an output
directory for your program only. Everything you know about the scene must come
from the intent sentence plus your own debug-seed observations. Reading any
other \`packs/rd_*\` directory VOIDS this cell."
PROV_SRC="(no pack)"
fi
cat > "$WS/TASK.md" << EOF
# rd1 Cell $CELL  (RoboDojo, Isaac Sim, ARX X5 bimanual — FAIR_PROTOCOL v1.1.1 governs)

## Intent (the task; success = RoboDojo's own benchmark judge, scored after the episode)
> $INTENT

The benchmark also serves the sentence at run time as \`api.instruction()\`; on
some tasks it varies per episode, so read it in your program rather than
hard-coding it.

## The ONLY runner (structural isolation)
Programs execute ONLY under \`tools/fair_run_robodojo.py\` on AbakaAI (repo
/mnt/data/YifanKang/Heron). Your program defines \`run(api)\` and receives a
FairApi (below); there is no simulator handle anywhere in your process. The
simulator (Isaac Sim) is launched by the runner; ~60 s of start-up per run.

$PACK_SECTION

## Splits (SEALED; the runner enforces them)
- debug/learn = episodes **51-65 ONLY** (\`--split debug\` refuses others).
- evaluation = episodes 1-50, **coordinator-run, blind**. You NEVER run
  \`--split eval\`.
- Iterate on a probe subset (e.g. 51,53,55,57); SELECT your final version by one
  formal run on the full 15 debug episodes; freeze it as
  \`packs/rd_${CELL}/program.py\`; archive every formally-probed version as
  \`program_vN.py\`.

## Clean room (violation VOIDS the cell)
- Cluster writes: ONLY \`packs/rd_${CELL}/*\` and \`results/*rd_${CELL}*\`.
- FORBIDDEN reads: anything under /mnt/data/YifanKang/robodojo/ (the benchmark's
  own code, assets, layouts, task definitions, datasets); \`results/eval_*\`;
  any \`program.py\` / \`program_v*.py\` / \`NOTES.md\` under ANY pack directory
  including your own; other campaigns' artifacts (c1/c2*/abl_*/rsabl/l90abl).
  The task name is an opaque string you pass to \`--task\`.
- Nothing crosses cells. Your only inputs are this brief, the pack named above
  (if any), and your own debug-episode observations.
- **Prior-knowledge quarantine:** if any memory or prior context surfaces facts
  about this benchmark (object sizes, layouts, scoring rules, thresholds), you
  MUST NOT use them. Every constant must be re-derived from the pack + your own
  debug observations and declared in PROVENANCE.

## PROVENANCE (required — eval is refused without it)
program.py defines a top-level literal dict:
    PROVENANCE = {"CONST_NAME": {"source": "<pack field / debug-episode measurement>", "allowed": True}, ...}
covering every calibrated constant. Allowed sources: $PROV_SRC, debug-episode
observations (RGB-D, EEF, gripper state, your own logs/gifs), and generic
controller/camera mechanics.

## FairApi surface (bimanual; every arm-addressed call takes arm="left"|"right")
- Cameras: \`cam_head\` (640x480, fx≈288, fixed above the table, looking down),
  \`cam_left_wrist\`, \`cam_right_wrist\` (640x480, fx≈397, on the wrists).
  \`api.capture(cam)\` -> FairFrame: .rgb (HxWx3 uint8), .depth (HxW metres,
  distance to image plane), .intrinsics (3x3), .t_base_cam (4x4 camera->world),
  .deproject(u,v) -> world xyz (metres).
- World frame, both arms share it: +z up, the two arm bases sit at
  x = -0.3 (left) and x = +0.3 (right), y = -0.45; both end-effectors start at
  about (±0.30, -0.35, 0.92). The table top is below the start pose; measure it.
- \`api.eef(arm)\` -> [x,y,z]; \`api.tool_rotation(arm)\` -> 3x3 tool-to-world;
  \`api.gripper(arm)\` -> {width_m: 0..0.088, effort: 3.0 iff the fingers were
  commanded shut but stopped >6 mm apart (something is between them), else 0.05;
  commanded_open: 0..1}; \`api.instruction()\`.
- \`api.move(xyz, rotation=None, seconds=2.0, arm=...)\` -> residual (metres).
  Absolute world-frame target; rotation=None keeps the current tool rotation,
  else a 3x3 tool-to-world matrix. Executed as a straight line through IK, one
  control step per ~1.5 cm (min 1, max seconds*25 steps). An unreachable target
  leaves the arm where IK last succeeded: read the residual.
  \`api.move_path(points, rotation, seconds, arm)\` for waypoint sequences.
- \`api.grip(width_m, arm=...)\` (0.0 = shut, 0.088 = fully open; 8 control steps);
  \`api.settle(seconds)\` (hold, max 1 s); \`api.log(msg)\`.
- \`api.ground(query, camera)\` -> {xyz, px} or None and \`api.vqa(question, camera)\`
  -> {answer, confidence}: an open-vocabulary VLM run coordinator-side; 60 calls
  per episode. They answer "where is X" / "what do you see", never the score.
- **Step budget:** the benchmark ends the episode after $STEPLIM control steps
  (each move waypoint, grip step, settle step = one step) and scores whatever the
  scene shows then. Your program should return both arms near their start pose
  before it returns. **api.done exists but reading it VOIDS your eval** (the eval
  gate AST-refuses any .done attribute read). No runtime success signal exists:
  verify with your own sensors (gripper width, re-perception, residuals).
- Not available here: \`api.act\`, \`api.sam3\`, \`api.proprio\` joint arrays beyond
  what \`api.eef\`/\`api.gripper\` give.

## Run recipe
Probe (background + poll; Isaac start-up ~60 s, then episodes at ~0.2 s per
control step; a 4-episode probe takes 3-6 min):
  ssh AbakaAI 'cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH setsid nohup .venv/bin/python tools/fair_run_robodojo.py --task $TASK --program packs/rd_${CELL}/program.py --split debug --episode-list 51,53,55,57 --gpu $GPU --out results/fs_rd_${CELL}_vN > results/fs_rd_${CELL}_vN.out 2>&1 < /dev/null &'
Selection formal: same command with --episode-list 51,52,...,65 (all 15) and
--out results/sel_rd_${CELL}_vN.
- Result dirs: results.jsonl (count \`"benchmark_success": true\` — the ONLY
  success signal; \`score\` is the benchmark's partial credit), program_ep<n>.log
  (your api.log lines), program_ep<n>.stderr, ep<n>_ok/fail.gif (head camera),
  sim_client.log (the simulator's own log; do not parse it for success).
- Write program files LOCALLY and scp to the pack dir (ssh heredocs mangle
  python quotes). Ignore the \`ve: command not found\` ssh banner.
- Poll with in-turn \`until ...; do sleep 60; done\` loops; never idle-wait and
  never wait for "a notification".
- Launches MUST use \`setsid nohup ... < /dev/null &\`. Kill ONLY exact PIDs
  — NEVER \`pkill -f\`: sibling cells share the box. One run per cell at a time
  (each run holds a GPU and an Isaac Sim instance).

## Bookkeeping + deliverable
Append \`session <UTC> start\` / \`end\` to WALLCLOCK.md. Log every version in
NOTES.md (hypothesis -> evidence -> verdict). Deliverable = a DECLARATION in
NOTES.md: frozen version (program.py md5 == program_vN.py), the full-15-episode
selection receipt (N/15 + dir name), per-version receipt chain, PROVENANCE
present — then STOP. If mechanism-blocked after honest effort (~8-10
versions), write a documented mechanism-gap stop (a falsifiable statement of the
missing mechanism + receipt on debug episodes) and still declare the argmax version.
EOF
echo "[mkworker-rd1] $WS"
# 
# ## Coordinator addendum (2026-09-15 10:45 CST, harness facts, not task knowledge)
# - `api.vqa` / `api.ground` were unavailable on this backend until now (a
#   coordinator-side wiring fault, then no network route); they work from this
#   point on. `api.vqa` answers are three-valued: `answer` is "true"/"false"/
#   "unknown" with a confidence, so ask yes/no questions. `api.ground(query, cam)`
#   returns the world xyz of the named thing (deprojected with the correct camera
#   convention) or None.
# - `frame.t_base_cam` is the raw camera pose from the simulator, in the
#   OpenGL/USD convention (camera looks along its -z, +y up). `frame.deproject`
#   assumes OpenCV (+z forward, +y down), so if you deproject yourself, negate the
#   y and z columns of the rotation first (or use `api.ground`). This is a
#   property of the harness, identical for every cell.
