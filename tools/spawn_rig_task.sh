#!/bin/bash
# Launch a headless task-workspace agent session (run on the Mac).
#
#   tools/spawn_rig_task.sh <task_name>
#
# The workspace autoresearch/tasks/<task_name>/ must already hold TASK.md,
# CLAUDE.md, LAWS.md (the generator or a human writes them). The agent runs
# with the workspace's .claude/settings.json permissions and leaves a
# transcript beside the task files, same convention as rig_handover.
set -e
TASK=${1:?usage: spawn_rig_task.sh <task_name>}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIR="$ROOT/autoresearch/tasks/$TASK"
[ -f "$DIR/TASK.md" ] || { echo "no TASK.md in $DIR" >&2; exit 1; }
STAMP=$(date +%Y%m%d_%H%M%S)
cd "$DIR"
claude --version > "transcript_$STAMP.meta" 2>&1 || true
nohup claude -p "Read TASK.md and CLAUDE.md in this directory, then begin the session. Work until a TASK.md stop condition is met and documented." \
  --output-format stream-json --verbose \
  > "transcript_$STAMP.jsonl" 2> "transcript_$STAMP.err" < /dev/null &
echo $! > launch.pid
echo "[spawn] $TASK agent launched (pid $(cat launch.pid)); transcript: $DIR/transcript_$STAMP.jsonl"
