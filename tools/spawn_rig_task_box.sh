#!/bin/bash
# Launch a headless task-workspace agent ON THE RIG BOX (rtx5090b) — no ssh
# layer between the agent and the hardware. claude lives on the box
# (~/.local/bin/claude); auth = the box repo's .env or the box's claude login. The Mac-side workspace is rsynced over first; the
# transcript stays on the box at the workspace dir.
#
#   tools/spawn_rig_task_box.sh <task_name>
set -e
TASK=${1:?usage: spawn_rig_task_box.sh <task_name>}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/autoresearch/tasks/$TASK"
[ -f "$SRC/TASK.md" ] || { echo "no TASK.md in $SRC" >&2; exit 1; }
# A workspace without .claude/settings.json ships an agent that cannot read the
# repo or run the rig python. It does not error — it silently lacks permission,
# reads what it can, and stops before writing a program. That cost one launch on
# 2026-08-24 (rig_cup_stack), and the symptom looked like a bad brief.
if [ ! -f "$SRC/.claude/settings.json" ]; then
  echo "refusing to launch: $SRC/.claude/settings.json is missing." >&2
  echo "  Without it the agent has no Bash/Read permission on the box and will" >&2
  echo "  stall before touching the robot. Create it, e.g.:" >&2
  echo '    mkdir -p "'"$SRC"'/.claude" && cat > "'"$SRC"'/.claude/settings.json" <<JSON' >&2
  echo '    {"permissions":{"allow":["Bash","Read","Edit","Write","Grep","Glob","TodoWrite","WebFetch"]}}' >&2
  echo '    JSON' >&2
  exit 1
fi
DST="/mnt/data1/aloha/agent_tasks/$TASK"
ssh rtx5090b "mkdir -p '$DST'"
rsync -a --update --exclude 'transcript_*' "$SRC/" "rtx5090b:$DST/"
STAMP=$(date +%Y%m%d_%H%M%S)
ssh rtx5090b "bash -s" <<REMOTE
export PATH="\$HOME/.local/bin:\$PATH"
# Credential comes from the BOX repo's .env (API key or subscription token),
# else the box's own 'claude login'. Never from the launching Mac.
# (No backticks inside this unquoted heredoc: they run ON THE MAC at build time.)
if [ -f /mnt/data1/aloha/Heron/tools/claude_auth.sh ]; then
  CLAUDE_AUTH_ROOT=/mnt/data1/aloha/Heron source /mnt/data1/aloha/Heron/tools/claude_auth.sh
fi
# headless sessions can't click the trust dialog — grant it up front, or the
# workspace's settings.json permission allowances are silently ignored
python3 - <<PYEOF
import json, pathlib
f = pathlib.Path.home() / ".claude.json"
d = json.loads(f.read_text()) if f.is_file() else {}
d.setdefault("projects", {}).setdefault("$DST", {})["hasTrustDialogAccepted"] = True
f.write_text(json.dumps(d, indent=2))
PYEOF
cd "$DST"
claude --version > "transcript_$STAMP.meta" 2>&1 || true
echo "auth=\${CLAUDE_AUTH_MODE:-machine-login}" >> "transcript_$STAMP.meta"
setsid nohup claude -p "Read TASK.md and CLAUDE.md in this directory, then begin the session. Work until a TASK.md stop condition is met and documented." \
  --output-format stream-json --verbose \
  > "transcript_$STAMP.jsonl" 2> "transcript_$STAMP.err" < /dev/null &
echo \$! > launch.pid
echo "[spawn-box] $TASK launched on rtx5090b (pid \$(cat launch.pid)); transcript: $DST/transcript_$STAMP.jsonl"
REMOTE
