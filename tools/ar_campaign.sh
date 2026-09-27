#!/bin/bash
# One-command autonomous campaign: an unattended COORDINATOR session run in
# stateless rounds by this dumb loop. All decisions happen inside rounds
# (see autoresearch/COORDINATOR.md); this script only re-invokes and stops.
#
#   tools/ar_campaign.sh <campaign_name> [round_interval_seconds]
#
# Prep: create autoresearch/campaigns/<name>/TASK.md (the campaign brief)
# first. Stop anytime: touch autoresearch/campaigns/<name>/STOP
set -euo pipefail
NAME=$1; INTERVAL=${2:-900}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
C=$ROOT/autoresearch/campaigns/$NAME

[ "$(git -C "$ROOT" branch --show-current)" = "main" ] || \
  echo "[ar_campaign] WARNING: not on main ($(git -C "$ROOT" branch --show-current))"
[ -f "$C/TASK.md" ] || { echo "missing $C/TASK.md (the campaign brief)"; exit 1; }

mkdir -p "$C/workers" "$C/rounds"
cp "$ROOT/autoresearch/COORDINATOR.md" "$C/CLAUDE.md"
[ -f "$C/PROTOCOL.md" ] || cp "$ROOT/autoresearch/PROTOCOL.md" "$C/PROTOCOL.md"
touch "$C/LAWS.md" "$C/STATE.md" "$C/WALLCLOCK.md"
if [ ! -f "$C/.claude/settings.json" ]; then
  mkdir -p "$C/.claude"
  printf '{\n  "permissions": {\n    "allow": ["Bash", "Read", "Edit", "Write", "Grep", "Glob", "TodoWrite"]\n  }\n}\n' > "$C/.claude/settings.json"
fi

if [ -z "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -f "$ROOT/.env" ]; then
  export CLAUDE_CODE_OAUTH_TOKEN=$(grep -m1 '^CLAUDE_CODE_OAUTH_TOKEN=' "$ROOT/.env" | cut -d= -f2-)
fi

echo "[ar_campaign] $NAME starting; round every ${INTERVAL}s; stop: touch $C/STOP"
ROUND=0
while true; do
  [ -f "$C/STOP" ] && { echo "[ar_campaign] STOP file found; exiting"; break; }
  grep -q CAMPAIGN_COMPLETE "$C/STATE.md" 2>/dev/null && \
    { echo "[ar_campaign] CAMPAIGN_COMPLETE; exiting"; break; }
  ROUND=$((ROUND+1)); STAMP=$(date +%Y%m%d_%H%M%S)
  echo "round $ROUND $STAMP start $(date -u +%FT%TZ)" >> "$C/WALLCLOCK.md"
  ( cd "$C" && claude -p "$(cat TASK.md)" \
      --model "${AR_COORD_MODEL:-opus}" \
      --output-format stream-json --verbose \
      --max-turns 80 \
      > "rounds/round_${ROUND}_$STAMP.jsonl" 2> "rounds/round_${ROUND}_$STAMP.err" ) || true
  echo "round $ROUND $STAMP end   $(date -u +%FT%TZ)" >> "$C/WALLCLOCK.md"
  sleep "$INTERVAL"
done
