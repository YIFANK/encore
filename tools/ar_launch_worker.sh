#!/bin/bash
# Launch a headless worker session in an ABSOLUTE workspace path (campaign
# variant of ar_launch.sh; the coordinator calls this).
#   tools/ar_launch_worker.sh <workspace_dir> [max_turns]
set -euo pipefail
WS=$1; TURNS=${2:-250}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
# Credential: API key, subscription token, or machine login (tools/claude_auth.sh).
source "$ROOT/tools/claude_auth.sh"
cd "$WS"
STAMP=$(date +%Y%m%d_%H%M%S)
echo "session $STAMP start $(date -u +%FT%TZ)" >> WALLCLOCK.md
claude --version > "transcript_$STAMP.meta" 2>&1 || true
echo "auth=$CLAUDE_AUTH_MODE model=${AR_MODEL:-}" >> "transcript_$STAMP.meta"
claude -p "$(cat TASK.md)" \
  --model "${AR_MODEL:-opus}" \
  --output-format stream-json --verbose \
  --max-turns "$TURNS" \
  > "transcript_$STAMP.jsonl" 2> "transcript_$STAMP.err"
echo "session $STAMP end   $(date -u +%FT%TZ) exit=$?" >> WALLCLOCK.md
