#!/bin/bash
# Launch a headless autoresearch session in a task workspace.
#   tools/ar_launch.sh <name> [max_turns]
# Transcript (stream-json) and wall-clock go into the workspace — they are
# paper artifacts. Uses the local `claude` CLI and whatever model the
# user's subscription session defaults to; record `claude --version` and
# the model in the transcript header for reproducibility.
set -euo pipefail
NAME=$1; TURNS=${2:-250}
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WS=$ROOT/autoresearch/tasks/$NAME
# Headless auth: the long-lived token lives in .env (git-ignored). Never
# echo it; export only into this process environment.
# Credential: API key, subscription token, or machine login (tools/claude_auth.sh).
source "$ROOT/tools/claude_auth.sh"
cd "$WS"
STAMP=$(date +%Y%m%d_%H%M%S)
echo "session $STAMP start $(date -u +%FT%TZ)" >> WALLCLOCK.md
claude --version > "transcript_$STAMP.meta" 2>&1 || true
echo "auth=$CLAUDE_AUTH_MODE model=${AR_MODEL:-}" >> "transcript_$STAMP.meta"
claude -p "$(cat TASK.md)" \
  --model "${AR_MODEL:-sonnet}" \
  --output-format stream-json --verbose \
  --max-turns "$TURNS" \
  > "transcript_$STAMP.jsonl" 2> "transcript_$STAMP.err"
echo "session $STAMP end   $(date -u +%FT%TZ) exit=$?" >> WALLCLOCK.md
