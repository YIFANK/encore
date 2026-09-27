#!/bin/bash
# Resume a worker session that returned without a DECLARATION (mid-probe), with a
# coordinator nudge. Usage: resume_worker.sh <workspace_dir> <session_id> [max_turns]
set -euo pipefail
WS=$1; SID=$2; TURNS=${3:-250}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
source "$ROOT/tools/claude_auth.sh"
cd "$WS"; rm -f .done
STAMP=$(date +%Y%m%d_%H%M%S)
echo "session $STAMP resume $SID start $(date -u +%FT%TZ)" >> WALLCLOCK.md
NUDGE="${NUDGE:-Coordinator note: your previous session ended before a DECLARATION was written (it stopped in the middle of the development loop, not by your decision). Any probe or selection run you had launched has finished; read its receipt under results/ on the cluster, continue the development loop under exactly the same rules as TASK.md, and end this session with a DECLARATION in NOTES.md (frozen argmax version, full-15 selection receipt, receipt chain) or, if mechanism-blocked after honest effort, a documented mechanism-gap stop that still declares the argmax version. Do not stop before that.}"
claude -p "$NUDGE" --resume "$SID" --model "${AR_MODEL:-opus}" --output-format stream-json --verbose --max-turns "$TURNS" > "transcript_$STAMP.jsonl" 2> "transcript_$STAMP.err" || true
echo "session $STAMP end   $(date -u +%FT%TZ)" >> WALLCLOCK.md
touch .done; echo "$(date '+%F %T') finished $(basename "$WS") (resume)" >> "$(dirname "$WS")/../scheduler.log"
