#!/bin/bash
# rd1 scheduler: keep MAX_CONCURRENT worker sessions alive until every cell
# has run once. Resumable -- a cell with a .done marker is skipped, so the
# scheduler can be killed and restarted without losing work.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
MAX=${MAX_CONCURRENT:-6}
LOG=$HERE/scheduler.log
say(){ echo "$(date '+%F %T') $*" | tee -a "$LOG"; }

say "scheduler start (max concurrent $MAX)"
for ws in "$HERE"/workers/*/; do
  cell=$(basename "$ws")
  if [ -f "$ws/.done" ]; then say "skip $cell (done)"; continue; fi
  case " ${SKIP:-} " in *" $cell "*) say "skip $cell (already running)"; continue;; esac
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAX" ]; do sleep 20; done
  say "launch $cell"
  (
    bash "$ROOT/tools/ar_launch_worker.sh" "$ws" 250 >/dev/null 2>&1
    touch "$ws/.done"
    echo "$(date '+%F %T') finished $cell" >> "$LOG"
  ) &
  sleep 5
done
wait
say "ALL CELLS DONE"
