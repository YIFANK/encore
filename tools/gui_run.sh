#!/bin/bash
# Run a command inside Terminal.app so it inherits the GUI camera permission.
#
# macOS denies camera access to ssh sessions (TCC has no way to grant it), so
# anything that opens a camera must be launched from a GUI app. This drives
# Terminal.app via AppleScript, waits for the command to finish, and prints its
# output — making camera work drivable over ssh after all.
#
#   tools/gui_run.sh '.venv/bin/python -m heron.cli calibrate --config ... '
#   tools/gui_run.sh --timeout 600 'long running thing'
set -euo pipefail

TIMEOUT=300
if [ "${1:-}" = "--timeout" ]; then TIMEOUT="$2"; shift 2; fi
CMD="${1:?usage: gui_run.sh [--timeout SECONDS] '<command>'}"

LOG="/tmp/gui_run.$$.log"
DONE="/tmp/gui_run.$$.done"
# macOS ships bash 3.2, so no ${var@Q}; printf %q does the quoting.
WRAPPED="cd $(printf %q "$PWD"); { $CMD ; } > $LOG 2>&1; echo \$? > $DONE; exit"
# AppleScript string literal: escape backslashes first, then double quotes.
ESCAPED=$(printf '%s' "$WRAPPED" | sed -e 's/\\/\\\\/g' -e 's/"/\\"/g')

osascript -e "tell application \"Terminal\" to do script \"$ESCAPED\"" >/dev/null

elapsed=0
while [ ! -f "$DONE" ]; do
    sleep 1
    elapsed=$((elapsed + 1))
    if [ "$elapsed" -ge "$TIMEOUT" ]; then
        echo "--- TIMED OUT after ${TIMEOUT}s; output so far: ---"
        cat "$LOG" 2>/dev/null || true
        exit 124
    fi
done

status=$(cat "$DONE")
cat "$LOG"
rm -f "$LOG" "$DONE"
exit "$status"
