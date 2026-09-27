#!/bin/bash
# abox.sh — run a command on AbakaAI inside the ASPIRE aspire/sim env.
#
#   abox.sh 'some shell command'      # command string as $1
#   abox.sh < script.sh               # or pipe a whole script on stdin
#
# The remote shell always starts with /mnt/data/YifanKang/aspire_env.sh sourced,
# so $ASPIRE_ROOT/$PYTHON_ROOT/MUJOCO_GL/HF_* are set and cwd is aspire/sim.
# ssh on this box intermittently refuses connections; this retries the *connect*
# only (it never re-runs a command that actually started).

set -uo pipefail
SSHOPTS=(-o ControlMaster=no -o ControlPath=none -o ConnectTimeout=25 -o BatchMode=yes
         -o ServerAliveInterval=30 -o ServerAliveCountMax=1000)

if [ $# -ge 1 ]; then
  BODY="$1"
else
  BODY="$(cat)"
fi

REMOTE="source /mnt/data/YifanKang/aspire_env_ablP.sh
$BODY"

for attempt in 1 2 3 4 5 6; do
  out=$(ssh "${SSHOPTS[@]}" AbakaAI "bash -s" <<< "$REMOTE" 2>&1)
  rc=$?
  # Only a pre-execution connect failure is retryable.
  if [ $rc -ne 255 ] || ! grep -q "Connection refused\|Connection timed out\|Connection closed by\|Operation timed out" <<< "$out"; then
    grep -v "bash_aliases" <<< "$out"
    exit $rc
  fi
  sleep 8
done
echo "ABOX_SSH_CONNECT_FAILED_AFTER_RETRIES" >&2
grep -v "bash_aliases" <<< "$out" >&2
exit 255
