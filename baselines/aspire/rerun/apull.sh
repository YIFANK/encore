#!/bin/bash
# apull.sh [-r] <remote_path> <local_path> — copy a file (or -r a dir) from AbakaAI.
# Use for keyframe PNGs you want to look at with the Read tool.
set -euo pipefail
ARGS=()
if [ "${1:-}" = "-r" ]; then ARGS+=(-r); shift; fi
for a in 1 2 3 4 5; do
  if scp -q ${ARGS[@]+"${ARGS[@]}"} -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=25 "AbakaAI:$1" "$2"; then
    echo "pulled $1 -> $2"; exit 0
  fi
  sleep 8
done
echo "APULL_FAILED" >&2; exit 1
