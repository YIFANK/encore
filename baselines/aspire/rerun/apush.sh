#!/bin/bash
# apush.sh <local_path> <remote_path> — copy a local file to AbakaAI.
# Use this to put code on the box: write the file locally with Write, then push.
# Never try to create python files with nested ssh heredocs — quoting gets mangled.
set -euo pipefail
for a in 1 2 3 4 5; do
  if scp -q -o ControlMaster=no -o ControlPath=none -o ConnectTimeout=25 "$1" "AbakaAI:$2"; then
    echo "pushed $1 -> $2"; exit 0
  fi
  sleep 8
done
echo "APUSH_FAILED" >&2; exit 1
