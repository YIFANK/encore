#!/bin/bash
# 10 tasks x {k3,k0} (classify_objects_by_language has no demonstration data: k0 only).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); GPUS=(0 1 2 3 4 5 6 7); i=0
rm -f "$HERE/eval_manifest.txt"
while IFS='|' read -r task steplim intent; do
  [ -z "${task:-}" ] && continue
  if [ "$task" != classify_objects_by_language ]; then
    ARM=k3 bash "$HERE/mkworker.sh" "$task" "$steplim" "$intent" "${GPUS[$((i % 8))]}" >/dev/null; echo "${task}_k3|$task" >> "$HERE/eval_manifest.txt"; i=$((i+1))
  fi
  ARM=k0 bash "$HERE/mkworker.sh" "$task" "$steplim" "$intent" "${GPUS[$((i % 8))]}" >/dev/null; echo "${task}_k0|$task" >> "$HERE/eval_manifest.txt"; i=$((i+1))
done < "$HERE/cells.txt"
echo "[rd1] $i workspaces"
