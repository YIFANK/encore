#!/bin/bash
# 12 tasks x {k3 full pack, vis images-only pack, k0 no pack} = 36 workspaces.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); GPUS=(0 1 2 3 4 5 6 7); i=0
rm -f "$HERE/eval_manifest.txt"
while IFS='|' read -r task steplim intent; do
  [ -z "${task:-}" ] && continue
  for arm in k3 vis k0; do
    ARM=$arm bash "$HERE/mkworker.sh" "$task" "$steplim" "$intent" "${GPUS[$((i % 8))]}" >/dev/null
    echo "${task}_${arm}|$task" >> "$HERE/eval_manifest.txt"; i=$((i+1))
  done
done < "$HERE/cells.txt"
echo "[rd2] $i workspaces"
