#!/bin/bash
# K=1 RoboDojo workspaces: each task uses its own campaign's brief generator (rd1 or rd2) with ARM=k1,
# so the brief is identical to that task's K=3/K=0 cells except the pack section.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); GPUS=(0 1 2 3 4 5 6 7); i=0
while IFS='|' read -r camp task steplim intent; do
  [ -z "${task:-}" ] && continue
  ARM=k1 bash "$HERE/../$camp/mkworker.sh" "$task" "$steplim" "$intent" "${GPUS[$((i % 8))]}" >/dev/null
  i=$((i+1))
done < "$HERE/cells.txt"
echo "[k1rd] $i workspaces"
