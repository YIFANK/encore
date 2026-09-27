#!/bin/bash
# Generate 60 c2k1clean workspaces: 60 cells x K=1. Mates from skillmate.txt for _task cells.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); C2=$HERE/../c2
GPUS=(1 2 3 4 5 6 7); i=0
for m in pos task; do
  while IFS='|' read -r cell bddl intent; do
    [ -z "${cell:-}" ] && continue
    mate=""; [ "$m" = task ] && mate=$(awk -F'|' -v c="$cell" '$1==c && $2!="NONE"{print $2}' "$HERE/skillmate.txt")
    ARM=k1 bash "$HERE/mkworker.sh" "$cell" "$bddl" "$intent" "${GPUS[$((i % 7))]}" $mate >/dev/null
    echo "${cell}_k1|$bddl|$intent" >> "$HERE/eval_manifest.txt"
    i=$((i+1))
  done < "$C2/${m}_manifest.txt"
done
echo "[c2k1clean] $i cells x K=1"
