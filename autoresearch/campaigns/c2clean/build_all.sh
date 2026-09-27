#!/bin/bash
# Generate 120 c2clean workspaces: 60 cells x {k3,k0}. Mates from skillmate.txt for _task cells (k3 only).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); C2=$HERE/../c2
GPUS=(1 2 3 4 5 6 7); i=0
for m in pos task; do
  while IFS='|' read -r cell bddl intent; do
    [ -z "${cell:-}" ] && continue
    mate=""; [ "$m" = task ] && mate=$(awk -F'|' -v c="$cell" '$1==c && $2!="NONE"{print $2}' "$HERE/skillmate.txt")
    ARM=k3 bash "$HERE/mkworker.sh" "$cell" "$bddl" "$intent" "${GPUS[$((i % 7))]}" $mate >/dev/null
    ARM=k0 bash "$HERE/mkworker.sh" "$cell" "$bddl" "$intent" "${GPUS[$(((i+3) % 7))]}" >/dev/null
    echo "${cell}_k3|$bddl|$intent" >> "$HERE/eval_manifest.txt"; echo "${cell}_k0|$bddl|$intent" >> "$HERE/eval_manifest.txt"
    i=$((i+1))
  done < "$C2/${m}_manifest.txt"
done
echo "[c2clean] $i cells x 2 arms"
