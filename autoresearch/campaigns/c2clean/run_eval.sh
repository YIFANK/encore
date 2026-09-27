#!/bin/bash
# c2clean coordinator sealed eval: one blind run of seeds 1-50 per declared cell (both arms), md5 frozen first.
# stdin-safe (ssh -n everywhere). Resumable: a cell with results/eval_c2clean_<cell>/results.jsonl is skipped.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); MAX=${MAX_CONCURRENT:-6}; LOG=$HERE/eval.log
say(){ echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
R="cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH"
say "eval start (max concurrent $MAX)"
while IFS='|' read -r cell bddl lang; do
  [ -z "$cell" ] && continue
  base=${cell%_k3}; base=${base%_k0}; arm=${cell##*_}
  [ -f "$HERE/workers/$cell/.done" ] || { say "skip $cell (worker not done)"; continue; }
  grep -q "DECLARATION" "$HERE/workers/$cell/NOTES.md" 2>/dev/null || { say "skip $cell (no DECLARATION)"; continue; }
  if ssh -n AbakaAI "test -s /mnt/data/YifanKang/Heron/results/eval_c2clean_$cell/results.jsonl"; then say "skip $cell (eval exists)"; continue; fi
  md5=$(ssh -n AbakaAI "md5sum /mnt/data/YifanKang/Heron/packs/c2clean_${base}_${arm}/program.py 2>/dev/null | cut -d' ' -f1")
  [ -z "$md5" ] && { say "SKIP $cell: no frozen program on cluster"; continue; }
  echo "$cell $md5 $(date -u +%FT%TZ)" >> "$HERE/eval_freeze.txt"
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAX" ]; do sleep 30; done
  say "eval $cell ($md5)"
  (
    ssh -n AbakaAI "$R .venv/bin/python tools/fair_run.py program --seed-episodes --bddl $bddl --language \"$lang\" --program packs/c2clean_${base}_${arm}/program.py --split eval --out results/eval_c2clean_$cell --tmp-root /mnt/data/YifanKang/tmp > results/eval_c2clean_$cell.out 2>&1"
    rc=$?; n=$(ssh -n AbakaAI "grep -c '\"benchmark_success\": true' /mnt/data/YifanKang/Heron/results/eval_c2clean_$cell/results.jsonl 2>/dev/null")
    say "done $cell rc=$rc success=${n:-?}/50"
  ) &
  sleep 3
done < "$HERE/eval_manifest.txt"
wait; say "ALL EVALS DONE"
