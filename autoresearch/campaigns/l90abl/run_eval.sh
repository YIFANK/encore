#!/bin/bash
# l90abl coordinator sealed eval: for every cell whose worker is .done and has a
# frozen program on the cluster, run fair_run --split eval EXACTLY once
# (seeds 1-50, blind, publish-at-end), out dir results/eval_l90abl_<cell>.
# Records the program md5 before each run in eval_freeze.txt. Resumable:
# a cell with results/eval_l90abl_<cell>/results.jsonl is skipped.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
MAX=${MAX_CONCURRENT:-6}
LOG=$HERE/eval.log
say(){ echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
R="cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH"
say "eval start (max concurrent $MAX)"
while IFS='|' read -r cell bddl lang; do
  [ -z "$cell" ] && continue
  [ -f "$HERE/workers/$cell/.done" ] || { say "skip $cell (worker not done)"; continue; }
  if ssh -n AbakaAI "test -s /mnt/data/YifanKang/Heron/results/eval_l90abl_$cell/results.jsonl"; then say "skip $cell (eval exists)"; continue; fi
  md5=$(ssh -n AbakaAI "md5sum /mnt/data/YifanKang/Heron/packs/l90abl_$cell/program.py 2>/dev/null | cut -d' ' -f1")
  [ -z "$md5" ] && { say "SKIP $cell: no frozen program on cluster"; continue; }
  echo "$cell $md5 $(date -u +%FT%TZ)" >> "$HERE/eval_freeze.txt"
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAX" ]; do sleep 30; done
  say "eval $cell ($md5)"
  (
    ssh -n AbakaAI "$R .venv/bin/python tools/fair_run.py program --seed-episodes --bddl $bddl --language \"$lang\" --program packs/l90abl_$cell/program.py --split eval --out results/eval_l90abl_$cell --tmp-root /mnt/data/YifanKang/tmp > results/eval_l90abl_$cell.out 2>&1"
    rc=$?; n=$(ssh -n AbakaAI "grep -c '\"benchmark_success\": true' /mnt/data/YifanKang/Heron/results/eval_l90abl_$cell/results.jsonl 2>/dev/null")
    say "done $cell rc=$rc success=${n:-?}/50"
  ) &
  sleep 3
done < "$HERE/eval_manifest.txt"
wait
say "ALL EVALS DONE"
