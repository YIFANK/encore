#!/bin/bash
# rd1 coordinator sealed eval: one blind run of episodes 1-50 per declared cell, md5 frozen first. Resumable.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); MAX=${MAX_CONCURRENT:-4}; LOG=$HERE/eval.log
say(){ echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
R="cd /mnt/data/YifanKang/Heron && env -u PYTHONPATH"
say "eval start (max concurrent $MAX)"; g=0
while IFS='|' read -r cell task; do
  [ -z "$cell" ] && continue
  [ -f "$HERE/workers/$cell/.done" ] || { say "skip $cell (worker not done)"; continue; }
  grep -qi "^#\+ .*DECLARATION\|Argmax version declared" "$HERE/workers/$cell/NOTES.md" 2>/dev/null || { say "skip $cell (no DECLARATION)"; continue; }
  if ssh -n AbakaAI "test -s /mnt/data/YifanKang/Heron/results/eval_rd_$cell/results.jsonl"; then say "skip $cell (eval exists)"; continue; fi
  md5=$(ssh -n AbakaAI "md5sum /mnt/data/YifanKang/Heron/packs/rd_$cell/program.py 2>/dev/null | cut -d' ' -f1")
  [ -z "$md5" ] && { say "SKIP $cell: no frozen program on cluster"; continue; }
  echo "$cell $md5 $(date -u +%FT%TZ)" >> "$HERE/eval_freeze.txt"
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAX" ]; do sleep 30; done
  gpu=$((g % 8)); g=$((g+1))
  say "eval $cell ($md5) gpu $gpu"
  (
    ssh -n AbakaAI "$R .venv/bin/python tools/fair_run_robodojo.py --task $task --program packs/rd_$cell/program.py --split eval --gpu $gpu --out results/eval_rd_$cell > results/eval_rd_$cell.out 2>&1"
    rc=$?; n=$(ssh -n AbakaAI "grep -c '\"benchmark_success\": true' /mnt/data/YifanKang/Heron/results/eval_rd_$cell/results.jsonl 2>/dev/null")
    say "done $cell rc=$rc success=${n:-?}/50"
  ) &
  sleep 3
done < "$HERE/eval_manifest.txt"
wait; say "ALL EVALS DONE"
