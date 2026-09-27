#!/bin/bash
# rd1 sealed eval scheduler, runs ON THE BOX under nohup so laptop ssh drops cannot kill an eval
# (fair_run_robodojo writes results once and never appends: a killed cell restarts from zero).
cd /mnt/data/YifanKang/Heron || exit 1
MAX=${MAX:-4}; MAN=/mnt/data/YifanKang/tmp/rd1_eligible_pack.txt; LOG=/mnt/data/YifanKang/tmp/rd1_eval_sched_pack.log
say(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
say "start max=$MAX"; g=0
while IFS='|' read -r cell task; do
  [ -z "$cell" ] && continue
  if [ -s results/eval_rd_$cell/results.jsonl ]; then say "skip $cell (exists)"; continue; fi
  md5=$(md5sum packs/rd_$cell/program.py 2>/dev/null | cut -d' ' -f1); [ -z "$md5" ] && { say "SKIP $cell no program"; continue; }
  echo "$cell $md5 $(date -u +%FT%TZ)" >> /mnt/data/YifanKang/tmp/rd1_eval_freeze.txt
  while [ "$(jobs -rp | wc -l)" -ge "$MAX" ]; do sleep 30; done
  # disk guard: the 09-16 03:25 batch died of ENOSPC on /mnt/data (a co-tenant filled it); wait for headroom
  while [ "$(df --output=avail -BG /mnt/data | tail -1 | tr -dc 0-9)" -lt 150 ]; do say "waiting: /mnt/data free < 150G"; sleep 300; done
  gpu=$(( g % 2 )); g=$((g+1))   # gpus 4-7 (aspire evals sit on 3/5)
  say "eval $cell ($md5) gpu $gpu"
  ( rm -rf results/eval_rd_$cell; env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task $task --program packs/rd_$cell/program.py --split eval --gpu $gpu --out results/eval_rd_$cell > results/eval_rd_$cell.out 2>&1; rc=$?; n=$(grep -c '"benchmark_success": true' results/eval_rd_$cell/results.jsonl 2>/dev/null); say "done $cell rc=$rc success=${n:-?}/50" ) &
  sleep 20
done < "$MAN"
wait; say "ALL EVALS DONE"
