#!/bin/bash
# 50-episode sealed eval of an rd2 cell (same frozen program, episodes 1..50 of the eval pool; the 20-ep first pass is its prefix)
cell=$1; task=$2; N=50
cd /mnt/data/YifanKang/Heron || exit 1
LOG=/mnt/data/YifanKang/tmp/rd2_eval50.log; say(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
out=results/eval50_rd2_$cell
[ -s $out/results.jsonl ] && { say "skip $cell (exists)"; exit 0; }
exec 9>/mnt/data/YifanKang/tmp/rd2_eval50_$cell.lock; flock -n 9 || { say "skip $cell (queued)"; exit 0; }
md5=$(md5sum packs/rd2_$cell/program.py 2>/dev/null | cut -d' ' -f1); [ -z "$md5" ] && { say "SKIP $cell: no program"; exit 1; }
m20=$(grep "^$cell " /mnt/data/YifanKang/tmp/rd2_eval_freeze.txt | tail -1 | awk '{print $2}'); [ -n "$m20" ] && [ "$m20" != "$md5" ] && { say "SKIP $cell: program changed since the 20-ep eval ($m20 -> $md5)"; exit 1; }
echo "$cell $md5 $(date -u +%FT%TZ) n=$N" >> /mnt/data/YifanKang/tmp/rd2_eval50_freeze.txt
while [ "$(pgrep -fc 'fair_run_robodojo.py.*--split eval')" -ge 8 ]; do sleep 60; done
while [ "$(df --output=avail -BG /mnt/data | tail -1 | tr -dc 0-9)" -lt 150 ]; do say "waiting: disk"; sleep 300; done
gpu=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits | sort -t, -k2 -n | head -1 | cut -d, -f1)
say "eval50 $cell ($md5) gpu $gpu"
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task $task --program packs/rd2_$cell/program.py --split eval --eval-n $N --gpu $gpu --out $out > $out.out 2>&1
rc=$?; n=$(grep -c '"benchmark_success": true' $out/results.jsonl 2>/dev/null); say "done $cell rc=$rc success=${n:-?}/$N"
