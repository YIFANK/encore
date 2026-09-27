#!/bin/bash
# on-box sealed eval of ONE rd2 cell: episodes 1..N of the eval pool, frozen md5, written once. Usage: rd2_eval_one.sh <cell> <task> <n>
cell=$1; task=$2; N=${3:-20}
cd /mnt/data/YifanKang/Heron || exit 1
LOG=/mnt/data/YifanKang/tmp/rd2_eval.log; say(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
out=results/eval_rd2_$cell
[ -s $out/results.jsonl ] && { say "skip $cell (exists)"; exit 0; }
exec 9>/mnt/data/YifanKang/tmp/rd2_eval_$cell.lock; flock -n 9 || { say "skip $cell (already queued)"; exit 0; }
md5=$(md5sum packs/rd2_$cell/program.py 2>/dev/null | cut -d' ' -f1); [ -z "$md5" ] && { say "SKIP $cell: no program.py"; exit 1; }
echo "$cell $md5 $(date -u +%FT%TZ) n=$N" >> /mnt/data/YifanKang/tmp/rd2_eval_freeze.txt
while [ "$(pgrep -fc 'fair_run_robodojo.py.*--split eval')" -ge 4 ]; do sleep 60; done
while [ "$(df --output=avail -BG /mnt/data | tail -1 | tr -dc 0-9)" -lt 150 ]; do say "waiting: /mnt/data free < 150G"; sleep 300; done
gpu=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits | sort -t, -k2 -n | head -1 | cut -d, -f1)
say "eval $cell ($md5) gpu $gpu n=$N"
rm -rf results/.eval_rd2_$cell.staging
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task $task --program packs/rd2_$cell/program.py --split eval --eval-n $N --gpu $gpu --out $out > $out.out 2>&1
rc=$?; n=$(grep -c '"benchmark_success": true' $out/results.jsonl 2>/dev/null)
say "done $cell rc=$rc success=${n:-?}/$N"
