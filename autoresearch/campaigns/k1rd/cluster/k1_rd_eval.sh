#!/bin/bash
# On-box sealed eval of ONE K=1 RoboDojo cell: episodes 1..50 of the eval pool, frozen md5, written once.
# Usage: k1_rd_eval.sh <prefix rd|rd2> <cell e.g. build_tower_k1> <task>
pfx=$1; cell=$2; task=$3; N=50
cd /mnt/data/YifanKang/Heron || exit 1
LOG=/mnt/data/YifanKang/tmp/k1_rd_eval.log; say(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
out=results/eval50_${pfx}_$cell
[ -s $out/results.jsonl ] && { say "skip $pfx $cell (exists)"; exit 0; }
exec 9>/mnt/data/YifanKang/tmp/k1_rd_eval_${pfx}_$cell.lock; flock -n 9 || { say "skip $pfx $cell (already queued)"; exit 0; }
md5=$(md5sum packs/${pfx}_$cell/program.py 2>/dev/null | cut -d' ' -f1); [ -z "$md5" ] && { say "SKIP $pfx $cell: no program.py"; exit 1; }
echo "${pfx}_$cell $md5 $(date -u +%FT%TZ) n=$N" >> /mnt/data/YifanKang/tmp/k1_rd_eval_freeze.txt
while [ "$(pgrep -fc 'fair_run_robodojo.py.*--split eval')" -ge 8 ]; do sleep 60; done
while [ "$(df --output=avail -BG /mnt/data | tail -1 | tr -dc 0-9)" -lt 150 ]; do say "waiting: /mnt/data free < 150G"; sleep 300; done
gpu=$(nvidia-smi --query-gpu=index,memory.used --format=csv,noheader,nounits | sort -t, -k2 -n | head -1 | cut -d, -f1)
say "eval $pfx $cell ($md5) gpu $gpu n=$N"
rm -rf results/.eval50_${pfx}_$cell.staging
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task $task --program packs/${pfx}_$cell/program.py --split eval --eval-n $N --gpu $gpu --out $out > $out.out 2>&1
rc=$?; n=$(grep -c '"benchmark_success": true' $out/results.jsonl 2>/dev/null)
say "done $pfx $cell rc=$rc success=${n:-?}/$N"
