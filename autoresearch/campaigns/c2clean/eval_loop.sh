#!/bin/bash
# Re-run the resumable sealed-eval pass every 15 min until all 120 cells have a sealed result.
cd "$(dirname "$0")"
while :; do
  n=$(grep -c '^.* done .* success=' eval.log)
  [ "$n" -ge 120 ] && { echo "$(date '+%F %T') all 120 evaluated" >> eval.log; break; }
  MAX_CONCURRENT=4 bash run_eval.sh >> eval.stdout 2>&1
  sleep 900
done
