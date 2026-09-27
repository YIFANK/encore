#!/bin/bash
# stage2_v4.sh <suite> <task> <gpu> — coordinator-only held-out eval, seeds 1-50.
#
# Same as session 15's stage2.sh with the suite parameterized. Launches ASPIRE's
# run_fix_loop_validation.py nohup-detached ON THE BOX (ssh drops kill foreground
# jobs) and returns immediately.
#
# Stage 1 agents must NEVER run this: seeds 1-50 are held out.
set -uo pipefail
S=/Users/yifankang/aspire_ab_run
SUITE="$1"; TASK="$2"; GPU="$3"
CELL="${SUITE}__${TASK}"
BOX="$S/agent_A_${CELL}/abox.sh"
LOG="/mnt/data/YifanKang/logs/stage2_armA_${CELL}.log"
PIDF="/mnt/data/YifanKang/logs/stage2_armA_${CELL}.pid"

"$BOX" <<EOS
set -uo pipefail
SUITE=$SUITE
TASK=$TASK
GPU=$GPU
mkdir -p /mnt/data/YifanKang/logs
FIX="outputs/libero_fix_loop/\$SUITE/\$TASK/fix_code.py"
if [ ! -f "\$FIX" ]; then echo "STAGE2_ABORT: no \$FIX (cell is not stage1-done)"; exit 2; fi
if [ -f $PIDF ] && kill -0 \$(cat $PIDF) 2>/dev/null; then
  echo "STAGE2_ALREADY_RUNNING pid=\$(cat $PIDF)"; exit 0
fi
nohup .venv-libero/bin/python3 scripts/libero/run_fix_loop_validation.py \\
  --suite "\$SUITE" --task "\$TASK" --gpu "\$GPU" \\
  --fix-code "\$FIX" \\
  --output-dir outputs/libero_fix_loop_eval \\
  --seeds \$(seq 1 50) --resume \\
  > $LOG 2>&1 &
echo \$! > $PIDF
sleep 2
echo "STAGE2_LAUNCHED cell=$CELL gpu=\$GPU pid=\$(cat $PIDF) log=$LOG"
EOS
