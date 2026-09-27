#!/bin/bash
# coordinator_v4.sh — the Stage 2 half of Arm A' x 50, detached.
#
# Polls every POLL seconds. For each cell that has reached stage1-done (its
# supervisor exited, AND both artifacts are on the box), launches the
# coordinator-only held-out eval over seeds 1-50 exactly once, verifies the
# evaluated code hash against the delivered fix_code.py, then PRUNES that cell's
# depth keyframe dumps so the 48-cell campaign's disk footprint stays bounded.
#
# Protocol: this is the ONLY thing that ever touches seeds 1-50, exactly as the
# runbook requires. Stage 1 agents are forbidden from running it.
#
# DONE-SIGNAL (session 9's fix, carried forward): artifacts on the box are NOT
# the done-signal -- they appear mid-run, and in v1 that caused a cell to be
# evaluated on a program Stage 1 had not finished writing. The done-signal is
# THE AGENT PROCESS HAVING EXITED, which the supervisor records by writing
# STAGE1_STATUS on its way out and only on its way out. We require that file in
# addition to the two artifacts.
#
# CONCURRENCY: Stage 1 is capped by scheduler_v4.sh (SLOTS, model-API bound).
# Stage 2 is capped separately here (EVAL_SLOTS), because a held-out eval is
# pure MuJoCo replay and consumes NO model API at all -- the resource the
# coordinator directive's five-slot ceiling was actually protecting. See NOTES.
set -uo pipefail
S=/Users/yifankang/aspire_abl
ARM="${ARM:?set ARM=P or T}"
CELLS="${CELLS:-$S/cells20.txt}"
POLL="${POLL:-300}"
EVAL_SLOTS="${EVAL_SLOTS:-3}"
LOGFILE="${LOGFILE:-$S/coordinator_${ARM}.log}"
PRUNE="${PRUNE:-1}"
BOX="$S/abox_${ARM}.sh"

log() { echo "[$(date '+%F %T')] $*" >> "$LOGFILE"; }

dir_of() { echo "$S/agent_${ARM}_${1}__${2}"; }

n_cells=$(grep -c . "$CELLS")
log "coordinator_v4 start (poll=${POLL}s, $n_cells cells, eval_slots=$EVAL_SLOTS, prune=$PRUNE)"

while :; do
  alldone=1
  cycle=""
  running=0

  # Count live evals first so the slot cap is enforced within this cycle.
  while read -r cell; do
    [ -n "$cell" ] || continue
    d="$(dir_of "${cell%%/*}" "${cell#*/}")"
    [ -f "$d/STAGE2_RUNNING" ] && running=$((running+1))
  done < "$CELLS"

  while read -r cell; do
    [ -n "$cell" ] || continue
    suite="${cell%%/*}"; task="${cell#*/}"
    CELLKEY="${suite}__${task}"
    DIR="$(dir_of "$suite" "$task")"
    res="$DIR/STAGE2_RESULT"
    [ -f "$res" ] && continue
    alldone=0

    [ -d "$DIR" ] || continue
    [ -f "$DIR/session.id" ] || continue
    if [ ! -f "$DIR/STAGE1_STATUS" ]; then
      cycle="$cycle ${task:0:10}:s1-live"
      continue
    fi
    s1status=$(cat "$DIR/STAGE1_STATUS")
    GPU=$(cat "$DIR/GPU" 2>/dev/null || echo 3)

    # Authoritative progress comes from the eval run's own manifest.json
    # (trials/passes + identity.code_sha256), never from counting directories.
    state=$("$BOX" "cd \$ASPIRE_ROOT
D=outputs/libero_fix_loop/$suite/$task
E=outputs/libero_fix_loop_eval/$suite/$task/runs
s1=no; [ -f \$D/fix_code.py ] && [ -f \$D/findings.md ] && s1=yes
CH=none; [ -f \$D/fix_code.py ] && CH=\$(sha256sum \$D/fix_code.py | cut -c1-16)
M=\$(ls -t \$E/*/manifest.json 2>/dev/null | head -1)
n=0; ok=0; mh=none; rd=none
if [ -n \"\$M\" ]; then
  rd=\$(dirname \$M)
  read n ok mh <<< \$(env PYTHONPATH= .venv-libero/bin/python3 -c \"import json,sys;m=json.load(open(sys.argv[1]));print(m.get('trials',0),m.get('passes',0),(m.get('identity',{}).get('code_sha256') or 'none')[:16])\" \"\$M\" 2>/dev/null)
fi
run=no; P=/mnt/data/YifanKang/logs/stage2_arm${ARM}_${CELLKEY}.pid
[ -f \$P ] && kill -0 \$(cat \$P) 2>/dev/null && run=yes
echo \"S1=\$s1 N=\${n:-0} OK=\${ok:-0} RUNNING=\$run CH=\$CH MH=\${mh:-none} RD=\$rd\"" 2>/dev/null | tr -d '\r')

    s1=$(grep -o 'S1=[a-z]*'           <<< "$state" | cut -d= -f2)
    n=$(grep  -o 'N=[0-9]*'            <<< "$state" | cut -d= -f2)
    ok=$(grep -o 'OK=[0-9]*'           <<< "$state" | cut -d= -f2)
    runf=$(grep -o 'RUNNING=[a-z]*'    <<< "$state" | cut -d= -f2)
    codeh=$(grep -o 'CH=[a-z0-9]*'     <<< "$state" | cut -d= -f2)
    manh=$(grep  -o 'MH=[a-z0-9]*'     <<< "$state" | cut -d= -f2)
    rundir=$(grep -o 'RD=[^ ]*'        <<< "$state" | cut -d= -f2)
    [ -z "${s1:-}" ] && { log "$cell: probe failed, skipping this cycle"; continue; }
    cycle="$cycle ${task:0:10}:${ok:-0}/${n:-0}${runf:+,run=$runf}"

    if [ "${n:-0}" -ge 50 ]; then
      echo "heldout ${ok}/50" > "$res"
      rm -f "$DIR/STAGE2_RUNNING"
      log "$cell: STAGE2 COMPLETE — held-out ${ok}/50"
      # The session-8 audit, automated: was the DELIVERED program evaluated?
      if [ "$codeh" = "$manh" ]; then
        log "$cell: code hash verified ($codeh) — evaluated the delivered fix_code.py"
      else
        log "$cell: *** EVALUATED_STALE_CODE *** delivered=$codeh evaluated=$manh — RE-RUN REQUIRED"
        echo "STALE delivered=$codeh evaluated=$manh" > "$DIR/STAGE2_STALE"
      fi
      # Reclaim the depth dumps now that the cell is banked (~2.5 G/cell). The
      # pruner writes a recount ledger first and refuses any run whose own
      # manifest reports fewer than 50 trials, so it cannot touch a live run.
      if [ "$PRUNE" = 1 ] && [ "$rundir" != none ]; then
        out=$("$BOX" "cd \$ASPIRE_ROOT && env PYTHONPATH= .venv-libero/bin/python3 /mnt/data/YifanKang/prune_depth.py --apply --roots '\$ASPIRE_ROOT/$rundir'" 2>&1 | tail -2 | tr '\n' ' ')
        log "$cell: prune -> $out"
      fi
      continue
    fi

    if [ "$s1" != yes ]; then
      log "$cell: supervisor exited '$s1status' but artifacts incomplete — no Stage 2, needs coordinator review"
      echo "no_stage1_artifacts ($s1status)" > "$DIR/STAGE2_BLOCKED"
      echo "blocked" > "$res"
      rm -f "$DIR/STAGE2_RUNNING"
      continue
    fi
    if [ "$runf" = yes ]; then
      touch "$DIR/STAGE2_RUNNING"
      log "$cell: stage2 running (${n}/50 trials, ${ok} success)"
      continue
    fi

    # stage1-done, agent exited, nothing running: launch (or relaunch to finish
    # a partial run — the run id hashes code+config+seeds, so a relaunch resumes
    # the same immutable run rather than starting a second one).
    if [ "$running" -ge "$EVAL_SLOTS" ]; then
      cycle="$cycle ${task:0:10}:s2-queued"
      continue
    fi
    out=$(ARM="$ARM" "$S/stage2_abl.sh" "$suite" "$task" "$GPU" 2>&1 | tail -1)
    running=$((running+1))
    touch "$DIR/STAGE2_RUNNING" "$DIR/STAGE2_LAUNCHED"
    log "$cell: stage2 dispatch (stage1=$s1status, gpu=$GPU, code=$codeh, had ${n:-0}/50) -> $out"
  done < "$CELLS"

  log "cycle: evals_live=$running |$cycle"
  [ "$alldone" = 1 ] && { log "ALL $n_cells HELD-OUT EVALS COMPLETE"; break; }
  sleep "$POLL"
done
log "coordinator_v4 exit"
