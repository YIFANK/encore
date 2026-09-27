#!/bin/bash
# scheduler_v4.sh — Arm A' Stage 1 scheduler for the remaining 50-cell matrix.
#
# Session 15's scheduler2.sh was a set of per-GPU FIFO queues: each card had a
# fixed task list, and a card whose queue drained sat idle while another card's
# queue was three deep (that is exactly what forced the logged GPU 4 -> 6 requeue
# late in session 15). With 48 cells and two cards that failure mode would cost
# hours, so this is a SLOT scheduler instead:
#
#   - one global queue (cells48.txt, in file order);
#   - at most SLOTS Stage 1 agents live at any moment (the directive says 5);
#   - a cell is assigned a GPU at dispatch, choosing the card with fewer of our
#     live cells on it, and its prompt is GENERATED THEN with that GPU baked in
#     (ASPIRE's template writes the GPU number into the body, so a prompt cannot
#     be written before the card is known);
#   - the assigned card is recorded in $DIR/GPU so the coordinator's Stage 2
#     lands on the same one, and so the sibling campaign can see what we hold.
#
# Protocol: this script only ever starts STAGE 1. It never touches seeds 1-50 —
# coordinator_v4.sh remains the only thing that does.
set -uo pipefail
S=/Users/yifankang/aspire_ab_run
CELLS="${CELLS:-$S/cells48.txt}"
SLOTS="${SLOTS:-5}"
GPUS="${GPUS:-3 4}"
POLL="${POLL:-120}"
LOG="${LOG:-$S/scheduler_v4.log}"
CONVERGE="${CONVERGE:-1}"
MAX_CLEAN_NUDGES="${MAX_CLEAN_NUDGES:-6}"
CONVERGE_CHECKS="${CONVERGE_CHECKS:-2}"
# Stagger dispatches so five `claude -p` sessions do not all open at once and
# trip the model API's rate limiter (SCOPE v4 budget note).
STAGGER="${STAGGER:-90}"

log() { echo "[$(date '+%F %T')] $*" >> "$LOG"; }

dir_of() { echo "$S/agent_A_${1}__${2}"; }

# notstarted | live | done | orphaned
# NB: separate `local` lines. bash expands every argument to `local` BEFORE
# assigning any of them, so `local a="$1" b="...$a"` reads $a while still unset
# and dies under `set -u`. That bug has been introduced three times in this
# campaign; do not collapse these.
state_of() {
  local suite="$1"
  local task="$2"
  local d
  d="$(dir_of "$suite" "$task")"
  [ -f "$d/session.id" ] || { echo notstarted; return; }
  [ -f "$d/STAGE1_STATUS" ] && { echo done; return; }
  if [ -f "$d/supervisor.pid" ] && kill -0 "$(cat "$d/supervisor.pid")" 2>/dev/null; then
    echo live
  else
    echo orphaned
  fi
}

# How many of OUR live Stage 1 cells sit on a given card.
count_on_gpu() {
  local want="$1"
  local n=0
  local d
  for d in "$S"/agent_A_*; do
    [ -d "$d" ] || continue
    [ -f "$d/GPU" ] || continue
    [ -f "$d/STAGE1_STATUS" ] && continue
    [ -f "$d/supervisor.pid" ] || continue
    kill -0 "$(cat "$d/supervisor.pid")" 2>/dev/null || continue
    [ "$(cat "$d/GPU")" = "$want" ] && n=$((n+1))
  done
  echo "$n"
}

pick_gpu() {
  local best="" bestn=999 g n
  for g in $GPUS; do
    n=$(count_on_gpu "$g")
    if [ "$n" -lt "$bestn" ]; then bestn=$n; best=$g; fi
  done
  echo "$best"
}

launch() { # suite task
  local suite="$1"
  local task="$2"
  local dir gpu
  dir="$(dir_of "$suite" "$task")"
  gpu="$(pick_gpu)"
  mkdir -p "$dir"
  # Prompt must be (re)generated now: the GPU is baked into its body.
  if ! python3 "$S/gen_prompts_v4.py" "$suite" "$task" "$gpu" >/dev/null 2>>"$LOG"; then
    log "$suite/$task: PROMPT GENERATION FAILED, skipping this cycle"
    return 1
  fi
  echo "$gpu" > "$dir/GPU"
  rm -f "$dir/STAGE1_STATUS" "$dir/.clean_return_nudges" "$dir/.converge_checks"
  CONVERGE="$CONVERGE" MAX_CLEAN_NUDGES="$MAX_CLEAN_NUDGES" CONVERGE_CHECKS="$CONVERGE_CHECKS" \
    nohup "$S/supervise_stage1_v4.sh" "$suite" "$task" > "$dir/supervisor.out" 2>&1 &
  echo $! > "$dir/supervisor.pid"
  log "$suite/$task: DISPATCHED gpu=$gpu supervisor=$(cat "$dir/supervisor.pid") converge=$CONVERGE"
}

n_cells=$(grep -c . "$CELLS")
log "scheduler_v4 start: $n_cells cells, slots=$SLOTS, gpus=[$GPUS], poll=${POLL}s, stagger=${STAGGER}s, converge=$CONVERGE"

while :; do
  live=0; done_n=0; notstarted=0
  # Pass 1: census, and relaunch anything orphaned (supervisor died without
  # writing STAGE1_STATUS — e.g. the laptop slept).
  while read -r cell; do
    [ -n "$cell" ] || continue
    suite="${cell%%/*}"; task="${cell#*/}"
    case "$(state_of "$suite" "$task")" in
      live)       live=$((live+1)) ;;
      done)       done_n=$((done_n+1)) ;;
      orphaned)   log "$suite/$task: ORPHANED (supervisor gone, no STAGE1_STATUS) — relaunching"
                  launch "$suite" "$task" && live=$((live+1)) ;;
      notstarted) notstarted=$((notstarted+1)) ;;
    esac
  done < "$CELLS"

  # Pass 2: fill free slots from the queue, in file order.
  started=0
  if [ "$live" -lt "$SLOTS" ]; then
    while read -r cell; do
      [ -n "$cell" ] || continue
      [ "$live" -ge "$SLOTS" ] && break
      suite="${cell%%/*}"; task="${cell#*/}"
      [ "$(state_of "$suite" "$task")" = notstarted ] || continue
      if launch "$suite" "$task"; then
        live=$((live+1)); notstarted=$((notstarted-1)); started=$((started+1))
        sleep "$STAGGER"
      fi
    done < "$CELLS"
  fi

  log "cycle: live=$live done=$done_n queued=$notstarted started_this_cycle=$started gpu3=$(count_on_gpu 3) gpu4=$(count_on_gpu 4)"
  if [ "$live" = 0 ] && [ "$notstarted" = 0 ]; then
    log "ALL $n_cells STAGE 1 CELLS HAVE EXITED"
    break
  fi
  sleep "$POLL"
done
log "scheduler_v4 exit"
