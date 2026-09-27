#!/bin/bash
# supervise_stage1_abl.sh <suite> <task> — keep ONE Arm A' Stage 1 agent alive.
#
# Same supervisor as session 15's supervise_stage1.sh, with the arm/suite change
# SCOPE v4 needs: a cell is (suite, task), not a task in one fixed suite. Copied
# rather than edited because bash reads a running script incrementally from disk
# and session 15's script must stay byte-stable as the historical record of how
# the banked cells were produced.
#
# CONVERGE mode is ON by default here. Session 15's Arm A ran all ten cells with
# CONVERGE=1 and TASK.md v4 says "dev 51-65 to convergence" unchanged, so the
# 48 new cells get the same supervisor as the ten they extend. That keeps the
# 58-cell column internally consistent.
#
# Protocol: this script NEVER touches seeds 1-50. Stage 2 is the coordinator's.
set -uo pipefail
# Same credential the Encore campaign workers use (subscription token from Heron/.env);
# the default OAuth login on this laptop expires and every attempt then dies with api_error.
source /Users/yifankang/Heron/tools/claude_auth.sh

S=/Users/yifankang/aspire_abl
ARM="${ARM:?set ARM=P or T}"
SUITE="$1"; TASK="$2"
CELL="${SUITE}__${TASK}"
DIR="$S/agent_${ARM}_${CELL}"
PROMPT="$S/prompts/arm${ARM}_${CELL}.md"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-80}"
DEADLINE_H="${DEADLINE_H:-40}"
MODEL="${MODEL:-claude-opus-5}"
CONVERGE="${CONVERGE:-1}"
MAX_CLEAN_NUDGES="${MAX_CLEAN_NUDGES:-6}"
CONVERGE_CHECKS="${CONVERGE_CHECKS:-2}"

[ -f "$PROMPT" ] || { echo "MISSING PROMPT $PROMPT" >&2; exit 1; }
mkdir -p "$DIR"
cd "$DIR" || exit 1

SIDF="$DIR/session.id"
[ -f "$SIDF" ] || uuidgen | tr 'A-Z' 'a-z' > "$SIDF"
SID="$(cat "$SIDF")"

sup() { echo "[$(date '+%F %T')] $*" >> "$DIR/supervisor.log"; }

is_done() {
  local out
  out=$("$DIR/abox.sh" "cd \$ASPIRE_ROOT && T=outputs/libero_fix_loop/$SUITE/$TASK && \
        if [ -f \$T/fix_code.py ] && [ -f \$T/findings.md ]; then echo STAGE1_DONE; else echo STAGE1_PENDING; fi" 2>/dev/null)
  grep -q STAGE1_DONE <<< "$out"
}

RESUME_NUDGE="Your previous turn was cut off by a transient network/API error on this laptop (not by anything in the task). Nothing about the assignment has changed. Continue your Stage 1 work exactly where you left off, following the same instructions you were given: re-check what you already wrote on the box before redoing anything, and carry on to fix_code.py and findings.md. Do not restart from scratch, and do not run held-out seeds 1-50."

START=$(date +%s)
BACKOFF=45
PENDING_NUDGE=0
CONVERGE_USED=0
sup "supervisor start cell=$CELL sid=$SID model=$MODEL converge=$CONVERGE max_attempts=$MAX_ATTEMPTS deadline=${DEADLINE_H}h"
rm -f "$DIR/STAGE1_STATUS"

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
  if [ "$PENDING_NUDGE" = 0 ] && is_done; then
    sup "DONE (fix_code.py + findings.md present) after $((attempt-1)) attempts"
    echo done > "$DIR/STAGE1_STATUS"; exit 0
  fi
  now=$(date +%s)
  if [ $(( (now-START)/3600 )) -ge "$DEADLINE_H" ]; then
    sup "DEADLINE ${DEADLINE_H}h reached; stopping"
    echo deadline > "$DIR/STAGE1_STATUS"; exit 3
  fi

  ALOG="$DIR/attempt_${attempt}.log"
  before=$(wc -l < "$DIR/agent.log" 2>/dev/null || echo 0)
  if [ "$attempt" -eq 1 ] && [ ! -s "$DIR/agent.log" ]; then
    sup "attempt $attempt: fresh launch (session-id $SID)"
    claude -p "$(cat "$PROMPT")" --session-id "$SID" --model "$MODEL" \
      --permission-mode bypassPermissions \
      --output-format stream-json --verbose \
      > "$ALOG" 2>>"$DIR/agent.err"
  else
    sup "attempt $attempt: resume $SID"
    claude -p "$RESUME_NUDGE" --resume "$SID" --model "$MODEL" \
      --permission-mode bypassPermissions \
      --output-format stream-json --verbose \
      > "$ALOG" 2>>"$DIR/agent.err"
  fi
  rc=$?
  cat "$ALOG" >> "$DIR/agent.log"
  after=$(wc -l < "$DIR/agent.log" 2>/dev/null || echo 0)

  if grep -q '"terminal_reason":"api_error"' "$ALOG" 2>/dev/null; then
    reason=api_error
  elif grep -q '"type":"result"' "$ALOG" 2>/dev/null; then
    reason=agent_returned
  else
    reason=no_result_rc${rc}
  fi
  sup "attempt $attempt ended: $reason (rc=$rc, +$((after-before)) events)"

  if [ "$reason" = agent_returned ]; then
    PENDING_NUDGE=0
    if is_done; then
      if [ "$CONVERGE" = 1 ] && [ "$CONVERGE_USED" -lt "$CONVERGE_CHECKS" ] \
         && ! grep -q 'DEV_SWEEP_COMPLETE' "$ALOG" 2>/dev/null; then
        CONVERGE_USED=$((CONVERGE_USED+1))
        sup "converge check $CONVERGE_USED/$CONVERGE_CHECKS: artifacts present, dev sweep not confirmed complete"
        RESUME_NUDGE="Before your Stage 1 delivery is accepted: your instructions require the development sweep over seeds 51-65 (all 15) to be run to completion with the fix_code.py you are delivering, and the result recorded in findings.md. Nothing is cutting you short and there is no time pressure — an under-converged program is a worse outcome than a slow one. If that sweep did NOT cover all 15 dev seeds with the delivered program, run it now, keep debugging the seeds that fail, and update fix_code.py and findings.md. Only when the sweep is genuinely complete, reply with the single token DEV_SWEEP_COMPLETE plus a one-line per-seed tally, and stop. Do not run held-out seeds 1-50 under any circumstances."
        PENDING_NUDGE=1
        sleep 5; continue
      fi
      sup "DONE: agent returned and both artifacts are on the box (converge_checks_used=$CONVERGE_USED)"
      echo done > "$DIR/STAGE1_STATUS"; exit 0
    fi
    if [ -f "$DIR/.clean_return_nudges" ]; then n=$(cat "$DIR/.clean_return_nudges"); else n=0; fi
    n=$((n+1)); echo $n > "$DIR/.clean_return_nudges"
    if [ "$n" -gt "$MAX_CLEAN_NUDGES" ]; then
      sup "agent returned cleanly $n times without fix_code.py+findings.md; stopping for coordinator review"
      echo needs_review > "$DIR/STAGE1_STATUS"; exit 4
    fi
    sup "clean return #$n without artifacts; nudging (converge=$CONVERGE)"
    if [ "$CONVERGE" = 1 ]; then
      RESUME_NUDGE="You returned without writing both required artifacts. Nothing has cut you short and there is no time pressure — take as long as the task needs. Per your instructions, Stage 1 is complete only when your development sweep over seeds 51-65 (all 15) has been run to completion with your chosen program AND both outputs/libero_fix_loop/$SUITE/$TASK/fix_code.py and .../findings.md exist on the box. Continue debugging where you left off; finish the sweep before you deliver, and record the per-seed tally in findings.md. Do not run held-out seeds 1-50."
    else
      RESUME_NUDGE="You returned without writing both required artifacts. Per your instructions, Stage 1 is complete only when BOTH outputs/libero_fix_loop/$SUITE/$TASK/fix_code.py and .../findings.md exist on the box (Step 4 and Step 5). If no attempt succeeded, still write the best program you have as fix_code.py (a minimal legal program if everything crashed) and write findings.md. Do not run held-out seeds 1-50."
    fi
    PENDING_NUDGE=1
    sleep 10; continue
  fi

  if [ $((after-before)) -gt 20 ]; then BACKOFF=45; else BACKOFF=$(( BACKOFF*2 )); [ $BACKOFF -gt 600 ] && BACKOFF=600; fi
  sup "transient death; sleeping ${BACKOFF}s then resuming"
  sleep "$BACKOFF"
done

sup "MAX_ATTEMPTS $MAX_ATTEMPTS exhausted"
echo exhausted > "$DIR/STAGE1_STATUS"
exit 5
