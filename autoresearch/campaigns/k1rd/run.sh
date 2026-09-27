#!/bin/bash
# k1rd scheduler (K=1 RoboDojo cells in rd1/ and rd2/ workspaces), derived from rd2/run.sh: MAX concurrent worker sessions; each cell = launch -> (auto-resume after outages, same session) -> on-box sealed eval.
# Resumable: a cell with .done is skipped. The Mac's network drops several times a day and kills sessions with
# "API Error ... ENOTFOUND"; rd1 needed hand resumes, rd2 does it here.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/../../.." && pwd)
MAX=${MAX_CONCURRENT:-6}; LOG=$HERE/scheduler.log
say(){ echo "$(date '+%F %T') $*" >> "$LOG"; }
declared(){ grep -qi '^#\+ .*DECLARATION' "$1/NOTES.md" 2>/dev/null; }
net_wait(){ until curl -s -m 10 -o /dev/null https://api.anthropic.com; do sleep 60; done; }
NUDGE="Coordinator note: your previous session ended before a DECLARATION was written (it stopped in the middle of the development loop, not by your decision; usually a network outage on the coordinator machine). Any probe or selection run you had launched has finished; read its receipt under results/ on the cluster, continue the development loop under exactly the same rules as TASK.md, and end this session with a '## DECLARATION' in NOTES.md (frozen argmax version, full-15 selection receipt, receipt chain) or, if mechanism-blocked after honest effort, a documented mechanism-gap stop that still declares the argmax version. Do not stop before that."
limited(){ f=$(ls -t "$1"/transcript_*.jsonl 2>/dev/null | head -1); [ -n "$f" ] && [ "$(grep -c . "$f")" -le 8 ] && grep -qi "hit your .*limit\|usage limit\|limit reached\|API Error: 5[0-9][0-9]\|Overloaded\|Internal server error" "$f"; }
limit_wait(){ # the subscription's 5-hour / weekly window: a refused session is not a resume attempt. Drop the stub, wait, probe.
  while limited "$1"; do
    f=$(ls -t "$1"/transcript_*.jsonl | head -1); say "$(basename "$1"): usage limit ($(grep -o "resets [^\"]*" "$f" | head -1)); waiting 20 min"
    rm -f "$f" "${f%.jsonl}.err" "${f%.jsonl}.meta"; sleep 1200
    ( source "$ROOT/tools/claude_auth.sh" >/dev/null 2>&1; cd /tmp; claude -p "Reply with the single word ok." --model "${AR_MODEL:-opus}" --output-format json --max-turns 1 2>/dev/null | grep -qi "limit" ) && { touch "$1/.limit_probe"; continue; }
    return 0
  done
}
trigger_eval(){ # prefix cell task -- detached on the box; retried until the ssh itself gets through
  for i in 1 2 3 4 5 6 7 8 9 10; do
    ssh -n -o ConnectTimeout=20 -o BatchMode=yes AbakaAI "nohup bash /mnt/data/YifanKang/tmp/k1_rd_eval.sh $1 $2 $3 > /dev/null 2>&1 < /dev/null & echo queued" 2>/dev/null | grep -q queued && { say "eval queued $1 $2 (n=50)"; return 0; }
    sleep 120
  done; say "EVAL TRIGGER FAILED $1 $2"
}
if ! mkdir "$HERE/.sched.lock" 2>/dev/null; then echo "another k1rd scheduler holds $HERE/.sched.lock (pid $(cat "$HERE/run.pid" 2>/dev/null)); refusing to start" >&2; exit 1; fi
echo $$ > "$HERE/run.pid"; trap 'rmdir "$HERE/.sched.lock" 2>/dev/null' EXIT
say "scheduler start pid $$ (max $MAX, eval_n 50)"
while IFS='|' read -r camp task _sl _in; do [ -z "${task:-}" ] && continue; for arm in k1; do
  cell=${task}_${arm}; ws="$HERE/../$camp/workers/$cell"; [ -d "$ws" ] || continue
  case $camp in rd1) pfx=rd;; *) pfx=rd2;; esac
  [ -f "$ws/.done" ] && { say "skip $cell (done)"; continue; }
  mkdir "$ws/.cell.lock" 2>/dev/null || { say "skip $cell (cell lock held: a session owns it)"; continue; }
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAX" ]; do sleep 20; done
  say "launch $cell"
  (
    net_wait
    if ls "$ws"/transcript_*.jsonl >/dev/null 2>&1; then :; else bash "$ROOT/tools/ar_launch_worker.sh" "$ws" 250 >/dev/null 2>&1; fi
    n=0
    while ! declared "$ws" && [ $n -lt 6 ]; do
      if limited "$ws"; then limit_wait "$ws"; [ -z "$(ls "$ws"/transcript_*.jsonl 2>/dev/null)" ] && { bash "$ROOT/tools/ar_launch_worker.sh" "$ws" 250 >/dev/null 2>&1; continue; }; fi
      n=$((n+1)); net_wait
      f=$(ls -t "$ws"/transcript_*.jsonl | head -1); sid=$(grep -o '"session_id":"[0-9a-f-]*"' "$f" | tail -1 | cut -d'"' -f4)
      [ -z "$sid" ] && { say "$cell: no session id, cannot resume"; break; }
      say "$cell: resume $n (session $sid)"
      ( source "$ROOT/tools/claude_auth.sh"; cd "$ws"; S=$(date +%Y%m%d_%H%M%S); echo "session $S resume $sid start $(date -u +%FT%TZ)" >> WALLCLOCK.md
        claude -p "$NUDGE" --resume "$sid" --model "${AR_MODEL:-opus}" --output-format stream-json --verbose --max-turns 250 > "transcript_$S.jsonl" 2> "transcript_$S.err" || true
        echo "session $S end   $(date -u +%FT%TZ)" >> WALLCLOCK.md )
    done
    touch "$ws/.done"
    if declared "$ws"; then say "finished $cell (declared, $n resumes)"; trigger_eval "$pfx" "$cell" "$task"; else say "finished $cell (NO DECLARATION after $n resumes)"; fi
  ) &
  sleep 5
done; done < "$HERE/cells.txt"
wait; say "ALL CELLS DONE"
