#!/bin/bash
# launch_stage1.sh [armA|armB|all] — start detached Stage 1 agents.
#
# Each agent is a standalone `claude -p` OS process, NOT an in-session subagent:
# session 5 lost three agents when its coordinator session ended. These survive
# the coordinator; they are polled with status_stage1.sh and are re-launchable
# individually.
set -uo pipefail
S=/Users/yifankang/aspire_ab_run
WHICH="${1:-all}"

TASKS=(turn_on_the_stove put_the_bowl_on_the_plate open_the_middle_drawer_of_the_cabinet)

launch() {
  local arm="$1" task="$2"
  local dir="$S/agent_${arm}_${task}"
  local prompt="$S/prompts/arm${arm}_${task}.md"
  [ -f "$prompt" ] || { echo "MISSING PROMPT $prompt" >&2; return 1; }
  mkdir -p "$dir"
  if [ -f "$dir/agent.pid" ] && kill -0 "$(cat "$dir/agent.pid")" 2>/dev/null; then
    echo "arm${arm}/${task}: already running (pid $(cat "$dir/agent.pid"))"; return 0
  fi
  ( cd "$dir" && nohup claude -p "$(cat "$prompt")" \
      --permission-mode bypassPermissions \
      --output-format stream-json --verbose \
      > "$dir/agent.log" 2>"$dir/agent.err" &
    echo $! > "$dir/agent.pid" )
  sleep 1
  echo "arm${arm}/${task}: launched pid $(cat "$dir/agent.pid")  log=$dir/agent.log"
}

for t in "${TASKS[@]}"; do
  case "$WHICH" in
    armA|all) launch A "$t" ;;
  esac
done
for t in "${TASKS[@]}"; do
  case "$WHICH" in
    armB|all) launch B "$t" ;;
  esac
done
