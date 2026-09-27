#!/bin/bash
# Source this (do not execute) before any headless `claude -p`. It selects
# one credential and exports it, so a session runs under whichever account
# the operator configured, never silently under the machine's login:
#
#   1. ANTHROPIC_API_KEY       env, else repo .env   -> API billing
#   2. CLAUDE_CODE_OAUTH_TOKEN env, else repo .env   -> subscription (claude setup-token)
#   3. neither                                       -> the machine's `claude login`
#
# The value is never printed; only the mode is, on stderr, as CLAUDE_AUTH_MODE.
# Override the .env location with CLAUDE_AUTH_ROOT=<repo root>.
_ca_root="${CLAUDE_AUTH_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
_ca_env="$_ca_root/.env"
_ca_get() { [ -f "$_ca_env" ] && grep -m1 "^$1=" "$_ca_env" | cut -d= -f2- | tr -d '"' ; }
if [ -n "${ANTHROPIC_API_KEY:-}" ] || [ -n "$(_ca_get ANTHROPIC_API_KEY)" ]; then
  export ANTHROPIC_API_KEY="${ANTHROPIC_API_KEY:-$(_ca_get ANTHROPIC_API_KEY)}"
  unset CLAUDE_CODE_OAUTH_TOKEN
  export CLAUDE_AUTH_MODE=api
elif [ -n "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] || [ -n "$(_ca_get CLAUDE_CODE_OAUTH_TOKEN)" ]; then
  export CLAUDE_CODE_OAUTH_TOKEN="${CLAUDE_CODE_OAUTH_TOKEN:-$(_ca_get CLAUDE_CODE_OAUTH_TOKEN)}"
  unset ANTHROPIC_API_KEY
  export CLAUDE_AUTH_MODE=subscription-token
elif python3 -c 'import json,os,sys; d=json.load(open(os.path.expanduser("~/.claude.json"))); sys.exit(0 if d.get("oauthAccount") else 1)' 2>/dev/null; then
  export CLAUDE_AUTH_MODE=machine-login
else
  echo "claude_auth: no credential. Put ANTHROPIC_API_KEY or CLAUDE_CODE_OAUTH_TOKEN in $_ca_env (see .env.example), or run 'claude login'." >&2
  return 1 2>/dev/null || exit 1
fi
echo "claude_auth: mode=$CLAUDE_AUTH_MODE (.env: $_ca_env)" >&2
unset -f _ca_get; unset _ca_root _ca_env
