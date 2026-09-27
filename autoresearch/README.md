# The Encore acquisition harness — a reproducible pipeline

This directory is the **method**: a fleet of unattended coding-agent
sessions that each turn K=3 human demonstrations + one sentence of intent
into a banked, audited policy program. Everything a paper number rests on
is derived from on-disk artifacts by the scripts listed here — no human
(and no coordinating agent) judgment enters any reported number. The
coordinator's remaining duties are written below as mechanical procedures
so that anyone — a person, or another agent — can run them.

## 1. Prerequisites

- A workstation with the `claude` CLI installed (Claude Code ≥ 2.x).
- A GPU box with the LIBERO-PRO benchmark and this repo's Python env:
  - repo at `$H` (here: `/mnt/data/YifanKang/Heron`, venv `.venv/`)
  - benchmark at `$L` (here: `/mnt/data/YifanKang/LIBERO-PRO/libero/libero`)
  - an ssh alias to it (here: `AbakaAI`). With many concurrent sessions,
    enable ssh connection multiplexing on the workstation:

    ```
    Host <alias>
      ControlMaster auto
      ControlPath ~/.ssh/cm-%r@%h-%p
      ControlPersist 10m
    ```

- **Auth, either way works:**
  - *Subscription*: run `claude setup-token` once, put the token in the
    repo's git-ignored `.env` as `CLAUDE_CODE_OAUTH_TOKEN=...`;
    `tools/ar_launch.sh` exports it for headless sessions.
  - *API*: export `ANTHROPIC_API_KEY` instead; the CLI picks it up.
- Model per session via `AR_MODEL` (default `sonnet`; use `opus` for
  mechanism-development waves). Observed cost: sessions are ~99.9%
  cached-input reads; a 70-task campaign fits comfortably inside a
  subscription day.

## 2. The three files that define a session

| file | role |
|---|---|
| `PROTOCOL.md` | the session contract: measure-first, one hypothesis per version, 5-episode spread probe gating a 20-episode formal, stop conditions (≥15/20, 12 versions, or documented mechanism gap), argmax version selection, session discipline (never wait for notifications) |
| `LAWS.md` | the law library — falsifiable claims with evidence receipts; the only memory shared between sessions |
| per-task `TASK.md` | paths (bddl / official init file / demo hdf5), the intent sentence, eval command templates, and any wave-specific brief |

## 3. Reproduce, level 0 — re-evaluate a banked program (no agent)

Every reported cell is `results/final20_*/results.jsonl` on the cluster:
20 lines, one per official-init episode, `benchmark_success` judged by
the benchmark's own goal predicate. To re-run one:

```bash
ssh <alias> 'H=<repo>; L=<libero>; cd $H && MUJOCO_EGL_DEVICE_ID=<gpu> \
  $H/.venv/bin/python tools/fewshot_run.py program \
  --bddl $L/bddl_files/<suite>/<task>.bddl \
  --init-states $L/init_files/<suite>/<task>.pruned_init \
  --program packs/<task>/program.py --reward packs/<task>/reward.py \
  --episodes 20 --out results/repro_<task>'
```

The whole scoreboard (argmax over all archived versions of every cell):

```bash
python3 tools/harvest_pro_scoreboard.py
```

## 4. Reproduce, level 1 — acquire one task from scratch

```bash
# 1. build the workspace (extracts the K=3 pack on the cluster, stamps
#    TASK.md/CLAUDE.md/LAWS.md and a permissions allowlist)
tools/ar_new_task.sh <name> <suite>/<task_bddl_basename> \
    <demo_hdf5_cluster_path> "<one sentence of intent>"

# 2. launch the unattended session (max 250 turns)
AR_MODEL=sonnet tools/ar_launch.sh <name> 250
```

The session works alone: it measures, writes `packs/<name>/program.py`
and `reward.py`, probes, runs formals, and banks numbers + diagnosis in
the workspace's `NOTES.md`. Its full stream-json transcript,
per-version wall-clock, and every archived program version are the audit
trail. Acceptance = `final20_<name>_*/results.jsonl` exists on the
cluster **and** `NOTES.md` states the banked number (a session that
"launched a run and will report back" has delivered nothing).

## 5. Reproduce, level 2 — a wave (many tasks in parallel)

Launch sessions staggered (45–60 s apart) to avoid ssh/login bursts;
~10–13 concurrent sessions is routine on a laptop. Supervision is a
mechanical loop (run it yourself, or hand it to any agent):

0. **Branch guard**: before any campaign work, `git branch --show-current`
   — commit and push to the branch you think you are on. (This campaign
   spent a day pushing a stale `main` no-op while its commits sat on a
   leftover feature branch; harmless locally, invisible remotely.)
1. **Liveness**: a session is dead when its `WALLCLOCK.md` gains an
   `end` line. Check whether its bank is complete (every member's
   formal + `NOTES.md` non-empty).
2. **Early exit** (the dominant failure mode: the model "pauses to wait
   for a notification" that headless sessions never receive): append a
   `## SESSION N ADDENDUM` to `TASK.md` stating (a) what the
   predecessor already banked — sessions are stateless, the cluster is
   the state, nothing is lost; (b) what remains; (c) the reminder that
   the only legal wait is an in-turn `until <check>; do sleep 60; done`
   loop. Relaunch with `ar_launch.sh` — the addendum makes the restart
   a continuation, not a redo.
3. **Never re-run a passed cell**: argmax bookkeeping means banked
   numbers cannot go down; new versions can only add.

## 6. The coordinator's duties, as mechanical procedures

- **Law promotion**: a session's `NOTES.md` candidate law is promoted to
  `LAWS.md` iff it (a) is stated falsifiably, (b) carries a banked
  20-episode receipt demonstrating the delta, and (c) does not duplicate
  an existing law. Copy the session's own wording where possible; add
  the receipt. Never promote without a receipt.
- **Audits** (run over all workspace transcripts; all three were run on
  this campaign and came back clean):
  - *init-file audit*: no `fewshot_run.py program ... final20` command
    may lack `--init-states` (omission silently evaluates one
    deterministic layout 20×).
  - *episode audit*: no formal may use `--episode-list` (spread lists
    are for probes only).
  - *fabrication audit*: no `Write`/`Edit` tool call may target any
    `results.jsonl`; no direct appends. Success is only ever written by
    `fewshot_run.py` from the env's own goal predicate.
  - *demo-budget audit*: no transcript may touch `demo_3+` in any hdf5
    (the K=3 budget; `names[:3]` / indices 0–2 only).
- **Scoreboard**: `tools/harvest_pro_scoreboard.py` (argmax scoreboard) and
  `tools/harvest_all.py` (sealed-evaluation scoreboard). Both read only
  banked artifacts. The paper's figures and tables are regenerated from the
  sealed results in `reproduce/sealed/`.

## 7. What is and is not deterministic

Re-running a *banked program* reproduces its number up to MuJoCo/EGL
episode-level noise (see LAWS #23; 20-episode counts are stable to ±1
in our experience). Re-running an *acquisition session* is stochastic —
the agent may take a different path — but the protocol bounds it (12
versions, argmax, banked formals), and the claim under reproduction is
the *distribution* of outcomes the harness produces, not a specific
transcript. The campaign's own transcripts, wall-clocks, and archived
versions are kept precisely so both senses of reproduction are possible.

## 8. Honest boundary

The method as defined here begins *after* bootstrap: `PROTOCOL.md` and
the 20 seed laws were produced during method development (with a human
and a coordinating agent in the loop) and ship as fixed inputs. During
the autonomous campaign the human role was experiment design only:
choosing which cells to attack and with which model; no human turn
occurs inside any session, and 6 further laws were promoted from the
sessions' own candidate laws under the rule in §6.
