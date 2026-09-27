#!/usr/bin/env python3
"""gen_prompts_v4.py — Stage 1 prompts for SCOPE v4, where a cell is (suite, task).

Session 15's `gen_prompts.py` hardcoded `SUITE = libero_goal_swap` and a static
task->GPU table, because the campaign was ten tasks in one suite across three
dedicated cards. SCOPE v4 is 60 cells over SIX suites run through a slot
scheduler, so:

  - the suite is a parameter, not a module constant;
  - the cell key is `<suite>__<task>`, so `open_the_middle_drawer_of_the_cabinet`
    in `libero_goal_swap` and in `libero_goal_task` are different cells with
    different scratch dirs and different prompts (they are different cells in
    c2 too, and conflating them would silently overwrite one with the other);
  - the prompt is generated AT DISPATCH with whichever GPU the scheduler
    assigned, because ASPIRE's template bakes the GPU number into the body.

Arm A' only. The Arm B addendum is not reachable from here: those 50 cells have
no +demos counterpart, and generating one by accident is exactly the kind of
cross-arm leak the session-15 ablation check existed to prevent.

Usage:  gen_prompts_v4.py <suite> <task> <gpu>     # writes one prompt, prints path
        gen_prompts_v4.py --all                    # regenerate every cell at gpu 3 (dry inspect)
"""
import pathlib
import sys

import os
S = pathlib.Path(__file__).resolve().parent
ARM = os.environ["ARM"]
TEMPLATE_SRC = S / f"runbook_{ARM}" / "fix-loop" / "subagent-prompt.md"
CELLS_FILE = S / "cells20.txt"

SUITE_PLACEHOLDER = (
    "SUITE: <libero_goal_swap|libero_goal_task|libero_object_swap|"
    "libero_object_task|libero_spatial_swap|libero_spatial_task>"
)

# Same host preamble session 15 used, with ONE substantive edit, in the
# "Only GPU {gpu} is yours" bullet. v3's text promised the card was shared with
# "exactly one sibling agent working the same task in the other arm's repo
# checkout" -- true then, false now: under the v4 slot scheduler a card carries
# several Arm A' cells at once, plus the sibling encore_capx campaign. Telling
# an agent it has half a card when it has a fifth of one would make it
# mis-attribute normal contention to a fault in its own program. This is
# host-access guidance, not protocol, and Arm A' has no Arm B counterpart for
# these cells, so there is no ablation symmetry to preserve.
HOST_PREAMBLE = """\
# READ THIS FIRST — how to reach the compute host

You are running on a laptop. The ASPIRE repo, the GPUs, the MuJoCo simulator and
the perception servers are all on a remote box (`AbakaAI`). **You cannot run any
repo command locally.** Everything goes through three helpers:

```
{scratch}/abox.sh 'CMD'        # run CMD on the box, inside the aspire/sim env
{scratch}/abox.sh <<'EOS' ... EOS   # or pipe a whole script body on stdin
{scratch}/apush.sh <local> <remote> # copy a local file TO the box
{scratch}/apull.sh [-r] <remote> <local>  # copy a file/dir FROM the box
```

These three helpers live in your own scratch directory and are already wired to
YOUR repo checkout — use these copies, not any other path on this laptop.

`abox.sh` already sources the session env, so on the remote side you always have
`$ASPIRE_ROOT`, `$PYTHON_ROOT`, `PYTHONPATH`, `MUJOCO_GL=egl`,
`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`, the HF settings, and cwd = your checkout's
`aspire/sim`. Do not re-export those, and do not hardcode an absolute repo path —
always use `$ASPIRE_ROOT` (or relative paths, since cwd is already correct).

Host rules, all of them learned the hard way — violating them wastes hours:

- **Each `abox.sh` call is a fresh shell.** Shell variables do not persist between
  calls. Put `SUITE=... TASK=... GPU=...` at the top of every script body that
  needs them.
- **Never create a Python file with a nested ssh heredoc** — quoting gets mangled
  and you will debug a file that is not what you wrote. Always: `Write` the file
  locally, then `apush.sh` it to the box.
- **Long jobs must be nohup-detached with a log file**, because ssh drops. Launch
  with `nohup ... > logfile 2>&1 &`, return immediately, then poll the log with
  later `abox.sh` calls. A 15-seed sweep is a long job. Never hold one ssh
  session open for an hour.
- **Never inline a `pkill`/`pgrep` pattern in a remote command.** This box is
  shared with other tenants and other projects' servers; a stray pattern kills
  their work. If you think you need to kill something, kill it by the exact PID
  you launched.
- **Only GPU {gpu} is yours.** GPUs 0 and 7 belong to other tenants — never touch
  them. GPUs 1/2 run SAM3/GraspNet for everyone. GPU {gpu} is shared with several
  sibling agents working *different* tasks, and with a second campaign in another
  checkout. A LIBERO trial needs well under 1 GB and the card has ~75 GB free, so
  they all fit — but **expect your replays to be slower than a solo run, and
  expect wall-clock timings to vary between identical runs.** That is contention,
  not a defect in your program: never conclude anything about your fix from how
  long a replay took. Never set `CUDA_VISIBLE_DEVICES` to anything but {gpu}.
- **The perception servers are shared by every agent running right now**, so a
  replay may queue behind siblings. Expect replays to be slower than a solo run;
  that is normal and is not a reason to change your approach.
- To look at a keyframe image, `apull.sh` it to your local scratch dir and use
  `Read` on the local copy.
- `trace.json` can be large. Parse it on the box with
  `.venv-libero/bin/python3 -c ...` and print a summary; do not pull it whole.
- Disk: the box's `/` is 100% full. Write nothing outside
  `/mnt/data/YifanKang/`. `$TMPDIR` is already redirected there, so the
  template's `/tmp/...` paths below are fine to use verbatim *only* because
  `$TMPDIR` is set — but prefer explicit `/mnt/data/YifanKang/tmp/...` paths.
- **Disk is tight (a few hundred GB for ~50 sibling cells).** Put sweep and probe
  output under `/mnt/data/YifanKang/tmp/<something unique to your cell>` and
  delete a sweep directory once you have read its numbers out. Do not keep every
  version's full output tree.
- Your local scratch dir for pulled images and drafted code is
  `{scratch}` (create it).

Sanity-check the servers before your first replay (404 = UP, 000 = DOWN):

```
{scratch}/abox.sh 'for p in 8114 8115 8116; do echo -n "port $p: "; curl -s -o /dev/null -w "%{{http_code}}\\n" --max-time 3 http://127.0.0.1:$p/health; done'
```

If any is DOWN, stop and report it — do not try to restart it yourself, the
coordinator owns the services.

---

"""


def cell_key(suite: str, task: str) -> str:
    return f"{suite}__{task}"


def extract_template() -> str:
    text = TEMPLATE_SRC.read_text()
    start = text.index("<!-- ==================== TEMPLATE START ==================== -->")
    end = text.index("<!-- ==================== TEMPLATE END ==================== -->")
    body = text[start:end].split("-->", 1)[1]
    return body.strip() + "\n"


def build(suite: str, task: str, gpu: int) -> str:
    key = cell_key(suite, task)
    scratch = f"{S}/agent_{ARM}_{key}"
    body = extract_template()
    if SUITE_PLACEHOLDER not in body:
        raise SystemExit("suite placeholder not found — ASPIRE's template changed, check it")
    body = body.replace(SUITE_PLACEHOLDER, f"SUITE: {suite}")
    body = body.replace("TASK:  <task_name_with_underscores>", f"TASK:  {task}")
    body = body.replace("GPU:   <3|4|5|6|7>", f"GPU:   {gpu}")
    # Make the template's own $SUITE/$TASK/$GPU literal: each abox.sh call is a
    # fresh shell, so an unexpanded var would silently become the empty string
    # and the agent would run against suite "" .  $TASK_DIR is a real variable
    # the template defines itself -- shield it before substituting $TASK, or it
    # becomes "<task>_DIR".
    body = body.replace("$TASK_DIR", "\x00TD\x00")
    body = body.replace("${SUITE}", suite).replace("${TASK}", task)
    body = body.replace("$GPU", str(gpu)).replace("$SUITE", suite).replace("$TASK", task)
    body = body.replace("\x00TD\x00", "$TASK_DIR")
    return HOST_PREAMBLE.format(gpu=gpu, scratch=scratch) + body


def materialize_scratch(suite: str, task: str) -> pathlib.Path:
    scratch = S / f"agent_{ARM}_{cell_key(suite, task)}"
    scratch.mkdir(parents=True, exist_ok=True)
    (scratch / ".claude").mkdir(exist_ok=True)
    (scratch / ".claude" / "settings.json").write_text(
        '{"permissions":{"allow":["Bash","Read","Edit","Write","Grep","Glob","TodoWrite"]}}\n'
    )
    (scratch / "abox.sh").write_text((S / f"abox_{ARM}.sh").read_text())   # this arm's checkout
    for h in ("apush.sh", "apull.sh"):
        (scratch / h).write_text((S / h).read_text())
    for h in ("abox.sh", "apush.sh", "apull.sh"):
        (scratch / h).chmod(0o755)
    return scratch


def write_cell(suite: str, task: str, gpu: int) -> pathlib.Path:
    materialize_scratch(suite, task)
    outdir = S / "prompts"
    outdir.mkdir(exist_ok=True)
    p = outdir / f"arm{ARM}_{cell_key(suite, task)}.md"
    p.write_text(build(suite, task, gpu))
    return p


def load_cells():
    out = []
    for line in CELLS_FILE.read_text().split():
        suite, task = line.split("/")
        out.append((suite, task))
    return out


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--all":
        cells = load_cells()
        for suite, task in cells:
            p = write_cell(suite, task, 3)
            print(f"{suite}/{task}: {p.name} ({len(p.read_text())} chars)")
        # Every prompt must name its own suite/task and nobody else's.
        print("\n--- cell-identity check ---")
        bad = 0
        for suite, task in cells:
            txt = (S / "prompts" / f"arm{ARM}_{cell_key(suite, task)}.md").read_text()
            if f"SUITE: {suite}" not in txt or f"TASK:  {task}" not in txt:
                print(f"BAD {suite}/{task}: header missing"); bad += 1; continue
            others = {s for s, _ in cells if s != suite}
            leaked = sorted(s for s in others if s in txt)
            if leaked:
                print(f"BAD {suite}/{task}: mentions other suites {leaked}"); bad += 1
        print("CELL IDENTITY CHECK:", "PASS" if bad == 0 else f"FAIL ({bad})")
        sys.exit(1 if bad else 0)
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    suite, task, gpu = sys.argv[1], sys.argv[2], int(sys.argv[3])
    print(write_cell(suite, task, gpu))
