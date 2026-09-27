#!/usr/bin/env python3
"""Generate Stage 1 subagent prompts for both arms.

Arm A and Arm B prompts are byte-identical except for ARM_B_ADDENDUM, which is
the ONLY delta the ablation allows. Everything else -- ASPIRE's own template,
the host-access preamble, SUITE/TASK/GPU -- is shared text, generated once.
"""
import pathlib
import sys

S = pathlib.Path("/Users/yifankang/aspire_ab_run")  # pinned: prompts must not depend on where this file resolves
TEMPLATE_SRC = S / "runbook" / "fix-loop" / "subagent-prompt.md"

# All TEN libero_goal_swap tasks (QUICKSTART.md allowlist order).
#
# We have three task GPUs (4/5/6), not ASPIRE's five, so the ten tasks run in
# four waves. Within a wave, each GPU carries exactly one task's TWO arms --
# same task, opposite arms -- so the two halves of every comparison contend for
# the same card under the same load. Both arms of a task always share a GPU and
# a wave, so no cell is advantaged by scheduling.
#
# (task, gpu, wave)
TASKS = [
    ("open_the_middle_drawer_of_the_cabinet", 4, 1),
    ("put_the_bowl_on_the_stove", 5, 1),
    ("put_the_wine_bottle_on_top_of_the_cabinet", 6, 1),
    ("open_the_top_drawer_and_put_the_bowl_inside", 4, 2),
    ("put_the_bowl_on_top_of_the_cabinet", 5, 2),
    ("push_the_plate_to_the_front_of_the_stove", 6, 2),
    ("put_the_cream_cheese_in_the_bowl", 4, 3),
    ("turn_on_the_stove", 5, 3),
    ("put_the_bowl_on_the_plate", 6, 3),
    ("put_the_wine_bottle_on_the_rack", 4, 4),
]
SUITE = "libero_goal_swap"

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
  them. GPUs 1/2 run SAM3/GraspNet for everyone. GPU {gpu} is shared with exactly
  one sibling agent working the same task in the other arm's repo checkout; the
  card has ~75 GB free, so both fit. Other sibling agents own the remaining task
  GPUs. Never set `CUDA_VISIBLE_DEVICES` to anything but {gpu}.
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
- Your local scratch dir for pulled images and drafted code is
  `{scratch}` (create it).

Sanity-check the servers before your first replay (404 = UP, 000 = DOWN):

```
{S}/abox.sh 'for p in 8114 8115 8116; do echo -n "port $p: "; curl -s -o /dev/null -w "%{{http_code}}\\n" --max-time 3 http://127.0.0.1:$p/health; done'
```

If any is DOWN, stop and report it — do not try to restart it yourself, the
coordinator owns the services.

---

"""

ARM_B_ADDENDUM = """\

---

## Additional measurement source available for this task

K=3 human demonstrations for this task are available on the box at
`{demo_path}` (hdf5: obs/actions/ee poses). You may consult them as measurement
sources. See `/mnt/data/YifanKang/aspire_demos_k3/DEMONSTRATIONS.md` for the file
layout.
"""


def extract_template() -> str:
    text = TEMPLATE_SRC.read_text()
    start = text.index("<!-- ==================== TEMPLATE START ==================== -->")
    end = text.index("<!-- ==================== TEMPLATE END ==================== -->")
    body = text[start:end]
    body = body.split("-->", 1)[1]
    return body.strip() + "\n"


def build(arm: str, task: str, gpu: int) -> str:
    scratch = f"{S}/agent_{arm}_{task}"
    body = extract_template()
    body = body.replace(
        "SUITE: <libero_goal_swap|libero_goal_task|libero_object_swap|libero_object_task|libero_spatial_swap|libero_spatial_task>",
        f"SUITE: {SUITE}",
    )
    body = body.replace("TASK:  <task_name_with_underscores>", f"TASK:  {task}")
    body = body.replace("GPU:   <3|4|5|6|7>", f"GPU:   {gpu}")
    # ASPIRE's template writes $GPU/$SUITE/$TASK inline; make them literal so a
    # fresh remote shell cannot silently expand them to empty strings.
    # $TASK_DIR is a real shell variable the template defines itself -- shield it
    # before substituting $TASK, or it becomes "<task>_DIR".
    body = body.replace("$TASK_DIR", "\x00TD\x00")
    body = body.replace("${SUITE}", SUITE).replace("${TASK}", task)
    body = body.replace("$GPU", str(gpu)).replace("$SUITE", SUITE).replace("$TASK", task)
    body = body.replace("\x00TD\x00", "$TASK_DIR")

    preamble = HOST_PREAMBLE.format(S=S, gpu=gpu, scratch=scratch)
    out = preamble + body
    if arm == "B":
        demo_path = f"/mnt/data/YifanKang/aspire_demos_k3/{task}/{task}_K3_demos.hdf5"
        out += ARM_B_ADDENDUM.format(demo_path=demo_path)
    return out


def materialize_scratch(arm: str, task: str) -> pathlib.Path:
    """Give each agent its own abox/apush/apull, wired to that arm's checkout.

    The prompts must stay arm-symmetric (the demo addendum is the only allowed
    delta), so the arm-specific repo path cannot appear in the prompt text. It
    lives here instead, inside the per-agent helper copies.
    """
    scratch = S / f"agent_{arm}_{task}"
    scratch.mkdir(parents=True, exist_ok=True)
    (scratch / ".claude").mkdir(exist_ok=True)
    (scratch / ".claude" / "settings.json").write_text(
        '{"permissions":{"allow":["Bash","Read","Edit","Write","Grep","Glob","TodoWrite"]}}\n'
    )
    src = "abox.sh" if arm == "A" else "abox_armB.sh"
    (scratch / "abox.sh").write_text((S / src).read_text())
    for h in ("apush.sh", "apull.sh"):
        (scratch / h).write_text((S / h).read_text())
    for h in ("abox.sh", "apush.sh", "apull.sh"):
        (scratch / h).chmod(0o755)
    return scratch


if __name__ == "__main__":
    outdir = S / "prompts"
    outdir.mkdir(exist_ok=True)
    for arm in ("A", "B"):
        for task, gpu, wave in TASKS:
            materialize_scratch(arm, task)
            p = outdir / f"arm{arm}_{task}.md"
            p.write_text(build(arm, task, gpu))
            print(f"wave {wave} gpu {gpu}: wrote {p.name} ({len(p.read_text())} chars)")
    # The ablation contract, asserted on content rather than on hunk count:
    # EVERY line that differs between the two arms must be either
    #   (a) a scratch-directory path line, identical once agent_A_/agent_B_ is
    #       normalised away (the agents need different scratch dirs), or
    #   (b) part of the Arm B demo addendum, which is the treatment itself.
    # Anything else is a leak and fails the check.
    import difflib
    print("\n--- ablation content check ---")
    bad = 0
    for task, gpu, wave in TASKS:
        a = (outdir / f"armA_{task}.md").read_text().splitlines()
        b = (outdir / f"armB_{task}.md").read_text().splitlines()
        addendum = set(ARM_B_ADDENDUM.format(
            demo_path=f"/mnt/data/YifanKang/aspire_demos_k3/{task}/{task}_K3_demos.hdf5"
        ).splitlines())
        leaks, scratch_lines, addendum_lines = [], 0, 0
        norm = lambda s: s.replace(f"agent_A_{task}", "AGENT").replace(f"agent_B_{task}", "AGENT")
        for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, a, b).get_opcodes():
            if tag == "equal":
                continue
            la, lb = a[i1:i2], b[j1:j2]
            if len(la) == len(lb) and all(norm(x) == norm(y) for x, y in zip(la, lb)):
                scratch_lines += len(la)          # (a) pure scratch-path rename
                continue
            if not la and all(x in addendum for x in lb):
                addendum_lines += len(lb)         # (b) the treatment, Arm B only
                continue
            leaks.extend(la + lb)
        if leaks:
            bad += 1
            print(f"BAD {task}: {len(leaks)} leaked line(s): {leaks[:3]}")
        else:
            print(f"OK  {task}: {scratch_lines} scratch-path line(s), "
                  f"{addendum_lines} addendum line(s), 0 leaks")
    print("ABLATION CONTENT CHECK:", "PASS" if bad == 0 else f"FAIL ({bad} tasks)")
