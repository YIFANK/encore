#!/usr/bin/env python3
"""Let the EVAL_NUM environment variable override a task's eval_nums in RoboDojo.

tools/fair_run_robodojo.py evaluates a private band of N renumbered layouts and
sets EVAL_NUM=N for the eval client. Upstream process_config() returns the
per-task eval_nums from _task.yml (25 for arrange_largest_number, fold_clothes,
hang_mugs, stack_blocks, press_by_number, ...), and the client runs
min(EVAL_NUM, eval_nums) episodes, so a 50-layout sealed band was silently cut
to 25. Idempotent; applied by install_encore.sh (found 2026-09-17).
"""
import pathlib
import sys

p = pathlib.Path(sys.argv[1]) / "utils/pipeline_utils.py"
s = p.read_text()
if 'os.environ.get("EVAL_NUM")' in s:
    print("pipeline_utils already patched"); sys.exit(0)
anchor = '    eval_num = task_info.get("eval_nums", common_info.get("eval_nums", 50))\n'
assert s.count(anchor) == 1, "anchor not found; pipeline_utils.py changed upstream"
add = ('    # Encore fair harness: a private band of N renumbered layouts must be evaluated\n'
       '    # in full; the per-task eval_nums would truncate a 50-layout band to 25.\n'
       '    if os.environ.get("EVAL_NUM"):\n'
       '        eval_num = int(os.environ["EVAL_NUM"])\n')
s = s.replace(anchor, anchor + add)
if not any(l.strip() in ("import os", "import os, sys") or l.startswith("import os") for l in s.splitlines()):
    s = "import os\n" + s
p.write_text(s)
print("pipeline_utils patched")
