#!/usr/bin/env python3
"""collect_results.py — pull every held-out manifest off the box and build the A/B table.

Reads ASPIRE's own run manifests (the authoritative record: they hash the fix
code + config + seed set into a run identity), never a directory glob. Writes
the merged JSON to /mnt/data/YifanKang/aspire_ab_results/ab_heldout.json and
prints a markdown table for NOTES.md.
"""
import json
import pathlib
import subprocess
import sys

S = pathlib.Path("/Users/yifankang/aspire_ab_run")
TASKS = ["turn_on_the_stove", "put_the_bowl_on_the_plate",
         "open_the_middle_drawer_of_the_cabinet"]
SUITE = "libero_goal_swap"

REMOTE = r"""
import glob, json, os
suite, task = "%s", "%s"
base = os.path.join("outputs/libero_fix_loop_eval", suite, task, "runs")
out = []
for m in glob.glob(os.path.join(base, "*", "manifest.json")):
    d = json.load(open(m))
    out.append({
        "run_id": d.get("run_id"), "status": d.get("status"),
        "passes": d.get("passes"), "trials": d.get("trials"),
        "pass_rate": d.get("pass_rate"),
        "evidence_scope": d.get("evidence_scope"),
        "code_sha256": d.get("identity", {}).get("code_sha256"),
        "n_seeds": len(d.get("identity", {}).get("seeds", [])),
        "updated_at": d.get("updated_at"),
    })
print(json.dumps(out))
"""


def fetch(arm: str, task: str):
    box = S / f"agent_{arm}_{task}" / "abox.sh"
    code = REMOTE % (SUITE, task)
    body = (
        "cat > /mnt/data/YifanKang/tmp/_collect.py <<'PYEOF'\n"
        + code
        + "\nPYEOF\n.venv-libero/bin/python3 /mnt/data/YifanKang/tmp/_collect.py\n"
    )
    r = subprocess.run([str(box)], input=body, capture_output=True, text=True)
    for line in r.stdout.splitlines():
        line = line.strip()
        if line.startswith("["):
            try:
                return json.loads(line)
            except Exception:
                pass
    print(f"  ! arm{arm}/{task}: no manifest JSON ({r.stdout.strip()[:120]})", file=sys.stderr)
    return []


def main():
    all_res = {}
    for arm in ("A", "B"):
        for task in TASKS:
            runs = fetch(arm, task)
            full = [r for r in runs if r.get("n_seeds") == 50]
            best = max(full or runs, key=lambda r: (r.get("trials") or 0), default=None)
            all_res[f"{arm}/{task}"] = {"chosen": best, "all_runs": runs}

    outp = pathlib.Path("/tmp/ab_heldout.json")
    outp.write_text(json.dumps(all_res, indent=2))
    subprocess.run([str(S / "agent_A_turn_on_the_stove" / "apush.sh"), str(outp),
                    "/mnt/data/YifanKang/aspire_ab_results/ab_heldout.json"])

    print()
    print("| Task | Arm A (ASPIRE) | Arm B (ASPIRE+demos) |")
    print("|---|---|---|")
    for task in TASKS:
        cells = []
        for arm in ("A", "B"):
            c = all_res[f"{arm}/{task}"]["chosen"]
            if not c or not c.get("trials"):
                cells.append("not run")
            else:
                cells.append(f"{c['passes']}/{c['trials']}"
                             + ("" if c["trials"] == 50 else " (PARTIAL)"))
        print(f"| {task} | {cells[0]} | {cells[1]} |")


if __name__ == "__main__":
    main()
