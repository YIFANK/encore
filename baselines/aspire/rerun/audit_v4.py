#!/usr/bin/env python3
"""audit_v4.py — RUNS ON THE BOX. Collect + audit the Arm A' 58-cell column.

Reads ASPIRE's own run manifests (authoritative: they hash fix code, config and
the requested seed set into an immutable run identity), never a directory glob,
and then re-derives every claim from the EXECUTED data rather than trusting the
manifest's summary:

  executed_seed_set     the seeds actually present in results.jsonl / trial dirs
  dev_leak              any executed seed in 51-65 -> the held-out run is void
  duplicate_trials      the same trial index evaluated twice
  recount               trial directories on disk vs manifest.trials
  code_hash_match       manifest identity.code_sha256 vs the DELIVERED fix_code.py
  one_run_per_cell      more than one run dir means an ambiguous number
  evidence_scope        ASPIRE's own label; must be heldout_full

Pruned cells are handled: `prune_depth.py` deletes only `keyframes/*_depth_*.npy`,
so trial directories, their names, results.jsonl and manifest.json all survive
and the recount is unaffected. Where a prune ledger exists it is cross-checked
too, which catches a trial directory that vanished AFTER the prune.

Usage (on the box, cwd = an ASPIRE checkout's aspire/sim):
    .venv-libero/bin/python3 audit_v4.py <cells_file> [more_cells_file ...]
Prints JSON between JSONSTART/JSONEND.
"""
import glob
import hashlib
import json
import os
import re
import sys

EVAL = "outputs/libero_fix_loop_eval"
FIX = "outputs/libero_fix_loop"
LEDGERS = "/mnt/data/YifanKang/aspire_ab_results/prune_ledgers"
TRIAL_RE = re.compile(r"trial_(\d+)_")


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest() if os.path.exists(p) else None


def executed_seeds_from_dirs(run_dir):
    seeds, dirs = [], []
    for root, dnames, _f in os.walk(run_dir):
        for d in dnames:
            m = TRIAL_RE.match(d)
            if m:
                seeds.append(int(m.group(1)))
                dirs.append(d)
    return seeds, dirs


def executed_seeds_from_results(run_dir):
    """results.jsonl is the record the reported number is computed from."""
    seeds, passes = [], 0
    for f in glob.glob(os.path.join(run_dir, "**", "results.jsonl"), recursive=True):
        for line in open(f):
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except Exception:
                continue
            s = r.get("trial", r.get("seed"))
            if s is not None:
                seeds.append(int(s))
            if r.get("benchmark_success") or r.get("task_completed") or r.get("success"):
                passes += 1
    return seeds, passes


def audit_cell(suite, task):
    base = os.path.join(EVAL, suite, task, "runs")
    fc = os.path.join(FIX, suite, task, "fix_code.py")
    out = {
        "suite": suite, "task": task,
        "fix_code_exists": os.path.exists(fc),
        "findings_exists": os.path.exists(os.path.join(FIX, suite, task, "findings.md")),
        "delivered_code_sha256": sha(fc),
        "runs": [],
    }
    for m in sorted(glob.glob(os.path.join(base, "*", "manifest.json"))):
        d = json.load(open(m))
        run_dir = os.path.dirname(m)
        ident = d.get("identity", {}) or {}
        dir_seeds, dir_names = executed_seeds_from_dirs(run_dir)
        res_seeds, res_passes = executed_seeds_from_results(run_dir)
        seedset = sorted(set(dir_seeds))
        ledger_path = os.path.join(LEDGERS, os.path.abspath(run_dir).replace("/", "_").strip("_") + ".json")
        ledger_ok = None
        if os.path.exists(ledger_path):
            led = json.load(open(ledger_path))
            ledger_ok = sorted(led["trial_dirs"]) == sorted(dir_names)
        out["runs"].append({
            "run_id": d.get("run_id"), "status": d.get("status"),
            "passes": d.get("passes"), "trials": d.get("trials"),
            "pass_rate": d.get("pass_rate"),
            "evidence_scope": d.get("evidence_scope"),
            "code_sha256": ident.get("code_sha256"),
            "code_hash_match": (ident.get("code_sha256") == out["delivered_code_sha256"]),
            "n_trial_dirs": len(dir_seeds),
            "recount_matches_manifest": len(dir_seeds) == d.get("trials"),
            "duplicate_trials": len(dir_seeds) != len(set(dir_seeds)),
            "executed_seed_min": min(seedset) if seedset else None,
            "executed_seed_max": max(seedset) if seedset else None,
            "executed_is_exactly_1_50": seedset == list(range(1, 51)),
            "dev_seed_leak": sorted(s for s in seedset if 51 <= s <= 65),
            "results_jsonl_seeds": len(set(res_seeds)),
            "results_jsonl_passes": res_passes,
            "prune_ledger_consistent": ledger_ok,
            "path": m,
        })
    out["one_run_per_cell"] = len(out["runs"]) == 1
    return out


def main():
    cells = []
    for f in sys.argv[1:]:
        for line in open(f):
            line = line.strip()
            if line:
                cells.append(tuple(line.split("/")))
    res = {f"{s}/{t}": audit_cell(s, t) for s, t in cells}
    print("JSONSTART" + json.dumps(res) + "JSONEND")


if __name__ == "__main__":
    main()
