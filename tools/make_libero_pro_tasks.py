#!/usr/bin/env python3
"""Generate LIBERO-PRO perturbed suites and the task list Heron evaluates on.

LIBERO-PRO's own driver has three traps that silently corrupt an evaluation, so
they are handled here rather than worked around later:

  * every perturbation writes to `<suite>_temp`, so consecutive runs overwrite
    each other — each combination is renamed to `<suite>_<perturbation>`;
  * the spatial swap is seeded from process state, not from the config (its
    `seed` key is absent from the shipped YAML, and the code then passes the
    TYPE `int` as the seed), so runs are irreproducible — we seed explicitly;
  * for task-redefinition suites the benchmark derives the instruction from the
    FILENAME, which still describes the original task. A model given that would
    be graded against a goal it was never asked for. We parse the instruction
    out of the perturbed bddl instead.

    python tools/make_libero_pro_tasks.py --num-inits 4 --tasks-per-suite 3
"""
from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import subprocess
import sys
from pathlib import Path

PRO = Path("/mnt/data/YifanKang/LIBERO-PRO")
SUITES = ["libero_object", "libero_spatial", "libero_goal"]
PERTURBATIONS = ["swap", "task"]
LANGUAGE_RE = re.compile(r"\(:language\s+(.+?)\)", re.S)


def read_language(bddl: Path) -> str:
    m = LANGUAGE_RE.search(bddl.read_text())
    return " ".join(m.group(1).split()) if m else ""


def perturb(suite: str, perturbation: str, seed: int, force: bool) -> Path:
    """Produce <suite>_<perturbation>/ bddl files; return that directory."""
    sys.path.insert(0, str(PRO))
    import perturbation as P  # noqa: PLC0415

    src = PRO / "libero/libero/bddl_files" / suite
    out = PRO / "libero/libero/bddl_files" / f"{suite}_{perturbation}"
    if out.exists() and not force:
        print(f"  {out.name} exists, reusing")
        return out
    tmp = PRO / "libero/libero/bddl_files" / f"{suite}_temp"
    shutil.rmtree(tmp, ignore_errors=True)

    flags = P.PerturbFlags(
        use_environment=False, use_swap=(perturbation == "swap"),
        use_object=False, use_language=False, use_task=(perturbation == "task"),
    )
    configs = {k: str(PRO / v) for k, v in {
        "environment": "libero_ood/ood_environment.yaml",
        "swap": "libero_ood/ood_spatial_relation.yaml",
        "object": "libero_ood/ood_object.yaml",
        "language": "libero_ood/ood_language.yaml",
        "task": "libero_ood/ood_task.yaml",
    }.items()}

    random.seed(seed)  # the swap perturbator reads the global RNG, not the config
    P.process_bddl_file_mixed(str(src), suite, flags, configs, seed=seed)
    shutil.rmtree(out, ignore_errors=True)
    tmp.rename(out)
    print(f"  wrote {len(list(out.glob('*.bddl')))} bddl files -> {out.name}")
    return out


def make_init_states(bddl_dir: Path, suite: str, perturbation: str,
                     num_inits: int, force: bool) -> Path:
    out = PRO / "libero/libero/init_files" / f"{suite}_{perturbation}"
    if out.exists() and list(out.glob("*.pruned_init")) and not force:
        print(f"  init states exist, reusing ({len(list(out.glob('*.pruned_init')))})")
        return out
    out.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(PRO / "notebooks/generate_init_states.py"),
           "--bddl_base_dir", str(bddl_dir), "--output_dir", str(out),
           "--num_inits", str(num_inits)]
    print(f"  generating init states ({num_inits}/task) …", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        raise RuntimeError(f"init-state generation failed for {bddl_dir.name}")
    print(f"  wrote {len(list(out.glob('*.pruned_init')))} init files")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=28)
    ap.add_argument("--num-inits", type=int, default=4)
    ap.add_argument("--tasks-per-suite", type=int, default=3)
    ap.add_argument("--out", default="/mnt/data/YifanKang/Heron/libero_pro_tasks.json")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    tasks = []
    for suite in SUITES:
        for pert in PERTURBATIONS:
            print(f"\n== {suite} / {pert}")
            bddl_dir = perturb(suite, pert, args.seed, args.force)
            init_dir = make_init_states(bddl_dir, suite, pert, args.num_inits, args.force)
            # Deterministic subset: the first N by filename, so re-runs of the
            # sweep compare like with like.
            for bddl in sorted(bddl_dir.glob("*.bddl"))[: args.tasks_per_suite]:
                init = init_dir / f"{bddl.stem}.pruned_init"
                if not init.exists():
                    print(f"  SKIP {bddl.stem}: no init states")
                    continue
                tasks.append({
                    "id": f"{suite}-{pert}-{bddl.stem[:40]}",
                    "suite": suite,
                    "perturbation": pert,
                    "bddl_file": str(bddl),
                    "init_states_file": str(init),
                    "language": read_language(bddl),
                })
    Path(args.out).write_text(json.dumps(tasks, indent=1))
    print(f"\n{len(tasks)} tasks -> {args.out}")
    for t in tasks:
        print(f"  {t['id']}: {t['language']!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
