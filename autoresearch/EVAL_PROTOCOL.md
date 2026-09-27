# Encore Evaluation Protocol — v1.0 (frozen)

The canonical evaluation contract for all reported numbers. Matched to
ASPIRE's protocol (arXiv:2607.00272, §3.3, Tables 7-8 notes, Appendix
E.2) wherever a counterpart exists; stricter where our audits found
holes. The dev campaign (tag `dev-campaign-final`) predates this
document and is reported as exploratory; every matched-row number comes
from a run under this contract.

## 1. Principles

1. **Three-way split.** Learn (iteration/probes), validation (version
   selection), evaluation (touched exactly once, by the final frozen
   program). No information may flow from evaluation instances into any
   program-writing or version-selection decision.
2. **Success** is the benchmark's own goal predicate, recorded by the
   runner (`benchmark_success`); crashes count as failures.
3. **Argmax on validation only.** The reported program is the argmax
   over archived versions *by validation score*; evaluation is run once
   on that program. Iteration cannot lower a banked number, but it also
   cannot peek.
4. **Matched observability** (ASPIRE Appendix E.2, adopted verbatim for
   matched campaigns): simulator state is forbidden everywhere —
   including probes, debug scripts, and anchor measurement; benchmark
   asset files (.bddl/.xml/.urdf) may not be read; "if a real robot
   with a camera could do it, it is allowed." The task is defined by
   the intent sentence and the K=3 demonstrations alone.
5. **Provenance.** Every run archives its exact program
   (`program_archived.py`) and per-episode results; every reported
   number is regenerable by re-running a frozen artifact.

## 2. Splits per benchmark

| benchmark | instance source | learn | validation | evaluation (once) |
|---|---|---|---|---|
| LIBERO-PRO stock / LONG | official 50-state init file | states 40-49 | states 30-39 | states 0-29 (n=30) |
| LIBERO-PRO Pos / Task | seeded resets from the perturbation bddl (`--seed-episodes`; a seed fixes the full instance) | seeds 101-115 | seeds 116-130 | seeds 1-50 (n=50) |
| Robosuite (robomimic PH tasks) | seeded resets (`--robosuite`) | seeds 101-115 | seeds 116-125 | seeds 1-100 (n=100) |

ASPIRE counterparts: LIBERO-PRO learn 51-65 / validation 66-80 / eval
1-50; Robosuite learn 101-125 / eval 1-100. Sample sizes match on the
perturbation axes and Robosuite; our stock row (n=30) has no ASPIRE
counterpart (they do not report unperturbed suites).

Notes: the official LIBERO-PRO perturbation init files contain only 4
states per task (two spatial-Task files are empty); they are reported,
if at all, as a secondary "official-file" row labeled n=4 layouts.
Two spatial-Task bddls ship with an unbalanced parenthesis; evaluation
uses syntax-repaired copies differing by exactly that character.

## 3. Reporting rows

Per cell, in order of strictness:
- **matched** — the protocol above (headline row, ASPIRE-comparable).
- **pristine** (where a cell's history predates this protocol) — the
  frozen argmax program re-evaluated once on never-touched seeds;
  labeled with the seed range.
- **official-file** (LIBERO only) — the 4-state official perturbation
  file, labeled as such.

Statistics: Wilson 95% CIs on all proportions; two-proportion z-tests
for any A-vs-B claim, always annotated that cross-paper tests quantify
sampling noise, not protocol equivalence; per-task comparisons use
ASPIRE's Appendix D tables.

## 4. Session-side rules (enforced by audit)

- Probe `--episode-list` ⊆ learn. Iteration formals ⊆ learn ∪
  validation. Workers never execute the evaluation range: the
  coordinator runs the single evaluation command itself on the declared
  frozen version. Any worker-initiated run touching the evaluation
  range voids the cell (recover via a pristine re-evaluation on fresh
  seeds).
- Audit battery (per campaign, over all transcripts): (i) init-states
  presence on stock formals; (ii) episode-list range compliance;
  (iii) no writes to any results.jsonl; (iv) demo indices 0-2 only;
  (v) namespace clean-room; (vi) no simulator-state or asset-file
  access in matched campaigns.
