# Campaign c2 — information-fair rerun (FAIR_PROTOCOL v1.0)

Matrix: same 90 cells as c1 (3 suites x 10 tasks x {stock,pos,task}), namespace
`c2_`. Wave order: stock first; pos/task gated on the task's stock being
terminal. Splits: debug seeds 51-65 / eval seeds 1-50 (blind, coordinator-run,
via tools/fair_run.py --split eval). Laws promoted only from c2-clean
derivations with debug receipts; LAWS.md starts EMPTY. c1 artifacts are
CONTAMINATED context — never referenced in briefs or laws.
