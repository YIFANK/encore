# Frozen programs and results

Every program the paper evaluates is here, exactly as it was frozen, next to
its per-episode results. You can run any of them yourself; no agent is needed.

```
reproduce/sealed/<experiment>/<cell>/
    program.py         the frozen program
    results.jsonl      one line per evaluation episode (success, steps, time)
```

`sealed/MANIFEST.tsv` lists every cell with its number of episodes, number of
successes, and the md5 of its program.

## Run a program

Set up the harness and benchmark first (see the [main README](../README.md#setup)).

**LIBERO-PRO.** The BDDL file and task sentence for each cell are in
`autoresearch/campaigns/c2clean/eval_manifest.txt`.

```bash
env -u PYTHONPATH .venv/bin/python tools/fair_run.py program --seed-episodes \
  --bddl <bddl file> --language "<task sentence>" \
  --program reproduce/sealed/libero_pro_clean/<cell>/program.py \
  --split eval --out results/eval_<cell>
```

**RoboDojo.**

```bash
env -u PYTHONPATH .venv/bin/python tools/fair_run_robodojo.py --task <task> \
  --program reproduce/sealed/robodojo_rd2/<cell>/program.py \
  --split eval --eval-n 50 --gpu 0 --out results/eval_<cell>
```

The evaluation episodes are fixed (LIBERO-PRO seeds 1–50; the first 50 official
RoboDojo evaluation layouts), so a rerun should land within a few episodes of the
saved result. Programs that call `api.ground` or `api.vqa` also depend on the
vision-language model behind those calls.

## Which folder is which result

| Result in the paper | Folder in `sealed/` | Cells |
|---|---|---|
| LIBERO-PRO, K=3 and K=0 (Fig. 4, Tables 3–4) | `libero_pro_clean` | 120 |
| LIBERO-PRO, K=1 (Tables 3–4) | `libero_pro_k1` | 60 |
| Unperturbed LIBERO-PRO tasks | `libero_stock` | 30 |
| RoboDojo, ten tasks (Table 2, left) | `robodojo_rd1` | 19 |
| RoboDojo, twelve tasks (Table 2, right) | `robodojo_rd2` | 36 |
| RoboDojo, K=1 (Table 2) | `robodojo_k1` | 21 |
| Verification ablation (Table 5) | `verification_ablation` | 9 |
| LIBERO-90 fixture study | `libero90_fixtures` | 24 |

ASPIRE baseline results are in `reproduce/aspire/`, and the environment
versions we used (package lists, benchmark commits, the RoboDojo patch) are in
`reproduce/env/`.

## Notes

- **Grounding model.** RoboDojo evaluations that ran from September 24 on use
  `gemini-3.5-flash` through an OpenAI-compatible relay (`configs/robodojo_ppapi.yaml`)
  for `api.ground` and `api.vqa`; earlier ones used Gemini Robotics-ER.
- **Earlier agent brief.** The verification ablation and the LIBERO-90 study ran
  with an earlier version of the agent brief, as the paper states.
- **Not included.** Agent session transcripts, demonstration packs (rebuilt from
  the benchmarks' own demonstration data by `tools/fair_pack*.py`), and episode videos.
