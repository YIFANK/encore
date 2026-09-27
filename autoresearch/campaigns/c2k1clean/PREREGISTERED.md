# c2k1clean: K=1 arm under the clean brief (2026-09-25)

Same brief, harness, model (claude-opus-5), development band (seeds 51-65),
budget and sealed evaluation (seeds 1-50, blind, once) as c2clean; the only
change is one demonstration per pack instead of three (fresh program-free packs
`packs/c2k1clean_<cell>_k1`, `_mate` for Task cells, first demo of the stock
task's official hdf5, `tools/fair_pack.py --k 1`).

Prediction, written before any cell runs: K=1 lands within 2 points of c2clean
K=3 (96.3%) over the 60 cells, and open_middle_drawer-Task (K=3 50/50, K=0 0/50)
stays at or above 40/50 with one demonstration.
