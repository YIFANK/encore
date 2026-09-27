# c2clean — K=3 and K=0 re-acquired with NO shared note file. Pre-registered 2026-09-12, before any cell ran.

## Why

Every worker in c2fix (K=3), c2k0 (K=0), c2k1 (K=1) and abl_noact received one
identical 72-line `LAWS.md` (md5 afefe8fa…), seeded from the unperturbed c2
campaign; no worker wrote to it. Three of its seven entries tell an agent how
to read a demonstration (closing-keyframe anchor, near-zero closing width =>
rim pinch, pre-release EEF points => fixture pose). It is therefore a fixed
prior, identical across arms, not a memory — but it is an input the paper's
Method does not want to carry. This campaign removes it entirely so the method
is exactly distiller + API + development loop, and doubles as a second
independent acquisition of both arms on all sixty perturbation cells.

## What changes, and only this

* No `LAWS.md` in any workspace; the brief says nothing crosses cells.
* Pack namespaces `packs/c2clean_<cell>_k3` (pack.json + keyframes copied from
  `packs/c2fix_<cell>`, program-free), `packs/c2clean_<cell>_mate` for the 27
  two-pack `_task` cells, `packs/c2clean_<cell>_k0` empty output dirs.
* Everything else identical to c2fix / c2k0: same sixty cells, same intent
  sentences, same debug band 51–65, one formal 15-seed selection, md5 freeze,
  PROVENANCE, sealed coordinator eval on seeds 1–50 once per cell, inner model
  opus-5, 250-turn cap, fresh agent per cell.

## Predictions, written before the run

1. **The note file was not load-bearing.** K=3 lands within 2 points of
   95.8 % over the sixty cells and K=0 within 3 points of 87.4 %.
2. **The eight-cell structure replicates.** On the eight mechanism-gap cells
   of the original K=0 draw, K=3 − K=0 ≥ 30 points on at least six of eight.
3. **The demo-reading notes did not create the closing-width mechanism.** On
   the four cells attributed to the closing-width scalar, clean K=3 scores
   ≥ 40/50 on at least three.

Failure directions: if (1) fails for K=3 by > 5 points, the prior file was a
component and the paper must say so (or report only the clean numbers). If
(2) fails, the eight-cell claim is softened to the single original draw and
draw variance is reported per cell. If (3) fails, the "what the demonstration
supplies" paragraph is rewritten around the LIBERO-90 rule only.

## Reporting

Both draws are reported: per-cell min/median/max over the two acquisitions of
each arm. The clean draw is the paper's primary number if prediction 1 holds.
