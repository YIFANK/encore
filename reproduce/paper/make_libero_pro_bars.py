#!/usr/bin/env python3
"""Fig. LIBERO-PRO bars: ASPIRE (opus-5 rerun) vs Encore K=0 / K=3, per suite and axis.
Encore columns from autoresearch/campaigns/c2clean/final_counts_clean.json (50 sealed seeds
per cell, 10 cells per suite x axis); ASPIRE opus-5 rerun as in make_tables.py (BASE[...][3])."""
import json, os
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams.update({"pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 8.5})
import matplotlib.pyplot as plt
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
d = json.load(open(os.path.join(REPO, "autoresearch/campaigns/c2clean/final_counts_clean.json")))
ASPIRE5 = {("obj", "pos"): 100, ("obj", "task"): 100, ("goal", "pos"): 86, ("goal", "task"): 86,
           ("spa", "pos"): 84, ("spa", "task"): 80}
def enc(arm, suite, ax):
    cs = [c for c in d[arm] if c.startswith(suite + "_") and c.endswith("_" + ax)]
    assert len(cs) == 10, (arm, suite, ax, len(cs))
    return 100 * sum(d[arm][c] for c in cs) / (50 * len(cs))
suites = [("obj", "object"), ("goal", "goal"), ("spa", "spatial")]
arms = [("ASPIRE (opus-5 rerun)", "#9aa3ad", lambda s, a: ASPIRE5[(s, a)]),
        (r"Encore $K{=}0$", "#2d3440", lambda s, a: enc("k0", s, a)),
        (r"Encore $K{=}3$", "#ad7614", lambda s, a: enc("k3", s, a))]
fig, axes = plt.subplots(1, 2, figsize=(5.6, 2.1), sharey=True)
w = 0.26; x = np.arange(len(suites))
for ax, (axis, title) in zip(axes, (("pos", "Pos: layout perturbation"), ("task", "Task: instruction rewrite"))):
    for i, (lab, col, f) in enumerate(arms):
        v = [f(s, axis) for s, _ in suites]
        b = ax.bar(x + (i - 1) * w, [u / 100 for u in v], w, color=col, label=lab)
        for r, u in zip(b, v):
            ax.text(r.get_x() + r.get_width() / 2, r.get_height() + 0.008, f"{u:.0f}", ha="center", va="bottom", fontsize=6.2)
    ax.set_xticks(x); ax.set_xticklabels([n for _, n in suites]); ax.set_title(title, fontsize=9)
    ax.set_ylim(0.5, 1.06); ax.spines[["top", "right"]].set_visible(False)
axes[0].set_ylabel("sealed success")
h, l = axes[0].get_legend_handles_labels()
fig.legend(h, l, loc="upper center", ncol=3, frameon=False, fontsize=8, bbox_to_anchor=(0.5, 1.0))
fig.tight_layout(rect=(0, 0, 1, 0.88))
fig.savefig(os.path.join(HERE, "libero_pro_bars.pdf"), bbox_inches="tight")
print({(s, a): (round(enc("k0", s, a)), round(enc("k3", s, a))) for s, _ in suites for a in ("pos", "task")})
