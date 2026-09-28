"""RoboDojo per-task successes (of 50) for the page: K=3, images only, K=0 (paper Table 2)."""
import os
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Helvetica", "Arial", "DejaVu Sans"], "font.size": 11})
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
# task, K=3, images only (None = not run), K=0
ROWS = [("build tower", 36, None, 0), ("make kong", 36, None, 0), ("bottles to dustbin", 15, None, 0), ("classify objects", 8, None, 0),
        ("press buttons", 50, 50, 0), ("stack three blocks", 43, 0, 10), ("swap two T blocks", 47, 15, 38),
        ("push T onto outline", 18, 8, 0), ("insert tubes", 14, 0, 2), ("tic-tac-toe", 5, 21, 4)]
C3, CI, C0 = "#1565c0", "#9aa3ad", "#6cb6e6"
fig, ax = plt.subplots(figsize=(6.2, 6.4))
h = 0.26
ys = []
for i in range(len(ROWS)):
    ys.append(-(i + (0.9 if i >= 4 else 0)))          # a gap between the two groups
for i, (name, k3, im, k0) in enumerate(ROWS):
    y = ys[i]
    ax.barh(y + h, k3, h, color=C3, label="K=3" if i == 0 else None)
    if im is not None:
        ax.barh(y, im, h, color=CI, label="images only" if i == 4 else None)
    ax.barh(y - h, k0, h, color=C0, label="K=0" if i == 0 else None)
    for dy, v in ((h, k3), (0, im), (-h, k0)):
        if v is not None:
            ax.text(v + 0.8, y + dy, str(v), va="center", fontsize=8.5, color="#444")
ax.text(0, ys[0] + 0.62, "Goal unstated in the sentence", fontsize=10, color="#16181b", fontweight="bold", va="bottom")
ax.text(0, ys[4] + 0.62, "Contact-sensitive", fontsize=10, color="#16181b", fontweight="bold", va="bottom")
ax.set_yticks(ys); ax.set_yticklabels([r[0] for r in ROWS])
ax.set_ylim(ys[-1] - 0.7, ys[0] + 1.05)
ax.set_xlim(0, 54); ax.set_xticks([0, 10, 20, 30, 40, 50]); ax.set_xlabel("successes out of 50")
ax.spines[["top", "right"]].set_visible(False)
ax.legend(frameon=False, loc="lower right", fontsize=9.5)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "media", "robodojo_chart.png"), dpi=220)
print("ok")
