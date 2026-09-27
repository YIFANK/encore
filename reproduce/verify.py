#!/usr/bin/env python3
"""Recompute every quantitative claim of the Encore paper from
files in this repository, and check that each bundled frozen program is the one
that was evaluated.

    python3 reproduce/verify.py      # claim-by-claim table; exit 1 on any mismatch

Inputs (all in-repo; no cluster access needed):
  reproduce/sealed/MANIFEST.tsv and <experiment>/<cell>/{results.jsonl,program.py}
      per-episode sealed results and the archived frozen program of every
      evaluation the paper reports, collected from the evaluation box.
  autoresearch/campaigns/*/{eval_freeze.txt,cluster/*freeze*.txt}
      md5 of each program, recorded when it was frozen, before its evaluation.
  autoresearch/campaigns/c2clean/metrics_per_cell.json
      development-cost metrics (c2clean/metrics.py over the agent transcripts,
      which are not in the repository).
  reproduce/aspire/   ASPIRE held-out manifests from its own fix-loop validator.
"""
import csv, hashlib, json, os, re, statistics as S, sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SEALED = os.path.join(ROOT, "reproduce", "sealed")
CAMP = os.path.join(ROOT, "autoresearch", "campaigns")
ASP = os.path.join(ROOT, "reproduce", "aspire")

allrows = list(csv.DictReader(open(os.path.join(SEALED, "MANIFEST.tsv")), delimiter="\t"))
pending = [f'{r["experiment"]}/{r["cell"]}' for r in allrows if r["program_source"] == "missing"]
rows = [r for r in allrows if r["program_source"] != "missing"]   # not yet evaluated
by = defaultdict(dict)
for r in rows:
    by[r["experiment"]][r["cell"]] = r
succ = lambda e, c: int(by[e][c]["success"])
n_of = lambda e, c: int(by[e][c]["n"])
checks, fails = [], 0


def check(claim, got, want):
    global fails
    ok = got == want
    fails += not ok
    checks.append(("ok  " if ok else "FAIL", claim, got, want))


# ------------------------------------------------------------------ integrity
freeze = {}
for path, key in ((f"{CAMP}/c2clean/eval_freeze.txt", "libero_pro_clean"),
                  (f"{CAMP}/l90abl/eval_freeze.txt", "libero90_fixtures"),
                  (f"{CAMP}/rd1/cluster/rd1_eval_freeze.txt", "robodojo_rd1"),
                  (f"{CAMP}/rd2/cluster/rd2_eval_freeze.txt", "robodojo_rd2"),
                  (f"{CAMP}/rd2/cluster/rd2_eval50_freeze.txt", "robodojo_rd2"),
                  (f"{CAMP}/c2k1clean/eval_freeze.txt", "libero_pro_k1"),
                  (f"{CAMP}/k1rd/cluster/k1_rd_eval_freeze.txt", "robodojo_k1")):
    if os.path.isfile(path):
        for l in open(path):
            p = l.split()
            if len(p) >= 2:
                cell = p[0]
                if key == "robodojo_k1":  # recorded as <rd|rd2>_<task>_k1
                    cell = cell.split("_", 1)[1]
                freeze.setdefault((key, cell), set()).add(p[1])
bad_file, bad_freeze, checked = [], [], 0
for r in rows:
    p = os.path.join(SEALED, r["experiment"], r["cell"], "program.py")
    if hashlib.md5(open(p, "rb").read()).hexdigest() != r["program_md5"]:
        bad_file.append(r["cell"])
    f = freeze.get((r["experiment"], r["cell"]))
    if f is not None:
        checked += 1
        if r["program_md5"] not in f:
            bad_freeze.append(r["cell"])
check("bundled programs match their recorded md5 (mismatches)", len(bad_file), 0)
check("evaluated programs match an md5 frozen before evaluation", bad_freeze, [])
print(f"[integrity] {len(rows)} sealed evaluations, {checked} cross-checked against freeze records")

# ------------------------------------------------------------------ LIBERO-PRO
E = "libero_pro_clean"
arm = lambda a: {c[:-3]: succ(E, c) for c in by[E] if c.endswith("_" + a)}
k3, k0 = arm("k3"), arm("k0")
check("LIBERO-PRO K=3 sealed successes (of 3000)", sum(k3.values()), 2889)
check("LIBERO-PRO K=0 sealed successes (of 3000)", sum(k0.values()), 2846)
w = sum(k3[c] > k0[c] for c in k3); l = sum(k3[c] < k0[c] for c in k3)
check("paired per cell: K=3 wins / losses / ties", (w, l, len(k3) - w - l), (16, 9, 35))
check("cells differing by >= 15 episodes", sum(abs(k3[c] - k0[c]) >= 15 for c in k3), 2)
for ax, want in (("pos", (1454, 1437, 6, 7, 17)), ("task", (1435, 1409, 10, 2, 18))):
    cs = [c for c in k3 if c.endswith("_" + ax)]
    check(f"{ax.upper()}: K=3 / K=0 successes, K=3 wins / losses / ties",
          (sum(k3[c] for c in cs), sum(k0[c] for c in cs), sum(k3[c] > k0[c] for c in cs),
           sum(k3[c] < k0[c] for c in cs), sum(k3[c] == k0[c] for c in cs)), want)
check("put_bowl_top_cabinet-Task (plate): K=3 / K=0",
      (k3["goal_put_bowl_top_cabinet_task"], k0["goal_put_bowl_top_cabinet_task"]), (0, 50))
check("open_middle_drawer-Task: K=3 / K=0",
      (k3["goal_open_middle_drawer_task"], k0["goal_open_middle_drawer_task"]), (50, 0))
grid = [(s, ax) for s in ("obj", "goal", "spa") for ax in ("pos", "task")]


def macro(d):
    out = []
    for s, ax in grid:
        cs = [c for c in d if c.startswith(s + "_") and c.endswith("_" + ax)]
        out.append(round(sum(d[c] for c in cs) / (50 * len(cs)), 2))
    return out


check("macro K=3 (obj,goal,spa x pos,task)", macro(k3), [1.00, 1.00, 0.93, 0.90, 0.98, 0.97])
check("macro K=0 (obj,goal,spa x pos,task)", macro(k0), [1.00, 1.00, 0.92, 0.87, 0.96, 0.95])
st = sum(succ("libero_stock", c) for c in by["libero_stock"])
check("unperturbed (stock) column, % of 1500", round(100 * st / 1500, 1), 98.6)

# ------------------------------------------------------------------ development efficiency
m = json.load(open(f"{CAMP}/c2clean/metrics_per_cell.json"))
a3, a0 = m["clean k3"], m["clean k0"]
# as in c2clean/metrics.py: medians skip cells where the metric is undefined
# (no development success), paired counts use cells defined in both arms
med = lambda d, k: S.median(v[k] for v in d.values() if v[k] is not None)
both = [c for c in a3 if a3[c]["ver_first"] is not None and a0[c]["ver_first"] is not None]
check("first version succeeds on a dev state, cells (K=3, K=0)",
      (sum(bool(v["v1_any"]) for v in a3.values()), sum(bool(v["v1_any"]) for v in a0.values())), (29, 1))
# Figure 2A at k=1: mean over all sixty cells of the first version's development success (untested counts 0)
check("mean development success of the first version, % (K=3, K=0)",
      tuple(round(100 * sum(v["v1_ok"] or 0 for v in d.values()) / 60) for d in (a3, a0)), (41, 1))
check("median version of first success", (med(a3, "ver_first"), med(a0, "ver_first")), (2, 4))
for ax, want in (("pos", ((11, 0), (29, 0), (2, 4), (21, 5))), ("task", ((18, 1), (53, 2), (1, 5), (24, 2)))):
    b3 = {c: v for c, v in a3.items() if c.endswith("_" + ax)}; b0 = {c: v for c, v in a0.items() if c.endswith("_" + ax)}
    bb = [c for c in b3 if b3[c]["ver_first"] is not None and b0[c]["ver_first"] is not None]
    check(f"{ax.upper()}: first-version cells, first-version mean %, median first-success version, earlier/later",
          ((sum(bool(v["v1_any"]) for v in b3.values()), sum(bool(v["v1_any"]) for v in b0.values())),
           tuple(round(100 * sum(v["v1_ok"] or 0 for v in d.values()) / len(d)) for d in (b3, b0)),
           (med(b3, "ver_first"), med(b0, "ver_first")),
           (sum(b3[c]["ver_first"] < b0[c]["ver_first"] for c in bb), sum(b3[c]["ver_first"] > b0[c]["ver_first"] for c in bb))), want)
check("put_bowl_top_cabinet-Task versions before freezing (K=3, K=0)",
      (a3["goal_put_bowl_top_cabinet_task"]["versions"], a0["goal_put_bowl_top_cabinet_task"]["versions"]), (11, 23))
check("median versions before freezing", (med(a3, "versions"), med(a0, "versions")), (4, 6))
check("median development episodes", (med(a3, "dev_eps"), med(a0, "dev_eps")), (46.5, 53))
check("K=3 first success earlier / later, cells",
      (sum(a3[c]["ver_first"] < a0[c]["ver_first"] for c in both),
       sum(a3[c]["ver_first"] > a0[c]["ver_first"] for c in both)), (45, 7))
check("median agent hours per cell", (round(med(a3, "hours"), 2), round(med(a0, "hours"), 2)), (0.51, 0.55))
check("median output tokens per cell, k", (round(med(a3, "out_tok") / 1e3), round(med(a0, "out_tok") / 1e3)), (66, 69))
check("total agent hours, 60 cells",
      (round(sum(v["hours"] for v in a3.values())), round(sum(v["hours"] for v in a0.values()))), (37, 44))

# ------------------------------------------------------------------ ASPIRE (their validator's manifests)
v4 = json.load(open(f"{ASP}/v4_60cells/heldout_50cells.json"))
passes = lambda f: {k: v["runs"][-1]["passes"] for k, v in json.load(open(f)).items()}
gsA = passes(f"{ASP}/goalswap_demos/armA_no_demos.json")
gsB = passes(f"{ASP}/goalswap_demos/armB_k3_demos.json")
tot = sum(v4.values()) + sum(gsA.values())
check("ASPIRE opus-5 rerun, 60 cells (of 3000)", tot, 2679)
check("ASPIRE opus-5 rerun, %", round(100 * tot / 3000, 1), 89.3)
su = defaultdict(int)
for k, v in v4.items():
    su[k.split("__")[0]] += v
su["libero_goal_swap"] = sum(gsA.values())
check("ASPIRE macro (obj,goal,spa x swap,task)",
      [round(su[f"libero_{s}_{a}"] / 500, 2) for s in ("object", "goal", "spatial") for a in ("swap", "task")],
      [1.00, 1.00, 0.86, 0.86, 0.84, 0.80])
check("ASPIRE goal-swap without / with three demos (of 500)", (sum(gsA.values()), sum(gsB.values())), (431, 437))
NAME = {"between_the_plate_and_the_ramekin": "between", "from_table_center": "table_center",
        "in_the_top_drawer_of_the_wooden_cabinet": "top_drawer_cabinet", "next_to_the_cookie_box": "cookie_box",
        "next_to_the_plate": "next_to_plate", "next_to_the_ramekin": "next_to_ramekin",
        "on_the_cookie_box": "on_cookie_box", "on_the_ramekin": "on_ramekin", "on_the_stove": "on_stove",
        "on_the_wooden_cabinet": "on_wooden_cabinet"}
A = P = T = K0 = n = 0
for line in open(f"{ASP}/planner_threshold_ablation/RESULTS.md"):
    if not re.match(r"\| (swap|task)/", line):
        continue
    f = [x.strip() for x in line.strip().strip("|").split("|")]
    if f[1] == "n/a":
        continue
    ax, rest = f[0].split("/", 1)
    key = next(v for k, v in sorted(NAME.items(), key=lambda kv: -len(kv[0])) if rest.startswith(k[:len(rest)]))
    A += int(f[1]); P += int(f[2]); T += int(f[3]); n += 1
    K0 += k0[f"spa_bowl_{key}_{'pos' if ax == 'swap' else 'task'}"]
# no longer claimed in the paper (removed 2026-09-26); reported for the record only
print(f"[info] ASPIRE planner/threshold ablation on {n} shared spatial cells: control {A}, "
      f"no planner {P}, no thresholds {T}; Encore K=0 {K0} (of {50 * n})")

# ------------------------------------------------------------------ verification ablation (earlier brief, shared note file)
V = by["verification_ablation"]
check("verification loop removed, nine goal tasks (of 450)", sum(int(r["success"]) for r in V.values()), 117)
ref = {"turn_on_stove": "goal_turn_on_stove", "bowl_on_plate": "goal_put_bowl_on_plate",
       "open_middle_drawer": "goal_open_middle_drawer", "open_top_drawer_put_bowl": "goal_open_top_drawer_put_bowl",
       "push_plate_front_stove": "goal_push_plate_front_stove", "put_bowl_on_stove": "goal_put_bowl_on_stove",
       "put_bowl_top_cabinet": "goal_put_bowl_top_cabinet", "put_cream_cheese_in_bowl": "goal_put_cream_cheese_in_bowl",
       "put_wine_on_rack": "goal_put_wine_on_rack"}
check("full system on the same nine tasks (of 450)",
      sum(succ("libero_stock", ref[c[len("ablC_goal_"):]]) for c in V), 440)

# ------------------------------------------------------------------ LIBERO-90 fixtures (earlier brief)
L = {c: succ("libero90_fixtures", c) for c in by["libero90_fixtures"]}
check("open_bottom_drawer: full / images / none",
      (L["open_bottom_drawer_k3"], L["open_bottom_drawer_vis"], L["open_bottom_drawer_k0"]), (50, 0, 0))
others = sorted({c.rsplit("_", 1)[0] for c in L} - {"open_bottom_drawer"})
check("other seven fixture tasks: arms below 50/50",
      {f"{t}_{a}": L[f"{t}_{a}"] for t in others for a in ("k3", "vis", "k0") if L[f"{t}_{a}"] != 50},
      {"close_bottom_drawer_k0": 42})

# ------------------------------------------------------------------ RoboDojo
R1 = {c: succ("robodojo_rd1", c) for c in by["robodojo_rd1"]}
pairs = [c[:-3] for c in R1 if c.endswith("_k3")]
check("RoboDojo ten tasks, K=3 / K=0 (of 450)",
      (sum(R1[t + "_k3"] for t in pairs), sum(R1[t + "_k0"] for t in pairs)), (95, 0))
check("build tower / make kong / bottles / classify, K=3",
      [R1[t + "_k3"] for t in ("build_tower", "make_kong", "put_bottles_into_dustbin", "classify_objects")],
      [36, 36, 15, 8])


def rd2(t):
    return [(succ("robodojo_rd2", f"{t}_{a}"), n_of("robodojo_rd2", f"{t}_{a}")) for a in ("k3", "vis", "k0")]


for t, want in (("press_by_number", [50, 50, 0]), ("stack_blocks", [43, 0, 10]), ("swap_T", [47, 15, 38]),
                ("push_T", [18, 8, 0]), ("insert_tubes", [14, 0, 2]), ("play_tic_tac_toe", [5, 21, 4])):
    got = rd2(t)
    check(f"rd2 {t}: full / images / none, n=50", [s for s, n in got] if all(n == 50 for s, n in got) else got, want)
zero = ("plug_in_charger", "insert_key", "fasten_screws", "make_toast", "store_laptop_and_headphones", "hang_mugs")
check("six contact-precise rd2 tasks, successes over all arms",
      sum(succ("robodojo_rd2", f"{t}_{a}") for t in zero for a in ("k3", "vis", "k0")), 0)
print(f"[info] rd2 tic-tac-toe (success, n) full / images / none: {rd2('play_tic_tac_toe')}")

# ------------------------------------------------------------------ K=1 arm (clean brief, one demonstration)
K1 = {c[:-3]: succ("libero_pro_k1", c) for c in by["libero_pro_k1"]}
check("LIBERO-PRO K=1 cells evaluated", len(K1), 60)
check("LIBERO-PRO K=1 sealed successes (of 3000)", sum(K1.values()), 2791)
check("LIBERO-PRO K=1, %", round(100 * sum(K1.values()) / 3000, 1), 93.0)
check("macro K=1 (obj,goal,spa x pos,task)", macro(K1), [0.99, 1.00, 0.89, 0.85, 0.90, 0.94])
RK = {c[:-3]: succ("robodojo_k1", c) for c in by["robodojo_k1"]}
rd1_tasks = [c[:-3] for c in R1 if c.endswith("_k3")]
check("RoboDojo ten tasks, K=1 (of 450)", sum(RK[t] for t in rd1_tasks), 40)
check("build tower / make kong / bottles / classify / arrange, K=1",
      [RK[t] for t in ("build_tower", "make_kong", "put_bottles_into_dustbin", "classify_objects", "arrange_largest_number")],
      [0, 0, 2, 7, 31])
check("rd2 press / stack / swap / push / tubes / tic-tac-toe, K=1",
      [RK[t] for t in ("press_by_number", "stack_blocks", "swap_T", "push_T", "insert_tubes", "play_tic_tac_toe")],
      [50, 22, 44, 0, 34, 0])
check("six contact-precise rd2 tasks, K=1: evaluated / successes", (sum(t in RK for t in zero), sum(RK[t] for t in zero if t in RK)), (6, 0))
if pending:
    print(f"[info] not yet evaluated (paper marks them pending): {pending}")

# ------------------------------------------------------------------ the numbers appear in the paper source
PAPER = os.environ.get("PAPER_TEX", os.path.join(ROOT, "..", "..", "..", "paper_ws_corl", "main.tex"))
if os.path.isfile(PAPER):
    tex = re.sub(r"\s+", " ", open(PAPER).read())
    must = ["2889 of 3000", "on 2846", "2791 (93.0\\%", "96.3\\%", "94.9\\%", "89.3\\%", "71.7\\%",
            "95 of 450", "reach 40 of 450", "in 11 of 30 cells", "in 18 of 30 cells", "29\\% of its development episodes", "53\\% of its development episodes", "wins 10, loses 2, and ties 18", "1454 and 1437 of 1500", "1435 of 1500 sealed episodes against 1409", "version 1 against 5", "earlier in 21 cells and later in 5",
            "440 to 117 of 450", "scores $42/50$", "$431/500$", "$437/500$",
            "build tower & $\\mathbf{36}$ & $0$ & $0$", "arrange by number & $0$ & $\\mathbf{31}$ & $0$",
            "stack three blocks & $\\mathbf{43}$ & $22$ & $0$ & $10$", "insert tubes & $14$ & $\\mathbf{34}$ & $0$ & $2$",
            "tic-tac-toe & $5$ & $0$ & $\\mathbf{21}$ & $4$", "10 tasks & $\\mathbf{95}$ & $40$ & $0$"]
    check("key numbers present verbatim in the paper source", [m for m in must if m not in tex], [])
    # the appendix listings are the sealed drawer programs minus their PROVENANCE dict
    def strip_prov(t):
        i = t.index("PROVENANCE = {"); depth = 0
        for j in range(t.index("{", i), len(t)):
            depth += t[j] == "{"; depth -= t[j] == "}"
            if depth == 0: break
        end = t.index("\n", j) + 1
        return t[:i] + t[end + (t[end:end + 1] == "\n"):]
    lst = os.path.join(os.path.dirname(PAPER), "listings")
    bad = [a for a in ("k0", "k3") if not os.path.isfile(f"{lst}/program_{a}.py") or
           open(f"{lst}/program_{a}.py").read() != strip_prov(open(os.path.join(
               SEALED, "libero_pro_clean", f"goal_open_middle_drawer_task_{a}", "program.py")).read())]
    check("appendix listings equal the sealed drawer programs (mismatches)", bad, [])
else:
    print(f"[info] paper source not found at {PAPER}; text cross-check skipped")

# ------------------------------------------------------------------ report
w = max(len(c[1]) for c in checks)
for s, claim, got, want in checks:
    print(f"{s} {claim:<{w}}  {got}" + ("" if s.startswith("ok") else f"   (paper: {want})"))
print(f"\n{len(checks) - fails}/{len(checks)} claims reproduced")
sys.exit(1 if fails else 0)
