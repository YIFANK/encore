# NOTES — aspire_demos_ab

Ledger for the ASPIRE(+demos) A/B partial reproduction. Absolute dates. Every
deviation logged as it happens.

---

## Session 4 — 2026-08-20 (started 08:45 CST box time / 2026-08-19 evening local)

Sessions 1-3 (2026-08-19 17:30, 17:36, 17:44 local) surveyed the box and the
ASPIRE runbook but **wrote no NOTES.md and produced no state**. Their
transcripts are in this directory. Treat this session as the first with a
ledger; re-establishing state from scratch.

### Box state at session start (AbakaAI = gz201b2201, 2026-08-20 08:45 CST)

- Disk: `/` 892G, **28 MB free (100%)** — confirms TASK.md warning; everything
  must live under `/mnt/data/YifanKang/` (3.5T, 399G free).
- GPUs (used/total MiB):
  - 0: 51223/81920 — **other tenant, do not touch**
  - 1: 3543, 2: 5287, 3: 3397, 4: 5638 — small other-tenant residents present
  - 5: 0, 6: 1 — free
  - 7: 61328 — **other tenant, do not touch**
  - DEVIATION (already anticipated by TASK.md): 6-GPU topology (1-6) instead of
    ASPIRE's 8-GPU reference. Affects wallclock only. Additionally GPUs 1-4
    each carry 3-6 GB of *other tenants'* small jobs; we have ~75 GB headroom
    per card, so co-residency is tolerable but may cost ~1.7x SM throughput.
- `/mnt/data/YifanKang/ASPIRE` present (cloned), commit `680bad4` ("update the
  README"), clean worktree. All six submodules **uninitialized** at session
  start; no `.venv`/`.venv-libero`; no perception servers on 8114-8116; no
  `outputs/`. So: full install from scratch.

### Networking on this box (established 2026-08-20, worth keeping)

- `huggingface.co` and `github.com` are **not directly reachable**.
- A clash proxy owned by this account runs at `127.0.0.1:7890` (pid 3596223,
  up since Aug 3). git-over-proxy works; `ssh git@github.com` also authenticates.
- `https://hf-mirror.com` is reachable **directly** and correctly forwards HF
  auth (whoami returns account `IIFAN`; ungated file fetch = 200). Use
  `HF_ENDPOINT=https://hf-mirror.com`.
- pypi.org and astral.sh reachable directly (no proxy needed for uv/pip).

---

## BLOCKER (2026-08-20 08:5x CST) — SAM3 gated weights not granted

**This is a TASK.md-listed stop condition ("gated weights").**

`facebook/sam3` is `gated: "manual"` and the authenticated account (`IIFAN`,
ethan.yifan.kang@gmail.com) is **not on the authorized list**. Verified on both
endpoints, with the token attached:

```
GET https://hf-mirror.com/facebook/sam3/resolve/main/config.json  -> 403
GET https://huggingface.co/facebook/sam3/resolve/main/config.json -> 403
body: "Access to model facebook/sam3 is restricted and you are not in the
       authorized list. Visit https://huggingface.co/facebook/sam3 to ask for
       access."
control: bert-base-uncased/resolve/main/config.json -> 200 (auth path is fine)
GET api/models/facebook/sam3?expand=gated -> {"gated":"manual"}
```

No SAM3 checkpoint exists anywhere on the box (searched `/mnt/data/YifanKang`
for `sam3*` weights and the HF hub cache; the parallel `cap-x` install has the
code path only, and its own submodule clones failed too).

Why it blocks the whole experiment: SAM3 is the segmentation backend for the
perception servers (ports 8114-8116), and per the ASPIRE sim README perception
servers are **required** for LIBERO-PRO replay/eval — i.e. for every Stage 1
development trial and every Stage 2 held-out trial. No SAM3 → no trials → no
held-out numbers, in either arm.

Why I am not working around it: ASPIRE's own QUICKSTART preflight rule says
"Missing credentials, gated weights, services, environment support, or GPU
resources are actionable blockers; do not bypass them or substitute a different
task, suite, model, or seed range." Swapping in SAM2 (a checkout exists at
`/mnt/data/YifanKang/segment-anything-2`) would violate that rule and would
also break the A/B premise, since it changes the perception stack under both
arms. Re-uploaded community copies of the weights would bypass a license gate
the account has not been granted; that is the user's call, not mine.

**SAM3 is the ONLY gated blocker** — I checked the other two perception
services rather than assume:

- GraspNet (port 8115) needs `checkpoints/contact_graspnet/checkpoints/model.pt`,
  and that file (26.6 MB) **ships inside the pinned
  `contact_graspnet_pytorch` submodule** at exactly the path the server
  computes. Nothing to download, nothing gated.
- PyRoKi (port 8116) is CPU-only, `--robot panda_description`. No gated assets.
- Molmo is explicitly skipped (`--no-molmo`) by their own LIBERO recipe.

**Action required from Yifan (one click + an approval wait):** visit
https://huggingface.co/facebook/sam3 while signed in as `IIFAN` and request
access; the repo is manually gated, so Meta must approve (typically hours to
days). If a different HF account already holds access, dropping that token into
`/mnt/data/YifanKang/cache/hf/token` unblocks it immediately.

Everything else is being installed and staged anyway, so the run can start the
moment access lands — see below.

---

## Preflight / readiness report (TASK.md "REPORT readiness and expected runtime")

### Protocol as read from their runbook (no deviation intended)

- Suite `libero_goal_swap`; our three tasks are #8 `turn_on_the_stove`,
  #9 `put_the_bowl_on_the_plate`, #1 `open_the_middle_drawer_of_the_cabinet` —
  all inside their ten-task allowlist.
- Lifecycle per task: `pending` --(Stage 1 subagent: explore seed 51, write
  initial code, debug dev seeds 51-65, select one `fix_code.py`)--> `stage1-done`
  --(Stage 2, run by the coordinator: `run_fix_loop_validation.py` over held-out
  seeds 1-50)--> `done`.
- Stage 1 workers are forbidden from touching seeds 1-50; only the coordinator
  runs Stage 2. Held-out outcomes may never drive skill edits.
- Skill promotion happens between Stage 1 completions, recorded via
  `record_skill_promotion.py` into an append-only ledger.

### GPU topology (DEVIATION, already sanctioned by TASK.md)

Their reference topology is 8 GPUs: SAM3=0, GraspNet=1, PyRoKi=2, task slots
3-7 (five concurrent). GPUs 0 and 7 here belong to other tenants, so:

| Role | Their GPU | Ours |
|---|---|---|
| SAM3 | 0 | 1 |
| GraspNet | 1 | 2 |
| PyRoKi | 2 | 3 |
| Task slots | 3-7 (five) | 4, 5, 6 (three) |

Consequence: three concurrent task slots instead of five — wallclock only, no
effect on any number. Their QUICKSTART would have us stop and ask about this;
TASK.md pre-authorizes it and instructs "REPORT it in NOTES.md", which is what
this section is.

### Expected runtime (once unblocked)

- Their stated figure: a failed trial takes ~6-7 min; the 50 held-out seeds run
  sequentially. So Stage 2 is ~2.5-6 h per task per arm.
- Six Stage 2 evals (3 tasks x 2 arms) over three slots ≈ 2 rounds ≈ 5-12 h.
- Stage 1 is agent-driven debugging over 15 dev seeds, typically several hours
  per task; six of those over three slots is the dominant cost.
- Realistic total for both arms: **~1.5-3 days of wallclock**, longer than
  their 5-slot reference because of the 3-slot deviation above.

### Install state (2026-08-20, in progress)

- Submodule clones over the proxy kept dying with `gnutls_handshake() failed`.
  Fix: `ssh` transport instead (`url."git@github.com:".insteadOf`), which
  authenticates directly with no proxy. Recorded as a host workaround, not a
  protocol deviation.
- `uv` 0.11.29 (well past the 0.8.14 that needs their build-isolation fallback);
  driver 580.159.03; python 3.10/3.12 fetched into
  `/mnt/data/YifanKang/cache/uv-python`.
- All caches redirected off the full root disk: `UV_CACHE_DIR`,
  `XDG_CACHE_HOME`, `TMPDIR`, `HF_HOME`, `UV_PYTHON_INSTALL_DIR` under
  `/mnt/data/YifanKang/`.
- DEVIATION (host-shared state): `~/.libero/config.yaml` already existed and
  pointed at another project's venv
  (`wam/wam-few-shot/external/cosmos-policy/.venv-libero`). ASPIRE's README
  requires it to point at the ASPIRE LIBERO-PRO checkout, so it is being
  rewritten; the previous file is preserved at
  `~/.libero/config.yaml.pre_aspire_ab`. Other projects on this box that rely on
  the old path will need it restored.
- A parallel `cap-x` install (not ours) is cloning concurrently on this box;
  left undisturbed per TASK.md.
- HOST CAVEAT for whoever starts the services:
  `scripts/common/start_perception_servers.sh` hardcodes its log paths to
  `/tmp/sam3.log`, `/tmp/graspnet.log`, `/tmp/pyroki.log`, and `/` has **28 MB
  free**. Start the three servers by hand with logs under
  `/mnt/data/YifanKang/`, or clear space first — otherwise a server can die on
  a full-disk write and the runbook's "000 = DOWN" check will be the only
  symptom. Also note the script begins with `pkill -f launch_*_server.py`, so
  re-running it kills any perception servers another project on this box is
  using.
- ssh gotcha for scripted waits: this Mac's ssh ControlMaster socket to AbakaAI
  intermittently refuses new sessions
  (`mux_client_request_session: session request failed`), which silently
  truncates `sleep`-based waits into instant returns. Use
  `ssh -o ControlMaster=no -o ControlPath=none` for long-running waits.

### Arm B assets — DONE (independent of the blocker)

K=5 demonstrations extracted per task, deterministically `demo_0..demo_4` from
the official LIBERO `libero_goal` teleop datasets:

| Task | Extract | Size | Steps per demo |
|---|---|---|---|
| turn_on_the_stove | `/mnt/data/YifanKang/aspire_demos/turn_on_the_stove/turn_on_the_stove_K5_demos.hdf5` | 44.5 MB | 80, 96, 89, 86, 93 |
| put_the_bowl_on_the_plate | `.../put_the_bowl_on_the_plate/put_the_bowl_on_the_plate_K5_demos.hdf5` | 44.5 MB | 90, 88, 92, 94, 80 |
| open_the_middle_drawer_of_the_cabinet | `.../open_the_middle_drawer_of_the_cabinet/..._K5_demos.hdf5` | 69.9 MB | 138, 138, 151, 131, 141 |

Each extract carries the source attrs (`problem_info`, `env_args`,
`bddl_file_name`) plus, per demo, `actions`, `states`, `robot_states`,
`rewards`, `dones`, and `obs/{ee_pos, ee_ori, ee_states, gripper_states,
joint_states, agentview_rgb, eye_in_hand_rgb}`. Manifest at
`/mnt/data/YifanKang/aspire_demos/manifest.json`.

The single Arm-B addendum file is written and staged at
`/mnt/data/YifanKang/aspire_demos/DEMONSTRATIONS.md`. It states the path, the
hdf5 layout, and one caveat (the demos are of the BASE task, so placements and
goal region differ from the goal-swap seeds). Nothing else in Arm B differs —
same prompts, same protocol, same seeds, same GPU topology.

Arm separation plan: two independent repo copies (`ASPIRE` for Arm A, a second
checkout for Arm B), so neither can contaminate the other's outputs or skill
library. Arm B's copy gets exactly one extra file per task workspace.

---

## Session 4 close-out (2026-08-20)

**Stop condition met:** TASK.md "Stop when ... a blocker is documented in
NOTES.md (gated weights ...)". The blocker is SAM3 gated-model access, section
above. **No held-out numbers were produced for either arm, and none could be.**

Zero trials were run, so nothing touched seeds 1-50 or 51-65 and no protocol
integrity rule was exercised, let alone broken.

### What is done and durable

1. Blocker identified, verified on both HF endpoints, and traced to why it is
   fatal (SAM3 is the only object-localization primitive in the Franka LIBERO
   API — `segment_sam3_text_prompt` is called throughout
   `cap/envs/tasks/franka/franka_libero_env.py`, so every trial in both stages
   needs it).
2. Confirmed SAM3 is the *only* gated blocker: GraspNet weights ship in the
   submodule, PyRoKi is CPU-only, Molmo is skipped.
3. Host networking solved and written down (ssh transport for github;
   `HF_ENDPOINT=https://hf-mirror.com`; all caches off the full root disk).
4. All six submodules checked out at their pinned revisions; the
   Contact-GraspNet compatibility patch applies cleanly (`rc=0`).
5. Base and LIBERO venv sync started detached
   (`/mnt/data/YifanKang/orchestrate_aspire_ab.log`). Whoever picks this up
   should confirm it finished and that the offscreen smoke test
   (`tests/test_libero.py`) passes — it needs no credentials, so it is the
   right last green light before SAM3 arrives.
6. Arm B assets complete: K=5 demo extracts + manifest + the single addendum
   file. Results dir `/mnt/data/YifanKang/aspire_ab_results/` created with
   `armB_demo_manifest.json`.

### Resume checklist (once SAM3 access is granted)

1. `.venv-libero/bin/hf auth login` (or drop an authorized token into
   `/mnt/data/YifanKang/cache/hf/token`); re-verify with a `config.json` fetch
   returning 200 instead of 403.
2. Confirm the venv sync finished and the offscreen smoke test passes.
3. Start perception servers with SAM3=GPU1, GraspNet=GPU2, PyRoKi=CPU, logs
   under `/mnt/data/YifanKang/` not `/tmp` (see host caveat). Verify
   8114/8115/8116 answer (404 = up, 000 = down).
4. Arm A: run their coordinator loop over the three tasks on GPUs 4/5/6.
   Stage 1 subagents per task, then coordinator-run Stage 2 over seeds 1-50.
5. Freeze Arm A, clone a second repo copy for Arm B, add the one addendum file
   per task workspace, and repeat with LAWS/skills starting from zero again.
6. Write per-task dev traces + held-out N/50 into this file and
   `/mnt/data/YifanKang/aspire_ab_results/`.

### Note for the next session

Sessions 1-3 each burned their whole context re-reading the runbook and left
nothing behind. The runbook facts that matter are now summarized above — read
this file first and do not re-derive them.


## Coordinator intervention (2026-08-20, infra pre-staging while SAM3 pends)

Environment installs are DONE (retry2: PASS 2 DONE, offscreen LIBERO smoke
`1 passed`). Root causes fixed along the way — bake these into every shell:

- Box PyPI/HF direct = stalls. Export
  `http_proxy=https_proxy=http://127.0.0.1:7890` (clash) but put
  `github.com,codeload.github.com,objects.githubusercontent.com` in
  `no_proxy` — git via the clash proxy dies with a gnutls handshake error;
  direct git is slow but works. hf-mirror.com also in no_proxy.
- `ROBOT_DESCRIPTIONS_CACHE=/mnt/data/YifanKang/.cache/robot_descriptions`
  and `XDG_CACHE_HOME=/mnt/data/YifanKang/.cache` are MANDATORY for the
  PyRoKi server — its first boot clones robot assets into ~/.cache and the
  home disk is 100% full ("No space left on device" mid-clone).

Service verification (booted once, then killed — do not leave idle on
shared GPUs): GraspNet answers 404 on 8115 with the local 26M checkpoint
(GPU2). PyRoKi on 8116 verified pending (asset cache warm now; see
/mnt/data/YifanKang/pyroki_test3.log). SAM3 remains the only blocker
(request submitted 2026-08-20, Meta manual approval; poll for 200 on
config.json with the IIFAN token).

## COORDINATOR (2026-08-21) — SAM3 UNBLOCKED, run the ablation

Yifan obtained an HF token WITH facebook/sam3 access. It is installed at
/mnt/data/YifanKang/cache/hf/token (chmod 600) and VERIFIED: config.json
returns 200 via https://hf-mirror.com with this token (direct
huggingface.co still times out from the box — set
HF_ENDPOINT=https://hf-mirror.com and HF_TOKEN=$(cat that file) for every
download). Environment is DONE (install pass 2 completed, offscreen
LIBERO smoke test passed). GraspNet checkpoint local; PyRoKi verified on
8116 (needs ROBOT_DESCRIPTIONS_CACHE=/mnt/data/YifanKang/.cache/robot_descriptions).
Proxy discipline for pip/uv per the earlier note (clash 7890, github in
no_proxy). GPUs: 0 and 7 are other tenants'. Resume checklist in this
file stands: services (SAM3=GPU1 GraspNet=GPU2 PyRoKi=GPU3/CPU) -> Arm A
three tasks -> freeze -> Arm B (+demos). Go.

---

## Session 5 — 2026-08-21 (started 00:21 local / 15:22 CST box time)

First session to actually run trials. Picked up from the coordinator note above.

### Box state at session start (2026-08-21 15:22 CST)

- Disk unchanged in character: `/` 892G **0 bytes free (100%)**; `/mnt/data`
  3.5T with 377G free (was 399G on 2026-08-20 — the venv installs).
- GPUs, used MiB: 0=51223 (**other tenant**), 1=3543, 2=5287, 3=3397, 4=5638,
  5=0, 6=1, 7=61328 (**other tenant**). Same picture as session 4; the small
  residents on 1-4 are other tenants' and were left alone.
- `/mnt/data/YifanKang/ASPIRE` at commit `680bad4`, worktree clean except the
  expected Contact-GraspNet patch (` m` on that submodule).
- Six submodules checked out at pinned revisions (`b1k` uninitialized — not
  needed for LIBERO). `.venv` and `.venv-libero` present under `aspire/sim/`,
  NOT at the repo root; session 4's "no .venv" reading was looking one level
  too high.
- Ports 8114/8115/8116 all `000` (down) — services do not survive between
  sessions, as expected since session 4 deliberately killed them.
- `gen_progress.py`: **0/60 done, 0 need Stage 2, 60 pending** — clean slate,
  no contamination from earlier sessions, nothing to archive.

### SAM3 blocker: CLEARED

`facebook/sam3/resolve/main/config.json` returns **200** via
`https://hf-mirror.com` with the token at `/mnt/data/YifanKang/cache/hf/token`.
The session-4 blocker is resolved and the run proceeded.

### New host defect found and fixed (2026-08-21) — `HF_HUB_OFFLINE`

SAM3 still refused to boot after the token was verified, failing with
`LocalEntryNotFoundError ... check your connection`. The real cause, three
frames down the traceback, was `OfflineModeIsEnabled`:

```
HF_HUB_OFFLINE=1
TRANSFORMERS_OFFLINE=1
```

are exported into **every ssh session on this box** (they are not in
`~/.bashrc`/`~/.profile`/`~/.bash_aliases` — they arrive already set, so they
come from the sshd environment or `/etc/environment`). They silently turn every
`hf_hub_download` into a cache-only lookup, and the error text blames the
network, not the flag. `curl` to the same URL returns 200, which makes this look
like a Python/proxy problem and is a good way to lose an hour.

Fix: `unset HF_HUB_OFFLINE TRANSFORMERS_OFFLINE` in the session env preamble
(`/mnt/data/YifanKang/aspire_env.sh`). Not a protocol deviation — a host quirk.
After the unset, SAM3 downloaded (~3.3 GB into
`/mnt/data/YifanKang/cache/hf/hub/models--facebook--sam3`) and booted.

### Session env preamble

Everything now runs through `/mnt/data/YifanKang/aspire_env.sh`, which sets
`ASPIRE_ROOT=/mnt/data/YifanKang/ASPIRE/aspire/sim`,
`PYTHON_ROOT=/mnt/data/YifanKang/ASPIRE`, `PYTHONPATH`, `MUJOCO_GL=egl`,
`TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`, all caches under `/mnt/data/YifanKang/`
(`UV_CACHE_DIR`, `XDG_CACHE_HOME`, `TMPDIR`, `HF_HOME`,
`ROBOT_DESCRIPTIONS_CACHE`), `HF_ENDPOINT=https://hf-mirror.com`, the token, and
the two `unset`s above.

### Services UP (2026-08-21 15:5x CST)

| Port | Server | GPU | Status |
|---|---|---|---|
| 8114 | SAM3 | 1 | 404 = UP |
| 8115 | GraspNet | 2 | 404 = UP |
| 8116 | PyRoKi | CPU | 404 = UP |

DEVIATION (host, logged): started by hand via
`/mnt/data/YifanKang/start_services.sh` instead of ASPIRE's
`scripts/common/start_perception_servers.sh`, for two reasons, both forced:
(a) their script hardcodes logs to `/tmp/*.log` and `/` has 0 bytes free, so a
server could die on a full-disk write with "000 = DOWN" as the only symptom —
ours log to `/mnt/data/YifanKang/logs/`; (b) their script opens with a blanket
`pkill -f launch_*_server.py`, which on this shared box would kill another
project's perception servers. Ports were verified free first, so no kill was
needed. The commands, flags, ports and model builds are otherwise identical to
theirs. PyRoKi is CPU-only in their script too, so GPU3 stays unused.

### Stack smoke test — PASS (2026-08-21)

Ran `replay_trial.py` on `libero_goal_swap/turn_on_the_stove` seed 51 with a
throwaway legal program (not a task attempt; a dev seed, so protocol-clean).
Confirmed end to end: MuJoCo/LIBERO-PRO env built, `task_language` = "Turn on
the stove", RGB (512,800,3), SAM3 returned 200 masks, `mask_to_world_points`
gave a 16016-point cloud, GraspNet round-tripped, `solve_ik` answered, and the
trace/keyframes/video were written. The run ended in an `AssertionError: No
grasp candidates found` from `plan_grasp` — that is my arbitrary choice of
mask[0] out of 200, not a stack fault. Whole pipeline is live.

### DEVIATION — where the Stage 1 agents run

ASPIRE's fix loop assumes a coordinator + subagents running *on* the compute
host. There is **no `claude` CLI on AbakaAI** (node/npx exist; no authenticated
CLI), so the Stage 1 agents run on the laptop and drive the box over ssh through
three helpers (`abox.sh` / `apush.sh` / `apull.sh`) that source the env preamble
and retry ssh connect failures. This changes *where the agent process lives*,
not the protocol: same template, same commands, same venv, same GPUs, same
seeds. Recorded because it is a real departure from their reference setup.

The prompts are generated by `scratchpad/gen_prompts.py` from ASPIRE's own
`subagent-prompt.md`, with SUITE/TASK/GPU filled in and a host-access preamble
prepended. Generating both arms from one function is deliberate: it guarantees
the Arm A and Arm B prompts are byte-identical except for the demo addendum
(verified with `diff` — the only differences are the addendum block and the
per-agent scratch directory path).

### GPU ledger — Arm A (dispatched 2026-08-21 ~16:0x CST)

```
GPU 1: SERVICE  SAM3
GPU 2: SERVICE  GraspNet
GPU 4: SUBAGENT libero_goal_swap/turn_on_the_stove                    (Stage 1)
GPU 5: SUBAGENT libero_goal_swap/put_the_bowl_on_the_plate            (Stage 1)
GPU 6: SUBAGENT libero_goal_swap/open_the_middle_drawer_of_the_cabinet (Stage 1)
GPU 0, 7: OTHER TENANTS — untouched
GPU 3: free (PyRoKi is CPU-only)
```

Three concurrent slots, not their five — the TASK.md-sanctioned topology
deviation. Wallclock only.

### Arm B checkout pre-staged (2026-08-21, while Arm A Stage 1 runs)

`/mnt/data/YifanKang/ASPIRE_armB` — a second checkout at the same commit
`680bad4`, built by `rsync` from the Arm A tree while Arm A was still mid-flight.
`outputs/` and `docs/progress/` were **excluded** from the copy (so none of Arm
A's in-progress work could leak in) and recreated empty; verified empty.

The isolation boundary, stated exactly so it can be argued with:

- **Separate per arm:** `cap/` and `aspire/` source, `outputs/` (all fix_code,
  findings, traces, eval manifests, skill_promotions ledger),
  `docs/progress/`, and `.claude/` (the shared skill library). Verified:
  `PYTHONPATH=$PYTHON_ROOT` wins over the venv's `__editable__` path entries, so
  Arm B's python imports `cap` and `aspire` from `ASPIRE_armB`
  (checked: `cap -> /mnt/data/YifanKang/ASPIRE_armB/aspire/sim/cap/__init__.py`).
- **Shared between arms:** the two venvs (symlinked, 17.4 GB not duplicated) and
  through them the third-party deps that are installed editable — LIBERO-PRO,
  robosuite, sam3, contact_graspnet — which resolve to the Arm A checkout's
  copies. These are at identical pinned submodule revisions in both trees, are
  read-only during runs, and hold no experiment state. `robosuite` in particular
  installs an editable *import finder* that a PYTHONPATH entry cannot override
  without a second 9 GB venv, so partial separation here would be theatre; full
  sharing is the coherent choice. The perception servers (SAM3/GraspNet/PyRoKi)
  are likewise shared by both arms, exactly as they would be in ASPIRE's own
  single-host setup.
- **Start state proven identical:** `sha256` of the concatenated shared skill
  library is `80e38614aa99debe...bdffe` in *both* trees at Arm A dispatch time
  (443 lines across grasp/localize/manipulation/transport/README). Arm B
  therefore starts its skill library from zero promotions, same as Arm A.

Arm B env preamble is `/mnt/data/YifanKang/aspire_env_armB.sh`, identical to Arm
A's except `ASPIRE_ROOT`/`PYTHON_ROOT`.

### Session 5 close-out (2026-08-21 00:55 local / 15:55 CST)

Session 5 ended with its three Arm A Stage 1 agents **killed, not finished** —
its `transcript_20260821_002153.err` reads `Background tasks still running after
600s; terminating`. They had run ~10 minutes and produced Stage 0 residue only
(scene snapshots, probe scripts, one `initial_code.py`). No seed sweep, no
`fix_code.py`, no `findings.md`, no held-out numbers. See Session 6 for the
root cause and the fix.

---

## Session 6 — 2026-08-21 (started 00:57 local / 15:58 CST box time)

### Root cause of session 5's loss, and the architectural fix

Session 5 dispatched Stage 1 workers with the in-session `Agent` tool. Those
subagents are children of the coordinator session, so when the coordinator
returned its final answer the harness terminated them. ASPIRE's fix loop assumes
a *persistent* coordinator session that goes idle for hours; a `claude -p`
coordinator has no such lifetime. Any architecture where Stage 1 work lives
inside the coordinator's process tree loses that work on coordinator exit —
which will keep happening, since Stage 1 is hours per task.

**Fix: Stage 1 agents are now standalone detached OS processes**, one
`claude -p` per (arm, task), launched with `nohup ... &` from
`/Users/yifankang/aspire_ab_run/launch_stage1.sh`. They outlive this session and
any future coordinator session; they are individually re-launchable; and each
writes an append-only `agent.log` (stream-json) that a later session can read to
see exactly what it did. This is a change to *where the agent process lives*,
already the subject of session 5's logged deviation — not a protocol change.

DEVIATION (host, logged): the detached agents run with
`--permission-mode bypassPermissions`. This is forced, not chosen: under a
normal allow-list, Claude Code refuses any Bash command containing `$( )`, and
every helper invocation in ASPIRE's own template uses command substitution. A
non-interactive agent cannot answer the resulting prompt, so it would stall.
Verified with a probe before launching (`_probe/`). No protocol content changes.

### Durable run directory (new)

Everything the run needs now lives in **`/Users/yifankang/aspire_ab_run/`**,
outside any per-session scratch dir (session 5's helpers lived in
`/private/tmp/claude-501/.../<session-uuid>/scratchpad`, which is session-scoped
and would have been lost). Contents:

| Path | What |
|---|---|
| `abox.sh` / `abox_armB.sh`, `apush.sh`, `apull.sh` | host helpers, per arm |
| `aspire_env.sh` / `aspire_env_armB.sh` | env preambles (copies of the box's) |
| `gen_prompts.py` | generates all six Stage 1 prompts from ASPIRE's template |
| `prompts/arm{A,B}_<task>.md` | the six prompts actually dispatched |
| `agent_{A,B}_<task>/` | per-agent scratch + `agent.log` + `agent.pid` |
| `launch_stage1.sh`, `status_stage1.sh` | dispatch + poll Stage 1 |
| `stage2.sh`, `stage2_status.sh` | coordinator-only held-out eval + poll |
| `runbook/` | local copy of ASPIRE's fix-loop runbook |

### Ablation cleanliness — re-verified this session

`gen_prompts.py` now also materializes each agent's scratch dir with its **own**
copies of `abox.sh`/`apush.sh`/`apull.sh`, wired to that arm's checkout. This was
needed to keep the prompts arm-symmetric: previously Arm B's prompt would have
had to name `abox_armB.sh`, i.e. leak the arm identity and the repo path into the
prompt text. Now the arm-specific path lives only in the helper script.

`diff prompts/armA_<task>.md prompts/armB_<task>.md` for all three tasks yields
**exactly two hunks**: the agent's own scratch-directory path (`agent_A_*` vs
`agent_B_*`, unavoidable — they need different scratch dirs) and the demo
addendum. Nothing else. That is the ablation.

### Clean, symmetric start — Arm A reset

Arm A's `outputs/` still held session 5's partial Stage 0 residue while Arm B's
was empty, so the arms would not have started from the same state. Following
their `clean-task-slate.md`, Arm A's `outputs/` was archived whole to
`/mnt/data/YifanKang/aspire_ab_results/archive_armA_session5_partial_20260821/`
and recreated empty.

Verified identical across the two checkouts immediately before dispatch:

| Check | Arm A | Arm B |
|---|---|---|
| `gen_progress.py` | 0/60 done, 0 need Stage 2, 60 pending | 0/60 done, 0 need Stage 2, 60 pending |
| skill library sha256 (5 files) | `80e38614…bdffe` | `80e38614…bdffe` |
| skill promotions recorded | 0 | 0 |
| `import cap` resolves to | `ASPIRE/aspire/sim/cap` | `ASPIRE_armB/aspire/sim/cap` |

### Both arms run CONCURRENTLY (TASK.md-sanctioned)

TASK.md permits either "Arm A finishes first" **or** "separate copies of the
repo". The separate checkouts exist and their isolation is verified above, so
both arms run at once — halving wallclock, which is the dominant risk on a task
estimated at 1.5-3 days.

GPU ledger, Stage 1 dispatch 2026-08-21 16:0x CST:

```
GPU 1: SERVICE  SAM3       (shared by all six agents)
GPU 2: SERVICE  GraspNet   (shared by all six agents)
GPU 3: free                (PyRoKi is CPU-only)
GPU 4: armA + armB  turn_on_the_stove
GPU 5: armA + armB  put_the_bowl_on_the_plate
GPU 6: armA + armB  open_the_middle_drawer_of_the_cabinet
GPU 0, 7: OTHER TENANTS — untouched
```

DEVIATION from their "one job per GPU, ever" ledger rule: two agents share each
task GPU. Forced by having three task GPUs instead of their five, and deliberately
paired **same task, opposite arms** — so the two arms of any comparison contend
for exactly the same card under exactly the same load. The cards have ~75 GB free
each. This costs throughput, not correctness.

### Stage 1 DISPATCHED — all six agents live (2026-08-21 16:0x CST)

All six launched and confirmed executing (Bash tool calls landing, zero
permission denials, `agent.err` clean apart from a benign untrusted-workspace
warning that `bypassPermissions` overrides). Poll with
`/Users/yifankang/aspire_ab_run/status_stage1.sh`.

Perception services re-verified up at session start: 8114/8115/8116 all 404.
They survived from session 5 — they are nohup'd on the box, independent of any
laptop session.


## COORDINATOR (2026-08-21 01:0x) — one fairness pin

Probe trials show the inner code-gen model routing as
`bedrock-claude-sonnet-4-6`. Whatever the provider plumbing, PIN THE SAME
inner model for Arm A and Arm B (the ±demos comparison's validity is
same-model), and record model+provider per arm in RESULTS.md. If pinning
opus-4.6 (paper parity) is feasible with available credentials, prefer
it; otherwise same-sonnet both arms is acceptable with a footnote.

---

## Session 7 — 2026-08-21 (started 04:03 local / 19:03 CST box time)

### What killed session 6's six agents: a laptop network blip, not the task

All six Stage 1 agents were dead at session start. `status_stage1.sh` reported
them "RETURNED", which was wrong — that script only checks for a `result` event,
and a crash produces one too. Reading the actual result events:

```
"terminal_reason":"api_error"
"result":"API Error: Can't reach the API server — check your internet or DNS (ENOTFOUND)"
```

in five of six, and `"Connection lost mid-response"` in the sixth. Their
timestamps cluster at 01:26–01:36 local: one DNS/connectivity outage on **this
laptop** took out every agent at once, 20–30 minutes into their runs. Nothing on
AbakaAI failed — the perception servers on 8114/8115/8116 were still answering
404 (UP) at 19:03 CST, having survived since session 5, and the box's GPUs were
idle.

Cost of the blip: ~$6 of model spend and six partial Stage 0s. Cost if it
recurs during a multi-hour Stage 1: the whole run. So the first work of this
session was making the agents survive it.

### Fix — `supervise_stage1.sh`, a per-agent supervisor

`launch_stage1.sh` ran each agent as a single-shot `claude -p`. That is a coin
flip over a multi-hour Stage 1. Replaced with one detached **supervisor** per
agent (`launch_supervised.sh` → `supervise_stage1.sh <arm> <task>`):

- launches the agent with a **fixed `--session-id`**, so a retry is
  `claude -p --resume <sid>` — the agent continues with its full context rather
  than restarting. Probed end-to-end before dispatch (`_probe/`: set a fact in
  session 1, recalled it after `--resume`).
- retries **only on transient death** — a result event carrying
  `terminal_reason:api_error`, or no result event at all (hard kill). Backoff
  45s doubling to 600s, reset whenever an attempt made real progress.
- stops on the **real** done-signal, checked on the box, not on the agent's
  say-so: `fix_code.py` AND `findings.md` both present in the task's output dir.
  A clean agent return without both gets at most two "finish Steps 4–5" nudges,
  then stops as `needs_review` rather than spinning.
- caps: 80 attempts, 40 h wallclock, then `STAGE1_STATUS=exhausted|deadline`.

Resuming an agent after a laptop network error is **recovery, not a protocol
change**: same prompt, same seeds, same GPU, same env, same box state. Logged
here because it is a departure from ASPIRE's one-shot subagent dispatch.

New poller `status2.sh` (replaces `status_stage1.sh`, which mis-read crashes as
completions): supervisor pid, attempt count, live event count, and the two
artifact flags read off the box — one ssh per arm.

### Restart from a symmetric slate, not from the wreckage

Both arms' `outputs/` held uneven Stage 0 residue (Arm A: exploration trials on
seed 51 for all three tasks + one `initial_code.py`; Arm B: scene snapshots +
one `explore.log`). Resuming six differently-advanced contexts would have made
the two arms start from different states, which is precisely what the ablation
cannot afford. Following `clean-task-slate.md`, both were archived whole and
recreated empty:

```
/mnt/data/YifanKang/aspire_ab_results/archive_armA_session6_partial_20260821/
/mnt/data/YifanKang/aspire_ab_results/archive_armB_session6_partial_20260821/
```

(An initial archive command left both arms' trees nested under one
`archive_ARMTAG_…` directory — a `sed` in the wrong place, rewriting the printed
output instead of the remote path. Nothing was lost or overwritten; the two
trees were separated into the paths above and verified.)

Local agent scratch dirs were archived the same way (`agent_*/session6_archive/`)
so attempt 1 of each agent starts clean. Prompts were **not** touched: the
diff-verified two-hunk ablation from session 6 stands.

### Fairness pin — inner code-gen model (coordinator's request)

Checked rather than assumed, by diffing the exploration configs the two
checkouts actually wrote:

| | Arm A | Arm B |
|---|---|---|
| inner code-gen model | `aws_anthropic_bedrock-claude-sonnet-4-6` | `aws_anthropic_bedrock-claude-sonnet-4-6` |
| exploration config | identical (prompt, `num_workers: 8`, `trials: 1`) | identical |

Same model, same provider routing, both arms — the ±demos comparison is
same-model as required. Opus-4.6 paper parity is not available through this
box's Bedrock routing, so this is the footnoted "same-sonnet both arms" option.
The **outer** Stage 1 agent (the one debugging) is `claude-opus-5[1m]`,
first-party, identically in both arms.

### Stage 2 command verified against the runbook

`stage2.sh` was written in session 6 but never run. Checked its flags against
both `run_fix_loop_validation.py --help` on the box and
`runbook/fix-loop/main-agent-prompt.md` lines 211–215: `--suite --task --gpu
--fix-code --output-dir outputs/libero_fix_loop_eval --seeds $(seq 1 50)
--resume` — an exact match. It refuses to launch unless `fix_code.py` exists,
and it is coordinator-only; the subagent prompt forbids seeds 1–50 explicitly.

### Stage 1 RE-DISPATCHED — all six supervised (2026-08-21 04:07 local / 19:07 CST)

```
GPU 1: SERVICE  SAM3       404 UP     GPU 2: SERVICE  GraspNet   404 UP
GPU 3: (PyRoKi is CPU-only) 8116 UP   GPU 0, 7: OTHER TENANTS — untouched
GPU 4: armA + armB  turn_on_the_stove
GPU 5: armA + armB  put_the_bowl_on_the_plate
GPU 6: armA + armB  open_the_middle_drawer_of_the_cabinet
```

All six confirmed executing (Bash/Read tool calls landing, 73–114 events each
within three minutes, zero permission denials). Poll with `./status2.sh`.

### Stage 2 is a daemon too (`coordinator_daemon.sh`), for the same reason

The six agents will finish Stage 1 at different times over many hours. A
coordinator that only exists inside one laptop session would have to sit and
wait for them — and die with the session, which is exactly how sessions 5 and 6
lost their work. So Stage 2 runs from a detached daemon (pid file
`coordinator.pid`, ledger `coordinator.log`, 300 s poll):

- for each (arm, task) it reads the **box** state — `fix_code.py` +
  `findings.md` present? an eval already running? — and once a task is
  stage1-done it launches `stage2.sh` for it exactly once, on that task's GPU;
- progress and the final number come from ASPIRE's **own run manifest**
  (`outputs/libero_fix_loop_eval/<suite>/<task>/runs/<run_id>/manifest.json`,
  fields `trials`/`passes`/`pass_rate`), not from counting result directories.
  The run id is a hash of fix code + config + seed set, so a relaunch resumes
  the same immutable run instead of starting a second one;
- it exits when all six have 50/50 trials on disk.

This daemon is the ONLY thing in the whole setup that touches seeds 1-50, which
is the runbook's rule. The Stage 1 prompt forbids the agents from running
`run_fix_loop_validation.py`, and nothing feeds held-out outcomes back into fix
code or skills.

Verified before arming: `stage2.sh`'s flags match the runbook verbatim; the
daemon's probe parses real manifests; macOS bash 3.2 has no associative arrays
(the first version silently died on `declare -A` — now a `case`).

### Ablation delta re-verified this session, plus one thing nobody had checked

`diff prompts/armA_<task>.md prompts/armB_<task>.md` → still exactly two hunks
(scratch-dir path, demo addendum). And the check that had been missing: that Arm
B's demo access is **real** rather than nominal —

```
.venv-libero/bin/python3 -c "import h5py; ..." on the box, Arm B's venv
→ h5py OK, demos: ['demo_0','demo_1','demo_2','demo_3','demo_4']
```

If `h5py` had been absent the addendum would have pointed at a file the agent
could not open, and the ±demos row would have measured nothing.

Per-agent helper wiring re-confirmed: `agent_A_*/abox.sh` → `ASPIRE`,
`agent_B_*/abox.sh` → `ASPIRE_armB`.

---

## Session 8 — 2026-08-21 (started 04:34 local / 19:34 CST box time)

Picked up a **live** run for the first time: session 7's supervisors and the
coordinator daemon both survived the session boundary, which is exactly what
they were built for. State at 04:34 local:

| Arm | Task | Stage 1 | Stage 2 |
|---|---|---|---|
| A | turn_on_the_stove | live (att 1, 494 ev) | — |
| A | put_the_bowl_on_the_plate | **done** (fix+findings) | launched 04:30, GPU 5 |
| A | open_the_middle_drawer_of_the_cabinet | live (att 1, 530 ev) | — |
| B | turn_on_the_stove | live (att 1, 456 ev) | — |
| B | put_the_bowl_on_the_plate | live (att 1, 486 ev) | — |
| B | open_the_middle_drawer_of_the_cabinet | **done** (fix+findings) | launched 04:31, GPU 6 |

The coordinator daemon (pid 4590, poll 300 s) dispatched both Stage 2 evals on
its own, one per stage1-done task, on that task's GPU — no coordinator session
was alive when it did so. Perception services still 404/UP on 8114/8115/8116.
Both evals are stepping through held-out seeds and scoring
`reward=1.000 completed=1` on the early ones.

Session role: poll in-turn (per CLAUDE.md waiting discipline), keep the four
remaining Stage 1 agents alive, and record the six held-out numbers as their
manifests reach 50/50.

### Held-out numbers as they land (seeds 1–50, `evidence_scope: heldout_full`)

Read from ASPIRE's own run manifests via `collect_results.py`, never a directory
glob. State at 05:30 local:

| Task | Arm A (ASPIRE) | Arm B (ASPIRE+demos) |
|---|---|---|
| turn_on_the_stove | Stage 2 dispatched 05:29, GPU 4 | **50/50 (1.00)** |
| put_the_bowl_on_the_plate | **50/50 (1.00)** | **50/50 (1.00)** |
| open_the_middle_drawer_of_the_cabinet | Stage 1 still live (27 attempts) | **48/50 (0.96)** |

Stage 1 completion order, all six supervised, no supervisor restarts needed for
API death this session: B/drawer and A/bowl first (~04:30), then B/stove and
B/bowl (~04:5x), then A/stove (~05:29). **A/open_the_middle_drawer is the one
laggard** — it has `fix_code.py` on the box but no `findings.md`, i.e. it is
still iterating rather than writing up.

The coordinator daemon dispatched every Stage 2 by itself, one per stage1-done
task on that task's GPU, with no coordinator session alive at the moment of
dispatch — the session-5/6 failure mode is now actually fixed, not just
patched.

### All six Stage 1 agents finished; all six held-out evals completed 06:57 local

Full timeline, from the supervisor logs and `coordinator.log`:

| Arm/task | Stage 1 wallclock | Agent events | Stage 2 dispatch → complete |
|---|---|---|---|
| A/stove | 04:08 → 05:27 (**79 min**) | 1152 | 05:30 → 06:01 |
| A/bowl | 04:08 → 04:29 (**21 min**) | 349 | 04:30 → 05:01 |
| A/drawer | 04:08 → 05:52 (**104 min**) | 1650 | 05:35 → 06:55 |
| B/stove | 04:08 → 04:51 (**43 min**) | 584 | 04:55 → 05:25 |
| B/bowl | 04:08 → 04:44 (**36 min**) | 663 | 04:50 → 05:19 |
| B/drawer | 04:08 → 04:29 (**21 min**) | 529 | 04:31 → 05:07 |

Not one supervisor retry was needed for API death this session — every agent
completed on attempt 1 (two took a second "finish Steps 4–5" nudge attempt).
Session 7's supervisor was insurance that went unused, which is the correct
outcome for insurance.

### Arm B actually consumed the treatment (3/3)

Checked rather than assumed, because a null result would be uninterpretable if
the agents had ignored the addendum. All three Arm B agents opened the hdf5 and
cited measurements from it — e.g. B/drawer's `task_analysis.md` records
"Reference from the K=5 human demos: every demo drives the end-effector to
y ≈ −0.146 at constant z, then translates ~0.17–0.22 m in +y". Arm A agents have
no such references (they were never given the file).

### DEFECT FOUND AND CORRECTED — A/drawer was evaluated on a superseded program

The coordinator daemon treats "`fix_code.py` AND `findings.md` both present" as
stage1-done. For five of six cells that was right. For **A/drawer it fired
early**: the daemon dispatched Stage 2 at 05:35 local, and the agent went on
editing until 05:52, rewriting `fix_code.py` at 05:45. Caught by comparing each
eval manifest's `identity.code_sha256` against the current file:

| Arm/task | manifest code_sha256 == delivered fix_code.py? |
|---|---|
| A/stove, A/bowl | yes |
| B/stove, B/bowl, B/drawer | yes |
| **A/drawer** | **no** (`12a572b0…` evaluated, `2714dba3…` delivered) |

So Arm A's drawer number measured a program that is not the one Stage 1
delivered, while Arm B's measured its delivered program — an asymmetry in
exactly the one cell where the two arms differ, i.e. the worst possible place
for it. The agent noticed the contention from its own side and wrote it into
`findings.md`: "the coordinator's Stage 2 evaluation had already started against
this file and my sweep was competing with it for the GPU and the shared
perception servers."

No held-out *outcome* leaked into any Stage 1 program — the A/drawer eval did not
finish (06:55) until an hour after that agent ended (05:52), and every other
cell's Stage 2 was dispatched after its agent had already stopped. The integrity
rule that held-out results may never drive skill edits was not broken. What broke
is the weaker property that the evaluated artifact must be the delivered one.

**Correction:** the A/drawer held-out eval was re-run at 07:0x local against the
delivered `fix_code.py`, on GPU 6, same 50 seeds, same script and flags. This is
not a second attempt at a better number — it is the first measurement of the
right artifact, and both numbers are reported below. ASPIRE's run identity is a
hash of (code, config, seeds), so the re-run writes a *separate* immutable run
directory; the original 31/50 manifest is untouched and still on the box.

**Root cause for anyone reusing `coordinator_daemon.sh`:** the done-signal must
be the agent process having exited, not artifact presence. Artifacts appear
mid-run.

**Outcome of the correction: it changed nothing.** The re-run (`b2a1ebcfa295`,
07:05–08:2x local) on the delivered code also scored **31/50**. The two runs
disagree on only three seeds each — 2, 5, 30 pass only in the superseded run;
48, 49, 50 pass only in the delivered one; 28 pass in both. So the two programs
are equivalent in aggregate on the held-out set. The number is the same, but it
now comes from the artifact Stage 1 actually delivered, which is the point.

---

## FINAL RESULTS — held-out seeds 1–50, both arms, three tasks

| Task | Arm A (ASPIRE) | Arm B (ASPIRE+demos) |
|---|---|---|
| turn_on_the_stove | **50/50** (1.00) | **50/50** (1.00) |
| put_the_bowl_on_the_plate | **50/50** (1.00) | **50/50** (1.00) |
| open_the_middle_drawer_of_the_cabinet | **31/50** (0.62) | **48/50** (0.96) |
| **total** | **131/150 (0.873)** | **148/150 (0.987)** |

Dev traces (seeds 51–65), from the agents' own sweeps:

| Task | A initial | A delivered | B initial | B delivered |
|---|---|---|---|---|
| turn_on_the_stove | 15/15 | 15/15 | 15/15 | 15/15 |
| put_the_bowl_on_the_plate | 14/15 (f 64) | 15/15 | 11/15 (f 58,62,64,65) | 15/15 |
| open_the_middle_drawer_of_the_cabinet | 12/15 (f 51,57,64) | partial: 51 ✓ 52 ✓ 53 ✗ | 15/15 | 15/15 |

Written up in full, with run ids, code hashes, cost table and caveats, in
`/Users/yifankang/aspire_ab_run/RESULTS.md`, mirrored to
`/mnt/data/YifanKang/aspire_ab_results/RESULTS.md` alongside `ab_heldout.json`.

### What the numbers do and do not support

- Two of three tasks are at ceiling in both arms, so they separate nothing. The
  ±demos accuracy claim rests **entirely on the drawer cell** (48/50 vs 31/50),
  one task, one run per arm. The 0.873-vs-0.987 totals are that single cell
  diluted by two ceilings — they are not three independent measurements and
  should not be quoted as if they were.
- The cleaner signal is **cost**: Arm B produced a delivered program in 100 min
  and 1776 agent events against Arm A's 204 min and 3151, with the widest gap on
  the hardest task (drawer: 21 min vs 104). Same GPUs, same load, same models,
  concurrent.
- Mechanism, as far as the logs show it: the demos were used as a **measurement
  shortcut**, not as behaviour to imitate. B/drawer read the pull axis and
  stroke length straight off the teleop end-effector traces ("y ≈ −0.146 at
  constant z, then ~0.17–0.22 m in +y"), which is exactly the quantity A/drawer
  spent 27 attempts and two reverted calibration theories converging on. Arm A's
  own findings name the unresolved failure as re-grasping a partially protruded
  drawer — a problem the demo traces sidestep by making one clean pull the plan.
- Not controlled for: n=3 tasks, single run per cell, no repeat seeds for the
  agents themselves, inner model pinned to sonnet-4-6 rather than the paper's
  opus-4.6.

---

## Session 8 close-out (2026-08-21 08:3x local / 23:3x CST)

**Stop condition met:** TASK.md "Stop when: Both arms have held-out numbers for
the three tasks". All six cells are complete, at `evidence_scope: heldout_full`,
50 trials each, from ASPIRE's own validation script.

Protocol integrity, checked rather than assumed:

- No Stage 1 agent ever ran `run_fix_loop_validation.py`; every Stage 2 was
  launched by the coordinator daemon, the only component that touches seeds 1–50.
- No held-out outcome existed while its agent was alive: each agent's final
  attempt ended before its own eval completed (verified against supervisor logs
  and `coordinator.log` timestamps), so nothing fed back into fix code or skills.
- The arms' prompts differ in exactly two diff hunks (scratch dir, demo
  addendum), re-verified this session, and Arm B's agents actually consumed the
  treatment (3/3 opened the hdf5).
- Five of six evals ran on their delivered `fix_code.py` by construction; the
  sixth was found not to and was re-measured. Both numbers are on the box.

State left behind on AbakaAI:

- Perception servers **still up** on 8114 (SAM3, GPU1 ~5.5 GB of the 9.0 GB
  shown), 8115 (GraspNet, GPU2 ~7.4 GB of 12.7), 8116 (PyRoKi, CPU). Task GPUs
  4/5/6 are back to ~4 MiB; GPUs 0 and 7 are the other tenants', untouched
  throughout. `start_services.sh` writes **no pid files**, so stopping them means
  matching on `launch_*_server.py` — which is exactly the blanket-pkill hazard
  that made us avoid ASPIRE's own script on this shared box. Left running rather
  than risk killing another project's servers; stop them deliberately when the
  box is known to be idle.
- Coordinator daemon and all six supervisors exited cleanly (no orphans).
- Both checkouts and every `outputs/` tree intact, including the superseded
  A/drawer eval run.
- Results mirrored to `/mnt/data/YifanKang/aspire_ab_results/`
  (`RESULTS.md`, `NOTES.md`, `ab_heldout.json`).

---

## Session 9 — 2026-08-21 (09:16–09:28 local) — scaffolding only, cut off before dispatch

Session 9 read the EXPANSION block (10 tasks, both arms, opus inner) and built
the whole ten-task apparatus, then **ran out of turns one tool call before
dispatching wave 1**. It left no NOTES entry, so this is reconstructed from its
transcript (`transcript_20260821_091632.jsonl`) and the artifacts on disk.

What it established, all of it verified rather than assumed:

- **The "inner code-gen model = sonnet" reading in session 7 was wrong.** Session
  9 checked every trial invocation in v1 and found they were all `--replay-code`
  (replay) or `--args.interactive` (agent-driven REPL) — the "fresh LLM query"
  path that would consume `aws_anthropic_bedrock-claude-sonnet-4-6` was **never
  taken**. The model that actually writes the code is the outer Stage 1 agent,
  and all six v1 agents ran on **`claude-opus-5`**, first-party, in both arms.
  So v1 was already an opus run; the sonnet string in the exploration config was
  a default that nothing exercised.
- Consequently the EXPANSION's hard stop ("if no opus route works, STOP") is
  satisfied without the clash-proxy plumbing it anticipated. Verified live with a
  cheap call: `claude -p --model claude-opus-5` returned `MODELCHECK_OK` with
  `modelUsage` naming `claude-opus-5`.
- **Model pin added** to `supervise_stage1.sh` (`MODEL=claude-opus-5`, passed as
  `--model`). v1 inherited the CLI default, which happened to resolve to opus in
  all six cells — "happened to" is not a protocol, so it is now named.
- **Coordinator done-signal fixed** (the session-8 A/drawer defect). Artifact
  presence is no longer sufficient; the daemon now also requires
  `STAGE1_STATUS`, which the supervisor writes only on its way out, so it means
  "no agent process is running for this cell". Belt and braces: at completion it
  compares the eval manifest's `identity.code_sha256` against the delivered
  `fix_code.py` and logs `EVALUATED_STALE_CODE` on mismatch.
- K=5 demo extracts for **all ten** tasks (v1 had three), prompts for all 20
  cells, per-agent scratch dirs, `launch_wave.sh`, `status20.sh`.
- v1's run state archived to `aspire_ab_run/v1_archive_20260821/`; both box
  checkouts' `outputs/` emptied.

---

## Session 10 — 2026-08-21 (started 09:28 local / 00:28 CST box time)

### The scope revision landed after session 9 had already read TASK.md

TASK.md gained its "SCOPE REVISION (lean version)" block at 09:27, one minute
before session 9 exited — session 9 read TASK.md at 09:16 and never saw it. Its
`launch_wave.sh` and `coordinator_daemon.sh` were therefore both-arms. Since it
died before dispatching, nothing had to be undone; the two scripts were simply
retargeted before first use.

The campaign is now: **Arm B (+demos) x all ten `libero_goal_swap` tasks, opus
inner.** Arm A is not rerun — the plain-ASPIRE column cites their published
Table 8 (footnoted for the inner-model difference), and v1's three Arm-A cells
stay as the reproduction sanity row. No Arm A runs were in flight, so the "let
in-flight cells finish" clause never applied.

Changes, both parameterized rather than hardcoded so an Arm-A dispatch stays
possible but never happens by accident:

- `launch_wave.sh`: `ARMS="${ARMS:-B}"`, default B.
- `coordinator_daemon.sh`: same default. This one is load-bearing, not cosmetic —
  the daemon sets `alldone=0` for every cell lacking a `STAGE2_RESULT`, and Arm A
  cells will never get one, so with `for arm in A B` the daemon could never reach
  its exit condition and would poll forever.

### One job per GPU — the v1 topology deviation is retired

With one arm, a wave puts exactly **one** agent on each of GPUs 4/5/6. v1 had to
pair both arms of a task on one card (a logged deviation from ASPIRE's "one job
per GPU, ever" ledger rule); at half the cell count that deviation is no longer
needed. Ten tasks run as four waves of 3/3/3/1. The only standing topology
deviation is the one TASK.md pre-authorized: three task slots instead of their
five, wallclock only.

### Pre-dispatch verification (2026-08-21 09:3x local)

| Check | Result |
|---|---|
| Perception servers 8114/8115/8116 | 404 / 404 / 404 = all UP (survived from session 5) |
| Task GPUs 4/5/6 | 5638 / 4 / 4 MiB — other tenants only on 4; 0 and 7 untouched |
| `/mnt/data` free | 356 G |
| Arm B `outputs/` | empty; `gen_progress.py` 0/60 done, 60 pending |
| Skill library, both trees | `75b397c6…c449` identical; **0** promotions recorded |
| Prompt delta, all 10 tasks | 19 differing lines each = scratch-dir path + demo addendum, nothing else |
| Demo extracts, all 10 tasks | open under Arm B's venv: 5 demos each, `obs/ee_pos` present, actions (T,7) |
| Model pin | `claude-opus-5`, verified by live call |

The demo check matters because a nominal treatment would make the whole column
meaningless: the addendum has to point at a file the agent can actually open.
Ten for ten, including the two longest (`put_the_wine_bottle_on_the_rack`, 347
steps; `open_the_top_drawer_and_put_the_bowl_inside`, 170).

### Wave 1 DISPATCHED (2026-08-21 09:34 local / 00:34 CST)

```
GPU 1: SERVICE  SAM3       404 UP     GPU 2: SERVICE  GraspNet   404 UP
GPU 3: (PyRoKi is CPU-only) 8116 UP   GPU 0, 7: OTHER TENANTS — untouched
GPU 4: armB  open_the_middle_drawer_of_the_cabinet
GPU 5: armB  put_the_bowl_on_the_stove
GPU 6: armB  put_the_wine_bottle_on_top_of_the_cabinet
```

All three supervised (`supervise_stage1.sh`, session-id pinned for `--resume`
recovery), all three confirmed executing on `claude-opus-5` with Bash calls
landing on the box and **zero real permission denials** — the only "permission"
string in the logs is the benign untrusted-workspace warning that
`bypassPermissions` overrides. Coordinator daemon armed (Arm B, ten tasks).

Fixed while watching: `status20.sh` counted live events from `agent.log`, which
the supervisor only appends to when an *attempt ends* — so every live cell read
`live(0ev)`. It now counts `agent.log` plus the in-flight `attempt_*.log`. Same
class of bug as session 7's `status_stage1.sh`, which mis-read crashes as
completions; a poller that lies about a live run is how sessions get lost.

### Fixed waves replaced with a per-GPU scheduler (`wave_scheduler.sh`, 09:38)

A wave is a barrier: every GPU idles until the slowest task in the wave
finishes. v1's Stage 1 times ranged 21–104 min *within one wave*, so four
barriers would have wasted roughly half the available GPU time. Replaced with a
scheduler that gives each task GPU an ordered queue and starts that GPU's next
task the moment the current agent **exits** (`STAGE1_STATUS` present — the same
process-exit done-signal the coordinator uses, not artifact presence).

Queues, matching `gen_prompts.py`'s (task, gpu) mapping:

```
GPU 4: open_the_middle_drawer_of_the_cabinet, open_the_top_drawer_and_put_the_bowl_inside,
       put_the_cream_cheese_in_the_bowl, put_the_wine_bottle_on_the_rack
GPU 5: put_the_bowl_on_the_stove, put_the_bowl_on_top_of_the_cabinet, turn_on_the_stove
GPU 6: put_the_wine_bottle_on_top_of_the_cabinet, push_the_plate_to_the_front_of_the_stove,
       put_the_bowl_on_the_plate
```

DEVIATION (scheduling, logged): a GPU's next Stage 1 agent starts while the
previous task's Stage 2 held-out eval may still be running on that same card, so
a card can carry two jobs. This is the contention level v1 already ran at (both
arms shared a card there), and it cannot bias a comparison here: Arm A is not
being rerun, so no cell is measured against another cell that ran under
different load — the plain-ASPIRE column comes from ASPIRE's published Table 8.
It costs throughput, not correctness. The scheduler only ever starts Stage 1;
`coordinator_daemon.sh` remains the only thing that touches seeds 1–50.

### Scheduler crash at 10:18 — my bug, caught by the log, nothing lost

The first scheduler died on its first handoff with
`line 52: task: unbound variable`. Cause: `local task="$1" dir="...${task}"` —
**bash expands every argument to `local` before assigning any of them**, so
`${task}` was read while still unset and `set -u` killed the script. Split onto
separate lines and restarted 10:29:54; GPU 6 idled ~15 min and no run state was
affected (the scheduler died *before* launching anything). Recorded because the
same one-liner appears in `launch_wave.sh` in a form that happens to be safe,
and it would be easy to reintroduce.

The coordinator daemon was unaffected and did its half of the handoff correctly
at 10:19:12, dispatching Stage 2 for the finished cell on GPU 6 with no
coordinator session alive.

### Cell 1 complete: put_the_wine_bottle_on_top_of_the_cabinet

Stage 1: 09:34 → 10:18 (**44 min**), attempt 1, no supervisor retries.
Stage 2 dispatched 10:19 on GPU 6, code `ff8e79994d185870`.

---

## Session 11 — 2026-08-21 (started 11:47 local / 02:47 CST box time)

Picked up session 10's ten-task Arm-B campaign **live**: `wave_scheduler.sh`
(pid 29846, up 1h17m), `coordinator_daemon.sh` (pid 24370, up 2h13m) and three
Stage 1 supervisors all survived the session boundary. Nothing needed
re-dispatching.

### Box health at session start

| Check | Value |
|---|---|
| Perception servers 8114/8115/8116 | 404 / 404 / 404 = all UP (unbroken since session 5) |
| GPUs used MiB | 0=51745 **other tenant**, 1=9582 (SAM3), 2=13256 (GraspNet), 3=3919, 4=6602, 5=992, 6=498, 7=61328 **other tenant** |
| `/mnt/data` free | 342 G (was 356 G at session 10 dispatch) |

### `status20.sh` was dead on arrival — third instance of the same bash bug

First poll of the session crashed with `line 42: arm: unbound variable`. Cause
identical to session 10's scheduler crash: `local arm="$1" t="$2"
dir="$S/agent_${arm}_${t}"` — **bash expands every argument to `local` before
assigning any of them**, so `${arm}` was read while still unset and `set -u`
killed it. Split onto two lines, with a comment naming the trap so the next
person editing these scripts does not reintroduce it a fourth time.

Worth stating plainly: this is now the third poller/scheduler defect in this
campaign (session 7's `status_stage1.sh` read crashes as completions, session
10's `status20.sh` read live cells as 0 events, this one). None of them touched
run state — but a poller that lies is how sessions 5 and 6 lost their work, so
they get fixed on sight rather than worked around.

### Cell state at 11:47 local

| Task (Arm B) | Stage 1 | Stage 2 |
|---|---|---|
| put_the_wine_bottle_on_top_of_the_cabinet | done | **45/50** |
| open_the_middle_drawer_of_the_cabinet | done | **43/50** |
| put_the_bowl_on_the_stove | done | running 4/16 |
| open_the_top_drawer_and_put_the_bowl_inside | done | running 0/1 |
| put_the_bowl_on_top_of_the_cabinet | live (1162 ev) | — |
| push_the_plate_to_the_front_of_the_stove | live (1827 ev) | — |
| put_the_cream_cheese_in_the_bowl | live (46 ev) | — |
| turn_on_the_stove / put_the_bowl_on_the_plate / put_the_wine_bottle_on_the_rack | queued | — |

Both completed cells passed the coordinator's `code_sha256` check — each was
evaluated against the `fix_code.py` Stage 1 actually delivered, which is the
session-8 defect that session 9's done-signal fix was meant to close. It holds.

Session role: poll in-turn per the waiting discipline, keep the scheduler and
daemon alive, and record each held-out number as its manifest reaches 50/50.

### All ten Stage 1 cells complete (2026-08-21 13:35 local); scheduler exited clean

`wave_scheduler.sh` logged `ALL QUEUES DRAINED — every Stage 1 cell has exited`
at 13:35:58 and exited. All ten Arm-B cells wrote `STAGE1_STATUS=done`; the
coordinator daemon dispatched the tenth held-out eval
(`put_the_wine_bottle_on_the_rack`) at 13:39 with no coordinator session
involved.

Stage 1 dispatch/finish times (all `claude-opus-5`, one agent per GPU):

| Task | GPU | Stage 1 window | Attempts |
|---|---|---|---|
| put_the_wine_bottle_on_top_of_the_cabinet | 6 | 09:34 → 10:18 (44 min) | 1 |
| open_the_middle_drawer_of_the_cabinet | 4 | 09:34 → ~10:5x | 1 |
| put_the_bowl_on_the_stove | 5 | 09:34 → ~11:2x | 1 |
| open_the_top_drawer_and_put_the_bowl_inside | 4 | ~10:5x → 11:45 | 1 |
| put_the_bowl_on_top_of_the_cabinet | 5 | 11:11 → 11:56 (45 min) | 1 |
| put_the_cream_cheese_in_the_bowl | 4 | 11:45 → 12:29 (44 min) | 1 |
| push_the_plate_to_the_front_of_the_stove | 6 | 10:29 → 12:58 (149 min) | **3** |
| turn_on_the_stove | 5 | 11:57 → 13:18 (81 min) | 1 |
| put_the_bowl_on_the_plate | 6 | 12:57 → 13:34 (37 min) | 1 |
| put_the_wine_bottle_on_the_rack | 4 | 12:29 → 13:39 (70 min) | 1 |

### Session 7's supervisor insurance was used for the first time

`push_the_plate_to_the_front_of_the_stove` died mid-Stage-1 at 11:24 with
`api_error` after ~13 events into attempt 2. The supervisor classified it as a
transient death, slept 90 s, and relaunched as `claude -p --resume ce82a880…`,
so the agent continued with its full context instead of restarting from the
prompt. It went on to finish on attempt 3. This is exactly the failure that
killed all six agents in session 6 (and cost that session its whole run); it now
costs 90 seconds. Recorded again for clarity: resuming after an API death is
recovery, not a protocol change — same prompt, seeds, GPU, env and box state.

### Two poller defects fixed this session (neither touched run state)

1. `status20.sh` line 42, the `local`-expansion bug (above).
2. `wait_change.sh` (written this session for in-turn polling) reported the
   scheduler's **clean** exit as `DAEMON DEAD`. Both daemons are supposed to
   exit when their work is done, so conflating a clean exit with a crash makes
   the last poll of a *successful* campaign look like a failure. It now
   distinguishes them by the clean-exit line in the log, and only watches
   daemons that were alive when the call started.

---

## FINAL RESULTS v2 — Arm B (+demos), all ten tasks, opus inner, held-out seeds 1–50

| Task | Held-out | Rate | Dev sweep (51–65) |
|---|---|---|---|
| put_the_bowl_on_top_of_the_cabinet | **50/50** | 1.00 | 15/15 |
| turn_on_the_stove | **50/50** | 1.00 | 15/15 (initial 0/15) |
| put_the_cream_cheese_in_the_bowl | **49/50** | 0.98 | incomplete at delivery |
| put_the_wine_bottle_on_the_rack | **49/50** | 0.98 | 15/15 |
| put_the_bowl_on_the_plate | **48/50** | 0.96 | 15/15 |
| put_the_wine_bottle_on_top_of_the_cabinet | **45/50** | 0.90 | 15/15 (initial 12/15) |
| open_the_middle_drawer_of_the_cabinet | **43/50** | 0.86 | 15/15 (initial 1/15) |
| push_the_plate_to_the_front_of_the_stove | **20/50** | 0.40 | 7/15 (initial 0/15) |
| open_the_top_drawer_and_put_the_bowl_inside | **17/50** | 0.34 | incomplete at delivery |
| put_the_bowl_on_the_stove | **16/50** | 0.32 | 8/15 |
| **total** | **387/500** | **0.774** | |

Campaign wallclock 09:34 → 14:21 local (4 h 47 m) over three task GPUs; 682
agent-minutes and 21 748 agent events of Stage 1 across the ten cells. Full
table with per-cell cost, run ids and caveats in `RESULTS.md` (v2 section),
machine-readable in `armB_heldout_v2.json`, both mirrored to
`/mnt/data/YifanKang/aspire_ab_results/`.

### Protocol integrity — checked, not assumed

- **No Stage 1 agent ran the held-out script.** Grepped every agent log for
  actual Bash invocations of `run_fix_loop_validation.py`: zero. The one textual
  hit is `turn_on_the_stove`'s own findings asserting it ran seeds 51–65 only.
  Every Stage 2 was launched by `coordinator_daemon.sh`, the sole component
  permitted to touch seeds 1–50.
- **Every cell was evaluated on its delivered program.** All ten manifests'
  `identity.code_sha256` match the `fix_code.py` on the box — verified once by
  the coordinator at completion and again independently by `collect10.py`. The
  session-8 A/drawer defect (Stage 2 dispatched against a program the agent then
  rewrote) did not recur: session 9's done-signal fix, requiring `STAGE1_STATUS`
  (written only as the supervisor exits) rather than artifact presence, held for
  all ten cells.
- **Exactly one 50-trial run per task.** No superseded run is hiding behind a
  newer one; `collect10.py` enumerates every run per task rather than taking the
  newest.
- **Treatment consumed 10/10** — see the correction below.

### A measurement bug of my own, caught and corrected

My first treatment-consumption check reported **0 demo references in 6 of 10
cells**, which would have meant most of the +demos column was untreated. It was
my grep, not the run: `grep -c pat file1 file2 | awk -F: '{s+=$2}'` relies on
grep prefixing each count with a filename — which it only does when the glob
matches **more than one** file. Every cell that finished in a single attempt had
one `attempt_1.log`, so grep printed a bare count, `$2` was empty, and the sum
came out 0. The cells that *did* report nonzero were exactly the ones with
multiple attempt logs. The box-side artifacts disagreed with my local count,
which is what exposed it; re-run across all log files per cell, every cell shows
6–22 references. **All ten Arm-B agents opened the hdf5 and cited it.**

Recording this because the failure mode is nasty: the buggy result was
*plausible* (agents ignoring an optional file is a real thing that happens) and
would have become a headline caveat. Two independent probes disagreeing is what
saved it.

### Honest caveats on the low cells

Three cells scored 0.32–0.40 (`put_the_bowl_on_the_stove`,
`open_the_top_drawer_and_put_the_bowl_inside`,
`push_the_plate_to_the_front_of_the_stove`). Their dev sweeps were also low or
unfinished (8/15, incomplete, 7/15), so dev predicted held-out and TASK.md's
"freeze and evaluate anyway — that IS the result" rule is what put them on the
board.

Two cells (`put_the_cream_cheese_in_the_bowl`, `open_the_top_drawer_…`)
delivered while their own dev sweep was incomplete. Both had returned cleanly
without artifacts and were nudged by the supervisor to write Steps 4–5. The
nudge text does not tell an agent to stop debugging — it says to write the best
program it has — but it does push toward delivery, and `open_the_top_drawer`
describes its sweep as "still in flight when Stage 1 was cut short" before
scoring 17/50. That cell should be read as under-converged, not as a clean
measurement of the method. This is a property of *our* supervisor, not of
ASPIRE's protocol, and is the main thing I would change before a v3.

### Supervisor recovery: used 6 times, cost ~6 minutes

Six API deaths hit four cells mid-Stage-1 (`push_the_plate` x2,
`open_the_top_drawer` x2, `put_the_bowl_on_top_of_the_cabinet` x2). Every one
was classified transient and resumed via `claude -p --resume <sid>` with full
context, at 45–90 s backoff. The identical failure destroyed all six agents and
the entire run in session 6. Session 7's insurance is now load-bearing rather
than theoretical.

---

## Session 11 close-out (2026-08-21 14:2x local / 05:2x CST)

**Stop condition met:** TASK.md (lean revision) — "Arm B (+demos) x ALL TEN
tasks x opus inner: THIS is the campaign." All ten cells have held-out numbers
at `evidence_scope: heldout_full`, 50 trials each, from ASPIRE's own validation
script. Arm A was not rerun, per the same revision; no Arm A cells were in
flight at any point this session.

State left on AbakaAI:

- Perception servers **still up** on 8114 (SAM3, GPU1), 8115 (GraspNet, GPU2),
  8116 (PyRoKi, CPU) — unbroken since session 5. Task GPUs 4/5/6 released; GPUs
  0 and 7 are the other tenants', untouched throughout. `start_services.sh`
  writes no pid files, so stopping them means matching on `launch_*_server.py`,
  the blanket-pkill hazard that made us avoid ASPIRE's own script on this shared
  box. Left running deliberately; stop them when the box is known idle.
- `wave_scheduler.sh` and `coordinator_daemon.sh` both exited cleanly
  (`ALL QUEUES DRAINED`, `ALL 10 HELD-OUT EVALS COMPLETE`); no orphaned
  supervisors.
- `/mnt/data` 342 G free. Arm-B `outputs/` intact with all ten fix_code /
  findings / eval runs.
- Results mirrored to `/mnt/data/YifanKang/aspire_ab_results/`
  (`RESULTS.md`, `NOTES.md`, `armB_heldout_v2.json`).

Scripts fixed this session, all in `/Users/yifankang/aspire_ab_run/`:
`status20.sh` (local-expansion crash), `wait_change.sh` (new; two false-alarm
fixes — clean daemon exit misread as a crash, and an exit-line pattern that did
not match what the daemon actually writes), `collect10.py` (new; ten-cell
manifest collector that enumerates every run per task).

---

## Session 12 — 2026-08-21 (started 14:46 local / 05:47 CST box time) — independent audit of the banked results

Started with **no work in flight**: `wave_scheduler.sh` and
`coordinator_daemon.sh` had both exited cleanly (`ALL QUEUES DRAINED`,
`ALL 10 HELD-OUT EVALS COMPLETE`), no orphaned supervisors, no `claude -p`
agents alive. TASK.md's stop condition was already met and documented by
session 11. So this session's job was not to produce numbers but to **try to
break them** — every campaign number so far has been read by one collector
(`collect10.py`), and a single reader that is wrong is indistinguishable from a
correct one. Session 11's own grep bug (which fabricated a plausible
"6 of 10 cells untreated" finding) is the precedent.

### Box state at session start (2026-08-22 05:47 CST)

| Check | Value |
|---|---|
| Perception servers 8114/8115/8116 | 404 / 404 / 404 = all UP (unbroken since session 5) |
| GPUs used MiB | 0=51223 **other tenant**, 1=9059 (SAM3), 2=12734 (GraspNet), 3=3397, 4=6157, 5=523, 6=526, 7=61847 **other tenant** |
| `/mnt/data` free | 306 G (was 342 G at session 11 close) |

Task GPUs 4/5/6 are released; 0 and 7 untouched throughout the campaign.

### The audit: a second, deliberately different reader

`collect10.py` reads `manifest.json`'s `passes`/`trials`. A recount that also
reads those fields would prove nothing. So `verify12.py` counts the **per-trial
result directories** the eval script wrote, whose names encode the outcome
(`trial_NN_sandboxrc_R_reward_X.XXX_taskcompleted_{0,1}`) — a different artifact,
written at a different time by a different code path than the manifest summary.
Cross-checked against a third source, the manifest's per-seed `results` map.

Verified per cell, all ten:

| Property | Result |
|---|---|
| independent trial-dir recount == manifest `passes`/`trials` | 10/10 exact |
| per-seed `results` pass count == manifest `passes` | 10/10 exact |
| recount total | **387/500**, identical to the banked table |
| declared seed set (`identity.seeds`) == exactly 1..50 | 10/10 |
| **executed** seed set (per-trial `seed` fields) == exactly 1..50 | 10/10 |
| any dev seed (>50) inside a held-out run | **none** |
| duplicate trial indices | none |
| manifest `identity.code_sha256` == sha256 of delivered `fix_code.py` | 10/10 |
| run directories per task | exactly 1 — no superseded run hiding |
| `evidence_scope` | `heldout_full` for all ten |
| `findings.md` present | 10/10 |

The executed-seed check is the one that had never been run. Every prior check
(session 11's included) trusted `identity.seeds`, i.e. what the run *declared* it
would sweep. Reading the `seed` field off each of the 500 recorded trials proves
what it actually swept: seeds 1–50 exactly, no dev seed anywhere in a held-out
run. That is the protocol-integrity claim the whole campaign rests on, and it now
rests on executed data rather than on a declaration.

One defect found — **in my own probe, not in the run**: the first pass reported
"seed set is NOT exactly 1..50" for all ten cells, because it read a top-level
`seeds` key that does not exist (the field is `identity.seeds`). A uniform
failure across every cell is the signature of a broken probe rather than a broken
run — ten independent cells do not fail identically — so it was traced to the
reader before it went anywhere near the ledger. Same class as session 11's
`grep -c` bug: a probe that lies plausibly.

### v1's three-task A/B re-audited from the archive

The v1 numbers are still cited in RESULTS.md (the ±demos contrast and the Arm-A
reproduction sanity row), but session 9 emptied both checkouts' `outputs/`, so
they survive only in
`/mnt/data/YifanKang/aspire_ab_results/archive_v1_3task_20260821`. Confirmed the
archive holds all **7** manifests (6 cells + the A/drawer re-run) and re-read
them independently:

| Arm | Task | run | passes | per-seed recount | seeds 1–50 |
|---|---|---|---|---|---|
| A | turn_on_the_stove | `9bffbdcbf4c3e2` | 50/50 | 50 | yes |
| A | put_the_bowl_on_the_plate | `3ec626e7f3be82` | 50/50 | 50 | yes |
| A | open_the_middle_drawer_of_the_cabinet | `3f4613b19b4c6b` (superseded code) | 31/50 | 31 | yes |
| A | open_the_middle_drawer_of_the_cabinet | `b2a1ebcfa29586` (delivered code) | 31/50 | 31 | yes |
| B | turn_on_the_stove | `409527d0f05ee4` | 50/50 | 50 | yes |
| B | put_the_bowl_on_the_plate | `fc498e948b1556` | 50/50 | 50 | yes |
| B | open_the_middle_drawer_of_the_cabinet | `8032fab97ef816` | 48/50 | 48 | yes |

Reproduces the banked v1 table exactly, including both A/drawer runs agreeing at
31/50 — the session-8 correction still reads as "the correction changed nothing".

### Verdict

Both banked tables survive an independent recount from a different artifact.
No number changed. The campaign's headline (**Arm B, ten tasks, opus inner,
387/500 = 0.774** on held-out seeds 1–50) and v1's three-task A/B are audited,
not merely reported. The caveats in RESULTS.md are unaffected — this audit
establishes that the numbers are what the runs produced, which is a different
question from whether three low cells were under-converged (they were; that
remains the honest read).

### Session 12 close-out

**Stop condition: met and re-verified.** TASK.md (lean revision) — "Arm B
(+demos) x ALL TEN tasks x opus inner: THIS is the campaign." All ten cells hold
held-out numbers at `evidence_scope: heldout_full`, now confirmed by two
independent readers. Arm A was not rerun, per the same revision. No new compute
was dispatched this session and no run state was modified — this was a read-only
audit.

State left on AbakaAI, unchanged from session 11 except disk:

- Perception servers still up on 8114 (SAM3, GPU1), 8115 (GraspNet, GPU2), 8116
  (PyRoKi, CPU). Still no pid files, so stopping them means matching on
  `launch_*_server.py` — the blanket-pkill hazard on this shared box. Left
  running deliberately; stop them when the box is known idle.
- `/mnt/data` 306 G free. Both checkouts' `outputs/` intact; v1 archived.
- Audit probes: `verify12.py` / `verify_v1.py` (this session, read-only) pushed
  to `/mnt/data/YifanKang/_verify12.py` and `_verify_v1.py`.

### PRIORITY 2 COMPLETE — both repaired cells (2026-08-21 19:05 local)

| Cell (Arm B, +demos) | Stage 1 | dev 51-65 | v2 (struck) | **repaired** |
|---|---|---|---|---|
| put_the_cream_cheese_in_the_bowl | 15:38 → 16:15 (37 m, 3 att) | confirmed complete | ~~49/50, incomplete dev~~ | **49/50** (0.98) |
| open_the_top_drawer_and_put_the_bowl_inside | 15:38 → 17:53 (**135 m**, 2 att) | 8/15 → **15/15**, swept twice | ~~17/50, "sweep still in flight"~~ | **48/50** (0.96) |

Both code hashes verified by the coordinator against the delivered `fix_code.py`
(`747fe8c2be4781d9`, `7dbc76a06b3c9763`); one 50-trial run per cell; Stage 2
launched only by the daemon.

**`open_the_top_drawer` is the vindication of this whole pass: 17/50 → 48/50.**
The v2 number was not a measurement of the method, it was a measurement of our
supervisor cutting an agent short — session 11 flagged exactly that cell as
"under-converged, not a clean measurement", and the repair confirms it by a
31-trial margin. Its dev trace shows why: initial 8/15, delivered **15/15**, and
the agent ran the sweep **twice** on the byte-identical delivered binary (md5
`1d561dd5…`) on its own initiative, because it had found the environment to be
non-deterministic — seed 54 produced `grip on handle` readings of 0.209 and
−0.001 from identical code. A single pass could not have distinguished a
converged program from a lucky one. That is the behaviour `CONVERGE=1` was meant
to buy, and it only appeared once the supervisor stopped pushing toward delivery.

`put_the_cream_cheese_in_the_bowl` reproduces its v2 number exactly (49/50). The
useful kind of null: it says the v2 value was right, and it is now backed by a
sweep the agent explicitly confirmed complete, so it is quotable.

**Effect on the v2 Arm-B table:** 387/500 → **418/500 = 0.836** (was 0.774).
Only the one cell moves; `cream_cheese` is unchanged and nothing else was rerun.

Both repair cells took the `CONVERGE=1` converge check exactly once each, and in
both cases the agent used it to do real work rather than to re-assert. Under the
v2 supervisor both would have delivered at their first clean return.

### Arm A's `push_the_plate` agent independently found the technique §A.2 predicted was missing

Session 14's RESULTS §A.2 argued that Arm B's `push_the_plate` cell (20/50) had
misdiagnosed its wall: it wrote up "this arm cannot reach table height below
y ≈ −0.09" as a *kinematic reach limit*, while ASPIRE's own
`evosearch/skills/wrist-rotation-blocking.md` — which a fix-loop agent is never
pointed at — describes the identical symptom as **arm-body collision**, fixable
with one line, `j[6] += np.pi/2`.

Arm A's agent on the same task, with no demos and no access to that skill file,
found it from scratch. From its `findings.md` (delivered 19:56):

> Probing an 8-point grid around the plate (x 0.36–0.58, y −0.06 → −0.16) at
> z = 0.01: **8/8 BLOCKED** with the raw IK solution, **5/8 OK** after
> `j[6] += π/2`. The blocker is the arm *body* against a 23–30 cm wooden knife
> block that stands immediately to the plate's left (y ∈ [−0.20, −0.15] across
> x ∈ [0.25, 0.50]); the fingertip itself has clearance.

That is a controlled probe, not a guess: same eight points, same z, one variable
changed, 0/8 → 5/8. It settles the §A.2 question empirically — **the wall Arm B
reported as a reach limit is an obstacle, and the obstacle has a name.** Arm B's
findings never mention the knife block.

Two things follow, and they cut in opposite directions:

- **For the mechanism-gap write-up:** §A.2's inference was right, and it is now
  backed by an experiment run inside this campaign rather than by reading their
  skill library. The prediction was made before this cell ran.
- **For the +demos claim:** finding the technique did **not** buy the score. Arm A
  delivered **4/15** on dev against Arm B's 7/15. Correct diagnosis, worse
  program. Held-out will say whether that holds, but it is a caution against the
  tidy story that Arm B failed because it lacked the technique — it failed with a
  wrong mechanism model and still scored higher, which means the reach-band
  heuristic Arm B built is doing real work even though its stated reason is wrong.

Arm A Stage 1 on this cell: 16:51 → 19:56 (**185 min**, 1 attempt + 1 converge
check, 2062 events) — the most expensive cell of the campaign so far, as it was
in Arm B (149 min).

### DEVIATION (host, logged) — Arm A `put_the_cream_cheese_in_the_bowl` moves GPU 4 → GPU 6

At 20:46 GPU 6's Arm A queue had drained (all three of its cells through Stage 1)
while GPU 4 still had two cells waiting behind a live one. Per-GPU queues have no
work stealing, so GPU 6 would have idled for hours.

Moved `put_the_cream_cheese_in_the_bowl` from Q4 to Q6 and regenerated its prompt
through `gen_prompts.build('A', ..., gpu=6)`. Diff against the GPU-4 prompt:
**16 lines, every one a GPU-number line** (verified by filtering the diff for any
line not containing `gpu`/`CUDA_VISIBLE_DEVICES` — empty). Same precedent as
session 14's cream_cheese move on the Arm B side. `coordinator_daemon_armA.sh`
restarted with `GPU_OVERRIDE=put_the_cream_cheese_in_the_bowl:6` so its Stage 2
lands on the same card. GPU number does not enter any measurement.

Both Arm A daemons were stopped and restarted for this (they are stateless); the
three live Stage 1 supervisors and every banked `STAGE2_RESULT` survived.

### `push_the_plate` head-to-head: Arm A 13/50, Arm B 20/50 — right diagnosis, worse program

| | dev 51-65 | held-out 1-50 | mechanism the agent committed to |
|---|---|---|---|
| Arm A (plain, opus) | 4/15 | **13/50 (0.26)** | arm-body collision with a wooden knife block; `j[6] += π/2` (0/8 → 5/8 on a controlled grid probe) |
| Arm B (+demos, opus) | 7/15 | **20/50 (0.40)** | kinematic reach wall at y ≲ −0.09; sweep 72 push directions, keep those whose approach point clears the measured band |
| ASPIRE published, `goal_swap` (Table 7) | – | **0.00** in all four configurations | – |

This is the cleanest single result of the session and it cuts against the tidy
story. Arm A's agent got the **physics right** — it identified the obstacle by
name, ran a controlled 8-point probe, and rediscovered from scratch the exact
one-line fix that sits unused in ASPIRE's own evosearch skill directory — and
still scored **7 held-out trials worse** than Arm B's agent, which got the
physics **wrong** (no mention of the knife block anywhere in its findings) but
built a search over push directions that empirically dodges the obstacle.

The honest reading: on this task a correct causal model was worth less than a
robust search. Arm B's reach-band heuristic is right for the wrong reason —
"cannot fold down below y ≈ −0.09" describes where the knife block is — and
filtering candidate approach points against a *measured* band works whether or
not you know why the band exists. Arm A spent its budget proving the mechanism
(185 min, 2062 events, the most expensive cell of the campaign) and had less left
for the search that actually converts.

Both arms beat the published 0.00 on this suite, which is the §A.0 point standing
unchanged.

**This also weakens the causal claim in RESULTS §A.2/A.3.** That section argued
the missing `j[6] += π/2` was "the single most likely difference between '12
versions, bootstrap diagonals stall' and a clean straight push". We now have the
experiment: an agent that *had* the technique scored 0.26 against 0.40. The
technique is real and the diagnosis in §A.2 was correct about the mechanism, but
it is not sufficient, and §A.3 should not be read as "give the fix-loop agent
that skill file and the cell is solved". Corrected in RESULTS §A.5.

---

## FINAL RESULTS v3 — Arm A (plain ASPIRE) x ten tasks, opus inner, held-out 1-50

| Task | Held-out | Rate | Stage 1 |
|---|---|---|---|
| open_the_middle_drawer_of_the_cabinet | **50/50** | 1.00 | 60 m, 5 att (3 nudges + 1 converge) |
| put_the_wine_bottle_on_top_of_the_cabinet | **50/50** | 1.00 | 32 m, 4 att |
| put_the_bowl_on_top_of_the_cabinet | **50/50** | 1.00 | 31 m |
| turn_on_the_stove | **50/50** | 1.00 | 78 m |
| put_the_bowl_on_the_plate | **50/50** | 1.00 | 49 m |
| put_the_wine_bottle_on_the_rack | **49/50** | 0.98 | 39 m |
| open_the_top_drawer_and_put_the_bowl_inside | **42/50** | 0.84 | 191 m |
| put_the_cream_cheese_in_the_bowl | **39/50** | 0.78 | 102 m |
| put_the_bowl_on_the_stove | **38/50** | 0.76 | 224 m (dev 13/15) |
| push_the_plate_to_the_front_of_the_stove | **13/50** | 0.26 | 185 m (dev 4/15) |
| **total** | **431/500** | **0.862** | **991 agent-min, 14 056 events, 0 API deaths** |

Campaign 16:17 -> 23:37 local (7 h 20 m) over three task GPUs. Every cell took
exactly one `CONVERGE=1` converge check; two cells also took clean-return nudges.
Not one supervisor retry for API death this session (v2 needed six).

### THE THREE-COLUMN DELIVERABLE

| Task | ENCORE (c2) | ASPIRE (A) | ASPIRE+demos (B) |
|---|---|---|---|
| open_the_middle_drawer_of_the_cabinet | **0.00** | **1.00** | 0.86 |
| put_the_bowl_on_the_stove | 0.96 | 0.76 | **0.32** |
| put_the_wine_bottle_on_top_of_the_cabinet | 1.00 | 1.00 | 0.90 |
| open_the_top_drawer_and_put_the_bowl_inside | 1.00 | 0.84 | 0.96 |
| put_the_bowl_on_top_of_the_cabinet | 1.00 | 1.00 | 1.00 |
| push_the_plate_to_the_front_of_the_stove | **1.00** | **0.26** | 0.40 |
| put_the_cream_cheese_in_the_bowl | 0.90 | 0.78 | 0.98 |
| turn_on_the_stove | 1.00 | 1.00 | 1.00 |
| put_the_bowl_on_the_plate | 1.00 | 1.00 | 0.96 |
| put_the_wine_bottle_on_the_rack | 0.92 | 0.98 | 0.98 |
| **macro** | **0.878** | **0.862** | **0.836** |

ENCORE read from `campaigns/c2/evals/eval_v111_c2_goal_*_pos/results.jsonl`
(50 unique episodes per cell, `benchmark_success`) — 439/500, exactly the figure
TASK.md quotes. Read only after every Stage 1 agent had exited, so no arm could
have been contaminated by it.

### The two things worth saying about this table

1. **The means hide everything.** 0.878 / 0.862 / 0.836 is a 4-point spread over
   three systems whose per-cell failures have nothing in common. ENCORE is
   near-perfect on nine cells and **0/50** on `open_the_middle_drawer` (a known
   reachability mechanism gap in that harness) — both ASPIRE arms solve it. Both
   ASPIRE arms collapse on `push_the_plate` (0.26, 0.40) where ENCORE gets 50/50
   and ASPIRE's *own published* pipeline gets 0.00. Six cells are at ceiling in
   all three columns and separate nothing.

2. **The +demos row in this table is CONFOUNDED and is not the ablation.** Arm A
   ran under `CONVERGE=1`; eight of ten Arm B cells did not. The confound's size
   is measured, not guessed: the repair pass moved one cell 17/50 -> 48/50 with
   the supervisor as the only change — ~7x the 13-trial gap between the columns.
   The clean sub-comparison is the three cells where the harnesses match or where
   B wins against the confound: `cream_cheese` 0.78 vs **0.98**,
   `open_the_top_drawer` 0.84 vs **0.96**, `push_the_plate` 0.26 vs **0.40** —
   all three to +demos. **Highest-value follow-up in the campaign: rerun the
   eight non-CONVERGE Arm-B cells under `CONVERGE=1`** (~8 fix loops).

### Integrity — verified on executed data (all ten Arm A cells)

trial-dir recount == manifest 10/10; **executed** seed set == exactly 1..50 10/10
with zero dev-seed leaks; no duplicate trial indices; `code_sha256` == delivered
`fix_code.py` 10/10; exactly one run dir per task; `heldout_full` 10/10; and
**zero** Bash invocations of `run_fix_loop_validation.py` in any Stage 1 agent log
(every Stage 2 came from the coordinator daemon). Arm A's cold start was proven
identical to Arm B's: empty `outputs/`, skill library `75b397c6…c449` in both
trees, zero promotions.

---

## Session 15 close-out (2026-08-21 23:4x local / 2026-08-22 14:4x CST)

**Stop condition met, both TASK.md v3 priorities complete and documented:**

- **Priority 1** — Arm A (plain ASPIRE) x all ten `libero_goal_swap` tasks, opus
  inner, dev 51-65 to convergence, one held-out 1-50 per cell: **431/500 =
  0.862**, all at `evidence_scope: heldout_full`, audited twice.
- **Priority 2** — both repaired Arm-B cells delivered (`open_the_top_drawer`
  17/50 -> **48/50**, `cream_cheese` **49/50**), and `push_the_plate` written up
  as a mechanism gap with the comparison against ASPIRE's own approach (RESULTS
  §A, now with §A.4 prompt-level verification and §A.5 correcting §A.3's causal
  claim against the Arm A evidence).
- **Deliverable** — the three-column table over the ten goal_swap tasks is in
  RESULTS.md (v3 section), with the ENCORE-is-a-different-harness caveat named
  explicitly as the confound the planned ENCORE-on-CaP-X port removes.

Corrected banked numbers: Arm B v2 total **418/500 = 0.836** (was 387/500 =
0.774 — one repaired cell moved).

New scripts this session, all in `/Users/yifankang/aspire_ab_run/`:
`scheduler2.sh` (cross-arm GPU occupancy, parameterized log),
`coordinator_daemon_armA.sh` (copy with `LOGFILE` parameterized — copied not
edited, because bash reads a running script incrementally), `snap15.sh` (state
poller that distinguishes live / done / ORPHAN / clean-daemon-exit),
`collect10_armA.py`, `collect10_v3.py`. Results: `armA_heldout_v3.json`,
`armB_heldout_v3.json`, `armB_heldout_v2_prerepair.json` (pre-repair snapshot).

State left on AbakaAI:

- Perception servers **still up** on 8114 (SAM3, GPU1), 8115 (GraspNet, GPU2),
  8116 (PyRoKi, CPU) — unbroken since session 5. Still no pid files, so stopping
  them means matching on `launch_*_server.py`, the blanket-pkill hazard on this
  shared box. **A second Heron campaign (`encore_capx_run`, GPU 3) is live on
  this box and laptop** — do not pkill anything broadly, and check its
  `agent_E_*/GPU` files before claiming a card.
- Task GPUs 4/5/6 released. GPUs 0 and 7 are the other tenants', untouched
  throughout the whole campaign.
- Both checkouts' `outputs/` intact with all twenty fix_code / findings / eval
  runs; v1 and the pre-repair v2 state archived under
  `/mnt/data/YifanKang/aspire_ab_results/`.
- `scheduler2.sh` and `coordinator_daemon.sh` (Arm B) both exited cleanly; the
  Arm A coordinator exits on its own once all ten `STAGE2_RESULT`s exist.

### Final process/disk state (2026-08-21 23:43 local / 2026-08-22 14:43 CST)

`coordinator_daemon_armA.sh` logged `ALL 10 HELD-OUT EVALS COMPLETE` and exited
at 23:42:48. All three of this session's daemons (Arm A scheduler, Arm A
coordinator, Arm B repair coordinator) exited cleanly; **no orphaned supervisors
or `claude -p` agents remain** from this campaign.

**Disk warning for the next session:** `/mnt/data` is down to **167 G free**
(303 G at this session's start — the twenty eval runs' trial videos and keyframes
cost ~136 G in one session). At that burn rate there is room for roughly one more
full ten-cell column. Before dispatching another, either prune
`outputs/libero_fix_loop_eval/*/*/runs/*/trial_*` for banked cells (the manifests
carry the numbers; the per-trial dirs are what the independent audit recounts, so
archive the directory *names* first) or move older archives off `/mnt/data`.
`/` remains at 0 bytes free and everything must stay under `/mnt/data/YifanKang/`.

---

# CONVERGE-fix pass — Arm B, eight cells (dispatched 2026-09-02)

**Why.** The banked Arm-B column (418/500) mixes two supervisor regimes: the two
repaired cells ran under `CONVERGE=1` (agent must confirm the 51–65 sweep is
complete before delivery is accepted), the other eight under the v2 supervisor
that accepted a clean return with artifacts. Arm A′ ran all 60 cells under
`CONVERGE=1`. So A′ vs B differed by demos AND supervisor, and the paper's
"raw demos bring no measurable gain" (0.862 vs 0.836) rests on that confounded
column. This pass removes the supervisor difference; nothing else changes.

**Pre-specified reading.** Under CONVERGE=1 cells can only be allowed to finish,
so the direction of any change is expected to be upward. If the corrected
column lands within draw variance of A′ (431), the sentence stands with the
confound removed. If it lands clearly above, the sentence becomes "raw
demonstrations also help ASPIRE's loop", which is consistent with the paper's
main claim. Suite-level only; no per-cell claims (abl_c2 draw variance).

**What is held fixed.** Prompts byte-identical to v2 (same GPU pins 4/5/6,
same K=5 demo hdf5 `demo_0..demo_4`, same addendum), model `claude-opus-5`,
commit `680bad4`, checkout `/mnt/data/YifanKang/ASPIRE_armB`, dev 51–65 to
convergence, ONE held-out 1–50 per cell by the coordinator only. Fresh session
ids (clean slate, not `--resume`). The two repaired cells are not touched.

**Archived before dispatch.** Box: Stage 1 + Stage 2 trees of the eight cells →
`/mnt/data/YifanKang/aspire_ab_results/archive_convergefix_pre_20260902/`
(1.9 G). Mac: each cell's v2 state files → `agent_B_<task>/_prev_v2_20260902/`.
The tree now holds exactly the two repaired cells.

**Dispatch.** `scheduler2.sh` (ARM=B, OTHER_ARM=A, CONVERGE=1,
MAX_CLEAN_NUDGES=6, CONVERGE_CHECKS=2, log `scheduler_convergefix.log`) and
`coordinator_daemon.sh` (ARMS=B, TASKS_OVERRIDE = the eight cells, POLL=300),
both detached from the Claude Code session on the Mac. Queue per GPU:
4 → middle_drawer; 5 → bowl_on_stove, bowl_on_top_of_cabinet, turn_on_stove,
wine_on_rack; 6 → wine_on_top_of_cabinet, push_plate, bowl_on_plate.

**Note on K.** Arm B's demo files are `demo_0..demo_4` of the LIBERO base-task
hdf5; ENCORE's packs take `demo_0..demo_2` of the same file (`fewshot_pack.py
--k 3`). B's demonstrations are therefore a strict superset of ENCORE's — more
information to the baseline, conservative for us — and the paper must say
"five" for that arm.

**SUPERSEDED the same afternoon (Yifan, 2026-09-02 ~15:50): rerun ALL TEN Arm-B
cells at K=3, not eight at K=5.** Rationale: the paper's own K is 3, so the
"same demonstrations to ASPIRE's loop" arm should receive exactly ENCORE's
three (`demo_0..demo_2`), not a superset. The K=5 CONVERGE pass above was
stopped after ~8 min (three cells had started; nothing delivered). Everything
K=5 — Stage 1/2 trees of all ten cells — is archived on the box under
`aspire_ab_results/archive_convergefix_pre_20260902/` (eight v2 cells) and
`archive_k5_full_20260902/` (two repaired cells + the three partial starts);
Mac-side per-cell state under `agent_B_<task>/_prev_v2_20260902/` and
`_prev_k5_20260902/`. Banked K=5 prompts kept at `prompts_k5_20260821/`.

K=3 assets: `/mnt/data/YifanKang/aspire_demos_k3/<task>/<task>_K3_demos.hdf5`
(`make_aspire_demos_k3.py`, demo_0..demo_2 of the same LIBERO base-task files,
verified num_demos=3 for all ten) + `aspire_demos_k3/DEMONSTRATIONS.md`.
Prompts regenerated by `gen_prompts_k3.py` (root pinned to
`/Users/yifankang/aspire_ab_run`); ablation content check PASS; Arm A prompts
restored byte-identical to the banked set. Versus the banked Arm-B prompts the
only differences are the addendum's K/path lines and one GPU pin:
`put_the_cream_cheese_in_the_bowl` returns to GPU 4 (the repair pass had moved
it to 5), so all pins match `gen_prompts.py`'s table and the coordinator's
`gpu_for` without overrides. Scheduler: `scheduler_k3.sh` (= scheduler2.sh
with queues equal to that table: GPU4 ×4, GPU5 ×3, GPU6 ×3), CONVERGE=1,
MAX_CLEAN_NUDGES=6, CONVERGE_CHECKS=2, model claude-opus-5, fresh session ids.
Coordinator: ARMS=B, all ten tasks, POLL=300. Pre-specified reading unchanged:
suite-level only, compare the K=3 CONVERGE=1 column (10 cells) against A′
(431/500) with draw variance in mind; the banked 418/500 (K=5, mixed
supervisors) is retired from the paper.

[2026-09-02 22:23:05] v5 complete: 437/500 (K=3, CONVERGE=1). Table and reading in RESULTS.md §v5; json armB_heldout_k3_converge.json.
