# FAIR_PROTOCOL v1.0 — information-fair worker contract (campaign c2+)

Supersedes the c1 observability contract for every campaign that claims a
strict held-out, GT-free benchmark number. Written after the c1 audit
(2026-08-14) found: (a) `ProgramApi.robot` exposed `_sim()`/`gt_poses()`/`env`
to any program; (b) development-time diagnosis scripts read simulator GT;
(c) several probe scripts examined episodes inside the evaluation band.
c1 numbers therefore remain **exploratory**; c2 numbers are the citable ones.

## 1. Structural isolation (not a promise — a process boundary)

Programs run under `tools/fair_run.py` ONLY:

    Coding-agent program  (python -I, scrubbed env, sandbox cwd)
            │  unix-socket RPC only (tools/fair_client.py)
            ▼
    fair_run.py server — capture / eef / tool_rotation / gripper / proprio /
                         move / grip / settle / log / instruction / done
            │
            ▼
    Simulator + GT judge (LiberoRobot; check_success is the verdict)

The program process contains NO simulator object. `api.robot`, `env`, `sim`
do not exist. A PEP-578 audit hook additionally denies file reads matching
benchmark-asset patterns (.bddl/.xml/.hdf5/init_states/gt_trace/LIBERO/...).

The program MAY receive: RGB, aligned depth, camera intrinsics/extrinsics,
EEF pose, tool rotation, gripper state, joint proprioception, the task
instruction, command results/residuals, the scalar episode outcome, and
non-GT videos/logs of its own runs.

The program (and the coding agent, during a cell) may NEVER access:
`robot`/`env`/`sim` objects, body/site/geom poses, BDDL/XML/URDF, init-state
files, HDF5 simulator states, GT traces, or evaluation-seed artifacts.

## 2. Sealed splits (ASPIRE-comparable)

- **debug/learn = seeds 51–65.** The ONLY seeds a worker may run or inspect.
  `fair_run.py --split debug` refuses anything outside the band.
- **evaluation = seeds 1–50, exactly, once.** `--split eval` runs the full
  band blind: per-episode results, logs and GIFs stage privately and publish
  only after seed 50 completes. Coordinator-run only; workers never invoke it.
- Version selection happens on debug seeds; the selected program is FROZEN
  (md5 recorded) before eval. After eval publishes, the program may not be
  edited and re-scored against the same claim — a re-run is a NEW cell.
- No per-seed result, GIF, or log from the eval band reaches any coding agent
  before all 50 episodes finish; after publication they are post-hoc record,
  not iteration material.

## 3. Demo packs

Built by `tools/fair_pack.py` only (whitelist-validated): K=3 demos'
keyframe RGB, EEF/proprioception paths, gripper commands/state, raw actions,
task language. No `source` paths, no simulator states, no pose fields —
the validator refuses path-like strings anywhere in pack.json.

## 4. Provenance of constants (hard-coding is fine; the SOURCE is the rule)

Every calibrated constant in a frozen program must appear in a top-level
`PROVENANCE` dict; `fair_run.py --split eval` refuses programs without one,
or with any entry not `allowed: True`:

    PROVENANCE = {
        "BASKET_XY":    {"source": "demo keyframes 0/1/2 release EEF", "allowed": True},
        "RIM_OFF":      {"source": "debug seeds 51-65 RGB-D measurement", "allowed": True},
    }

Allowed sources: demo-pack contents, debug-seed observations (RGB-D, EEF,
gripper, success bits), robot/camera/controller documentation. Forbidden
sources: anything in §1's never-list, eval-seed observations, or constants
inherited from a privileged-era artifact (see §5).

## 5. Clean-context requirement

c1 laws, packs, NOTES, memories and any constant derived during the
privileged era are CONTAMINATED for c2 purposes. A c2 worker starts from a
fresh workspace containing only: this protocol, its TASK.md, the fair pack,
and the c2 law library (grown from zero, c2 receipts only). Coordinator
sessions must not paste c1-derived diagnoses into c2 briefs, and c2 runs in
a fresh working directory so no agent memory namespace carries c1 priors.

## 6. Enforcement layers (defense in depth)

1. Process boundary (fair_run/fair_client) — the guarantee.
2. Audit-hook file tripwire in the program process — catches honest mistakes.
3. Static token scan at launch (`gt_pose|env.sim|h5py|.bddl|...`) — refuses
   before the sim ever steps.
4. PROVENANCE gate on eval.
5. Coordinator audit per round: grep worker transcripts for forbidden reads,
   verify exactly one eval dir per cell, verify eval preceded by debug-only
   artifacts. Same sweep as c1's, plus: any eval-band seed number appearing
   in a worker transcript before publication voids the cell.

## Amendment v1.1 (2026-08-15) — no runtime success oracle; blindness is procedural
1. `api.done` reflects EPISODE TERMINATION ONLY (horizon). The benchmark
   success predicate is never queryable at runtime — v1.0's done flag
   included `task_success`, which made it an online evaluator oracle
   (per-step success feedback no real robot or ASPIRE/CaP-X policy has).
   The §1 allowed list is amended: "the scalar episode outcome" is available
   POST-EPISODE ONLY (the `benchmark_success` field of debug-split
   results.jsonl). Numbers produced under v1.0 are labeled
   "RGB-D + proprio + online success oracle" and kept only as an ablation row;
   official numbers require v1.1 programs (no done-based success branching)
   re-selected and re-evaluated.
2. Blind-eval staging and --gt-dir are chmod 0700, and the coordinator is
   bound by an explicit no-read rule until publish; on a single-user cluster
   this is PROCEDURAL (any same-user process can read them). Structural
   blindness requires a second OS account / server-held results; documented
   as a limitation. Coordinator staging reads are incidents, logged in
   STATE.md.
