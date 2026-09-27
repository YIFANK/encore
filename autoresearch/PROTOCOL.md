# Autoresearch Session Protocol

You are an autonomous coding agent solving ONE manipulation task from K=3
human demonstrations. No human will answer questions; decide and act.

## Inputs (in this workspace)

- `TASK.md` — the task language, bddl/init/demo paths, and eval commands.
- `packs/<task>/pack.json` — 3 demos: keyframes, strided ee_path6 (see the
  `stride` field), raw action streams, action scale.
- `LAWS.md` — the accumulated execution/perception/protocol laws. Follow
  them; they are measurements, not opinions.
- Reference implementation: `reference/program_l2c.py` (a banked 15/20
  program with the canonical skeleton) — adapt, don't reinvent.

## Contract

- The deployed program touches ONLY RGB-D (`api.capture`), proprioception
  (`api.eef()`, `api.robot.get_gripper`), and the pack. Simulator ground
  truth is legal ONLY in probes/diagnosis and in `reward.py` (judge-only).
- Write `program.py` + `reward.py` into `packs/<task>/` (cluster copy);
  keep every version's diagnosis in `NOTES.md` (one line per version:
  hypothesis → evidence → verdict).
- All runs happen on the cluster via ssh (alias `AbakaAI`); sync with
  `rsync -az packs/<task> AbakaAI:/mnt/data/YifanKang/Heron/packs/`.

## Session discipline

Work directly in THIS session. Do NOT spawn background agents, subagents,
or detached "autoresearch loops" — a headless session's children die the
moment you stop, and a session that ends with work "delegated" has
delivered nothing. Long waits are fine: launch the cluster run with
nohup, then poll its log with sleep-and-check until it finishes. Do not
end your session until the formal 20-episode result is banked (or a stop
condition below is met and documented). "Banked" means: the formal run
FINISHED, you read its results.jsonl, and you wrote the N/20 into
NOTES.md — launching it and promising to "report back later" is not
banking; a headless session cannot come back.

## Loop

1. Measure first: demo segment map (grip flips), scene GT probe on inits
   {0,7,14}, same-instrument anchors on the demo's own init (LAWS #6).
2. Program version → probe `--episode-list 0,7,14,3,11` → read the
   program log, GT traces, and (when confused) the episode film. One
   hypothesis, one control, one fix per version (LAWS #19).
3. At probe ≥ 4/5: run the 20-episode formal on the official init file
   (`--episodes 20`, out dir `final20_<task>_stock`). That number is the
   deliverable.
4. Stop when: formal ≥ 15/20, or 12 program versions, or you judge the
   remaining failures need a mechanism outside the laws (document it in
   NOTES.md as a candidate new law — that is also a deliverable).
5. **Version selection**: the task's final program is the version with the
   BEST banked 20-episode score, never merely the newest — iteration can
   regress, and archived versions are kept precisely so the argmax is
   recoverable. If different versions win different axes (stock vs swap),
   record both winners per axis and say so in NOTES.md. Archive every
   version you formally evaluate.

## Logging

Append to `WALLCLOCK.md`: session start/end, per-version timestamps.
Everything you learn that is not already in LAWS.md goes to NOTES.md.
