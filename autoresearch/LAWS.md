# The Laws Library

Execution knowledge distilled from ~120 instrumented iteration runs across
nine LIBERO tasks. Every law below was purchased with a diagnosed failure;
the one-line evidence is the receipt. Programs that follow all of these
routinely reach 4/5 probe within a handful of versions; programs that
violate one usually fail in the documented way.

## Program anatomy (all winning programs converged to this)

- **Perception layer (WHERE)**: ground every object reference at runtime
  from the robot's RGB-D (`api.capture("cam_high")` → depth + intrinsics +
  extrinsics → world point cloud). Never trust bddl regions or hard-coded
  coordinates: LIBERO-PRO swaps teleport AND rotate fixtures while the
  bddl text stays unchanged.
- **Execution layer (HOW)**: all contact-rich motion is the demo's RAW
  action stream, retargeted but never paraphrased. Paraphrase ceiling,
  measured: described motions 0/20, native replay 20/20 on the same task.
- **Verification layer (GATES)**: read proprioception against typed
  signatures — gripper gap < 3 mm after a close is AIR; gap pinned at the
  open width is a rigid OBSTRUCTION; anything between is a HOLD.

## Execution laws

1. **Seamlessness.** No hand-authored motion inside a chain of demo
   segments, and no `api.move()` hops into mid-demo waypoints: the OSC
   integrates deltas differently from configurations the demo never
   visited. Evidence: an api.move entry left every close firing 11 cm
   above the box; a synthesized wrist twist saturated the joint and killed
   all downstream rotation deltas.
2. **Relativity (motion).** Corrections to a HELD grasp enter as relative
   deltas, never absolute re-alignment to recorded coordinates — working
   grasps ride a consistent controller drift and re-alignment destroys it
   (measured 3/5 → 0/5).
3. **Seam alignment (free space only).** Official inits randomize the ARM
   start pose too; open-loop replay amplifies a 2-3 cm start offset into a
   7 cm miss. At free-space seams, closed-loop align to (demo waypoint +
   grounding offset) with raw clipped deltas, THEN hand over to raw
   replay. Absolute alignment is legal only when nothing is held.
4. **Closed-loop bridges.** To carry a grounding offset mid-chain, walk
   `target = current_eef + Δ` with raw deltas clipped to the action range
   (`step = clip(err/0.05 * 0.6, -1, 1)`). Never open-loop with a guessed
   gain — a guessed ×14 saturated the clip and swung a phase 8 cm wide.
5. **Segment boundaries come from the demo's gripper events** (sign flips
   in `actions[:, -1]`); phases are [approach..close], [carry..open].
   Replay through close+8 to latch, lift with the demo's own next ~18
   steps before bridging.

## Perception laws

6. **Same-instrument anchoring.** Demo-side constants (object anchors)
   MUST be measured by the exact grounding code the program runs, on the
   demo's own init (`env.set_init_state(hdf5 states[0])`) — never taken
   from simulator body origins (a cheese box's body frame sat 1.5 cm off
   its geometric centre) and never from a different grounding variant
   (changing the instrument on one end re-introduces the bias; this alone
   cost three versions in one task).
7. **Top-disk law (cylinders/pots/cans).** A cluster median carries a
   grazing-angle wall bias that CHANGES as the object moves relative to
   the camera — differencing cannot cancel it. The top disk's median is
   the cylinder axis at any position. Slice `z ∈ top±~1.5 cm`, crop 5 cm
   around the prior, median.
8. **Mouth-disk law (bowls/containers).** Rim points carry 2-3 cm wall
   bias; the interior floor seen through the opening is a symmetric disk
   (1-2 mm accuracy). Beware: no fixed z-band separates "table" from a
   near-table bowl floor — qualify mouths by the RING of the cluster's own
   high points enclosing the low disk in all 8 bearings, not by height.
9. **Height-sliced grounding.** Ground flat objects (boxes ≤ ~4 cm) in a
   low slice tall neighbours' TOPS can't enter — but tall objects' lower
   WALLS enter every slice, so chain-merging is still possible: keep a
   merge-immune fallback (prior-window crop + shrinking-window median
   re-centre, 5 cm → 3.5 cm). Cluster median is mm-true when the object
   stands alone; prefer cluster primary, crop fallback.
10. **Grid clustering merges anything within one pitch** (2 cm default),
    and merged clusters elect wrong centres. No pitch fixes touching
    objects — that's what the fallback is for.
11. **Priors mislead under layout perturbation.** The demo position is a
    disambiguation prior ONLY; under swap suites it actively lies. Shape
    evidence must outrank it (extent, aspect, ring/mouth counts). Programs
    grounded in shape survive fixture teleports untouched; prior-anchored
    layers collapse to 0.

## Controller & environment facts

12. The position controller settles ~1 cm high; command descents deeper
    than the target and verify before committing.
13. `obs` images refresh ONLY on `env.step` — capture after stepping, or
    you diagnose stale pixels (cost two debugging sessions).
14. Rim/edge pinches are knife-edged (a bowl wall is 4.5 mm): identical
    pinch geometry can flip outcomes. Let the pinch arbitrate itself: on
    AIR, micro-search ±7 mm and one deeper bite, gap decides.
15. `ee_path6` in packs is strided; use the pack's `stride` field. Never
    infer by floor division (9-vs-10 became a 12 cm overshoot).
16. Sim wall-clock ≈ 25-30 macro-steps/s; a 250-action demo replays in
    ~10 s. Budget probe time accordingly.

21. **Closed-loop strokes for pushes.** A demo push stroke has fixed
    length; varied layouts change the required travel. Never one-shot a
    push: measure → replay the stroke → re-ground the object → repeat
    until it reaches the target region (push family scored 2/20 open-loop
    with the plate moving partway in EVERY episode).
22. **Re-ground before retrying a grasp.** A retry aimed at the same
    stale coordinate repeats the same miss; recapture and re-ground the
    target first (the scene may also have been nudged by the miss).
23. **Knife-edge comparisons need repeated probes.** MuJoCo+EGL is
    non-deterministic enough to flip a byte-identical program's episode;
    a single 5-episode probe cannot rank micro-search geometries — use
    >=3 repeats or don't conclude. Also: wider first-sweep rings can
    BUMP the object before the real attempt; search outward from small.

## Protocol laws

17. **Probe on SPREAD init indices** (`--episode-list 0,7,14,3,11`), never
    the first k — the first five official inits under-represent the layout
    distribution (a 5/5 probe collapsed to 3/20 at scale once).
18. **Two-tier evaluation**: 3-5 episode probes for diagnosis (read traces
    and films, not rates); every banked claim is 20 episodes on the
    official init file. A probe/banked disagreement is itself a finding.
19. **Decisive controls beat hypothesis-chasing.** The fastest debugging
    move is a control experiment that bisects the cause space: run the
    program on the demo's own init (grounding vs chain), null out the
    bridges (chain structure vs offsets), diff two instrument readings.
    One control per version; never stack two fixes in one version.
20. **Ops traps**: ssh compound commands lose cwd after `&` (use launcher
    scripts or absolute paths); `pgrep/pkill -f` matches its own pattern
    (split it: `"fewshot""_run"`); AbakaAI downloads need the proxy
    (`socks5h://127.0.0.1:7891`); ALL cluster data under
    `/mnt/data/YifanKang/` (root disk is 99% full).

26. **Translated-replay offsets have a scale ceiling.** LAW #2's "raw
    deltas preserve their offset" holds at the few-cm scale it was
    measured at (single-demo grounding corrections). Cross-donor splices
    (TASK-axis re-authoring) need 10-30cm transits — an order of
    magnitude larger — and at that scale translated raw-tail replay
    accumulates error; use closed-loop `_align`-only transits (LAWS
    #3/#4) instead, even while holding a grasp (align at safe hover
    height per the spa_next_to_the_cookie_box release-exception note).
    Receipt: 3/3 members of task_goal_c (cc_on_rack 20/20 after
    switching, wine_in_bowl_a/b 15/20).

24. **Closed-loop placement: servo the carried object, not the hand.**
    Terminal placement under a long retarget must re-ground the TARGET at
    hover height and servo until the CARRIED OBJECT (not the eef) is
    centred over it, then descend; orientation corrections happen at hover
    height too. Receipt: wine_top swap 0/20 -> 20/20 (v3), wine_rack
    0/20 -> 15/20 (opus_goal_a).
25. **The wrist camera grounds a held object that fixed cameras cannot.**
    A held vertical object self-occludes under top/front cameras (cluster
    pins to the gripper). cam_arm_wrist looks down the gripper axis; a
    crop-shrink median of the band just below the fingertips
    [eef_z-0.06, eef_z-0.02] grounds the held object to ~10mm. Only helps
    where the placement can absorb an eef offset (flat tops, not tight
    slots). Receipt: same runs as #22.

**Known gap (no law-legal fix yet):** tall-object set-down toppling —
replaying a donor's raw open+retreat calibrated for a squat object tips
tall ones (wine_on_plate 5/20, second occurrence in task_goal_b). A
controlled orientation-aware set-down would require hand-authored motion
inside a demo chain, which LAW #1 currently forbids. Flagged for a
future mechanism, not silently patched.

## Reward (judge-only)

Write `reward.py: score(trace) -> [0,1]` as 2-3 weighted subgoal terms,
each `clip(measured/calibrated, 0, 1)`; calibrate constants from the demos
(a drawer's demonstrated travel, a knob's final angle). Progress terms take
the trace max; end-state terms take the last frame. The reward navigates
zero-success stretches and gates distillation rollouts — it never trains
the policy. Make it strictly stricter than the benchmark bit.
