# l90abl — LIBERO-90 articulated-fixture modality ablation: results (2026-09-09)

24 cells (8 tasks × k3 / vis / k0), one draw each. Blind coordinator-run sealed
evals on seeds 1-50, one eval per declared frozen program, md5 recorded at
launch (`eval_freeze.txt`), 50 unique episodes per cell, no program modified
after its eval. All 24 workers declared normally; no outage this time.
Pre-registration: PREREGISTERED.md.

## Headline (sealed, /50)

| group | task | k3 | vis | k0 | k3−vis | k3−k0 |
|---|---|---|---|---|---|---|
| open | open_microwave | 50 | 50 | 50 | 0 | 0 |
| open | open_top_drawer_s1 | 50 | 50 | 50 | 0 | 0 |
| open | open_top_drawer_s2 | 50 | 50 | 50 | 0 | 0 |
| open | **open_bottom_drawer** | **50** | **0** | **0** | **+50** | **+50** |
| control | close_microwave | 50 | 50 | 50 | 0 | 0 |
| control | close_bottom_drawer | 50 | 50 | 42 | 0 | +8 |
| control | close_top_drawer | 50 | 50 | 50 | 0 | 0 |
| control | turn_on_stove | 50 | 50 | 50 | 0 | 0 |

Development-band selection scores matched the sealed picture cell for cell
(every declared 15/15 went 50/50; open_bottom_drawer vis/k0 declared 0/15).

## Verdict on the pre-registered predictions

1. *Open group: k3 > vis ≈ k0 by ≥15 in ≥3/4 cells* — **refuted as a group
   claim** (1 of 4). The one cell that separates does so by the full 50, with
   vis = k0 = 0, exactly the LIBERO-PRO drawer pattern.
2. *Control group: all arms within 10 in ≥3/4 cells* — **supported** (4/4; the
   largest control gap is close_bottom_drawer k0 at −8).
3. *If k0 solves the open group, the drawer effect is fixture-specific* —
   **supported, and sharpened**: it is specific to a fixture *class* that two
   measurements identify, not to "drawers" or to "opening".

## The mechanism, from the agents' own notes

**Where every arm succeeded, the handle could be pinched.** Both top drawers
were opened by a pinch in every arm ("grasp, don't hook" — open_top_drawer_s2
vis). On the microwave a pinch fails, but it fails *loudly*: `open_microwave_k0`
tried four pinch geometries and each "stalled at 40-45° with the finger gap
prised open 15-25 mm before it went empty" — the door leaf rotates the jaw axis
into the pull and friction lets go. That signal let it diagnose "a hinged door
defeats a pinch; hook it" and reach an open-jaw straddle with the rear finger
bearing on the bar's back face by v12 (14/15 dev, 50/50 sealed). The vis arm
solved it by a pinch that follows the door's arc.

**On the bottom drawer the pinch is geometrically impossible and fails
silently.** The bar stands 0.032 m proud of the cabinet face; the jaw's open
half-width is 0.039 m; the arm cannot bring the jaw axis parallel to the face
without a horizontal wrist that its ground clearance forbids below z = 1.000.
A pinch therefore closes on air (width 0.002) without ever touching the bar —
and closing on air says nothing about whether a hook would work. Both no-action
arms wrote confident, arithmetically argued, falsifiable laws that were wrong:
vis — "no pose in this controller's repertoire grasps the bottom bar"; k0 — "a
handle that protrudes from a wall can be its own reach boundary" (it read the
bar's front face as the arm's envelope). The k3 arm read from the pack that
"this task is never a grasp; every demo has the same shape", used an open-jaw
press-hook, and still had to learn one more thing the demo did not say
outright: engage on a *blocked* descent, not a converged one — a move that
converges to the demo's own z lands one bar-thickness outside the opening and
the pull carries no load (v1 0/4 → 15/15).

## The rule, across the three modality campaigns

* abl_noact (LIBERO-PRO, 60 cells): images + language carry the demonstration
  effect; the action channel decided 1-2 cells (drawer hook; plate closing height).
* rsabl (robosuite Door, Wipe): action channel no gain on Door (K=0 found the
  latch turn — a held, locked door is a loud failure), a cost on Wipe (the
  demonstrated path shape was a decoy under the move budget).
* l90abl (LIBERO-90 fixtures): 1 of 8 fixtures separates, by 50 episodes.

**The action channel is load-bearing when (a) the manipulation the fixture
requires cannot be reached by the strategy family the agent defaults to
(here: bar protrusion < jaw half-width forbids a pinch), and (b) the default
strategy's failure is silent in the proximal signal (closing on air).** Where
(a) fails, search finds the same configuration; where (a) holds but (b) fails
(microwave: pinch grips then prises open), search diagnoses its way to the
right type. Both conditions are measurable on a new fixture before running
anything, which makes the claim predictive rather than post hoc.

## Limits
One draw per arm; a single separating fixture (n = 1 for the effect within this
suite, though it replicates the LIBERO-PRO drawer cell); stock tasks with their
own language, human LIBERO-90 demos.
