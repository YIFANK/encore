"""All Gemini Robotics-ER prompt templates.

Conventions baked in from Pigey's hard lessons:
- Grounding queries are PURELY VISUAL phrases (color/shape/size), never task
  nouns — task-biased detection hallucinates the target onto lookalikes.
- Points come back [y, x] normalized 0-1000 (ER convention), parsed leniently.
- Plans and repairs are schema-constrained JSON, one decision per call; no
  accumulating chat state.
"""
from __future__ import annotations

# Both detectors are asked for EVERY matching instance rather than the single
# best one. Scenes contain lookalikes, and a detector that silently commits to
# one of them hands the caller a confident answer about the wrong object; with
# the alternatives visible, the caller can pick the one consistent with where it
# already believes the object to be.
POINT_PROMPT = (
    'Point to the {query}. Answer as a JSON list: '
    '[{{"point": [y, x], "label": "<label>", "confidence": <0..1>}}]. '
    "Points are [y, x] normalized to 0-1000. "
    "If several objects match the description, return EVERY one of them, best first "
    "(at most 4). Return [] if none is visible."
)

# One request for both. They were two: a POINT call and a BOX call, on the same
# image, about the same object, moments apart — 23 detection round trips for a
# two-object task, 127 s of a 207 s episode. The model has to look at the frame
# either way; asking for both costs one look instead of two.
#
# The box may be omitted. On the lab's wide overhead frame ER-2 points at
# everything reliably and boxes almost nothing, so a point with a null box is
# the normal answer there, not a failure.
LOCATE_PROMPT = (
    'Locate the {query}. Answer as a JSON list: '
    '[{{"label": "<label>", "point": [y, x], '
    '"box": [<ymin>, <xmin>, <ymax>, <xmax>] or null, "confidence": <0..1>}}]. '
    "All coordinates normalized to 0-1000; points are [y, x]. "
    "Always give the point. Give the box only if you are confident of the "
    "object's full extent; use null otherwise. "
    "If several objects match the description, return EVERY one of them, best "
    "first (at most 4). Return [] if none is visible."
)

# Everything at once. Six objects were costing six calls because the code asked
# about one query string at a time, while the model looks at the whole frame
# either way — the second question re-reads the same image to answer about a
# different corner of it.
LOCATE_MANY_PROMPT = (
    "Locate each of these in the image:\n{queries}\n\n"
    'Answer as a JSON list, one entry per object you find: '
    '[{{"query": "<the exact query text from the list above>", '
    '"point": [y, x], "box": [<ymin>, <xmin>, <ymax>, <xmax>] or null, '
    '"confidence": <0..1>}}]. '
    "All coordinates normalized to 0-1000; points are [y, x]. "
    "Always give the point. Give the box only if you are confident of the "
    "object's full extent; use null otherwise. "
    "Copy each query string EXACTLY as written so the answers can be matched up. "
    "Omit any object you cannot see rather than guessing. If several objects "
    "match one query, return the best one for that query."
)

BOX_PROMPT = (
    'Give the 2D bounding box of the {query}. Answer as a JSON list: '
    '[{{"label": "<label>", "y": <ymin>, "x": <xmin>, "y2": <ymax>, "x2": <xmax>, "confidence": <0..1>}}]. '
    "Coordinates normalized to 0-1000. "
    "If several objects match the description, return EVERY one of them, best first "
    "(at most 4). Return [] if none is visible."
)

# "Which of these is it?" in one request, instead of "is this it?" once per
# candidate. The model reads every image it is given either way, so this costs
# one call where the loop cost up to six — and it asks a better question, since
# comparing the candidates against each other is exactly what a yes/no about
# each one in isolation cannot do.
#
# Zero has to be sayable. A search that cannot come back empty would bind the
# entity to whichever region looked least unlike it.
CHOOSE_PROMPT = (
    "You are shown {n} cropped images, numbered 1 to {n} in the order given. "
    "Each is centred on one object.\n"
    "Which single image shows {query}?\n"
    'Answer as JSON: {{"choice": <image number, or 0 if none of them shows it>, '
    '"confidence": <0..1>, "reason": "<15 words max>"}}. '
    "Answer 0 rather than guessing when none of them is a good match."
)

VQA_PROMPT = (
    "{question}\n"
    'Answer as JSON: {{"answer": "yes"|"no"|"unsure", "confidence": <0..1>, "reason": "<15 words max>"}}. '
    'Say "unsure" when the image genuinely cannot settle the question — do not guess.'
)

PLANNER_SYSTEM = """You are the task planner of {embodiment} \
Workspace: x in {wx}, y in {wy}.

You compile the user's instruction into a TASK PROGRAM: declared entities, an ordered list of \
skill steps with pre/postconditions, and goal predicates that define success. An executor will run \
it, VERIFY every postcondition against sensors, and come back for repairs when reality disagrees — \
so make assumptions explicit as preconditions instead of hoping.

QUOTE, DON'T PARAPHRASE: for every object the instruction names, the entity
description MUST contain the instruction's own words for it (e.g. instruction
"pick the akita black bowl" -> description "the akita black bowl", optionally
plus visual detail). The description is the detector's search query; a visual
paraphrase ("the metal bowl with a gold rim") discards the one phrase the task
guaranteed to identify the object.

PREDICATE VOCABULARY (the only predicates the executor can verify):
- holding(entity, arm)       arm is "left" or "right"
- gripper_empty(arm)
- visible(entity)
- in(entity, container)
- on(entity, support)
- near(entity, target)  # beside/at: within the target's footprint plus ~6 cm. Use for
                        # push/move-toward goals ("push X to the front of Y" -> near(X, Y))
                        # A table REGION is a declarable entity like any object — the
                        # grounder can point at "the lower-left corner of the table" or
                        # "the front edge" — so "move X to the lower-left corner" is
                        # entity lower_left_corner + goal near(X, lower_left_corner).
                        # NEVER reduce a move-to-place goal to gripper_empty.
- open(entity)              a drawer or panel pulled out of the thing it sits in
- all_in(entity, container) EVERY instance of the entity's class is in the container \
                            (class-level; `in` is existential). The loop predicate for \
                            "all"/"every" tasks.
Any of these may appear negated (negated=true).

CONTROL FLOW. A step may carry `when`: a guard predicate — the step runs only \
if the guard holds, and is SKIPPED (not failed) otherwise. Use it for \
conditional repairs the scene may or may not need: "if the socket is occupied, \
clear it first" is a clear step guarded by in(blocker, socket). The program may \
carry `repeat_until`: a predicate verified after the body completes; if \
unsatisfied the body re-runs. Use it for "every"/"all" tasks — "move all red \
blocks to the tray" is ONE entity `red_block` ("the red block", relation \
inside(grey_tray) negated=true so each pass grounds one still OUTSIDE), a \
perceive-pick-place body, and repeat_until all_in(red_block, grey_tray) — \
never one pasted copy per visible block: enumeration breaks the moment a new \
instance appears mid-task, which is exactly what "all" promises to survive. Do not use `when` in place of a precondition: a precondition \
failing is a fault to repair, a guard failing just means the step is unneeded.

An articulated task ("open the middle drawer") is expressed with open(...), and needs BOTH the drawer front and the cabinet declared as entities — the direction to pull is derived from where the handle sits relative to the cabinet body. Do not reduce such a goal to gripper_empty or to a pick: a plan whose predicates cannot express its goal verifies something else and reports success.

AVAILABLE SKILLS:
{skills}

RULES
1. Entity descriptions must be PURELY VISUAL phrases (color + shape + size, e.g. "the small red \
cube"), never task roles ("the target"). Declare every object you will refer to, including \
containers and obstacles.
1b. When the instruction tells objects apart by WHERE they are rather than by how they look \
("the black bowl BETWEEN the plate and the ramekin", "the bowl NEXT TO the cookie box", "the bowl \
ON the stove"), do NOT invent a visual difference — the objects genuinely look identical, and a \
guess binds the whole task to the wrong one. Keep `description` as the plain appearance shared by \
all of them ("the black bowl") and put the position in `relation`:
   {{"kind": "between"|"near"|"far_from"|"on"|"inside", "of": [<entity ids>], "negated": <bool>}}
   - "between" takes exactly two anchors; the others take exactly one.
   - Anchors must themselves be declared entities, and the perceive step must list them.
   - Keep the description GENERIC — the wording all the lookalikes share ("the black bowl"), \
not one of them ("the metal bowl with a gold rim"). A description that fits only one object \
finds only one object, and then there is nothing for the relation to choose between.
   - NEGATION IS PART OF THE SENTENCE. Read the instruction for "not", "other", "away from", \
"apart from" and set negated=true. Worked example — "pick the black bowl NOT between the plate \
and the ramekin" is
       {{"id": "bowl", "description": "the black bowl",
         "relation": {{"kind": "between", "of": ["plate", "ramekin"], "negated": true}}}}
     Dropping that flag selects the bowl the instruction excluded, and every later step then \
succeeds on the wrong object.
   The executor locates every matching object, then picks the one the relation names. Omit \
`relation` when appearance alone is enough.
2. Start with a perceive step over all entities. Never assume locations.
3. A pick step needs pre [visible(entity), gripper_empty(arm-you-chose or omit arm)] and post \
[holding(entity, arm)]. A place step needs pre [holding(entity, arm)] and post [in(...) or on(...), \
plus negated holding].
4. Prefer scripted skills (pick/place/push/open_drawer/close_drawer). Drawers and panels use \
open_drawer, not pick and not vla. A push step names its destination: push(entity, toward=Y) \
measures the direction at execution and ends beside Y — never guess dx/dy numbers — with post \
[near(entity, Y)]. Use the vla skill only for deformables, non-grasp verbs with no \
scripted equivalent, or as recovery — and always give vla steps explicit postconditions.
4b. NEVER use vla for any of these, measured failure modes of the policy behind it:
   - negation ("not between", "not next to", "the other one") — it drops the "not" and acts on the \
wrong instance;
   - telling apart identical objects by position — that is what `relation` is for;
   - pushing or sliding — it does not do it;
   - articulated verbs (open/close/turn on) — measured 0 of 4 on the policy's own benchmark suite.
   A failed vla rollout does not merely fail: it moves things, so the scene the scripted recovery \
needs is gone. Use vla for plain pick-and-place of a uniquely-named object, or not at all.
5. Decide container clearing from the instruction's end state: most tasks are additive (leave \
existing contents); remove contents FIRST only if the end state forbids them or they physically \
block placement.
6. "an X / any X" = exactly one, choose the easiest; "all/every X" = loop over each; when ambiguous \
prefer the fewer-action reading.
7. Plans should be minimal — the executor inserts recovery steps itself when things fail.
8. LESSONS FROM PREVIOUS EPISODES:
{hints}

Respond ONLY with the JSON program."""

PLANNER_USER = """INSTRUCTION: {instruction}

The attached images are the current camera views ({cameras}). Compile the task program."""

REPAIR_SYSTEM = """You are the repair module of a bimanual stationary robot executing a task \
program. A step failed; deterministic diagnosis already classified the failure. Propose ONE typed \
edit to the program — the smallest change that makes progress.

EDIT TYPES
- retry_step: re-run the failed step, optionally with new_args merged over its args.
- insert_steps: add recovery steps (given in "steps") immediately before the failed step, then the \
failed step runs again.
- replace_step: swap the failed step for the step(s) in "steps" (e.g. scripted pick -> vla).
- skip_step: the step is unnecessary (its effect already holds).
- reground_entity: the entity description was wrong; supply entity_id + new_description (visual \
phrase).
- replace_tail: replan everything from the failed step onward; "steps" is the new tail.
- abort: the goal is unreachable — explain in rationale.

GUIDANCE BY FAILURE CLASS
- skill_effect after repeated retries: replace_step with different parameters or a different arm. \
The vla skill is a LAST resort and is FORBIDDEN for articulated verbs (open/close/turn on), pushing, \
negation, and telling identical objects apart by position — the policy behind it was measured at 0 \
of 4 on exactly those, and a failed rollout does not merely fail, it moves things, so the scene the \
scripted recovery needed is gone.
- perception: usually reground_entity with a better visual description.
- state_drift: insert perceive steps, or replace_tail if the change invalidates the approach.
- program: replace_tail with a corrected decomposition.
- safety: change the motion's parameters/placement so it stays inside the workspace.

HARD REQUIREMENTS
- step_id is MANDATORY for retry_step / insert_steps / replace_step / skip_step / \
replace_tail — copy the failing step's id from the failure report (field "step_id").
- Steps you introduce must use fresh ids, skills from the catalog, and the predicate \
vocabulary (holding/gripper_empty/visible/in/on).
- Keep edits local; do not rewrite steps that already succeeded.

AVAILABLE SKILLS:
{skills}

Respond ONLY with the JSON edit."""

REPAIR_USER = """PROGRAM (current state):
{program}

BELIEF:
{belief}

FAILURE REPORT:
{failure}

Attached: current camera view(s). Propose the single best edit."""


# ONE CALL FOR THE WHOLE EPISODE'S PERCEPTION AND ITS GOAL.
#
# Measured on the rig 2026-08-06: 54 model calls in one episode, of which two
# did the useful work and the rest were the same question in a different voice.
# The waste is structural, not a matter of tuning: the entity list is decided
# before the image is read, so every name has to be located separately, and the
# goal is translated in a third call that cannot see what the first two found.
#
# Gemini Robotics-ER 1.5 answers all of it at once — detect everything, name it
# so no two names collide, and translate the instruction into predicates over
# those names. Two identical trays become white_tray_left and white_tray_right
# because the model looking at the picture is the thing best placed to tell
# them apart; asking "where is the white plate" when there are two is a question
# with no right answer, and we have been paying for it all day.
SURVEY_PROMPT = (
    "You are the perception front end of a tabletop robot.\n\n"
    'The operator said: "{instruction}"\n\n'
    "From this image, do BOTH of the following in one answer.\n\n"
    "1. List every distinct manipulable object on the table. Give each one a "
    "unique snake_case id. WHEN TWO OBJECTS WOULD OTHERWISE SHARE AN ID, "
    "distinguish them by what you can see — position (white_plate_left, "
    "white_plate_right), size (large_white_plate, small_white_plate), or "
    "material — never by a number alone. Skip the robot arms, the cameras, the "
    "table, calibration boards, and anything not on the table surface.\n\n"
    "2. Translate the instruction into goal predicates over the ids you just "
    "gave. Use only: on(entity, support), in(entity, container), "
    "near(entity, target), holding(entity, arm), gripper_empty(arm), "
    "open(entity), clear(entity). Negate with a leading 'not '.\n\n"
    'Answer as JSON: {{"objects": [{{"id": "<snake_case>", '
    '"description": "<short noun phrase a person would use>", '
    '"point": [y, x], "box": [<ymin>, <xmin>, <ymax>, <xmax>] or null, '
    '"confidence": <0..1>}}], '
    '"goal": ["on(a, b)", ...], '
    '"note": "<one sentence: anything ambiguous about the instruction>"}}\n\n'
    "All coordinates normalized to 0-1000; points are [y, x]. Always give the "
    "point; give the box only when you are sure of the full extent. If the "
    "instruction names something you cannot see, say so in note and leave it "
    "out of objects rather than guessing at a location."
)
