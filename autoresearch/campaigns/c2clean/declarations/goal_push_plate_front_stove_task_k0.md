# c2clean — goal_push_plate_front_stove_task_k0

Intent: **"Push the cream cheese to the front of the stove"** (no demonstration pack).
Runner: `tools/fair_run.py` only. Debug seeds 51-65.

## Scene, derived only from debug-seed cam_high RGB-D

Dumped both cameras' RGB-D through `api.log` (zlib+base64, no sim steps spent on
perception) on seeds 51,53,...,65 and deprojected offline.

| thing | where (base frame) | how it was identified |
|---|---|---|
| table top | z = 0.9009 ± 0.0001 | dominant depth plane |
| cream cheese | 0.078 x 0.040 footprint, top z = **0.920**; centre x -0.06..-0.03, y 0.11..0.15 | only blue-dominant (B > R+8), dark (mean<120) prop in the 0.911-0.945 height band |
| plate | disc r~0.067 at (0.05, 0.00), top 0.920 | same height band but whitish (146,135,133) |
| bowl | (-0.10, 0.00), rim top 0.952 | |
| wine bottle | (-0.19, -0.05), top 1.058 | |
| stove slab | x[-0.45,-0.16], y[0.11,0.30], top ~0.95 | height band 0.913-0.945 with x<-0.14, y>0.05 |
| cabinet + rack | y < -0.12 | |

Target = **(stove front edge x + 0.060, stove y centre)** ~ (-0.11, 0.21),
i.e. the table patch directly in front of the stove. Both terms are measured
per episode from the same frame.

## Version chain

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump RGB-D through api.log and do perception offline | 8/8 seeds decoded; api.log truncates at ~2000 chars, so chunks are 1900 | scene solved |
| v2 | find the fingertip/table contact height with a closed gripper | descents floor at **EEF z = 0.9102** at (0.15,0.25); table plane 0.9009 -> tips sit 0.010 below the EEF ref. Episode cap is 1000 sim steps and a *stalled* move burns ~100 of them (9 moves = 1000) | calibration |
| v3 | L-push (+y then -x) with Z_PUSH commanded 0.945 | 0/4. PERC2 identical to PERC: the box never moved. Converged EEF z was 0.943-0.952, so the tips rode ~2 cm **over** the 0.920 box top. Converged moves are cheap (~25 steps), so the budget is not the constraint | refuted |
| v4 | same push with Z_PUSH = 0.915 (tips ~0.905, inside the box's 0.900-0.920 span) | **4/4** (51,53,55,57), 82-129 sim steps: the episode terminates during **PUSH1**, so the +y push alone puts the cream cheese in the goal region | mechanism found |
| v5 | v4 + gripper pixels excluded from the colour mask (v4's PERC2 read the box as 8 cm deep because the closed jaws are dark and blue), park-before-relook, and a corrective +y push | **8/8** on 51,53,...,65. Six seeds ended inside PUSH1; seed 61 needed PUSH2+PUSH3 (713 steps) because its 0.070 m approach point sits inside the bowl, and the corrective chain recovered it | selected |

Mechanism in one line: the cream cheese is a 2 cm slab, so a **closed gripper with
its fingertips at z~0.905 slides it**; nothing needs to be grasped or lifted, and
the goal region is reached by the +y push alone.

Failure mode still present (v5 seed 61, recovered): the descent point 0.070 m on
the -y side of the box can land inside the bowl, so the first push shoves the bowl
instead. The relook + push-x + corrective-push-y chain fixes it at the cost of
~600 extra sim steps, well inside the 1000-step cap.

## DECLARATION

- **Frozen version: v5.** `packs/c2clean_goal_push_plate_front_stove_task_k0/program.py`
  md5 `6e86fafac7f08f565e97af3899c7c34d` == `program_v5.py` (identical on the
  cluster and locally).
- **Selection receipt: 15/15** on the full debug split 51-65,
  `results/sel_c2clean_goal_push_plate_front_stove_task_k0_v5`
  (every episode `"benchmark_success": true`; 79-713 sim steps of the 1000 cap).
- Per-version receipt chain: v1 perception dump (8/8 seeds decoded) ->
  v2 contact calibration (EEF floor 0.9102) -> v3 0/4 (push height too high,
  box unmoved) -> v4 4/4 on 51,53,55,57 -> v5 8/8 on 51,53,...,65 -> v5 15/15
  formal. Archived: `program_v1.py` ... `program_v5.py`.
- `PROVENANCE` present in program.py: 11 constants, every one sourced to a
  debug-seed measurement, a prior-version receipt, or generic gripper mechanics.
  No pack was given and none was read; nothing outside this worker directory,
  its own pack dir and its own `results/*c2clean_goal_push_plate_front_stove_task_k0*`
  dirs was written or read.
