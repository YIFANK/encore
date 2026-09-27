# l90abl / open_bottom_drawer_k3 — worker notes

Cell: LIBERO-90 `KITCHEN_SCENE1_open_the_bottom_drawer_of_the_cabinet`,
arm **k3** (full K=3 pack).  Runner: `tools/fair_run.py` only.
Debug seeds 51-65; eval band 1-50 never touched.

## What the pack says

3 demos, 124/156/136 steps, `gripper_cmd = -1.0` (open) for every step of every
demo — this task is never a grasp.  Every demo has the same shape:

1. start at home `(-0.19..-0.21, 0.00, 1.16..1.18)`, wrist straight down;
2. rotate ~90 deg while translating to `y ~ -0.20`, `z ~ 0.96..0.98`;
3. hold that attitude and translate `+y` by 0.164 / 0.174 / 0.192 m at constant z.

Pre-pull keyframes: `(0.0179,-0.2023,0.9703)`, `(-0.0197,-0.2074,0.9574)`,
`(0.0490,-0.2020,0.9792)`; rotations `(1.4781,1.6816,-0.5879)`,
`(1.5061,1.6424,-0.6047)`, `(1.5296,1.4241,-0.6380)`.  `y` and `z` agree to a
few mm across demos, `x` scatters 7 cm — the contact is a bar the hand can meet
anywhere along its axis.

`demo1` t=115 has `gripper_state = [0.0241,-0.0405]` (gap 0.0646 against 0.0763
free): one finger is deflected 12 mm at the moment of engagement.  That is the
pack's only load signal, and it is what the runtime `api.gripper().width_m`
reproduces when the pull is actually loaded.

## Version chain

| ver | hypothesis | receipt | verdict |
|-----|-----------|---------|---------|
| v1 | replay the demo-mean engage pose and pull | `fs_..._v1`, seeds 51,53,55,57 — **0/4**, every move converged, 157 sim steps, gripper width flat at 0.0796 | refuted: the pose is reachable but nothing is engaged |
| v2 | contact probe: drive -y at two heights until blocked | `fs_..._v2`, seeds 51,53 — blocked at eef `y=-0.2144` for `z=0.9886`, at `y=-0.1831` for `z=0.9550` | the obstacle is 31 mm further out at the lower height: a bar protruding from the face |
| v3 | press to the face high, drop 19 mm as demo0 does (`z` 0.9886 -> 0.9700), pull | `fs_..._v3`, seeds 51..65 odd — **0/8**, drop ran free to 0.9746, pull free, width flat | refuted: 0.970 is still above the bar |
| v4 | same, but keep the -y command pressed and drop **past** the bar; sweep `z` in (0.960, 0.950, 0.940) with a re-perception check between attempts | `fs_..._v4` seeds 51,53 — **2/2**; `fs_..._v4p` seeds 51..65 odd — **8/8**, all on the FIRST depth (0.9600), 198-199 sim steps | accepted |

### Measured scene (debug seeds, agentview depth + contact)

The fixture is identical on every debug seed inspected (51,53 dumps agree to
1 cm, and v1/v3/v4 spend an identical number of sim steps on every seed) — only
the tabletop props are resampled.

* cabinet front face `y = -0.230`, `x` in `[-0.05, 0.11]`, `z` in `[0.90, 1.13]`
* three handle bars, axis along base `x`, front edge `y = -0.200`, at
  `z = 1.10 / 1.02 / 0.95`; the bottom drawer is the `z = 0.95` bar
* pressing -y stops at `y=-0.2144` when the tool clears the bar (`z >= 0.975`)
  and at `y=-0.1831` when it does not (`z = 0.955`) — a 2 cm effective tool
  offset along the tool `z` axis reconciles both with the two surfaces above

### Why v3 failed and v4 works

Both press the tool against the drawer face above the bar.  v3 then commanded
`z = 0.9700` and the descent *converged* there, leaving the tool above the bar,
so the pull slid off with no load (width 0.0798 throughout).  v4 commands
`z = 0.9600`, which the descent cannot reach: it is blocked at `z = 0.9725`
with the tool wedged in the opening between the bar and the drawer face.  The
pull is then loaded — it stops 44 mm short of its target (`y=-0.0709` for a
target of `-0.0271`) and the finger gap falls from 0.0805 to 0.0742, the same
deflection the pack shows at demo1 t=115 — and the drawer comes out.

The load-bearing detail is that the *unconverged* descent is what engages: a
converged move to the demo's own recorded `z` leaves the tool one bar-thickness
too high, and nothing in the proximal signal says so until the descent is
commanded past the obstacle and the residual reports the block.

## Candidate law (for LAWS.md)

**A hooked pull engages on a blocked descent, not a converged one.**  When a
demo's engage pose sits inside a narrow opening (bar vs face), commanding
exactly the demo's `z` converges just outside it and the pull carries no load.
Command past the opening and read the *residual* as the receipt that the tool is
wedged; the finger gap moving off its free value is the second receipt.

## DECLARATION

* Frozen version: **v4**.  `packs/l90abl_open_bottom_drawer_k3/program.py`
  md5 `fb51e553348067231d086d27669c4d3b` == `program_v4.py` (same md5).
* PROVENANCE: present, 8 entries, every calibrated constant sourced to the pack
  or to a debug-seed measurement.
* Probe receipt: `results/fs_l90abl_open_bottom_drawer_k3_v4p` — **8/8** on
  seeds 51,53,55,57,59,61,63,65.
* Selection receipt (full 15 debug seeds):
  `results/sel_l90abl_open_bottom_drawer_k3_v4` — **15/15** on seeds 51-65,
  every one on the first drop depth (0.9600), 198-199 sim steps each.
* Archived versions in the pack dir: `program_v1.py` .. `program_v4.py`.
* Not mechanism-blocked; no gap stop needed.
