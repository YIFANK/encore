# NOTES — l90abl / open_bottom_drawer_k0 (zero-demo)

Intent: "open the bottom drawer of the cabinet" (KITCHEN_SCENE1).
No demonstration pack. Everything below is derived from debug seeds 51-65 via
cam_high RGB-D, eef/gripper telemetry and my own logs/gifs.

## Scene, as measured (stable across all 15 debug seeds)
- Table plane z = 0.901.
- Cabinet: top face z = 1.1275, x-extent [-0.133, 0.118], front face y = -0.2325,
  body extends to y < -0.38. It faces +y, so its drawers open toward +y.
- Three handle bars on the front face, bar top-surfaces at z = 0.9553 / 1.0245 /
  1.0980. The **bottom** drawer is the lowest, z_top = 0.9553.
- Bottom bar geometry (yz occupancy map, x-slice [-0.02,0.03]): the bar occupies
  y in [-0.2164, -0.2014] and is ~0.015 m in section; its x-span is
  [-0.05, +0.046] with a standoff at each end. The slot between the bar's back
  and the drawer face is **15 mm** wide (-0.2325 to -0.2175).
- Per-seed jitter: the whole cabinet shifts up to ~19 mm in y across seeds
  (y_front ranges -0.1824 .. -0.2014 over seeds 51-65). Perception is therefore
  done per episode, never hardcoded.
- A bowl sits near the table centre (~(0, 0)); it lies on the straight-line path
  from home to the cabinet.

## Version log (hypothesis -> evidence -> verdict)

| v | hypothesis | evidence | verdict |
|---|---|---|---|
| v1 | dump RGB-D, locate the cabinet | table z=0.901; cabinet + 3 handles found at left | perception works |
| v2 | is the scene seed-stable? | 5 seeds: identical z bands, cabinet shifts ~10 mm in y | per-seed perception needed |
| v3 | horizontal wrist, straddle bar top/bottom, pull | burned all 1000 steps in 2 moves | **L1**: blocked moves cost `seconds*200` |
| v4 | how expensive is a converged move? | 8 moves = 108 steps, resid 0.010 | budget model established |
| v5 | calibrate fingertip offset; horizontal wrist | L_tip = 0.008-0.010; horizontal pose unreachable (roterr large, eef flew to y=-0.40) | fully horizontal wrist rejected |
| v6 | scan descent vs y beside the handle | every descent stalls at z=1.095, open *and* closed | not the fingers |
| v7 | is it the cabinet or a reach limit? | descent free at y=0.00 (z=0.937), blocked at y=-0.17/-0.19; gif shows the **forearm lying on the cabinet top** | **L4** |
| v8 | descend in free space first, then translate in | holds z=0.95, creeps to y=-0.133 and stops (same open/closed) | wrist/arm, not fingers |
| v9 | does yaw or pitch tuck the wrist? | yaw90 -> -0.176, pitch40 -> -0.181 vs -0.133 upright | orientation buys reach |
| v10 | sweep 10 orientations | **pitch 55 is the argmax: y=-0.2026**; yaw180/-45 unstable | **L3** |
| v11 | pitch-55 grasp + pull | grabbed the **bowl**, dragged it 0.20 m at effort 3.0, handle unmoved | **L5** |
| v12 | stage high over the tableware, then descend | bowl avoided; jaw stalls 15 mm in front of the bar; close catches nothing | approach fixed, reach still short |
| v13 | is the stall a collision or the reach envelope? | x=0.25 unreachable, run wasted; upright creep far from the cabinet also stalls (~-0.157) | envelope is real |
| v14 | can a different path reach deeper? | z=1.20 reaches y=-0.2534 easily but descent stops at z=1.163 (cabinet top) | reach improves with height only |
| v15 | saturate with repeated moves | p55 -> -0.2027 (hard), p35 -> -0.1646 | saturation is sharp |
| v16 | approach from the cabinet's open +x side | x=0.20/z=0.93 cannot hold orientation, reaches only y=-0.09 | +x side rejected |
| v17 | lower finger skims the table, upper finger hooks | **finger A reaches y=-0.2230, behind the bar's back (-0.2164)** but 30 mm above the bar top | height, not depth, is the blocker |
| v18 | trade z for y on the envelope | commanded z 0.900..0.965 all settle at z=0.955-0.959 | z cannot be lowered near the cabinet |
| v19 | mirror pitch (tool tilted away, finger hangs down-back) | arm cannot hold z (ends 1.09-1.17), reaches only y=-0.10 | rejected |
| v20 | is the stall a bounded-controller equilibrium? | commanded y=-0.26 / -0.60 / -1.50 all give eef y=-0.194; hard push to z=0.80 does not lower z | **L2**: hard boundary |
| v21 | best-aimed full pipeline, per-seed perception | see selection receipt below | **frozen (argmax)** |
| v22 | creep in above the bar, then descend behind it | at 45 mm above the bar the creep is *shallower* (jaw y=-0.1849); descending pushes it back out | rejected |
| v23 | creep in below the bar, then lift behind it | commanded jaw z 0.912/0.925 both settle at jaw z=0.952; jaw again at bar_front+0.002 | rejected |

## Selection receipt (formal, full 15 debug seeds)
`results/sel_l90abl_open_bottom_drawer_k0_v21` — **0/15 benchmark_success**.

The per-seed telemetry is unusually uniform and is the core evidence:

| quantity | across seeds 51-65 |
|---|---|
| handle y_front (perceived) | -0.1824 .. -0.2014 (19 mm spread) |
| `bar_along_tz` at the stall | **+0.0098 .. +0.0100** on all 15 |
| `bar_along_ty` at the stall | -0.0009 .. -0.0012 on all 15 |
| jaw z - bar top | -0.0009 .. -0.0023 |
| gripper after close | width 0.0013-0.0016, effort 0.05 (nothing captured) |
| handle dy after the pull | -0.0014 .. -0.0017 (unmoved; residual is the gripper entering the mask) |

## MECHANISM-GAP STOP

**The gripper reaches the handle and touches it, but no reachable pose puts the
bar inside the jaw.** At the deepest reachable pose the jaw centre sits
0.2 - 2.3 mm below the bar's top surface and 1-2 mm in front of the bar's front
face, i.e. resting against the bar — it is ~10 mm short along the approach axis
of enclosing it, on every seed and at every commanded height.

### Candidate law (falsifiable)
> **A handle that protrudes from a wall can be its own reach boundary.** Near
> the cabinet the arm's reachable set terminates *on the handle bar's front
> surface* rather than at a fixed point in space. The proof that this is the
> handle and not the arm's envelope: when the cabinet shifts between seeds, the
> stall point shifts with it by the same amount, holding
> `bar_along_tz = +0.0099 +/- 0.0001` across all 15 seeds and a 19 mm spread of
> handle positions. The geometry is a scissor: the wrist pitch that buys the
> lateral reach (55 deg, worth +70 mm over a straight-down wrist) simultaneously
> raises the deep finger 27 mm above the eef, so the finger that *can* get
> behind the bar (y=-0.2230, past the bar back at -0.2164) is always ~30 mm too
> high, and the pitch that lowers it (<=40 deg) loses the reach that got it
> there. Commanded z between 0.912 and 0.965 changes nothing: the eef settles at
> z=0.955-0.960 near the cabinet regardless.

### How to falsify / what would break it
1. Any pose reaching `bar_along_tz <= 0` — the bar at or behind the fingertip
   plane — would refute it directly and should immediately yield a grasp.
2. If `api.grip` accepted an intermediate width, a half-open jaw at pitch 55
   would place the deep finger ~13 mm lower and could clear the bar top; the
   API is binary (`<0.025` closes, else opens), so this is untestable here.
3. A null-space or joint-configuration handle on the controller (elbow on the
   -y side rather than +y) would move the boundary; FairApi exposes no such
   control, and no waypoint sequence I tried (over the cabinet top, from the +x
   side, from a high-left pose) changed it.

### Not the blocker (ruled out with receipts)
- Perception: the handle is found on all 15 seeds, and its measured position
  tracks the cabinet's per-seed jitter.
- The tableware snag (v11) — fixed in v12 by staging high, and absent since.
- Controller laziness (v20) and step budget (v4/L1).
- The forearm-on-cabinet-top block (v7/L4) — avoided by the low front approach.

## Deliverable / DECLARATION
- **Frozen version: v21.** `packs/l90abl_open_bottom_drawer_k0/program.py`
  md5 `395fc54add7e7ab77181563e1ddf24e5` == `program_v21.py` (verified on the
  cluster).
- **Selection receipt: 0/15** on the full debug split,
  `results/sel_l90abl_open_bottom_drawer_k0_v21`.
- Per-version receipt chain: `results/fs_l90abl_open_bottom_drawer_k0_v{1..23}`.
- `PROVENANCE` present in program.py, covering all 13 calibrated constants; every
  source is a debug-seed measurement or generic controller/camera mechanics.
- Archived versions: `program_v1.py` .. `program_v23.py` in the pack dir.
- Status: **mechanism-blocked**, documented above. v21 is declared as the argmax
  (all versions scored 0; v21 is the cleanest full pipeline with per-seed
  perception, PROVENANCE and self-verification).
