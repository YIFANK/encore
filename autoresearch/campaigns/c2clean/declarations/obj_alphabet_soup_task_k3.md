# c2clean / obj_alphabet_soup_task_k3 — "Pick the cream cheese and place it in the basket"

No shared note file. Everything below comes from the two named packs
(`pack.json` + `keyframes/` only) and my own debug-seed (51–65) captures.

## Evidence gathered before writing any motion

**Who is the cream cheese?**
- `c2clean_obj_alphabet_soup_task_mate/pack.json` language =
  "pick up the cream cheese and place it in the basket". Its demo0 keyframe
  `t0000` shows six props; `t0120` (arm carrying something to the basket)
  shows the same six minus one **small blue box**. Pack-difference names the
  target: cream cheese = the small blue box.
- `c2clean_obj_alphabet_soup_task_k3/pack.json` language = "pick up the
  alphabet soup …"; its `t0000` frames show *my* scene (bottle, two cans, a
  lying red box, a small brown box, a small blue box, basket). Cropping the
  blue-box patch out of both packs' `t0000` frames gives the same asset, so
  the cream cheese is present in the scene I am run in.
- Neither pack's scene layout matches mine (mate grasps at y ≈ −0.115, my box
  sits at y ≈ +0.06), so every position is perceived at run time; only the
  *grasp height* and *release height* transfer from the mate pack.

**Scene geometry (v0 probe, all 15 debug seeds).** A no-motion program that
streamed a downsampled cam_high RGB-D out through `api.log` (zlib+base64,
1800-char chunks) and was decoded offline. Findings:
- table plane deprojects to z ≈ −0.001; `t_base_cam` = camera at
  (0.897, 0, 0.65) looking down the −x/−z diagonal, so image-u ≈ base +y.
- prop tops: 0.017–0.018 (small brown box, small blue box), 0.079 (can),
  0.136–0.146 (bottle, tall can, basket rim); robot body above 0.24.
- the blue box's top face deprojects to a constant z within 0.002 m, spans
  0.080 m in x by 0.041 m in y, and is the **only** low cluster with
  B > max(R,G) (the other low cluster is red-brown, rgb ≈ 80,50,37).
- seed-to-seed variation is small: the box centre moves ~6 mm, the basket
  centre ~16 mm across 51–65. Both are still perceived per episode.

## Version chain

| ver | hypothesis | evidence | verdict |
|-----|-----------|----------|---------|
| v0 | *(not a policy)* stream cam_high RGB-D out of the sandbox so the scene can be read offline | `results/fs_…_v0`, 0/15 (no motion, as intended); yielded the height map, the colour signature and the camera extrinsics above | probe succeeded |
| v1 | height-band + colour identification of the blue box, straddle the **short** (y) axis with the home straight-down wrist, close at the mate pack's own grasp height (eef z = 0.010), carry at z = 0.25, release at z = 0.185 over the perceived basket centre | probe `results/fs_…_v1` **8/8** (51,53,…,65); formal `results/sel_…_v1` **15/15** | **frozen** |

**Margins on v1** (identical across every logged seed): descent residual
0.0111, closed gripper width 0.0422 m with effort 3.0 held from the close
through the lift, the carry and the drop; release width 0.0533. The closed
width equals the box's measured y-thickness (0.041 m), i.e. the jaws are on
the two long faces, not on a corner. No seed needed a yaw-90 grasp
(`xspan` 0.079–0.080 > `yspan` 0.041–0.042 everywhere).

## DECLARATION

- **Frozen version:** `program_v1.py`, copied to
  `packs/c2clean_obj_alphabet_soup_task_k3/program.py`.
  `md5 = f32372092cec0e54a1613b7da3c2f9f3` — identical for both files on the
  cluster.
- **Selection receipt (full 15 debug seeds):** **15/15**, directory
  `results/sel_c2clean_obj_alphabet_soup_task_k3_v1`.
- **Receipt chain:** v0 `results/fs_c2clean_obj_alphabet_soup_task_k3_v0`
  (perception probe, 0/15 by construction) → v1 probe
  `results/fs_c2clean_obj_alphabet_soup_task_k3_v1` 8/8 →
  v1 formal `results/sel_c2clean_obj_alphabet_soup_task_k3_v1` 15/15.
- **PROVENANCE:** present in `program.py` as a top-level literal dict covering
  GRASP_Z, R_DOWN, Z_BAND, WORKSPACE, TOP_PLANE_TOL, LOW_TOP_MAX, CARRY_Z,
  RELEASE_Z, HOVER_Z, OPEN_W, CLOSE_W — each sourced to the mate/k3 pack or to
  a debug-seed measurement listed above.
- Splits respected: only seeds 51–65 were ever run, always `--split debug`,
  always under `tools/fair_run.py`. No forbidden file was opened.

STOP.
