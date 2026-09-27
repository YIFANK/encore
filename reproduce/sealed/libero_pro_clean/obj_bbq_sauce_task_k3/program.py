"""c2clean obj_bbq_sauce_task_k3 -- "Pick the ketchup and place it in the basket".

Mechanism
---------
The mate pack (`c2clean_obj_bbq_sauce_task_mate`) demonstrates the OBJECT this
intent names: three demos whose close-command keyframes sit at ee z ~0.113 on a
bottle whose crop (projecting the demo EEF into the 128px keyframe with the
debug-seed camera matrices) shows a grey/silver screw cap over a red-orange
body.  The k3 pack (bbq sauce) grasps a plain amber bottle with a dark cap, and
releases -- like the mate pack -- over a basket at y ~ +0.24.

So: find the upright bottle whose top band is achromatic (the silver cap),
pinch its neck 0.035 m below its top (the mate-pack grasp offset), carry it
over the perceived basket and open.
"""
import numpy as np

PROVENANCE = {
    "GRASP_BELOW_TOP": {
        "source": "mate pack keyframes: close-command ee z = 0.1130/0.1134 "
                  "(demo1 t49, demo2 t42) on the ketchup; debug-seed depth "
                  "gives that bottle top z = 0.148 -> 0.035 m below the top",
        "allowed": True},
    "R_DOWN": {
        "source": "debug-seed api.tool_rotation() at reset "
                  "([[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]]) rounded to "
                  "the exact straight-down tool frame",
        "allowed": True},
    "CAP_BAND_M": {
        "source": "debug-seed depth: the ketchup cap disc spans the top ~0.012 m "
                  "and is the only band whose x extent (0.026) matches its y "
                  "extent (0.030), i.e. the only unbiased centre estimate",
        "allowed": True},
    "TOP_MIN / TOP_MAX": {
        "source": "debug-seed depth on seeds 51-65: candidate props measure "
                  "top z 0.020 (box) / 0.081 (can) / 0.113 (amber bottle) / "
                  "0.143 (basket) / 0.148 (silver-cap bottle, lying green "
                  "bottle); the [0.12,0.22] window keeps the tall bottles",
        "allowed": True},
    "EXT_MAX": {
        "source": "debug-seed depth: basket footprint 0.16x0.17 m and the lying "
                  "green bottle 0.28 m long, vs <=0.07 m for the upright "
                  "bottles",
        "allowed": True},
    "Z_FLOOR / WORKSPACE": {
        "source": "debug-seed depth: table plane deprojects to z ~0.001 and the "
                  "back wall to x ~ -1.99, so props are cropped to |y|<0.45, "
                  "-0.40<x<0.40, z>0.012",
        "allowed": True},
    "CARRY_Z": {
        "source": "debug-seed depth: basket rim top z = 0.143; carry at 0.28 "
                  "clears it with the 0.113 m of bottle hanging below the eef "
                  "(bottle top 0.148 - grasp 0.035 = base 0.113 below the eef)",
        "allowed": True},
    "RELEASE_Z": {
        "source": "mate pack keyframes: open-command ee z = 0.1697 (demo1 t148) "
                  "/ 0.2124 (demo2 t120) / 0.1768 (demo0 t229) over the basket",
        "allowed": True},
    "GRIP_OPEN_M / GRIP_CLOSE_M": {
        "source": "FairApi mechanics (grip(<0.025) closes, else opens); "
                  "debug-seed api.gripper() at reset reads width 0.0778 open",
        "allowed": True},
}

R_DOWN = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
GRASP_BELOW_TOP = 0.035
CAP_BAND_M = 0.012
TOP_MIN, TOP_MAX = 0.12, 0.22
EXT_MAX = 0.12
Z_FLOOR = 0.012
CARRY_Z = 0.28
RELEASE_Z = 0.20
GRIP_OPEN_M = 0.08
GRIP_CLOSE_M = 0.0


# ---------------------------------------------------------------- perception
def _cloud(f):
    rgb = np.asarray(f.rgb, np.uint8)
    d = np.asarray(f.depth, np.float32)
    K = np.asarray(f.intrinsics, float)
    T = np.asarray(f.t_base_cam, float)
    h, w = d.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    x = (uu - K[0, 2]) / K[0, 0] * d
    y = (vv - K[1, 2]) / K[1, 1] * d
    P = np.stack([x, y, d], -1) @ T[:3, :3].T + T[:3, 3]
    return P, rgb


def _components(mask):
    """4-connected labelling, numpy-only."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    idx = np.argwhere(mask)
    cur = 0
    for sy, sx in idx:
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(int(sy), int(sx))]
        lab[sy, sx] = cur
        while stack:
            y, x = stack.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def _blobs(P, rgb, min_px=60):
    m = ((P[..., 2] > Z_FLOOR) & (P[..., 2] < 0.40)
         & (P[..., 0] > -0.40) & (P[..., 0] < 0.40) & (np.abs(P[..., 1]) < 0.45))
    lab, n = _components(m)
    out = []
    for i in range(1, n + 1):
        sel = lab == i
        npx = int(sel.sum())
        if npx < min_px:
            continue
        pts = P[sel]
        cols = rgb[sel].astype(float)
        top = float(pts[:, 2].max())
        hi = pts[:, 2] > top - CAP_BAND_M
        q = pts[hi]
        cap = cols[hi].mean(0)
        out.append(dict(
            n=npx, top=top,
            ext=(float(np.ptp(pts[:, 0])), float(np.ptp(pts[:, 1]))),
            cap_xy=(float((q[:, 0].min() + q[:, 0].max()) / 2),
                    float((q[:, 1].min() + q[:, 1].max()) / 2)),
            cap_rgb=[int(c) for c in cap],
            chroma=float(cap.max() - cap.min()),
            xy=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
            pts=pts))
    return out


def _pick(blobs):
    cands = [b for b in blobs
             if TOP_MIN <= b["top"] <= TOP_MAX and max(b["ext"]) <= EXT_MAX
             and b["n"] >= 100]
    if not cands:
        return None
    cands.sort(key=lambda b: b["chroma"])
    return cands[0]


def _basket(blobs):
    big = [b for b in blobs if max(b["ext"]) > EXT_MAX]
    if not big:
        return None
    b = max(big, key=lambda b: b["n"])
    rim = b["pts"][b["pts"][:, 2] > b["top"] - 0.03]
    return (float((rim[:, 0].min() + rim[:, 0].max()) / 2),
            float((rim[:, 1].min() + rim[:, 1].max()) / 2),
            b["top"])


# ------------------------------------------------------------------- program
def run(api):
    api.log("INSTR %s" % api.instruction())
    f = api.capture("cam_high")
    P, rgb = _cloud(f)
    blobs = _blobs(P, rgb)
    for b in blobs:
        api.log("BLOB n%d top%.4f ext%s xy%s cap_xy%s cap%s chroma%.1f"
                % (b["n"], b["top"], np.round(b["ext"], 3).tolist(),
                   np.round(b["xy"], 4).tolist(), np.round(b["cap_xy"], 4).tolist(),
                   b["cap_rgb"], b["chroma"]))
    tgt = _pick(blobs)
    bsk = _basket(blobs)
    if tgt is None or bsk is None:
        api.log("ABORT tgt=%s bsk=%s" % (tgt is not None, bsk is not None))
        return
    gx, gy = tgt["cap_xy"]
    gz = tgt["top"] - GRASP_BELOW_TOP
    api.log("TARGET grasp=%.4f,%.4f,%.4f top=%.4f cap=%s chroma=%.1f"
            % (gx, gy, gz, tgt["top"], tgt["cap_rgb"], tgt["chroma"]))
    api.log("BASKET %s" % (np.round(bsk, 4).tolist(),))

    api.grip(GRIP_OPEN_M)
    r = api.move([gx, gy, tgt["top"] + 0.10], rotation=R_DOWN, seconds=3.0)
    api.log("HOVER res=%s eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    r = api.move([gx, gy, gz], rotation=R_DOWN, seconds=2.0)
    api.log("DESCEND res=%s eef=%s" % (r, np.round(api.eef(), 4).tolist()))
    api.grip(GRIP_CLOSE_M)
    api.settle(0.5)
    api.log("CLOSED %s" % api.gripper())

    r = api.move([gx, gy, CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.log("LIFT res=%s eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([bsk[0], bsk[1], CARRY_Z], rotation=R_DOWN, seconds=3.0)
    api.log("OVER res=%s eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    r = api.move([bsk[0], bsk[1], RELEASE_Z], rotation=R_DOWN, seconds=2.0)
    api.log("DOWN res=%s eef=%s grip=%s"
            % (r, np.round(api.eef(), 4).tolist(), api.gripper()))
    api.grip(GRIP_OPEN_M)
    api.settle(1.0)
    api.log("RELEASED %s" % api.gripper())
    api.move([bsk[0], bsk[1], CARRY_Z], rotation=R_DOWN, seconds=2.0)
    api.settle(0.5)

    f2 = api.capture("cam_high")
    P2, rgb2 = _cloud(f2)
    for b in _blobs(P2, rgb2):
        api.log("POST n%d top%.4f ext%s xy%s cap%s chroma%.1f"
                % (b["n"], b["top"], np.round(b["ext"], 3).tolist(),
                   np.round(b["xy"], 4).tolist(), b["cap_rgb"], b["chroma"]))
