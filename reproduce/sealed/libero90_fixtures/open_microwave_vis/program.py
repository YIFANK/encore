import os
import numpy as np

PROVENANCE = {
    "BODY_Z_LO=0.93 / BODY_Z_HI=1.12": {
        "source": "debug-seed cam_high depth (v0 probe, seeds 51/53/57/61): table plane z=0.901, microwave top slab z=1.108-1.118; the band brackets the on-table structure",
        "allowed": True},
    "Z_TABLE=0.901": {
        "source": "debug-seed cam_high depth: modal plane of the table surface",
        "allowed": True},
    "BAR_R=0.009": {
        "source": "debug-seed measurement: handle-bar silhouette spans ~0.018 m in y (5-95 pct), so radius 0.009; used to step from the visible skin to the bar axis along the camera view direction",
        "allowed": True},
    "R_X": {
        "source": "generic controller/gripper mechanics: tool-z straight down, finger closing axis along base +x (chosen so no finger enters the door plane at y=yface)",
        "allowed": True},
    "R_Y": {
        "source": "generic controller mechanics: same, finger axis along base +y (defined but unused)",
        "allowed": True},
    "yface = 92nd percentile of body y": {
        "source": "debug-seed measurement: the microwave y-histogram is flat to y~-0.14 and then drops by >10x; the handle is the residue beyond that edge",
        "allowed": True},
    "handle band = middle 30-70 pct of the handle z-span": {
        "source": "debug-seed measurement: the C-handle's two stubs sit at the ends of the z-span (v1 showed the top stub is grabbable but slides out under a +y pull); the middle is the free vertical bar",
        "allowed": True},
    "grasp z candidates zmid+0.02, zmid-0.02, zmid+0.06": {
        "source": "debug-seed measurement: bar spans z 0.951-1.092; v1 close-sweep bracketed the fingertip offset to <0.038 m, so zmid+0.02 puts the fingers on the bar middle (v2: 4/4 grasps, w=0.0227, effort 3.0)",
        "allowed": True},
    "hinge = (box.xhi, box.yface)": {
        "source": "debug-seed measurement: in v2 the gripped handle drifted +x as it was pulled +y, which is the sign a +x-edge hinge predicts and the opposite of a -x-edge hinge; the door plane is the yface edge",
        "allowed": True},
    "DPHI=3 deg": {
        "source": "debug-seed tuning: v4/v5 step size; 3 deg at R~0.24 is a ~12 mm command step, below the observed steady-state tracking error",
        "allowed": True},
    "MAXLEAD=10 deg": {
        "source": "debug-seed measurement: v4 let the commanded point run 40+ mm ahead of a stalled eef, which became a radial pull and pried the jaws from 0.0227 to 0.0403 m; capping the lead at 10 deg (~40 mm of arc) keeps the command tangential",
        "allowed": True},
    "radius clamp [R0-0.03, R0]": {
        "source": "debug-seed measurement: the bar slides at most a few mm in the jaws, so the grasp radius only ever shrinks; clamping stops the servo from pulling outward",
        "allowed": True},
    "NSTEP=60 / lost>=3 stop": {
        "source": "debug-seed measurement: the door stalls by ~78 deg of rotation (26 steps) and v5 used 365-544 sim steps of a 900 s / 1000-step episode; 60 steps is slack, and 3 consecutive zero-effort steps is the observed signature of the bar leaving the jaws",
        "allowed": True},
    "OUT": {
        "source": "own logging path under results/", "allowed": True},
}

Z_TABLE = 0.901
BODY_Z_LO = 0.93
BODY_Z_HI = 1.12
BAR_R = 0.009
R_X = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # fingers close along base x
R_Y = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # fingers close along base y


def cloud(f):
    D = f.depth
    K = np.asarray(f.intrinsics)
    T = np.asarray(f.t_base_cam)
    H, W = D.shape
    v, u = np.mgrid[0:H, 0:W]
    x = (u - K[0, 2]) / K[0, 0] * D
    y = (v - K[1, 2]) / K[1, 1] * D
    return np.stack([x, y, D], -1) @ T[:3, :3].T + T[:3, 3]


def perceive(api, log=True):
    f = api.capture("cam_high")
    P = cloud(f)
    view = np.asarray(f.t_base_cam)[:3, 2]
    body = ((P[..., 2] > BODY_Z_LO) & (P[..., 2] < BODY_Z_HI) &
            (P[..., 1] > -0.45) & (P[..., 1] < 0.0) &
            (P[..., 0] > -0.5) & (P[..., 0] < 0.4))
    pts = P[body]
    yface = float(np.percentile(pts[:, 1], 92))
    hm = body & (P[..., 1] > yface + 0.005)
    hp = P[hm]
    zlo, zhi = float(np.percentile(hp[:, 2], 2)), float(np.percentile(hp[:, 2], 98))
    mid = hm & (P[..., 2] > zlo + 0.30 * (zhi - zlo)) & (P[..., 2] < zlo + 0.70 * (zhi - zlo))
    surf = np.median(P[mid], axis=0)
    ctr = surf - BAR_R * view
    box = dict(xlo=float(np.percentile(pts[:, 0], 0.5)),
               xhi=float(np.percentile(pts[:, 0], 99.5)),
               yback=float(np.percentile(pts[:, 1], 0.5)),
               yface=yface,
               ztop=float(np.percentile(pts[:, 2], 99.5)),
               zlo=zlo, zhi=zhi)
    if log:
        api.log("perceive bar=%s box=%s" % (np.round(ctr, 4).tolist(),
                                            {k: round(v, 3) for k, v in box.items()}))
    return ctr, box, f


OUT = "/mnt/data/YifanKang/Heron/results/sel_l90abl_open_microwave_vis_v6/dbg"
DPHI = np.deg2rad(3.0)
MAXLEAD = np.deg2rad(10.0)
NSTEP = 60


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def grasp_handle(api, ctr, box):
    bx, by = float(ctr[0]), float(ctr[1])
    zmid = 0.5 * (box["zlo"] + box["zhi"])
    api.grip(0.08)
    api.settle(0.2)
    api.move([bx, by, box["ztop"] + 0.07], rotation=R_X, seconds=2.0)
    for zt in [zmid + 0.02, zmid - 0.02, zmid + 0.06]:
        api.move([bx, by, zt], rotation=R_X, seconds=2.0)
        api.grip(0.0)
        api.settle(0.3)
        g = api.gripper()
        api.log("grasp try z=%.3f eef=%s w=%.4f eff=%.2f"
                % (zt, np.round(api.eef(), 4).tolist(), g["width_m"], g["effort"]))
        if g["effort"] > 2.0 and g["width_m"] > 0.004:
            return True
        api.grip(0.08)
        api.settle(0.2)
    return False


def run(api):
    os.makedirs(OUT, exist_ok=True)
    tag = "u%d" % np.random.randint(10 ** 6)
    api.log("tag=%s" % tag)
    ctr, box, f0 = perceive(api)
    got = grasp_handle(api, ctr, box)
    api.log("GOT=%s" % got)

    hx, hy = box["xhi"], box["yface"]
    e0 = api.eef()
    r0 = np.array([e0[0] - hx, e0[1] - hy])
    R0 = float(np.linalg.norm(r0))
    phi0 = float(np.arctan2(r0[1], r0[0]))
    zk = float(e0[2])
    api.log("hinge=(%.4f,%.4f) R0=%.4f phi0=%.1f z=%.4f" % (hx, hy, R0, np.rad2deg(phi0), zk))

    phi_des = phi0
    trace = []
    lost = 0
    for k in range(1, NSTEP + 1):
        phi_des -= DPHI
        e = api.eef()
        r = np.array([e[0] - hx, e[1] - hy])
        phi_cur = float(np.arctan2(r[1], r[0]))
        Rc = float(np.clip(np.linalg.norm(r), R0 - 0.03, R0))
        phi_cmd = max(phi_des, phi_cur - MAXLEAD)   # bounded tangential lead
        tgt = [hx + Rc * np.cos(phi_cmd), hy + Rc * np.sin(phi_cmd), zk]
        api.move(tgt, rotation=rz(phi_cur - phi0) @ R_X, seconds=1.0)
        e2 = api.eef()
        g = api.gripper()
        r2 = np.array([e2[0] - hx, e2[1] - hy])
        ang = np.rad2deg(phi0 - np.arctan2(r2[1], r2[0]))
        trace.append([float(e2[0]), float(e2[1]), float(e2[2]), float(g["width_m"]),
                      float(g["effort"]), float(ang)])
        if k % 5 == 0 or k < 4 or g["effort"] < 2.0:
            api.log("arc %02d des=%.0f cmd=%.0f door=%.1f eef=%s w=%.4f eff=%.2f"
                    % (k, np.rad2deg(phi0 - phi_des), np.rad2deg(phi0 - phi_cmd), ang,
                       np.round(e2, 4).tolist(), g["width_m"], g["effort"]))
        if g["effort"] < 2.0:
            lost += 1
            if lost >= 3:
                api.log("grip lost at door=%.1f, stopping arc" % ang)
                break
        else:
            lost = 0

    f1 = api.capture("cam_high")
    ctr2, box2, _ = perceive(api)
    api.log("AFTER bar=%s box=%s" % (np.round(ctr2, 4).tolist(),
                                     {k: round(v, 3) for k, v in box2.items()}))
    np.savez_compressed(os.path.join(OUT, "%s.npz" % tag),
                        rgb0=f0.rgb, depth0=f0.depth, K=f0.intrinsics, T=f0.t_base_cam,
                        rgb1=f1.rgb, depth1=f1.depth, trace=np.asarray(trace, dtype=float))
    api.log("done")
