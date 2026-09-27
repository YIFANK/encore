"""c2clean obj_chocolate_pudding_task_k3 -- 'Pick the salad dressing and place it in the basket'.

Perceive the table from cam_high, identify the dressing bottle by the greenness of its
top band, grasp its cap, carry it over the basket and release. Every stage is checked
with the program's own sensors (gripper width/effort, re-perception); a failed close is
retried along a ladder of x aims that spans the measured aim window, and a bottle left
outside the basket triggers a fresh pick-place cycle.
"""
import numpy as np

PROVENANCE = {
    "R_DOWN": {
        "source": "generic controller mechanics: tool-to-world matrix for a straight-down "
                  "wrist; matches the debug-seed 51 start pose api.tool_rotation() "
                  "[[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]] rounded to the exact axes",
        "allowed": True,
    },
    "TABLE_Z": {
        "source": "debug seeds 51/53/55/57 cam_high depth deprojected to base frame: the "
                  "dominant height bin is z in 0.001..0.009 m (207k of 262k pixels)",
        "allowed": True,
    },
    "WORKSPACE": {
        "source": "debug seeds 51-57 cam_high: all table props deproject inside "
                  "x in [-0.25,0.25], y in [-0.32,0.40]",
        "allowed": True,
    },
    "PROP_ZMAX": {
        "source": "debug seeds 51-57: the tallest table prop tops out at z=0.148 m while the "
                  "robot arm occupies z>0.20 m; 0.19 separates them",
        "allowed": True,
    },
    "ARM_EXCLUDE_R": {
        "source": "debug seed 51: at the start pose api.eef()=[-0.1485,0,0.2613] the gripper "
                  "footprint spans x[-0.167,-0.127] y[-0.098,+0.092]; 0.085 m covers it",
        "allowed": True,
    },
    "CELL": {
        "source": "generic: 5 mm top-down height-map cell, ~2x the cam_high ground sampling "
                  "at table range (depth 0.74-2.79 m, f=618 px)",
        "allowed": True,
    },
    "CAP_BAND": {
        "source": "debug seed 51 height-band profile of the dressing bottle: the green cap "
                  "spans z 0.110..0.148 with y-width 0.035 while the body below z=0.08 is "
                  "0.060-0.063 wide; the top 25 mm is pure cap",
        "allowed": True,
    },
    "GRASP_DZ": {
        "source": "debug seed 51 band profile (cap top z=0.148, cap cylinder 0.110..0.148) "
                  "plus mate-pack demos 0/1/2 which close 0.020-0.031 below their bottle top; "
                  "debug-seed envelope probes dz=+0.015 and dz=-0.015 both scored 4/4",
        "allowed": True,
    },
    "X_AIM_LADDER": {
        "source": "debug-seed aim-envelope sweep on seeds 51,55,59,63 (4 episodes per offset) "
                  "of the commanded grasp x relative to the perceived cap centre: "
                  "-0.014:0/4 -0.010:0/4 -0.007:0/4 -0.004:4/4 0.000:8/8 +0.006:4/4 "
                  "+0.010:4/4 +0.012:0/4 +0.014:0/4 -> pass band centre +0.003, "
                  "half-width ~0.008. Ladder walks that band outward from its centre.",
        "allowed": True,
    },
    "Y_AIM_OK": {
        "source": "debug-seed envelope probes dy=+0.020 and dy=-0.020 both scored 4/4, so y "
                  "needs no correction",
        "allowed": True,
    },
    "CARRY_Z": {
        "source": "mate pack ee_path6 demos 0/1/2 carry at z=0.29-0.31; k3 pack demos carry "
                  "at z>=0.28. Tallest debug-seed prop top is 0.148",
        "allowed": True,
    },
    "RELEASE_DZ": {
        "source": "k3 pack demos release at z=0.164-0.176 and mate pack at z=0.176-0.205, "
                  "with the debug-seed basket rim measured at z=0.144",
        "allowed": True,
    },
    "BASKET_MIN_CELLS": {
        "source": "debug seeds 51-65: the basket footprint is 492-505 height-map cells, every "
                  "other prop is <=145",
        "allowed": True,
    },
    "HOLD_WIDTH": {
        "source": "debug seed 51 v1 run: a good close on the 0.035-wide cap reads width "
                  "0.0368 with effort 3.0; an air close (envelope probe dx=+0.020) reads "
                  "width 0.0010 with effort 0.05",
        "allowed": True,
    },
    "OBS_POSE": {
        "source": "debug seed 51: the start pose is [-0.1485,0,0.2613]; lifting it to z=0.40 "
                  "puts the whole hand above PROP_ZMAX, and x=-0.15 is behind every prop "
                  "along the cam_high optical axis (-0.849,0,-0.529), so nothing is occluded",
        "allowed": True,
    },
    "MOVE_BUDGET": {
        "source": "debug-seed v1 runs used 199-200 sim steps for 6 moves plus grips; the "
                  "budget caps retries well inside the episode horizon",
        "allowed": True,
    },
}

R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
TABLE_Z = 0.006
WORKSPACE = (-0.25, 0.25, -0.32, 0.40)
PROP_ZMAX = 0.19
ARM_EXCLUDE_R = 0.085
CELL = 0.005
CAP_BAND = 0.025
GRASP_DZ = 0.022
X_AIM_LADDER = (0.003, 0.009, -0.003, 0.012, -0.006)
CARRY_Z = 0.31
RELEASE_DZ = 0.055
BASKET_MIN_CELLS = 300
HOLD_WIDTH_MIN = 0.005
HOLD_WIDTH_MAX = 0.055
OBS_POSE = (-0.15, 0.0, 0.40)
MOVE_BUDGET = 26


# ----------------------------------------------------------------- perception
def _cloud(api, cam="cam_high"):
    f = api.capture(cam)
    d = np.nan_to_num(np.asarray(f.depth, dtype=np.float64), nan=0.0,
                      posinf=0.0, neginf=0.0)
    K, T = np.asarray(f.intrinsics, float), np.asarray(f.t_base_cam, float)
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - K[0, 2]) / K[0, 0] * d, (v - K[1, 2]) / K[1, 1] * d, d], -1)
    P = pc @ T[:3, :3].T + T[:3, 3]
    return P, np.asarray(f.rgb, dtype=np.float64) / 255.0


def _heightmap(P, rgb, eef_xy):
    x0, x1, y0, y1 = WORKSPACE
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    m = ((z > TABLE_Z + 0.012) & (z < PROP_ZMAX) &
         (x > x0) & (x < x1) & (y > y0) & (y < y1))
    if eef_xy is not None:
        m &= (np.abs(x - eef_xy[0]) > ARM_EXCLUDE_R) | (np.abs(y - eef_xy[1]) > ARM_EXCLUDE_R)
    nx, ny = int((x1 - x0) / CELL) + 1, int((y1 - y0) / CELL) + 1
    Hm = np.zeros((nx, ny))
    Cm = np.zeros((nx, ny, 3))
    px, py, pz, pc = x[m], y[m], z[m], rgb[m]
    ix = ((px - x0) / CELL).astype(int).clip(0, nx - 1)
    iy = ((py - y0) / CELL).astype(int).clip(0, ny - 1)
    o = np.argsort(pz)
    Hm[ix[o], iy[o]] = pz[o]
    Cm[ix[o], iy[o]] = pc[o]
    return Hm, Cm


def _components(Hm):
    m = Hm > 0
    nx, ny = Hm.shape
    seen = np.zeros_like(m)
    out = []
    for s in np.argwhere(m):
        if seen[s[0], s[1]]:
            continue
        st = [(s[0], s[1])]
        seen[s[0], s[1]] = True
        comp = []
        while st:
            r, c = st.pop()
            comp.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < nx and 0 <= cc < ny and m[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        st.append((rr, cc))
        out.append(np.array(comp))
    return out


def _describe(comp, Hm, Cm):
    x0, _, y0, _ = WORKSPACE
    xs = x0 + (comp[:, 0] + 0.5) * CELL
    ys = y0 + (comp[:, 1] + 0.5) * CELL
    hs = Hm[comp[:, 0], comp[:, 1]]
    cs = Cm[comp[:, 0], comp[:, 1]]
    top = float(hs.max())
    tb = hs > top - CAP_BAND
    tc = cs[tb].mean(0)
    return dict(
        n=len(comp), ztop=top,
        xmid=float((xs.min() + xs.max()) / 2), ymid=float((ys.min() + ys.max()) / 2),
        capx=float((xs[tb].min() + xs[tb].max()) / 2),
        capy=float((ys[tb].min() + ys[tb].max()) / 2),
        capw=float(ys[tb].max() - ys[tb].min()),
        green=float(tc[1] - 0.5 * (tc[0] + tc[2])),
        rgb=[round(float(v), 2) for v in tc],
    )


def perceive(api, eef_xy, tag=""):
    P, rgb = _cloud(api)
    Hm, Cm = _heightmap(P, rgb, eef_xy)
    props = [_describe(c, Hm, Cm) for c in _components(Hm) if len(c) >= 8]
    props.sort(key=lambda p: -p["n"])
    for p in props:
        api.log("PROP%s n=%d ztop=%.3f mid=(%.3f,%.3f) cap=(%.3f,%.3f) capw=%.3f "
                "green=%.3f rgb=%s"
                % (tag, p["n"], p["ztop"], p["xmid"], p["ymid"], p["capx"], p["capy"],
                   p["capw"], p["green"], p["rgb"]))
    baskets = [p for p in props if p["n"] >= BASKET_MIN_CELLS]
    others = [p for p in props if p["n"] < BASKET_MIN_CELLS]
    basket = max(baskets, key=lambda p: p["n"]) if baskets else None
    target = max(others, key=lambda p: p["green"]) if others else None
    return basket, target


def in_basket(p, basket):
    return (abs(p["xmid"] - basket["xmid"]) < 0.09 and
            abs(p["ymid"] - basket["ymid"]) < 0.10)


# -------------------------------------------------------------------- actions
class Arm(object):
    def __init__(self, api):
        self.api = api
        self.moves = 0

    def left(self):
        return MOVE_BUDGET - self.moves

    def goto(self, xyz, seconds=2.0):
        self.moves += 1
        r = self.api.move(np.asarray(xyz, float), rotation=R_DOWN, seconds=seconds)
        e = self.api.eef()
        self.api.log("MOVE#%d -> [%.4f %.4f %.4f] res=%.4f eef=[%.4f %.4f %.4f]"
                     % (self.moves, xyz[0], xyz[1], xyz[2], float(np.ravel(r)[0]),
                        e[0], e[1], e[2]))
        return r


def holding(api):
    g = api.gripper()
    return (g["effort"] >= 3.0 and HOLD_WIDTH_MIN < g["width_m"] < HOLD_WIDTH_MAX), g


def try_grasp(arm, api, target, dx):
    """One close attempt at the target's cap, offset by dx along x. Returns (ok, gripper)."""
    gx = target["capx"] + dx
    gy = target["capy"]
    gz = target["ztop"] - GRASP_DZ
    api.log("ATTEMPT dx=%+.3f aim=(%.4f,%.4f,%.4f) capw=%.3f"
            % (dx, gx, gy, gz, target["capw"]))
    api.grip(0.08)
    arm.goto([gx, gy, CARRY_Z], 2.0)
    arm.goto([gx, gy, gz], 2.0)
    api.grip(0.0)
    api.settle(0.4)
    arm.goto([gx, gy, CARRY_Z], 2.0)
    ok, g = holding(api)
    api.log("AFTER_LIFT ok=%s width=%.4f effort=%.2f" % (ok, g["width_m"], g["effort"]))
    return ok, g


def deliver(arm, api, basket):
    bx, by = basket["xmid"], basket["ymid"]
    arm.goto([bx, by, CARRY_Z], 3.0)
    arm.goto([bx, by, basket["ztop"] + RELEASE_DZ], 2.0)
    ok, g = holding(api)
    api.log("AT_RELEASE held=%s width=%.4f" % (ok, g["width_m"]))
    api.grip(0.08)
    api.settle(0.6)
    arm.goto([bx, by, CARRY_Z], 2.0)


def run(api):
    api.log("INSTRUCTION %r" % (api.instruction(),))
    arm = Arm(api)
    e0 = api.eef()
    api.log("EEF0 [%.4f %.4f %.4f] GRIP0 %s" % (e0[0], e0[1], e0[2], api.gripper()))
    api.grip(0.08)
    api.settle(0.2)

    obs_xy = (e0[0], e0[1])
    basket, target = perceive(api, obs_xy)
    if basket is None or target is None:
        api.log("ABORT basket=%s target=%s" % (basket is not None, target is not None))
        return
    api.log("BASKET mid=(%.3f,%.3f) ztop=%.3f n=%d"
            % (basket["xmid"], basket["ymid"], basket["ztop"], basket["n"]))
    api.log("TARGET green=%.3f cap=(%.3f,%.3f) ztop=%.3f"
            % (target["green"], target["capx"], target["capy"], target["ztop"]))

    for cycle in range(2):
        grabbed = False
        for dx in X_AIM_LADDER:
            if arm.left() < 7:
                api.log("BUDGET stop before dx=%+.3f (moves=%d)" % (dx, arm.moves))
                break
            ok, _ = try_grasp(arm, api, target, dx)
            if ok:
                grabbed = True
                break
            # missed: drop whatever we may be pinching, park clear, look again
            api.grip(0.08)
            arm.goto(list(OBS_POSE), 2.0)
            b2, t2 = perceive(api, (OBS_POSE[0], OBS_POSE[1]), tag="_R")
            if t2 is not None:
                target = t2
            if b2 is not None:
                basket = b2
            api.log("RETRY target cap=(%.3f,%.3f) ztop=%.3f"
                    % (target["capx"], target["capy"], target["ztop"]))
        if not grabbed:
            api.log("NO_GRASP after ladder, cycle=%d moves=%d" % (cycle, arm.moves))
            return

        deliver(arm, api, basket)

        if arm.left() < 9:
            api.log("BUDGET stop after deliver (moves=%d)" % arm.moves)
            return
        arm.goto(list(OBS_POSE), 2.0)
        b3, t3 = perceive(api, (OBS_POSE[0], OBS_POSE[1]), tag="_V")
        if b3 is None or t3 is None:
            api.log("VERIFY inconclusive; stopping")
            return
        if in_basket(t3, b3):
            api.log("VERIFY dressing at (%.3f,%.3f) is inside basket (%.3f,%.3f) -- done"
                    % (t3["xmid"], t3["ymid"], b3["xmid"], b3["ymid"]))
            return
        api.log("VERIFY dressing at (%.3f,%.3f) still outside basket (%.3f,%.3f) -- redo"
                % (t3["xmid"], t3["ymid"], b3["xmid"], b3["ymid"]))
        basket, target = b3, t3
    api.log("END moves=%d" % arm.moves)
