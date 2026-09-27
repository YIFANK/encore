"""Client for the M2T2 grasp-proposal service (tools/m2t2_service.py).

The service is optional by construction: `cfg.grasp_service_url` unset means
nobody even builds this client, and every failure mode — dead tunnel, slow GPU,
refused frame — raises `GraspServiceError` for the caller to catch and fall
back on. The rig must never be blocked by a dead tunnel: a proposal is an
upgrade to the heuristic grasp point, not a dependency of it.

The service returns grasps in whatever frame the supplied extrinsic maps into
(for Heron, the world frame). Poses come back in M2T2's hand convention:
`approach` is the direction the hand advances (down-ish for a table grasp),
`binormal` is the line the fingers close along.
"""
from __future__ import annotations

import base64
import io
import json
import time
import urllib.request
from dataclasses import dataclass

import numpy as np
from PIL import Image

from ..types import Frame


class GraspServiceError(RuntimeError):
    """Any reason the service produced no usable answer. Callers fall back."""


@dataclass
class GraspProposal:
    tcp_xyz: np.ndarray  # where the fingers close, world frame
    approach: np.ndarray  # unit vector the hand advances along, world frame
    binormal: np.ndarray  # unit vector of the finger-closing line, world frame
    score: float
    # M2T2's predicted contact point ON the object surface, when given. The
    # tcp is derived through a hand-model offset; the contact is the model
    # pointing at the thing itself, which makes it the honest evidence for
    # "this grasp is about THAT object".
    contact: np.ndarray | None = None

    @property
    def tilt_deg(self) -> float:
        """Angle between the approach and straight-down. 0 = perfectly top-down."""
        return float(np.degrees(np.arccos(np.clip(-self.approach[2], -1.0, 1.0))))

    @property
    def yaw_rad(self) -> float:
        """World-z rotation taking the gripper's default finger line (world y
        under the top-down approach_rvec) onto this grasp's closing line.

        The finger line is a line, not a direction, so the answer lives in
        (-pi/2, pi/2] — the smallest wrist rotation that aligns the fingers.
        """
        yaw = float(np.arctan2(-self.binormal[0], self.binormal[1]))
        while yaw <= -np.pi / 2:
            yaw += np.pi
        while yaw > np.pi / 2:
            yaw -= np.pi
        return yaw


def _png_b64(arr: np.ndarray) -> str:
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode()


# DEPTH MUST BE LOSSLESS; COLOUR NEED NOT BE. The link to the GPU measured
# 19 KB/s on this rig, and a cropped workspace view costs ~150 KB as PNG and
# ~15 KB as JPEG — ten seconds of a 34-second pick, spent shipping pixels the
# model looks at rather than measures from. The depth image stays PNG16: a
# lossy millimetre is a lossy grasp.
RGB_JPEG_QUALITY = 85


def _rgb_b64(arr: np.ndarray, lossless: bool = False) -> str:
    if lossless:
        return _png_b64(arr)
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(arr).astype(np.uint8)).save(
        buf, "JPEG", quality=RGB_JPEG_QUALITY)
    return base64.b64encode(buf.getvalue()).decode()


def _crop_to_workspace(frame: Frame, center_xy, radius_m: float, z_range,
                       margin_px: int = 24):
    """(depth, rgb, intrinsics) restricted to the image region that can see
    the crop volume: the box (center +/- radius) x z_range in the world frame,
    projected through the camera. The principal point moves with the crop, so
    deprojection on the server is unchanged."""
    corners = np.array([
        [center_xy[0] + sx * radius_m, center_xy[1] + sy * radius_m, z]
        for sx in (-1, 1) for sy in (-1, 1) for z in z_range])
    cam_from_world = np.linalg.inv(np.asarray(frame.t_base_cam, dtype=float))
    cam = (cam_from_world @ np.c_[corners, np.ones(len(corners))].T)[:3]
    k = np.asarray(frame.intrinsics, dtype=float)
    h, w = frame.depth.shape[:2]
    if np.any(cam[2] <= 1e-6):  # volume pokes behind the camera: keep it all
        return frame.depth, frame.rgb, k
    us = k[0, 0] * cam[0] / cam[2] + k[0, 2]
    vs = k[1, 1] * cam[1] / cam[2] + k[1, 2]
    u0 = int(np.clip(np.floor(us.min()) - margin_px, 0, w - 1))
    u1 = int(np.clip(np.ceil(us.max()) + margin_px, u0 + 1, w))
    v0 = int(np.clip(np.floor(vs.min()) - margin_px, 0, h - 1))
    v1 = int(np.clip(np.ceil(vs.max()) + margin_px, v0 + 1, h))
    k = k.copy()
    k[0, 2] -= u0
    k[1, 2] -= v0
    return frame.depth[v0:v1, u0:u1], frame.rgb[v0:v1, u0:u1], k


def _surface_z(frame: Frame, center_xy, radius_m: float,
               z_range: tuple[float, float] | None = None) -> float | None:
    """Median world height this view sees inside the crop volume, or None.

    Not the object's height and not meant to be: it is dominated by whatever
    surface fills the crop, which is the same physical surface for every camera
    looking at it. That makes it a per-view reading of one shared quantity, and
    two views that disagree about it disagree about the scene.

    THE VOLUME MUST BE THE ONE THAT WILL BE FUSED, not just its xy footprint.
    An xy cylinder alone reaches from the ceiling to the floor, and cam_low —
    a side view that sees the table's edge and the ground beyond it — answered
    -407 mm, a median taken half on the tabletop and half on the floor. The
    comparison is only meaningful over the slab the service actually keeps.
    """
    if frame.depth is None or frame.intrinsics is None or frame.t_base_cam is None:
        return None
    k = np.asarray(frame.intrinsics, dtype=float)
    d = np.asarray(frame.depth, dtype=float)
    h, w = d.shape[:2]
    step = max(1, min(h, w) // 120)          # ~14k samples is plenty for a median
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = d[::step, ::step].reshape(-1)
    good = np.isfinite(z) & (z > 0.1) & (z < 3.0)
    if good.sum() < 50:
        return None
    u, v, z = us.reshape(-1)[good], vs.reshape(-1)[good], z[good]
    pts = np.stack([(u - k[0, 2]) * z / k[0, 0], (v - k[1, 2]) * z / k[1, 1], z], 1)
    t = np.asarray(frame.t_base_cam, dtype=float)
    world = (t[:3, :3] @ pts.T).T + t[:3, 3]
    keep = np.ones(len(world), dtype=bool)
    if center_xy is not None:
        keep &= (np.abs(world[:, 0] - center_xy[0]) < radius_m) & \
                (np.abs(world[:, 1] - center_xy[1]) < radius_m)
    if z_range is not None:
        keep &= (world[:, 2] >= z_range[0]) & (world[:, 2] <= z_range[1])
    if keep.sum() < 50:
        return None
    return float(np.median(world[keep, 2]))


# Two calibrations that disagree by more than this are not describing one scene.
# The same bar tools/cross_calibrate.py refuses to write above.
VIEW_AGREEMENT_MAX_M = 0.010


def views_that_agree(usable: list[Frame], center_xy, radius_m: float,
                     z_range: tuple[float, float] | None = None):
    """The views that describe the same world as the first one.

    A SECOND CAMERA IS ONLY A SECOND OPINION IF IT AGREES ABOUT WHERE THINGS
    ARE. Concatenating clouds from two extrinsics that differ puts every
    surface in the scene twice, a fixed distance apart, and the model then
    ranks grasps on a phantom between the copies. Measured on the rig
    2026-08-06: cam_high put the tabletop at -21 mm across the working area
    and cam_low at +1..+11 mm, a 21-31 mm split; the fused cloud doubled a
    25 mm block, and the grasp success rate over 400 picks fell from 65-73%
    (single view, 08-03/04) to 37% the day multi-view landed.

    So the anchor is kept unconditionally and every other view has to earn its
    place by agreeing with it. Returns (views, dropped) for the caller to log.
    """
    if len(usable) < 2:
        return usable, []
    anchor_z = _surface_z(usable[0], center_xy, radius_m, z_range)
    if anchor_z is None:
        return usable, []
    keep, dropped = [usable[0]], []
    for f in usable[1:]:
        z = _surface_z(f, center_xy, radius_m, z_range)
        if z is None or abs(z - anchor_z) <= VIEW_AGREEMENT_MAX_M:
            keep.append(f)
        else:
            dropped.append((f.camera, float(z - anchor_z)))
    return keep, dropped


class GraspServiceClient:
    def __init__(self, url: str, timeout_s: float = 2.0) -> None:
        self.url = url.rstrip("/")
        self.timeout_s = float(timeout_s)
        # Flipped for good on the first refusal: a server that will not decode
        # JPEG says so once, and every later call pays PNG rather than a retry.
        self._rgb_lossless = False

    def propose(self, frame: Frame, center_xy=None, radius_m: float = 0.14,
                z_range: tuple[float, float] | None = None,
                num_runs: int = 1, max_grasps: int = 20,
                num_points: int = 4096) -> list[GraspProposal]:
        """Ranked grasp proposals for the scene in `frame`, best first.

        Needs aligned depth, intrinsics, and an extrinsic on the frame — the
        same three things deprojection needs. `center_xy`/`radius_m` crop the
        cloud around the object in the world frame so the model ranks grasps on
        the thing we asked about, not the clutter next to it.
        """
        frames = list(frame) if isinstance(frame, (list, tuple)) else [frame]
        usable = [f for f in frames
                  if f.depth is not None and f.intrinsics is not None
                  and f.t_base_cam is not None]
        if not usable:
            raise GraspServiceError(
                f"{[f.camera for f in frames]} have no depth, intrinsics, or "
                "extrinsics — the service needs all three")
        usable, disagreeing = views_that_agree(usable, center_xy, radius_m, z_range)
        self.dropped_views = disagreeing
        t0 = time.perf_counter()

        def _view(f):
            depth, rgb, k = f.depth, f.rgb, np.asarray(f.intrinsics, dtype=float)
            if center_xy is not None:
                # Crop to the pixels the service would keep anyway: the link
                # to the GPU is an intercontinental tunnel, and a full
                # 1280x720 PNG pair costs more wall time than the model does.
                depth, rgb, k = _crop_to_workspace(
                    f, center_xy, radius_m, z_range or (-0.05, 0.4))
            depth_mm = np.nan_to_num(depth * 1000.0, nan=0.0, posinf=0.0,
                                     neginf=0.0)
            return {
                "depth_png16_b64": _png_b64(np.clip(depth_mm, 0, 65535).astype(np.uint16)),
                "depth_scale_m": 0.001,
                "rgb_png_b64": _rgb_b64(rgb, self._rgb_lossless),
                "intrinsics": k.tolist(),
                "world_from_cam": np.asarray(f.t_base_cam, dtype=float).tolist(),
            }

        req = {
            **_view(usable[0]),
            "views": [_view(f) for f in usable] if len(usable) > 1 else None,
            "crop_center_xy": [float(center_xy[0]), float(center_xy[1])]
            if center_xy is not None else None,
            "crop_radius_m": float(radius_m),
            "z_range": list(z_range) if z_range is not None else None,
            "num_runs": int(num_runs),
            "max_grasps": int(max_grasps),
            # The model's latency scales with its sample count, and a cropped
            # single-object cloud does not need the DROID-scene default
            # (16384): measured 6.4 s vs 1.5 s on the rig's frames for the
            # same grasps on a 2 cm block.
            "num_points": int(num_points),
        }
        try:
            r = urllib.request.urlopen(
                urllib.request.Request(
                    f"{self.url}/propose", json.dumps(req).encode(),
                    {"Content-Type": "application/json"}),
                timeout=self.timeout_s)
            out = json.loads(r.read())
        except Exception as e:  # timeout, refused, HTTP error, bad JSON — all one fate
            raise GraspServiceError(f"{type(e).__name__}: {e}") from e
        if not out.get("ok"):
            # A server that cannot read JPEG says so once. Fall back for good
            # rather than paying a failed round trip per grasp for the rest of
            # the session; the next call re-sends the same scene losslessly.
            if not self._rgb_lossless and "jpeg" in str(out.get("error", "")).lower():
                self._rgb_lossless = True
                raise GraspServiceError(
                    "service refused the JPEG colour image; retrying losslessly")
            raise GraspServiceError(str(out.get("error", "service said not ok")))

        proposals = [
            GraspProposal(
                tcp_xyz=np.asarray(g["tcp_xyz"], dtype=float),
                approach=np.asarray(g["approach"], dtype=float),
                binormal=np.asarray(g["binormal"], dtype=float),
                score=float(g["score"]),
                contact=np.asarray(g["contact"], dtype=float)
                if g.get("contact") is not None else None,
            )
            for g in out.get("grasps", [])
        ]
        self.last_latency_s = time.perf_counter() - t0
        self.last_n_points = int(out.get("n_points", 0))
        # The service's own clock, for telling GPU time from tunnel time.
        self.last_timings_ms = dict(out.get("timings_ms", {}))
        return proposals


def yawed_orientation(approach_rvec, yaw_rad: float) -> np.ndarray:
    """Angle-axis wrist orientation: the arm's top-down approach, rotated about
    world z by `yaw_rad` so the fingers close along the proposed line."""
    r0 = _rodrigues(np.asarray(approach_rvec, dtype=float))
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return _rotvec(rz @ r0)


def _rodrigues(rvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * (kx @ kx)


def _rotvec(r: np.ndarray) -> np.ndarray:
    theta = float(np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)))
    if theta < 1e-12:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    n = np.linalg.norm(axis)
    if n < 1e-12:  # theta ~ pi: axis from the diagonal
        d = np.clip((np.diag(r) + 1) / 2, 0, None)
        axis = np.sqrt(d)
        axis[np.argmax(np.abs(axis))] *= np.sign(
            axis[np.argmax(np.abs(axis))]) or 1.0
        return axis / (np.linalg.norm(axis) or 1.0) * theta
    return axis / n * theta
