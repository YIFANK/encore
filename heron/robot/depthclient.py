"""Client for the sidecar depth server (tools/depth_server.py).

Frames come back with real aligned depth + intrinsics from the server; the
camera->world extrinsics come from `heron calibrate` (Kabsch mode — usable
again on this rig once depth flows). With both, Frame.deproject gives full 3D,
upgrading grounding beyond the table-plane homography.
"""
from __future__ import annotations

import base64
import io
import json
import time
import urllib.request

import numpy as np
from PIL import Image

from ..types import Frame


class DepthServerClient:
    """Client for tools/depth_server.py, which owns every camera on the rig.

    One client serves all roles; `t_base_cam` is per-camera and supplied by the
    caller from calibration.
    """

    def __init__(self, url: str, t_base_cam: np.ndarray | None = None,
                 timeout_s: float = 20.0, h_pixel_world: np.ndarray | None = None,
                 plane_z: float = 0.0,
                 homography_wh: tuple[int, int] | None = None) -> None:
        self.url = url.rstrip("/")
        self.t_base_cam = t_base_cam
        # Carried so that switching a camera to RGB-D can never be a regression.
        # Without it, a depth frame with no extrinsics has neither route to 3D —
        # `deproject` returns None and grounding fails outright, which is worse
        # than the homography-only setup it replaced.
        self.h_pixel_world = h_pixel_world
        self.plane_z = plane_z
        # (width, height) the homography was fitted at, or None to trust it at
        # any resolution.
        self.homography_wh = homography_wh
        self._warned_resolution = False
        self.timeout_s = timeout_s
        self._info: dict[str, dict] = {}

    def _get(self, path: str) -> bytes:
        with urllib.request.urlopen(f"{self.url}{path}", timeout=self.timeout_s) as r:
            return r.read()

    def info(self, camera: str) -> dict:
        if camera not in self._info:
            self._info[camera] = json.loads(self._get(f"/info/{camera}"))
        return self._info[camera]

    def intrinsics(self, camera: str) -> np.ndarray:
        i = self.info(camera)
        return np.array([[i["fx"], 0, i["ppx"]], [0, i["fy"], i["ppy"]], [0, 0, 1.0]])

    def healthy(self, camera: str | None = None, max_frame_age_s: float = 3.0) -> tuple[bool, str]:
        try:
            h = json.loads(self._get("/health"))
        except Exception as e:
            return False, f"camera server unreachable at {self.url}: {e}"
        cams = h.get("cameras", [])
        if camera is not None:
            cams = [c for c in cams if c.get("role") == camera]
            if not cams:
                return False, f"camera server does not serve {camera!r}"
        bad = [c for c in cams if not c.get("ok")
               or (c.get("frame_age_s") is not None and c["frame_age_s"] > max_frame_age_s)]
        if bad:
            return False, "stale/absent: " + ", ".join(f"{c['role']}({c.get('error') or 'no frames'})" for c in bad)
        return True, ", ".join(f"{c['role']} {c['fps']}fps" for c in cams)

    def capture(self, camera: str) -> Frame:
        if self.t_base_cam is None and self.h_pixel_world is not None:
            # No measured extrinsics, but a plane homography plus the server's K
            # determines the pose — and one derived this way agrees with the
            # homography EXACTLY on the table plane, which measured extrinsics
            # never quite did (their solve plateaued at 12.8 mm rms against it).
            from .workspace import pose_from_plane_homography  # noqa: PLC0415
            try:
                self.t_base_cam = pose_from_plane_homography(
                    self.h_pixel_world, self.intrinsics(camera), self.plane_z)
            except Exception as e:
                # The planar path still answers without the pose — but with no
                # parallax correction, silently 13-16 mm worse on anything
                # tall. That degradation was invisible for a whole capture
                # once (a transient /info timeout) and read as "still
                # inaccurate"; say it, and the next capture retries anyway.
                print(f"[heron] {camera}: pose-from-homography failed ({e}); "
                      "grounding is UNCORRECTED planar this frame")
        payload = json.loads(self._get(f"/frame/{camera}"))
        rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(payload["rgb_png_b64"]))), dtype=np.uint8)
        depth_raw = np.asarray(Image.open(io.BytesIO(base64.b64decode(payload["depth_png16_b64"]))))
        depth_m = depth_raw.astype(np.float32) * float(self.info(camera)["depth_scale"])
        depth_m[depth_raw == 0] = np.nan  # z16 zero = no return
        # The homography lives in the pixel space it was FITTED in. A D405's
        # 640x480 mode is a horizontal crop of 720p, not a scaling, so carrying
        # the same matrix into a different stream size would deproject to
        # confidently wrong world coordinates — and only as a fallback, when
        # depth had already failed and nobody was watching. Withhold it instead.
        hpw, pz = self.h_pixel_world, self.plane_z
        if hpw is not None and self.homography_wh not in (None, (rgb.shape[1], rgb.shape[0])):
            if not self._warned_resolution:
                self._warned_resolution = True
                print(f"[heron] {camera}: homography was fitted at "
                      f"{self.homography_wh[0]}x{self.homography_wh[1]} and this stream is "
                      f"{rgb.shape[1]}x{rgb.shape[0]} — not using it as a fallback",
                      flush=True)
            hpw = None
        return Frame(camera=camera, rgb=rgb, depth=depth_m, intrinsics=self.intrinsics(camera),
                     t_base_cam=self.t_base_cam, h_pixel_world=hpw,
                     plane_z=pz, t=payload.get("t", time.time()))
