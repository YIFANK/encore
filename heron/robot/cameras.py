"""RealSense capture for the stationary kit's four D405s, with depth aligned to
color and per-camera extrinsics loaded from calibration files."""
from __future__ import annotations

from typing import Any

import numpy as np

from ..config import HeronConfig
from ..types import Frame
from .workspace import load_extrinsics


class RealSenseRig:
    def __init__(self, cfg: HeronConfig) -> None:
        try:
            import pyrealsense2 as rs  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("pyrealsense2 is not installed; pip install 'heron[real]'") from e
        self._rs = rs
        self.cfg = cfg
        self._pipes: dict[str, Any] = {}
        self._aligns: dict[str, Any] = {}
        self._intrinsics: dict[str, np.ndarray] = {}
        self._extrinsics: dict[str, np.ndarray | None] = {}
        for name, cam in cfg.cameras.items():
            if not cam.serial:
                continue
            pipe = rs.pipeline()
            conf = rs.config()
            conf.enable_device(cam.serial)
            conf.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            conf.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            profile = pipe.start(conf)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self._intrinsics[name] = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.0]])
            self._pipes[name] = pipe
            self._aligns[name] = rs.align(rs.stream.color)
            self._extrinsics[name] = load_extrinsics(cam.extrinsics_file) if cam.extrinsics_file else None
        # Warm up auto-exposure.
        for pipe in self._pipes.values():
            for _ in range(5):
                pipe.wait_for_frames()

    def capture(self, camera: str, t_base_cam: np.ndarray | None = None) -> Frame:
        """t_base_cam override serves wrist cameras, whose extrinsics depend on
        the current arm pose (computed by the robot backend, not stored)."""
        if camera not in self._pipes:
            raise KeyError(f"camera {camera!r} not configured (serial missing in config?)")
        pipe = self._pipes[camera]
        frames = self._aligns[camera].process(pipe.wait_for_frames())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        rgb = np.asanyarray(color.get_data()).copy()
        depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * float(depth.get_units())
        return Frame(camera=camera, rgb=rgb, depth=depth_m,
                     intrinsics=self._intrinsics[camera],
                     t_base_cam=t_base_cam if t_base_cam is not None else self._extrinsics[camera])

    def close(self) -> None:
        for pipe in self._pipes.values():
            try:
                pipe.stop()
            except Exception:
                pass
