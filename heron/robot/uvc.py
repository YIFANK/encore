"""UVC/AVFoundation camera rig: OpenCV capture by device index, RGB only.

This is the backend for the lab's Mac workstation, where librealsense cannot
open the D405s over ssh (documented in their calib_table.py). Grounding runs
depth-free via per-camera table-plane homographies; wrist/uncalibrated cameras
still serve VQA verification.

The lab's index convention (STATIONARY_AI_使用说明.md):
    0 = cam_left_wrist   1 = cam_high   2 = cam_low   3 = cam_right_wrist
"""
from __future__ import annotations

import sys
import threading
import time
from typing import Any

import numpy as np

from ..config import HeronConfig
from ..types import Frame
from .workspace import load_homography, load_projection


class UVCRig:
    def __init__(self, cfg: HeronConfig) -> None:
        try:
            import cv2  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("opencv is required for the UVC camera backend "
                               "(pip install opencv-python-headless)") from e
        self._cv2 = cv2
        self.cfg = cfg
        self._caps: dict[str, Any] = {}
        self._homographies: dict[str, tuple[np.ndarray, float]] = {}
        self._projections: dict[str, np.ndarray] = {}
        self._lock = threading.Lock()
        for name, cam in cfg.cameras.items():
            if cam.index is None:
                continue
            if cam.homography_file:
                try:
                    self._homographies[name] = load_homography(cam.homography_file)
                except FileNotFoundError:
                    pass  # not calibrated yet; frames stay VQA-only
            if cam.projection_file:
                try:
                    self._projections[name] = load_projection(cam.projection_file)
                except FileNotFoundError:
                    pass

    def _open(self, name: str):
        cam = self.cfg.cameras[name]
        # Explicit AVFoundation: without it OpenCV may fall back to FFMPEG, which
        # cannot enumerate macOS devices at all.
        backend = getattr(self._cv2, "CAP_AVFOUNDATION", 0) if sys.platform == "darwin" else 0
        cap = self._cv2.VideoCapture(cam.index, backend) if backend else self._cv2.VideoCapture(cam.index)
        if not cap.isOpened():
            raise RuntimeError(
                f"camera {name!r} (index {cam.index}) failed to open — is another "
                "process (camera_dashboard.py?) holding it, or is this an ssh "
                "session (macOS denies camera access there — use a GUI Terminal)?"
            )
        self._caps[name] = cap
        return cap

    def capture(self, camera: str) -> Frame:
        if camera not in self.cfg.cameras or self.cfg.cameras[camera].index is None:
            raise KeyError(f"camera {camera!r} has no UVC index configured")
        with self._lock:
            cap = self._caps.get(camera) or self._open(camera)
            # Drain the driver buffer so the frame is current, not seconds old.
            for _ in range(2):
                cap.grab()
            ok, bgr = cap.read()
        if not ok or bgr is None:
            raise RuntimeError(f"camera {camera!r} returned no frame")
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        h = self._homographies.get(camera)
        return Frame(
            camera=camera, rgb=rgb,
            h_pixel_world=h[0] if h else None,
            plane_z=h[1] if h else self.cfg.table_z,
            proj=self._projections.get(camera),
            t=time.time(),
        )

    def close(self) -> None:
        with self._lock:
            for cap in self._caps.values():
                try:
                    cap.release()
                except Exception:
                    pass
            self._caps.clear()
