"""ArUco marker detection for calibration.

An alternative fingertip detector for `heron calibrate`: clamp a printed ArUco
marker between the gripper fingers (its center then sits at the pinch point,
i.e. the point forward kinematics knows) and the corner-refined detection
replaces ER pointing — sub-pixel accuracy, no API calls, immune to gripper
reflections. Same grid, same solvers.

Print from calib.io: dictionary DICT_4X4_50, ~30-35 mm side, mounted flat.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def detect_aruco_center(rgb: np.ndarray, dict_name: str = "4X4_50",
                        marker_id: Optional[int] = None) -> Optional[tuple[int, int, float]]:
    """(u, v, confidence) of the marker center, or None. Sub-pixel corners averaged."""
    try:
        import cv2  # noqa: PLC0415
    except ImportError as e:
        raise RuntimeError("opencv is required for aruco detection "
                           "(pip install opencv-python-headless)") from e
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, f"DICT_{dict_name}"))
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector = aruco.ArucoDetector(dictionary, params)
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    if marker_id is not None:
        matches = [c for c, i in zip(corners, ids.flatten()) if int(i) == marker_id]
        if not matches:
            return None
        quad = matches[0]
    else:
        quad = corners[0]
    center = quad.reshape(-1, 2).mean(axis=0)
    return int(round(center[0])), int(round(center[1])), 0.99
