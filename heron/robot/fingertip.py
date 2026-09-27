"""Locate the gripper's pinch point by making only the fingers move.

Pointing a VLM at "the gripper fingertips" is ambiguous on a two-arm rig: the
idle arm's gripper is just as good an answer, and being idle it lands on the
same pixel every time — which silently produces a degenerate calibration rather
than an error. Marker-based detection avoids that but needs a printed marker
clamped in the hand.

This detector needs neither. At a fixed arm pose the gripper is opened and
closed; the fingers are then the only things in the scene that moved, and since
they travel symmetrically about the pinch point, the centroid of the changed
pixels IS the point forward kinematics reports. Deterministic, free, and immune
to whatever else is in frame.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

MIN_CHANGED_PX = 60
MAX_CHANGED_FRACTION = 0.08  # more than this means something else moved too


def pinch_point_from_pair(rgb_open: np.ndarray, rgb_closed: np.ndarray,
                          threshold: int = 25) -> Optional[tuple[int, int, float]]:
    """(u, v, confidence) of the pinch point, or None if the change is unusable."""
    import cv2  # noqa: PLC0415

    a = cv2.GaussianBlur(rgb_open, (5, 5), 0).astype(np.int16)
    b = cv2.GaussianBlur(rgb_closed, (5, 5), 0).astype(np.int16)
    diff = np.abs(a - b).max(axis=2).astype(np.uint8)
    mask = (diff > threshold).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    changed = int(mask.sum())
    if changed < MIN_CHANGED_PX:
        return None
    if changed > MAX_CHANGED_FRACTION * mask.size:
        # The arm drifted, someone walked past, or the exposure jumped: the
        # centroid would no longer mean "pinch point".
        return None

    # The fingers sweep symmetrically about the pinch point, so the centroid of
    # ALL the finger pixels is the pinch point — but only the finger pixels. Each
    # finger leaves a blob at its old position and one at its new position, so
    # counting blobs is no way to find the pair; instead drop blobs too small to
    # be a finger (specular flicker, sensor noise) and average what remains.
    n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    keep = np.flatnonzero(areas >= max(MIN_CHANGED_PX * 0.15, 0.15 * areas.max())) + 1
    if keep.size == 0:
        return None
    weights = stats[keep, cv2.CC_STAT_AREA].astype(float)
    u, v = (centroids[keep] * weights[:, None]).sum(axis=0) / weights.sum()
    confidence = float(min(1.0, weights.sum() / 400.0))
    return int(round(u)), int(round(v)), max(0.5, confidence)
