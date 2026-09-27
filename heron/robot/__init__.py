"""Hardware abstraction. Implementations: MockRobot (no hardware), TrossenStationary (real kit).

All poses at this boundary are in the shared WORLD frame (table center origin,
x forward, y left, z up, table top z=0). Implementations own the conversion to
per-arm base frames.
"""
from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from ..types import Frame


@runtime_checkable
class RobotInterface(Protocol):
    arms: list[str]
    cameras: list[str]

    def home(self, arm: str) -> None: ...

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0) -> None:
        """Move the tool to world-frame xyz with a fixed top-down grasp orientation."""
        ...

    def get_cartesian(self, arm: str) -> np.ndarray: ...

    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None: ...

    def get_gripper(self, arm: str) -> dict[str, float]:
        """{'width_m': ..., 'effort': ...} — effort magnitude is the grasp-verification signal."""
        ...

    def get_external_effort(self, arm: str) -> float:
        """Magnitude of gravity/friction-compensated external effort on the arm.

        The contact signal: descend-until-touch replaces depth for grasp height.
        """
        ...

    def capture(self, camera: str) -> Frame: ...

    def stop(self) -> None: ...

    def shutdown(self) -> None: ...
