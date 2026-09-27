"""LeRobot policy backend for the vla skill (Trossen AI Stationary).

Talks to the upstream LeRobot async-inference policy server (gRPC), which is
how TrossenRobotics' docs run pi0/ACT/SmolVLA on this kit. The robot side uses
their plugin (`lerobot-robot-trossen`, robot type `bi_widowxai_follower_robot`).

This adapter deliberately runs the plugin's OWN robot object for the rollout
(cameras + 30 Hz joint streaming are its native habitat) while Heron's driver
connections are closed — the trossen controllers accept a single owner. Heron
therefore treats a VLA rollout as an exclusive lease on the hardware:

    heron primitives  ──release──▶  lerobot robot_client rollout  ──return──▶  heron

Because two robot stacks can't hold the arms at once, `vla` on real hardware is
only available when Heron is launched with --robot lerobot-shared (see cli),
where Heron itself drives motion through the plugin. Until that mode is wired
for your checkpoints, the vla skill reports unavailable rather than pretending.
"""
from __future__ import annotations

from typing import Any


class LeRobotAsyncClient:
    """Placeholder that resolves only when the integration prerequisites exist.

    Wire-up checklist (see README 'VLA policies'):
      1. pip install lerobot + the TrossenRobotics plugin packages.
      2. Start the policy server with your checkpoint:
         python -m lerobot.async_inference.policy_server --host 0.0.0.0 --port 8080 ...
      3. Implement start/step/stop below against lerobot.async_inference.robot_client
         with robot type bi_widowxai_follower_robot and your camera serials.
    """

    def __init__(self, ctx: Any) -> None:
        raise RuntimeError(
            "LeRobot VLA backend not wired up yet on this install — "
            "see heron/skills/lerobot_backend.py for the checklist"
        )

    def start(self, instruction: str) -> None:  # pragma: no cover - interface doc
        raise NotImplementedError

    def step(self) -> None:  # pragma: no cover
        raise NotImplementedError

    def stop(self) -> None:  # pragma: no cover
        raise NotImplementedError
