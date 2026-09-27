"""Run a locally-trained ACT checkpoint in the twin, in this process.

The other two policy backends talk to a server: openpi over HTTP, LeRobot over
gRPC. Both are right for a frozen foundation model somebody else is hosting and
wrong for the training loop, where the checkpoint was written by
tools/train_local.py forty minutes ago and lives on this disk. A server in
between would add a process to supervise, a port to collide, and nothing else.

The contract is the vla skill's: start(instruction) / step() / stop(), with a
`finished` flag the skill polls. What makes the rollout meaningful is that the
observation is assembled exactly as tools/collect.py recorded it — the same
cameras at the same size through the same render call, and the same
joint_state — and the action goes back through apply_joint_command, the exact
inverse of the joint_command the dataset stored. A policy is being asked to
control the robot it actually saw.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


class LocalActClient:
    """An ACT checkpoint driving one arm of the twin at the dataset's rate."""

    def __init__(self, ctx: Any, checkpoint: str | Path, cameras: list[str] | None = None,
                 arm: str = "right", fps: float = 30.0, device: str | None = None,
                 action_steps: int | None = None) -> None:
        import torch  # noqa: PLC0415

        from lerobot.policies.act.modeling_act import ACTPolicy  # noqa: PLC0415

        self.ctx = ctx
        self.arm = arm
        self.fps = float(fps)
        self.finished = False
        self.steps = 0
        # The backend, not the SafeRobot wrapper: this is a raw command channel
        # by design. The safety box guards Cartesian targets, and a joint-space
        # action from a policy is not a Cartesian target — clamping it against
        # a workspace box would silently rewrite what the policy asked for and
        # then score the rewrite.
        self.backend = getattr(ctx.robot, "_robot", ctx.robot)
        if not hasattr(self.backend, "apply_joint_command"):
            raise RuntimeError(
                f"{type(self.backend).__name__} cannot take joint commands; the local "
                f"ACT backend drives the twin, not hardware")
        self.device = device or ("mps" if torch.backends.mps.is_available()
                                 else "cuda" if torch.cuda.is_available() else "cpu")
        self.policy = ACTPolicy.from_pretrained(str(checkpoint)).to(self.device).eval()
        # HOW MUCH OF EACH CHUNK TO COMMIT TO BEFORE LOOKING AGAIN. ACT predicts
        # a whole chunk and the checkpoint's own default executes all of it —
        # 50 steps is 1.7 seconds of open loop at 30 Hz, during which the policy
        # cannot see that it missed. Shortening it costs nothing at training
        # time and is the difference between acting on a stale plan and acting
        # on a fresh one; left at the checkpoint's value unless asked, so a
        # score never changes for a reason the caller did not choose.
        if action_steps:
            self.policy.config.n_action_steps = max(1, min(int(action_steps),
                                                          self.policy.config.chunk_size))
        self.action_steps = self.policy.config.n_action_steps
        self._torch = torch
        # Which cameras, and at what size, the checkpoint was trained on. Read
        # off the policy's own input features rather than passed in: a rollout
        # that feeds a 480x640 frame to a model trained at 224 fails as a bad
        # success rate rather than as an error, which is the worst way to fail.
        feats = self.policy.config.input_features
        self.cameras = {}
        for key, ft in feats.items():
            if key.startswith("observation.images."):
                c, h, w = ft.shape
                self.cameras[key.removeprefix("observation.images.")] = (int(h), int(w))
        if cameras:
            missing = [c for c in cameras if c not in self.cameras]
            if missing:
                raise RuntimeError(f"checkpoint has no input for cameras {missing}; "
                                   f"it was trained on {sorted(self.cameras)}")

    # -- the vla skill's contract --------------------------------------------
    def start(self, instruction: str) -> None:
        self.policy.reset()
        self.finished = False
        self.steps = 0
        self.instruction = instruction
        self.ctx.log.event("act_rollout_start", checkpoint=str(self.policy.name_or_path)
                           if hasattr(self.policy, "name_or_path") else "",
                           cameras=sorted(self.cameras), device=self.device,
                           instruction=instruction)

    def observation(self) -> dict:
        torch = self._torch
        obs = {"observation.state": torch.from_numpy(
            np.asarray(self.backend.joint_state(self.arm), dtype=np.float32))[None]}
        for cam, (h, w) in self.cameras.items():
            rgb = np.asarray(self.backend.render_rgb(cam, w, h), dtype=np.float32) / 255.0
            obs[f"observation.images.{cam}"] = torch.from_numpy(
                np.transpose(rgb, (2, 0, 1)))[None]
        obs = {k: v.to(self.device) for k, v in obs.items()}
        obs["task"] = [getattr(self, "instruction", "")]
        return obs

    def step(self) -> None:
        torch = self._torch
        with torch.inference_mode():
            action = self.policy.select_action(self.observation())
        q = action.squeeze(0).float().cpu().numpy()
        self.backend.apply_joint_command(self.arm, q, seconds=1.0 / self.fps)
        self.steps += 1

    def stop(self) -> None:
        self.finished = True
        self.ctx.log.event("act_rollout_stop", steps=self.steps)
