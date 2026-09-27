"""pi0.5 as a skill backend, so the simulator has the tool Pigey's numbers use.

Heron's LIBERO evaluation has been scripted-only: `disable_skills: [vla, ...]`,
every motion an operational-space primitive we wrote. Pigey's published LIBERO
numbers come from a frozen pi0.5-LIBERO policy orchestrated by an LLM, and its
own system prompt is explicit about the division of labour:

    Branch B - Articulated verb (open, close, turn on/off, lift the lid)
    "Geometric primitives cannot model articulated motion. If phase 0 didn't
    already solve it, try VLARollout(task verbatim) ONCE."

So they open drawers with the policy. We were opening them with trigonometry,
and libero_goal has been 0% for six runs. This is the missing tool.

Two things are deliberately NOT copied from their harness:

  * the rollout stays MONITORED. `heron/skills/vla.py` checks progress every
    couple of seconds and stops early on success or regression; theirs runs a
    fixed step budget. A policy that has finished and is now knocking the object
    over is a real failure mode, and a fixed budget cannot see it.
  * success is still decided by the program's postconditions afterwards. The
    monitor only decides when to stop, never what to believe.

The client is duck-typed to `start(instruction) / step() / stop()`, which is
what the vla skill already expects from the LeRobot backend on hardware.

Serving side (once, on a free GPU):

    OPENPI_DATA_HOME=/mnt/data/YifanKang/cache/openpi \\
    CUDA_VISIBLE_DEVICES=1 .venv/bin/python scripts/serve_policy.py --port 8000 \\
        policy:checkpoint --policy.config=pi05_libero \\
        --policy.dir=<checkpoint>/pi05_libero
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

# The policy was trained at this resolution with padded resize; anything else
# silently changes the input distribution.
RESIZE_SIZE = 224
# How much of each predicted action chunk to execute before asking again.
# openpi's own LIBERO example uses 5: the chunk is 10 long, and running it to
# the end means acting on a plan made from an observation ten steps stale.
DEFAULT_REPLAN_STEPS = 5


def quat_to_axisangle(quat) -> np.ndarray:
    """robosuite quaternion (x, y, z, w) -> axis-angle, as openpi's LIBERO
    example does it. Reproduced rather than imported because the state vector
    the policy was normalised against is built exactly this way, and a different
    convention here is a silent distribution shift, not an error."""
    q = np.asarray(quat, dtype=np.float64)
    w = float(np.clip(q[3], -1.0, 1.0))
    den = math.sqrt(1.0 - w * w)
    if math.isclose(den, 0.0):
        return np.zeros(3)
    return (q[:3] * 2.0 * math.acos(w)) / den


class OpenPiLiberoClient:
    """Drives a LiberoRobot from a pi0.5 policy served over openpi's websocket.

    LIBERO-specific on purpose: it reads `robot.obs` and steps `robot.env`
    directly, because the policy consumes the simulator's own observation keys
    and emits 7-DoF actions in the environment's action space. Nothing here is
    reusable on the hardware backend, and pretending otherwise would produce a
    class that works on neither.
    """

    def __init__(self, ctx, host: str = "127.0.0.1", port: int = 8000,
                 replan_steps: int = DEFAULT_REPLAN_STEPS) -> None:
        from openpi_client import websocket_client_policy  # noqa: PLC0415

        self.robot = _unwrap(ctx.robot)
        if not hasattr(self.robot, "env") or not hasattr(self.robot, "obs"):
            raise RuntimeError(
                "the pi0.5 backend drives a LIBERO environment directly and this "
                f"robot is a {type(self.robot).__name__}")
        self.log = ctx.log
        self.client = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
        self.replan_steps = int(replan_steps)
        self._plan: deque = deque()
        self._prompt: Optional[str] = None
        self.steps = 0
        self.infers = 0

    # -- the duck-typed contract the vla skill uses -------------------------
    def start(self, instruction: str) -> None:
        self._prompt = instruction
        self._plan.clear()
        self.steps = 0
        self.infers = 0
        self.log.event("openpi_rollout_start", instruction=instruction,
                       replan_steps=self.replan_steps)

    @property
    def finished(self) -> bool:
        """The environment will accept no more actions.

        LIBERO stops at its horizon and then refuses. Without this the caller
        spins for its whole time budget calling step() on a dead environment
        and paying for a progress check every two seconds — measured, rollouts
        logging `steps: 0, infers: 0` and still costing thirty seconds each.
        """
        return bool(getattr(self.robot, "terminated", False))

    def step(self) -> None:
        if self._prompt is None:
            raise RuntimeError("start(instruction) before step()")
        if self.finished:
            return
        if not self._plan:
            chunk = self.client.infer(self._observation())["actions"]
            self.infers += 1
            self._plan.extend(np.asarray(chunk)[: self.replan_steps])
        action = self._plan.popleft()
        # Straight through to the environment, not through move_cartesian: the
        # policy emits an action in the env's own space, and re-interpreting it
        # as a cartesian target would be a different controller entirely.
        self.robot.obs, _r, done, _info = self.robot.env.step(
            np.asarray(action, dtype=float).tolist())
        self.robot.sim_steps += 1
        self.steps += 1
        if done:
            self.robot.terminated = True
        # Keep the film and the benchmark verdict in step with the scripted path.
        record_every = getattr(self.robot, "_record_every", 0)
        if record_every and self.robot.sim_steps % record_every == 0:
            view = self.robot.obs.get("agentview_image")
            if view is not None:
                self.robot.frames.append(np.asarray(view, dtype=np.uint8)[::-1].copy())
        if self.robot.sim_steps % 5 == 0 or done:
            try:
                if bool(self.robot.env.check_success()):
                    self.robot.task_success = True
            except Exception:
                pass

    def stop(self) -> None:
        self.log.event("openpi_rollout_stop", steps=self.steps, infers=self.infers,
                       instruction=self._prompt)
        self._plan.clear()
        self._prompt = None

    # -- observation --------------------------------------------------------
    def _observation(self) -> dict:
        from openpi_client import image_tools  # noqa: PLC0415

        obs = self.robot.obs
        # 180 degrees, both axes. robosuite renders bottom-up and openpi's
        # LIBERO preprocessing assumes the doubly-flipped view; the scripted
        # path's single flip is for OUR deprojection maths and is not the same
        # image. Feeding the policy the wrong one is not an error anywhere, it
        # just quietly degrades everything it does.
        img = np.ascontiguousarray(obs["agentview_image"][::-1, ::-1])
        wrist = np.ascontiguousarray(obs["robot0_eye_in_hand_image"][::-1, ::-1])
        return {
            "observation/image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(img, RESIZE_SIZE, RESIZE_SIZE)),
            "observation/wrist_image": image_tools.convert_to_uint8(
                image_tools.resize_with_pad(wrist, RESIZE_SIZE, RESIZE_SIZE)),
            "observation/state": np.concatenate((
                np.asarray(obs["robot0_eef_pos"], dtype=float),
                quat_to_axisangle(obs["robot0_eef_quat"]),
                np.asarray(obs["robot0_gripper_qpos"], dtype=float),
            )),
            "prompt": self._prompt,
        }


def _unwrap(robot):
    """Reach the backend through SafeRobot's wrapper."""
    inner = getattr(robot, "_robot", None)
    return inner if inner is not None else robot
