"""VLA rollout skill: run a learned LeRobot policy as one step of the program.

Design differences from Pigey's VLARollout:
- The rollout is *monitored*: every couple of seconds the overhead frame goes to
  ER 2's progress/success detection, and the rollout stops early on success or
  regression instead of always burning the full step budget.
- Success is still decided by the program's postconditions afterwards — the
  monitor only controls when to stop, never what to believe.

Hardware/policy serving depends on the TrossenRobotics lerobot plugin
(`lerobot-robot-trossen`, robot type `bi_widowxai_follower_robot`); this module
keeps that behind a lazy import so mock-mode installs never need it.
"""
from __future__ import annotations

import time

from ..types import SkillResult, Value
from . import SkillContext, skill

MONITOR_PERIOD_S = 2.0


@skill(
    name="vla",
    description=(
        "Delegate a short-horizon subgoal to the learned VLA policy (closed-loop, bimanual-capable). "
        "Use for deformables, articulated objects, non-grasp verbs, and recovery when scripted "
        "primitives fail. Phrase the instruction short and concrete: one verb, named objects."
    ),
    params={
        "instruction": {"type": "string"},
        "max_seconds": {"type": "number"},
    },
    required=["instruction"],
    post_hint="whatever the instruction claims — ALWAYS pair with explicit postconditions; locations must be re-perceived",
)
def vla(ctx: SkillContext, instruction: str, max_seconds: float = 30.0) -> SkillResult:
    client = _policy_client(ctx)
    if client is None:
        return SkillResult(
            ok=False,
            error=(
                "vla: no policy backend available. Install the TrossenRobotics lerobot plugin and "
                "start a policy server (see README 'VLA policies'), or remove vla steps from the plan."
            ),
        )
    t0 = time.time()
    last_monitor = 0.0
    stop_reason = "timeout"
    client.start(instruction)
    try:
        while time.time() - t0 < max_seconds:
            if getattr(client, "finished", False):
                # The environment has stopped accepting actions. Continuing
                # burns the whole time budget and a progress check every two
                # seconds on a world that cannot change.
                stop_reason = "environment ended"
                break
            client.step()
            now = time.time()
            if now - last_monitor >= MONITOR_PERIOD_S:
                last_monitor = now
                frame = ctx.robot.capture("cam_high")
                ctx.log.frame(frame, tag="vla_monitor")
                verdict, conf, detail = ctx.grounder.vqa(
                    [frame], f"Has this subgoal been completed: {instruction!r}? Consider the scene only."
                )
                if verdict is Value.TRUE and conf >= 0.7:
                    stop_reason = "monitor_success"
                    break
    finally:
        client.stop()
    # A rollout scrambles anything it touched; force re-grounding before reuse.
    for tr in ctx.belief.entities.values():
        if not tr.held_by:
            ctx.belief.invalidate_entity(tr.id, reason="vla rollout ran; location stale")
    return SkillResult(ok=True, info=f"vla({instruction!r}) ran {time.time() - t0:.1f}s, stopped by {stop_reason}")


def _policy_client(ctx: SkillContext):
    """Duck-typed policy backend: object with start(instruction)/step()/stop().

    Resolution order: an injected test double on the robot (mock), then an
    openpi policy server if one is configured, then the LeRobot async-inference
    client on hardware.

    The openpi branch is what makes the simulator comparable to the published
    baselines at all: those numbers come from a frozen pi0.5 doing the
    manipulation, and without it every articulated task is being attempted with
    trigonometry we wrote by hand.
    """
    injected = getattr(ctx.robot, "vla_client", None) or getattr(getattr(ctx.robot, "_robot", None), "vla_client", None)
    if injected is not None:
        return injected
    # A checkpoint on this disk, before anything that needs a server. This is
    # the loop's own policy: written by tools/train_local.py an hour ago, run
    # in this process against the twin. Putting a gRPC hop between a file and
    # the simulator reading it would add a process to supervise and a port to
    # collide, and buy nothing.
    ckpt = getattr(ctx.cfg, "policy_checkpoint", None)
    if ckpt:
        try:
            from .act_backend import LocalActClient  # noqa: PLC0415

            return LocalActClient(ctx, ckpt, arm=(ctx.cfg.active_arms or ctx.robot.arms)[0])
        except Exception as e:
            ctx.log.event("policy_backend_unavailable", checkpoint=str(ckpt),
                          error=f"{type(e).__name__}: {e}")
    url = getattr(ctx.cfg, "policy_url", None)
    if url:
        try:
            from urllib.parse import urlparse  # noqa: PLC0415

            from .openpi_backend import OpenPiLiberoClient  # noqa: PLC0415

            u = urlparse(url if "//" in url else f"//{url}")
            return OpenPiLiberoClient(ctx, host=u.hostname or "127.0.0.1",
                                      port=u.port or 8000)
        except Exception as e:
            ctx.log.event("policy_backend_unavailable", url=url,
                          error=f"{type(e).__name__}: {e}")
    try:
        from .lerobot_backend import LeRobotAsyncClient  # noqa: PLC0415

        return LeRobotAsyncClient(ctx)
    except Exception:
        return None
