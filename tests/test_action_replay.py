"""A dataset nobody can replay is a dataset nobody can learn from.

Round 0 trained two policies to a loss of 0.09 and both scored 0/20. The cause
was not the policies: replaying a recorded episode's own commands, from the
recorded world state, also failed — so no policy trained on that data could
have worked, however well it fitted.

This pins the property the whole loop rests on: what `joint_command` records
and what `apply_joint_command` executes are the same signal, and a trajectory
played back through it reproduces the trajectory it came from.
"""
from __future__ import annotations

import numpy as np

from heron.config import HeronConfig
from heron.robot.trossen_sim import TrossenSim


def _sim(tmp_path):
    cfg = HeronConfig.load("configs/trossen_sorting.yaml")
    cfg.episodes_dir = str(tmp_path / "ep")
    return TrossenSim(cfg, scene=cfg.sim_scene, record_camera=None)


def test_a_held_pose_stays_held_when_its_own_command_is_replayed(tmp_path):
    """ctrlrange is only a range when ctrllimited says so.

    On these six actuators the flag is off and the range reads [0, 0], so
    clipping to it commanded every joint to zero: the arm dropped whatever it
    held and swung to the zero pose on the first tick of every rollout.
    """
    sim = _sim(tmp_path)
    sim.home("right")
    for _ in range(50):
        sim._step()
    held = sim.joint_state("right")[:6].copy()
    cmd = sim.joint_command("right").copy()
    for _ in range(30):                      # a second of holding still
        sim.apply_joint_command("right", cmd, seconds=1 / 30)
    drift = np.degrees(np.max(np.abs(sim.joint_state("right")[:6] - held)))
    assert drift < 2.0, f"the arm moved {drift:.1f} deg while holding its own command"
    sim.shutdown()


def test_recorded_commands_reproduce_the_motion_they_recorded(tmp_path):
    """The round trip, on a real motion rather than a held pose."""
    sim = _sim(tmp_path)
    sim.home("right")
    for _ in range(50):
        sim._step()
    start = (sim.data.qpos.copy(), sim.data.qvel.copy(), sim.data.ctrl.copy())

    period = max(1, round(1.0 / (30 * sim.model.opt.timestep)))
    trace: list[tuple[np.ndarray, np.ndarray]] = []
    n = {"i": 0}

    def tap() -> None:
        n["i"] += 1
        if n["i"] % period == 0:
            trace.append((sim.joint_command("right").copy(),
                          sim.joint_state("right").copy()))

    sim.step_listeners.append(tap)
    sim.move_cartesian("right", np.array([0.30, 0.05, 0.10]), seconds=2.0)
    sim.step_listeners.remove(tap)
    assert len(trace) > 20, "the tap recorded nothing to replay"

    sim.data.qpos[:], sim.data.qvel[:], sim.data.ctrl[:] = start
    sim._mj.mj_forward(sim.model, sim.data)
    worst = 0.0
    for cmd, was in trace:
        sim.apply_joint_command("right", cmd, seconds=1 / 30)
        worst = max(worst, float(np.max(np.abs(sim.joint_state("right")[:6] - was[:6]))))
    assert np.degrees(worst) < 3.0, \
        f"replay diverged from its own recording by {np.degrees(worst):.1f} deg"
    sim.shutdown()
