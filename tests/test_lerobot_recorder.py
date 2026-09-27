"""The recorder's one job: a dataset lerobot itself will load and vouch for.

These tests drive the MuJoCo twin through scripted motion (no model calls),
record it at control rate, and then hold the result to the standard that
matters — not "did our writer run" but "does lerobot's own loader read it
back": aligned image/state/action frames, monotonic uniform timestamps, and
the verdict sidecar that the success filter feeds on.
"""
from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco", reason="the twin needs MuJoCo")
pytest.importorskip("lerobot", reason="recording needs the lerobot package")

from heron.config import HeronConfig  # noqa: E402
from heron.record import EpisodeRecorder, LeRobotWriter  # noqa: E402
from heron.record.writer import read_sidecar  # noqa: E402

CONFIG = "configs/trossen_sorting.yaml"
HW = (120, 160)      # small frames keep the suite fast; the shape is what matters
FPS = 30


@pytest.fixture(scope="module")
def sim():
    from heron.robot.trossen_sim import TrossenSim

    cfg = HeronConfig.load(CONFIG)
    s = TrossenSim(cfg, scene=cfg.sim_scene, record_camera=None)
    yield s
    s.shutdown()


@pytest.fixture(scope="module")
def dataset_root(sim, tmp_path_factory):
    """Two scripted episodes: one marked a verified success, one a failure."""
    root = tmp_path_factory.mktemp("lerobot") / "ds"
    writer = LeRobotWriter(root, "heron/twin-test",
                           cameras={c: HW for c in sim.cameras}, fps=FPS,
                           use_videos=False)
    rec = EpisodeRecorder(sim, writer, arm="right")
    for ep, verdict in enumerate((True, False)):
        rec.start(f"put the red block on the white plate (episode {ep})")
        sim.move_cartesian("right", np.array([0.30, 0.10, 0.06]), seconds=0.5)
        sim.set_gripper("right", 0.02)
        sim.move_cartesian("right", np.array([0.26, -0.04, 0.10]), seconds=0.5)
        index = rec.stop(verified_success=verdict,
                         verifier_status="succeeded" if verdict else "failed",
                         gt_on_support=verdict, gt_centred=verdict)
        assert index == ep
    writer.finalize()
    return root


def _load(root):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    return LeRobotDataset("heron/twin-test", root=root)


def test_lerobot_loads_the_dataset_back(dataset_root):
    ds = _load(dataset_root)
    assert ds.meta.total_episodes == 2
    assert ds.num_frames > 0
    assert int(round(float(ds.fps))) == FPS


def test_every_frame_is_aligned_and_complete(dataset_root, sim):
    """Each sample carries every camera plus 7-dof state AND action — a frame
    missing a modality is exactly the misalignment fine-tuning cannot survive."""
    ds = _load(dataset_root)
    item = ds[0]
    for cam in sim.cameras:
        key = f"observation.images.{cam}"
        assert key in item, f"frame 0 lacks {key}"
        assert tuple(item[key].shape[-2:]) == HW or tuple(item[key].shape[:2]) == HW
    assert tuple(item["observation.state"].shape) == (7,)
    assert tuple(item["action"].shape) == (7,)


def test_timestamps_are_monotonic_and_uniform(dataset_root):
    ds = _load(dataset_root)
    per_episode: dict[int, list[float]] = {}
    for i in range(ds.num_frames):
        item = ds[i]
        per_episode.setdefault(int(item["episode_index"]), []).append(
            float(item["timestamp"]))
    assert set(per_episode) == {0, 1}
    for ep, ts in per_episode.items():
        assert all(b > a for a, b in zip(ts, ts[1:])), f"episode {ep} not monotonic"
        gaps = np.diff(ts)
        assert np.allclose(gaps, 1.0 / FPS, atol=1e-4), f"episode {ep} not uniform"


def test_action_is_the_commanded_target_not_the_measurement(dataset_root, sim):
    """The last commanded gripper aperture was 0.02 m; the action stream must
    say so even though the measured jaws sit wherever physics left them."""
    ds = _load(dataset_root)
    last = ds[ds.num_frames - 1]
    assert abs(float(last["action"][6]) - 0.02) < 1e-6
    # and the arm-joint action equals the final interpolation target the sim
    # was driving toward, which by episode end the joints have converged to
    assert np.allclose(np.asarray(last["action"][:6]),
                       np.asarray(last["observation.state"][:6]), atol=0.05)


def test_the_task_string_rides_with_the_frames(dataset_root):
    ds = _load(dataset_root)
    assert "red block" in str(ds[0]["task"])


def test_sidecar_carries_the_verdicts(dataset_root):
    rows = read_sidecar(dataset_root)
    assert [r["episode_index"] for r in rows] == [0, 1]
    assert rows[0]["verified_success"] is True
    assert rows[1]["verified_success"] is False
    assert rows[0]["frames"] > 0
    assert "task" in rows[0]


def test_success_filter_exports_only_the_verified_episode(dataset_root, tmp_path):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path("tools")))
    import filter_success

    dst = tmp_path / "subset"
    result = filter_success.export(dataset_root, dst, require="both",
                                   use_videos=False)
    assert result["kept"] == 1
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    sub = LeRobotDataset("subset", root=dst)
    assert sub.meta.total_episodes == 1
    rows = read_sidecar(dst)
    assert rows[0]["verified_success"] is True and rows[0]["source_episode"] == 0


def test_a_motionless_episode_is_discarded_not_saved(sim, tmp_path):
    """An agent that aborts before the arm moves leaves a near-empty episode.
    Saving it once BROKE every later save: lerobot types single-frame stats as
    ints and multi-frame stats as floats, and parquet refuses the mix."""
    writer = LeRobotWriter(tmp_path / "ds3", "heron/short-test",
                           cameras={sim.cameras[0]: HW}, fps=FPS, use_videos=False)
    rec = EpisodeRecorder(sim, writer, arm="right", cameras=[sim.cameras[0]])
    rec.start("aborted before anything moved")
    # no motion: only the start-of-episode sample lands
    assert rec.stop(verifier_status="aborted") is None
    assert writer.num_episodes == 0
    # and a real episode afterwards still saves cleanly
    rec.start("a real one")
    sim.move_cartesian("right", np.array([0.30, 0.05, 0.08]), seconds=0.5)
    assert rec.stop(verifier_status="failed") == 0
    writer.finalize()


def test_abort_discards_the_episode(sim, tmp_path):
    writer = LeRobotWriter(tmp_path / "ds2", "heron/abort-test",
                           cameras={sim.cameras[0]: HW}, fps=FPS, use_videos=False)
    rec = EpisodeRecorder(sim, writer, arm="right", cameras=[sim.cameras[0]])
    rec.start("doomed episode")
    sim.move_cartesian("right", np.array([0.30, 0.05, 0.08]), seconds=0.3)
    rec.abort()
    assert writer.num_episodes == 0
    assert not rec.recording
    # a listener left behind would keep rendering forever
    assert rec._on_sim_step not in sim.step_listeners
