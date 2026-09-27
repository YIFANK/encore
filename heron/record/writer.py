"""LeRobotDataset writer, insulated from lerobot's moving API.

lerobot has renamed its dataset module and reshaped `add_frame`/`save_episode`
between releases (task-in-frame vs task-as-argument; save_episode with and
without arguments). Everything version-shaped lives HERE, resolved once by
inspection of the installed package, so the recorder and the tools above it
write one shape of data and this file argues with the library.

The sidecar: LeRobotDataset has no episode-level field for "did the verifier
believe this one", and inventing a per-frame feature for it would bake a
Heron-ism into a dataset other tools consume. So per-episode metadata (verdict,
ground truth, instruction) goes to `heron_meta.jsonl` next to the dataset's own
meta directory, keyed by episode_index. tools/filter_success.py reads it back.
"""
from __future__ import annotations

import inspect
import json
from pathlib import Path
from typing import Any, Optional

import numpy as np

JOINT_NAMES = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5",
               "gripper"]
SIDECAR = "heron_meta.jsonl"


def _import_lerobot_dataset():
    try:  # lerobot >= 0.3 (v3 layout)
        from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: PLC0415
    except ImportError:  # older releases
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset  # noqa: PLC0415
    return LeRobotDataset


def image_features(cameras: dict[str, tuple[int, int]], use_videos: bool) -> dict:
    dtype = "video" if use_videos else "image"
    return {
        f"observation.images.{name}": {
            "dtype": dtype,
            "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        }
        for name, (h, w) in cameras.items()
    }


def dataset_features(cameras: dict[str, tuple[int, int]], use_videos: bool,
                     state_dim: int | None = None,
                     action_dim: int | None = None) -> dict:
    sd = state_dim or len(JOINT_NAMES)
    ad = action_dim or len(JOINT_NAMES)
    s_names = list(JOINT_NAMES) if sd == len(JOINT_NAMES) \
        else [f"s{i}" for i in range(sd)]
    a_names = list(JOINT_NAMES) if ad == len(JOINT_NAMES) \
        else [f"a{i}" for i in range(ad)]
    return {
        "observation.state": {"dtype": "float32", "shape": (sd,), "names": s_names},
        "action": {"dtype": "float32", "shape": (ad,), "names": a_names},
        **image_features(cameras, use_videos),
    }


class LeRobotWriter:
    """Create-or-resume a LeRobotDataset and append episodes to it.

    `cameras` maps camera name -> (height, width) of the recorded stream.
    """

    def __init__(self, root: str | Path, repo_id: str,
                 cameras: dict[str, tuple[int, int]], fps: int = 30,
                 use_videos: bool = True, robot_type: str = "trossen_wxai",
                 state_dim: int | None = None, action_dim: int | None = None) -> None:
        self.root = Path(root)
        self.repo_id = repo_id
        self.fps = int(fps)
        self.cameras = dict(cameras)
        self.use_videos = bool(use_videos)
        self.robot_type = robot_type
        LeRobotDataset = _import_lerobot_dataset()
        if (self.root / "meta").exists():
            # Resuming an interrupted collection: append to what is there.
            # Newer releases split "open to read" from "open to append";
            # resume() is the append path and never asks the hub about a
            # dataset that only exists on this disk. Releases without
            # resume() (0.5.x) hub-check inside the constructor and die on
            # a local-only repo_id — for those, rotate to a fresh sibling
            # root (the rig's long-standing dp1/dp2/... workaround) instead
            # of failing the collection.
            try:
                resume = getattr(LeRobotDataset, "resume", None)
                self.ds = (resume(repo_id, root=self.root) if callable(resume)
                           else LeRobotDataset(repo_id, root=self.root))
            except Exception as e:
                # ANY failure to resume a local-only dataset (hub 401, hub
                # unreachable, version-check network errors) must not kill the
                # collection — rotate to a fresh sibling root instead.
                n = 2
                while (self.root.parent / f"{self.root.name}_r{n}" / "meta").exists():
                    n += 1
                self.root = self.root.parent / f"{self.root.name}_r{n}"
                print(f"[writer] hub refused local-only resume ({type(e).__name__}); "
                      f"rotating to fresh root {self.root}", flush=True)
                self.ds = None
            if self.ds is not None:
                if int(round(float(self.ds.meta.fps))) != self.fps:
                    raise ValueError(
                        f"existing dataset at {self.root} runs at {self.ds.meta.fps} fps, "
                        f"not the requested {self.fps} — pass the matching --fps or a new root")
        else:
            self.ds = None
        if self.ds is None:
            create_kwargs = {}
            if "metadata_buffer_size" in inspect.signature(LeRobotDataset.create).parameters:
                # Flush episode metadata as it happens. The default buffers ten
                # episodes in memory, so a run interrupted by the estop — the
                # expected way a rig session ends early — would lose up to nine
                # recorded episodes' bookkeeping.
                create_kwargs["metadata_buffer_size"] = 1
            self.ds = LeRobotDataset.create(
                repo_id, self.fps, root=self.root, robot_type=robot_type,
                features=dataset_features(self.cameras, self.use_videos,
                                          state_dim=state_dim, action_dim=action_dim),
                use_videos=self.use_videos, **create_kwargs)
        # Resolve the installed API's shape once.
        self._task_in_add_frame = "task" in inspect.signature(self.ds.add_frame).parameters
        save_params = inspect.signature(self.ds.save_episode).parameters
        self._save_takes_task = "task" in save_params
        # Encode video in-process. The default process pool forks a Python that
        # has MuJoCo/torch loaded, which macOS kills — the episode then fails at
        # save, after every frame was already collected.
        self._save_kwargs = {"parallel_encoding": False} if "parallel_encoding" in save_params else {}

    # -- episode append ------------------------------------------------------
    @property
    def num_episodes(self) -> int:
        return int(self.ds.meta.total_episodes)

    def add_frame(self, state: np.ndarray, action: np.ndarray,
                  images: dict[str, np.ndarray], task: str) -> None:
        frame: dict[str, Any] = {
            "observation.state": np.asarray(state, dtype=np.float32),
            "action": np.asarray(action, dtype=np.float32),
        }
        for name, rgb in images.items():
            frame[f"observation.images.{name}"] = np.ascontiguousarray(rgb)
        if self._task_in_add_frame:
            self.ds.add_frame(frame, task=task)
        else:
            frame["task"] = task
            self.ds.add_frame(frame)

    def save_episode(self, task: str, meta: Optional[dict] = None) -> int:
        """Close the buffered episode; returns its episode_index."""
        index = self.num_episodes
        if self._save_takes_task:
            self.ds.save_episode(task=task, **self._save_kwargs)
        else:
            self.ds.save_episode(**self._save_kwargs)
        if meta is not None:
            self.append_sidecar(index, {"task": task, **meta})
        return index

    def finalize(self) -> None:
        """Flush everything to disk so the dataset loads standalone.

        Newer lerobot buffers episode metadata and only writes it here; a
        dataset that skipped this step re-loads by asking the Hugging Face hub
        for a repo that does not exist. Call once, when collection ends."""
        fn = getattr(self.ds, "finalize", None)
        if callable(fn):
            fn()

    def discard_episode(self) -> None:
        """Drop the frames buffered since the last save (aborted episode)."""
        clear = getattr(self.ds, "clear_episode_buffer", None)
        if callable(clear):
            clear()
        else:  # very old releases kept a plain dict buffer
            self.ds.episode_buffer = self.ds.create_episode_buffer()

    # -- sidecar -------------------------------------------------------------
    @property
    def sidecar_path(self) -> Path:
        return self.root / SIDECAR

    def append_sidecar(self, episode_index: int, meta: dict) -> None:
        row = {"episode_index": int(episode_index), **meta}
        with self.sidecar_path.open("a") as fh:
            fh.write(json.dumps(row) + "\n")


def read_sidecar(root: str | Path) -> list[dict]:
    path = Path(root) / SIDECAR
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
