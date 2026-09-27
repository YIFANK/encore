"""Control-rate episode recorder over a Trossen backend (real or twin).

What a frame holds:
  observation.state   measured joints (6) + gripper aperture (m)
  action              the joint targets the backend is currently driving toward
                      (6) + commanded gripper aperture (0.0 for a force close)
  observation.images.<cam>  RGB from each recorded camera

Two clocks, because the twin and the rig disagree about what "30 Hz" means:

  * The twin runs on SIMULATED time — wall-clock sampling would space frames by
    however fast MuJoCo happens to step, which varies with rendering load. So
    in sim the recorder rides the physics: a step listener fires every
    mj_step, and the recorder keeps one sample per 1/fps of SIM time. Samples
    land on the thread that moves the robot, so there is no cross-thread
    access to MjData at all.

  * The rig runs on wall time, and its driver getters are served from the
    controller's own feedback stream, so a background thread samples them at
    fps while the agent's blocking motion call occupies the main thread.
    Camera capture goes through the same depth-sidecar HTTP client the agent
    uses; contention is a queue, not a crash.

Timestamps are the dataset's own (frame_index / fps): monotonic and uniform by
construction, which is what the loader validates. The recorder counts how long
each real-mode tick actually took and reports the achieved rate at stop() —
a dataset that says 30 Hz and was sampled at 11 is worse than one that admits
its rate, so a big shortfall is surfaced rather than absorbed.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from .writer import LeRobotWriter

DEFAULT_FPS = 30
# Below this fraction of the nominal rate the recording is misdescribed by its
# own fps and the operator should lower --fps instead.
ACHIEVED_RATE_WARN = 0.9
# Episodes shorter than this many SECONDS are discarded rather than saved.
# Two reasons. As data: an episode where the agent aborted before the arm
# moved teaches nothing. As format: lerobot computes single-frame episode
# stats down an integer-typed code path and every longer episode down a
# float-typed one, and parquet refuses the mixed schema — one degenerate
# episode then poisons every save after it.
MIN_EPISODE_S = 0.5


class BackendTap:
    """Uniform state/action/image access over TrossenSim and TrossenStationary."""

    def __init__(self, backend, arm: str = "right") -> None:
        self.backend = backend
        self.arm = arm
        self.is_sim = hasattr(backend, "model") and hasattr(backend, "step_listeners")

    def state(self) -> np.ndarray:
        return np.asarray(self.backend.joint_state(self.arm), dtype=np.float32)

    def action(self) -> np.ndarray:
        return np.asarray(self.backend.joint_command(self.arm), dtype=np.float32)

    def image(self, camera: str, hw: tuple[int, int]) -> np.ndarray:
        h, w = hw
        if self.is_sim:
            return self.backend.render_rgb(camera, w, h)
        rgb = self.backend.capture(camera).rgb
        if rgb.shape[:2] != (h, w):
            from PIL import Image  # noqa: PLC0415

            rgb = np.asarray(Image.fromarray(rgb).resize((w, h)))
        return rgb


class EpisodeRecorder:
    """start() ... stop() around each episode; frames go straight to the writer."""

    def __init__(self, backend, writer: LeRobotWriter, arm: str = "right",
                 cameras: Optional[list[str]] = None) -> None:
        self.writer = writer
        self.tap = BackendTap(backend, arm)
        self.cameras = list(cameras if cameras is not None else writer.cameras)
        missing = [c for c in self.cameras if c not in writer.cameras]
        if missing:
            raise ValueError(f"cameras {missing} have no (height, width) in the writer")
        self.fps = writer.fps
        self._task: Optional[str] = None
        self._frames_added = 0
        self._errors = 0
        # sim clock
        self._steps_per_sample = 1
        self._steps_since = 0
        # real clock
        self._thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self._t_started = 0.0

    @property
    def recording(self) -> bool:
        return self._task is not None

    # -- lifecycle -----------------------------------------------------------
    def start(self, task: str) -> None:
        if self.recording:
            raise RuntimeError("already recording; stop() or abort() first")
        self._task = str(task)
        self._frames_added = 0
        self._errors = 0
        self._t_started = time.time()
        if self.tap.is_sim:
            timestep = float(self.tap.backend.model.opt.timestep)
            self._steps_per_sample = max(1, round(1.0 / (self.fps * timestep)))
            self._steps_since = 0
            self.tap.backend.step_listeners.append(self._on_sim_step)
            self._sample()          # frame 0: the scene as the episode found it
        else:
            self._stop_flag.clear()
            self._thread = threading.Thread(target=self._wall_clock_loop,
                                            name="heron-recorder", daemon=True)
            self._thread.start()

    def stop(self, save: bool = True, **meta) -> Optional[int]:
        """End the episode. Returns the episode_index, or None when discarded.

        `meta` lands in the sidecar: verifier verdict, ground truth, whatever
        the caller can attest to. An episode with no frames cannot be saved and
        is discarded with a note instead — an empty episode teaches nothing and
        breaks the loader.
        """
        if not self.recording:
            raise RuntimeError("not recording")
        task = self._task
        self._task = None
        if self.tap.is_sim:
            try:
                self.tap.backend.step_listeners.remove(self._on_sim_step)
            except ValueError:
                pass
        elif self._thread is not None:
            self._stop_flag.set()
            self._thread.join(timeout=5.0)
            self._thread = None
        elapsed = max(time.time() - self._t_started, 1e-6)
        min_frames = max(2, int(MIN_EPISODE_S * self.fps))
        if not save or self._frames_added < min_frames:
            if save and self._frames_added:
                print(f"[recorder] discarding a {self._frames_added}-frame episode "
                      f"(< {min_frames}): nothing happened worth training on",
                      flush=True)
            self.writer.discard_episode()
            return None
        achieved = self._frames_added / elapsed
        if not self.tap.is_sim and achieved < ACHIEVED_RATE_WARN * self.fps:
            print(f"[recorder] achieved {achieved:.1f} Hz against a nominal "
                  f"{self.fps} — lower --fps until these agree, or the dataset "
                  f"misdescribes its own timing", flush=True)
        return self.writer.save_episode(task, meta={
            **meta,
            "frames": self._frames_added,
            "sample_errors": self._errors,
            "achieved_fps": round(achieved, 2),
            "wall_s": round(elapsed, 1),
        })

    def abort(self) -> None:
        """Stop and discard — for episodes that crashed before they were data."""
        if self.recording:
            self.stop(save=False)

    # -- sampling ------------------------------------------------------------
    def _sample(self) -> None:
        try:
            state = self.tap.state()
            action = self.tap.action()
            images = {c: self.tap.image(c, self.writer.cameras[c])
                      for c in self.cameras}
            self.writer.add_frame(state, action, images, task=self._task or "")
            self._frames_added += 1
        except Exception as e:
            # One bad sample must not end an episode that the arm is still
            # executing; a run of them is reported at stop().
            self._errors += 1
            if self._errors <= 3:
                print(f"[recorder] sample failed: {type(e).__name__}: {e}", flush=True)

    def _on_sim_step(self) -> None:
        self._steps_since += 1
        if self._steps_since >= self._steps_per_sample:
            self._steps_since = 0
            self._sample()

    def _wall_clock_loop(self) -> None:
        period = 1.0 / self.fps
        next_t = time.monotonic()
        while not self._stop_flag.is_set():
            self._sample()
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                self._stop_flag.wait(delay)
            else:
                next_t = time.monotonic()   # fell behind: don't burst to catch up
