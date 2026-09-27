"""Recording: a control-rate tap on the robot's command/state stream.

The teleop rigs record what a human does with a leader arm; here the agent IS
the operator, so what gets recorded is the follower's own stream — measured
joint state and the joint targets the backend is currently driving toward —
plus camera frames, at a fixed rate, into a LeRobotDataset that Trossen's
openpi fork can fine-tune pi0.5 on.

Only verified successes teach (the repo's standing doctrine), but everything is
RECORDED: each episode carries the verifier's verdict and, in the twin, the
ground-truth landing in a sidecar, and the success-only subset is exported as a
separate filter step (tools/filter_success.py) rather than by throwing data
away at collection time.
"""
from .recorder import EpisodeRecorder
from .writer import LeRobotWriter

__all__ = ["EpisodeRecorder", "LeRobotWriter"]
