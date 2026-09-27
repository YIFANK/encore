"""The ground-truth sidecar must answer "did the drawer actually move".

Every drawer task in every LIBERO-PRO run so far has failed or aborted after
320-460 s, and the journal only records what the agent BELIEVED its open()
verdicts meant. gt_poses covers free bodies — where the bowl landed — but a
drawer is a slide joint on a fixed cabinet: no free joint, so no trace. The
autopsy needs the joint qpos next to the agent's verdicts, on the same clock.

gt_joints is offline evidence only: nothing in the decision stack reads it
(see the module comment in heron/robot/libero.py).
"""
from __future__ import annotations

import numpy as np

from heron.robot.libero import LiberoRobot


class _Model:
    njnt = 5
    #           free  slide  hinge  slide(robot)  slide(gripper)
    jnt_type = [0, 2, 3, 2, 2]
    jnt_qposadr = [0, 7, 8, 9, 10]
    _names = ["akita_black_bowl_joint", "cabinet_drawer_top_joint",
              "microwave_hinge", "robot0_slide", "gripper_finger_joint"]

    def joint_id2name(self, j):
        return self._names[j]


class _Data:
    qpos = np.array([0.0] * 7 + [0.132, 1.05, 0.3, 0.02])


class _Sim:
    model = _Model()
    data = _Data()


class _Env:
    sim = _Sim()


def test_gt_joints_reports_the_furniture_and_not_the_arm():
    robot = LiberoRobot.__new__(LiberoRobot)
    robot.env = _Env()
    joints = robot.gt_joints()
    # The drawer's slide and the microwave's hinge, at their qpos addresses.
    assert joints == {"cabinet_drawer_top_joint": 0.132, "microwave_hinge": 1.05}
    # The bowl's free joint belongs in gt_poses; the arm is not furniture.
    assert "akita_black_bowl_joint" not in joints
    assert not any(n.startswith(("robot", "gripper")) for n in joints)
