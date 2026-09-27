#!/usr/bin/env python3
"""Add gripper joint states to RoboDojo observations when observation.robot.gripper_states is true.
Idempotent. One of two RoboDojo-side code changes Encore needs; the other is
patch_pipeline_evalnum.py (evaluation-count override). Both run from install_encore.sh."""
import sys, pathlib
p = pathlib.Path(sys.argv[1]) / "env/observation_manager/obs_manager.py"
s = p.read_text()
if "gripper_states" in s:
    print("obs_manager already patched"); sys.exit(0)
anchor = """                    if self.robot_cfg["world_ee_state"]:
                        endpose = self.robot_manager.get_real_endpose(robot=robot, env_idx_list=env_idx_list)"""
assert s.count(anchor) == 1, "anchor not found; obs_manager.py changed upstream"
add = """                    if self.robot_cfg.get("gripper_states", False) and robot.robot_type == "arm":
                        gj = self.robot_manager.get_end_effector_real_val(robot, env_idx_list=env_idx_list)
                        for env_idx in env_idx_list:
                            name = robot.arm_name.split("_")[0]
                            obs[env_idx]["state"][f"{name}_gripper_joint_state"] = gj[env_idx]

"""
s = s.replace(anchor, add + anchor)
p.write_text(s)
print("obs_manager patched")
