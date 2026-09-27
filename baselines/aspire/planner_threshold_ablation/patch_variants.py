#!/usr/bin/env python3
"""Apply the two single-factor ablations to the ASPIRE variant checkouts on the cluster.
P: no grasp planner (registry + brief + skill note).  T: no gripper-width thresholds in the brief."""
import pathlib
base = pathlib.Path("/mnt/data/YifanKang")
P = base / "ASPIRE_ablP/aspire/sim"
T = base / "ASPIRE_ablT/aspire/sim"

def edit(path, pairs):
    s = path.read_text()
    for a, b in pairs:
        assert s.count(a) == 1, f"{path}: expected exactly one match for {a[:60]!r}, got {s.count(a)}"
        s = s.replace(a, b)
    path.write_text(s)

# ---- P
edit(P / "cap/integrations/franka/libero_reduced.py", [
    ('        fns["plan_grasp"] = self.plan_grasp\n', ""),
    ('        fns["plan_grasp_from_point_clouds"] = self.plan_grasp_from_point_clouds\n', ""),
])
edit(P / "cap/integrations/franka/libero_reduced_skill_library.py", [
    ('        fns["select_top_down_grasp"] = self.select_top_down_grasp\n', ""),
])
edit(P / ".claude/libero/fix-loop/subagent-prompt.md", [
    ("  plan_grasp(depth, intrinsics, mask)        → (grasp_poses, grasp_scores) in camera frame\n", ""),
    ("  select_top_down_grasp(poses, scores, E)   → best grasp 4×4 world-frame matrix\n", ""),
    ("  decompose_transform(T)                    → (position, quaternion_wxyz)\n",
     "  decompose_transform(T)                    → (position, quaternion_wxyz)\n"
     "  (There is NO grasp planner in this environment: no plan_grasp, no\n"
     "   select_top_down_grasp. Derive grasp positions and orientations yourself\n"
     "   from masks, point clouds and bounding boxes, then command them with\n"
     "   solve_ik + move_to_joints or goto_pose.)\n"),
])
g = P / ".claude/libero/skills/grasp.md"
note = ("> NOTE (this environment): plan_grasp and select_top_down_grasp are NOT available here. "
        "Snippets below that call them are historical; derive the grasp pose from the object point cloud / OBB yourself.\n\n")
if not g.read_text().startswith("> NOTE (this environment)"):
    g.write_text(note + g.read_text())

# ---- T
edit(T / ".claude/libero/fix-loop/subagent-prompt.md", [
    ("      close_gripper → gripper_width: >0.06 grasped, 0.03–0.06 marginal, <0.03 air grasp\n",
     "      close_gripper → gripper_width: the finger gap after closing; what gap means\n"
     "        a real grasp depends on the object, establish it from your own traces\n"),
])
edit(T / ".claude/libero/skills/grasp.md", [
    ("**Gripper width thresholds** — add validated per-object thresholds here as you discover them:\n\n"
     "| Object | Good grasp (gw >) | Air grasp (gw <) | Notes |\n|---|---|---|---|\n", ""),
])
print("patched P and T")
