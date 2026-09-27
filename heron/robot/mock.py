"""A tabletop mock of the stationary kit: kinematic arms, PIL-rendered cameras,
synthetic aligned depth, and injectable failures.

The mock exists so the *entire* agent loop — grounding, verification, diagnosis,
repair — runs and is testable without hardware. Rendered frames are real images,
so the mock also works under the real Gemini orchestrator for prompt debugging.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw

from ..types import Frame

TABLE_COLOR = (208, 188, 158)
GRASP_RADIUS = 0.05  # how close the tool must be to an object to grab it
HELD_EFFORT = 3.2  # proxy N·m the gripper reports when squeezing an object


@dataclass
class MockObject:
    name: str
    kind: str  # block | bowl | cup | plush
    color: tuple[int, int, int]
    xyz: np.ndarray  # center position, world frame
    size: tuple[float, float, float]  # dx, dy, dz
    graspable: bool = True

    @property
    def top_z(self) -> float:
        return float(self.xyz[2] + self.size[2] / 2)

    def footprint_contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            abs(x - self.xyz[0]) <= self.size[0] / 2 + margin
            and abs(y - self.xyz[1]) <= self.size[1] / 2 + margin
        )


@dataclass
class _ArmState:
    xyz: np.ndarray
    gripper_width: float = 0.08
    holding: Optional[str] = None
    commanded_width: float = 0.08


@dataclass
class _Injections:
    grasp_misses: int = 0  # next N grasp attempts close on air
    drop_next_lift: bool = False  # next lift while holding drops the object
    freeze_captures: dict[str, int] = field(default_factory=dict)  # camera -> N stale frames


class MockRobot:
    """World frame: origin at table center, x forward, y left, z up, table top z=0."""

    def __init__(self, image_size: tuple[int, int] = (640, 480)) -> None:
        self.arms = ["left", "right"]
        self.cameras = ["cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist"]
        self.image_w, self.image_h = image_size
        self.objects: dict[str, MockObject] = {}
        self._arms = {
            "left": _ArmState(xyz=np.array([0.0, 0.25, 0.25])),
            "right": _ArmState(xyz=np.array([0.0, -0.25, 0.25])),
        }
        self.inject = _Injections()
        self._stale_frames: dict[str, Frame] = {}
        self.motion_log: list[str] = []

    # -- world authoring (tests / demo) -------------------------------------
    def add_object(self, name: str, kind: str, color: tuple[int, int, int], xyz, size) -> MockObject:
        obj = MockObject(name=name, kind=kind, color=color, xyz=np.array(xyz, dtype=float), size=tuple(size))
        self.objects[name] = obj
        return obj

    def teleport(self, name: str, xyz) -> None:
        """External drift: something other than the robot moved an object."""
        obj = self.objects[name]
        obj.xyz = np.array(xyz, dtype=float)
        holder = next((a for a in self._arms.values() if a.holding == name), None)
        if holder:
            holder.holding = None

    def gt_xyz(self, name: str) -> np.ndarray:
        return self.objects[name].xyz.copy()

    def supporting_container(self, name: str) -> Optional[str]:
        obj = self.objects[name]
        for other in self.objects.values():
            if other.name != name and other.kind == "bowl" and other.footprint_contains(obj.xyz[0], obj.xyz[1]):
                if obj.xyz[2] < other.top_z + 0.02:
                    return other.name
        return None

    # -- RobotInterface ------------------------------------------------------
    def home(self, arm: str) -> None:
        side = 0.25 if arm == "left" else -0.25
        self.move_cartesian(arm, np.array([0.0, side, 0.25]))

    def move_cartesian(self, arm: str, xyz: np.ndarray, seconds: float = 2.0,
                       orientation=None) -> None:
        # `orientation` is accepted (a yawed grasp from the proposal service)
        # and ignored: the mock has no wrist to turn.
        st = self._arms[arm]
        start_z = st.xyz[2]
        st.xyz = np.array(xyz, dtype=float)
        self.motion_log.append(f"{arm} -> {np.round(st.xyz, 3).tolist()}")
        if st.holding:
            if self.inject.drop_next_lift and st.xyz[2] > start_z + 0.03:
                # The object slips out mid-transit and lands under the release point.
                dropped = self.objects[st.holding]
                dropped.xyz = np.array([st.xyz[0], st.xyz[1], self._support_z(st.xyz[0], st.xyz[1], dropped)])
                st.holding = None
                st.gripper_width = 0.0
                self.inject.drop_next_lift = False
            else:
                held = self.objects[st.holding]
                held.xyz = st.xyz - np.array([0.0, 0.0, held.size[2] / 2])

    def get_cartesian(self, arm: str) -> np.ndarray:
        return self._arms[arm].xyz.copy()

    def set_gripper(self, arm: str, width_m: float, effort_limit: float | None = None) -> None:
        st = self._arms[arm]
        st.commanded_width = width_m
        if width_m >= 0.05:  # opening
            if st.holding:
                obj = self.objects[st.holding]
                obj.xyz = np.array([st.xyz[0], st.xyz[1], self._support_z(st.xyz[0], st.xyz[1], obj)])
                st.holding = None
            st.gripper_width = width_m
            return
        # Closing: try to grab the nearest graspable object under the tool.
        candidate = self._grabbable_at(st.xyz)
        if candidate is not None and self.inject.grasp_misses > 0:
            self.inject.grasp_misses -= 1
            candidate = None
        if candidate is None:
            st.holding = None
            st.gripper_width = max(width_m, 0.0)
        else:
            st.holding = candidate.name
            st.gripper_width = min(candidate.size[0], candidate.size[1])
            candidate.xyz = st.xyz - np.array([0.0, 0.0, candidate.size[2] / 2])

    def get_gripper(self, arm: str) -> dict[str, float]:
        st = self._arms[arm]
        return {"width_m": st.gripper_width, "effort": HELD_EFFORT if st.holding else 0.05}

    def get_external_effort(self, arm: str) -> float:
        """Contact proxy: rises sharply once the tool — or the object it carries —
        presses against a surface. Baseline differs when carrying a load."""
        st = self._arms[arm]
        held = self.objects.get(st.holding) if st.holding else None
        # Height of the lowest point being lowered: the tool, or the held object's base.
        probe_z = float(st.xyz[2]) - (held.size[2] if held else 0.0)
        surface = 0.0
        for obj in self.objects.values():
            if held is not None and obj.name == held.name:
                continue
            if obj.footprint_contains(float(st.xyz[0]), float(st.xyz[1]), margin=0.01):
                surface = max(surface, obj.top_z if obj.kind != "bowl" else obj.xyz[2] - obj.size[2] / 2)
        baseline = 0.4 if held else 0.3
        penetration = surface - probe_z
        if penetration <= 0:
            return baseline
        # Contact is stiff: effort climbs sharply the moment something touches down.
        return min(20.0, baseline + 800.0 * penetration)

    def settle(self, seconds: float = 1.0) -> None:
        """No physics to advance in the mock; objects are already at rest."""
        return

    def stop(self) -> None:
        self.motion_log.append("STOP")

    def shutdown(self) -> None:
        self.motion_log.append("SHUTDOWN")

    # -- physics helpers -----------------------------------------------------
    def _grabbable_at(self, tool_xyz: np.ndarray) -> Optional[MockObject]:
        best, best_d = None, GRASP_RADIUS
        for obj in self.objects.values():
            if not obj.graspable:
                continue
            d = float(np.linalg.norm(obj.xyz[:2] - tool_xyz[:2]))
            near_top = abs(tool_xyz[2] - obj.top_z) < 0.08
            if d < best_d and near_top:
                best, best_d = obj, d
        return best

    def _support_z(self, x: float, y: float, obj: MockObject) -> float:
        """Resting height for obj released at (x, y): bowl interior, stack, or table."""
        base = 0.0
        for other in self.objects.values():
            if other.name == obj.name:
                continue
            if other.footprint_contains(x, y):
                if other.kind == "bowl":
                    base = max(base, float(other.xyz[2] - other.size[2] / 2) + 0.01)
                else:
                    base = max(base, other.top_z)
        return base + obj.size[2] / 2

    # -- cameras -------------------------------------------------------------
    def capture(self, camera: str) -> Frame:
        if self.inject.freeze_captures.get(camera, 0) > 0 and camera in self._stale_frames:
            self.inject.freeze_captures[camera] -= 1
            return self._stale_frames[camera]
        if camera == "cam_high":
            frame = self._render_topdown(center=(0.0, 0.0), height=0.85, fov_scale=1.0, camera=camera)
        elif camera == "cam_low":
            frame = self._render_topdown(center=(0.05, 0.0), height=0.75, fov_scale=1.0, camera=camera)
        elif camera in ("cam_left_wrist", "cam_right_wrist"):
            arm = "left" if camera == "cam_left_wrist" else "right"
            st = self._arms[arm]
            frame = self._render_topdown(
                center=(float(st.xyz[0]), float(st.xyz[1])), height=float(st.xyz[2]) + 0.12,
                fov_scale=2.2, camera=camera,
            )
        else:
            raise KeyError(f"unknown camera {camera!r}")
        self._stale_frames[camera] = frame
        return frame

    def _render_topdown(self, center: tuple[float, float], height: float, fov_scale: float, camera: str) -> Frame:
        w, h = self.image_w, self.image_h
        fx = fy = 520.0 * fov_scale
        cx, cy = w / 2, h / 2
        cam_pos = np.array([center[0], center[1], height])
        # Camera looks straight down: cam x = world -y, cam y = world -x, cam z = world -z.
        rot = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        t_base_cam = np.eye(4)
        t_base_cam[:3, :3] = rot
        t_base_cam[:3, 3] = cam_pos

        def project(p: np.ndarray) -> tuple[float, float, float]:
            rel = p - cam_pos
            xc, yc, zc = -rel[1], -rel[0], -rel[2]
            zc = max(zc, 1e-3)
            return cx + fx * xc / zc, cy + fy * yc / zc, zc

        img = Image.new("RGB", (w, h), TABLE_COLOR)
        draw = ImageDraw.Draw(img)
        depth = np.full((h, w), height, dtype=np.float32)

        def paint(obj_xyz: np.ndarray, size: tuple[float, float, float], color, kind: str) -> None:
            u, v, z = project(np.array([obj_xyz[0], obj_xyz[1], obj_xyz[2] + size[2] / 2]))
            su = fx * size[0] / z / 2
            sv = fy * size[1] / z / 2
            # Image u tracks world -y, v tracks world -x: size dy maps to u, dx to v.
            box = (u - fx * size[1] / z / 2, v - fy * size[0] / z / 2,
                   u + fx * size[1] / z / 2, v + fy * size[0] / z / 2)
            del su, sv
            if kind == "bowl":
                # Open container: tinted interior + thick rim, contents drawn after
                # (sorted by bottom z) stay visible — like a real bowl from above.
                draw.ellipse(box, fill=tuple(min(255, c + 90) for c in color), outline=color, width=8)
            elif kind == "cup":
                draw.ellipse(box, fill=color, outline=(30, 30, 30), width=2)
            elif kind == "plush":
                draw.ellipse(box, fill=color, outline=color, width=1)
            else:
                draw.rectangle(box, fill=color, outline=(30, 30, 30), width=2)
            x0, y0, x1, y1 = (int(max(0, box[0])), int(max(0, box[1])), int(min(w - 1, box[2])), int(min(h - 1, box[3])))
            if x1 > x0 and y1 > y0:
                if kind == "bowl":
                    # Depth: only the rim ring sits at the bowl top; the camera sees
                    # through the opening to whatever is inside (drawn later).
                    rim = max(2, (x1 - x0) // 10)
                    top = height - (obj_xyz[2] + size[2] / 2)
                    depth[y0:y0 + rim, x0:x1] = np.minimum(depth[y0:y0 + rim, x0:x1], top)
                    depth[y1 - rim:y1, x0:x1] = np.minimum(depth[y1 - rim:y1, x0:x1], top)
                    depth[y0:y1, x0:x0 + rim] = np.minimum(depth[y0:y1, x0:x0 + rim], top)
                    depth[y0:y1, x1 - rim:x1] = np.minimum(depth[y0:y1, x1 - rim:x1], top)
                else:
                    depth[y0:y1, x0:x1] = np.minimum(depth[y0:y1, x0:x1], height - (obj_xyz[2] + size[2] / 2))

        for obj in sorted(self.objects.values(), key=lambda o: float(o.xyz[2]) - o.size[2] / 2):
            held = any(st.holding == obj.name for st in self._arms.values())
            paint(obj.xyz, obj.size, obj.color, obj.kind)
            if held:  # draw a thin outline so a held object is visibly "in gripper"
                u, v, z = project(obj.xyz)
                r = fx * 0.02 / z
                draw.ellipse((u - r, v - r, u + r, v + r), outline=(20, 20, 20), width=2)

        # Grippers: dark T markers.
        for side, st in self._arms.items():
            u, v, z = project(st.xyz)
            r = max(4.0, fx * 0.012 / z)
            color = (40, 40, 40) if side == "left" else (70, 70, 70)
            draw.line((u - 2 * r, v, u + 2 * r, v), fill=color, width=3)
            draw.line((u, v, u, v + 2 * r), fill=color, width=3)

        intr = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        return Frame(camera=camera, rgb=np.asarray(img, dtype=np.uint8), depth=depth,
                     intrinsics=intr, t_base_cam=t_base_cam, t=time.time())


def standard_scene(robot: MockRobot) -> None:
    """The canonical demo scene: two blocks, a bowl, a distractor cup."""
    robot.add_object("red_block", "block", (200, 40, 40), (0.05, 0.10, 0.02), (0.04, 0.04, 0.04))
    robot.add_object("green_block", "block", (40, 160, 60), (-0.08, 0.18, 0.02), (0.04, 0.04, 0.04))
    robot.add_object("blue_bowl", "bowl", (40, 80, 200), (0.05, -0.15, 0.03), (0.14, 0.14, 0.06))
    robot.add_object("yellow_cup", "cup", (230, 200, 40), (-0.12, -0.05, 0.045), (0.07, 0.07, 0.09))


assert math.isfinite(GRASP_RADIUS)
