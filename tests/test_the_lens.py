"""The arm can only reach eleven percent of the picture we send.

Measured on the rig's calibrated overhead D405: the safety workspace projects to
138x251 px of a 640x480 frame, and a 32 mm block inside it is about 22 px across.
The twin's camera IS the real one's pose, intrinsics and field of view, so this
is a fact about the cell and not about the simulator.

What it cost was not detector noise. Asked to "pick up the red block and place it
on the white plate", the planner replied that the scene held "two blue blocks and
a gray plate" and aborted the task as unreachable — on a frame containing two red
blocks, two blue ones, a white plate and a grey tray, all obvious to a human.

The fix is to show the model the workspace instead of a table lost in a black
surround. What has to be exactly right is the way back: a detection is made
against the view and used against the frame, and an error there would put the
gripper somewhere plausible and wrong.
"""
from __future__ import annotations

import numpy as np

from heron.config import HeronConfig
from heron.orchestrator.gemini import (LENS_MAX_COVERAGE, _Lens, _box_from, _points_from,
                                       _to_px, workspace_box)
from heron.types import Frame

# The twin's overhead camera, from the rig's own calibration: a D405 at
# 640x480, fx=fy=391.86, looking down at the table from 1.03 m.
K = np.array([[391.86, 0.0, 320.0], [0.0, 391.86, 240.0], [0.0, 0.0, 1.0]])


def _rig() -> HeronConfig:
    """The sorting cell's own numbers, written out rather than loaded, so the
    measurement below stays fixed if the config file moves on."""
    cfg = HeronConfig()
    cfg.safety.workspace_x = (0.15, 0.48)
    cfg.safety.workspace_y = (-0.35, 0.35)
    cfg.table_z = -0.015
    return cfg


# base <- camera, read straight off the twin, which builds it from the same
# hand-eye calibration the real cell runs on.
T_BASE_CAM = np.array([[-0.999893, 0.001320, -0.014572, 0.48849],
                       [0.007043, 0.916363, -0.400285, 0.29784],
                       [0.012825, -0.400345, -0.916275, 1.03455],
                       [0.0, 0.0, 0.0, 1.0]])


def _overhead() -> Frame:
    return Frame(camera="cam_high", rgb=np.zeros((480, 640, 3), np.uint8),
                 intrinsics=K, t_base_cam=T_BASE_CAM.copy())


def test_the_reachable_workspace_is_a_small_part_of_the_frame():
    """The measurement this whole mechanism exists for."""
    box = workspace_box(_overhead(), _rig())
    assert box is not None, "a calibrated overhead frame must produce a crop"
    u0, v0, u1, v1 = box
    area = (u1 - u0) * (v1 - v0)
    assert area < LENS_MAX_COVERAGE * 640 * 480, (box, area / (640 * 480))


def test_an_uncalibrated_frame_is_sent_whole():
    """Nothing to project with, so nothing to crop by. Sending the whole picture
    is the honest fallback — a guessed crop could hide the object entirely."""
    bare = Frame(camera="c", rgb=np.zeros((480, 640, 3), np.uint8))
    assert workspace_box(bare, _rig()) is None


def test_a_view_that_already_fills_the_frame_is_left_alone():
    """Every wrist frame: looking at one object from 0.15 m, where the workspace
    covers everything and cropping would only throw away context."""
    f = _overhead()
    close = np.eye(4)
    close[:3, :3] = np.diag([1.0, -1.0, -1.0])
    close[:3, 3] = [0.30, 0.0, 0.25]
    f.t_base_cam = close
    assert workspace_box(f, _rig()) is None


def test_a_point_made_against_the_view_lands_where_it_was_meant_to():
    """The round trip. A detection is produced against the cropped, upscaled
    image and consumed against the full frame; an error here would send the
    gripper somewhere plausible and wrong."""
    lens = _Lens(np.zeros((400, 400, 3), np.uint8), u0=100, v0=50, factor=4)
    # The model points at the exact middle of what it was shown.
    (u, v, _), = _points_from([{"point": [500, 500]}], lens)
    # 400 px of view at 4x is 100 px of frame, so the middle is 50 px in.
    assert (u, v) == (100 + 49, 50 + 49), (u, v)


def test_the_corners_of_the_view_are_the_corners_of_the_crop():
    lens = _Lens(np.zeros((400, 400, 3), np.uint8), u0=100, v0=50, factor=4)
    (top_left, _), (bottom_right, _) = (
        (p[:2], p[2]) for p in _points_from(
            [{"point": [0, 0]}, {"point": [1000, 1000]}], lens))
    assert top_left == (100, 50)
    assert bottom_right == (100 + 99, 50 + 99)


def test_a_box_makes_the_same_trip():
    lens = _Lens(np.zeros((400, 400, 3), np.uint8), u0=100, v0=50, factor=4)
    u0, v0, u1, v1, _ = _box_from({"box_2d": [0, 0, 1000, 1000]}, lens)
    assert (u0, v0, u1, v1) == (100, 50, 199, 149)


def test_an_uncropped_lens_changes_nothing():
    """The identity case has to stay exactly what it was, because every
    uncalibrated camera and every already-cropped image goes through it."""
    lens = _Lens(np.zeros((480, 640, 3), np.uint8))
    assert not lens.cropped
    for y, x in ((0, 0), (500, 500), (1000, 1000), (37, 912)):
        assert lens.to_frame(*_to_px(y, x, *lens.wh)) == _to_px(y, x, 640, 480)


def test_the_upscale_is_an_integer_so_the_way_back_is_exact():
    """Nearest neighbour and a whole-number factor, deliberately. A fractional
    resize would make `to_frame` nearly right, which is the worst kind."""
    for factor in (1, 2, 3, 4):
        lens = _Lens(np.zeros((100 * factor, 100 * factor, 3), np.uint8),
                     u0=7, v0=11, factor=factor)
        for u in range(0, 100 * factor, 13):
            back = lens.to_frame(u, u)[0]
            assert back == 7 + u // factor


# -- the depth image as a mask ------------------------------------------------

def _boxed_scene(block_h=0.030, plate_h=0.008, cam_z=0.9):
    """A synthetic overhead frame: a plate with a block resting on it."""
    from heron.types import Frame

    h = w = 120
    depth = np.full((h, w), cam_z, np.float32)                     # table at z=0
    depth[20:100, 20:100] = cam_z - plate_h                        # the plate
    depth[50:70, 50:70] = cam_z - block_h                          # block on it
    k = np.array([[120.0, 0, 60.0], [0, 120.0, 60.0], [0, 0, 1.0]])
    t = np.array([[1.0, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, cam_z], [0, 0, 0, 1.0]])
    return Frame(camera="cam_high", rgb=np.zeros((h, w, 3), np.uint8), depth=depth,
                 intrinsics=k, t_base_cam=t)


def test_the_depth_mask_keeps_the_dominant_height_not_the_tallest():
    """A support's box usually contains whatever sits on it. The first version
    kept the upper half of the height range — right for a block, exactly
    backwards for the plate under one: it kept the block and located the plate
    AT the block. Measured: supports' median fell to 1.9 mm while the tail grew
    a 208 mm blunder. The object is the height there is most of."""
    from heron.skills.sensing import _mask_from_depth

    frame = _boxed_scene()
    plate_box = (20, 20, 100, 100)         # encloses the block too
    mask = _mask_from_depth(frame, plate_box, table_z=0.0)
    assert mask is not None
    assert mask[30, 30], "plate pixels are the object"
    assert not mask[60, 60], "the block riding on it is not"

    block_box = (50, 50, 70, 70)
    mask = _mask_from_depth(frame, block_box, table_z=0.0)
    assert mask is not None
    assert mask[60, 60], "inside its own box the block is the dominant height"


def test_the_depth_mask_declines_rather_than_guessing():
    from heron.skills.sensing import _mask_from_depth
    from heron.types import Frame

    bare = Frame(camera="c", rgb=np.zeros((60, 60, 3), np.uint8))
    assert _mask_from_depth(bare, (10, 10, 50, 50), table_z=0.0) is None, \
        "no depth, no mask — the box path stands"
    flat = _boxed_scene(block_h=0.0, plate_h=0.0)
    assert _mask_from_depth(flat, (10, 10, 50, 50), table_z=0.0) is None, \
        "nothing above the table: refusing beats masking the table itself"
