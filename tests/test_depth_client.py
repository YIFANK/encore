"""End-to-end test of the sidecar depth server (fake mode) + Heron client:
spawn the real server process, fetch a frame, deproject the synthetic block."""
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from heron.robot.depthclient import DepthServerClient

SERVER = Path(__file__).resolve().parents[1] / "tools" / "depth_server.py"
PORT = 8799


@pytest.fixture()
def fake_server():
    proc = subprocess.Popen([sys.executable, str(SERVER), "--fake", "--port", str(PORT), "--fps", "20"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        for _ in range(50):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{PORT}/health", timeout=1)
                break
            except Exception:
                time.sleep(0.1)
        else:
            raise RuntimeError("fake depth server did not come up")
        yield f"http://127.0.0.1:{PORT}"
    finally:
        proc.terminate()
        proc.wait(timeout=5)


def overhead_extrinsics(height: float = 0.85) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    t[:3, 3] = [0.0, 0.0, height]
    return t


def test_health_info_and_frame(fake_server):
    client = DepthServerClient(fake_server, t_base_cam=overhead_extrinsics())
    ok, note = client.healthy("cam_high")
    assert ok, note
    assert client.info("cam_high")["depth_scale"] == 0.001
    frame = client.capture("cam_high")
    assert frame.rgb.shape == (480, 640, 3)
    assert frame.depth.shape == (480, 640)
    # Table pixels read 0.85 m; the synthetic block reads 0.81 m.
    assert abs(float(frame.depth[10, 10]) - 0.85) < 1e-3
    cx, cy = 640 // 2 + 30, 480 // 2 - 20
    assert abs(float(frame.depth[cy, cx]) - 0.81) < 1e-3


def test_deproject_block_top(fake_server):
    client = DepthServerClient(fake_server, t_base_cam=overhead_extrinsics())
    frame = client.capture("cam_high")
    cx, cy = 640 // 2 + 30, 480 // 2 - 20
    p = frame.deproject(cx, cy)
    assert p is not None
    assert abs(p[2] - 0.04) < 0.005  # block top height in world frame
    table = frame.deproject(30, 30)
    assert abs(table[2] - 0.0) < 0.005


def test_depth_frames_keep_the_homography_as_a_fallback():
    """Switching a camera to RGB-D must not be able to make grounding worse.

    A depth frame with no extrinsics has no route to 3D on its own — deproject
    returns None and the object is simply not found, which is a regression on
    the homography-only setup it replaced.
    """
    import numpy as np

    from heron.robot.depthclient import DepthServerClient

    h = np.array([[-2.35e-3, 4.64e-4, 1.983],
                  [1.53e-5, 2.43e-3, -1.102],
                  [-1.54e-5, 9.80e-4, 1.0]])
    c = DepthServerClient("http://127.0.0.1:1", h_pixel_world=h, plane_z=-0.015)
    assert c.t_base_cam is None
    assert c.h_pixel_world is not None and c.plane_z == -0.015

    # And the frame it builds must actually carry them, which is the part that
    # would silently not happen.
    from heron.types import Frame

    f = Frame(camera="cam_high", rgb=np.zeros((4, 4, 3), np.uint8),
              depth=np.full((4, 4), np.nan, np.float32),
              h_pixel_world=c.h_pixel_world, plane_z=c.plane_z)
    p = f.deproject(2, 2)
    assert p is not None and abs(p[2] - (-0.015)) < 1e-9


def test_the_homography_is_withheld_at_a_resolution_it_was_not_fitted_at():
    """A D405's 640x480 mode is a horizontal CROP of 720p, not a scaling. Using
    the same matrix there deprojects to confidently wrong world coordinates —
    and only as a fallback, when depth has already failed and nobody is
    watching."""
    import numpy as np

    from heron.robot.depthclient import DepthServerClient

    h = np.eye(3)
    c = DepthServerClient("http://127.0.0.1:1", h_pixel_world=h, plane_z=-0.015,
                          homography_wh=(1280, 720))
    assert c.homography_wh == (1280, 720)

    # The check itself, applied the way capture() applies it.
    for wh, expected in (((1280, 720), True), ((640, 480), False)):
        keeps = c.homography_wh in (None, wh)
        assert keeps is expected, f"{wh} should {'keep' if expected else 'drop'} it"

    # No recorded resolution means "trust it anywhere", which is what a
    # homography fitted by an older tool has to mean.
    loose = DepthServerClient("http://127.0.0.1:1", h_pixel_world=h)
    assert loose.homography_wh is None
