"""Opening a drawer: the direction, the wrist, and what counts as evidence.

libero_goal scored 0/6, and the reason was not that the pull failed — it was
never attempted. With no open/closed predicate the planner reduced "open the
middle drawer of the cabinet" to gripper_empty(arm), a condition already true
before the episode began, verified it correctly, and declared victory.

The numbers quoted below were measured in the simulator before any of this was
written; see tools/probe_open_drawer.py.
"""
from __future__ import annotations

import numpy as np
import pytest

from heron.skills.articulate import face_direction, facing_rotation
from heron.types import EntityTrack

# From the scene probe: cabinet body and the front of its middle drawer, in the
# two placements LIBERO-PRO produces.
STOCK_CABINET = np.array([0.0425, -0.2400, 0.9050])
STOCK_FACE = np.array([0.0425, -0.1152, 1.0155])
SWAPPED_CABINET = np.array([-0.4004, 0.2160, 0.9050])
SWAPPED_FACE = np.array([-0.4034, 0.0972, 1.0155])


def test_direction_matches_the_measured_joint_axis_in_both_placements():
    """The simulator says these drawers slide along +y and -y respectively. The
    skill has to reach the same answer from two grounded points and nothing else,
    because on hardware there is no joint to read."""
    def off_by(got, want):
        return float(np.degrees(np.arccos(np.clip(np.asarray(got) @ np.asarray(want), -1, 1))))

    # Within a couple of degrees, not exact: the cabinet's centroid and its
    # drawer front are 3 mm apart in x, which tilts the inferred direction by
    # about 1.4 deg. That is the accuracy the pull actually needs — the handle
    # is 90 mm tall and the fingers open to 78 mm.
    assert off_by(face_direction(STOCK_FACE, STOCK_CABINET), [0, 1, 0]) < 3.0
    assert off_by(face_direction(SWAPPED_FACE, SWAPPED_CABINET), [0, -1, 0]) < 3.0


def test_direction_is_horizontal_even_when_the_handle_is_lower_than_the_cabinet():
    """A drawer near the floor is still a drawer. Height difference between the
    handle and the cabinet's centroid says nothing about which way it slides."""
    low = np.array([0.0425, -0.1152, 0.60])
    u = face_direction(low, STOCK_CABINET)
    assert abs(float(u[2])) < 1e-9
    assert np.isclose(float(np.linalg.norm(u)), 1.0)


def test_a_degenerate_pair_is_refused_rather_than_invented():
    """Handle and cabinet grounded to the same spot give a direction made of
    noise, and a made-up direction drives the arm into the furniture."""
    assert face_direction(STOCK_CABINET + 0.005, STOCK_CABINET) is None


@pytest.mark.parametrize("u", [
    np.array([0.0, 1.0, 0.0]), np.array([0.0, -1.0, 0.0]),
    np.array([1.0, 0.0, 0.0]), np.array([0.6, -0.8, 0.0]),
])
def test_the_facing_frame_is_a_rotation_not_a_reflection(u):
    """The first version of this was three matrices written out by hand, all
    with determinant -1. The arm chased each for four seconds and reported a
    177-degree error, which looks exactly like an unreachable pose."""
    R = facing_rotation(u)
    assert np.isclose(float(np.linalg.det(R)), 1.0, atol=1e-6)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-6)


@pytest.mark.parametrize("u", [
    np.array([0.0, 1.0, 0.0]), np.array([0.0, -1.0, 0.0]), np.array([0.6, -0.8, 0.0]),
])
def test_the_tool_faces_the_drawer_with_the_fingers_vertical(u):
    """Both halves matter. Facing the drawer is what makes the point reachable;
    fingers vertical is what lets them close around a horizontal handle bar —
    the horizontal-finger variant moved the drawer 0 mm in every trial."""
    R = facing_rotation(u)
    assert np.allclose(R[:, 2], -u / np.linalg.norm(u), atol=1e-6)   # tool z into the face
    assert abs(float(R[2, 1])) > 0.99                                # tool y vertical


def test_the_open_baseline_comes_only_from_a_sighting():
    """open(entity) compares against where the drawer was first SEEN. A pose
    written by the pulling skill would make the predicate a restatement of what
    the skill tried to do."""
    from heron.belief import BeliefStore

    b = BeliefStore()
    b.update_track("drawer", xyz_base=(0.04, -0.115, 1.015), from_sighting=False)
    assert b.track("drawer").first_xyz is None
    b.update_track("drawer", xyz_base=(0.04, -0.115, 1.015), from_sighting=True)
    assert b.track("drawer").first_xyz == (0.04, -0.115, 1.015)
    # And it is the FIRST sighting, not the latest one.
    b.update_track("drawer", xyz_base=(0.04, 0.030, 1.015), from_sighting=True)
    assert b.track("drawer").first_xyz == (0.04, -0.115, 1.015)


def test_open_reads_true_only_after_a_real_travel():
    from heron.verify import OPEN_MIN_TRAVEL_M

    start = np.array([0.0425, -0.1152])
    barely = start + np.array([0.0, 0.02])       # grounding noise, not an open drawer
    opened = start + np.array([0.0, 0.140])      # what the probe measured
    assert float(np.linalg.norm(barely - start)) < OPEN_MIN_TRAVEL_M
    assert float(np.linalg.norm(opened - start)) >= OPEN_MIN_TRAVEL_M


def test_a_robot_that_cannot_turn_its_wrist_says_so_instead_of_trying():
    """Measured: the pre-grasp point in front of the handle is 314 mm out of
    reach with the wrist held vertical. Attempting it anyway would be a long,
    confident failure attributed to grasping."""
    from heron.skills.articulate import open_drawer

    class _NoOrientation:
        cameras = ["cam_high"]
        arms = ["arm"]

        def can_orient(self):
            return False

    class _Ctx:
        robot = _NoOrientation()

    res = open_drawer(_Ctx(), entity="drawer", cabinet="cabinet")
    assert not res.ok
    assert "orientation" in res.error


# -- the direction taken from the drawer's own face --------------------------

def _panel_frame(normal, res=(120, 160), dist=0.9):
    """A synthetic depth image of a flat panel with the given world normal.

    Built rather than recorded because the question is geometric: does the fit
    recover the normal of a plane, and does it point out of the cabinet rather
    than into it.
    """
    from heron.types import Frame

    h, w = res
    K = np.array([[200.0, 0, w / 2], [0, 200.0, h / 2], [0, 0, 1.0]])
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    # Camera sits out along the normal, looking back at the panel.
    eye = n * dist
    fwd = -n
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([right, down, fwd])
    T[:3, 3] = eye
    # The panel is the plane through the origin with normal n: every ray from the
    # camera hits it at the same depth, tilted with the plane.
    depth = np.zeros((h, w), np.float32)
    for v in range(h):
        for u in range(w):
            ray = np.linalg.inv(K) @ np.array([u, v, 1.0])
            ray_w = T[:3, :3] @ ray
            denom = float(n @ ray_w)
            depth[v, u] = np.nan if abs(denom) < 1e-6 else float(-(n @ eye) / denom)
    return Frame(camera="cam_high", rgb=np.zeros((h, w, 3), np.uint8),
                 depth=depth, intrinsics=K, t_base_cam=T)


@pytest.mark.parametrize("normal", [[0, 1, 0], [0, -1, 0], [1, 0, 0], [0.6, -0.8, 0]])
def test_the_face_normal_is_recovered_from_depth_alone(normal):
    from heron.skills.articulate import face_normal

    frame = _panel_frame(normal)
    got = face_normal(frame, (20, 20, 140, 100))
    assert got is not None
    off = float(np.degrees(np.arccos(np.clip(got @ (np.asarray(normal, float)
                                                    / np.linalg.norm(normal)), -1, 1))))
    assert off < 5.0, (got, normal, off)


def test_the_normal_points_out_of_the_cabinet_not_into_it():
    """Sign is the whole answer: the wrong end of the axis pushes the drawer
    shut and calls it an open."""
    from heron.skills.articulate import face_normal

    frame = _panel_frame([0, -1, 0])
    got = face_normal(frame, (20, 20, 140, 100))
    assert float(got @ np.array([0.0, -1.0, 0.0])) > 0.9   # toward the camera


def test_a_region_with_no_depth_is_refused():
    from heron.skills.articulate import face_normal

    frame = _panel_frame([0, 1, 0])
    frame.depth[:] = np.nan
    assert face_normal(frame, (20, 20, 140, 100)) is None
    assert face_normal(frame, None) is None


def test_a_sighting_off_the_rail_is_not_progress():
    """Measured: one episode reported 179 mm of travel on a drawer that had
    moved 15 mm, because the detector latched onto the handle one shelf up —
    126 mm higher and 143 mm to the side. The VQA verifier caught it and the
    goal was correctly reported unmet, but the skill should not have claimed it.

    A drawer front slides along its rail. Displacement with a large sideways or
    vertical component is a different object, whatever its magnitude.
    """
    from heron.skills.articulate import OFF_RAIL_MAX_M

    u = np.array([-0.295, -0.956, 0.0])
    start = np.array([-0.398, 0.122, 1.002])

    def decompose(now):
        step = np.asarray(now) - start
        along = float(step[:2] @ u[:2])
        sideways = float(np.linalg.norm(step[:2] - u[:2] * along))
        return along, sideways, float(step[2])

    # The real (failed) drawer: barely moved, but honestly on the rail.
    along, side, dz = decompose([-0.4078, 0.1105, 1.0234])
    assert side <= OFF_RAIL_MAX_M and abs(dz) <= OFF_RAIL_MAX_M
    assert along < 0.05

    # The impostor the run believed: 126 mm of height change alone disqualifies it.
    along, side, dz = decompose([-0.5054, -0.0209, 1.0624])
    assert abs(dz) > OFF_RAIL_MAX_M, "a drawer does not rise 60 mm as it opens"


def test_a_face_patch_is_produced_even_when_nothing_boxes_the_drawer():
    """The live failure: ER-2 points at the drawer and declines to box it, and no
    segmentation service runs in the simulator environment — so fit_face was
    handed None in every episode, the plane fit never ran, and the pull fell back
    to the cabinet-relative direction 12.8 degrees off the rail. Three attempts,
    17 mm of travel, against the 140 mm required."""
    from heron.skills.articulate import FACE_WINDOW_FRACTION, face_box

    frame = _panel_frame([0, 1, 0])
    w = frame.rgb.shape[1]
    got = face_box(frame, None, None, (80, 60))
    assert got is not None
    assert (got[2] - got[0]) >= FACE_WINDOW_FRACTION * w - 2
    # And it stays inside the image when the point is near an edge.
    edge = face_box(frame, None, None, (2, 2))
    assert edge[0] >= 0 and edge[1] >= 0
    assert edge[2] < frame.rgb.shape[1] and edge[3] < frame.rgb.shape[0]


def test_a_real_box_is_preferred_over_the_fallback_window():
    from heron.skills.articulate import face_box

    frame = _panel_frame([0, 1, 0])
    assert face_box(frame, (10, 10, 90, 70), None, (50, 40)) == (10, 10, 90, 70)


def test_the_fallback_window_still_recovers_the_normal():
    """The point of the fallback is that the fit works from it, not merely that
    a box exists."""
    from heron.skills.articulate import face_box, face_normal

    for normal in ([0, 1, 0], [0.6, -0.8, 0]):
        frame = _panel_frame(normal)
        box = face_box(frame, None, None, (80, 60))
        got = face_normal(frame, box)
        assert got is not None
        want = np.asarray(normal, float) / np.linalg.norm(normal)
        off = float(np.degrees(np.arccos(np.clip(got @ want, -1, 1))))
        assert off < 5.0, (normal, got, off)


def test_a_refused_fit_says_why():
    """A silent None is indistinguishable from never having tried, and that is
    exactly how four rounds of pulling went by on a fallback direction 53 degrees
    off the rail before anyone noticed the measurement was missing."""
    from heron.skills import articulate as A

    frame = _panel_frame([0, 1, 0])
    assert A.fit_face(frame, None) is None
    assert "box" in A.last_fit_refusal

    assert A.fit_face(frame, (10, 10, 14, 14)) is None
    assert "too small" in A.last_fit_refusal

    # A band nowhere near the panel leaves nothing to fit.
    A.fit_face(frame, (20, 20, 140, 100), None, near_z=99.0)
    assert "height" in A.last_fit_refusal or "near surface" in A.last_fit_refusal

    # And a good fit clears the note rather than leaving a stale one.
    assert A.fit_face(frame, (20, 20, 140, 100)) is not None
    assert A.last_fit_refusal == ""


def test_a_plane_that_disagrees_with_the_cabinet_is_overruled_not_obeyed():
    """Measured: the fallback patch around the handle caught the cabinet's side
    panel, fitted it beautifully (flatness 0.012) and returned a normal along +x
    for a drawer that opens along -y. Two estimates that disagree mean one is
    about the wrong surface, and the cabinet-relative one — derived from two
    separately grounded objects — cannot lock onto a single flat face by
    accident."""
    from heron.skills.articulate import CROSSCHECK_MAX_DEG

    plane = np.array([1.0, -0.004, 0.0])
    body = np.array([-0.221, 0.975, 0.0])
    off = float(np.degrees(np.arccos(np.clip(plane @ body, -1, 1))))
    assert off > CROSSCHECK_MAX_DEG, off

    # And an agreeing pair is left alone: 7.8 degrees, from a real episode.
    a, b = np.array([0.173, 0.985, 0.0]), np.array([0.305, 0.952, 0.0])
    assert float(np.degrees(np.arccos(np.clip(a @ b, -1, 1)))) < CROSSCHECK_MAX_DEG


def test_the_segmenter_is_never_asked_about_a_drawer_with_a_point_alone():
    """Order of operations, and it was wrong.

    ER-2 declines to box a drawer front, so the segmenter used to be asked with
    a point alone — and a point on a cabinet is genuinely ambiguous between the
    handle, the drawer, and the cabinet. SAM2 answered "the cabinet": 22 098 px,
    no plane, no direction, and the pull fell back to a cabinet-relative
    estimate. Building the fallback window FIRST and handing it over as a box
    says which scale was meant. Measured against the simulator's own joint axis
    on the scene that had been failing: window alone 14.9 degrees off the rail,
    point-prompted mask no fit at all, window-as-box plus point 1.9 degrees off.
    """
    from heron.skills import articulate as A
    from heron.types import SpatialRelation  # noqa: F401  (import parity)

    asked = []

    class _Seg:
        enabled = True

        def candidates(self, rgb, box=None, point=None):
            asked.append({"box": box, "point": point})
            return []

    class _Track:
        description = "the middle drawer"
        camera = "cam_high"
        px = (306, 151)
        xyz_base = (-0.4034, 0.0972, 1.0155)
        first_xyz = xyz_base
        relation = None

    class _Belief:
        entities = {}

        def track(self, _):
            return _Track()

    class _Log:
        def event(self, *a, **k):
            pass

    frame = _panel_frame([0, 1, 0], res=(240, 320))

    class _Robot:
        cameras = ["cam_high"]

        def capture(self, _):
            return frame

    class _Ctx:
        robot = _Robot()
        belief = _Belief()
        log = _Log()
        segmenter = _Seg()
        cfg = None

    A._pull_direction(_Ctx(), "middle_drawer", None,
                      np.array([-0.4034, 0.0972, 1.0155]))
    assert asked, "the segmenter was never consulted at all"
    assert asked[0]["box"] is not None, (
        "the segmenter was asked with a point alone; on a cabinet that returns "
        "the whole cabinet")
    assert asked[0]["point"] == (306, 151)
