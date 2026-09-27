

def test_first_binding_prefers_the_instance_the_goal_still_needs():
    """Two lookalike reds, one already inside the plate's box: 'put a red on
    the plate' must bind the OTHER one — acting on the satisfied instance
    re-places a placed block."""
    from heron.skills.sensing import _prefer_goal_pending

    class _Ctx:
        goal_predicates = [("on", ["red_block", "white_plate"], False)]

        class log:
            @staticmethod
            def event(*a, **k):
                pass

    # keep[i] = (cx, cy, pt, mask-dict); plate box spans px 300..420
    keep = [
        (360, 340, (0.38, 0.13, -0.007), {"box": [300, 280, 420, 400]}),  # plate
        (360, 345, (0.38, 0.14, 0.02), {"box": [345, 330, 375, 360]}),    # red ON plate
        (700, 500, (0.25, 0.30, 0.02), {"box": [685, 485, 715, 515]}),    # red on table
    ]
    claimed = {0: "white_plate"}
    assert _prefer_goal_pending(_Ctx(), "red_block", [1, 2], keep, claimed) == 2
    # Negated goal (move it OFF): prefer the one already on the plate.
    _Ctx.goal_predicates = [("on", ["red_block", "white_plate"], True)]
    assert _prefer_goal_pending(_Ctx(), "red_block", [1, 2], keep, claimed) == 1
    # No anchor claimed yet: no opinion.
    assert _prefer_goal_pending(_Ctx(), "red_block", [1, 2], keep, {}) is None
