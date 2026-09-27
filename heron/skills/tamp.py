"""TiPToP TAMP backend skills: open-vocabulary pick/place with the heavy pipeline
(Gemini-ER detection + SAM2 segmentation + FoundationStereo depth + M2T2 grasp
prediction + cuRobo motion planning), exposed over the same HTTP protocol
Pigey's real-robot agent used (tamp_server on :7777 wrapping tiptop-run).

These are ALTERNATIVES to the scripted top-down pick/place: the planner (or a
repair) reaches for them when geometry is hard — elevated grasps, clutter,
non-trivial approach directions. They require `tamp_url` in the config and a
running TAMP server; Heron still verifies their postconditions itself, exactly
as with every other skill.

Protocol (from Pigey real/agent.ts):
  POST /pick        {"label": "<open-vocab description>"}
  POST /drop_above  {"target_label": ..., "dx_m", "dy_m", "abs_x", "abs_y", "abs_z"}
  POST /perceive    {}   -> known_objects, objects_detail (3D centroids+extents), images
  POST /release     {}
"""
from __future__ import annotations

import json
import urllib.request
from typing import Optional

from ..types import SkillResult
from . import SkillContext, skill

TAMP_TIMEOUT_S = 300.0  # a tiptop-run subprocess legitimately takes ~45s per call


def _tamp_post(ctx: SkillContext, path: str, body: dict) -> dict:
    url = getattr(ctx.cfg, "tamp_url", None)
    if not url:
        raise RuntimeError("tamp skills need tamp_url in the config (a running TiPToP tamp_server)")
    req = urllib.request.Request(f"{url.rstrip('/')}{path}", method="POST",
                                 data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=TAMP_TIMEOUT_S) as r:
        return json.loads(r.read())


@skill(
    name="tamp_pick",
    description=(
        "Heavy-pipeline grasp via TiPToP (detect -> segment -> stereo depth -> M2T2 grasp -> "
        "cuRobo plan). Use when the scripted top-down pick keeps failing, or geometry is hard "
        "(elevated/rimmed objects, clutter, odd shapes). Requires tamp_url. label is an "
        "open-vocabulary visual description."
    ),
    params={
        "entity": {"type": "string", "description": "entity id — its description is sent as the label"},
    },
    required=["entity"],
    post_hint="holding(entity, arm)",
)
def tamp_pick(ctx: SkillContext, entity: str) -> SkillResult:
    label = ctx.belief.track(entity).description
    out = _tamp_post(ctx, "/pick", {"label": label})
    grasped = bool(out.get("is_grasped"))
    if grasped:
        arm = ctx.robot.arms[0]
        ctx.belief.update_track(entity, confidence=0.9, held_by=arm)
    return SkillResult(
        ok=out.get("success") is not False,
        info=f"tamp_pick({label!r}): success={out.get('success')} is_grasped={out.get('is_grasped')} "
             f"width={out.get('gripper_width_m')} reason={out.get('fail_reason')}",
        proprio={"width_m": float(out.get("gripper_width_m") or 0.0),
                 "effort": 3.0 if grasped else 0.05},
        error=None if out.get("success") is not False else str(out.get("fail_reason")),
    )


@skill(
    name="tamp_place",
    description=(
        "Heavy-pipeline placement via TiPToP: release the held entity above a target using its "
        "cached 3D centroid (cuRobo-planned approach). dx/dy offset in meters; or absolute x/y/z. "
        "Requires tamp_url."
    ),
    params={
        "entity": {"type": "string"},
        "target": {"type": "string", "description": "destination entity id"},
        "dx": {"type": "number"},
        "dy": {"type": "number"},
        "x": {"type": "number"},
        "y": {"type": "number"},
    },
    required=["entity", "target"],
    post_hint="in(entity, target) or on(entity, target); not holding(entity, arm)",
)
def tamp_place(ctx: SkillContext, entity: str, target: str, dx: float = 0.0, dy: float = 0.0,
               x: Optional[float] = None, y: Optional[float] = None) -> SkillResult:
    label = ctx.belief.track(target).description
    body = {"target_label": label, "dx_m": dx, "dy_m": dy,
            "abs_x": x, "abs_y": y, "abs_z": None}
    out = _tamp_post(ctx, "/drop_above", body)
    if out.get("success") is not False:
        ctx.belief.update_track(entity, confidence=0.5, held_by=None,
                                note="placed by tamp; re-perceive for exact pose")
    return SkillResult(
        ok=out.get("success") is not False,
        info=f"tamp_place({entity} -> {label!r}): success={out.get('success')} "
             f"reason={out.get('fail_reason')}",
        error=None if out.get("success") is not False else str(out.get("fail_reason")),
    )
