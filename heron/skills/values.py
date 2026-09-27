"""Perception as values, not as a side effect each skill arranges for itself.

Today `pick(entity)` goes and looks for the entity. So does the `visible`
precondition checked a tenth of a second earlier, and so does `place`, and so
does every verifier afterwards. Measured on the rig: 23 detection round trips
and 127 seconds of a 207-second episode, for two objects that never moved. The
count is not knowable before the run because nobody decides it — each skill
decides for itself, in private.

The fix is not a cache. It is to make what a camera saw a VALUE:

    det   = locate(ctx, "the red block")     # one call, or none if cached
    where = to_base(ctx, det)                # arithmetic; no call
    grasp_at(ctx, where.xyz, "right")        # motion; no call

Each stage is pure enough to hand its result to the next, so a program can be
read off as a graph and its cost counted before anything moves. Waddle's
primitives are cut the same way, and for the same reason.

The composed skills (`pick`, `place`) still exist and still take an entity —
they are these primitives in sequence. What changes is that the sequence is
visible, and a caller holding a Detection can skip straight to the motion.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..types import Frame
from . import SkillContext


@dataclass
class Detection:
    """What one camera saw of one thing, in one frame.

    Carries the frame it came from so that everything downstream — deprojection,
    a mask, a crop for a second opinion — works from the same pixels rather than
    capturing again and getting a different world.
    """

    query: str
    camera: str
    point: tuple[int, int]
    conf: float
    frame: Frame
    box: Optional[tuple[int, int, int, int]] = None
    mask: Optional[np.ndarray] = None
    alternatives: int = 0        # how many other candidates the detector offered

    @property
    def extent_px(self) -> Optional[tuple[int, int]]:
        if self.box is None:
            return None
        return (self.box[2] - self.box[0], self.box[3] - self.box[1])


@dataclass
class Located:
    """A world point, and how it came to be known.

    `source` and `conf` travel with the number because a position from a mask
    and a position from a single pixel are not the same claim — measured 11.0 mm
    against 22.7 mm — and whoever acts on it should be able to tell.
    """

    xyz: np.ndarray
    source: str
    conf: float
    detection: Optional[Detection] = None
    span_m: Optional[float] = None      # how wide the thing is, when a mask says


@dataclass
class Perception:
    """Everything seen in one look, keyed by query.

    A step that needs three objects located should cost one look, not three.
    """

    frames: dict[str, Frame] = field(default_factory=dict)
    seen: dict[str, Located] = field(default_factory=dict)
    calls: int = 0

    def get(self, query: str) -> Optional[Located]:
        return self.seen.get(query)


def locate(ctx: SkillContext, query: str, camera: Optional[str] = None,
           frame: Optional[Frame] = None) -> Optional[Detection]:
    """One detector call: where is `query` in this frame?

    Returns a value. Nothing is written to the belief store, nothing is
    verified, no motion is planned — those are separate decisions made by
    whoever asked.
    """
    from .sensing import _mask_for, _usable_cameras   # noqa: PLC0415

    cams = [camera] if camera else _usable_cameras(ctx)
    for cam in cams:
        f = frame if frame is not None else ctx.robot.capture(cam)
        try:
            cands = list(ctx.grounder.points(f, query))
        except Exception as e:
            ctx.log.event("locate_error", query=query, camera=cam,
                          error=f"{type(e).__name__}: {e}")
            continue
        if not cands:
            continue
        u, v, conf = cands[0]
        boxes = []
        try:
            boxes = list(ctx.grounder.boxes(f, query))
        except Exception:
            pass
        box = tuple(int(x) for x in boxes[0][:4]) if boxes else None
        mask = _mask_for(ctx, f, box, (u, v))
        if box is None and mask is not None:
            vs, us = np.where(mask)
            box = (int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))
        det = Detection(query=query, camera=cam, point=(int(u), int(v)),
                        conf=float(conf), frame=f, box=box, mask=mask,
                        alternatives=max(0, len(cands) - 1))
        ctx.log.event("locate", query=query, camera=cam, point=[int(u), int(v)],
                      has_box=box is not None, has_mask=mask is not None,
                      alternatives=det.alternatives, conf=round(float(conf), 2))
        return det
    ctx.log.event("locate_missed", query=query, cameras=cams)
    return None


def to_base(ctx: SkillContext, det: Detection,
            support_z: Optional[float] = None) -> Optional[Located]:
    """Detection -> a point in the arm's frame. Pure arithmetic; no API call.

    Prefers the mask, falls back to the box, falls back to the single pixel —
    in that order because that is the order of their measured accuracy on this
    rig: 11.0 mm, then the box, then 22.7 mm.
    """
    if det is None:
        return None
    z = ctx.cfg.table_z if support_z is None else support_z
    span = None
    if det.box is not None:
        p = det.frame.deproject_region(*det.box, support_z=z, mask=det.mask)
        if p is not None:
            if det.mask is not None:
                vs, us = np.where(det.mask)
                lo = det.frame.deproject(int(us.min()), int(vs.max()))
                hi = det.frame.deproject(int(us.max()), int(vs.max()))
                if lo is not None and hi is not None:
                    span = float(np.linalg.norm(np.asarray(hi[:2]) - np.asarray(lo[:2])))
            source = "mask" if det.mask is not None else "box"
            return Located(np.asarray(p, float), source, det.conf, det, span)
    p = det.frame.deproject(*det.point)
    if p is None:
        return None
    return Located(np.asarray(p, float), "single-pixel", min(det.conf, 0.7), det, None)


def look(ctx: SkillContext, queries: list[str], camera: Optional[str] = None,
         support_z: Optional[float] = None) -> Perception:
    """Locate several things from ONE capture.

    The frame is shared deliberately: three objects located from three separate
    captures are three descriptions of three slightly different worlds, and the
    arm may have moved between them.
    """
    from .sensing import _mask_for, _usable_cameras  # noqa: PLC0415

    out = Perception()
    cams = [camera] if camera else _usable_cameras(ctx)
    for cam in cams:
        frame = ctx.robot.capture(cam)
        out.frames[cam] = frame
        todo = [q for q in queries if q not in out.seen]

        # One request for everything. The model reads the whole frame to answer
        # about any part of it, so asking six times costs six readings of one
        # image — measured on the sorting task, six calls for six objects.
        many = {}
        ask_many = getattr(ctx.grounder, "locate_many", None)
        if callable(ask_many) and len(todo) > 1:
            try:
                many = ask_many(frame, todo)
                out.calls += 1
            except Exception as e:
                ctx.log.event("locate_many_failed", error=f"{type(e).__name__}: {e}")
                many = {}

        for q in todo:
            hit = many.get(q)
            if hit is not None:
                box = hit["box"]
                mask = _mask_for(ctx, frame, box, hit["point"])
                if box is None and mask is not None:
                    vs, us = np.where(mask)
                    box = (int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))
                det = Detection(query=q, camera=cam, point=hit["point"],
                                conf=hit["conf"], frame=frame, box=box, mask=mask)
            else:
                # Whatever the batch missed is worth one question of its own —
                # a partial answer plus a few singles still beats one call each.
                det = locate(ctx, q, camera=cam, frame=frame)
                out.calls += 1
            if det is None:
                continue
            where = to_base(ctx, det, support_z=support_z)
            if where is not None:
                out.seen[q] = where
        if len(out.seen) == len(queries):
            break
    ctx.log.event("look", queries=queries, found=sorted(out.seen),
                  detector_calls=out.calls, cameras=list(out.frames))
    return out
