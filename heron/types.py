"""Shared vocabulary: truth values, predicates, entities, evidence, failures.

Everything the planner, belief store, verifier, and repairer exchange is one of
these types. Serializable models are pydantic; runtime-only carriers of numpy
arrays (camera frames) are dataclasses.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

import numpy as np
from pydantic import BaseModel, Field


class Value(str, Enum):
    """Three-valued logic for grounded predicates (Seeing-is-Believing style)."""

    TRUE = "true"
    FALSE = "false"
    UNKNOWN = "unknown"

    def __bool__(self) -> bool:  # pragma: no cover - guard against misuse
        raise TypeError("Value is three-valued; compare explicitly against Value.TRUE/FALSE/UNKNOWN")


class PredicateSpec(BaseModel):
    """A predicate the program wants to hold (or not hold), e.g. in(red_block, blue_bowl)."""

    name: str
    args: list[str] = Field(default_factory=list)
    negated: bool = False

    @property
    def key(self) -> str:
        return f"{self.name}({','.join(self.args)})"

    def __str__(self) -> str:
        return ("not " if self.negated else "") + self.key

    def satisfied_by(self, value: Value) -> Value:
        """Truth of this spec given the raw predicate value (handles negation)."""
        if value is Value.UNKNOWN:
            return Value.UNKNOWN
        if self.negated:
            return Value.TRUE if value is Value.FALSE else Value.FALSE
        return value


class SpatialRelation(BaseModel):
    """Which of several lookalikes is meant, expressed as geometry.

    "The black bowl between the plate and the ramekin" names an object that no
    visual phrase can pick out: all three bowls are the same bowl. Asked for a
    purely visual description anyway, the planner invents a distinguishing
    feature that is not there — measured, it answered "the metal bowl on the far
    right", picked that, placed it correctly, and every predicate verified while
    the benchmark scored zero. A wrong binding cannot be caught downstream,
    because everything downstream is about the object it bound to.

    So the relation stays a relation until the anchors have positions, and then
    it is arithmetic.
    """

    kind: str                       # between | near | far_from | on | inside
    of: list[str] = Field(default_factory=list)   # anchor entity ids
    negated: bool = False           # "NOT between the plate and the ramekin"

    def __str__(self) -> str:
        return ("not " if self.negated else "") + f"{self.kind}({', '.join(self.of)})"


class EntityDecl(BaseModel):
    """An object the program refers to. `id` is stable for the whole episode.

    Stable ids are the fix for Pigey's label-drift failures: the program and the
    belief store speak entity ids; open-vocabulary detections are re-associated
    to ids at grounding time via `description`.
    """

    id: str
    description: str  # open-vocabulary phrase used to ground it, e.g. "the red cube"
    role: str = "object"  # object | container | surface | tool | region
    # Optional. Present when the instruction distinguishes this object from its
    # lookalikes by WHERE it is rather than what it looks like.
    relation: Optional[SpatialRelation] = None


class Evidence(BaseModel):
    """Why we believe a predicate value. Persisted to the episode journal."""

    id: str
    kind: str  # proprioceptive | visual_point | visual_vqa | oracle | model | assumption
    detail: str = ""
    camera: Optional[str] = None
    image_path: Optional[str] = None
    t: float = Field(default_factory=time.time)


class PredicateInstance(BaseModel):
    value: Value = Value.UNKNOWN
    confidence: float = 0.0
    t: float = 0.0
    evidence_id: Optional[str] = None

    def age(self) -> float:
        return time.time() - self.t if self.t else float("inf")


class EntityTrack(BaseModel):
    """Belief about where an entity is. Updated by grounding, moved by skills."""

    id: str
    description: str
    xyz_base: Optional[tuple[float, float, float]] = None
    camera: Optional[str] = None  # camera the last grounding came from
    px: Optional[tuple[int, int]] = None  # last image point (u, v) in that camera
    confidence: float = 0.0
    t: float = 0.0
    held_by: Optional[str] = None  # arm side if we believe we hold it
    notes: str = ""  # accumulated hints ("grasp the handle", "deformable")
    # Where the object rides relative to the tool once grasped. Skills command
    # the TOOL, so releasing at a target puts the OBJECT this much away from it.
    carry_offset: Optional[tuple[float, float, float]] = None
    # True only when this position came from actually seeing the object. A pose
    # written by pick or place is dead reckoning: correct enough to act on, but
    # not evidence that anything is visible.
    from_sighting: bool = False
    # How wide the thing looked, in metres, the last time it was grounded.
    #
    # Kept because grounding already draws the box this is measured from, and
    # `pick` needs it to decide whether an object is too wide to take centrally.
    # Without it, pick re-captured the frame and asked the detector for a box it
    # had just been given: 511 detector calls and 1166 seconds across a
    # 60-episode sweep, spent re-learning something already known, on the
    # critical path of every grasp.
    span_m: Optional[float] = None
    # How this entity is told apart from its lookalikes, when appearance cannot.
    # Carried here because grounding is what has to honour it.
    relation: Optional[SpatialRelation] = None
    # Where it was the first time we saw it, before anything acted on it. Only
    # a baseline is what makes "the drawer front has travelled 140 mm out" a
    # sensor reading rather than a claim about what we intended to do.
    first_xyz: Optional[tuple[float, float, float]] = None

    def age(self) -> float:
        return time.time() - self.t if self.t else float("inf")


class FailureClass(str, Enum):
    """The four-way (plus safety) failure taxonomy the diagnosis engine emits."""

    SKILL_EXEC = "skill_exec"  # the skill itself errored / could not run
    SKILL_EFFECT = "skill_effect"  # skill ran but did not achieve its postcondition
    PERCEPTION = "perception"  # a verifier or grounding was wrong / contradictory
    STATE_DRIFT = "state_drift"  # world changed outside our own actions
    PROGRAM = "program"  # the plan itself is wrong for the world as observed
    SAFETY = "safety"  # the safety guard rejected the motion


class FailureReport(BaseModel):
    step_id: str
    skill: str
    failure_class: FailureClass
    summary: str
    failed_predicates: list[PredicateSpec] = Field(default_factory=list)
    evidence_ids: list[str] = Field(default_factory=list)
    attempts: int = 0
    details: dict[str, Any] = Field(default_factory=dict)


# Depth-consistency cut inside a region: keep points within K median absolute
# deviations of the region's median depth, but never cut inside MIN_M — a flat
# object has a MAD near zero and must not be reduced to a single depth plane.
DEPTH_CONSISTENCY_K = 4.0
DEPTH_CONSISTENCY_MIN_M = 0.03
# A relative-depth height reading above this is the sensor failing (IR-dark
# objects), not a tall object; the parallax slide-back must not trust it.
TABLETOP_HEIGHT_SANITY_M = 0.10


@dataclass
class Frame:
    """One camera capture. `depth` is meters aligned to `rgb`; may be None.

    intrinsics: 3x3 K. t_base_cam: 4x4 camera->base transform (None until the
    camera is calibrated; overhead mock cameras always carry one).

    Depth-free rigs (macOS cannot give librealsense the D405s at all — verified
    2026-07-30) instead carry:
      - `h_pixel_world`: 3x3 homography, pixel -> world XY on the table plane at
        `plane_z`. Exact for anything resting on the table.
      - `proj`: 3x4 DLT projection matrix, world -> pixel. Absorbs intrinsics and
        extrinsics together, so it needs no depth and no factory intrinsics; two
        cameras with `proj` triangulate full 3D for objects off the table plane.
    """

    camera: str
    rgb: np.ndarray  # HxWx3 uint8
    depth: Optional[np.ndarray] = None  # HxW float32 meters
    intrinsics: Optional[np.ndarray] = None
    t_base_cam: Optional[np.ndarray] = None
    h_pixel_world: Optional[np.ndarray] = None
    plane_z: float = 0.0
    proj: Optional[np.ndarray] = None  # 3x4, world homogeneous -> pixel homogeneous
    t: float = field(default_factory=time.time)

    def __post_init__(self) -> None:
        """A camera with K and a pose can already project; say so.

        `proj` is what `triangulate` needs, and grounding prefers two-view
        triangulation over single-view deprojection precisely because it needs no
        depth and is not confined to the table plane. It arrived as a field for
        the depth-free rigs, where a DLT fit is the ONLY calibration there is,
        and so nothing ever derived it for a camera that had K and an extrinsic
        instead.

        Measured after the twin grew its second fixed viewpoint: both frames
        carried intrinsics and a pose, both carried `proj = None`, and the
        triangulation path — the whole reason a second camera is worth its
        detection call — never ran once. An explicitly supplied `proj` still
        wins: a DLT fitted against the rig is a measurement, and this is an
        inference from two others.
        """
        if self.proj is None and self.intrinsics is not None and self.t_base_cam is not None:
            try:
                world_from_cam = np.linalg.inv(np.asarray(self.t_base_cam, dtype=float))
            except np.linalg.LinAlgError:
                return
            self.proj = np.asarray(self.intrinsics, dtype=float) @ world_from_cam[:3, :4]

    def deproject(self, u: int, v: int) -> Optional[np.ndarray]:
        """Pixel -> xyz in world frame. A homography camera answers from the
        plane (parallax-corrected when depth can say how high the point is);
        otherwise aligned depth. None if no calibration exists.

        The homography WINS over depth when both exist, and not as a tie-break:
        on this rig's overhead camera the board-fitted plane is good to 1.1 mm
        while the sensor's absolute depth at that range is 18 mm off median —
        and every grounding path in the agent funnels through this method, so
        whichever branch is preferred here IS the system's accuracy.
        """
        if self.h_pixel_world is not None:
            return self._planar_point(int(u), int(v))
        if self.depth is None or self.intrinsics is None or self.t_base_cam is None:
            return None
        h, w = self.depth.shape[:2]
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))
        # Median over a small window rides over depth speckle at object edges.
        window = self.depth[max(0, v - 2) : v + 3, max(0, u - 2) : u + 3]
        valid = window[np.isfinite(window) & (window > 0)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]
        p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        return (self.t_base_cam @ p_cam)[:3]

    def _plane_depth_fit(self) -> Optional[np.ndarray]:
        """Per-frame model of what the depth sensor reads ON the table plane.

        The geometric plane depth at any pixel follows exactly from the camera
        pose; the sensor's deviation from it is real and spatially smooth
        (18 mm median, 40 mm at the edges on this rig's overhead D405, which is
        past its working range). Fit that deviation as a plane in (u, v) over
        the image's near-table pixels, two rounds so objects and glossy-dish
        artifacts fall out of the fit. Heights measured against this model need
        no ring around the object — the ring was the previous estimator, and a
        white plate's washed-out depth fed it a 6 mm difference for a 30 mm
        block, which zeroed the grasp height and closed the jaws on air.
        """
        if getattr(self, "_plane_fit_done", False):
            return self._plane_fit
        self._plane_fit_done = True
        self._plane_fit = None
        if self.depth is None or self.intrinsics is None or self.t_base_cam is None:
            return None
        h, w = self.depth.shape[:2]
        T = np.asarray(self.t_base_cam, dtype=float)
        R, C = T[:3, :3], T[:3, 3]
        K = np.asarray(self.intrinsics, dtype=float)
        uu, vv = np.meshgrid(np.arange(8, w - 8, 24, dtype=float),
                             np.arange(8, h - 8, 24, dtype=float))
        d = self.depth[vv.astype(int), uu.astype(int)].astype(float)
        rays = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1],
                         np.ones_like(uu)])
        rz = np.einsum("j,jhw->hw", R[2], rays)
        with np.errstate(divide="ignore", invalid="ignore"):
            d_pred = (self.plane_z - C[2]) / rz
        res = d - d_pred
        # Bound the residual band up front: a 65 m garbage return stays finite,
        # passes every finite-ness test, and then overflows the refit algebra.
        good = np.isfinite(res) & (d > 0) & (d_pred > 0.1) & (np.abs(res) < 1.0)
        a = np.stack([uu[good], vv[good], np.ones(int(good.sum()))], axis=1)
        r = res[good]
        keep = np.abs(r) < 0.06
        if keep.sum() < 40:
            return None
        coef = np.linalg.lstsq(a[keep], r[keep], rcond=None)[0]
        again = np.abs(r - a @ coef) < 0.015
        if again.sum() >= 40:
            coef = np.linalg.lstsq(a[again], r[again], rcond=None)[0]
        self._plane_fit = coef
        return coef

    def _height_from_depth(self, u: float, v: float, d_obj: float) -> float:
        """Height of a depth reading above the table plane, bias-corrected."""
        coef = self._plane_depth_fit()
        if coef is None:
            return 0.0
        T = np.asarray(self.t_base_cam, dtype=float)
        K = np.asarray(self.intrinsics, dtype=float)
        ray = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
        rz = float(T[:3, :3][2] @ ray)
        if abs(rz) < 1e-6:
            return 0.0
        d_pred = (self.plane_z - T[2, 3]) / rz + float(
            coef[0] * u + coef[1] * v + coef[2])
        return float(np.clip((d_pred - d_obj) * abs(rz), 0.0, 0.5))

    def _planar_point(self, u: int, v: int) -> Optional[np.ndarray]:
        """One pixel through the plane, lifted by RELATIVE depth.

        Same geometry as _planar_region, at a point: the pixel's height comes
        from (table ring) - (small window at the pixel), which cancels the
        sensor's local bias, and the plane intersection slides back up the ray
        from the camera centre to that height. A pixel that IS on the table
        measures a difference of ~0 and gets no correction, so surface-contact
        callers (box bottom edges, footprint corners) keep their exact planar
        answer — the correction only spends itself on pixels that are off the
        plane, which are exactly the ones the plain homography misplaces.
        """
        p = self.h_pixel_world @ np.array([float(u), float(v), 1.0])
        if abs(p[2]) < 1e-9:
            return None
        xy = np.array([p[0] / p[2], p[1] / p[2]])
        height = 0.0
        if self.depth is not None and self.intrinsics is not None \
                and self.t_base_cam is not None:
            h, w = self.depth.shape[:2]
            u = int(np.clip(u, 0, w - 1))
            v = int(np.clip(v, 0, h - 1))
            win = self.depth[max(0, v - 3) : v + 4, max(0, u - 3) : u + 4]
            wv = win[np.isfinite(win) & (win > 0)]
            if wv.size >= 4:
                height = self._height_from_depth(u, v, float(np.median(wv)))
                if height > TABLETOP_HEIGHT_SANITY_M:
                    height = 0.0  # garbage depth, not a tall object — see _planar_region
        if height > 0.0 and self.t_base_cam is not None:
            cam = np.asarray(self.t_base_cam, dtype=float)[:3, 3]
            denom = cam[2] - self.plane_z
            if abs(denom) > 1e-6:
                s = (cam[2] - self.plane_z - height) / denom
                xy = cam[:2] + s * (xy - cam[:2])
        return np.array([xy[0], xy[1], self.plane_z + height])

    def _planar_region(self, u0: int, v0: int, u1: int, v1: int,
                       mask: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Object centre for an oblique camera whose trusted calibration is a
        table-plane homography.

        The homography is exact for the plane, but the box centre pixel is the
        object's TOP, and from an oblique camera that pixel's plane intersection
        is displaced along the view ray by height x tan(incidence) — measured
        at 13-16 mm for a 25 mm block on the far side of this rig's table. The
        correction is pure geometry: estimate the top's height from RELATIVE
        depth (top vs the table ring around the object; the sensor's local bias
        cancels), then slide the plane point back up the ray from the camera
        centre to that height. No depth, no ring, or no pose -> plain planar,
        which is the uncorrected behaviour, never a failure.
        """
        uc, vc = (u0 + u1) / 2.0, (v0 + v1) / 2.0
        p = self.h_pixel_world @ np.array([uc, vc, 1.0])
        if abs(p[2]) < 1e-9:
            return None
        xy = np.array([p[0] / p[2], p[1] / p[2]])
        if self.depth is None or self.intrinsics is None or self.t_base_cam is None:
            return np.array([xy[0], xy[1], self.plane_z])
        h, w = self.depth.shape[:2]
        u0, u1 = sorted((int(np.clip(u0, 0, w - 1)), int(np.clip(u1, 0, w - 1))))
        v0, v1 = sorted((int(np.clip(v0, 0, h - 1)), int(np.clip(v1, 0, h - 1))))
        patch = self.depth[v0 : v1 + 1, u0 : u1 + 1]
        ok = np.isfinite(patch) & (patch > 0)
        if mask is not None and mask.shape[:2] == (h, w):
            m = ok & np.asarray(mask, dtype=bool)[v0 : v1 + 1, u0 : u1 + 1]
            if m.sum() >= 8:
                ok = m
        h_top = 0.0
        if ok.sum() >= 8:
            h_top = self._height_from_depth(uc, vc, float(np.median(patch[ok])))
            # An implausible height IS the depth failing, not a tall object:
            # dark cubes absorb the D405's IR and read garbage, and a garbage
            # height drives the ray slide-back — capped at 0.20 this injected
            # up to ~100 mm of toward-camera shift on the far half of the
            # table (measured 2026-08-19, the "blue cube isn't where it says"
            # bug). Plain planar's worst case is the true parallax, ~12 mm
            # for a cube: fall back to it rather than trust a bounded lie.
            if h_top > TABLETOP_HEIGHT_SANITY_M:
                h_top = 0.0
        cam = np.asarray(self.t_base_cam, dtype=float)[:3, 3]
        denom = cam[2] - self.plane_z
        if abs(denom) > 1e-6 and h_top > 0.0:
            s = (cam[2] - self.plane_z - h_top) / denom
            xy = cam[:2] + s * (xy - cam[:2])
        return np.array([xy[0], xy[1], self.plane_z + h_top / 2.0])

    def deproject_region(self, u0: int, v0: int, u1: int, v1: int,
                         trim: float = 0.15, support_z: Optional[float] = None,
                         clearance: float = 0.005,
                         mask: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """World-frame centre of the object occupying a 2D region.

        A single pixel reports the object's NEAR SURFACE, not its centre — the
        estimate is biased toward the camera by roughly half the object's depth.
        On a small can that is a few centimetres; on a basket it was measured at
        79 mm, which is enough to release the object on the rim instead of
        inside.

        Three things narrow "the pixels in the box" toward "the pixels of the
        object", in increasing order of how much they know:

          `trim`      drops the nearest and farthest depth quantiles, so a box
                      that clips background does not drag the estimate away;
          `support_z` drops everything at or below the surface the object stands
                      on — exact for the table, useless against a neighbour;
          `mask`      an HxW boolean from SAM2 saying which pixels ARE the object.
                      This is the only one that separates two touching objects,
                      or a bowl's wall from the table seen through its opening.

        `mask` is optional and independent: with it the estimate is computed over
        the object alone, without it over the box, and nothing else changes.
        """
        if self.depth is None or self.intrinsics is None or self.t_base_cam is None:
            # Depth-free rigs deproject onto the table plane, where the box
            # centre already IS the footprint centre — no bias to correct.
            return self.deproject((u0 + u1) // 2, (v0 + v1) // 2)
        if self.h_pixel_world is not None:
            # A camera that carries a board-fitted homography trusts the PLANE,
            # not the depth: this rig's overhead D405 sits ~1 m up, past its
            # working range, and its absolute depth measured 18 mm off median
            # against the 1.1 mm board fit. Absolute depth is unusable there —
            # but RELATIVE depth (object top vs the table beside it) keeps its
            # accuracy because the local bias cancels in the difference.
            return self._planar_region(u0, v0, u1, v1, mask=mask)
        h, w = self.depth.shape[:2]
        u0, u1 = sorted((int(np.clip(u0, 0, w - 1)), int(np.clip(u1, 0, w - 1))))
        v0, v1 = sorted((int(np.clip(v0, 0, h - 1)), int(np.clip(v1, 0, h - 1))))
        patch = self.depth[v0 : v1 + 1, u0 : u1 + 1]
        if patch.size == 0:
            return None
        vs, us = np.mgrid[v0 : v1 + 1, u0 : u1 + 1]
        ok = np.isfinite(patch) & (patch > 0)
        if mask is not None:
            m = np.asarray(mask, dtype=bool)
            if m.shape[:2] != (h, w):
                raise ValueError(f"mask is {m.shape[:2]}, image is {(h, w)}")
            # Too few masked pixels have valid depth to work with: fall through
            # on the box rather than return nothing. A degraded estimate beats a
            # skill that reports the object missing because a mask was thin.
            if (ok & m[v0 : v1 + 1, u0 : u1 + 1]).sum() >= 8:
                ok = ok & m[v0 : v1 + 1, u0 : u1 + 1]
        if ok.sum() < 8:
            return None
        z = patch[ok].astype(float)
        us = us[ok].astype(float)
        vs = vs[ok].astype(float)
        # One object's depths are unimodal. A handful of pixels from a DIFFERENT
        # surface — the shelf a box stands on, the arm crossing in front — are
        # not, and they are lethal to the silhouette rule below, which reads the
        # extremes and so has no averaging to hide behind. Measured: a mask that
        # clipped the object underneath moved the answer by 92 mm. Cut on median
        # absolute deviation rather than a fixed quantile, so a uniform object
        # keeps all its points and a contaminated one loses only the intruders.
        if z.size > 20:
            med = np.median(z)
            mad = np.median(np.abs(z - med))
            span = max(DEPTH_CONSISTENCY_MIN_M, DEPTH_CONSISTENCY_K * mad)
            keep = np.abs(z - med) <= span
            if keep.sum() >= 8:
                z, us, vs = z[keep], us[keep], vs[keep]
        if 0.0 < trim < 0.5 and z.size > 20:
            lo, hi = np.quantile(z, [trim, 1.0 - trim])
            keep = (z >= lo) & (z <= hi)
            if keep.sum() >= 8:
                z, us, vs = z[keep], us[keep], vs[keep]
        fx, fy = self.intrinsics[0, 0], self.intrinsics[1, 1]
        cx, cy = self.intrinsics[0, 2], self.intrinsics[1, 2]
        pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)])
        world = (self.t_base_cam @ pts)[:3]

        if support_z is not None:
            # A box drawn around an object also contains the surface it stands on
            # and whatever is behind it. Those points drag the estimate toward
            # the camera and downward. Everything above the support plane is the
            # object; everything at or below it is not.
            above = world[2] > float(support_z) + clearance
            if above.sum() >= 8:
                world = world[:, above]

        # The centroid of a visible surface is not the centre of the object: from
        # an oblique view you see the near wall and the far interior, never the
        # far wall, so the average sits in front of where the object really is.
        # The silhouette's extremes do not have that problem — the leftmost and
        # rightmost points ARE the object's edges — so take their midpoint.
        lo_xy = np.percentile(world[:2], 5, axis=1)
        hi_xy = np.percentile(world[:2], 95, axis=1)
        centre_xy = (lo_xy + hi_xy) / 2.0
        top = float(np.percentile(world[2], 95))
        # Looking down, the only height you ever measure is the top surface. The
        # centre is halfway between that and the surface the object stands on —
        # which is a quantity we know, so use it instead of guessing.
        centre_z = (float(support_z) + top) / 2.0 if support_z is not None else float(np.median(world[2]))
        return np.array([centre_xy[0], centre_xy[1], centre_z])


def triangulate(views: list[tuple[np.ndarray, tuple[float, float]]]) -> Optional[np.ndarray]:
    """Least-squares 3D point from >=2 views of (3x4 projection matrix, (u, v)).

    Each view contributes two linear constraints (u*P3 - P1)·X = 0 and
    (v*P3 - P2)·X = 0; the smallest singular vector is the point. Depth-free.
    """
    rows = []
    for proj, (u, v) in views:
        if proj is None:
            continue
        rows.append(u * proj[2] - proj[0])
        rows.append(v * proj[2] - proj[1])
    if len(rows) < 4:
        return None
    _, _, vt = np.linalg.svd(np.asarray(rows, dtype=float))
    x = vt[-1]
    if abs(x[3]) < 1e-12:
        return None
    return x[:3] / x[3]


def reprojection_error(proj: np.ndarray, xyz: np.ndarray, px: tuple[float, float]) -> float:
    p = proj @ np.array([xyz[0], xyz[1], xyz[2], 1.0])
    if abs(p[2]) < 1e-12:
        return float("inf")
    return float(np.hypot(p[0] / p[2] - px[0], p[1] / p[2] - px[1]))


@dataclass
class SkillResult:
    ok: bool  # the skill *ran to completion*; effect truth comes from verification
    info: str = ""
    proprio: dict[str, float] = field(default_factory=dict)  # gripper width/effort etc.
    frames: list[Frame] = field(default_factory=list)
    error: Optional[str] = None
