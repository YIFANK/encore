"""A grounder that already knows the answer, for probing everything else.

The twin has true poses. Handing them to the skills as if a detector had
produced them separates two questions that otherwise arrive as one number: can
the arms do this at all, and can perception see well enough for them to. A
probe that runs on ground truth and still fails has found a motion problem.

It is deliberately a Grounder and nothing more — it answers where things are in
the frame, and every downstream step (deprojection, support heights, the
contact descent) runs exactly as it does with Gemini on the other end.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ..types import Frame
from .scripted import project


class SimTruthGrounder:
    """Grounds by name against the simulator's own bodies.

    Matching is by word overlap with the body name, the same loose match a real
    grounder is asked to do — "the red block" finds `red_block`. A description
    that matches nothing returns None rather than guessing, so a probe cannot
    quietly succeed on the wrong object.
    """

    def __init__(self, sim, bodies: list[str] | None = None) -> None:
        self.sim = sim
        self.bodies = list(bodies) if bodies else None
        self.calls = 0

    def _resolve(self, query: str) -> Optional[str]:
        names = self.bodies if self.bodies is not None else self._scene_bodies()
        words = set(query.lower().replace("-", " ").replace("_", " ").split())
        best, score = None, 0
        for name in names:
            n = len(words & set(name.lower().split("_")))
            if n > score:
                best, score = name, n
        return best if score else None

    def _scene_bodies(self) -> list[str]:
        m = self.sim.model
        mj = self.sim._mj
        out = []
        for i in range(m.nbody):
            nm = mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, i)
            # Bodies with a free joint are the movable scene; the arms and the
            # table are not things anybody asks a detector to find.
            if nm and m.body_jntadr[i] >= 0 \
                    and m.jnt_type[m.body_jntadr[i]] == mj.mjtJoint.mjJNT_FREE:
                out.append(nm)
        return out

    def _top_of(self, name: str) -> Optional[np.ndarray]:
        try:
            return np.asarray(self.sim.body_xyz(name), dtype=float)
        except KeyError:
            return None

    # -- Grounder ------------------------------------------------------------
    def point(self, frame: Frame, query: str) -> Optional[tuple[int, int, float]]:
        self.calls += 1
        name = self._resolve(query)
        if name is None:
            return None
        p = self._top_of(name)
        px = project(frame, p) if p is not None else None
        return (*px, 0.99) if px else None

    def points(self, frame: Frame, query: str) -> list[tuple[int, int, float]]:
        hit = self.point(frame, query)
        return [hit] if hit else []

    def box(self, frame: Frame, query: str) -> Optional[tuple[int, int, int, int, float]]:
        self.calls += 1
        name = self._resolve(query)
        if name is None:
            return None
        p = self._top_of(name)
        if p is None:
            return None
        half = self._half_extent(name)
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                px = project(frame, p + np.array([sx * half[0], sy * half[1], half[2]]))
                if px is None:
                    return None
                corners.append(px)
        us = [c[0] for c in corners]
        vs = [c[1] for c in corners]
        return min(us), min(vs), max(us), max(vs), 0.99

    def boxes(self, frame: Frame, query: str):
        hit = self.box(frame, query)
        return [hit] if hit else []

    def _half_extent(self, name: str) -> np.ndarray:
        m, mj = self.sim.model, self.sim._mj
        bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, name)
        best = np.array([0.02, 0.02, 0.02])
        for g in range(m.body_geomadr[bid], m.body_geomadr[bid] + m.body_geomnum[bid]):
            best = np.maximum(best, np.asarray(m.geom_size[g][:3], dtype=float))
        return best

    def vqa(self, frame: Frame, question: str) -> str:
        # A probe that needs an opinion has outgrown ground truth. Saying so is
        # better than inventing a yes and having a verifier believe it.
        raise NotImplementedError(
            f"SimTruthGrounder has no opinion to give: {question!r}. Run this with a "
            f"real orchestrator if the question is a judgement rather than a position.")
