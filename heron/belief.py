"""Grounded belief store: entities, three-valued predicates, staleness, evidence.

The executor and verifiers read/write this instead of stuffing observations
into prompt history. Every write carries Evidence, so any belief can be audited
back to the frame or sensor reading that produced it.
"""
from __future__ import annotations

import itertools
import time
from typing import Optional

from .types import EntityDecl, EntityTrack, Evidence, PredicateInstance, PredicateSpec, Value

_evidence_counter = itertools.count(1)


class BeliefStore:
    def __init__(self) -> None:
        self.entities: dict[str, EntityTrack] = {}
        self.predicates: dict[str, PredicateInstance] = {}
        self.evidence: dict[str, Evidence] = {}
        # Full verification history (never overwritten) — diagnosis reads this to
        # distinguish "was established, then changed" from "never established".
        self.history: list[tuple[str, PredicateInstance]] = []

    # -- entities -----------------------------------------------------------
    def declare_entities(self, decls: list[EntityDecl]) -> None:
        for d in decls:
            if d.id not in self.entities:
                self.entities[d.id] = EntityTrack(id=d.id, description=d.description,
                                                  relation=d.relation)
            else:
                self.entities[d.id].description = d.description
                # A reground_entity edit rewrites the description and says nothing
                # about the relation, so an existing one is kept unless the new
                # declaration supplies its own.
                if d.relation is not None:
                    self.entities[d.id].relation = d.relation

    def track(self, entity_id: str) -> EntityTrack:
        if entity_id not in self.entities:
            self.entities[entity_id] = EntityTrack(id=entity_id, description=entity_id)
        return self.entities[entity_id]

    def update_track(
        self,
        entity_id: str,
        *,
        xyz_base: Optional[tuple[float, float, float]] = None,
        camera: Optional[str] = None,
        px: Optional[tuple[int, int]] = None,
        span_m: Optional[float] = None,
        confidence: float = 1.0,
        held_by: Optional[str] = "__keep__",
        from_sighting: bool = False,
        note: str = "",
    ) -> EntityTrack:
        tr = self.track(entity_id)
        if xyz_base is not None:
            tr.xyz_base = tuple(float(x) for x in xyz_base)
            # Only a real sighting establishes the baseline, and only the first
            # one: a pose written by pick or place is where we MEANT to put it,
            # and a baseline made of intentions cannot later be evidence that
            # something moved.
            if from_sighting and tr.first_xyz is None:
                tr.first_xyz = tr.xyz_base
        tr.camera = camera if camera is not None else tr.camera
        tr.px = px if px is not None else tr.px
        tr.span_m = span_m if span_m is not None else tr.span_m
        tr.confidence = confidence
        tr.from_sighting = bool(from_sighting)
        tr.t = time.time()
        if held_by != "__keep__":
            tr.held_by = held_by
        if note:
            tr.notes = (tr.notes + "; " + note).strip("; ")
        return tr

    def invalidate_entity(self, entity_id: str, reason: str = "") -> None:
        """Mark an entity's location belief stale (e.g. after drift is detected)."""
        tr = self.track(entity_id)
        tr.confidence = 0.0
        tr.t = 0.0
        if reason:
            tr.notes = (tr.notes + "; " + reason).strip("; ")
        # Location-dependent predicates about it are unknown now. Fresh instances,
        # never in-place mutation — history must stay a faithful record.
        for key in list(self.predicates):
            if f"({entity_id}" in key or f",{entity_id}" in key or f"{entity_id})" in key:
                self.predicates[key] = PredicateInstance(value=Value.UNKNOWN, confidence=0.0, t=time.time())

    # -- predicates ---------------------------------------------------------
    def add_evidence(self, kind: str, detail: str = "", camera: str | None = None, image_path: str | None = None) -> Evidence:
        ev = Evidence(id=f"ev{next(_evidence_counter)}", kind=kind, detail=detail, camera=camera, image_path=image_path)
        self.evidence[ev.id] = ev
        return ev

    def set_predicate(self, spec: PredicateSpec, value: Value, confidence: float, evidence: Evidence) -> None:
        inst = PredicateInstance(value=value, confidence=confidence, t=time.time(), evidence_id=evidence.id)
        self.predicates[spec.key] = inst
        self.history.append((spec.key, inst.model_copy()))

    def was_ever(self, spec: PredicateSpec, value: Value) -> bool:
        return any(key == spec.key and inst.value is value for key, inst in self.history)

    def get(self, spec: PredicateSpec, max_age_s: float | None = None) -> Value:
        """Raw stored value for the (un-negated) predicate; UNKNOWN if absent/stale."""
        inst = self.predicates.get(spec.key)
        if inst is None:
            return Value.UNKNOWN
        if max_age_s is not None and inst.age() > max_age_s:
            return Value.UNKNOWN
        return inst.value

    def holds(self, spec: PredicateSpec, max_age_s: float | None = None) -> Value:
        """Truth of the spec including negation."""
        return spec.satisfied_by(self.get(spec, max_age_s=max_age_s))

    # -- rendering ----------------------------------------------------------
    def summary(self) -> str:
        lines = ["BELIEF:"]
        for tr in self.entities.values():
            loc = (
                f"xyz={tuple(round(x, 3) for x in tr.xyz_base)}" if tr.xyz_base else "location unknown"
            )
            held = f" HELD by {tr.held_by} arm;" if tr.held_by else ""
            age = f"{tr.age():.0f}s ago" if tr.t else "never seen"
            notes = f" notes: {tr.notes}" if tr.notes else ""
            lines.append(f"  {tr.id} ({tr.description!r}): {loc}, conf={tr.confidence:.2f}, seen {age};{held}{notes}")
        known = [f"  {k} = {v.value.value} (conf {v.confidence:.2f}, {v.age():.0f}s old)" for k, v in self.predicates.items()]
        lines += ["PREDICATES:"] + (known or ["  (none verified yet)"])
        return "\n".join(lines)
