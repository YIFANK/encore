"""Episode journal: every model call, motion, belief update, verification, and
program edit goes to one append-only JSONL with saved frames alongside.

This is Heron's lightweight take on Retriever's reproducibility goal: an
episode can be re-read, re-rendered, and re-judged offline (`heron replay`),
and failures can be audited from evidence rather than reconstructed from
chat scrollback.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Optional

from PIL import Image

from .types import Frame


def _jsonable(x: Any) -> Any:
    if hasattr(x, "model_dump"):
        return x.model_dump(mode="json")
    if isinstance(x, (list, tuple)):
        return [_jsonable(i) for i in x]
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if hasattr(x, "tolist"):
        return x.tolist()
    return x


class EpisodeLogger:
    def __init__(self, root: str | Path, name: Optional[str] = None) -> None:
        # One directory per episode, even when two finish inside the same second.
        # The stamp is second-resolution and there was no collision handling, so
        # fast episodes silently SHARED a directory: three offline demo runs in a
        # row produced one journal with all three interleaved, one run.gif, and a
        # skill whose provenance listed a single episode because all three had
        # the same name. Sweeps run episodes back to back, which is exactly where
        # the evidence matters most.
        stamp = time.strftime("%Y%m%d-%H%M%S")
        base = Path(root) / (f"{stamp}-{name}" if name else stamp)
        self.dir = base
        for n in range(1, 1000):
            if not self.dir.exists():
                break
            self.dir = base.with_name(f"{base.name}-{n}")
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self._journal = (self.dir / "journal.jsonl").open("a")
        self._frame_counter = 0
        self.t0 = time.time()

    def event(self, kind: str, **data: Any) -> None:
        rec = {"t": round(time.time() - self.t0, 3), "kind": kind, **{k: _jsonable(v) for k, v in data.items()}}
        self._journal.write(json.dumps(rec) + "\n")
        self._journal.flush()
        self._echo(rec)

    # Events worth watching a run by, and how to say each in one line. Everything
    # else still goes to the journal; this is only about what a person standing
    # at the rig can follow. Without it the terminal sits silent for minutes at a
    # time while the arm moves, which reads as a hang.
    _ECHO = {
        "step_start": lambda r: f"-> {r['skill']} {r.get('args', '')}",
        "skill_exec": lambda r: ("   ok: " + str(r.get("info", ""))[:110]) if r.get("ok")
                                else "   FAILED: " + str(r.get("error", ""))[:110],
        "grounding": lambda r: (f"   found {r['entity']} at "
                                f"{[round(v, 3) for v in r['xyz']]} ({r.get('method', '')})"),
        # The reasoning, not just the verdict: an operator deciding whether to
        # hit s/f needs to see WHY the verifier answered, WHY the repair chose
        # that edit, and WHAT the plan intends — a bare "planned" told them
        # nothing while three model calls' worth of rationale sat in the journal.
        "verify": lambda r: f"   {r['predicate']} = {r['value']}"
                            + (f"  — {str(r['note'])[:120]}" if r.get("note") else ""),
        "failure": lambda r: f"   ! {r.get('failure_class', '')}: {str(r.get('summary', ''))[:140]}",
        "program_edit": lambda r: f"   repair: {r.get('edit', {}).get('type', '?')}"
                                  + (f"  — {str(r['edit']['rationale'])[:140]}"
                                     if r.get("edit", {}).get("rationale") else ""),
        "contact": lambda r: f"   felt the surface at z={r['z']}",
        "approach_height_lowered": lambda r: (f"   hover lowered {r['wanted']} -> {r['used']} "
                                              f"(unreachable up there)"),
        "fast_descent": lambda r: f"   descending {r['from_z']} -> {r['to_z']}",
        "identify_candidates": lambda r: (f"   detector found nothing; asking about "
                                          f"{r['asking']} of {r['proposals']} regions"),
        "grounding_rejected": lambda r: (f"   refused {r['entity']} sighting"
                                         + (f" (jump {r['jump_m']}m > {r['limit_m']}m)"
                                            if r.get("jump_m") is not None else "")
                                         + f": {str(r.get('reason', ''))[:110]}"),
        "crop_recheck_rejected": lambda r: (f"   close-up says the chosen patch is NOT "
                                            f"{r.get('desc', r.get('entity', '?'))}; re-looking"),
        "proposal_recheck_rejected": lambda r: (f"   close-up says crop {r.get('crop', '?')} is NOT "
                                                f"{r.get('desc', r.get('entity', '?'))}; re-looking"),
        "verify_view_blocked": lambda r: (f"   {r.get('camera', '?')} cannot see the subject "
                                          f"(arm on the sightline); trying another view"),
        "proposal_goal_pending_choice": lambda r: (
            f"   {r['entity']}: several lookalikes; binding the one "
            f"{'already on' if r.get('negated') else 'not yet on'} "
            f"{r['anchor']} (the goal still needs it)"),
        "plan": lambda r: "   plan: " + " -> ".join(
            f"{s.get('skill', '?')}({', '.join(str(v) for v in (s.get('args') or {}).values())})"
            for s in (r.get("program", {}).get("steps") or [])) + "  | goal: " + ", ".join(
            ("not " if p.get("negated") else "") + f"{p.get('name')}({','.join(p.get('args', []))})"
            for p in (r.get("program", {}).get("goal_predicates") or [])),
        "episode_start": lambda r: f"== {r.get('instruction', '')}",
    }

    def _echo(self, rec: dict) -> None:
        fn = self._ECHO.get(rec.get("kind", ""))
        if fn is None:
            return
        try:
            line = fn(rec)
        except Exception:
            return
        print(f"[{rec['t']:7.1f}s] {line}", flush=True)

    def frame(self, frame: Frame, tag: str = "") -> str:
        self._frame_counter += 1
        name = f"{self._frame_counter:04d}_{frame.camera}{('_' + tag) if tag else ''}.png"
        path = self.dir / "frames" / name
        Image.fromarray(frame.rgb).save(path)
        self.event("frame", camera=frame.camera, tag=tag, path=str(path.relative_to(self.dir)))
        return str(path)

    def model_call(self, role: str, prompt_chars: int, images: int, latency_s: float,
                   response_preview: str, thinking: str = "") -> None:
        self.event(
            "model_call", role=role, prompt_chars=prompt_chars, images=images,
            latency_s=round(latency_s, 2), thinking=thinking, response_preview=response_preview[:400],
        )

    def snapshot(self, name: str, payload: Any) -> None:
        (self.dir / f"{name}.json").write_text(json.dumps(_jsonable(payload), indent=2))

    def close(self, **summary: Any) -> None:
        self.event("episode_end", **summary)
        self.snapshot("summary", summary)
        self._journal.close()


def read_journal(episode_dir: str | Path) -> list[dict[str, Any]]:
    path = Path(episode_dir) / "journal.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
