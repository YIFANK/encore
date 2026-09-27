"""Persistent Gemini Live session as a transport for the vision-heavy roles.

Measured with tools/live_probe.py: session connect 0.36 s, frame+text to
answer in 1-2 s on gemini-robotics-er-2-streaming-preview — versus 2-3 s per
REST locate and a fresh connection every call. The session is long-lived and
the conversation context stays server-side, so consecutive grounding calls
against the same scene stop re-paying for the frame.

Scope: TEXT-answer roles only (point/locate/vqa/proposal choice). plan and
repair keep the REST path — they are rare, schema-constrained calls where a
fresh request is fine. Any Live failure falls back to REST for that call and
the session lazily reconnects on the next one; the rig must never be blocked
by a dead socket.

Heron is synchronous, so the asyncio Live client runs on a dedicated daemon
thread with its own loop, and ask() bridges via run_coroutine_threadsafe.
"""
from __future__ import annotations

import asyncio
import io
import threading
import time
from typing import Optional

import numpy as np


class LiveSession:
    def __init__(self, model: str, system_instruction: str,
                 connect_timeout_s: float = 15.0, log=None) -> None:
        self.model = model
        self.system_instruction = system_instruction
        self.connect_timeout_s = connect_timeout_s
        self.log = log
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._session = None
        self._cm = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    # -- lifecycle -----------------------------------------------------------
    def _ensure_loop(self) -> asyncio.AbstractEventLoop:
        if self._loop is None or not self._thread or not self._thread.is_alive():
            loop = asyncio.new_event_loop()
            t = threading.Thread(target=loop.run_forever, name="gemini-live", daemon=True)
            t.start()
            self._loop, self._thread = loop, t
        return self._loop

    # Answer tools: the streaming model is scrupulous about CALLING tools
    # (probe: 1-2 s, well-formed) and sloppy about emitting JSON text (measured:
    # a 34 s episode became 294 s of parse-retry spirals). So every question is
    # answered by calling the matching tool; the args ARE the structured answer.
    _TOOLS = {
        "answer_points": {
            "description": "Report every matching object as image points.",
            "parameters": {"type": "OBJECT", "properties": {"points": {
                "type": "ARRAY", "items": {"type": "OBJECT", "properties": {
                    "y": {"type": "INTEGER"}, "x": {"type": "INTEGER"},
                    "label": {"type": "STRING"},
                    "confidence": {"type": "NUMBER"}},
                    "required": ["y", "x"]}}},
                "required": ["points"]},
        },
        "answer_boxes": {
            "description": "Report every matching object as 2D bounding boxes.",
            "parameters": {"type": "OBJECT", "properties": {"boxes": {
                "type": "ARRAY", "items": {"type": "OBJECT", "properties": {
                    "y": {"type": "INTEGER"}, "x": {"type": "INTEGER"},
                    "y2": {"type": "INTEGER"}, "x2": {"type": "INTEGER"},
                    "label": {"type": "STRING"},
                    "confidence": {"type": "NUMBER"}},
                    "required": ["y", "x", "y2", "x2"]}}},
                "required": ["boxes"]},
        },
        "answer_vqa": {
            "description": "Answer the yes/no visual question.",
            "parameters": {"type": "OBJECT", "properties": {
                "answer": {"type": "STRING", "enum": ["yes", "no", "unsure"]},
                "confidence": {"type": "NUMBER"},
                "reason": {"type": "STRING"}},
                "required": ["answer", "confidence"]},
        },
    }

    async def _connect(self):
        from google import genai
        from google.genai import types

        client = genai.Client(http_options={"api_version": "v1alpha"})
        tools = [types.Tool(function_declarations=[
            types.FunctionDeclaration(name=n, description=t["description"],
                                      parameters=t["parameters"])
            for n, t in self._TOOLS.items()])]
        cm = client.aio.live.connect(
            model=self.model,
            config=types.LiveConnectConfig(
                response_modalities=["TEXT"],
                tools=tools,
                system_instruction=self.system_instruction,
            ),
        )
        session = await cm.__aenter__()
        return cm, session

    def _get_session(self):
        if self._session is not None:
            return self._session
        loop = self._ensure_loop()
        fut = asyncio.run_coroutine_threadsafe(self._connect(), loop)
        self._cm, self._session = fut.result(timeout=self.connect_timeout_s)
        if self.log is not None:
            self.log.event("live_session_connected", model=self.model)
        return self._session

    def drop(self) -> None:
        """Forget the session; the next ask() reconnects."""
        loop, cm = self._loop, self._cm
        self._session = None
        self._cm = None
        if loop is not None and cm is not None:
            async def _close():
                try:
                    await cm.__aexit__(None, None, None)
                except Exception:
                    pass
            try:
                asyncio.run_coroutine_threadsafe(_close(), loop).result(timeout=3)
            except Exception:
                pass

    # -- the one verb --------------------------------------------------------
    def ask(self, text: str, frames: list | None = None,
            timeout_s: float = 30.0) -> str:
        """Send frames + text on the persistent session, return the text answer."""
        with self._lock:   # the Live session is a single conversation
            session = self._get_session()
            loop = self._ensure_loop()
            fut = asyncio.run_coroutine_threadsafe(
                self._ask(session, text, frames or []), loop)
            try:
                return fut.result(timeout=timeout_s)
            except Exception:
                self.drop()
                raise

    async def _ask(self, session, text: str, frames: list) -> str:
        import cv2
        from google.genai import types

        parts = []
        for f in frames:
            rgb = getattr(f, "rgb", f)
            if rgb is None:
                continue
            ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok:
                parts.append(types.Part(inline_data=types.Blob(
                    data=jpg.tobytes(), mime_type="image/jpeg")))
        parts.append(types.Part(text=text))
        await session.send_client_content(
            turns=types.Content(role="user", parts=parts))
        out = []
        async for msg in session.receive():
            if msg.tool_call and msg.tool_call.function_calls:
                fc = msg.tool_call.function_calls[0]
                await session.send_tool_response(function_responses=[
                    types.FunctionResponse(id=fc.id, name=fc.name,
                                           response={"ok": True})])
                # DRAIN to turn_complete before returning: after the tool
                # response the model emits a closing turn, and leaving it in
                # the stream made the NEXT question read a stale empty answer
                # (measured: alternating good/empty answers, 53 parse errors,
                # 0.0 s "instant" replies that were leftovers).
                async for tail in session.receive():
                    tsc = tail.server_content
                    if tsc and tsc.turn_complete:
                        break
                return self._serialize(fc.name, dict(fc.args or {}))
            sc = msg.server_content
            if sc and sc.model_turn:
                for p in sc.model_turn.parts or []:
                    if p.text:
                        out.append(p.text)
            if sc and sc.turn_complete:
                break
        return "".join(out)

    @staticmethod
    def _serialize(tool: str, args: dict) -> str:
        """Tool args -> exactly the JSON text the existing parsers expect."""
        import json
        if tool == "answer_points":
            return json.dumps([
                {"point": [int(p.get("y", 0)), int(p.get("x", 0))],
                 "label": p.get("label", ""),
                 "confidence": float(p.get("confidence", 0.6))}
                for p in args.get("points", [])])
        if tool == "answer_boxes":
            return json.dumps([
                {"y": int(b.get("y", 0)), "x": int(b.get("x", 0)),
                 "y2": int(b.get("y2", 0)), "x2": int(b.get("x2", 0)),
                 "label": b.get("label", ""),
                 "confidence": float(b.get("confidence", 0.6))}
                for b in args.get("boxes", [])])
        if tool == "answer_vqa":
            return json.dumps({"answer": args.get("answer", "unsure"),
                               "confidence": float(args.get("confidence", 0.5)),
                               "reason": args.get("reason", "")})
        return json.dumps(args)
