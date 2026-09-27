"""A live window into what the agent is seeing, believing and doing.

Not a report written afterwards. This runs while the episode runs: the camera
frame the agent is actually working from, the grounded points drawn on it where
the agent thinks they are, and the plan advancing beside them.

    from heron.console import Console
    console = Console(agent).start(port=8770)      # http://127.0.0.1:8770

Two things make it safe to leave running. It never renders on its own thread —
MuJoCo's renderer is not thread-safe, so frames are stashed by a step listener
on the simulator's own thread and the HTTP thread only hands out bytes that
already exist. And it holds nothing the agent needs: every read is a snapshot
taken under a lock, so a slow browser cannot stall a robot.

Two controls so far, both episode-scoped by construction: assert a predicate
(the verifier is overruled for this episode, this predicate, and nothing else)
and mark a grounding "not there" (the belief is invalidated, forcing a
re-look). Each writes a Patch with the console state at the moment of the
click, because "what the operator was looking at" is the difference between a
motor lesson and a grounding lesson when the record is read later.
"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import numpy as np

# How often the frame stash is refreshed from the simulator, in sim-time steps
# per grab. Rendering is the expensive part of the loop, and a console at 5 Hz
# is as readable as one at 30 and costs a sixth as much.
# ONE camera per grab, round-robin. Rendering runs on the physics thread, so
# every frame the console takes is time the robot is not moving: two cameras at
# 5 Hz roughly doubled a pick-and-place episode. One camera at 2 Hz is a
# readable console for about a twentieth of that.
SIM_GRAB_HZ = 2.0
JPEG_QUALITY = 70
VIEW_W = 480
STALE_S = 3.0


class Console:
    """Live state for one agent, plus the HTTP server that shows it."""

    def __init__(self, agent: Any, cameras: Optional[list[str]] = None) -> None:
        self.agent = agent
        self.ctx = agent.ctx
        self.robot = agent.robot
        backend = getattr(self.robot, "_robot", self.robot)
        self.backend = backend
        self.cameras = list(cameras or getattr(backend, "cameras", []) or [])
        self._lock = threading.Lock()
        self._frames: dict[str, tuple[float, int, bytes, int, int]] = {}
        self._server: Optional[ThreadingHTTPServer] = None
        self._t0 = time.time()
        self._grab_every = 1
        self._since = 0
        self._grab_error = ""

    # -- frame supply --------------------------------------------------------
    def _stash(self, camera: str, rgb: np.ndarray) -> None:
        from PIL import Image  # noqa: PLC0415
        import io  # noqa: PLC0415

        im = Image.fromarray(np.asarray(rgb, dtype=np.uint8))
        if im.width > VIEW_W:
            im = im.resize((VIEW_W, max(1, round(im.height * VIEW_W / im.width))))
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=JPEG_QUALITY)
        with self._lock:
            self._frames[camera] = (time.time(), self._steps(), buf.getvalue(),
                                    im.width, im.height)

    def _steps(self) -> int:
        return int(getattr(self.backend, "_step_count", 0) or 0)

    def _attach_sim(self) -> bool:
        """Grab frames on the simulator's own thread, via its step listener.

        The alternative — rendering when the browser asks — puts a MuJoCo
        renderer on the HTTP thread while the physics thread is using it, which
        crashes the process rather than dropping a frame.
        """
        listeners = getattr(self.backend, "step_listeners", None)
        render = getattr(self.backend, "render_rgb", None)
        if listeners is None or not callable(render):
            return False
        timestep = float(self.backend.model.opt.timestep)
        self._grab_every = max(1, round(1.0 / (SIM_GRAB_HZ * timestep)))

        def grab() -> None:
            self._since += 1
            if self._since < self._grab_every:
                return
            self._since = 0
            self._next_cam = (getattr(self, "_next_cam", -1) + 1) % max(len(self.cameras), 1)
            for cam in self.cameras[self._next_cam:self._next_cam + 1]:
                try:
                    # THE SIZE THE BACKEND ALREADY RENDERS AT. Asking for a
                    # second size makes a second MuJoCo Renderer, and a second
                    # Renderer on macOS breaks the first one's context: frames
                    # stopped arriving fifteen seconds into the first run and
                    # the exception was being swallowed below. Downscaling in
                    # PIL costs a millisecond and keeps one renderer.
                    self._stash(cam, render(cam, 640, 480))
                except Exception as e:                          # noqa: BLE001
                    # ONCE, LOUDLY. A silent except in a frame grabber is a
                    # console that goes blank and does not say why.
                    if not self._grab_error:
                        self._grab_error = f"{type(e).__name__}: {e}"[:160]
                        print(f"[console] frame grab failed: {self._grab_error}",
                              flush=True)

        listeners.append(grab)
        self._grabber = grab
        return True

    def capture_now(self, camera: str) -> None:
        """Pull one frame from a real camera. For backends with no step loop."""
        try:
            self._stash(camera, self.robot.capture(camera).rgb)
        except Exception:
            pass

    def _is_stale(self, taken_at: float, taken_steps: int, now_steps: int) -> bool:
        """Has the WORLD moved since this frame, not the clock.

        In the twin nothing moves unless the physics steps, and an agent
        episode spends most of its wall time waiting on a model call — so a
        wall-clock rule blanked the console for exactly the twenty seconds a
        watcher most wants to look at the scene. On a real camera the world can
        move while the feed is stuck, so there the clock is the right rule.
        """
        if getattr(self, "_grabber", None) is not None:
            # "Any step at all" was far too strict: with cameras grabbed
            # round-robin the physics advances thousands of steps between one
            # camera's frames, so every frame was stale while the arm moved —
            # which is the only time anyone is watching. Stale means frames
            # have STOPPED arriving: more than a couple of grab intervals of
            # world has gone by since this one.
            budget = self._grab_every * max(len(self.cameras), 1) * 3
            return (now_steps - taken_steps) > budget
        return (time.time() - taken_at) > STALE_S

    def _eyes(self) -> dict:
        """Live pose + intrinsics per camera, for projecting the belief into it."""
        mj = getattr(self.backend, "_mj", None)
        model = getattr(self.backend, "model", None)
        out = {}
        if mj is None or model is None:
            return out
        for cam in self.cameras:
            cid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_CAMERA, cam)
            if cid < 0:
                continue
            t = np.eye(4)
            t[:3, :3] = self.backend.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
            t[:3, 3] = self.backend.data.cam_xpos[cid]
            out[cam] = (t, float(model.cam_fovy[cid]))
        return out

    def _project(self, eyes: dict, camera: str, xyz, w: int, h: int):
        """Where this world point falls in this camera's picture, right now."""
        got = eyes.get(camera)
        if got is None or xyz is None:
            return None
        t, fovy = got
        f = 0.5 * h / np.tan(np.radians(fovy) / 2.0)
        p = np.linalg.inv(t) @ np.append(np.asarray(xyz, dtype=float)[:3], 1.0)
        if p[2] <= 1e-6:
            return None
        u = w / 2.0 + f * p[0] / p[2]
        v = h / 2.0 + f * p[1] / p[2]
        return [round(float(u), 1), round(float(v), 1)] if 0 <= u < w and 0 <= v < h else None

    # -- state ---------------------------------------------------------------
    def snapshot(self) -> dict:
        """Everything the page draws, taken at one instant.

        Read straight off the belief store and the running program rather than
        reconstructed from the journal: the belief is what the agent will ACT
        on, and a console that shows a reconstruction can disagree with the
        thing it is supposed to be a window into.
        """
        belief = getattr(self.ctx, "belief", None)
        eyes = self._eyes()
        with self._lock:
            sizes = {c: (w, h) for c, (_t, _s, _b, w, h) in self._frames.items()}
        tracks = []
        if belief is not None:
            for tr in belief.entities.values():
                # WHERE IT IS BELIEVED TO BE NOW, in every view — not the pixel
                # the last sighting landed on. The stored px belongs to the
                # frame it was measured in; once pick moves the object the
                # belief's xyz is dead-reckoned forward and that pixel is a
                # picture of where the object USED to be. A console that draws
                # the old pixel invites a person to correct a grounding against
                # a scene that has moved — the first screenshot of this page
                # had a marker sitting 25 s and half a table away from the
                # block it named.
                at = {c: self._project(eyes, c, tr.xyz_base, w, h)
                      for c, (w, h) in sizes.items()}
                tracks.append({
                    "id": tr.id, "description": tr.description,
                    "at": {c: v for c, v in at.items() if v},
                    "px": list(tr.px) if tr.px else None,
                    "camera": tr.camera,
                    "xyz": [round(float(v), 4) for v in tr.xyz_base] if tr.xyz_base else None,
                    "conf": round(float(tr.confidence), 2),
                    "age_s": round(tr.age(), 1) if tr.t else None,
                    "seen": bool(tr.from_sighting),
                    "held_by": tr.held_by,
                })
        program = getattr(self.agent, "program", None)
        steps, goal = [], []
        if program is not None:
            for s in getattr(program, "steps", []) or []:
                steps.append({"id": s.id, "skill": s.skill,
                              "args": {k: str(v) for k, v in (s.args or {}).items()},
                              "status": str(getattr(s.status, "value", s.status))})
            for p in getattr(program, "goal_predicates", []) or []:
                goal.append(str(p))
        # WHAT IT IS DOING RIGHT NOW, inside the step. A stuck arm and an arm
        # descending 5 mm at a time look identical on a camera; they differ by
        # one word and one number here.
        import time as _time  # noqa: PLC0415
        lp = getattr(self.ctx, "live_phase", None)
        phase = None
        if lp:
            now = _time.perf_counter()
            in_skill = now - float(lp.get("t0", now))
            phase = {"skill": lp.get("skill", ""), "entity": lp.get("entity", "") or "",
                     "done": lp.get("done"),
                     "in_skill_s": round(in_skill, 1),
                     "in_phase_s": round(in_skill - float(lp.get("since", 0.0)), 1)}
        preds = []
        for key, val in (getattr(belief, "predicates", {}) or {}).items():
            preds.append({"key": str(key),
                          "value": str(getattr(val, "value", val)),
                          "note": str(getattr(val, "note", ""))[:160]})
        with self._lock:
            now_steps = self._steps()
            cams = {c: {"age_s": round(time.time() - t, 1),
                        "stale": self._is_stale(t, st, now_steps), "w": w, "h": h}
                    for c, (t, st, _b, w, h) in self._frames.items()}
        return {
            "t": round(time.time() - self._t0, 1),
            "instruction": getattr(self.agent, "instruction", "") or "",
            "cameras": cams, "tracks": tracks, "steps": steps,
            "goal": goal, "predicates": preds, "phase": phase,
        }

    # -- corrections ---------------------------------------------------------
    def correct(self, body: dict) -> dict:
        """One operator correction, applied narrowly and recorded as a Patch.

        The console's rule (reports/correction-console.md): apply now at the
        narrowest level that fixes this episode; attribution to anything
        broader happens later, on evidence. Everything written here is
        episode-scoped, so the store's own promotion rule retires it unless
        validation later says otherwise. `operator_saw` is the console state at
        the moment of the click — without it a correction cannot be interpreted
        after the fact, because "what the operator was looking at" is the
        difference between a motor lesson and a grounding lesson.
        """
        from .patches import Patch, PatchLevel, Scope  # noqa: PLC0415
        import os  # noqa: PLC0415

        store = getattr(self.ctx, "patches", None)
        if store is None:
            return {"ok": False, "error": "no patch store on this agent"}
        kind = body.get("kind", "")
        episode = os.path.basename(str(getattr(self.ctx.log, "dir", "live")))
        saw = self.snapshot()
        saw.pop("cameras", None)     # sizes/ages, not evidence; frames live on disk
        if kind == "predicate_assert":
            key, value = body.get("predicate", ""), body.get("value", "")
            if value not in ("true", "false") or not key:
                return {"ok": False, "error": "predicate_assert needs predicate + true/false"}
            patch = store.add(Patch(
                level=PatchLevel.PREDICATE, target=key,
                delta={"assert": value, "operator_saw": saw},
                scope=Scope(episode=episode),
                provenance="operator:predicate_panel",
                rationale=body.get("note", "")))
            self.ctx.log.event("operator_correction", correction=kind,
                               predicate=key, value=value, patch=patch.id)
            return {"ok": True, "patch": patch.id}
        if kind == "not_there":
            entity = body.get("entity", "")
            if entity not in getattr(self.ctx.belief, "entities", {}):
                return {"ok": False, "error": f"unknown entity {entity!r}"}
            self.ctx.belief.invalidate_entity(entity, reason="operator: not there")
            patch = store.add(Patch(
                level=PatchLevel.GROUNDING, target=entity,
                delta={"invalidate": True, "operator_saw": saw},
                scope=Scope(episode=episode, entity_id=entity),
                provenance="operator:grounding_panel"))
            self.ctx.log.event("operator_correction", correction=kind,
                               entity=entity, patch=patch.id)
            return {"ok": True, "patch": patch.id}
        return {"ok": False, "error": f"unknown correction kind {kind!r}"}

    # -- server --------------------------------------------------------------
    def start(self, port: int = 8770, host: str = "127.0.0.1") -> "Console":
        if not self._attach_sim():
            for cam in self.cameras:
                self.capture_now(cam)
        console = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):        # noqa: A003 - quiet
                pass

            def _send(self, code, ctype, body: bytes):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):                 # noqa: N802
                if self.path.split("?")[0] != "/correct":
                    return self._send(404, "text/plain", b"not found")
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    body = json.loads(self.rfile.read(n) or b"{}")
                    out = console.correct(body)
                except Exception as e:              # noqa: BLE001
                    out = {"ok": False, "error": f"{type(e).__name__}: {e}"}
                return self._send(200, "application/json", json.dumps(out).encode())

            def do_GET(self):                  # noqa: N802
                path = self.path.split("?")[0]
                if path in ("/", "/index.html"):
                    return self._send(200, "text/html; charset=utf-8", PAGE.encode())
                if path == "/state":
                    return self._send(200, "application/json",
                                      json.dumps(console.snapshot()).encode())
                if path.startswith("/frame/"):
                    cam = path[len("/frame/"):].removesuffix(".jpg")
                    if getattr(console, "_grabber", None) is None:
                        console.capture_now(cam)
                    with console._lock:
                        got = console._frames.get(cam)
                    if not got:
                        return self._send(404, "text/plain", b"no frame")
                    t, st, body, _w, _h = got
                    if console._is_stale(t, st, console._steps()):
                        # A stale frame with fresh markers on it is worse than
                        # no frame: it invites a person to correct a grounding
                        # against a picture of a world that has moved.
                        return self._send(503, "text/plain", b"frame is stale")
                    return self._send(200, "image/jpeg", body)
                return self._send(404, "text/plain", b"not found")

        self._server = ThreadingHTTPServer((host, port), Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        print(f"[console] http://{host}:{port}  cameras {self.cameras}", flush=True)
        return self

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None
        grabber = getattr(self, "_grabber", None)
        listeners = getattr(self.backend, "step_listeners", None)
        if grabber and listeners and grabber in listeners:
            listeners.remove(grabber)


PAGE = """<!doctype html><meta charset="utf-8"><title>heron console</title>
<style>
:root{--ink:#e8eaef;--dim:#98a0ac;--line:#2b3038;--bg:#111318;--card:#191c22;--ok:#3fa96a;--no:#d2543f;--un:#d8a23a}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 ui-sans-serif,-apple-system,sans-serif}
.top{padding:14px 20px;border-bottom:1px solid var(--line);display:flex;gap:16px;align-items:baseline}
h1{font-size:16px;margin:0;font-weight:600}
.dim{color:var(--dim);font-size:12px}
.grid{display:grid;grid-template-columns:1fr 340px;gap:18px;padding:18px 20px;align-items:start}
.cams{display:flex;flex-wrap:wrap;gap:14px}
.cam{position:relative}
.cam.stalecam img{opacity:.45;filter:grayscale(.5)}
.cam.stalecam .lbl::after{content:' · stale';color:#d2543f}
.cam img{display:block;border-radius:8px;border:1px solid var(--line);background:#000;
  width:480px;height:360px;object-fit:contain;max-width:100%}
.cam .lbl{position:absolute;left:8px;top:6px;font:11px ui-monospace,monospace;color:#fff;
  background:rgba(0,0,0,.55);padding:2px 6px;border-radius:4px}
.mark{position:absolute;width:18px;height:18px;border-radius:50%;transform:translate(-50%,-50%);
  border:2px solid var(--un);pointer-events:none}
.mark.seen{border-color:var(--ok)} .mark.stale{border-color:var(--no);border-style:dashed}
.mark b{position:absolute;left:16px;top:-6px;font:10px ui-monospace,monospace;white-space:nowrap;
  color:inherit;text-shadow:0 1px 3px #000}
.card{background:var(--card);border:1px solid var(--line);border-radius:9px;padding:12px 14px;margin-bottom:14px}
h2{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--dim);margin:0 0 9px;font-weight:600}
.row{display:flex;gap:8px;align-items:baseline;padding:4px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:0}
.mono{font:12px ui-monospace,SFMono-Regular,Menlo,monospace}
.pill{font:10px ui-monospace,monospace;padding:1px 6px;border-radius:99px;border:1px solid var(--line);color:var(--dim)}
.fix{font:11px ui-monospace,monospace;background:none;border:1px solid var(--line);color:var(--dim);
  border-radius:4px;padding:0 5px;cursor:pointer} .fix:hover{color:var(--ink);border-color:var(--dim)}
.done{color:var(--ok)} .fail{color:var(--no)} .run{color:#fff;font-weight:600} .todo{color:var(--dim)}
.true{color:var(--ok)} .false{color:var(--no)} .unknown{color:var(--un)}
.grow{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
</style>
<div class="top"><h1 id="instr">waiting for an episode…</h1><div class="dim" id="clock"></div></div>
<div class="grid">
  <div class="cams" id="cams"></div>
  <div>
    <div class="card"><h2>Doing now</h2><div id="phase" class="dim">idle</div></div>
    <div class="card"><h2>Plan</h2><div id="steps" class="dim">—</div></div>
    <div class="card"><h2>Goal</h2><div id="goal" class="dim mono">—</div></div>
    <div class="card"><h2>Predicates</h2><div id="preds" class="dim">—</div></div>
    <div class="card"><h2>Grounding</h2><div id="tracks" class="dim">—</div></div>
  </div>
</div>
<script>
const el = id => document.getElementById(id);
let cams = {};
function camNode(name){
  if (cams[name]) return cams[name];
  const d = document.createElement('div'); d.className = 'cam';
  d.innerHTML = `<img alt="${name}"><span class="lbl">${name}</span>`;
  el('cams').appendChild(d); cams[name] = d; return d;
}
function refreshFrames(){
  for (const name in cams){
    const img = cams[name].querySelector('img');
    img.src = '/frame/' + name + '.jpg?t=' + Date.now();
  }
}
async function tick(){
  let s;
  try { s = await (await fetch('/state')).json(); } catch(e) { return; }
  el('instr').textContent = s.instruction || '(no instruction)';
  el('clock').textContent = s.t + 's';
  for (const name in s.cameras) camNode(name);

  // markers, drawn on the frame they were read from
  for (const name in cams){
    const node = cams[name], img = node.querySelector('img');
    node.querySelectorAll('.mark').forEach(m => m.remove());
    const meta = s.cameras[name]; if (!meta || !img.clientWidth) continue;
    node.classList.toggle('stalecam', !!meta.stale);
    const sx = img.clientWidth / meta.w, sy = img.clientHeight / meta.h;
    for (const t of s.tracks){
      const p = t.at && t.at[name]; if (!p) continue;
      const m = document.createElement('div');
      m.className = 'mark ' + (t.seen ? (t.age_s > 10 ? 'stale' : 'seen') : 'stale');
      m.style.left = (p[0]*sx) + 'px'; m.style.top = (p[1]*sy) + 'px';
      const how = t.seen ? `seen ${t.age_s}s ago` : 'dead-reckoned';
      m.innerHTML = `<b>${t.id} ${t.conf} · ${how}${t.held_by ? ' · held' : ''}</b>`;
      node.appendChild(m);
    }
  }
  const ph = s.phase;
  el('phase').innerHTML = ph
    ? `<div class="row"><span class="grow mono run">${ph.skill}(${ph.entity})</span>
         <span class="pill">${ph.in_skill_s}s</span></div>
       <div class="row"><span class="grow dim">${ph.done ? 'after ' + ph.done : 'starting'}</span>
         <span class="mono ${ph.in_phase_s > 8 ? 'unknown' : ''}">${ph.in_phase_s}s</span></div>`
    : '<span class="dim">idle — no skill running</span>';
  el('steps').innerHTML = s.steps.length ? s.steps.map(x => {
    const cls = {succeeded:'done', failed:'fail', running:'run'}[x.status] || 'todo';
    const args = Object.entries(x.args).map(([k,v]) => k+'='+v).join(', ');
    return `<div class="row"><span class="mono ${cls}">${x.id}</span>
      <span class="grow ${cls}">${x.skill}(${args})</span>
      <span class="pill">${x.status}</span></div>`;
  }).join('') : '<span class="dim">no plan yet</span>';
  el('goal').innerHTML = s.goal.length ? s.goal.map(g=>`<div>${g}</div>`).join('') : '—';
  el('preds').innerHTML = s.predicates.length ? s.predicates.map(p =>
    `<div class="row"><span class="grow mono">${p.key}</span>
     <span class="${p.value}">${p.value}</span>
     <button class="fix" onclick="assertPred('${p.key}','true')">✓</button>
     <button class="fix" onclick="assertPred('${p.key}','false')">✗</button></div>`).join('')
    : '<span class="dim">nothing checked yet</span>';
  el('tracks').innerHTML = s.tracks.length ? s.tracks.map(t =>
    `<div class="row"><span class="grow mono">${t.id}</span>
     <span class="pill">${t.seen ? 'seen' : 'reckoned'} ${t.age_s ?? '–'}s</span>
     <span class="${t.conf >= 0.8 ? 'true' : 'unknown'}">${t.conf}</span>
     <button class="fix" title="mark not there; forces a re-look"
       onclick="notThere('${t.id}')">✗</button></div>`).join('')
    : '<span class="dim">nothing grounded yet</span>';
}
async function post(body){
  const r = await (await fetch('/correct', {method:'POST', body: JSON.stringify(body)})).json();
  if (!r.ok) alert(r.error);
}
function assertPred(key, value){ post({kind:'predicate_assert', predicate:key, value}); }
function notThere(id){ post({kind:'not_there', entity:id}); }
setInterval(tick, 400);
setInterval(refreshFrames, 200);
tick();
</script>
"""
