#!/usr/bin/env python3
"""RGB-D camera server for the Trossen AI Stationary rig — the single owner of
the RealSense cameras on this Mac.

Why a server: on macOS, librealsense must SEIZE each D405 from the kernel's UVC
driver, which needs root and makes the camera disappear from AVFoundation. Two
consequences shape this design:

  * one process must own the cameras, and everything else (the dashboard, Heron,
    recording tools) consumes frames from it over localhost;
  * on shutdown the cameras must be HANDED BACK — `--handback` issues a
    hardware_reset so macOS re-attaches its UVC driver and the AVFoundation
    workflow keeps working without anyone replugging a cable.

Endpoints (127.0.0.1 only):
    /health                    per-camera state, fps, frame age
    /cameras                   role -> serial mapping
    /info/<role>               intrinsics + depth scale
    /frame/<role>              {"t", "rgb_png_b64", "depth_png16_b64"}
    /rgb.png?role=<role>       latest color frame (also /rgb/<role>.png)
    /depth.png16?role=<role>   latest depth, 16-bit PNG, raw device units
    /depth_view/<role>.jpg     colorized depth preview (for humans/dashboards)

Run as root (required):
    sudo python3 tools/depth_server.py --config configs/abaka.yaml
    sudo python3 tools/depth_server.py --fake          # no hardware, for CI/dev
    sudo python3 tools/depth_server.py --handback      # give the cameras back and exit
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
from PIL import Image

DEFAULT_ROLES = ("cam_high", "cam_low", "cam_left_wrist", "cam_right_wrist")


class Stream:
    """One camera: capture thread, latest-frame buffer, simple fps accounting."""

    def __init__(self, role: str, serial: str, use_depth: bool = True) -> None:
        self.role = role
        self.serial = serial
        self.use_depth = use_depth
        self.ready = threading.Event()  # set once frames actually flow
        self.lock = threading.Lock()
        self.rgb: np.ndarray | None = None
        self.depth_raw: np.ndarray | None = None
        self.t = 0.0
        self.frames = 0
        self.fps = 0.0
        self.error = ""
        self.info: dict = {"serial": serial, "role": role}
        self.running = True

    def put(self, rgb, depth_raw) -> None:
        now = time.time()
        with self.lock:
            self.rgb, self.depth_raw = rgb, depth_raw
            if self.t:
                self.fps = 0.9 * self.fps + 0.1 / max(now - self.t, 1e-6)
            self.t = now
            self.frames += 1
            self.error = ""
        self.ready.set()

    def get(self):
        with self.lock:
            return self.rgb, self.depth_raw, self.t

    def health(self) -> dict:
        with self.lock:
            return {
                "role": self.role, "serial": self.serial,
                "ok": self.rgb is not None and (time.time() - self.t) < 3.0,
                "fps": round(self.fps, 1), "frames": self.frames,
                "frame_age_s": round(time.time() - self.t, 3) if self.t else None,
                "error": self.error,
            }


class RealSenseRig:
    """Owns every configured camera. Depth is aligned to color.

    Field lessons baked in (2026-07-30, four D405s on one hub):
      - simultaneous opens knock devices off the bus: bring-up is strictly
        SEQUENTIAL — the next camera starts only after the previous one is
        actually delivering frames;
      - hardware_reset mid-run ripples the whole hub and cascades every other
        camera offline. Never reset while serving; reset only at exit/handback.
    """

    _start_lock = threading.Lock()

    def __init__(self, roles: dict[str, str], width: int, height: int, fps: int,
                 color_only: bool = False, color_only_roles: set | None = None) -> None:
        import pyrealsense2 as rs  # noqa: PLC0415

        self.rs = rs
        self.streams: dict[str, Stream] = {}
        self.threads: list[threading.Thread] = []
        available = {d.get_info(rs.camera_info.serial_number) for d in enumerate_with_self_heal(rs)}
        missing = [s for s in roles.values() if s and s not in available]
        if missing:
            raise SystemExit(f"cameras not found: {missing}; visible: {sorted(available)}")
        prev: Stream | None = None
        for role, serial in roles.items():
            # DEPTH IS THE BANDWIDTH. Three D405s streaming colour AND depth on
            # one macOS bus take turns failing — "Frame didn't arrive", "No
            # device connected", a different camera each restart. The wrists are
            # looked at, not measured from, so they can give up depth and the
            # overhead pair keeps it. Measured: two with depth is stable, three
            # with depth is not.
            mono = bool(color_only) or role in (color_only_roles or set())
            stream = Stream(role, serial, use_depth=not mono)
            self.streams[role] = stream
            t = threading.Thread(target=self._loop, args=(stream, width, height, fps, prev),
                                 daemon=True)
            t.start()
            self.threads.append(t)
            prev = stream

    def _loop(self, stream: Stream, width: int, height: int, fps: int, wait_for: "Stream | None") -> None:
        rs = self.rs
        if wait_for is not None:  # sequential bring-up: predecessor must stream first
            if not wait_for.ready.wait(timeout=30.0):
                print(f"[{stream.role}] predecessor {wait_for.role} never became ready; "
                      "starting anyway", flush=True)
        consecutive_failures = 0
        while stream.running:
            pipe = None
            try:
                # After two failed rounds with the requested profile, fall back to
                # the device defaults — firmware revisions differ in what they
                # will actually deliver (5.17 pair vs 5.16 pair on this rig).
                use_defaults = consecutive_failures >= 2
                with self._start_lock:  # serialize opens; concurrent starts drop devices
                    pipe = rs.pipeline()
                    cfg = rs.config()
                    cfg.enable_device(stream.serial)
                    if use_defaults:
                        if stream.use_depth:
                            cfg.enable_stream(rs.stream.depth)
                        cfg.enable_stream(rs.stream.color)
                    else:
                        if stream.use_depth:
                            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
                    profile = pipe.start(cfg)
                    if use_defaults:
                        vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
                        print(f"[{stream.role}] running on DEFAULT profile "
                              f"{vsp.width()}x{vsp.height()}@{vsp.fps()}", flush=True)
                    align = rs.align(rs.stream.color) if stream.use_depth else None
                    dev = profile.get_device()
                    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                    stream.info.update({
                        "width": intr.width, "height": intr.height, "fx": intr.fx, "fy": intr.fy,
                        "ppx": intr.ppx, "ppy": intr.ppy, "use_depth": stream.use_depth,
                        "depth_scale": float(dev.first_depth_sensor().get_depth_scale())
                        if stream.use_depth else 0.001,
                    })
                    # Probation inside the lock: hold the next camera back until
                    # this one has proven it can deliver.
                    frames = pipe.wait_for_frames(15000)
                empty_depth = np.zeros((height, width), np.uint16)
                while stream.running:
                    if align is not None:
                        frames = align.process(frames)
                        stream.put(np.asanyarray(frames.get_color_frame().get_data()).copy(),
                                   np.asanyarray(frames.get_depth_frame().get_data()).copy())
                    else:
                        stream.put(np.asanyarray(frames.get_color_frame().get_data()).copy(),
                                   empty_depth)
                    consecutive_failures = 0
                    frames = pipe.wait_for_frames(10000)
            except Exception as e:
                consecutive_failures += 1
                stream.error = f"{type(e).__name__}: {e}"
                wait = min(3.0 * consecutive_failures, 15.0)
                print(f"[{stream.role}] {stream.error}; retry {consecutive_failures} in {wait:.0f}s",
                      flush=True)
                time.sleep(wait)
            finally:
                if pipe is not None:
                    try:
                        pipe.stop()
                    except Exception:
                        pass

    def stop(self) -> None:
        for s in self.streams.values():
            s.running = False
        for t in self.threads:
            t.join(timeout=3)


class FakeRig:
    """Synthetic scene so the HTTP contract is testable without hardware or root."""

    def __init__(self, roles: dict[str, str], width: int, height: int, fps: int) -> None:
        self.streams = {}
        for i, role in enumerate(roles):
            s = Stream(role, f"FAKE{i:04d}")
            s.info.update({"width": width, "height": height, "fx": 610.0, "fy": 610.0,
                           "ppx": width / 2, "ppy": height / 2, "depth_scale": 0.001})
            self.streams[role] = s
            threading.Thread(target=self._loop, args=(s, width, height, fps, i), daemon=True).start()

    def _loop(self, stream: Stream, w: int, h: int, fps: int, seed: int) -> None:
        while stream.running:
            rgb = np.full((h, w, 3), (208, 188, 158), np.uint8)
            depth = np.full((h, w), 850, np.uint16)
            cx, cy = w // 2 + 30 * (seed + 1), h // 2 - 20
            rgb[cy - 25: cy + 25, cx - 25: cx + 25] = (200, 40, 40)
            depth[cy - 25: cy + 25, cx - 25: cx + 25] = 810
            stream.put(rgb, depth)
            time.sleep(1.0 / fps)

    def stop(self) -> None:
        for s in self.streams.values():
            s.running = False


def handback() -> int:
    """Hardware-reset every camera so macOS re-attaches its UVC driver.

    Without this the cameras stay invisible to AVFoundation — i.e. to the
    dashboard and the LeRobot recording pipeline — until someone replugs USB.
    """
    import pyrealsense2 as rs  # noqa: PLC0415

    try:
        devices = enumerate_with_self_heal(rs)
    except Exception as e:
        print(f"handback: enumeration failed even after daemon kick ({e}) — replug the hub")
        return 1
    if not devices:
        print("no RealSense devices visible (already handed back?)")
        return 0
    for d in devices:
        serial = d.get_info(rs.camera_info.serial_number)
        try:
            d.hardware_reset()
            print(f"reset {serial}")
        except Exception as e:
            print(f"reset {serial} FAILED: {e}")
    print("waiting 10s for macOS to re-enumerate…")
    time.sleep(10)
    return 0


def colorize(depth_raw: np.ndarray, scale: float, near: float = 0.15, far: float = 1.2) -> np.ndarray:
    """Turquoise-to-red depth preview; invalid pixels stay black."""
    z = depth_raw.astype(np.float32) * scale
    valid = z > 0
    norm = np.clip((z - near) / max(far - near, 1e-6), 0, 1)
    hue = (1.0 - norm) * 0.66  # 0=red (far) .. 0.66=blue (near)
    h6 = hue * 6.0
    i = np.floor(h6).astype(np.int32) % 6
    f = h6 - np.floor(h6)
    v, p, q, t = 1.0, 0.0, 1.0 - f, f
    r = np.select([i == 0, i == 1, i == 2, i == 3, i == 4], [v, q, p, p, t], default=v)
    g = np.select([i == 0, i == 1, i == 2, i == 3, i == 4], [t, v, v, q, p], default=p)
    b = np.select([i == 0, i == 1, i == 2, i == 3, i == 4], [p, p, t, v, v], default=q)
    rgb = (np.stack([r, g, b], axis=-1) * 255).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


def make_handler(rig, started_at: float):
    def png_bytes(arr: np.ndarray, mode: str | None = None) -> bytes:
        buf = io.BytesIO()
        img = Image.fromarray(arr, mode=mode) if mode else Image.fromarray(arr)
        img.save(buf, format="PNG")
        return buf.getvalue()

    _jpeg_cache: dict = {}
    # Older than this and a frame is a record, not an observation.
    STALE_FRAME_S = 1.0

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200) -> None:
            self._send(code, json.dumps(obj).encode(), "application/json")

        # role -> (frame timestamp, encoded bytes)
        # Module-level so every request handler instance shares it: a
        # ThreadingHTTPServer builds a new handler per connection.
        def _role(self, path: str, query: dict) -> str | None:
            tail = path.rstrip(".png").rstrip(".jpg").split("/")[-1]
            role = query.get("role", [tail])[0]
            return role if role in rig.streams else None

        def do_GET(self):  # noqa: N802
            parsed = urlparse(self.path)
            path, query = parsed.path, parse_qs(parsed.query)
            if path == "/health":
                self._json({"ok": all(s.health()["ok"] for s in rig.streams.values()),
                            "uptime_s": round(time.time() - started_at, 1),
                            "cameras": [s.health() for s in rig.streams.values()]})
                return
            if path == "/cameras":
                self._json({r: s.serial for r, s in rig.streams.items()})
                return
            role = self._role(path, query)
            if role is None:
                self._json({"error": f"unknown role in {path!r}", "roles": list(rig.streams)}, code=404)
                return
            stream = rig.streams[role]
            if path.startswith("/info"):
                self._json(stream.info)
                return
            rgb, depth, t = stream.get()
            if rgb is None:
                self._json({"error": f"{role}: no frame yet", "detail": stream.error}, code=503)
                return
            if path.startswith("/frame"):
                self._json({"t": t, "role": role,
                            "rgb_png_b64": base64.b64encode(png_bytes(rgb)).decode(),
                            "depth_png16_b64": base64.b64encode(png_bytes(depth, "I;16")).decode()})
                return
            if path.startswith("/rgb"):
                # LOSSLESS COSTS 74 MILLISECONDS. Encoding a 1280x720 PNG on
                # every request capped a consumer at 13 fps, which is a
                # teleoperation recording at 30 fps repeating half its frames.
                # Colour for a viewer or a policy does not need to be lossless
                # — depth still is, and always will be.
                if ".jpg" in path or "fmt=jpeg" in (parsed.query or ""):
                    # ONE FRAME, ONE ENCODE. Teleoperation opens two cameras at
                    # 30 fps and the dashboard reads the same ones: sixty
                    # requests a second, each re-encoding a picture that had not
                    # changed, and the server stopped answering inside the
                    # client's timeout. The encode is now keyed on the frame's
                    # own timestamp, so N readers of one frame cost one encode.
                    # A CACHED FRAME IS STILL A FRAME FROM WHEN IT WAS TAKEN.
                    # Both heads dropped off the bus and this endpoint kept
                    # answering 200 in a millisecond with a picture 102 seconds
                    # old — a teleoperation recording would have filled with it
                    # and nothing downstream could have known. Age is knowable
                    # here, so it is checked here.
                    age = time.time() - float(t)
                    if age > STALE_FRAME_S:
                        self._json({"error": f"{role} frame is {age:.1f}s old; "
                                             "the camera is not delivering",
                                    "frame_age_s": round(age, 2)}, code=503)
                        return
                    hit = _jpeg_cache.get(role)
                    if hit is not None and hit[0] == t:
                        self._send(200, hit[1], "image/jpeg")
                        return
                    buf = io.BytesIO()
                    Image.fromarray(rgb).save(buf, "JPEG", quality=85)
                    body = buf.getvalue()
                    _jpeg_cache[role] = (t, body)
                    self._send(200, body, "image/jpeg")
                    return
                self._send(200, png_bytes(rgb), "image/png")
                return
            if path.startswith("/depth_view"):
                view = colorize(depth, float(stream.info.get("depth_scale", 0.001)))
                buf = io.BytesIO()
                Image.fromarray(view).save(buf, format="JPEG", quality=80)
                self._send(200, buf.getvalue(), "image/jpeg")
                return
            if path.startswith("/depth"):
                self._send(200, png_bytes(depth, "I;16"), "image/png")
                return
            self._json({"error": f"unknown path {path!r}"}, code=404)

    return Handler


_last_kick = 0.0


def kick_camera_daemons() -> None:
    """Kill macOS's camera daemons so their device claims drop (they respawn on
    demand). Field pattern: librealsense can only seize the D405s in the window
    after these die and before an AVFoundation client re-claims. Root required.
    Rate-limited so a retry loop does not spam killall.
    """
    import subprocess  # noqa: PLC0415

    global _last_kick
    if time.time() - _last_kick < 20.0:
        return
    _last_kick = time.time()
    for daemon in ("VDCAssistant", "appleh16camerad", "avconferenced"):
        subprocess.run(["/usr/bin/killall", daemon], capture_output=True)
    time.sleep(4.0)


def enumerate_with_self_heal(rs) -> list:
    """query_devices, kicking the camera daemons once if the bus looks wedged."""
    try:
        return list(rs.context().query_devices())
    except Exception as first:
        if os.geteuid() != 0:
            raise
        print(f"[depth_server] enumeration failed ({first}); kicking camera daemons and retrying",
              flush=True)
        kick_camera_daemons()
        return list(rs.context().query_devices())


def discover_roles() -> dict[str, str]:
    """Serve every connected camera under a provisional serial-based role.

    Bootstrap step: with the server up you can look at /rgb/<role>.png for each
    one, decide which is cam_high / cam_low / the wrists, and write the real
    mapping into the config.
    """
    import pyrealsense2 as rs  # noqa: PLC0415

    try:
        serials = sorted(d.get_info(rs.camera_info.serial_number) for d in enumerate_with_self_heal(rs))
    except Exception as e:
        raise SystemExit(
            f"camera enumeration failed ({type(e).__name__}: {e}).\n"
            "If this says 'failed to set power state', the devices are wedged or another\n"
            "process holds them: stop the dashboard/Chrome, or replug the USB hub, and retry."
        ) from e
    if not serials:
        raise SystemExit("no RealSense devices found — replug the USB hub and retry")
    return {f"cam_{s}": s for s in serials}


def load_roles(config_path: str | None, serials: list[str] | None):
    """(role -> serial, roles that stream colour only)."""
    if serials:
        roles, mono = {}, set()
        for item in serials:
            role, _, serial = item.partition("=")
            serial, sep, flag = serial.partition(":")
            roles[role] = serial
            if sep and flag.strip().lower() in ("color", "colour", "nodepth"):
                mono.add(role)
        return roles, mono
    if config_path:
        import yaml  # noqa: PLC0415

        cfg = yaml.safe_load(open(config_path)) or {}
        roles = {name: cam.get("serial", "") for name, cam in (cfg.get("cameras") or {}).items()}
        return {r: s for r, s in roles.items() if s}, set()
    return {}, set()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="heron config yaml (reads cameras[*].serial)")
    ap.add_argument("--camera", action="append", dest="serials", metavar="ROLE=SERIAL",
                    help="explicit role/serial pair; repeatable. Append ':color' to a "
                         "pair to stream that camera without depth — depth is what "
                         "saturates the bus, and a wrist view is looked at rather "
                         "than measured from.")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--auto", action="store_true",
                    help="serve every connected camera under a provisional serial-based role "
                         "(use once to identify which camera is which)")
    ap.add_argument("--wait-devices-s", type=float, default=180.0,
                    help="keep retrying enumeration this long at startup (lets you replug the hub "
                         "after starting the server)")
    ap.add_argument("--color-only", action="store_true",
                    help="skip depth streams (halves bus load; --auto implies this)")
    ap.add_argument("--fake", action="store_true", help="synthetic frames; no hardware, no root")
    ap.add_argument("--handback", action="store_true",
                    help="hardware-reset the cameras so macOS/AVFoundation gets them back, then exit")
    args = ap.parse_args()

    if args.handback:
        return handback()
    # macOS lets librealsense seize the D405s only as root; Linux grants
    # them to plugdev/video via udev rules, so non-root is the normal case.
    if not args.fake and sys.platform == "darwin" and os.geteuid() != 0:
        print("ERROR: root is required — macOS only lets librealsense seize the D405s as root.\n"
              "       sudo python3 tools/depth_server.py ...", flush=True)
        return 2

    # In auto mode discovery happens inside the wait-and-seize loop (the bus may
    # be wedged right now and clear later); only static configs resolve here.
    roles, mono_roles = ({}, set()) if (args.auto and not args.fake) \
        else load_roles(args.config, args.serials)
    if not roles and not (args.auto and not args.fake):
        if args.fake:
            roles = {r: f"FAKE{i}" for i, r in enumerate(DEFAULT_ROLES)}
        else:
            print("ERROR: no cameras configured — pass --config, --camera ROLE=SERIAL, or --auto")
            return 2

    color_only = args.color_only or (args.auto and not args.fake)
    if args.fake:
        rig = FakeRig(roles, args.width, args.height, args.fps)
    else:
        # Wait-and-seize: keep trying so the server can be started BEFORE a hub
        # replug — the moment devices enumerate, root claims them ahead of
        # VDCAssistant/AVFoundation. Ends the who-grabs-first race for good.
        deadline = time.time() + args.wait_devices_s
        while True:
            try:
                if args.auto:
                    roles = discover_roles()
                rig = RealSenseRig(roles, args.width, args.height, args.fps,
                                   color_only=color_only, color_only_roles=mono_roles)
                break
            except (SystemExit, Exception) as e:  # noqa: BLE001
                if time.time() > deadline:
                    print(f"[depth_server] giving up after {args.wait_devices_s:.0f}s: {e}", flush=True)
                    return 2
                print(f"[depth_server] devices not ready ({str(e)[:80]}); retrying in 3s "
                      "(replug the hub now if needed)", flush=True)
                time.sleep(3.0)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(rig, time.time()))
    print(f"[depth_server] serving {list(roles)} on http://127.0.0.1:{args.port} "
          f"({'fake' if args.fake else 'real'}, {args.width}x{args.height}@{args.fps})", flush=True)

    import signal  # noqa: PLC0415

    def _shutdown(signum, frame):  # SIGTERM from pkill/launchd, SIGINT from Ctrl-C
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)
    try:
        server.serve_forever()
    finally:
        # macOS: a dirty exit leaves half-open pipelines that wedge the whole
        # bus until someone replugs the hub (field-verified) — stop every
        # stream, then hardware-reset so macOS re-enumerates clean devices.
        # Linux: udev re-grants devices on plain close; a mass hardware_reset
        # here is what WEDGES the bus for the next opener, so never do it.
        rig.stop()
        if not args.fake and sys.platform == "darwin":
            try:
                handback()
            except Exception as e:
                print(f"[depth_server] exit reset failed: {e} — replug the hub if cameras act up", flush=True)
        print("[depth_server] stopped; streams closed.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
