"""Photograph each D405 one at a time, so the roles can be matched to serials.

Why not `depth_server --auto`: that opens all four cameras at once, and this rig
cannot do that. The log from the last attempt shows all four failing with "No
device connected" within seconds of each other. One camera at a time is the
configuration that works here.

Nothing about the role -> serial mapping is recorded anywhere on this machine:
the config addresses cameras by AVFoundation index, which is a different
namespace from librealsense serials and is reassigned on every replug. So the
mapping has to be established by looking.

Run in a GUI Terminal, as root — macOS gives librealsense the D405s under
neither condition alone:

    sudo ~/Heron/.venv/bin/python ~/Heron/tools/identify_cameras.py

Writes /tmp/camid/<serial>.png and prints a summary. Depth is captured too when
available, since proving depth works is the whole point of the exercise.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

OUT = "/tmp/camid"
LOG = f"{OUT}/run.log"
WARMUP_FRAMES = 12  # auto-exposure needs a few frames or every shot is black


def say(msg: str) -> None:
    """Print, and also record. This has to be run from a GUI Terminal, so the
    output lands somewhere nobody working over SSH can read — which cost a
    round-trip of asking the operator to copy their window back."""
    print(msg, flush=True)
    try:
        with open(LOG, "a") as fh:
            fh.write(msg + "\n")
    except OSError:
        pass


def capture(rs, serial: str, width: int, height: int, fps: int, want_depth: bool):
    """Open one device, grab a settled frame, and close it again.

    The close matters as much as the open: leaving a device streaming is what
    makes the next one fail.
    """
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    if want_depth:
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    try:
        pipe.start(cfg)
        frames = None
        for _ in range(WARMUP_FRAMES):
            frames = pipe.wait_for_frames(5000)
        colour = np.asanyarray(frames.get_color_frame().get_data())
        depth = None
        if want_depth:
            d = frames.get_depth_frame()
            if d:
                depth = np.asanyarray(d.get_data())
        return colour, depth, None
    except Exception as e:
        return None, None, f"{type(e).__name__}: {e}"
    finally:
        try:
            pipe.stop()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--no-depth", action="store_true")
    ap.add_argument("--settle-s", type=float, default=1.5,
                    help="pause between cameras so the bus releases the last one")
    ap.add_argument("--only", action="append", metavar="SERIAL",
                    help="capture just this camera (repeatable). Measured on this "
                         "rig: the first two open fine and the rest fail, so the "
                         "way to reach the last ones is to replug the hub and ask "
                         "for them alone.")
    ap.add_argument("--skip", action="append", metavar="SERIAL",
                    help="leave this camera closed (repeatable)")
    args = ap.parse_args()

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("pyrealsense2 is not installed in this interpreter", flush=True)
        return 2
    if os.geteuid() != 0:
        print("Run this with sudo. Without root, enumeration fails with\n"
              "'failed to set power state' — that is this machine's normal\n"
              "behaviour, not a fault.", flush=True)
        return 2

    os.makedirs(OUT, exist_ok=True)
    say(f"--- {time.strftime('%Y-%m-%d %H:%M:%S')} ---")
    devices = list(rs.context().query_devices())
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
    say(f"enumerated {len(serials)} camera(s): {serials}")
    if not serials:
        say("No devices. Unplug the USB hub's cable from the Mac for five "
            "seconds, plug it back in, and run this again immediately.")
        return 1
    if args.only:
        missing = [s for s in args.only if s not in serials]
        if missing:
            say(f"asked for {missing}, which did not enumerate - replug the hub")
        serials = [s for s in serials if s in args.only]
    if args.skip:
        serials = [s for s in serials if s not in args.skip]
    if not serials:
        say("nothing left to capture after --only/--skip")
        return 1

    from PIL import Image

    ok = 0
    for i, serial in enumerate(serials):
        if i:
            time.sleep(args.settle_s)
        colour, depth, err = capture(rs, serial, args.width, args.height,
                                     args.fps, not args.no_depth)
        if err is not None:
            say(f"  {serial}  FAILED - {err}")
            continue
        path = f"{OUT}/{serial}.png"
        Image.fromarray(colour).save(path)
        note = "no depth stream"
        if depth is not None:
            valid = int((depth > 0).sum())
            note = (f"depth {valid * 100 // depth.size}% valid, "
                    f"median {np.median(depth[depth > 0]) if valid else 0:.0f} raw units")
            np.save(f"{OUT}/{serial}_depth.npy", depth)
        say(f"  {serial}  ok -> {path}   {note}")
        ok += 1

    say(f"{ok}/{len(serials)} captured. Images and this log are in {OUT}.")
    say("Nothing else is needed from this Terminal - the results can be read "
        "over SSH.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
